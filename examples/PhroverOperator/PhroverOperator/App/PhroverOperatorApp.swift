import SwiftUI
import PhroverKit
import PhroverCloud

/// Reference app: a thin SwiftUI shell over PhroverKit (the brain) and PhroverCloud
/// (optional telemetry/auth/dialog-escalation client).
///
/// Cloud is genuinely optional — drop a `PhroverCloud.plist` next to this file (copy
/// `Config/PhroverCloud.example.plist` and fill in your backend's endpoints) to enable
/// sign-in, MQTT telemetry, and cloud dialog escalation. Without it, the app skips
/// straight to manual/voice driving using only on-device intelligence.
@main
struct PhroverOperatorApp: App {
    private enum Runtime {
        case scripted(ScriptedSilentSearchViewModel)
        case scriptedTalk(ConversationViewModel)
        case live(LiveAppRuntime)
    }

    @State private var runtime: Runtime

    init() {
        let arguments = ProcessInfo.processInfo.arguments
        if arguments.contains("-ui-testing"),
           let index = arguments.firstIndex(of: "-silent-search-scenario"),
           arguments.indices.contains(index + 1),
           let scenario = SilentSearchLaunchScenario(rawValue: arguments[index + 1]) {
            _runtime = State(initialValue: .scripted(ScriptedSilentSearchViewModel(scenario: scenario)))
        } else if arguments.contains("-ui-testing"),
                  let index = arguments.firstIndex(of: "-talk-scenario"),
                  arguments.indices.contains(index + 1) {
            let model = ConversationViewModel()
            let session = ScriptedTalkSession(state: arguments[index + 1] == "searching" ? .searching : .idle)
            model.configure(submit: { text in
                if text.lowercased() == "follow me" { session.state = .searching; return .accepted }
                return .rejected("Stop following first.")
            }, stop: { session.state = .stopped; return .accepted }, followState: { session.state })
            _runtime = State(initialValue: .scriptedTalk(model))
        } else {
            _runtime = State(initialValue: .live(LiveAppRuntime()))
        }
    }

    var body: some Scene {
        WindowGroup {
            switch runtime {
            case let .scripted(viewModel):
                TabView {
                    SilentSearchView(viewModel: viewModel)
                        .tabItem { Label("Silent Search", systemImage: "qrcode.viewfinder") }
                        .accessibilityIdentifier("silent_search_tab")
                }
            case let .scriptedTalk(model):
                let ar = ARSessionManager()
                let control = RoverControl()
                let nav = NavigationController(ar: ar, control: control)
                RootView(ar: ar, control: control, nav: nav, topology: SessionRoomTopology(), cloud: nil,
                         silentSearch: nil, scriptedTalk: model)
            case let .live(live):
                RootView(
                    ar: live.ar, control: live.control, nav: live.nav, topology: live.topology,
                    cloud: live.cloud, silentSearch: live.silentSearch
                )
                .task { await live.start() }
            }
        }
    }
}

@Observable
@MainActor
private final class ScriptedTalkSession {
    var state: FollowMeState
    init(state: FollowMeState) { self.state = state }
}

@Observable
@MainActor
final class LiveAppRuntime {
    let ar: ARSessionManager
    let control: RoverControl
    let nav: NavigationController
    let topology: SessionRoomTopology
    let cloud: CloudSession?
    private let roomSessionCoordinator: RoomSessionCoordinator
    private(set) var silentSearch: LiveSilentSearchViewModel?
    private var started = false

    init() {
        let ar = ARSessionManager()
        let control = RoverControl()
        let nav = NavigationController(ar: ar, control: control)
        let topology = SessionRoomTopology()
        self.ar = ar
        self.control = control
        self.nav = nav
        self.topology = topology
        roomSessionCoordinator = RoomSessionCoordinator(
            motion: nav,
            topology: topology,
            arSession: ar
        )
        cloud = CloudSession.loadIfConfigured(ar: ar, nav: nav)
    }

    func start() async {
        guard !started else { return }
        started = true
        // Starts the AR session (generation-tracked reset) as well as room topology.
        await roomSessionCoordinator.startIfNeeded()
        async let feedback: Void = enableFeedback()
        let detector = await Detector()
        silentSearch = LiveSilentSearchViewModel.compose(
            ar: ar, control: control, navigation: nav, detector: detector
        )
        await feedback
    }

    private func enableFeedback() async {
        do {
            try await control.enableFeedbackFlow()
            RuntimeFileLog.append("feedback_flow_enabled")
        } catch {
            RuntimeFileLog.append("feedback_flow_failed", fields: [
                "error": error.localizedDescription
            ])
        }
    }
}

/// Bundles the optional PhroverCloud services the app wires up when a config is present.
@Observable
@MainActor
final class CloudSession {
    let auth: AuthService
    let mqtt: MQTTService
    let dialog: ClaudeDialogClient
    /// Vision + tool-use mission brain — plugs into `MissionAgent` via `HybridBrain` once
    /// signed in; `ConversationView` falls back to on-device-only until then.
    let brain: CloudBrain
    let doorwayEvidenceProvider: CloudDoorwayEvidenceProvider
    private let telemetry: RoverTelemetryPublisher

    private init(config: PhroverCloudConfig, ar: ARSessionManager, nav: NavigationController) {
        auth = AuthService(config: config)
        mqtt = MQTTService(config: config)
        dialog = ClaudeDialogClient(config: config)
        brain = CloudBrain(config: config)
        doorwayEvidenceProvider = CloudDoorwayEvidenceProvider(brain: brain)
        telemetry = RoverTelemetryPublisher(ar: ar, nav: nav, mqtt: mqtt)
    }

    static func loadIfConfigured(ar: ARSessionManager, nav: NavigationController) -> CloudSession? {
        guard let url = Bundle.main.url(forResource: "PhroverCloud", withExtension: "plist"),
              let config = PhroverCloudConfig(contentsOfPlist: url) else {
            return nil
        }
        return CloudSession(config: config, ar: ar, nav: nav)
    }

    func syncWithAuthState() async {
        guard auth.isAuthenticated else {
            mqtt.disconnect()
            telemetry.stop()
            return
        }
        await dialog.setTokenProvider { [weak auth] in await auth?.idToken }
        brain.setTokenProvider { [weak auth] in await auth?.idToken }
        guard let token = auth.idToken else { return }
        do {
            try await mqtt.connect(withToken: token)
            telemetry.start()
        } catch {
            AppLogger.nav.error("MQTT connect failed: \(error.localizedDescription)")
        }
    }
}

struct RootView: View {
    let ar: ARSessionManager
    let control: RoverControl
    let nav: NavigationController
    let topology: SessionRoomTopology
    let cloud: CloudSession?
    let silentSearch: LiveSilentSearchViewModel?

    private enum Tab: Hashable { case drive, navigate, talk, silentSearch }
    private let scriptedTalk: Bool
    @State private var talkModel: ConversationViewModel
    @State private var selectedTab: Tab

    init(ar: ARSessionManager, control: RoverControl, nav: NavigationController,
         topology: SessionRoomTopology,
         cloud: CloudSession?, silentSearch: LiveSilentSearchViewModel?,
         scriptedTalk: ConversationViewModel? = nil) {
        self.ar = ar
        self.control = control
        self.nav = nav
        self.topology = topology
        self.cloud = cloud
        self.silentSearch = silentSearch
        self.scriptedTalk = scriptedTalk != nil
        _talkModel = State(initialValue: scriptedTalk ?? ConversationViewModel())
        _selectedTab = State(initialValue: scriptedTalk == nil ? .drive : .talk)
    }

    static func followMayStart(navigationState: NavigationController.State,
                               silentSearchPhase: SilentSearchOperatorPhase?) -> Bool {
        guard navigationState == .idle || navigationState == .arrived else { return false }
        guard let silentSearchPhase else { return true }
        return [.setup, .terminalFound, .terminalNotFound, .terminalFailure]
            .contains(silentSearchPhase)
    }

    var body: some View {
        Group {
            if let cloud {
                if cloud.auth.isAuthenticated {
                    tabs
                        .environment(cloud.auth)
                        .task(id: cloud.auth.isAuthenticated) { await cloud.syncWithAuthState() }
                } else {
                    AuthView().environment(cloud.auth)
                }
            } else {
                tabs
            }
        }
    }

    private var tabs: some View {
        TabView(selection: Binding(
            get: { selectedTab },
            set: { next in
                guard next != selectedTab else { return }
                if selectedTab == .talk {
                    talkModel.prepareToLeave()
                    Task {
                        if await talkModel.leaveTalk() { selectedTab = next }
                    }
                } else {
                    selectedTab = next
                }
            }
        )) {
            DriveView(ar: ar, control: control)
                .tabItem { Label("Drive", systemImage: "gamecontroller") }
                .tag(Tab.drive)
            NavigateView(ar: ar, nav: nav)
                .tabItem { Label("Navigate", systemImage: "map") }
                .tag(Tab.navigate)
            ConversationView(
                ar: ar,
                control: control,
                nav: nav,
                topology: topology,
                cloudBrain: cloud?.brain,
                doorwayEvidenceProvider: cloud?.doorwayEvidenceProvider,
                model: talkModel, scripted: scriptedTalk,
                mayStartFollow: {
                    Self.followMayStart(navigationState: nav.state,
                                        silentSearchPhase: silentSearch?.phase)
                }
            )
                .tabItem { Label("Talk", systemImage: "bubble.left.and.bubble.right") }
                .tag(Tab.talk)
            if let silentSearch {
                SilentSearchView(viewModel: silentSearch)
                    .tabItem { Label("Silent Search", systemImage: "qrcode.viewfinder") }
                    .accessibilityIdentifier("silent_search_tab")
                    .tag(Tab.silentSearch)
            }
        }
    }
}
