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
        TabView {
            DriveView(ar: ar, control: control)
                .tabItem { Label("Drive", systemImage: "gamecontroller") }
            NavigateView(ar: ar, nav: nav)
                .tabItem { Label("Navigate", systemImage: "map") }
            ConversationView(
                ar: ar,
                control: control,
                nav: nav,
                topology: topology,
                cloudBrain: cloud?.brain,
                doorwayEvidenceProvider: cloud?.doorwayEvidenceProvider
            )
                .tabItem { Label("Talk", systemImage: "bubble.left.and.bubble.right") }
            if let silentSearch {
                SilentSearchView(viewModel: silentSearch)
                    .tabItem { Label("Silent Search", systemImage: "qrcode.viewfinder") }
                    .accessibilityIdentifier("silent_search_tab")
            }
        }
    }
}
