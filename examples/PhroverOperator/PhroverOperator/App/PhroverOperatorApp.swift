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
    @State private var ar: ARSessionManager
    @State private var control: RoverControl
    @State private var nav: NavigationController
    @State private var cloud: CloudSession?

    init() {
        let ar = ARSessionManager()
        let control = RoverControl()
        _ar = State(initialValue: ar)
        _control = State(initialValue: control)
        _nav = State(initialValue: NavigationController(ar: ar, control: control))
        _cloud = State(initialValue: CloudSession.loadIfConfigured(ar: ar, nav: nav))
    }

    var body: some Scene {
        WindowGroup {
            RootView(ar: ar, control: control, nav: nav, cloud: cloud)
                .task {
                    ar.start()
                    Task {
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
    private let telemetry: RoverTelemetryPublisher

    private init(config: PhroverCloudConfig, ar: ARSessionManager, nav: NavigationController) {
        auth = AuthService(config: config)
        mqtt = MQTTService(config: config)
        dialog = ClaudeDialogClient(config: config)
        brain = CloudBrain(config: config)
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
    let cloud: CloudSession?

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
            ConversationView(ar: ar, control: control, nav: nav, cloudBrain: cloud?.brain)
                .tabItem { Label("Talk", systemImage: "bubble.left.and.bubble.right") }
        }
    }
}
