import Foundation
import PhroverKit

/// Publishes periodic rover telemetry to AWS IoT on `drone/{roverId}/status` — the
/// reference backend's `DroneStatusRule` (SELECT * FROM 'drone/+/status') stores whatever
/// is sent here with zero backend changes.
@MainActor
public final class RoverTelemetryPublisher {
    public let roverId: String

    private let ar: ARSessionManager
    private let nav: NavigationController
    private let mqtt: MQTTService
    private var task: Task<Void, Never>?

    public init(ar: ARSessionManager, nav: NavigationController, mqtt: MQTTService,
                idKey: String = "bot.coy.phrover.id") {
        self.ar = ar
        self.nav = nav
        self.mqtt = mqtt
        self.roverId = Self.loadOrCreateRoverId(idKey: idKey)
    }

    public func start(interval: TimeInterval = 2.0) {
        stop()
        task = Task { [weak self] in
            while let self, !Task.isCancelled {
                self.publishOnce()
                try? await Task.sleep(for: .seconds(interval))
            }
        }
    }

    public func stop() {
        task?.cancel()
        task = nil
    }

    private func publishOnce() {
        var payload: [String: Any] = [
            "vehicleType": "rover",
            "navState": navStateLabel,
            "trackingState": trackingLabel,
            "forwardClearanceM": ar.forwardClearance.isFinite ? ar.forwardClearance : -1,
        ]
        if let pose = ar.pose {
            payload["poseX"] = pose.position.x
            payload["poseY"] = pose.position.y
            payload["yawRad"] = pose.yaw
        }
        mqtt.publish(to: "drone/\(roverId)/status", payload: payload)
    }

    private var navStateLabel: String {
        switch nav.state {
        case .idle: return "idle"
        case .planning: return "planning"
        case .driving: return "driving"
        case .arrived: return "arrived"
        case .failed: return "failed"
        }
    }

    private var trackingLabel: String {
        switch ar.trackingState {
        case .normal: return "normal"
        case .limited: return "limited"
        case .notAvailable: return "none"
        @unknown default: return "unknown"
        }
    }

    // MARK: - Rover ID

    /// Stable per-install identifier, generated once and persisted.
    private static func loadOrCreateRoverId(idKey: String) -> String {
        if let existing = UserDefaults.standard.string(forKey: idKey) { return existing }
        let id = "rover-\(UUID().uuidString.prefix(8))"
        UserDefaults.standard.set(id, forKey: idKey)
        return id
    }
}
