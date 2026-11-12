import Foundation
import RoverNav

/// Limits increases in forward wheel requests, not measured physical acceleration.
/// Reductions and safety stops are immediate. Preserve the requested curvature.
struct FollowPursuitAcceleration {
    static let startingMagnitude = 0.10
    static let acceleration = 0.25
    static let maximumInterval = 0.20
    private var lastMagnitude: Double?
    private var lastSendTime: Double?

    mutating func reset() { lastMagnitude = nil; lastSendTime = nil }

    /// Brake before normal arrival/hold admission, while motion is still
    /// authorized. Never prolong a stop by sending a ramp after cancellation.
    static func approaching(_ command: WheelCommand, distance: Double,
                            params: PursuitController.Params) -> WheelCommand {
        guard distance.isFinite, command.left.isFinite, command.right.isFinite else { return .stop }
        guard command.left >= 0, command.right >= 0 else { return command }
        if distance <= params.goalTolerance { return .stop }
        let requested = max(command.left, command.right)
        guard requested > 0 else { return command }
        // Reach crawl speed before the normal hold band can request a stop.
        // The small final crawl avoids an asymptotic approach that never arrives.
        let crawlBoundary = params.goalTolerance + 0.10
        guard params.slowdownRadius > crawlBoundary else { return command }
        let fraction = max(0, min(1, (distance - crawlBoundary) / (params.slowdownRadius - crawlBoundary)))
        let ceiling = max(0.04, params.maxLinear * fraction)
        let scale = min(1, ceiling / requested)
        return .init(left: command.left * scale, right: command.right * scale)
    }

    func limited(_ command: WheelCommand, at time: Double) -> WheelCommand {
        guard time.isFinite, command.left.isFinite, command.right.isFinite else { return .init(left: 0, right: 0) }
        guard command.left >= 0, command.right >= 0 else { return command }
        let requested = max(command.left, command.right)
        guard requested > 0 else { return command }
        let ceiling: Double
        if let lastMagnitude, let lastSendTime {
            let elapsed = min(Self.maximumInterval, max(0, time - lastSendTime))
            ceiling = lastMagnitude + Self.acceleration * elapsed
        } else { ceiling = Self.startingMagnitude }
        let scale = min(1, ceiling / requested)
        return .init(left: command.left * scale, right: command.right * scale)
    }

    mutating func sent(_ command: WheelCommand, at time: Double) {
        guard time.isFinite, command.left.isFinite, command.right.isFinite,
              command.left >= 0, command.right >= 0, max(command.left, command.right) > 0 else {
            reset(); return
        }
        lastMagnitude = max(command.left, command.right)
        lastSendTime = time
    }
}
