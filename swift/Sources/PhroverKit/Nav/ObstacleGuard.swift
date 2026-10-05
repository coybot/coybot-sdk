import Foundation
import RoverNav

/// Safety layer sitting between the planner and the motors. Independent of the global
/// costmap so it reacts to *dynamic* obstacles (ground crew or a cart crossing its path)
/// and to link loss. Returns whether it is safe to keep driving; if not, the caller must stop.
public struct ObstacleGuard: Sendable {
    public var stopDistance: Double      // m — hard stop if forward clearance drops below
    public var watchdogTimeout: Double

    public init(stopDistance: Double = 0.45, watchdogTimeout: Double = RoverConfig.commsWatchdogTimeout) {
        self.stopDistance = stopDistance
        self.watchdogTimeout = watchdogTimeout
    }

    public enum Decision: Equatable {
        case go
        case stopObstacle(clearance: Double)
        case stopCommsLost
        case stopTipping
    }

    public func evaluate(forwardClearance: Double,
                          lastAckAt: Date?,
                          now: Date = Date(),
                          feedback: RoverFeedback?,
                          requireFreshAck: Bool = true,
                          checkForwardObstacle: Bool = true) -> Decision {
        if let fb = feedback, fb.isTipping() { return .stopTipping }
        if checkForwardObstacle, forwardClearance < stopDistance { return .stopObstacle(clearance: forwardClearance) }
        if requireFreshAck, let last = lastAckAt {
            if now.timeIntervalSince(last) > watchdogTimeout { return .stopCommsLost }
        }
        return .go
    }

    enum DepthDecision: Equatable {
        case allow(WheelCommand, depthSafety: DepthSafetyObservation)
        case stopDepth(DepthSafetyObservation)
    }

    func evaluate(command: WheelCommand,
                  depthSafety: DepthSafetyObservation) -> DepthDecision {
        switch depthSafety.state {
        case .clear:
            return .allow(command, depthSafety: depthSafety)
        case .caution:
            guard let limit = depthSafety.speedLimit, limit > 0 else {
                return .stopDepth(depthSafety)
            }
            let maximum = max(abs(command.left), abs(command.right))
            guard maximum > limit else {
                return .allow(command, depthSafety: depthSafety)
            }
            let scale = limit / maximum
            return .allow(
                WheelCommand(left: command.left * scale, right: command.right * scale),
                depthSafety: depthSafety
            )
        case .stop, .unavailable:
            return .stopDepth(depthSafety)
        }
    }
}
