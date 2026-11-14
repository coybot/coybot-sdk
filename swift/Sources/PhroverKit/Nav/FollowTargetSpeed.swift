import Foundation
import RoverNav

struct FollowTargetSpeedSample: Sendable {
    let frameID: ARFrameID
    let timestamp: Double
    let position: Vec2
    let velocity: Vec2
    let standOff: Double
    let minimumDistance: Double
}

/// One session's newest measured target, never a re-timestamped coast estimate.
@MainActor
final class FollowTargetSpeedFeed {
    var sample: FollowTargetSpeedSample?
}

enum FollowTargetSpeedScope {
    @TaskLocal static var feed: FollowTargetSpeedFeed?
}

struct FollowTargetSpeed {
    let range: Double
    let radialVelocity: Double
    let maximumWheelSpeed: Double
    let bypassesStartupRamp: Bool
    let stopDistance: Double

    static func evaluate(_ sample: FollowTargetSpeedSample?, pose: NavigationPoseSample,
                         now: Double, maximum: Double) -> Self? {
        guard let sample, let position = pose.pose?.position,
              pose.frameID?.generation == sample.frameID.generation,
              now.isFinite, sample.timestamp.isFinite, now >= sample.timestamp,
              now - sample.timestamp < 0.5,
              sample.position.x.isFinite, sample.position.y.isFinite,
              sample.velocity.x.isFinite, sample.velocity.y.isFinite,
              sample.standOff.isFinite, sample.standOff > 0,
              sample.minimumDistance.isFinite, sample.minimumDistance > 0,
              maximum.isFinite, maximum > 0 else { return nil }
        let delta = sample.position - position
        let range = hypot(delta.x, delta.y)
        guard range.isFinite else { return nil }
        let stopDistance = max(sample.minimumDistance, sample.standOff + 0.05)
        let radial = range > 0.001 ? (sample.velocity.x * delta.x + sample.velocity.y * delta.y) / range : 0
        // Feed forward only bounded target velocity. The gap term catches up;
        // the near-target cap smoothly removes speed before the holding boundary.
        let gap = max(0, range - stopDistance)
        let requested = max(0, min(maximum, max(-maximum, min(maximum, radial)) + 0.6 * gap))
        let nearCap = maximum * min(1, gap / 0.20)
        let speed = gap > 0 ? min(maximum, max(min(0.04, maximum), min(nearCap, requested))) : 0
        let catchup = range >= sample.standOff + 0.6 && radial >= 0.1
        return .init(range: range, radialVelocity: radial, maximumWheelSpeed: speed,
            bypassesStartupRamp: catchup, stopDistance: stopDistance)
    }
}
