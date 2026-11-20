import Foundation
import RoverNav

/// A search destination anchored to a real sighting, not a new person
/// measurement. Limits are fixed at entry and cannot be renewed by misses.
struct FollowLastSeenApproach: Equatable, Sendable {
    static let maximumAge = 3.0
    static let maximumDisplacement = 0.5
    static let maximumWheelSpeed = 0.12
    let origin: Vec2

    func reachedLimit(position: Vec2, observedAt: Double, now: Double) -> Bool {
        now >= observedAt + Self.maximumAge || position.distance(to: origin) >= Self.maximumDisplacement
    }
}

struct FollowTargetSpeedSample: Sendable {
    let frameID: ARFrameID
    let timestamp: Double
    let position: Vec2
    let velocity: Vec2
    let standOff: Double
    let minimumDistance: Double
    var holdsTranslation: Bool = false
    var centersContinuously: Bool = false
    var recoversBlockedMotion: Bool = false
    var lastSeenApproach: FollowLastSeenApproach?

    var validUntil: Double {
        timestamp + (lastSeenApproach == nil ? 0.5 : FollowLastSeenApproach.maximumAge)
    }
}

/// One session's measured target or explicit bounded last-seen destination.
/// Both retain the original capture timestamp; neither retimestamps an estimate.
@MainActor
final class FollowTargetSpeedFeed {
    var sample: FollowTargetSpeedSample?
}

enum FollowTargetSpeedScope {
    @TaskLocal static var feed: FollowTargetSpeedFeed?
}

/// Hysteresis keeps small measurement jitter from repeatedly restarting a hold turn.
enum FollowHoldingTurn {
    static func stopMargin(continuous: Bool) -> Double { continuous ? 0.15 : 0.05 }
    static let entryAngle = Double.pi / 18
    static let completionAngle = Double.pi / 36
    static let continuousEntryAngle = 2 * Double.pi / 180
    static let continuousCompletionAngle = Double.pi / 180
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
              now < sample.validUntil,
              sample.position.x.isFinite, sample.position.y.isFinite,
              sample.velocity.x.isFinite, sample.velocity.y.isFinite,
              sample.standOff.isFinite, sample.standOff > 0,
              sample.minimumDistance.isFinite, sample.minimumDistance > 0,
              maximum.isFinite, maximum > 0 else { return nil }
        if let approach = sample.lastSeenApproach {
            guard approach.origin.x.isFinite, approach.origin.y.isFinite,
                  !sample.holdsTranslation,
                  !approach.reachedLimit(position: position, observedAt: sample.timestamp, now: now) else { return nil }
        }
        let maximum = sample.lastSeenApproach == nil ? maximum : min(maximum, FollowLastSeenApproach.maximumWheelSpeed)
        let delta = sample.position - position
        let range = hypot(delta.x, delta.y)
        guard range.isFinite else { return nil }
        let stopDistance = max(sample.minimumDistance, sample.standOff +
            FollowHoldingTurn.stopMargin(continuous: sample.centersContinuously && sample.lastSeenApproach == nil))
        let radial = range > 0.001 && sample.lastSeenApproach == nil
            ? (sample.velocity.x * delta.x + sample.velocity.y * delta.y) / range : 0
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
