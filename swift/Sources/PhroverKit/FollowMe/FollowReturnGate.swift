import Foundation
import RoverNav

enum FollowReturnWaitReason {
    case targetLost, depthUnstable

    var message: String {
        switch self {
        case .targetLost:
            return "I can't see you. Come back into view, about two metres in front of the camera, and stand still. I'll wait for one minute and try following again when you're detected."
        case .depthUnstable:
            return "Depth isn't stable, so I've paused. Step out of view, then return about two metres in front of the camera and stand still. I'll wait for one minute and try again when tracking and depth are stable."
        }
    }
}

/// A fresh human-return interaction, not another motor-recovery attempt.
/// No motion is authorized here; success starts the normal readiness sequence.
struct FollowReturnGate {
    let generation: UInt64
    let stoppedAt: Double
    private var sawAway: Bool
    private var awaySince: Double?
    private var lastSequence: UInt64?
    private var lastTimestamp: Double?
    private var stableSince: Double?
    private var stablePosition: Vec2?
    private var stableCount = 0

    init(generation: UInt64, stoppedAt: Double, reason: FollowReturnWaitReason) {
        self.generation = generation; self.stoppedAt = stoppedAt
        sawAway = reason == .targetLost
    }

    mutating func observe(_ batch: FollowFrameBatch, now: Double, ready: Bool,
                          configuration: FollowMeConfiguration) -> Bool {
        guard batch.frameID.generation == generation,
              lastSequence.map({ batch.frameID.sequence > $0 }) ?? true,
              batch.timestamp > stoppedAt, lastTimestamp.map({ batch.timestamp > $0 }) ?? true else {
            awaySince = nil; resetStable(); return false
        }
        if let lastTimestamp, batch.timestamp - lastTimestamp > configuration.maximumObservationAge {
            awaySince = nil; resetStable()
        }
        lastSequence = batch.frameID.sequence; lastTimestamp = batch.timestamp
        guard FollowFrameHealth.issue(batch, now: now, configuration: configuration) == nil,
              batch.trackingQuality == .normal, let pose = batch.pose else {
            awaySince = nil; resetStable(); return false
        }
        let people = batch.people.filter {
            $0.bodyVerified && $0.confidence >= configuration.minimumConfidence &&
                $0.frameID == batch.frameID && $0.timestamp == batch.timestamp &&
                $0.position.x.isFinite && $0.position.y.isFinite
        }
        let range = people.count == 1 ? people[0].position.distance(to: pose.position) : nil
        // An unverified or ambiguous detection is not evidence that the user left.
        guard batch.partialPeople.isEmpty, people.count == batch.people.count, people.count <= 1 else {
            awaySince = nil; resetStable(); return false
        }
        if batch.people.isEmpty || range.map({ $0 > 2.5 }) == true {
            awaySince = awaySince ?? batch.timestamp
            if batch.timestamp - awaySince! >= 0.5 { sawAway = true }
            resetStable(); return false
        }
        awaySince = nil
        guard sawAway, ready, people.count == 1, let range,
              range >= max(configuration.readySignalClearance, configuration.standOffDistance), range <= 2.5 else {
            resetStable(); return false
        }
        let point = people[0].position
        if stablePosition.map({ $0.distance(to: point) > 0.25 }) ?? true {
            stablePosition = point; stableSince = batch.timestamp; stableCount = 0
        }
        stableCount += 1
        return stableCount >= 3 && batch.timestamp - stableSince! >= 0.25
    }

    private mutating func resetStable() { stableSince = nil; stablePosition = nil; stableCount = 0 }
}
