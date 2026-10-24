import Foundation
import RoverNav

/// Shared frame health gates. Diagnostics do not influence the decision.
enum FollowFrameHealth {
    static func issue(_ batch: FollowFrameBatch, now: TimeInterval,
                      configuration: FollowMeConfiguration) -> FollowPerceptionIssue? {
        let age = now - batch.timestamp
        if !age.isFinite || age < 0 || age > configuration.maximumObservationAge { return .staleFrame }
        if batch.trackingQuality == .unavailable || batch.trackingQuality == nil { return .trackingUnavailable }
        if batch.trackingQuality == .limited { return .trackingLimited }
        guard let pose = batch.pose, pose.position.x.isFinite, pose.position.y.isFinite, pose.yaw.isFinite else { return .poseUnavailable }
        if !batch.depthAvailable { return .depthUnavailable }
        return nil
    }

    static func rejectionCondition(_ issue: FollowPerceptionIssue, batch: FollowFrameBatch, now: TimeInterval) -> String {
        switch issue {
        case .staleFrame:
            if !batch.timestamp.isFinite { return "frame_timestamp_nonfinite" }
            if !now.isFinite { return "observation_clock_nonfinite" }
            return now < batch.timestamp ? "frame_from_future" : "frame_stale"
        case .trackingLimited: return "tracking_limited"
        case .trackingUnavailable: return batch.trackingQuality == nil ? "tracking_unknown" : "tracking_unavailable"
        case .poseUnavailable: return batch.pose == nil ? "frame_pose_missing" : "frame_pose_nonfinite"
        case .depthUnavailable: return "depth_unavailable"
        case .noFrames: return "frame_missing"
        }
    }
}

/// Pure evaluation of the newest observation, against the original locked track.
/// A pending evaluation is reused by the frame processor, never associated twice.
struct FollowAdmissionSnapshot {
    let batch: FollowFrameBatch
    let person: FollowPersonObservation?
    let association: (decision: FollowTrackMatch, evaluation: FollowAssociationEvaluation)?
    let rejection: String?
    let pending: Bool
    let evaluatedAt: TimeInterval

    init(batch: FollowFrameBatch, previous: FollowPersonObservation?, pending: Bool,
          tracker: FollowTargetTracker, now: TimeInterval, configuration: FollowMeConfiguration,
          acceptedAssociation: (decision: FollowTrackMatch, evaluation: FollowAssociationEvaluation)? = nil) {
        self.batch = batch
        self.pending = pending
        self.evaluatedAt = now
        var person: FollowPersonObservation?
        var association: (decision: FollowTrackMatch, evaluation: FollowAssociationEvaluation)?
        var rejection: String?
        if let issue = FollowFrameHealth.issue(batch, now: now, configuration: configuration) {
            rejection = FollowFrameHealth.rejectionCondition(issue, batch: batch, now: now)
        } else if let previous {
            if batch.frameID.generation != previous.frameID.generation {
                rejection = "frame_generation_mismatch"
            } else if pending || acceptedAssociation != nil {
                let evaluated = acceptedAssociation ?? tracker.continueTrackEvaluated(batch.people, previous: previous,
                    predictedPosition: previous.position, now: now, frameID: batch.frameID)
                association = evaluated
                switch evaluated.decision {
                case .matched(let selected): person = selected
                case .lost: rejection = "target_lost"
                case .ambiguous: rejection = "target_ambiguous"
                }
            } else {
                person = previous
            }
            if let selected = person {
                if selected.frameID != batch.frameID { rejection = "locked_frame_mismatch" }
                else if !selected.timestamp.isFinite { rejection = "observation_timestamp_nonfinite" }
                else if now < selected.timestamp { rejection = "observation_from_future" }
                else if now - selected.timestamp > configuration.maximumObservationAge { rejection = "observation_stale" }
            }
        } else { rejection = "locked_target_missing" }
        self.person = person
        self.association = association
        self.rejection = rejection
    }
}
