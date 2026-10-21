import Foundation

/// Value-only frozen episode. Replacement values preserve identity, anchor and budget.
public struct FollowReacquisitionEpisode: Sendable {
    public let id: UUID
    public let firstLoss: TimeInterval
    public let deadline: TimeInterval
    public let anchor: FollowReliableMemory?
    public let center: FollowReacquisitionPlanner.Center?
    public let stageIndex: Int
    public let segmentIndex: Int
    public let unavailableReason: String?

    public init(id: UUID = UUID(), firstLoss: TimeInterval, anchor: FollowReliableMemory?) {
        self.id = id; self.firstLoss = firstLoss; self.deadline = firstLoss + 10
        self.anchor = anchor; self.center = nil; self.stageIndex = 0; self.segmentIndex = 0
        self.unavailableReason = anchor == nil ? "missing_reliable_memory" : nil
    }
    private init(episode: Self, center: FollowReacquisitionPlanner.Center?, stage: Int, segment: Int, reason: String?) {
        id = episode.id; firstLoss = episode.firstLoss; deadline = episode.deadline; anchor = episode.anchor
        self.center = center; stageIndex = stage; segmentIndex = segment; unavailableReason = reason
    }
    public func selectingCenter(current: FollowRecoveryPose, now: TimeInterval) -> Self {
        guard center == nil else { return self }
        guard now.isFinite, deadline.isFinite, now < deadline, let anchor else { return self }
        let selection = FollowReacquisitionPlanner.selectCenter(anchor: anchor, current: current, now: now)
        return Self(episode: self, center: selection, stage: stageIndex, segment: segmentIndex,
                    reason: selection == nil ? "no_valid_center_or_source" : nil)
    }
    /// Command completion alone never advances a stage. Supply an authoritative fresh pose.
    public func recordingArrival(actual: FollowRecoveryPose, now: TimeInterval) -> Self {
        guard now.isFinite, now < deadline, let center, let anchor,
              actual.frameID.generation == anchor.frameID.generation,
              actual.trackingQuality == .normal, actual.timestamp.isFinite,
              (0...0.5).contains(now - actual.timestamp),
              actual.pose.position.x.isFinite, actual.pose.position.y.isFinite else { return self }
        let decision = FollowReacquisitionPlanner.nextSegment(center: center.heading, stage: stageIndex, actualYaw: actual.pose.yaw)
        switch decision {
        case .stageArrived:
            return Self(episode: self, center: center, stage: stageIndex + 1, segment: 0, reason: nil)
        case .turn, .exhausted, .unavailable: return self
        }
    }
    public func recordingSegmentArrival(target: Double, actual: FollowRecoveryPose, now: TimeInterval) -> Self {
        guard now.isFinite, now < deadline, let center, let anchor, stageIndex < FollowReacquisitionPlanner.offsets.count,
              actual.frameID.generation == anchor.frameID.generation, actual.trackingQuality == .normal,
              actual.timestamp.isFinite, (0...0.5).contains(now - actual.timestamp),
              actual.pose.position.x.isFinite, actual.pose.position.y.isFinite,
              abs(FollowReacquisitionPlanner.wrap(target - actual.pose.yaw)) <= FollowReacquisitionPlanner.tolerance else { return self }
        let stageArrival = recordingArrival(actual: actual, now: now)
        if stageArrival.stageIndex != stageIndex { return stageArrival }
        return Self(episode: self, center: center, stage: stageIndex, segment: segmentIndex + 1, reason: nil)
    }
}
