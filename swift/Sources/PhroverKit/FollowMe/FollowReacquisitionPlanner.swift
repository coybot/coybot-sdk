import Foundation
import RoverNav

/// Geometry and provenance stay paired to their original source, never renewed on read.
public struct FollowReliableMemory: Sendable {
    public enum Association: String, Sendable { case initial, continued, acceptedPendingContinuity }
    public let position: Vec2?
    public let pairedPose: Pose2D?
    public let pairedYaw: Double?
    public let bearing: Double?
    public let frameID: ARFrameID
    public let timestamp: TimeInterval
    public let rawPersonID: Int?
    public let association: Association

    public init(position: Vec2?, pairedPose: Pose2D?, bearing: Double?, frameID: ARFrameID,
                timestamp: TimeInterval, rawPersonID: Int? = nil, association: Association, pairedYaw: Double? = nil) {
        self.position = position
        self.pairedPose = pairedPose
        self.pairedYaw = pairedPose?.yaw ?? pairedYaw
        self.bearing = bearing
        self.frameID = frameID
        self.timestamp = timestamp
        self.rawPersonID = rawPersonID
        self.association = association
    }

    /// The normal processor supplies its accepted match; this validates source/geometry,
    /// without claiming that projection or association was performed by the planner.
    public init?(accepted observation: FollowPersonObservation, association: Association,
                 now: TimeInterval, trackingQuality: ARTrackingQuality) {
        guard now.isFinite, observation.timestamp.isFinite, (0...0.5).contains(now - observation.timestamp),
              trackingQuality == .normal, observation.confidence.isFinite, observation.confidence >= 0.5,
              observation.position.x.isFinite, observation.position.y.isFinite,
              observation.pose.position.x.isFinite, observation.pose.position.y.isFinite, observation.pose.yaw.isFinite else { return nil }
        let dx = observation.position.x - observation.pose.position.x
        let dz = observation.position.y - observation.pose.position.y
        guard dx.isFinite, dz.isFinite, dx != 0 || dz != 0 else { return nil }
        self.init(position: observation.position, pairedPose: observation.pose,
            bearing: FollowReacquisitionPlanner.wrap(atan2(dz, dx) - observation.pose.yaw),
            frameID: observation.frameID, timestamp: observation.timestamp,
            rawPersonID: observation.rawPersonID, association: association)
    }
}

/// A genuinely sourced current sample; historical memory is not a substitute.
public struct FollowRecoveryPose: Sendable {
    public let pose: Pose2D
    public let frameID: ARFrameID
    public let timestamp: TimeInterval
    public let trackingQuality: ARTrackingQuality
    public init(pose: Pose2D, frameID: ARFrameID, timestamp: TimeInterval, trackingQuality: ARTrackingQuality) {
        self.pose = pose; self.frameID = frameID; self.timestamp = timestamp; self.trackingQuality = trackingQuality
    }
}

public enum FollowReacquisitionPlanner {
    public enum SegmentDecision: Equatable, Sendable {
        case turn(delta: Double, target: Double)
        case stageArrived
        case exhausted
        case unavailable
    }
    public static let offsets: [Double] = [0, 15, -15, 30, -30, 45, -45].map { $0 * .pi / 180 }
    public static let tolerance = 7 * Double.pi / 180
    /// A ten-degree segment must not complete after only three degrees of travel.
    public static let segmentTolerance = 2 * Double.pi / 180

    public static func searchOffsets(direction: Int) -> [Double] {
        guard direction != 0 else { return offsets }
        let sign = direction > 0 ? 1.0 : -1.0
        return [15.0, 30, 45, 0, -15, -30, -45].map { $0 * sign * .pi / 180 }
    }

    /// Search ordering only: neither extrapolates a position nor grants motion authority.
    public static func movementDirection(previous: FollowReliableMemory?, current: FollowReliableMemory) -> Int {
        guard let previous, previous.frameID.generation == current.frameID.generation,
              current.frameID.sequence > previous.frameID.sequence,
              (0.05...0.75).contains(current.timestamp - previous.timestamp),
              let before = previous.position, let after = current.position,
              let rover = current.pairedPose?.position,
              before.distance(to: after) >= 0.03, before.distance(to: after) <= 1,
              before.distance(to: rover) > 0.2, after.distance(to: rover) > 0.2 else { return 0 }
        let delta = wrap(atan2(after.y - rover.y, after.x - rover.x) -
            atan2(before.y - rover.y, before.x - rover.x))
        guard delta.isFinite, abs(delta) >= 2 * .pi / 180, abs(delta) <= .pi / 4 else { return 0 }
        return delta > 0 ? 1 : -1
    }

    /// A unique, nearby, fresh bearing can select the first search side. It cannot
    /// move the frozen center, refresh the anchor, or resolve a person identity.
    public static func partialDirection(anchor: FollowReliableMemory?, batch: FollowFrameBatch?,
                                        now: TimeInterval) -> Int? {
        guard let anchor, let point = anchor.position, let batch, let pose = batch.pose,
              batch.trackingQuality == .normal, batch.frameID.generation == anchor.frameID.generation,
              batch.frameID.sequence > anchor.frameID.sequence, batch.people.isEmpty,
              batch.timestamp > anchor.timestamp,
              now.isFinite, (0...0.5).contains(now - batch.timestamp),
              (0...3).contains(now - anchor.timestamp),
              point.distance(to: pose.position) > 0.2 else { return nil }
        let expected = atan2(point.y - pose.position.y, point.x - pose.position.x)
        let hints = batch.partialPeople.filter {
            $0.frameID == batch.frameID && $0.timestamp == batch.timestamp &&
            $0.confidence.isFinite && $0.confidence >= 0.5 && $0.confidence <= 1 &&
            [$0.boundingBox.minX, $0.boundingBox.minY, $0.boundingBox.width, $0.boundingBox.height,
             $0.boundingBox.maxX, $0.boundingBox.maxY].allSatisfy(\.isFinite) &&
            $0.boundingBox.width > 0 && $0.boundingBox.height > 0 &&
            $0.boundingBox.minX >= -0.01 && $0.boundingBox.minY >= -0.01 &&
            $0.boundingBox.maxX <= 1.01 && $0.boundingBox.maxY <= 1.01 &&
            $0.pose == pose && $0.bearing.isFinite &&
            abs(wrap($0.bearing - expected)) <= 20 * .pi / 180
        }
        guard hints.count == 1 else { return nil }
        let delta = wrap(hints[0].bearing - expected)
        guard abs(delta) >= 2 * .pi / 180 else { return nil }
        return delta > 0 ? 1 : -1
    }
    public static func nextSegment(center: Double, stage: Int, actualYaw: Double) -> SegmentDecision {
        guard stage >= 0, center.isFinite, actualYaw.isFinite else { return .unavailable }
        guard stage < offsets.count else { return .exhausted }
        return resolveAbsoluteStage(stageHeading: wrap(center + offsets[stage]), actualYaw: actualYaw)
    }

    /// Resolve only against an authoritative current yaw; the returned segment target
    /// stays fixed while the controller executes its pulses and corrections.
    public static func resolveAbsoluteStage(stageHeading: Double, actualYaw: Double,
                                            maximumSegment: Double = .pi / 6) -> SegmentDecision {
        guard stageHeading.isFinite, actualYaw.isFinite, maximumSegment.isFinite, maximumSegment > tolerance else { return .unavailable }
        let error = wrap(wrap(stageHeading) - actualYaw)
        if abs(error) <= tolerance { return .stageArrived }
        let delta = min(abs(error), maximumSegment, .pi / 6) * (error > 0 ? 1 : -1)
        return .turn(delta: delta, target: wrap(actualYaw + delta))
    }
    public static func wrap(_ angle: Double) -> Double {
        guard angle.isFinite else { return .nan }
        var result = angle.truncatingRemainder(dividingBy: 2 * .pi)
        if result >= .pi { result -= 2 * .pi }
        if result < -.pi { result += 2 * .pi }
        return result
    }
    public enum CenterSource: String, Sendable { case worldFromPostStopPose, historicalPairedViewHeading }
    public struct Center: Sendable {
        public let heading: Double
        public let source: CenterSource
        public let sample: FollowRecoveryPose
    }
    public static func selectCenter(anchor: FollowReliableMemory, current: FollowRecoveryPose,
                                    now: TimeInterval) -> Center? {
        guard anchor.frameID.generation == current.frameID.generation,
              anchor.timestamp.isFinite, anchor.timestamp <= now,
              current.timestamp.isFinite, now.isFinite, (0...0.5).contains(now - current.timestamp),
              current.trackingQuality == .normal,
              current.pose.position.x.isFinite, current.pose.position.y.isFinite, current.pose.yaw.isFinite else { return nil }
        if let point = anchor.position {
            let dx = point.x - current.pose.position.x
            let dz = point.y - current.pose.position.y
            if dx.isFinite, dz.isFinite, dx != 0 || dz != 0 {
                return Center(heading: wrap(atan2(dz, dx)), source: .worldFromPostStopPose, sample: current)
            }
        }
        guard let pairedYaw = anchor.pairedYaw, pairedYaw.isFinite,
              let bearing = anchor.bearing, bearing.isFinite else { return nil }
        let heading = wrap(pairedYaw + bearing)
        guard heading.isFinite else { return nil }
        return Center(heading: heading, source: .historicalPairedViewHeading, sample: current)
    }
}
