import Foundation
import RoverNav

/// One depth-projected person detection and rover pose from the same AR frame.
public struct FollowPersonObservation {
    public let frameID: ARFrameID
    public let timestamp: TimeInterval
    public let confidence: Float
    /// Normalized Vision coordinates.
    public let boundingBox: CGRect
    public let position: Vec2
    public let pose: Pose2D
    /// Canonical-person index within this frame's raw detections; nil for older providers.
    public let rawPersonID: Int?
    /// Same-frame independent torso evidence. Allows visible side/top image
    /// edges; feet and depth-neighborhood validity remain mandatory.
    public let bodyVerified: Bool

    public init(frameID: ARFrameID, timestamp: TimeInterval, confidence: Float,
                boundingBox: CGRect, position: Vec2, pose: Pose2D, rawPersonID: Int? = nil, bodyVerified: Bool = false) {
        self.frameID = frameID
        self.timestamp = timestamp
        self.confidence = confidence
        self.boundingBox = boundingBox
        self.position = position
        self.pose = pose
        self.rawPersonID = rawPersonID
        self.bodyVerified = bodyVerified
    }
}

/// A person detection that could not become a full `FollowPersonObservation` (body check failed,
/// feet or hips out of frame or hidden, low confidence) but still shows which way they are.
/// Only the direction is trustworthy: depth read through a partly hidden body tends to land on
/// whatever is in front of it. Used solely to keep an existing track alive.
public struct FollowBearingObservation {
    public let frameID: ARFrameID
    public let timestamp: TimeInterval
    public let confidence: Float
    /// Normalized Vision coordinates.
    public let boundingBox: CGRect
    /// Nav-plane azimuth (`atan2(dy, dx)`, radians) of the ray through the box centre, from `pose.position`.
    public let bearing: Double
    public let pose: Pose2D
    public let rawPersonID: Int?

    public init(frameID: ARFrameID, timestamp: TimeInterval, confidence: Float, boundingBox: CGRect,
                bearing: Double, pose: Pose2D, rawPersonID: Int? = nil) {
        self.frameID = frameID
        self.timestamp = timestamp
        self.confidence = confidence
        self.boundingBox = boundingBox
        self.bearing = bearing
        self.pose = pose
        self.rawPersonID = rawPersonID
    }
}

public enum FollowTrackMatch {
    case matched(FollowPersonObservation)
    case lost
    case ambiguous
}

/// Shared follow thresholds in metres, seconds, radians, and normalized image units.
public struct FollowMeConfiguration {
    public var minimumConfidence: Float = 0.50
    public var maximumObservationAge: TimeInterval = 0.5
    public var maximumWorldDistance: Double = 0.75
    public var minimumBoxIoU: CGFloat = 0.10
    public var maximumScreenCenterDistance: CGFloat = 0.25
    public var reacquisitionDistance: Double = 1.5
    public var standOffDistance: Double = 1.5
    public var minimumHoldDistance: Double = 1.25
    public var readySignalClearance: Double { minimumHoldDistance + 0.12 }
    public var maximumHoldDistance: Double = 1.75
    public var scanIncrement: Double = .pi / 18
    /// Stationary observation after each completed search step. The next step
    /// requires a healthy processed capture taken after this interval.
    public var scanObservationSeconds: TimeInterval = 0.1
    public var maximumScanRotation: Double = 2 * .pi
    public var maximumScanAttempts: Int = 72
    public var maximumScanInterruptions: Int = 8
    public var minimumGoalChange: Double = 0.30
    public var maximumGoalUpdatesPerSecond: Double = 3
    public var perceptionRecoverySeconds: TimeInterval = 2
    public var startupReadinessSeconds: TimeInterval = 5
    public var stationaryPauseSeconds: TimeInterval = 5
    /// Bounded stopped grace for a previously body-verified acquisition whose
    /// next raw person fails verification. Never permits motion on stale evidence.
    public var acquisitionVerificationGraceSeconds: TimeInterval = 0.5
    public var departureRangeIncrease: Double = 0.30
    /// Matches NavigationController's continuous rotation tolerance (radians).
    public var alignmentAngularTolerance: Double = 0.05
    public var reacquisitionSeconds: TimeInterval = 10

    // MARK: Continuous following (all off by default; `.continuous` turns them on)

    /// Keep one follow drive running and move its goal with the person, instead of stopping,
    /// confirming the stop and relaunching for every goal change.
    public var continuousGoals = false
    public var continuousMinimumGoalChange: Double = 0.10
    public var continuousMaximumGoalUpdatesPerSecond: Double = 5
    /// How far ahead of the person's estimated velocity the stand-off goal is aimed (s).
    public var goalLeadSeconds: TimeInterval = 0.8
    public var maximumGoalLead: Double = 1.0
    /// Constant-velocity track that carries the estimate through detection dropouts. While the
    /// target is unseen for at most this long the follower keeps moving toward the predicted
    /// position instead of stopping to search. 0 disables the track entirely.
    public var coastSeconds: TimeInterval = 0
    /// Let a person who fails the full-body check (partly hidden, too close, clipped, low
    /// confidence) keep an existing track alive through a bearing-only update. Never used to
    /// acquire a new target.
    public var tolerantContinuation = false
    public var minimumPartialConfidence: Float = 0.25
    /// Longest the track may be sustained by bearings alone, with no full position measurement.
    public var maximumBearingOnlySeconds: TimeInterval = 6
    public var bearingSigma: Double = 0.04
    /// A bearing-only update further than this from the track's predicted direction is someone
    /// else, however uncertain the track has become while coasting (rad).
    public var maximumBearingInnovation: Double = 0.35

    public init() {}

    /// Continuous pursuit, 3 s of coasting and tolerant continuation. Tuned in the Godot
    /// follow bench (eco/rover/sim/follow_bench).
    public static var continuous: FollowMeConfiguration {
        var configuration = FollowMeConfiguration()
        configuration.continuousGoals = true
        configuration.coastSeconds = 3
        configuration.tolerantContinuation = true
        return configuration
    }
}
