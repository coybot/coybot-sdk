import Foundation
import RoverNav

public enum PoseTrackingQuality: Equatable, Sendable {
    case normal
    case limited
    case unavailable
}

public struct PoseObservation: Equatable, Sendable {
    public let pose: Pose2D
    public let frameSequence: UInt64
    public let timestamp: TimeInterval
    public let trackingQuality: PoseTrackingQuality
    public let sessionGeneration: UInt64

    public init(pose: Pose2D,
                frameSequence: UInt64,
                timestamp: TimeInterval,
                trackingQuality: PoseTrackingQuality,
                sessionGeneration: UInt64) {
        self.pose = pose
        self.frameSequence = frameSequence
        self.timestamp = timestamp
        self.trackingQuality = trackingQuality
        self.sessionGeneration = sessionGeneration
    }

    public var transitionObservation: TransitionObservation {
        TransitionObservation(
            pose: pose,
            frameSequence: frameSequence,
            timestamp: timestamp,
            isTrackingNormal: trackingQuality == .normal,
            sessionGeneration: sessionGeneration
        )
    }
}
