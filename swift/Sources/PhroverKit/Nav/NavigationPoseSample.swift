import Foundation
import RoverNav

/// Immutable facts from one source snapshot. Source time is AR/system uptime,
/// separate from transport UTC and the controller's Date-based watchdog.
struct NavigationPoseSample: Sendable {
    let pose: Pose2D?
    let frameID: ARFrameID?
    let sourceTimestamp: TimeInterval?
    let trackingQuality: ARTrackingQuality?
    let source: String

    init(pose: Pose2D?, frameID: ARFrameID?, sourceTimestamp: TimeInterval?,
         trackingQuality: ARTrackingQuality?, source: String = "ar_snapshot") {
        self.pose = pose
        self.frameID = frameID
        self.sourceTimestamp = sourceTimestamp
        self.trackingQuality = trackingQuality
        self.source = source
    }

    init(snapshot: ARFrameSnapshot) {
        self.init(pose: snapshot.pose, frameID: snapshot.id, sourceTimestamp: snapshot.timestamp,
                  trackingQuality: snapshot.trackingQuality)
    }

    static func legacy(_ pose: Pose2D?) -> Self {
        .init(pose: pose, frameID: nil, sourceTimestamp: nil, trackingQuality: nil, source: "legacy_unknown")
    }

    static let unavailable = Self(pose: nil, frameID: nil, sourceTimestamp: nil, trackingQuality: nil)

    func rejection(at uptime: TimeInterval, expectedGeneration: UInt64? = nil,
                   requireEnriched: Bool = false) -> String? {
        if requireEnriched, source == "legacy_unknown" { return "missing_follow_provenance" }
        guard let pose else { return "missing_pose" }
        guard pose.position.x.isFinite, pose.position.y.isFinite, pose.yaw.isFinite else { return "nonfinite_pose" }
        if source != "legacy_unknown" {
            guard let frameID else { return "missing_frame" }
            if let expectedGeneration, frameID.generation != expectedGeneration { return "source_generation_changed" }
            guard trackingQuality == .normal else { return "unhealthy_tracking" }
            guard let sourceTimestamp else { return "missing_source_time" }
            guard sourceTimestamp.isFinite, uptime.isFinite else { return "nonfinite_source_time" }
            if uptime - sourceTimestamp < 0 { return "future_source" }
            if uptime - sourceTimestamp > 0.500 { return "stale_source" }
        }
        return nil
    }
}
