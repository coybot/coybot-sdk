import Foundation
import RoverNav

/// One immutable readiness evaluation. No motion policy, timers, or retained frames.
struct FollowReadinessDiagnostics {
    @MainActor
    static func controllerPayload(_ admission: FollowReadyAdmission?, perceptionFrame: ARFrameID?, sendAuthorized: Bool,
                                  person: FollowPersonObservation? = nil) -> [String: FollowDiagnosticValue] {
        let sample = admission?.controllerSample
        let frame = sample?.frameID
        let timestamp = sample?.sourceTimestamp
        let read = admission?.controllerReadUptime
        let age = timestamp.flatMap { timestamp in read.map { $0 - timestamp } }
        let pairing = frame == nil || perceptionFrame == nil ? "unknown" : frame == perceptionFrame ? "same_frame" : "independently_sampled"
        var result: [String: FollowDiagnosticValue] = [
            "admission_boundary": admission?.boundary.map { .string($0.rawValue) } ?? .null,
            "first_wheel_telemetry_availability": .string(admission?.boundary == .controllerFirstSend && sendAuthorized ? "send_initiation_only" : "unknown"),
            "controller_pose_frame_id": frame.map { .string("\($0.generation):\($0.sequence)") } ?? .null,
            "controller_pose_generation": frame.map { .number(Double($0.generation)) } ?? .null,
            "controller_source_timestamp_s": timestamp.map { .number($0) } ?? .null,
            "controller_source_age_s": age.map { .number($0) } ?? .null,
            "controller_source_age_s_availability": .string(age == nil ? "unknown" : "available"),
            "controller_read_uptime_s": read.map { .number($0) } ?? .null,
            "controller_source_clock": .string(timestamp == nil ? "unknown" : "system_uptime"),
            "controller_tracking_state": sample?.trackingQuality.map { .string(String(describing: $0)) } ?? .null,
            "controller_pose_yaw_rad": sample?.pose.map { .number($0.yaw) } ?? .null,
            "controller_pose_position": sample?.pose.map { .object(["x": .number($0.position.x), "y": .number($0.position.y)]) } ?? .null,
            "controller_perception_pairing": .string(pairing)
        ]
        if let sample, sample.source != "legacy_unknown", let pose = sample.pose,
           let frame, frame.generation == perceptionFrame?.generation,
           let person, person.frameID == perceptionFrame {
            let heading = RotationDiagnosticMeasurement.normalize(atan2(person.position.y - pose.position.y,
                person.position.x - pose.position.x) - pose.yaw)
            result["range_m"] = .number(pose.position.distance(to: person.position))
            result["heading_rad"] = .number(heading)
            result["heading_rad_display"] = .string(RotationDiagnosticMeasurement.angleDisplay(heading))
            result["readiness_geometry_availability"] = .string("controller_pose_locked_world_position")
        }
        return result
    }

    let batch: FollowFrameBatch?
    let person: FollowPersonObservation?
    let now: Double
    let gate: Double
    let pending: Bool
    let attempted: Bool
    let succeeded: Bool
    let stopState: String

    var payload: [String: FollowDiagnosticValue] {
        let paired = person?.frameID == batch?.frameID && person != nil
        let range = paired ? person.map { $0.pose.position.distance(to: $0.position) } : nil
        let heading = paired ? person.map {
            RotationDiagnosticMeasurement.normalize(atan2($0.position.y - $0.pose.position.y,
                $0.position.x - $0.pose.position.x) - $0.pose.yaw)
        } : nil
        var result = batch.map { FollowAssociationEvaluation.healthPayload(batch: $0, now: now) } ?? [:]
        result.merge([
            "ready_signal_clearance_m": .number(gate), "range_m": range.map { .number($0) } ?? .null,
            "heading_rad": heading.map { .number($0) } ?? .null,
            "heading_rad_display": heading.map { .string(RotationDiagnosticMeasurement.angleDisplay($0)) } ?? .null,
            "ready_admission_pending": .bool(pending), "ready_signal_attempted": .bool(attempted),
            "ready_signal_succeeded": .bool(succeeded), "stop_state": .string(stopState),
            "readiness_geometry_availability": .string(paired ? "observation_paired_snapshot" : "not_evaluated")
        ]) { _, new in new }
        return result
    }
}
