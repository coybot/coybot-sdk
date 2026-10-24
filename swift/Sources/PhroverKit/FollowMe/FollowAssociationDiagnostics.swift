import Foundation
import RoverNav

/// One session's healthy-summary allowance and last evaluated outcome; no frame history.
struct FollowSummaryBudget {
    private(set) var previousOutcome: String?
    private var lastHealthy: Double?
    private var previousAssociationSignature: String?
    private var previousPipelineSignature: String?

    mutating func takeHealthy(now: Double) -> Bool {
        guard now.isFinite, lastHealthy == nil || now >= lastHealthy! + 1 else { return false }
        lastHealthy = now
        return true
    }

    mutating func takeAssociation(outcome: String, now: Double, reasons: String = "") -> Bool {
        let signature = outcome + "|" + reasons
        let transition = signature != previousAssociationSignature
        previousAssociationSignature = signature
        previousOutcome = outcome
        if transition {
            // Healthy transitions bypass the allowance but occupy this frame's slot,
            // preventing an immediate duplicate periodic perception summary.
            if outcome == "initial" || outcome == "continued" || outcome == "reacquired", now.isFinite {
                lastHealthy = now
            }
            return true
        }
        return outcome == "continued" && takeHealthy(now: now)
    }

    mutating func takePipeline(signature: String, now: Double, healthy: Bool) -> Bool {
        let transition = previousPipelineSignature != signature
        previousPipelineSignature = signature
        if transition { if now.isFinite { lastHealthy = now }; return true }
        return healthy && takeHealthy(now: now)
    }
}

/// Immutable evidence from the tracker's single authoritative gate execution.
struct FollowAssociationEvaluation {
    let mode: String
    let outcome: String
    let candidates: [[String: FollowDiagnosticValue]]
    let selectedIndex: Int?
    let eligibleCount: Int
    let matchedCount: Int?
    let thresholds: [String: FollowDiagnosticValue]

    func payload(batch: FollowFrameBatch, now: Double, previousOutcome: String?) -> [String: FollowDiagnosticValue] {
        var result = Self.healthPayload(batch: batch, now: now)
        let association: [String: FollowDiagnosticValue] = [
            "association_outcome": .string(outcome), "previous_outcome": previousOutcome.map { .string($0) } ?? .null,
            "association_mode": .string(mode),
            "tracker_input_count": .number(Double(candidates.count)),
            "eligible_candidate_count": .number(Double(eligibleCount)),
            "selected_candidate_count": .number(selectedIndex == nil ? 0 : 1),
            "tracker_counts_availability": .string("available"),
            "matched_candidate_count": matchedCount.map { .number(Double($0)) } ?? .null,
            "matched_candidate_count_availability": .string(matchedCount == nil ? "not_applicable_initial_selection" : "available"),
            "selected_candidate": selectedIndex.map { .object(candidates[$0]) } ?? .null,
            "candidate_evaluations": .array(candidates.map { candidate in
                var payload = candidate
                payload["gate_thresholds"] = .object(thresholds)
                return .object(payload)
            }),
            "gate_metrics": .array(candidates.map { $0["gate_metrics"] ?? .null }),
            "gate_thresholds": .object(thresholds),
            "candidate_availability": .string(candidates.isEmpty ? "no projected person candidate available" : "available"),
            "coordinate_convention": .string("Vec2.x=world_X;Vec2.y=world_Z"),
            "identity_claim": .string("spatial_association_only")
        ]
        result.merge(association) { _, new in new }
        return result
    }

    static func healthPayload(batch: FollowFrameBatch, now: Double) -> [String: FollowDiagnosticValue] {
        let age = now - batch.timestamp
        let evaluated = batch.perceptionDiagnostics?.inferenceStatus == .executed
        let paired = batch.pose != nil && (evaluated || !batch.people.isEmpty)
            && batch.perceptionDiagnostics?.inferenceStatus != .failed
            && batch.perceptionDiagnostics?.inferenceStatus != .skippedTracking
            && batch.people.allSatisfy { $0.frameID == batch.frameID }
        var result: [String: FollowDiagnosticValue] = [
            "frame_id": .string("\(batch.frameID.generation):\(batch.frameID.sequence)"),
            "observation_monotonic_s": .number(batch.timestamp), "observation_age_s": .number(age),
            "tracking_state": batch.trackingQuality.map { .string(String(describing: $0)) } ?? .null,
            "tracking_state_availability": .string(batch.trackingQuality == nil ? "unknown" : "available"),
            "tracking_reason": batch.trackingReason.map { .string($0.rawValue) } ?? .null,
            "tracking_reason_availability": .string(batch.trackingReason == nil ? "unknown" : "available"),
            "same_frame": paired ? .bool(true) : .null,
            "same_frame_availability": .string(paired ? "same_frame" : "unknown"),
            "pose_available": .bool(batch.pose != nil), "depth_available": .bool(batch.depthAvailable),
            "projected_person_count": batch.perceptionDiagnostics.map {
                $0.projectedPersonCount.map { .number(Double($0)) } ?? .null
            } ?? .number(Double(batch.people.count)),
            "projected_person_count_availability": .string(batch.perceptionDiagnostics != nil
                && batch.perceptionDiagnostics?.projectedPersonCount == nil ? "not_evaluated" : "available"),
            "eligible_candidate_count": .null, "matched_candidate_count": .null, "selected_candidate_count": .null,
            "tracker_counts_availability": .string("not_evaluated"),
            "inference_duration_s": batch.inferenceDuration.map { .number($0) } ?? .null,
            "inference_duration_s_availability": .string(batch.inferenceDuration == nil ? "not_measured" : "available")
        ]
        result.merge(FollowPerceptionDiagnostics.payload(batch.perceptionDiagnostics)) { _, new in new }
        return result
    }
}

/// A transient recorder. It contains facts, never threshold or selection policy.
struct FollowCandidateEvidence {
    static let metricNames = ["confidence", "observation_age_s", "finite_geometry", "world_distance_m",
                              "box_iou", "screen_displacement", "reacquisition_distance_m"]
    var metrics: [String: FollowDiagnosticValue] = Dictionary(uniqueKeysWithValues:
        metricNames.flatMap { [($0, FollowDiagnosticValue.null), ($0 + "_availability", .string("not_evaluated"))] })
    var rejection: String?
    var eligible = false
    var matched: Bool?

    mutating func record(_ name: String, _ value: Double) {
        metrics[name] = value.isFinite ? .number(value) : .null
        metrics[name + "_availability"] = .string(value.isFinite ? "available" : "nonfinite")
    }

    func payload(_ observation: FollowPersonObservation, index: Int) -> [String: FollowDiagnosticValue] {
        let box = observation.boundingBox
        let pose = observation.pose
        let dx = observation.position.x - pose.position.x
        let dy = observation.position.y - pose.position.y
        let range = hypot(dx, dy)
        let geometryAvailable = dx.isFinite && dy.isFinite && range.isFinite && pose.yaw.isFinite
        let heading = RotationDiagnosticMeasurement.normalize(atan2(dy, dx) - pose.yaw)
        return [
            "candidate_id": .number(Double(observation.rawPersonID ?? index)), "candidate_id_scope": .string("frame_local"),
            "raw_person_id": observation.rawPersonID.map { .number(Double($0)) } ?? .null,
            "raw_person_id_availability": .string(observation.rawPersonID == nil ? "unknown_legacy_provider" : "available"),
            "frame_id": .string("\(observation.frameID.generation):\(observation.frameID.sequence)"),
            "observation_monotonic_s": .number(observation.timestamp),
            "confidence": .number(Double(observation.confidence)),
            "bounding_box": .object(["x": .number(Double(box.origin.x)), "y": .number(Double(box.origin.y)),
                "width": .number(Double(box.width)), "height": .number(Double(box.height))]),
            "projected_position": .object(["x": .number(observation.position.x), "y": .number(observation.position.y)]),
            "rover_pose": .object(["x": .number(pose.position.x), "y": .number(pose.position.y), "yaw_rad": .number(pose.yaw)]),
            "coordinate_convention": .string("Vec2.x=world_X;Vec2.y=world_Z"),
            "same_frame": .bool(true), "geometry_provenance": .string("observation_paired_snapshot"),
            "range_m": geometryAvailable ? .number(range) : .null,
            "heading_rad": geometryAvailable ? .number(heading) : .null,
            "geometry_availability": .string(geometryAvailable ? "available" : "nonfinite_paired_geometry"),
            "eligible": .bool(eligible), "matched": matched.map { .bool($0) } ?? .null,
            "matched_availability": .string(matched == nil ? "not_evaluated" : "available"),
            "rejection_reason": rejection.map { .string($0) } ?? .null,
            "gate_metrics": .object(metrics)
        ]
    }
}
