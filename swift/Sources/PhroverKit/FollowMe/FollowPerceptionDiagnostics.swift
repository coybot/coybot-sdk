import Foundation
import RoverNav

public enum FollowProjectionRejection: String, Sendable {
    case invalidBox = "invalid_box", clippedBox = "clipped_box", depthUnavailable = "depth_unavailable"
    case invalidDepthLayout = "invalid_depth_layout", invalidConfidenceMap = "invalid_confidence_map"
    case clippedDepthWindow = "clipped_depth_window", insufficientValidDepth = "insufficient_valid_depth"
    case inconsistentDepth = "inconsistent_depth", invalidCalibration = "invalid_calibration"
    case nonfiniteProjection = "nonfinite_projection"
}

public enum FollowConfidenceAvailability: String, Sendable { case unavailable, available, invalid }
public enum FollowInferenceStatus: String, Sendable {
    case executed, skippedTracking = "skipped_tracking", failed, unknown
}

/// Bounded facts from one evaluation; no images, buffers, or sample arrays are retained.
public struct FollowProjectionEvidence {
    public let rawPersonID: Int
    public let box: CGRect
    public let detectorConfidence: Float
    public let imageSize: CGSize
    public let depthSource: ARDepthSource?
    public internal(set) var depthSize: CGSize?
    public internal(set) var depthBytesPerRow: Int?
    public internal(set) var depthPixelFormat: UInt32?
    public internal(set) var confidenceSize: CGSize?
    public internal(set) var confidenceBytesPerRow: Int?
    public internal(set) var confidencePixelFormat: UInt32?
    public internal(set) var confidenceAvailability: FollowConfidenceAvailability
    public internal(set) var clippedLeft: Bool?
    public internal(set) var clippedBottom: Bool?
    public internal(set) var clippedRight: Bool?
    public internal(set) var clippedTop: Bool?
    public internal(set) var feet: CGPoint?
    public internal(set) var depthAnchor: CGPoint?
    public internal(set) var depthAnchorKind: String?
    public internal(set) var sensorPixel: CGPoint?
    public internal(set) var depthPixel: CGPoint?
    public internal(set) var depthCenter: CGPoint?
    /// Policy request; evaluated counts stay nil until the full window is read.
    public let requestedSampleCount = 25
    /// Mutually exclusive partition: invalid depth wins over low/invalid confidence.
    public internal(set) var validSampleCount: Int?
    public internal(set) var invalidDepthCount: Int?
    public internal(set) var lowConfidenceCount: Int?
    public internal(set) var medianDepth: Double?
    public internal(set) var medianAbsoluteDeviation: Double?
    public internal(set) var inlierCount: Int?
    public internal(set) var requiredInlierCount: Int?
    public internal(set) var position: Vec2?
    public internal(set) var cameraRay: SIMD3<Double>?
    public internal(set) var worldPoint: SIMD3<Float>?
    public internal(set) var pairedPose: Pose2D?
    public internal(set) var groundRange: Double?
    public internal(set) var headingError: Double?
    public internal(set) var rejection: FollowProjectionRejection?

    init(rawPersonID: Int, box: CGRect, detectorConfidence: Float, snapshot: ARFrameSnapshot) {
        self.rawPersonID = rawPersonID
        self.box = box
        self.detectorConfidence = detectorConfidence
        imageSize = snapshot.imageResolution
        depthSource = snapshot.depthSource
        confidenceAvailability = snapshot.depthConfidenceMap == nil ? .unavailable : .available
    }
}

public struct FollowPerceptionDiagnostics {
    public let frameID: ARFrameID
    public let timestamp: TimeInterval
    public let inferenceStatus: FollowInferenceStatus
    public let inferenceFailureReason: Detector.FailureReason?
    /// Nil means upstream inference was not evaluated or its outcome is unavailable.
    public let rawDetectorCount: Int?
    public let rawPersonCount: Int?
    public let projectionAttemptedCount: Int?
    public let projectionAcceptedCount: Int?
    public let projectionRejectedCount: Int?
    public let projectedPersonCount: Int?
    public let candidates: [FollowProjectionEvidence]
    public internal(set) var personVerification: [PersonBodyVerifier.Decision]? = nil
}

extension FollowPerceptionDiagnostics {
    var transitionSignature: String {
        let reasons = Set(candidates.compactMap { $0.rejection?.rawValue } +
            (personVerification ?? []).map(\.reason)).sorted().joined(separator: ",")
        return inferenceStatus.rawValue + "|" + (inferenceFailureReason?.rawValue ?? "none") + "|" + reasons
    }

    static func payload(_ facts: Self?) -> [String: FollowDiagnosticValue] {
        func box(_ value: CGRect?) -> FollowDiagnosticValue {
            guard let value else { return .null }
            return .object(["x": .number(Double(value.origin.x)), "y": .number(Double(value.origin.y)),
                "width": .number(Double(value.width)), "height": .number(Double(value.height))])
        }
        var result: [String: FollowDiagnosticValue] = [
            "inference_status": .string(facts?.inferenceStatus.rawValue ?? "unknown"),
            "inference_failure_reason": facts?.inferenceFailureReason.map { .string($0.rawValue) } ?? .null,
            "pipeline_availability": .string(facts == nil ? "unknown_legacy_provider" : "available"),
            "projection_evaluations": facts.map { .array($0.candidates.map { .object($0.payload) }) } ?? .null
        ]
        result["person_body_verification"] = facts?.personVerification.map { .array($0.map { decision in .object([
            "raw_person_id": .number(Double(decision.rawPersonID)), "accepted": .bool(decision.accepted),
            "reason": .string(decision.reason), "matching_bodies": .number(Double(decision.matchingBodies)),
            "input_box": box(decision.inputBox), "normalized_box": box(decision.normalizedBox),
            "box_issue": decision.boxIssue.map { .string($0) } ?? .null,
            "verified_depth_anchor": decision.depthAnchor.map { .object([
                "x": .number(Double($0.x)), "y": .number(Double($0.y))]) } ?? .null,
            "identity_claim": .string("none_body_geometry_only")]) }) } ?? .null
        result["body_verified_person_count"] = facts?.personVerification.map { .number(Double($0.filter(\.accepted).count)) } ?? .null
        let counts: [(String, Int?)] = [
            ("raw_detector_count", facts?.rawDetectorCount), ("raw_person_count", facts?.rawPersonCount),
            ("projection_attempted_count", facts?.projectionAttemptedCount),
            ("projection_accepted_count", facts?.projectionAcceptedCount),
            ("projection_rejected_count", facts?.projectionRejectedCount)]
        for (name, count) in counts {
            result[name] = count.map { .number(Double($0)) } ?? .null
            result[name + "_availability"] = .string(count == nil ? "not_evaluated_or_unknown" : "available")
        }
        return result
    }
}

extension FollowProjectionEvidence {
    var payload: [String: FollowDiagnosticValue] {
        func number<T: BinaryInteger>(_ value: T?) -> FollowDiagnosticValue { value.map { .number(Double($0)) } ?? .null }
        func decimal(_ value: Double?) -> FollowDiagnosticValue { value.map { .number($0) } ?? .null }
        func point(_ value: CGPoint?) -> FollowDiagnosticValue {
            value.map { .object(["x": .number(Double($0.x)), "y": .number(Double($0.y))]) } ?? .null
        }
        func size(_ value: CGSize?) -> FollowDiagnosticValue {
            value.map { .object(["width": .number(Double($0.width)), "height": .number(Double($0.height))]) } ?? .null
        }
        func flag(_ value: Bool?) -> FollowDiagnosticValue { value.map { .bool($0) } ?? .null }
        return [
            "raw_person_id": .number(Double(rawPersonID)), "candidate_id_scope": .string("frame_local"),
            "detector_confidence": .number(Double(detectorConfidence)),
            "bounding_box": .object(["x": .number(Double(box.origin.x)), "y": .number(Double(box.origin.y)),
                "width": .number(Double(box.size.width)), "height": .number(Double(box.size.height))]),
            "image_size": size(imageSize), "depth_size": size(depthSize),
            "depth_source": depthSource.map { .string($0.rawValue) } ?? .null,
            "depth_bytes_per_row": number(depthBytesPerRow), "depth_pixel_format": number(depthPixelFormat),
            "confidence_size": size(confidenceSize), "confidence_bytes_per_row": number(confidenceBytesPerRow),
            "confidence_pixel_format": number(confidencePixelFormat),
            "confidence_availability": .string(confidenceAvailability.rawValue),
            "clipped_left": flag(clippedLeft), "clipped_right": flag(clippedRight),
            "clipped_bottom": flag(clippedBottom), "clipped_top": flag(clippedTop),
            "feet": point(feet), "sensor_pixel": point(sensorPixel), "depth_pixel": point(depthPixel),
            "depth_anchor": point(depthAnchor), "depth_anchor_kind": depthAnchorKind.map { .string($0) } ?? .null,
            "depth_center": point(depthCenter), "requested_sample_count": .number(25),
            "valid_sample_count": number(validSampleCount), "invalid_depth_count": number(invalidDepthCount),
            "low_confidence_count": number(lowConfidenceCount),
            "sample_counts_availability": .string(validSampleCount == nil ? "not_evaluated" : "available"),
            "sample_partition": .string("valid+invalid_depth+low_or_invalid_confidence=25;invalid_depth_precedence"),
            "median_depth_m": decimal(medianDepth), "mad_m": decimal(medianAbsoluteDeviation),
            "inlier_count": number(inlierCount), "required_inlier_count": number(requiredInlierCount),
            "projected_position": position.map { .object(["x": .number($0.x), "y": .number($0.y)]) } ?? .null,
            "camera_ray": cameraRay.map { .object(["x": .number($0.x), "y": .number($0.y), "z": .number($0.z)]) } ?? .null,
            "world_point": worldPoint.map { .object(["x": .number(Double($0.x)), "y": .number(Double($0.y)), "z": .number(Double($0.z))]) } ?? .null,
            "paired_pose": pairedPose.map { .object(["x": .number($0.position.x), "y": .number($0.position.y), "yaw_rad": .number($0.yaw)]) } ?? .null,
            "range_m": decimal(groundRange), "heading_rad": decimal(headingError),
            "rejection_reason": rejection.map { .string($0.rawValue) } ?? .null
        ]
    }
}
