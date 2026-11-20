import Foundation

/// Independent geometric evidence, not identity recognition or a guarantee
/// that a pictured body is a physical person.
public enum PersonBodyVerifier {
    public struct Joint: Sendable {
        public let location: CGPoint
        public let confidence: Float
        public init(location: CGPoint, confidence: Float) { self.location = location; self.confidence = confidence }
    }
    public struct Body: Sendable {
        public let leftShoulder: Joint?
        public let rightShoulder: Joint?
        public let leftHip: Joint?
        public let rightHip: Joint?
        public init(leftShoulder: Joint?, rightShoulder: Joint?, leftHip: Joint?, rightHip: Joint?) {
            self.leftShoulder = leftShoulder; self.rightShoulder = rightShoulder
            self.leftHip = leftHip; self.rightHip = rightHip
        }
    }
    public struct JointEvidence: Sendable {
        public let name: String
        public let reason: String
        public let joint: Joint?
    }
    public struct BodyEvidence: Sendable {
        public let joints: [JointEvidence]
        public let rejectionReasons: [String]
        public var accepted: Bool { rejectionReasons.isEmpty }
    }
    public struct Decision: Sendable {
        public let rawPersonID: Int
        public let accepted: Bool
        public let reason: String
        public let matchingBodies: Int
        public let inputBox: CGRect?
        public let normalizedBox: CGRect?
        public let boxIssue: String?
        /// Same-frame centroid of the verified shoulders and hips; not a box
        /// guess or a point reused from an earlier inference.
        public let depthAnchor: CGPoint?
        /// Nil means body inference was unavailable; zero means it ran and
        /// returned no bodies. Detailed diagnostics retain at most eight bodies.
        public internal(set) var bodyCount: Int?
        public internal(set) var bodyEvaluations: [BodyEvidence] = []
        init(rawPersonID: Int, accepted: Bool, reason: String, matchingBodies: Int,
             inputBox: CGRect? = nil, normalizedBox: CGRect? = nil, boxIssue: String? = nil,
             depthAnchor: CGPoint? = nil, bodyCount: Int? = nil) {
            self.rawPersonID = rawPersonID; self.accepted = accepted; self.reason = reason
            self.matchingBodies = matchingBodies; self.inputBox = inputBox
            self.normalizedBox = normalizedBox; self.boxIssue = boxIssue
            self.depthAnchor = depthAnchor
            self.bodyCount = bodyCount
        }
        public var verificationUnavailable: Bool {
            reason == "body_verification_failed" || reason == "body_verification_unavailable"
        }
    }
    public static func verify(box: CGRect, rawPersonID: Int, bodies: [Body]) -> Decision {
        guard [box.origin.x, box.origin.y, box.width, box.height, box.maxX, box.maxY].allSatisfy(\.isFinite),
              box.width > 0, box.height > 0 else {
            return .init(rawPersonID: rawPersonID, accepted: false, reason: "invalid_person_box", matchingBodies: 0,
                inputBox: box, boxIssue: "nonfinite_or_nonpositive_area", bodyCount: bodies.count)
        }
        // Edge detections can overrun the image slightly (recorded: 0.26–0.50%).
        // Clip at most 1% per edge; never grow the box or infer missing joints.
        // Acceptance still requires one independently verified visible torso.
        guard box.minX >= -0.01, box.minY >= -0.01, box.maxX <= 1.01, box.maxY <= 1.01 else {
            return .init(rawPersonID: rawPersonID, accepted: false, reason: "invalid_person_box", matchingBodies: 0,
                inputBox: box, boxIssue: "outside_edge_slack", bodyCount: bodies.count)
        }
        let normalized = box.intersection(CGRect(x: 0, y: 0, width: 1, height: 1))
        guard !normalized.isNull, normalized.width > 0, normalized.height > 0 else {
            return .init(rawPersonID: rawPersonID, accepted: false, reason: "invalid_person_box", matchingBodies: 0,
                inputBox: box, boxIssue: "no_visible_area", bodyCount: bodies.count)
        }
        let bounds = normalized.insetBy(dx: -0.02, dy: -0.02)
        let evaluations = bodies.map { body -> BodyEvidence in
            let namedJoints: [(String, Joint?)] = [
                ("left_shoulder", body.leftShoulder), ("right_shoulder", body.rightShoulder),
                ("left_hip", body.leftHip), ("right_hip", body.rightHip)
            ]
            let joints = namedJoints.map { name, joint -> JointEvidence in
                func reason() -> String {
                    guard let joint else { return "missing" }
                    guard joint.confidence.isFinite, joint.confidence >= 0, joint.confidence <= 1 else {
                        return "invalid_confidence"
                    }
                    guard joint.confidence >= 0.30 else { return "low_confidence" }
                    guard joint.location.x.isFinite, joint.location.y.isFinite else { return "nonfinite_position" }
                    guard joint.location.x >= 0, joint.location.x <= 1,
                          joint.location.y >= 0, joint.location.y <= 1 else { return "outside_image" }
                    guard joint.location.x >= bounds.minX, joint.location.x <= bounds.maxX,
                          joint.location.y >= bounds.minY, joint.location.y <= bounds.maxY else { return "outside_person_box" }
                    return "valid"
                }
                return .init(name: name, reason: reason(), joint: joint)
            }
            var reasons: [String] = []
            if joints.contains(where: { $0.reason != "valid" }) {
                reasons.append("invalid_joints")
            } else {
                // All four joints passed the original confidence, image and
                // box gates. Preserve the original torso geometry thresholds.
                let leftShoulder = body.leftShoulder!, rightShoulder = body.rightShoulder!
                let leftHip = body.leftHip!, rightHip = body.rightHip!
                if abs(leftShoulder.location.x - rightShoulder.location.x) < 0.02 {
                    reasons.append("shoulder_span_too_small")
                }
                if abs(leftHip.location.x - rightHip.location.x) < 0.02 {
                    reasons.append("hip_span_too_small")
                }
                if (leftShoulder.location.y + rightShoulder.location.y) / 2 -
                    (leftHip.location.y + rightHip.location.y) / 2 < 0.05 {
                    reasons.append("torso_height_too_small")
                }
            }
            return .init(joints: joints, rejectionReasons: reasons)
        }
        let matching = zip(bodies, evaluations).filter { $0.1.accepted }.map { $0.0 }
        let anchor = matching.count == 1 ? matching.first.map { body in
            // All four joints were validated by the matching predicate above.
            let joints = [body.leftShoulder!, body.rightShoulder!, body.leftHip!, body.rightHip!]
            return CGPoint(x: joints.reduce(0) { $0 + $1.location.x } / 4,
                           y: joints.reduce(0) { $0 + $1.location.y } / 4)
        } : nil
        var decision = Decision(rawPersonID: rawPersonID, accepted: matching.count == 1,
            reason: matching.count == 1 ? "body_verified" : (matching.isEmpty ? "no_matching_body" : "ambiguous_body"),
            matchingBodies: matching.count, inputBox: box, normalizedBox: normalized,
            boxIssue: normalized == box ? nil : "tiny_edge_overrun_normalized", depthAnchor: anchor)
        decision.bodyCount = bodies.count
        decision.bodyEvaluations = Array(evaluations.prefix(8))
        return decision
    }
}
