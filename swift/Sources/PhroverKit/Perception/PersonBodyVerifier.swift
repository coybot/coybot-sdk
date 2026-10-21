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
        init(rawPersonID: Int, accepted: Bool, reason: String, matchingBodies: Int,
             inputBox: CGRect? = nil, normalizedBox: CGRect? = nil, boxIssue: String? = nil, depthAnchor: CGPoint? = nil) {
            self.rawPersonID = rawPersonID; self.accepted = accepted; self.reason = reason
            self.matchingBodies = matchingBodies; self.inputBox = inputBox
            self.normalizedBox = normalizedBox; self.boxIssue = boxIssue
            self.depthAnchor = depthAnchor
        }
        public var verificationUnavailable: Bool {
            reason == "body_verification_failed" || reason == "body_verification_unavailable"
        }
    }
    public static func verify(box: CGRect, rawPersonID: Int, bodies: [Body]) -> Decision {
        guard [box.origin.x, box.origin.y, box.width, box.height, box.maxX, box.maxY].allSatisfy(\.isFinite),
              box.width > 0, box.height > 0 else {
            return .init(rawPersonID: rawPersonID, accepted: false, reason: "invalid_person_box", matchingBodies: 0,
                inputBox: box, boxIssue: "nonfinite_or_nonpositive_area")
        }
        // Detector rounding can put an otherwise visible torso fractionally
        // outside the unit image. Bound the repair to 0.2% per edge; never grow it.
        guard box.minX >= -0.002, box.minY >= -0.002, box.maxX <= 1.002, box.maxY <= 1.002 else {
            return .init(rawPersonID: rawPersonID, accepted: false, reason: "invalid_person_box", matchingBodies: 0,
                inputBox: box, boxIssue: "outside_edge_slack")
        }
        let normalized = box.intersection(CGRect(x: 0, y: 0, width: 1, height: 1))
        guard !normalized.isNull, normalized.width > 0, normalized.height > 0 else {
            return .init(rawPersonID: rawPersonID, accepted: false, reason: "invalid_person_box", matchingBodies: 0,
                inputBox: box, boxIssue: "no_visible_area")
        }
        let matching = bodies.filter { body in
            guard let leftShoulder = body.leftShoulder, let rightShoulder = body.rightShoulder,
                  let leftHip = body.leftHip, let rightHip = body.rightHip else { return false }
            let joints = [leftShoulder, rightShoulder, leftHip, rightHip]
            let bounds = normalized.insetBy(dx: -0.02, dy: -0.02)
            guard joints.allSatisfy({ joint in
                joint.confidence.isFinite && joint.confidence >= 0.30 && joint.confidence <= 1 &&
                    joint.location.x.isFinite && joint.location.y.isFinite &&
                    joint.location.x >= 0 && joint.location.x <= 1 && joint.location.y >= 0 && joint.location.y <= 1 &&
                    joint.location.x >= bounds.minX && joint.location.x <= bounds.maxX &&
                    joint.location.y >= bounds.minY && joint.location.y <= bounds.maxY
            }) else { return false }
            return abs(leftShoulder.location.x - rightShoulder.location.x) >= 0.02 &&
                abs(leftHip.location.x - rightHip.location.x) >= 0.02 &&
                (leftShoulder.location.y + rightShoulder.location.y) / 2 -
                    (leftHip.location.y + rightHip.location.y) / 2 >= 0.05
        }
        let anchor = matching.count == 1 ? matching.first.map { body in
            // All four joints were validated by the matching predicate above.
            let joints = [body.leftShoulder!, body.rightShoulder!, body.leftHip!, body.rightHip!]
            return CGPoint(x: joints.reduce(0) { $0 + $1.location.x } / 4,
                           y: joints.reduce(0) { $0 + $1.location.y } / 4)
        } : nil
        return .init(rawPersonID: rawPersonID, accepted: matching.count == 1,
            reason: matching.count == 1 ? "body_verified" : (matching.isEmpty ? "no_matching_body" : "ambiguous_body"),
            matchingBodies: matching.count, inputBox: box, normalizedBox: normalized,
            boxIssue: normalized == box ? nil : "tiny_edge_overrun_normalized", depthAnchor: anchor)
    }
}
