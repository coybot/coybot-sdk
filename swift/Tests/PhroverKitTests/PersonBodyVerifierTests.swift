import XCTest
@testable import PhroverKit

final class PersonBodyVerifierTests: XCTestCase {
    private func joint(_ x: Double, _ y: Double, confidence: Float = 0.9) -> PersonBodyVerifier.Joint {
        .init(location: CGPoint(x: x, y: y), confidence: confidence)
    }
    private func body(shoulderConfidence: Float = 0.9) -> PersonBodyVerifier.Body {
        .init(leftShoulder: joint(0.4, 0.75, confidence: shoulderConfidence), rightShoulder: joint(0.6, 0.75),
            leftHip: joint(0.43, 0.5), rightHip: joint(0.57, 0.5))
    }
    func testOneMatchingTorsoPassesButIncompleteLowConfidenceAndInvalidGeometryDoNot() {
        let box = CGRect(x: 0.2, y: 0.1, width: 0.6, height: 0.8)
        XCTAssertTrue(PersonBodyVerifier.verify(box: box, rawPersonID: 7, bodies: [body()]).accepted)
        for invalid in [body(shoulderConfidence: 0.29), body(shoulderConfidence: .nan),
                        .init(leftShoulder: nil, rightShoulder: joint(0.6, 0.75), leftHip: joint(0.43, 0.5), rightHip: joint(0.57, 0.5)),
                        .init(leftShoulder: joint(0.5, 0.5), rightShoulder: joint(0.5, 0.5), leftHip: joint(0.5, 0.5), rightHip: joint(0.5, 0.5)),
                        .init(leftShoulder: joint(.nan, 0.75), rightShoulder: joint(0.6, 0.75), leftHip: joint(0.43, 0.5), rightHip: joint(0.57, 0.5))] {
            XCTAssertFalse(PersonBodyVerifier.verify(box: box, rawPersonID: 7, bodies: [invalid]).accepted)
        }
        XCTAssertFalse(PersonBodyVerifier.verify(box: CGRect(x: 0.8, y: 0.1, width: 0.1, height: 0.1),
            rawPersonID: 7, bodies: [body()]).accepted)
        let ambiguous = PersonBodyVerifier.verify(box: box, rawPersonID: 7, bodies: [body(), body()])
        XCTAssertFalse(ambiguous.accepted)
        XCTAssertEqual(ambiguous.reason, "ambiguous_body")
    }
    func testHighConfidenceBoxWithoutIndependentBodyCannotBeVerified() {
        let result = PersonBodyVerifier.verify(box: CGRect(x: 0.2, y: 0.1, width: 0.6, height: 0.8),
            rawPersonID: 0, bodies: [])
        XCTAssertFalse(result.accepted, "Raw detector score—even 1.0—is not body evidence")
        XCTAssertEqual(result.reason, "no_matching_body")
        XCTAssertEqual(result.matchingBodies, 0)
    }
}
