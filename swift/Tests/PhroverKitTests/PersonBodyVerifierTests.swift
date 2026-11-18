import XCTest
@testable import PhroverKit

final class PersonBodyVerifierTests: XCTestCase {
    func testRecordedEdgeOverrunsRequireIndependentVisibleTorso() {
        let recorded = [
            CGRect(x: 0.6303116232, y: 0.056201756, width: 0.3743521869, height: 0.7421096563),
            CGRect(x: 0.6282535791, y: 0.054117918, width: 0.3767776489, height: 0.7481240034),
            CGRect(x: 0.5820810050, y: 0.060308784, width: 0.4204775393, height: 0.7309002280)
        ]
        for box in recorded {
            for mirrored in [false, true] {
                let input = mirrored ? CGRect(x: 1 - box.maxX, y: box.minY, width: box.width, height: box.height) : box
                func x(_ value: Double) -> Double { mirrored ? 1 - value : value }
                let torso = PersonBodyVerifier.Body(
                    leftShoulder: joint(x(0.75), 0.70), rightShoulder: joint(x(0.95), 0.70),
                    leftHip: joint(x(0.78), 0.40), rightHip: joint(x(0.92), 0.40))
                let result = PersonBodyVerifier.verify(box: input, rawPersonID: 0, bodies: [torso])
                XCTAssertTrue(result.accepted)
                XCTAssertEqual(result.normalizedBox, input.intersection(CGRect(x: 0, y: 0, width: 1, height: 1)))
                XCTAssertNotNil(result.depthAnchor)
                XCTAssertFalse(PersonBodyVerifier.verify(box: input, rawPersonID: 0, bodies: []).accepted)
                XCTAssertFalse(PersonBodyVerifier.verify(box: input, rawPersonID: 0, bodies: [torso, torso]).accepted)
            }
        }
        for overrun in [0.0101, 0.02, 0.5] {
            XCTAssertFalse(PersonBodyVerifier.verify(box: CGRect(x: -overrun, y: 0, width: 1 + overrun, height: 1),
                rawPersonID: 0, bodies: [body()]).accepted)
        }
    }
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
