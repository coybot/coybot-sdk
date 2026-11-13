import XCTest
@testable import PhroverKit
import RoverNav

final class FollowReturnGateTests: XCTestCase {
    private let configuration = FollowMeConfiguration()

    private func batch(_ sequence: UInt64, _ time: Double, ranges: [Double] = [2],
                       verified: Bool = true, depth: Bool = true, generation: UInt64 = 1,
                       partial: Bool = false) -> FollowFrameBatch {
        let id = ARFrameID(generation: generation, sequence: sequence)
        let pose = Pose2D(position: .zero, yaw: 0)
        let box = CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6)
        return FollowFrameBatch(frameID: id, timestamp: time, pose: pose, depthAvailable: depth,
            people: ranges.map { FollowPersonObservation(frameID: id, timestamp: time, confidence: 0.9,
                boundingBox: box, position: Vec2($0, 0), pose: pose, bodyVerified: verified) },
            trackingQuality: .normal, partialPeople: partial ? [FollowBearingObservation(frameID: id,
                timestamp: time, confidence: 0.9, boundingBox: box, bearing: 0, pose: pose)] : [])
    }

    func testTargetReturnRequiresThreeStableFreshCapturesAndReadiness() {
        var gate = FollowReturnGate(generation: 1, stoppedAt: 0, reason: .targetLost)
        XCTAssertFalse(gate.observe(batch(1, 0.1), now: 0.1, ready: true, configuration: configuration))
        XCTAssertFalse(gate.observe(batch(2, 0.25), now: 0.25, ready: true, configuration: configuration))
        XCTAssertFalse(gate.observe(batch(3, 0.4), now: 0.4, ready: false, configuration: configuration))
        XCTAssertFalse(gate.observe(batch(4, 0.5), now: 0.5, ready: true, configuration: configuration))
        XCTAssertFalse(gate.observe(batch(5, 0.65), now: 0.65, ready: true, configuration: configuration))
        XCTAssertTrue(gate.observe(batch(6, 0.8), now: 0.8, ready: true, configuration: configuration))
    }

    func testDepthRecoveryRequiresObservedDepartureBeforeReturn() {
        var gate = FollowReturnGate(generation: 1, stoppedAt: 0, reason: .depthUnstable)
        for sequence in 1...5 {
            let time = Double(sequence) * 0.2
            XCTAssertFalse(gate.observe(batch(UInt64(sequence), time), now: time, ready: true, configuration: configuration))
        }
        for (sequence, time) in [(6, 1.2), (7, 1.5), (8, 1.8)] {
            XCTAssertFalse(gate.observe(batch(UInt64(sequence), time, ranges: []), now: time, ready: true, configuration: configuration))
        }
        XCTAssertFalse(gate.observe(batch(9, 2), now: 2, ready: true, configuration: configuration))
        XCTAssertFalse(gate.observe(batch(10, 2.15), now: 2.15, ready: true, configuration: configuration))
        XCTAssertTrue(gate.observe(batch(11, 2.3), now: 2.3, ready: true, configuration: configuration))
    }

    func testUnverifiedAmbiguousAndPartialDetectionsCannotAuthorizeDeparture() {
        for mode in 0..<3 {
            var gate = FollowReturnGate(generation: 1, stoppedAt: 0, reason: .depthUnstable)
            for sequence in 1...4 {
                let time = Double(sequence) * 0.2
                let frame = batch(UInt64(sequence), time, ranges: mode == 1 ? [3, 4] : (mode == 2 ? [] : [2]),
                    verified: mode != 0, partial: mode == 2)
                XCTAssertFalse(gate.observe(frame, now: time, ready: true, configuration: configuration))
            }
            for sequence in 5...8 {
                let time = Double(sequence) * 0.2
                XCTAssertFalse(gate.observe(batch(UInt64(sequence), time), now: time, ready: true, configuration: configuration))
            }
        }
    }

    func testInvalidOrDiscontinuousFramesDiscardStableEvidence() {
        let invalid = [batch(2, 0.25), batch(3, 0.4, depth: false), batch(3, 0.4, generation: 2),
            batch(3, 0.4, ranges: [1]), batch(3, 0.4, ranges: [3]), batch(3, 0.4, ranges: [2, 2.1]),
            batch(3, 0.4, verified: false), batch(3, 0.4, partial: true), batch(3, 0.4, ranges: [2.4])]
        for frame in invalid {
            var gate = FollowReturnGate(generation: 1, stoppedAt: 0, reason: .targetLost)
            XCTAssertFalse(gate.observe(batch(1, 0.1), now: 0.1, ready: true, configuration: configuration))
            XCTAssertFalse(gate.observe(batch(2, 0.25), now: 0.25, ready: true, configuration: configuration))
            XCTAssertFalse(gate.observe(frame, now: 0.4, ready: true, configuration: configuration))
            XCTAssertFalse(gate.observe(batch(4, 0.55), now: 0.55, ready: true, configuration: configuration))
        }
        for now in [0.3, 1.1] {
            var gate = FollowReturnGate(generation: 1, stoppedAt: 0, reason: .targetLost)
            XCTAssertFalse(gate.observe(batch(1, 0.1), now: 0.1, ready: true, configuration: configuration))
            XCTAssertFalse(gate.observe(batch(2, 0.25), now: 0.25, ready: true, configuration: configuration))
            XCTAssertFalse(gate.observe(batch(3, 0.4), now: now, ready: true, configuration: configuration))
        }
        var gate = FollowReturnGate(generation: 1, stoppedAt: 0, reason: .targetLost)
        XCTAssertFalse(gate.observe(batch(1, 0.1), now: 0.1, ready: true, configuration: configuration))
        XCTAssertFalse(gate.observe(batch(2, 0.25), now: 0.25, ready: true, configuration: configuration))
        XCTAssertFalse(gate.observe(batch(3, 1), now: 1, ready: true, configuration: configuration))
    }
}
