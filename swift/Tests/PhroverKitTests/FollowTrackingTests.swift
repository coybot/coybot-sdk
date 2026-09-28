import XCTest
import RoverNav
@testable import PhroverKit

/// Association and safety-gate behaviour for follow mode. No ARKit, no CoreML, no sim:
/// `TargetTracker` takes already-grounded world points precisely so the logic that decides
/// *which* person is still being followed can be tested this cheaply.
@MainActor
final class FollowTrackingTests: XCTestCase {
    private let t0 = Date(timeIntervalSince1970: 5000)

    private func observation(_ point: Vec2,
                             label: String = "person",
                             confidence: Float = 0.9) -> TargetObservation {
        TargetObservation(label: label, confidence: confidence, worldPoint: point)
    }

    // MARK: - Acquisition

    func testAcquiresHighestConfidenceMatch() {
        let tracker = TargetTracker()
        tracker.lock(to: TargetSpec(query: "follow the person"))

        tracker.update(observations: [
            observation(Vec2(3, 0), confidence: 0.6),
            observation(Vec2(5, 1), confidence: 0.95)
        ], now: t0)

        XCTAssertEqual(tracker.state, .tracking)
        XCTAssertEqual(tracker.track?.position, Vec2(5, 1))
    }

    /// The operator named an attribute the detector can report. The plain `person` still
    /// matches "the guy" — it must not win just by being the louder detection.
    func testAcquiresTheCandidateTheQueryDescribesMostSpecifically() {
        let tracker = TargetTracker()
        tracker.lock(to: TargetSpec(query: "the guy with the hat"))

        tracker.update(observations: [
            observation(Vec2(3, 0), label: "person", confidence: 0.95),
            observation(Vec2(5, 1), label: "person_hat", confidence: 0.8)
        ], now: t0)

        XCTAssertEqual(tracker.track?.position, Vec2(5, 1))
    }

    /// Specificity picks who; position keeps them. A frame where only the plain label is
    /// reported at the tracked spot (the hat not made out) must not break the track.
    func testStaysOnTheTrackWhenTheAttributeDropsOutForAFrame() {
        let tracker = TargetTracker()
        tracker.lock(to: TargetSpec(query: "the person with the hat"))
        tracker.update(observations: [observation(Vec2(5, 1), label: "person_hat")], now: t0)
        let id = tracker.track?.id

        tracker.update(observations: [observation(Vec2(5.1, 1.2), label: "person")],
                       now: t0.addingTimeInterval(0.1))

        XCTAssertEqual(tracker.track?.id, id)
        XCTAssertEqual(tracker.state, .tracking)
        XCTAssertEqual(tracker.track?.position.x ?? 0, 5.1, accuracy: 0.2)
    }

    func testSeedBiasesTheInitialPick() {
        let tracker = TargetTracker()
        tracker.lock(to: TargetSpec(query: "person"), seed: Vec2(3, 0))

        tracker.update(observations: [
            observation(Vec2(3, 0), confidence: 0.6),
            observation(Vec2(9, 1), confidence: 0.95)
        ], now: t0)

        XCTAssertEqual(tracker.track?.position, Vec2(3, 0),
                       "a seeded lock must take the indicated candidate, not the loudest one")
    }

    func testIgnoresNonMatchingLabelsAndLowConfidence() {
        let tracker = TargetTracker()
        tracker.lock(to: TargetSpec(query: "person"))

        tracker.update(observations: [
            observation(Vec2(2, 0), label: "chair"),
            observation(Vec2(3, 0), confidence: 0.2)
        ], now: t0)

        XCTAssertEqual(tracker.state, .acquiring)
        XCTAssertNil(tracker.track)
    }

    // MARK: - Identity

    func testKeepsTheSameTrackIdAcrossAnOcclusion() {
        let tracker = TargetTracker()
        tracker.lock(to: TargetSpec(query: "person"))
        tracker.update(observations: [observation(Vec2(3, 0))], now: t0)
        let id = tracker.track?.id

        // Two ticks with nothing visible, then it comes back roughly where predicted.
        tracker.update(observations: [], now: t0.addingTimeInterval(0.1))
        tracker.update(observations: [], now: t0.addingTimeInterval(0.2))
        XCTAssertEqual(tracker.state, .coasting)
        tracker.update(observations: [observation(Vec2(3.1, 0))], now: t0.addingTimeInterval(0.3))

        XCTAssertEqual(tracker.state, .tracking)
        XCTAssertEqual(tracker.track?.id, id)
        XCTAssertEqual(tracker.unverifiedReacquisitions, 0)
    }

    /// The failure mode vision-only following exists to avoid: a second person walks
    /// through frame and the lock silently transfers to them.
    func testDoesNotSwapToADecoyWhileTheTrackIsFresh() {
        let tracker = TargetTracker()
        tracker.lock(to: TargetSpec(query: "person"))
        tracker.update(observations: [observation(Vec2(3, 0))], now: t0)
        let id = tracker.track?.id

        // The real target is momentarily undetected; a decoy is plainly visible 4 m away.
        for i in 1...5 {
            tracker.update(observations: [observation(Vec2(3, 4))],
                           now: t0.addingTimeInterval(Double(i) * 0.1))
        }

        XCTAssertEqual(tracker.track?.id, id)
        XCTAssertEqual(tracker.track?.position.y ?? .nan, 0, accuracy: 0.001,
                       "lock jumped to the decoy instead of coasting on the prediction")
        XCTAssertEqual(tracker.unverifiedReacquisitions, 0)
    }

    func testReattachesOutsideTheGateOnlyAfterCoastingAndCountsIt() {
        let tracker = TargetTracker()
        tracker.lock(to: TargetSpec(query: "person"))
        tracker.update(observations: [observation(Vec2(3, 0))], now: t0)

        // Past `reacquireAfter` (1.5 s) with only an ungated candidate available.
        tracker.update(observations: [observation(Vec2(3, 4))], now: t0.addingTimeInterval(2.0))

        XCTAssertEqual(tracker.state, .tracking)
        XCTAssertEqual(tracker.unverifiedReacquisitions, 1,
                       "an identity the gate could not vouch for must be reported, not hidden")
    }

    func testDeclaredLostAfterTheCoastWindow() {
        let tracker = TargetTracker()
        tracker.lock(to: TargetSpec(query: "person"))
        tracker.update(observations: [observation(Vec2(3, 0))], now: t0)

        tracker.update(observations: [], now: t0.addingTimeInterval(5))
        XCTAssertEqual(tracker.state, .coasting)
        tracker.update(observations: [], now: t0.addingTimeInterval(11))
        XCTAssertEqual(tracker.state, .lost)
        XCTAssertNotNil(tracker.track, "the last known position is kept so a search can aim at it")
    }

    // MARK: - Velocity

    func testEstimatesVelocityFromSuccessiveSightings() {
        let tracker = TargetTracker(params: .init(positionAlpha: 1.0, velocityAlpha: 1.0))
        tracker.lock(to: TargetSpec(query: "person"))
        tracker.update(observations: [observation(Vec2(3, 0))], now: t0)
        tracker.update(observations: [observation(Vec2(3.5, 0))], now: t0.addingTimeInterval(1))

        XCTAssertEqual(tracker.track?.velocity.x ?? 0, 0.5, accuracy: 0.01)
    }

    func testClampsImplausibleVelocity() {
        let tracker = TargetTracker(params: .init(positionAlpha: 1.0, velocityAlpha: 1.0, maxSpeed: 3.0))
        tracker.lock(to: TargetSpec(query: "person"))
        tracker.update(observations: [observation(Vec2(0, 0))], now: t0)
        // A 6 m jump in 0.1 s is an association or unprojection error, not a sprint.
        tracker.update(observations: [observation(Vec2(6, 0))], now: t0.addingTimeInterval(0.1))

        XCTAssertLessThanOrEqual(tracker.track?.velocity.length ?? 0, 3.0 + 1e-6)
    }

    // MARK: - Safety gate

    func testClearanceAttributedToTheTargetRelaxesTheStop() {
        XCTAssertTrue(FollowController.clearanceIsTheTarget(forwardClearance: 2.5,
                                                            distanceToTarget: 2.6))
    }

    func testSomethingNearerThanTheTargetStillStops() {
        // Target 2.5 m out, but the cone reads 0.4 m — a cart has swung in.
        XCTAssertFalse(FollowController.clearanceIsTheTarget(forwardClearance: 0.4,
                                                              distanceToTarget: 2.5))
    }

    func testUnknownClearanceIsNeverAttributedToTheTarget() {
        XCTAssertFalse(FollowController.clearanceIsTheTarget(forwardClearance: .infinity,
                                                              distanceToTarget: .infinity))
    }

    // MARK: - Motion mapping

    func testHoldCommandsNoMotion() {
        let command = FollowController.unicycleCommand(for: .hold,
                                                        pose: Pose2D(position: .zero, yaw: 0),
                                                        pursuit: PursuitController())
        XCTAssertEqual(command.v, 0)
        XCTAssertEqual(command.w, 0)
    }

    func testTurnInPlaceHasNoForwardComponent() {
        let command = FollowController.unicycleCommand(for: .turnInPlace(yawError: .pi / 2),
                                                        pose: Pose2D(position: .zero, yaw: 0),
                                                        pursuit: PursuitController())
        XCTAssertEqual(command.v, 0, accuracy: 1e-9)
        XCTAssertNotEqual(command.w, 0)
    }

    func testAdvanceRespectsThePolicySpeedCap() {
        let pursuit = PursuitController(params: .init(maxLinear: 0.35, wheelBase: RoverConfig.wheelBase))
        let command = FollowController.unicycleCommand(
            for: .advance(goal: Vec2(4, 0), speed: 0.1),
            pose: Pose2D(position: .zero, yaw: 0),
            pursuit: pursuit)

        XCTAssertGreaterThan(command.v, 0)
        XCTAssertLessThanOrEqual(command.v, 0.1 + 1e-9)
    }
}
