import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowTargetFacingTests: XCTestCase {
    private func drain() async { for _ in 0..<60 { await Task.yield() } }

    private func frame(_ sequence: UInt64, time: Double, range: Double = 1.65,
                       angle: Double = 0, yaw: Double = 0, visible: Bool = true, depth: Bool = true,
                       status: FollowInferenceStatus? = nil) -> FollowPerceptionEvent {
        let id = ARFrameID(generation: 1, sequence: sequence)
        let pose = Pose2D(position: .zero, yaw: yaw)
        return .frame(.init(frameID: id, timestamp: time, pose: pose, depthAvailable: depth,
            people: visible ? [.init(frameID: id, timestamp: time, confidence: 0.99,
                boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6),
                position: Vec2(range * cos(angle), range * sin(angle)), pose: pose, bodyVerified: true)] : [],
            trackingQuality: .normal, perceptionDiagnostics: status.map {
                .init(frameID: id, timestamp: time, inferenceStatus: $0, inferenceFailureReason: nil,
                    rawDetectorCount: nil, rawPersonCount: nil, projectionAttemptedCount: nil,
                    projectionAcceptedCount: nil, projectionRejectedCount: nil,
                    projectedPersonCount: visible ? 1 : 0, candidates: [])
            }))
    }

    private func setup() -> (FollowMeCoordinator, FollowPerceptionFake, FollowMotionFake, ManualFollowClock) {
        let perception = FollowPerceptionFake(), motion = FollowMotionFake(), clock = ManualFollowClock()
        motion.useSourceClock(clock)
        motion.suspendAlignment = true
        motion.suspendRotation = true
        motion.suspendNavigation = true
        var configuration = FollowMeConfiguration.continuous
        configuration.stationaryPauseSeconds = 0
        configuration.departureRangeIncrease = 0
        return (FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: configuration, eventSink: { _, _ in }), perception, motion, clock)
    }

    func testHoldingDistanceTurnsTowardEitherSideAfterFreshStoppedObservation() async {
        for direction in [-1.0, 1.0] {
            let (coordinator, perception, motion, clock) = setup()
            _ = await coordinator.start()
            perception.send(frame(1, time: 0))
            await drain()
            XCTAssertEqual(coordinator.state, .holdingDistance)
            let angle = direction * 26 * Double.pi / 180
            clock.advance(to: 0.1)
            perception.send(frame(2, time: 0.1, angle: angle))
            await drain()
            XCTAssertEqual(coordinator.state, .aligning, "Distance holding must still track bearing")
            XCTAssertTrue(motion.alignments.isEmpty, "No turning on a pre-stop frame")
            clock.advance(to: 0.5)
            perception.send(frame(3, time: 0.5, angle: angle))
            await drain()
            XCTAssertEqual(motion.alignments.count, 1)
            if let requested = motion.alignments.first { XCTAssertEqual(requested, angle, accuracy: 0.001) }
            XCTAssertTrue(motion.goals.isEmpty)
            _ = await coordinator.stop()
            motion.releaseAlignment()
            clock.advance(to: 100)
            await drain()
        }
    }

    func testLostTargetWhileHoldingDoesNotRestartTranslationFromPrediction() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, time: 0))
        await drain()
        clock.advance(to: 0.1)
        perception.send(frame(2, time: 0.1, visible: false))
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring)
        XCTAssertTrue(motion.goals.isEmpty)
        _ = await coordinator.stop()
        motion.releaseRotation()
        clock.advance(to: 100)
        await drain()
    }

    func testFollowingStopsTranslationBeforeLargeBearingCorrection() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, range: 2))
        await drain()
        XCTAssertEqual(motion.goals.count, 1)
        let stops = motion.stops
        let angle = 20 * Double.pi / 180
        clock.advance(to: 0.1)
        perception.send(frame(2, time: 0.1, range: 2, angle: angle))
        await drain()
        XCTAssertEqual(coordinator.state, .aligning)
        XCTAssertGreaterThan(motion.stops, stops)
        motion.releaseNavigation()
        clock.advance(to: 0.5)
        perception.send(frame(3, time: 0.5, range: 2, angle: angle))
        await drain()
        XCTAssertEqual(motion.alignments.count, 1)
        XCTAssertEqual(motion.goals.count, 1, "Large heading errors use a turn, not a tapered forward curve")
        _ = await coordinator.stop()
        motion.releaseAlignment()
        clock.advance(to: 100)
        await drain()
    }

    func testFacingCompletionResumesHoldingAndFollowingWithoutRepeatingReadiness() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, time: 0))
        await drain()
        let angle = 26 * Double.pi / 180
        clock.advance(to: 0.1)
        perception.send(frame(2, time: 0.1, angle: angle))
        await drain()
        clock.advance(to: 0.5)
        perception.send(frame(3, time: 0.5, angle: angle))
        await drain()
        motion.releaseAlignment()
        await drain()
        XCTAssertEqual(coordinator.state, .aligning, "Turn completion still needs a fresh post-stop match")
        clock.advance(to: 0.9)
        perception.send(frame(4, time: 0.9, angle: angle, yaw: angle))
        await drain()
        XCTAssertEqual(coordinator.state, .holdingDistance)
        XCTAssertEqual(motion.readySignals, 0)
        XCTAssertTrue(motion.goals.isEmpty)
        clock.advance(to: 1.1)
        perception.send(frame(5, time: 1.1, range: 2.1, angle: angle, yaw: angle))
        await drain()
        XCTAssertEqual(coordinator.state, .following)
        XCTAssertEqual(motion.goals.count, 1)
        XCTAssertEqual(motion.readySignals, 0)
        _ = await coordinator.stop()
        motion.releaseNavigation()
        clock.advance(to: 100)
        await drain()
    }

    func testFacingStopFailureAndExplicitStopPreventLateTurning() async {
        for failedStop in [true, false] {
            let (coordinator, perception, motion, clock) = setup()
            _ = await coordinator.start()
            perception.send(frame(1, time: 0))
            await drain()
            motion.stopError = failedStop
            clock.advance(to: 0.1)
            perception.send(frame(2, time: 0.1, angle: .pi / 7))
            await drain()
            if !failedStop { _ = await coordinator.stop() }
            clock.advance(to: 0.5)
            perception.send(frame(3, time: 0.5, angle: .pi / 7))
            await drain()
            XCTAssertTrue(motion.alignments.isEmpty)
            XCTAssertTrue(motion.goals.isEmpty)
            XCTAssertEqual(coordinator.state, failedStop ? .failed("Rover stop could not be confirmed.") : .stopped)
            clock.advance(to: 100)
            await drain()
        }
    }

    func testFacingDeadlineCannotBeExtendedByFreshTargetFrames() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, time: 0))
        await drain()
        clock.advance(to: 0.1)
        perception.send(frame(2, time: 0.1, angle: .pi / 7))
        await drain()
        clock.advance(to: 0.5)
        perception.send(frame(3, time: 0.5, angle: .pi / 7))
        await drain()
        clock.advance(to: 10.1, wakeSleepers: false)
        perception.send(frame(4, time: 10.1, angle: .pi / 7))
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Target-facing turn timed out. Stop and restart following."))
        motion.releaseAlignment()
        clock.advance(to: 100)
        await drain()
        XCTAssertTrue(motion.goals.isEmpty)
    }

    func testChangedTargetDirectionIsRecomputedAfterTurnRatherThanReusingOldAngle() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, time: 0))
        await drain()
        let first = 26 * Double.pi / 180, changed = 10 * Double.pi / 180
        clock.advance(to: 0.1)
        perception.send(frame(2, time: 0.1, angle: first))
        await drain()
        clock.advance(to: 0.5)
        perception.send(frame(3, time: 0.5, angle: first))
        await drain()
        motion.releaseAlignment()
        await drain()
        clock.advance(to: 0.9)
        perception.send(frame(4, time: 0.9, angle: changed, yaw: first))
        await drain()
        clock.advance(to: 1.3)
        perception.send(frame(5, time: 1.3, angle: changed, yaw: first))
        await drain()
        XCTAssertEqual(motion.alignments.count, 2)
        if motion.alignments.count == 2 {
            XCTAssertEqual(motion.alignments[1], changed - first, accuracy: 0.001)
        }
        XCTAssertTrue(motion.goals.isEmpty)
        _ = await coordinator.stop()
        motion.releaseAlignment()
        clock.advance(to: 100)
        await drain()
        XCTAssertEqual(coordinator.state, .stopped)
    }

    func testLossDuringFacingStopsAndReacquiresWithoutLateTranslation() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, time: 0))
        await drain()
        clock.advance(to: 0.1)
        perception.send(frame(2, time: 0.1, angle: .pi / 7))
        await drain()
        clock.advance(to: 0.5)
        perception.send(frame(3, time: 0.5, angle: .pi / 7))
        await drain()
        clock.advance(to: 0.6)
        perception.send(frame(4, time: 0.6, visible: false))
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring)
        motion.releaseAlignment()
        await drain()
        XCTAssertTrue(motion.goals.isEmpty)
        _ = await coordinator.stop()
        motion.releaseRotation()
        clock.advance(to: 100)
        await drain()
    }

    func testDelayedFramesResumeFacingWithoutRepeatedStoppedFenceAdvancement() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, time: 0))
        await drain()
        clock.advance(to: 0.1)
        perception.send(frame(2, time: 0.1, angle: .pi / 7))
        await drain()
        clock.advance(to: 0.5)
        perception.send(frame(3, time: 0.5, angle: .pi / 7))
        await drain()
        XCTAssertEqual(motion.alignments.count, 1)
        clock.advance(to: 1.01)
        await drain()
        motion.releaseAlignment()
        await drain()
        let stopsAfterStaleTurn = motion.stops
        // The source fence is now at 1.01. Capture 0.95 is fresh on delivery but
        // predates acknowledgement. Its later expiry must not move that fence
        // again while the rover is already confirmed stationary.
        clock.advance(to: 1.30)
        perception.send(frame(4, time: 0.95, angle: .pi / 7))
        await drain()
        clock.advance(to: 1.46)
        await drain()
        XCTAssertEqual(motion.stops, stopsAfterStaleTurn, "Age-only expiry must not re-stop an already stopped facing wait")
        clock.advance(to: 1.50)
        perception.send(frame(5, time: 1.10, angle: .pi / 7))
        await drain()
        XCTAssertEqual(motion.alignments.count, 2, "A fresh frame beyond the original stop fence must resume alignment")
        XCTAssertEqual(motion.stops, stopsAfterStaleTurn, "Resumption must not advance the acknowledged fence again")
        clock.advance(to: 1.61)
        await drain()
        XCTAssertGreaterThan(motion.stops, stopsAfterStaleTurn, "Once turning resumes, stale perception must request a new stop")
        _ = await coordinator.stop()
        motion.releaseAlignment()
        clock.advance(to: 100)
        await drain()
    }

    func testStationaryFacingAgeWaitRetainsOriginalPerceptionDeadline() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, time: 0))
        await drain()
        clock.advance(to: 0.1)
        perception.send(frame(2, time: 0.1, angle: .pi / 7))
        await drain()
        let stops = motion.stops
        clock.advance(to: 0.61)
        await drain()
        XCTAssertEqual(motion.stops, stops)
        XCTAssertTrue(motion.alignments.isEmpty)
        clock.advance(to: 1.1)
        perception.send(frame(3, time: 0.2, angle: .pi / 7))
        await drain()
        XCTAssertEqual(motion.stops, stops)
        clock.advance(to: 2.62)
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Camera frame stale or timestamp invalid. Check camera delivery and inference latency."))
        XCTAssertTrue(motion.alignments.isEmpty)
        clock.advance(to: 100)
        await drain()
    }

    func testStructuralFaultEscalatesStationaryFacingWaitAndFailedStopRemainsLatched() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, time: 0))
        await drain()
        clock.advance(to: 0.1)
        perception.send(frame(2, time: 0.1, angle: .pi / 7))
        await drain()
        let stops = motion.stops
        clock.advance(to: 0.61)
        await drain()
        XCTAssertEqual(motion.stops, stops)
        motion.stopError = true
        clock.advance(to: 0.7)
        perception.send(frame(3, time: 0.7, angle: .pi / 7, depth: false))
        await drain()
        XCTAssertGreaterThan(motion.stops, stops)
        XCTAssertEqual(coordinator.state, .failed("Rover stop could not be confirmed."))
        clock.advance(to: 1.1)
        perception.send(frame(4, time: 1.1, angle: .pi / 7))
        await drain()
        XCTAssertTrue(motion.alignments.isEmpty)
        clock.advance(to: 100)
        await drain()
    }

    func testTrackedFramesCannotAcquireOrReacquireAfterModeChange() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, status: .tracked))
        await drain()
        XCTAssertEqual(coordinator.state, .searching)
        XCTAssertFalse(perception.visualTrackingEnabled)
        clock.advance(to: 0.1)
        perception.send(frame(2, time: 0.1, status: .executed))
        await drain()
        XCTAssertEqual(coordinator.state, .holdingDistance)
        XCTAssertTrue(perception.visualTrackingEnabled)
        clock.advance(to: 0.2)
        perception.send(frame(3, time: 0.2, visible: false, status: .executed))
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring)
        XCTAssertFalse(perception.visualTrackingEnabled)
        clock.advance(to: 0.6)
        perception.send(frame(4, time: 0.6, status: .tracked))
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring)
        XCTAssertTrue(motion.goals.isEmpty)
        _ = await coordinator.stop()
        motion.releaseRotation()
        clock.advance(to: 100)
        await drain()
    }
}
