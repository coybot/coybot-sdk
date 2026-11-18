import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowTargetFacingTests: XCTestCase {
    func testContinuousHoldingFacingKeepsOneOwnerAndTransitionsToDriving() async {
        let perception = FollowPerceptionFake(), motion = LiveGoalFollowMotionFake(), clock = ManualFollowClock()
        motion.legacy.useSourceClock(clock)
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        config.planningMode = .reactiveDepth
        config.continuousHoldingFacing = true
        config.movingTargetFacingAngle = .pi / 2
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, range: 1.4)); await drain()
        XCTAssertEqual(coordinator.state, .holdingDistance)
        XCTAssertTrue(motion.legacy.goals.isEmpty)
        for (index, angle) in [0.3, 0.45, 0.6].enumerated() {
            let time = Double(index + 1) * 0.3
            clock.advance(to: time)
            perception.send(frame(UInt64(index + 2), time: time, range: 1.4, angle: angle))
            await drain()
            XCTAssertEqual(coordinator.state, .holdingDistance)
            XCTAssertTrue(motion.legacy.alignments.isEmpty)
            XCTAssertEqual(motion.legacy.goals.count, 1)
            XCTAssertEqual(motion.targetFeed?.sample?.holdsTranslation, true)
        }
        let stops = motion.legacy.stops
        clock.advance(to: 1.2)
        perception.send(frame(5, time: 1.2, range: 2, angle: 0.6)); await drain()
        XCTAssertEqual(coordinator.state, .following)
        XCTAssertEqual(motion.targetFeed?.sample?.holdsTranslation, false)
        XCTAssertEqual(motion.legacy.goals.count, 1)
        XCTAssertEqual(motion.legacy.stops, stops)
        _ = await coordinator.stop(); motion.legacy.releaseNavigation()
        clock.advance(to: 100); await drain()
    }

    func testWideMovingTargetBriefMissKeepsDriveButDoesNotExtendMeasuredFreshness() async {
        let perception = FollowPerceptionFake(), motion = LiveGoalFollowMotionFake(), clock = ManualFollowClock()
        motion.legacy.useSourceClock(clock)
        motion.legacy.suspendRotation = true
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        config.movingTargetFacingAngle = .pi / 2
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        for (index, angle) in [0.0, 10, 20, 30, 40].enumerated() {
            let time = Double(index) * 0.2
            clock.advance(to: time)
            perception.send(frame(UInt64(index + 1), time: time, range: 2, angle: angle * .pi / 180))
            await drain()
        }
        XCTAssertEqual(coordinator.state, .following)
        XCTAssertEqual(coordinator.trackedPersonFrameID, ARFrameID(generation: 1, sequence: 5))
        let stops = motion.legacy.stops
        for step in 1...6 {
            let time = 0.8 + Double(step) * 0.1
            clock.advance(to: time)
            perception.send(frame(UInt64(step + 5), time: time, visible: false))
            await drain()
            if step < 4 {
                XCTAssertEqual(coordinator.state, .following, "A brief lateral-target dropout must not interrupt its fresh measured arc")
                XCTAssertEqual(motion.legacy.stops, stops)
            }
        }
        XCTAssertEqual(coordinator.state, .reacquiring, "Predicted observations cannot renew the measured target's freshness")
        XCTAssertGreaterThan(motion.legacy.stops, stops)
        _ = await coordinator.stop()
        motion.legacy.releaseNavigation(); motion.legacy.releaseRotation()
        clock.advance(to: 100); await drain()
    }

    func testInitialVerificationGraceStaysStoppedAndDoesNotRenewItsDeadline() async {
        let perception = FollowPerceptionFake(), motion = FollowMotionFake(), clock = ManualFollowClock()
        motion.useSourceClock(clock)
        motion.suspendAlignment = true
        motion.suspendRotation = true
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0
        config.followImmediatelyAfterAlignment = true
        config.acquisitionVerificationGraceSeconds = 1
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, range: 2, angle: 0.4)); await drain()
        clock.advance(to: 0.35)
        perception.send(frame(2, time: 0.35, range: 2, angle: 0.4)); await drain()
        clock.advance(to: 0.4)
        perception.send(frame(3, time: 0.4, visible: false, verificationMiss: true)); await drain()
        let stops = motion.stops
        for (sequence, time) in [(4, 0.8), (5, 1.0), (6, 1.3)] {
            clock.advance(to: time)
            perception.send(frame(UInt64(sequence), time: time, visible: false, verificationMiss: true))
            await drain()
            XCTAssertEqual(coordinator.state, .aligning, "A short body-verification dropout must retain the stopped acquisition")
            XCTAssertEqual(motion.stops, stops)
            XCTAssertTrue(motion.goals.isEmpty)
        }
        clock.advance(to: 1.41); await drain()
        XCTAssertEqual(coordinator.state, .reacquiring, "Repeated misses cannot extend the original one-second deadline")
        _ = await coordinator.stop()
        motion.releaseAlignment(); motion.releaseRotation()
        clock.advance(to: 100); await drain()
    }

    func testImmediateStartupReusesCorrelatedNativeTurnStop() async {
        let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
        motion.legacy.useSourceClock(clock)
        motion.legacy.suspendNavigation = true
        var stopsAtTurn: Int?
        motion.performHandler = { request, context in
            if context.purpose == .followAlignment {
                stopsAtTurn = motion.legacy.stops
                let operation = FollowMotionOperationContext(request: context, controllerOperationID: 42,
                    purpose: .followAlignment, profile: nil)
                return .init(result: .arrived, context: operation, failure: nil, stopOutcome: .confirmed,
                    turnStopFence: .init(identity: UUID(), operationGeneration: 1, context: operation,
                        acknowledgementUptime: clock.now, sourceGeneration: 1,
                        highestSequence: 2, highestSourceTimestamp: clock.now))
            }
            return await motion.legacy.performContextual(request, context: context)
        }
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0
        config.followImmediatelyAfterAlignment = true
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, range: 2, angle: 0.3))
        await drain()
        clock.advance(to: 0.4)
        perception.send(frame(2, time: 0.4, range: 2, angle: 0.3))
        await drain()
        XCTAssertNotNil(stopsAtTurn)
        XCTAssertTrue(motion.legacy.goals.isEmpty)
        clock.advance(to: 0.8)
        perception.send(frame(3, time: 0.8, range: 2, angle: 0.3, yaw: 0.3))
        await drain()
        XCTAssertEqual(coordinator.state, .following)
        XCTAssertEqual(motion.legacy.stops, stopsAtTurn)
        _ = await coordinator.stop()
        motion.legacy.releaseNavigation()
        motion.performHandler = nil
        clock.advance(to: 100)
        await drain()
    }

    func testImmediateStartupLossStillRequiresAlignmentAfterReacquisition() async {
        let perception = FollowPerceptionFake(), motion = FollowMotionFake(), clock = ManualFollowClock()
        motion.useSourceClock(clock)
        motion.suspendAlignment = true
        motion.suspendRotation = true
        motion.suspendNavigation = true
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0
        config.followImmediatelyAfterAlignment = true
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, range: 2, angle: 0.4))
        await drain()
        clock.advance(to: 0.1)
        perception.send(frame(2, time: 0.1, range: 2, visible: false))
        await drain()
        motion.releaseAlignment()
        for step in 1...4 {
            let time = 0.1 + Double(step) * 0.4
            clock.advance(to: time)
            perception.send(frame(UInt64(step + 2), time: time, range: 2, angle: 0.4))
            await drain()
        }
        XCTAssertTrue(motion.goals.isEmpty, "A fresh target match cannot bypass unfinished initial alignment")
        XCTAssertEqual(coordinator.state, .aligning)
        _ = await coordinator.stop()
        motion.releaseAlignment()
        motion.releaseRotation()
        motion.releaseNavigation()
        clock.advance(to: 100)
        await drain()
    }

    func testImmediateStartupAlignsThenFollowsWithoutReadinessOrDepartureGesture() async {
        await assertImmediateReadiness(range: 2)
    }

    func testImmediateStartupAnnouncesReadinessWhileHoldingDistance() async {
        await assertImmediateReadiness(range: 1.5)
    }

    private func assertImmediateReadiness(range: Double) async {
        let perception = FollowPerceptionFake(), motion = FollowMotionFake(), clock = ManualFollowClock()
        motion.useSourceClock(clock)
        motion.suspendNavigation = true
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0
        config.followImmediatelyAfterAlignment = true
        var announcements: [String] = []
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion,
            stationaryReadyAnnouncement: { announcements.append($0) }, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        XCTAssertEqual(coordinator.state, .searching)
        perception.send(frame(1, time: 0, range: range))
        await drain()
        XCTAssertEqual(coordinator.state, .aligning)
        XCTAssertTrue(motion.goals.isEmpty, "Initial detection cannot bypass stopped alignment")
        XCTAssertTrue(announcements.isEmpty)
        for step in 1...4 {
            let time = Double(step) * 0.35
            clock.advance(to: time)
            perception.send(frame(UInt64(step + 1), time: time, range: range))
            await drain()
        }
        XCTAssertEqual(coordinator.state, range == 2 ? .following : .holdingDistance)
        XCTAssertEqual(motion.goals.count, range == 2 ? 1 : 0)
        XCTAssertEqual(motion.readySignals, 0)
        XCTAssertEqual(announcements, ["I'm ready to follow you."])
        XCTAssertTrue(motion.alignments.isEmpty, "Already aligned post-stop observations need no zero-angle motor operation")
        _ = await coordinator.stop()
        motion.releaseNavigation()
        clock.advance(to: 100)
        await drain()
    }

    func testMovingCenteringKeepsOwnerThroughModerateBearingButStopsForLargeTurn() async {
        await assertMovingBearing(threshold: .pi / 4, bearings: [20, 40], outside: 50)
    }

    func testWideMovingCenteringKeepsOwnerForFarSideVisibleTarget() async {
        await assertMovingBearing(threshold: .pi / 2, bearings: [20, 40, 60, 80, 85], outside: 100)
    }

    private func assertMovingBearing(threshold: Double, bearings: [Double], outside: Double) async {
        let perception = FollowPerceptionFake(), motion = LiveGoalFollowMotionFake(), clock = ManualFollowClock()
        motion.legacy.useSourceClock(clock)
        motion.legacy.suspendAlignment = true
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        config.movingTargetFacingAngle = threshold
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, range: 2))
        await drain()
        let stops = motion.legacy.stops
        for (index, degrees) in bearings.enumerated() {
            let step = index + 1
            let time = Double(step) * 0.3
            clock.advance(to: time)
            perception.send(frame(UInt64(step + 1), time: time, range: 2, angle: degrees * .pi / 180))
            await drain()
            XCTAssertEqual(coordinator.state, .following)
            XCTAssertEqual(motion.legacy.stops, stops)
        }
        XCTAssertFalse(motion.liveGoals.isEmpty)
        let finalTime = Double(bearings.count + 1) * 0.3
        clock.advance(to: finalTime)
        perception.send(frame(UInt64(bearings.count + 2), time: finalTime, range: 2, angle: outside * .pi / 180))
        await drain()
        XCTAssertEqual(coordinator.state, .aligning)
        XCTAssertGreaterThan(motion.legacy.stops, stops)
        _ = await coordinator.stop()
        motion.legacy.releaseNavigation()
        motion.legacy.releaseAlignment()
        clock.advance(to: 100)
        await drain()
    }

    private func drain() async { for _ in 0..<60 { await Task.yield() } }

    private func frame(_ sequence: UInt64, time: Double, range: Double = 1.65,
                       angle: Double = 0, yaw: Double = 0, visible: Bool = true, depth: Bool = true,
                       status: FollowInferenceStatus? = nil, verificationMiss: Bool = false) -> FollowPerceptionEvent {
        let id = ARFrameID(generation: 1, sequence: sequence)
        let pose = Pose2D(position: .zero, yaw: yaw)
        return .frame(.init(frameID: id, timestamp: time, pose: pose, depthAvailable: depth,
            people: visible ? [.init(frameID: id, timestamp: time, confidence: 0.99,
                boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6),
                position: Vec2(range * cos(angle), range * sin(angle)), pose: pose, bodyVerified: true)] : [],
            trackingQuality: .normal, perceptionDiagnostics: (verificationMiss ? .executed : status).map {
                var facts = FollowPerceptionDiagnostics(frameID: id, timestamp: time, inferenceStatus: $0, inferenceFailureReason: nil,
                    rawDetectorCount: nil, rawPersonCount: nil, projectionAttemptedCount: nil,
                    projectionAcceptedCount: nil, projectionRejectedCount: nil,
                    projectedPersonCount: visible ? 1 : 0, candidates: [])
                if verificationMiss {
                    facts.personVerification = [.init(rawPersonID: 0, accepted: false,
                        reason: "no_matching_body", matchingBodies: 0)]
                }
                return facts
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
