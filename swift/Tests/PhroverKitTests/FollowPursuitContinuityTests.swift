import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowPursuitContinuityTests: XCTestCase {
    private func drain() async { for _ in 0..<60 { await Task.yield() } }
    private func frame(_ sequence: UInt64, time: Double, range: Double, angle: Double = 0) -> FollowPerceptionEvent {
        let id = ARFrameID(generation: 1, sequence: sequence)
        let pose = Pose2D(position: .zero, yaw: 0)
        return .frame(.init(frameID: id, timestamp: time, pose: pose, depthAvailable: true,
            people: [.init(frameID: id, timestamp: time, confidence: 0.99,
                boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6),
                position: Vec2(range * cos(angle), range * sin(angle)), pose: pose, bodyVerified: true)], trackingQuality: .normal))
    }

    func testLastSeenAreaBridgesMissesWithoutReplacingOwnerThenStopsToSearch() async {
        for outcome in ["search", "return", "cancel", "camera", "failed_stop"] {
            let returns = outcome == "return"
            let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
            motion.legacy.suspendNavigation = true
            motion.legacy.suspendRotation = true
            motion.preservesFollowGoalOwner = true
            var targetFeed: FollowTargetSpeedFeed?
            motion.performHandler = { request, context in
                if let feed = FollowTargetSpeedScope.feed { targetFeed = feed }
                return await motion.legacy.performContextual(request, context: context)
            }
            var pauses = 0, updates: [Vec2] = []
            motion.pauseHandler = { pauses += 1; return true }
            motion.goalUpdateHandler = { updates.append($0); return true }
            var config = FollowMeConfiguration.continuous
            config.stationaryPauseSeconds = 0; config.departureRangeIncrease = 0
            config.planningMode = .reactiveDepth
            config.continuousTargetCentering = true
            config.continuousHoldingFacing = true
            config.followImmediatelyAfterAlignment = true
            config.approachLastSeenTarget = true
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                configuration: config, eventSink: { _, _ in })
            _ = await coordinator.start()
            perception.send(frame(1, time: 0, range: 2.5)); await drain()
            clock.advance(to: 0.1)
            perception.send(frame(2, time: 0.1, range: 2.5)); await drain()
            XCTAssertNotNil(targetFeed?.sample, "Establish measured pursuit before simulating target loss")
            updates.removeAll()
            for index in 1...5 {
                let time = Double(index) * 0.3
                clock.advance(to: time)
                perception.send(.frame(.init(frameID: .init(generation: 1, sequence: UInt64(index + 2)), timestamp: time,
                    pose: .init(position: .zero, yaw: 0), depthAvailable: true, people: [], trackingQuality: .normal)))
                await drain()
            }
            XCTAssertEqual(pauses, 0, "Healthy frames without a person should bridge to the last-seen area")
            XCTAssertEqual(coordinator.state, .following)
            XCTAssertEqual(motion.legacy.goals.count, 1)
            XCTAssertFalse(updates.isEmpty, "The approach must replace the predicted lead goal with the last-seen goal")
            XCTAssertEqual(updates.count, 1, "Misses must not move the destination or renew its authority")
            XCTAssertEqual(targetFeed?.sample?.timestamp, 0.1, "Never retimestamp the last image as a fresh sighting")
            XCTAssertEqual(targetFeed?.sample?.frameID, ARFrameID(generation: 1, sequence: 2))
            XCTAssertEqual(targetFeed?.sample?.velocity, .zero)
            XCTAssertNotNil(targetFeed?.sample?.lastSeenApproach)
            if outcome == "cancel" || outcome == "camera" {
                if outcome == "cancel" { _ = await coordinator.stop() }
                else {
                    clock.advance(to: 2.01); await drain()
                    XCTAssertEqual(pauses, 1, "Camera loss must stop even while approaching remembered space")
                }
                XCTAssertNil(targetFeed?.sample)
                clock.advance(to: 3.11); await drain()
                XCTAssertEqual(coordinator.state, outcome == "cancel" ? .stopped : .following,
                    "A revoked memory timer must not start search")
                _ = await coordinator.stop()
                motion.legacy.releaseNavigation(); motion.legacy.releaseRotation()
                continue
            }
            if returns {
                clock.advance(to: 1.6)
                perception.send(frame(8, time: 1.6, range: 2.5)); await drain()
                XCTAssertEqual(coordinator.state, .following)
                XCTAssertNil(targetFeed?.sample?.lastSeenApproach)
            }
            for index in 0...4 {
                let time = 1.8 + Double(index) * 0.3
                clock.advance(to: time)
                perception.send(returns ? frame(UInt64(index + 9), time: time, range: 2.5) :
                    .frame(.init(frameID: .init(generation: 1, sequence: UInt64(index + 9)), timestamp: time,
                        pose: .init(position: .zero, yaw: 0), depthAvailable: true, people: [], trackingQuality: .normal)))
                await drain()
            }
            if outcome == "failed_stop" { motion.legacy.stopError = true }
            clock.advance(to: 3.11); await drain()
            XCTAssertEqual(coordinator.state, outcome == "failed_stop" ? .failed("Rover stop could not be confirmed.") :
                returns ? .following : .reacquiring,
                "Only a fresh verified match may cancel the fixed last-seen search deadline")
            XCTAssertEqual(motion.legacy.goals.count, 1, "Never replace pursuit to replenish its depth recovery allowance")
            motion.legacy.stopError = false
            _ = await coordinator.stop()
            motion.legacy.releaseNavigation(); motion.legacy.releaseRotation()
        }
    }

    func testVerificationTimeoutOffersFullStoppedReturnWindow() async {
        for outcome in ["timeout", "return", "failed_stop", "cancel", "reset", "tracking_fault"] {
            let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
            motion.legacy.suspendNavigation = true
            motion.preservesFollowGoalOwner = true
            motion.pauseHandler = { true }
            var config = FollowMeConfiguration.continuous
            config.stationaryPauseSeconds = 0; config.departureRangeIncrease = 0
            config.planningMode = .reactiveDepth
            config.continuousTargetCentering = true
            config.continuousHoldingFacing = true
            config.followImmediatelyAfterAlignment = true
            config.waitForUserReturn = true
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                configuration: config, eventSink: { _, _ in })
            _ = await coordinator.start()
            perception.send(frame(1, time: 0, range: 2.5)); await drain()
            clock.advance(to: 0.1)
            perception.send(.frame(.init(frameID: .init(generation: 1, sequence: 2), timestamp: 0.1,
                pose: .init(position: .zero, yaw: 0), depthAvailable: true, people: [], trackingQuality: .normal)))
            await drain()
            if outcome == "failed_stop" { motion.legacy.stopError = true }
            if outcome == "tracking_fault" {
                clock.advance(to: 0.2)
                perception.send(.frame(.init(frameID: .init(generation: 1, sequence: 3), timestamp: 0.2,
                    pose: .init(position: .zero, yaw: 0), depthAvailable: true, people: [], trackingQuality: .limited)))
                await drain()
            }
            clock.advance(to: 2.11); await drain()
            motion.legacy.releaseNavigation(); await drain()
            if outcome == "failed_stop" {
                XCTAssertEqual(coordinator.state, .failed("Rover stop could not be confirmed."))
                motion.legacy.stopError = false
                _ = await coordinator.stop()
                continue
            }
            if outcome == "tracking_fault" {
                guard case .failed = coordinator.state else { XCTFail("AR fault must remain terminal"); continue }
                _ = await coordinator.stop()
                continue
            }
            guard case .waitingForReturn = coordinator.state else {
                XCTFail("Verification timeout must offer the configured return wait, got \(coordinator.state)")
                _ = await coordinator.stop()
                continue
            }
            XCTAssertEqual(motion.legacy.goals.count, 1)
            if outcome == "cancel" { _ = await coordinator.stop() }
            if outcome == "reset" {
                perception.send(.frame(.init(frameID: .init(generation: 2, sequence: 1), timestamp: 2.12,
                    pose: .init(position: .zero, yaw: 0), depthAvailable: true, people: [], trackingQuality: .normal)))
                await drain()
            }
            clock.advance(to: 16.5); await drain()
            if outcome == "return" || outcome == "cancel" || outcome == "reset" {
                for (sequence, time) in [(UInt64(3), 16.5), (4, 16.65), (5, 16.8)] {
                    clock.advance(to: time)
                    perception.send(frame(sequence, time: time, range: 2.5)); await drain()
                }
                if outcome == "return" {
                    XCTAssertEqual(coordinator.state, .searching, "Fresh verified return restarts readiness")
                } else {
                    XCTAssertEqual(coordinator.state, outcome == "cancel" ? .stopped :
                        .failed("AR session reset while waiting for your return."))
                }
            } else {
                guard case .waitingForReturn = coordinator.state else {
                    XCTFail("Return window must last 15 seconds after confirmed stop"); continue
                }
                clock.advance(to: 17.12); await drain()
                XCTAssertEqual(coordinator.state, .failed("Return wait expired. Come into view and say follow me to restart."))
            }
            XCTAssertEqual(motion.legacy.goals.count, 1, "No movement merely from waiting")
            _ = await coordinator.stop()
        }
    }

    func testStalePerceptionPausesAndResumesOriginalGoalRequest() async {
        let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
        motion.legacy.suspendNavigation = true
        motion.preservesFollowGoalOwner = true
        var pauses = 0, resumes = 0, updates: [Vec2] = []
        var boundary = 0.0
        motion.pauseHandler = { pauses += 1; boundary = clock.now; return true }
        motion.canResumeHandler = { $0 > boundary }
        motion.resumeHandler = { timestamp in
            guard timestamp > boundary else { return false }
            resumes += 1; return true
        }
        motion.goalUpdateHandler = { updates.append($0); return true }
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0; config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, range: 2.5))
        await drain()
        let stops = motion.legacy.stops
        XCTAssertEqual(motion.legacy.goals.count, 1)
        clock.advance(to: 0.51)
        await drain()
        XCTAssertEqual(pauses, 1)
        XCTAssertEqual(motion.legacy.stops, stops, "The existing driver owns the pause stop")
        clock.advance(to: 0.6)
        perception.send(frame(2, time: 0.49, range: 2.6))
        await drain()
        XCTAssertEqual(resumes, 0, "A pre-pause capture cannot resume motion")
        clock.advance(to: 0.8)
        perception.send(frame(3, time: 0.8, range: 2.9))
        await drain()
        XCTAssertEqual(resumes, 1)
        XCTAssertFalse(updates.isEmpty)
        XCTAssertEqual(motion.legacy.goals.count, 1)
        XCTAssertEqual(motion.contexts.count, 1, "No new owner or recovery budget")
        XCTAssertEqual(motion.legacy.stops, stops)
        // A terminating planned owner may refuse an update. Wait for its result
        // rather than replacing it and hiding a terminal depth failure.
        motion.goalUpdateHandler = { _ in false }
        clock.advance(to: 1.1)
        perception.send(frame(4, time: 1.1, range: 3.3))
        await drain()
        XCTAssertEqual(motion.legacy.goals.count, 1)
        _ = await coordinator.stop()
        motion.legacy.releaseNavigation()
        clock.advance(to: 100)
        await drain()
    }

    func testContinuousCenteringStartsOffAxisAndPausesVerificationGapsUnderOneOwner() async {
        for angle in [-24.4, -4, 4, 24.4] {
            let range = abs(angle) < 5 ? 1.5 : 2.5
            let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
            motion.legacy.useSourceClock(clock)
            motion.legacy.suspendNavigation = true
            motion.preservesFollowGoalOwner = true
            var pauses = 0, resumes = 0, boundary = 0.0
            motion.pauseHandler = { pauses += 1; boundary = clock.now; return true }
            motion.canResumeHandler = { $0 > boundary }
            motion.resumeHandler = { _ in resumes += 1; return true }
            motion.goalUpdateHandler = { _ in true }
            var config = FollowMeConfiguration.continuous
            config.stationaryPauseSeconds = 0
            config.planningMode = .reactiveDepth
            config.followImmediatelyAfterAlignment = true
            config.continuousHoldingFacing = true
            config.continuousTargetCentering = true
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                configuration: config, eventSink: { _, _ in })
            _ = await coordinator.start(); await drain()
            perception.send(frame(1, time: 0, range: range, angle: angle * .pi / 180)); await drain()
            XCTAssertTrue(motion.legacy.goals.isEmpty, "Detection alone cannot bypass the stop fence")
            for (sequence, time) in [(UInt64(2), 0.31), (3, 0.42)] {
                clock.advance(to: time)
                perception.send(frame(sequence, time: time, range: range, angle: angle * .pi / 180)); await drain()
            }
            XCTAssertEqual(coordinator.state, range == 1.5 ? .holdingDistance : .following)
            XCTAssertEqual(motion.legacy.goals.count, 1)
            XCTAssertFalse(motion.contexts.contains { $0.purpose == .followAlignment })
            for (sequence, time) in [(UInt64(4), 0.5), (5, 0.6)] {
                clock.advance(to: time)
                perception.send(.frame(.init(frameID: .init(generation: 1, sequence: sequence), timestamp: time,
                    pose: .init(position: .zero, yaw: 0), depthAvailable: true, people: [], trackingQuality: .normal)))
                await drain()
            }
            XCTAssertEqual(pauses, 1, "Repeated misses must neither replace the owner nor renew its pause")
            XCTAssertEqual(resumes, 0)
            clock.advance(to: 0.7)
            perception.send(frame(6, time: 0.49, range: range, angle: angle * .pi / 180)); await drain()
            XCTAssertEqual(resumes, 0, "Pre-stop evidence cannot resume centering")
            clock.advance(to: 0.8)
            perception.send(frame(7, time: 0.8, range: range, angle: angle * .pi / 180)); await drain()
            XCTAssertEqual(resumes, 1)
            XCTAssertEqual(motion.legacy.goals.count, 1)
            _ = await coordinator.stop(); motion.legacy.releaseNavigation()
        }
    }

    func testContinuousVerificationPauseIsBoundedAndCannotResumeAfterStopFailureOrCancellation() async {
        for fault in ["timeout", "failed_stop", "cancel"] {
            let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
            motion.legacy.suspendNavigation = true
            motion.preservesFollowGoalOwner = true
            var pauses = 0
            motion.pauseHandler = { pauses += 1; return fault != "failed_stop" }
            motion.canResumeHandler = { _ in true }
            motion.resumeHandler = { _ in XCTFail("No fresh verified target"); return false }
            var config = FollowMeConfiguration.continuous
            config.stationaryPauseSeconds = 0; config.departureRangeIncrease = 0
            config.planningMode = .reactiveDepth
            config.followImmediatelyAfterAlignment = true
            config.continuousHoldingFacing = true
            config.continuousTargetCentering = true
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                configuration: config, eventSink: { _, _ in })
            _ = await coordinator.start()
            perception.send(frame(1, time: 0, range: 2.5)); await drain()
            for (sequence, time) in [(UInt64(2), 0.1), (3, 0.3), (4, 0.5)] {
                clock.advance(to: time)
                perception.send(.frame(.init(frameID: .init(generation: 1, sequence: sequence), timestamp: time,
                    pose: .init(position: .zero, yaw: 0), depthAvailable: true, people: [], trackingQuality: .normal)))
                await drain()
                if fault == "cancel" { _ = await coordinator.stop(); break }
            }
            XCTAssertEqual(pauses, 1)
            clock.advance(to: 2.11); await drain()
            switch fault {
            case "timeout":
                XCTAssertEqual(coordinator.state, .failed(FollowPerceptionIssue.targetVerificationUnavailable.message))
            case "failed_stop": XCTAssertEqual(coordinator.state, .failed("Rover stop could not be confirmed."))
            default: XCTAssertEqual(coordinator.state, .stopped)
            }
            XCTAssertEqual(motion.legacy.goals.count, 1)
            _ = await coordinator.stop(); motion.legacy.releaseNavigation()
        }
    }

    func testPrePauseFrameCannotRenewOutageDeadline() async {
        let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
        motion.legacy.suspendNavigation = true
        motion.preservesFollowGoalOwner = true
        var boundary = 0.0
        motion.pauseHandler = { boundary = clock.now; return true }
        motion.canResumeHandler = { $0 > boundary }
        motion.resumeHandler = { _ in XCTFail("No eligible post-pause frame"); return false }
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0; config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, range: 2.2))
        await drain()
        clock.advance(to: 0.51)
        await drain()
        clock.advance(to: 0.6)
        perception.send(frame(2, time: 0.49, range: 2.2))
        await drain()
        clock.advance(to: 1.01)
        await drain()
        clock.advance(to: 2.52)
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Camera frame stale or timestamp invalid. Check camera delivery and inference latency."))
        motion.legacy.releaseNavigation()
        clock.advance(to: 100)
        await drain()
    }

    func testRecordedNearTargetPauseResumesAcrossHoldingBoundaryOrTimesOutWithRealCause() async {
        for range in [1.50, 1.55, 1.56, 1.65, 1.74, 1.75, 1.76] {
            for acceptsResume in [false, true] {
                let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
                motion.legacy.suspendNavigation = true
                motion.preservesFollowGoalOwner = true
                var pauses = 0, resumes = 0
                var goals: [Vec2] = []
                motion.pauseHandler = { pauses += 1; return true }
                motion.canResumeHandler = { $0 > 0.1 }
                motion.resumeHandler = { _ in resumes += 1; return true }
                motion.goalUpdateHandler = { goals.append($0); return acceptsResume }
                var config = FollowMeConfiguration.continuous
                config.stationaryPauseSeconds = 0; config.departureRangeIncrease = 0
                config.planningMode = .reactiveDepth
                config.followImmediatelyAfterAlignment = true
                config.continuousHoldingFacing = true
                config.continuousTargetCentering = true
                config.goalLeadSeconds = 0
                let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                    configuration: config, eventSink: { _, _ in })
                _ = await coordinator.start()
                perception.send(frame(1, time: 0, range: 2.5)); await drain()
                clock.advance(to: 0.1)
                perception.send(.frame(.init(frameID: .init(generation: 1, sequence: 2), timestamp: 0.1,
                    pose: .init(position: .zero, yaw: 0), depthAvailable: true, people: [], trackingQuality: .normal)))
                await drain()
                XCTAssertEqual(pauses, 1)
                for index in 0..<10 {
                    let time = 0.2 + Double(index) * 0.2
                    clock.advance(to: time)
                    perception.send(frame(UInt64(index + 3), time: time, range: range, angle: 0.44))
                    await drain()
                }
                XCTAssertFalse(goals.isEmpty, "Every near-target resume must reach goal admission: \(range)")
                clock.advance(to: 2.11); await drain()
                if acceptsResume {
                    XCTAssertEqual(resumes, 1)
                    XCTAssertEqual(coordinator.state, range <= 1.65 ? .holdingDistance : .following, "range \(range)")
                    if let goal = goals.first {
                        XCTAssertEqual(goal.length, range <= 1.65 ? 0 : range - 1.5, accuracy: 1e-9)
                    }
                } else {
                    XCTAssertEqual(resumes, 0)
                    XCTAssertEqual(coordinator.state, .failed(FollowPerceptionIssue.targetVerificationUnavailable.message),
                        "Healthy frames cannot erase the unresolved pause cause or renew its deadline")
                }
                XCTAssertEqual(motion.legacy.goals.count, 1)
                _ = await coordinator.stop(); motion.legacy.releaseNavigation()
            }
        }
    }

    func testPostPauseDirectionChangeNeverResumesOldForwardRoute() async {
        let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
        motion.legacy.suspendNavigation = true
        motion.legacy.suspendAlignment = true
        motion.legacy.useSourceClock(clock)
        motion.preservesFollowGoalOwner = true
        var boundary = 0.0, resumes = 0
        motion.pauseHandler = { boundary = clock.now; return true }
        motion.canResumeHandler = { $0 > boundary }
        motion.resumeHandler = { _ in resumes += 1; return true }
        motion.goalUpdateHandler = { _ in XCTFail("Turn first; do not release the old forward drive"); return true }
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0; config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, range: 2.2))
        await drain()
        clock.advance(to: 0.51)
        await drain()
        clock.advance(to: 0.8)
        perception.send(frame(2, time: 0.8, range: 2.2, angle: 0.32))
        await drain()
        XCTAssertEqual(coordinator.state, .aligning)
        XCTAssertEqual(resumes, 0)
        XCTAssertEqual(motion.legacy.goals.count, 1)
        _ = await coordinator.stop()
        motion.legacy.releaseNavigation()
        motion.legacy.releaseAlignment()
        clock.advance(to: 100)
        await drain()
    }
}
