import Foundation
import RoverNav
import XCTest
@testable import PhroverKit

@MainActor
final class FollowMeCoordinatorTests: XCTestCase {
    func testInvalidSearchAttemptBudgetsFailBeforeDispatch() async {
        for (attempts, interruptions) in [(0, 8), (-1, 8), (72, 0), (72, -1)] {
            let perception = FollowPerceptionFake()
            let motion = FollowMotionFake()
            motion.suspendRotation = true
            let clock = ManualFollowClock()
            var config = FollowMeConfiguration()
            config.stationaryPauseSeconds = 0
            config.maximumScanAttempts = attempts
            config.maximumScanInterruptions = interruptions
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                configuration: config, eventSink: { _, _ in })
            _ = await coordinator.start(); await drain()
            perception.send(frame(1)); await drain()
            XCTAssertEqual(coordinator.state, .failed("Invalid search configuration."))
            XCTAssertTrue(motion.rotations.isEmpty)
            _ = await coordinator.stop(); motion.releaseRotation()
        }
    }

    func testCancelledScanDoesNotSpendCompletedCoverageAndRepeatedInterruptionsAreBounded() async throws {
        for onlyInterruptions in [false, true] {
            let perception = FollowPerceptionFake()
            let motion = ContextualFollowMotionFake()
            let clock = ManualFollowClock()
            motion.legacy.useSourceClock(clock)
            motion.legacy.suspendRotation = true
            var config = FollowMeConfiguration()
            config.stationaryPauseSeconds = 0
            config.scanObservationSeconds = 0
            config.scanIncrement = .pi / 6
            config.maximumScanRotation = .pi / 3
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                configuration: config, eventSink: { _, _ in })
            _ = await coordinator.start(); await drain()
            perception.send(frame(1)); await drain()
            for step in 0..<(onlyInterruptions ? 10 : 3) {
                guard motion.contexts.count > step else { break }
                let context = motion.contexts[step]
                let cancelled = onlyInterruptions || step == 0
                motion.resultOverride = .init(result: cancelled ? .cancelled : .arrived,
                    context: .init(request: context, controllerOperationID: UInt64(step + 1), purpose: .followScan, profile: nil),
                    failure: nil, stopOutcome: .confirmed,
                    measuredScanResponse: cancelled ? nil : .init(sourceGeneration: 1, responseRate: 2.1),
                    // Absolute sampled travel includes back-and-forth motion;
                    // only the net directed arc counts as search coverage.
                    searchSweepTravel: cancelled ? nil : .pi / 3, searchSweepProgress: cancelled ? nil : .pi / 6)
                if onlyInterruptions {
                    // Match the device failure: perception revokes the current
                    // motor owner before its canceled result is delivered.
                    perception.send(frame(UInt64(2 + step * 2), at: clock.now - 0.6)); await drain()
                }
                motion.legacy.releaseRotation(); await drain()
                if onlyInterruptions, coordinator.isActive {
                    perception.send(frame(UInt64(3 + step * 2), at: clock.now)); await drain()
                }
                if !onlyInterruptions && step == 1 {
                    XCTAssertEqual(coordinator.state, .searching, "One canceled request plus one completed turn is not a completed scan")
                    XCTAssertEqual(motion.contexts.count, 3)
                }
            }
            if onlyInterruptions {
                XCTAssertEqual(coordinator.state, .failed("Search interrupted before the scan completed. Wait for stable perception and restart following."))
                XCTAssertLessThanOrEqual(motion.contexts.count, 8)
            } else {
                XCTAssertEqual(coordinator.state, .failed("No person found."))
                XCTAssertEqual(motion.contexts.count, 3)
            }
            _ = await coordinator.stop(); motion.legacy.releaseRotation()
        }
    }

    func testCompletedReadinessReacquiresIntoFollowingWithoutRepeatingInitialAlignment() async {
        let (coordinator, perception, motion, clock) = sourcedRecoverySetup()
        motion.suspendRotation = true
        await acquireWaiting(coordinator, perception, clock)
        let alignments = motion.alignments.count
        perception.send(frame(4, at: clock.now)); await drain()
        XCTAssertEqual(coordinator.state, .reacquiring)
        // Already beyond the retained 4.0 + 0.3 m departure baseline, but not
        // precisely facing the rover. The following controller owns steering.
        perception.send(frame(5, at: clock.now, pose: .init(position: .zero, yaw: .pi / 2),
            people: [person(5, at: clock.now, x: 0.7, y: 4.4)])); await drain()
        XCTAssertTrue(motion.goals.isEmpty, "The reacquisition frame cannot clear its own stop fence")
        motion.suspendStop = true // A redundant stop would stall the handoff and age its matched frame.
        await matchedBoundaryFrame(6, perception, clock, x: 0.7, y: 4.4)
        XCTAssertEqual(coordinator.state, .following)
        XCTAssertEqual(motion.goals.count, 1)
        XCTAssertEqual(motion.alignments.count, alignments)
        XCTAssertEqual(motion.readySignals, 1)
        motion.suspendStop = false; motion.releaseStop()
        _ = await coordinator.stop(); motion.releaseRotation()
    }

    func testWaitingForDepartureHoldsBriefVerificationMissAndRetainsBaseline() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        await acquireWaiting(coordinator, perception, clock)
        let alignments = motion.alignments.count
        func input(_ sequence: UInt64, verified: Bool, range: Double) -> FollowPerceptionEvent {
            let id = ARFrameID(generation: 1, sequence: sequence)
            let observed = FollowPersonObservation(frameID: id, timestamp: clock.now, confidence: 0.99,
                boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6), position: .init(0, range),
                pose: .init(position: .zero, yaw: .pi / 2), bodyVerified: true)
            var facts = FollowPerceptionDiagnostics(frameID: id, timestamp: clock.now, inferenceStatus: .executed,
                inferenceFailureReason: nil, rawDetectorCount: 1, rawPersonCount: 1,
                projectionAttemptedCount: verified ? 1 : 0, projectionAcceptedCount: verified ? 1 : 0,
                projectionRejectedCount: 0, projectedPersonCount: verified ? 1 : 0, candidates: [])
            facts.personVerification = [.init(rawPersonID: 0, accepted: verified,
                reason: verified ? "body_verified" : "no_matching_body", matchingBodies: verified ? 1 : 0)]
            return .frame(.init(frameID: id, timestamp: clock.now, pose: observed.pose, depthAvailable: true,
                people: verified ? [observed] : [], trackingQuality: .normal, perceptionDiagnostics: facts))
        }
        perception.send(input(4, verified: true, range: 4)); await drain()
        perception.send(input(5, verified: false, range: 4)); await drain()
        XCTAssertEqual(coordinator.state, .waitingForMovement)
        XCTAssertNil(coordinator.trackedPersonFrameID)
        XCTAssertTrue(motion.goals.isEmpty)
        clock.advance(to: clock.now + 0.4)
        motion.suspendStop = true
        perception.send(input(6, verified: true, range: 4.4)); await drain()
        XCTAssertEqual(coordinator.state, .following)
        XCTAssertEqual(motion.goals.count, 1)
        XCTAssertEqual(motion.alignments.count, alignments)
        XCTAssertEqual(motion.readySignals, 1)
        motion.suspendStop = false; motion.releaseStop()
        _ = await coordinator.stop()
    }

    func testProductionSearchUsesShortLookButStillRequiresFreshPostIntervalPerception() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        motion.suspendRotation = true
        _ = await coordinator.start(); await drain()
        clock.advance(to: 5); perception.send(frame(1, at: 5)); await drain()
        XCTAssertEqual(motion.rotations.count, 1)
        motion.releaseRotation(); await drain()
        clock.advance(to: 5.05); perception.send(frame(2, at: 5.05)); await drain()
        XCTAssertEqual(motion.rotations.count, 1)
        clock.advance(to: 5.11); perception.send(frame(3, at: 5.09)); await drain()
        XCTAssertEqual(motion.rotations.count, 1, "A capture from before the look boundary cannot authorize another turn")
        perception.send(frame(4, at: 5.11)); await drain()
        XCTAssertEqual(motion.rotations.count, 2, "Fresh settled perception need not wait a full second")
        _ = await coordinator.stop(); motion.releaseRotation()
    }

    func testVerifiedAcquisitionMissHoldsStoppedWithoutRecoveryChurnAndHasFixedDeadline() async {
        for scenario in ["recovers", "expires", "late_match", "stop_failed", "cancelled"] {
            let perception = FollowPerceptionFake()
            let motion = FollowMotionFake()
            let clock = ManualFollowClock()
            motion.useSourceClock(clock)
            motion.suspendRotation = true
            var config = FollowMeConfiguration()
            config.stationaryPauseSeconds = 0
            var events: [String] = []
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                configuration: config) { event, _ in events.append(event) }
            func input(_ sequence: UInt64, _ time: Double, verified: Bool) -> FollowPerceptionEvent {
                let id = ARFrameID(generation: 1, sequence: sequence)
                let person = FollowPersonObservation(frameID: id, timestamp: time, confidence: 0.99,
                    boundingBox: CGRect(x: 0.7, y: 0.2, width: 0.3, height: 0.75), position: .init(1.5, 0.3),
                    pose: .init(position: .zero, yaw: 0), rawPersonID: 0, bodyVerified: true)
                var facts = FollowPerceptionDiagnostics(frameID: id, timestamp: time, inferenceStatus: .executed,
                    inferenceFailureReason: nil, rawDetectorCount: 1, rawPersonCount: 1,
                    projectionAttemptedCount: verified ? 1 : 0, projectionAcceptedCount: verified ? 1 : 0,
                    projectionRejectedCount: 0, projectedPersonCount: verified ? 1 : 0, candidates: [])
                facts.personVerification = [.init(rawPersonID: 0, accepted: verified,
                    reason: verified ? "body_verified" : "invalid_person_box", matchingBodies: verified ? 1 : 0)]
                return .frame(.init(frameID: id, timestamp: time, pose: person.pose, depthAvailable: true,
                    people: verified ? [person] : [], trackingQuality: .normal, perceptionDiagnostics: facts))
            }
            _ = await coordinator.start(); await drain()
            perception.send(input(1, 0, verified: true)); await drain()
            XCTAssertEqual(coordinator.state, .aligning)
            if scenario == "stop_failed" { motion.stopError = true }
            clock.advance(to: 0.1); perception.send(input(2, 0.1, verified: false)); await drain()
            if scenario == "stop_failed" {
                XCTAssertEqual(coordinator.state, .failed("Rover stop could not be confirmed."))
                clock.advance(to: 1); await drain()
                XCTAssertFalse(events.contains("follow_recovery.started"))
                motion.stopError = false; _ = await coordinator.stop()
                continue
            }
            XCTAssertEqual(coordinator.state, .aligning, "A brief verification miss must not start a recovery sweep")
            XCTAssertNil(coordinator.trackedPersonFrameID)
            XCTAssertTrue(motion.rotations.isEmpty)
            XCTAssertTrue(motion.goals.isEmpty)
            if scenario == "cancelled" {
                _ = await coordinator.stop()
                clock.advance(to: 1); await drain()
                XCTAssertEqual(coordinator.state, .stopped)
                XCTAssertFalse(events.contains("follow_recovery.started"))
                continue
            }
            clock.advance(to: 0.3); perception.send(input(3, 0.3, verified: false)); await drain()
            if scenario == "late_match" {
                clock.advance(to: 0.601, wakeSleepers: false)
                perception.send(input(4, 0.601, verified: true)); await drain()
                XCTAssertEqual(coordinator.state, .reacquiring, "An undelivered timer cannot extend the first-miss deadline")
                _ = await coordinator.stop(); motion.releaseRotation()
                continue
            }
            if scenario == "recovers" {
                clock.advance(to: 0.45); perception.send(input(4, 0.45, verified: true)); await drain()
                XCTAssertEqual(coordinator.state, .aligning)
                XCTAssertFalse(events.contains("follow_recovery.started"))
            } else {
                clock.advance(to: 0.5); perception.send(input(4, 0.5, verified: false)); await drain()
                clock.advance(to: 0.601); await drain()
                XCTAssertEqual(coordinator.state, .reacquiring, "Repeated misses cannot extend the first-miss deadline")
            }
            _ = await coordinator.stop(); motion.releaseRotation()
        }
    }

    func testInitialSearchCountsOversweptArcThenLooksBeforeNextBoundedStep() async throws {
        let perception = FollowPerceptionFake()
        let motion = ContextualFollowMotionFake()
        let clock = ManualFollowClock()
        motion.legacy.useSourceClock(clock)
        motion.legacy.suspendRotation = true
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.maximumScanRotation = 0.6
        config.scanObservationSeconds = 1 // Exercise configured long-look accounting independently of the default.
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start(); await drain()
        perception.send(frame(1)); await drain()
        let first = try XCTUnwrap(motion.contexts.first)
        XCTAssertTrue(first.isSearchSweep)
        motion.resultOverride = .init(result: .arrived,
            context: .init(request: first, controllerOperationID: 1, purpose: .followScan, profile: nil),
            failure: nil, stopOutcome: .confirmed,
            measuredScanResponse: .init(sourceGeneration: 1, responseRate: 5.519739222415692),
            searchSweepTravel: 0.483189002088996, searchSweepProgress: 0.483189002088996)
        motion.legacy.releaseRotation(); await drain()
        XCTAssertEqual(coordinator.state, .searching, "Sweep completion is not person acquisition or readiness")
        XCTAssertEqual(motion.contexts.count, 1)
        for (id, time) in [(UInt64(2), 0.3), (3, 0.6), (4, 0.9), (5, 1.01)] {
            clock.advance(to: time); perception.send(frame(id, at: time)); await drain()
            if time < 1 { XCTAssertEqual(motion.contexts.count, 1) }
        }
        XCTAssertEqual(motion.contexts.count, 2)
        XCTAssertEqual(motion.legacy.rotations.last ?? -1, 0.116810997911004, accuracy: 1e-12,
            "The remaining arc includes measured overshoot, not just requested increments")
        XCTAssertEqual(motion.contexts.last?.scanRemaining, 0)
        let second = try XCTUnwrap(motion.contexts.last)
        motion.resultOverride = .init(result: .arrived,
            context: .init(request: second, controllerOperationID: 2, purpose: .followScan, profile: nil),
            failure: nil, stopOutcome: .confirmed,
            measuredScanResponse: .init(sourceGeneration: 1, responseRate: 5.6), searchSweepTravel: 0.2, searchSweepProgress: 0.2)
        motion.legacy.releaseRotation(); await drain()
        for (id, time) in [(UInt64(6), 1.3), (7, 1.6), (8, 1.9), (9, 2.02)] {
            clock.advance(to: time); perception.send(frame(id, at: time)); await drain()
        }
        XCTAssertEqual(motion.contexts.count, 2, "Measured sweep exhaustion cannot launch another increment")
        XCTAssertEqual(coordinator.state, .failed("No person found."))
        _ = await coordinator.stop(); motion.legacy.releaseRotation()
    }

    func testRecoveryObservationPauseSurvivesPerceptionAgingAtCompletion() async {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        let clock = ManualFollowClock()
        motion.legacy.useSourceClock(clock)
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        config.scanObservationSeconds = 1
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start(); await drain()
        perception.send(frame(1, people: [person(1)])); await drain()
        motion.autoArrive = true
        motion.onRecoveryArrival = {
            motion.onRecoveryArrival = nil
            clock.advance(to: 0.6, wakeSleepers: false)
        }
        perception.send(frame(2)); await drain()
        XCTAssertEqual(motion.headings.count, 1)
        clock.advance(to: 0.7, wakeSleepers: false)
        perception.send(frame(3, at: 0.7)); await drain()
        XCTAssertEqual(motion.headings.count, 1, "Completed confirmed turn retains its look obligation even if perception aged at completion")
        _ = await coordinator.stop()
    }
    func testSearchKeepsConfirmedResponseGainAcrossStepsAndResetsAtNewSession() async throws {
        let perception = FollowPerceptionFake()
        let motion = ContextualFollowMotionFake()
        let clock = ManualFollowClock()
        motion.legacy.useSourceClock(clock)
        motion.legacy.suspendRotation = true
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start(); await drain()
        perception.send(frame(1)); await drain()
        let first = try XCTUnwrap(motion.contexts.first)
        XCTAssertNil(first.scanResponseSeed)
        motion.resultOverride = .init(result: .arrived,
            context: .init(request: first, controllerOperationID: 1, purpose: .followScan, profile: nil),
            failure: nil, stopOutcome: .confirmed,
            measuredScanResponse: .init(sourceGeneration: 1, responseRate: 10))
        motion.legacy.releaseRotation(); await drain()
        for (id, time) in [(UInt64(2), 0.3), (3, 0.6), (4, 0.9), (5, 1.01)] {
            clock.advance(to: time); perception.send(frame(id, at: time)); await drain()
        }
        XCTAssertEqual(motion.contexts.count, 2)
        XCTAssertEqual(motion.contexts.last?.scanResponseSeed, .init(sourceGeneration: 1, responseRate: 10))
        _ = await coordinator.stop()
        motion.resultOverride = nil
        motion.legacy.releaseRotation(); await drain()
        _ = await coordinator.start(); await drain()
        perception.send(frame(6, at: clock.now)); await drain()
        XCTAssertEqual(motion.contexts.count, 3)
        XCTAssertNil(motion.contexts.last?.scanResponseSeed, "A new mission cannot inherit old turn calibration")
        _ = await coordinator.stop(); motion.legacy.releaseRotation()
    }
    func testConfiguredLongSearchLookRequiresCaptureAfterObservationInterval() async {
        let (coordinator, perception, motion, clock) = productionSetup(scanObservationSeconds: 1)
        motion.suspendRotation = true
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(1, at: 5))
        await drain()
        XCTAssertEqual(motion.rotations.count, 1)
        XCTAssertEqual(motion.rotations.first ?? 0, .pi / 18, accuracy: 1e-12)
        motion.releaseRotation()
        await drain()
        XCTAssertEqual(motion.rotations.count, 1, "No immediate next turn while the person is being searched for")
        for (id, time) in [(UInt64(2), 5.3), (3, 5.6), (4, 5.99)] {
            clock.advance(to: time); perception.send(frame(id, at: time)); await drain()
            XCTAssertEqual(motion.rotations.count, 1)
        }
        clock.advance(to: 6.01)
        perception.send(frame(5, at: 5.99))
        await drain()
        XCTAssertEqual(motion.rotations.count, 1, "A pre-interval capture cannot release the observation pause")
        perception.send(frame(6, at: 6.01))
        await drain()
        XCTAssertEqual(motion.rotations.count, 2)
        _ = await coordinator.stop()
        motion.releaseRotation()
    }
    func testDisplayedTargetRequiresFreshMatchedFrameAndClearsOnStop() async {
        let (coordinator, perception, _, clock) = productionSetup()
        XCTAssertNil(coordinator.trackedPersonFrameID)
        await acquireWaiting(coordinator, perception, clock)
        XCTAssertEqual(coordinator.trackedPersonFrameID, .init(generation: 1, sequence: 3))
        clock.advance(to: clock.now + 0.501)
        XCTAssertNil(coordinator.trackedPersonFrameID, "A stale lock cannot be presented as current")
        _ = await coordinator.stop()
        XCTAssertNil(coordinator.trackedPersonFrameID)
    }
    func testRecoveryRestorationRechecksDeadlineAtAtomicCommitWithoutTimerDelivery() async throws {
        // Reads after the matched association: preliminary eligibility (deadline/health),
        // follow expiry, restoration eligibility (deadline/health), final acceptance time.
        for (crossingRead, finalTime) in [(5, 10.0), (6, 10.0), (6, 10.001), (6, 9.999), (7, 10.0)] {
            let perception = FollowPerceptionFake()
            let motion = AbsoluteRecoveryMotionFake()
            let clock = ManualFollowClock()
            var config = FollowMeConfiguration()
            config.stationaryPauseSeconds = 0
            config.departureRangeIncrease = 0
            let sink = FollowDiagnosticRecordingSink()
            var armCommitClock = false
            var reads = 0
            func nextRead() {
                reads += 1
                if reads == crossingRead { clock.advance(to: finalTime, wakeSleepers: false) }
                else { clock.onNextRead = nextRead }
            }
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                configuration: config) { event, fields in
                sink.append(event, fields: fields)
                if armCommitClock && event == "follow_person.association" {
                    armCommitClock = false
                    clock.onNextRead = nextRead
                }
            }
            _ = await coordinator.start()
            perception.send(frame(1, people: [person(1, x: 1.5, y: 0)]))
            await drain()
            perception.send(frame(2))
            await drain()
            perception.send(frame(3, people: [person(3, x: 1.5, y: 0)]))
            await drain()
            XCTAssertEqual(coordinator.state, .reacquiring)
            clock.advance(to: 9.999, wakeSleepers: false)
            armCommitClock = true
            perception.send(frame(4, at: 9.8, people: [person(4, at: 9.8, x: 1.5, y: 0)]))
            await drain()
            XCTAssertEqual(reads, crossingRead, "Clock must reach the targeted restoration read")
            // Read 7 is the state-change diagnostic, after the phase/memory commit.
            let expired = crossingRead <= 6 && finalTime >= 10
            XCTAssertEqual(coordinator.state, expired ? .failed("Person lost.") : .holdingDistance,
                "Final validation at \(finalTime), read \(crossingRead), must decide actual restoration")
            XCTAssertEqual(sink.records.filter { $0.event == "follow_recovery.cleared" }.count, expired ? 0 : 1)
            XCTAssertEqual(sink.records.filter { $0.event == "follow_recovery.expired" }.count, expired ? 1 : 0)
            if expired {
                let record = try XCTUnwrap(sink.records.first { $0.event == "follow_recovery.expired" })
                let json = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap(record.fields["payload"]).utf8)) as? [String: Any])
                XCTAssertEqual(json["anchor_frame_id"] as? String, "1:1")
                XCTAssertEqual(json["reliable_frame_id"] as? String, "1:1")
            }
            _ = await coordinator.stop()
        }
    }

    func testRecoveryDiagnosticsClearAtActualFollowingHoldingAndClearancePhases() async throws {
        for range in [0.6, 1.5, 4.0] {
            let perception = FollowPerceptionFake()
            let motion = AbsoluteRecoveryMotionFake()
            let clock = ManualFollowClock()
            motion.legacy.useSourceClock(clock)
            var config = FollowMeConfiguration()
            config.stationaryPauseSeconds = 0
            if range != 0.6 { config.departureRangeIncrease = 0 }
            let sink = FollowDiagnosticRecordingSink()
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                configuration: config, eventSink: sink.append)
            _ = await coordinator.start()
            perception.send(frame(1, people: [person(1, x: range, y: 0)]))
            await drain()
            perception.send(frame(2))
            await drain()
            perception.send(frame(3, people: [person(3, x: range, y: 0)]))
            await drain()
            XCTAssertFalse(sink.records.contains { $0.event == "follow_recovery.cleared" })
            await matchedBoundaryFrame(4, perception, clock, yaw: 0, x: range, y: 0)
            if range == 0.6 { await matchedBoundaryFrame(5, perception, clock, yaw: 0, x: range, y: 0) }
            let expected = range == 0.6 ? "waitingForClearance" : (range == 1.5 ? "holdingDistance" : "following")
            let cleared = try XCTUnwrap(sink.records.first { $0.event == "follow_recovery.cleared" })
            let json = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap(cleared.fields["payload"]).utf8)) as? [String: Any])
            XCTAssertEqual(json["phase"] as? String, expected)
            XCTAssertEqual(json["restored_frame_id"] as? String, range == 0.6 ? "1:5" : "1:4")
            XCTAssertEqual(json["anchor_frame_id"] as? String, "1:1")
            XCTAssertEqual(json["reliable_frame_id"] as? String, range == 0.6 ? "1:5" : "1:4")
            XCTAssertEqual(json["stop_outcome"] as? String, "confirmed")
            _ = await coordinator.stop()
        }
    }

    func testRecoveryDiagnosticsPreserveFirstLossStopFailureOverDeadlineCleanup() async throws {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        let sink = FollowDiagnosticRecordingSink()
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: sink.append)
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        motion.legacy.stopError = true
        perception.send(frame(2))
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Rover stop could not be confirmed."))
        let stop = try XCTUnwrap(sink.records.last { $0.event == "follow_recovery.stop_response" })
        let json = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap(stop.fields["payload"]).utf8)) as? [String: Any])
        XCTAssertEqual(json["stop_outcome"] as? String, "failed")
        XCTAssertEqual(json["reason"] as? String, "stop_not_confirmed")
        XCTAssertEqual(json["anchor_frame_id"] as? String, "1:1")
        XCTAssertEqual(json["stale"] as? Bool, false)
        clock.advance(to: 10)
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Rover stop could not be confirmed."))
        XCTAssertFalse(sink.records.contains { $0.event == "follow_recovery.expired" || $0.event == "follow_recovery.cleared" })
        XCTAssertTrue(motion.headings.isEmpty)
        motion.legacy.stopError = false
        _ = await coordinator.stop()
    }

    func testRecoveryDiagnosticsFenceLifecycleAndIgnoreLateOldCompletionAcrossEpisodes() async throws {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        let gate = FollowDiagnosticSuspension()
        motion.recoveryGates[1] = gate
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        let sink = FollowDiagnosticRecordingSink()
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: sink.append)
        func records(_ name: String) throws -> [[String: Any]] {
            try sink.records.filter { $0.event == name }.map {
                try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap($0.fields["payload"]).utf8)) as? [String: Any])
            }
        }
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        perception.send(frame(2))
        await gate.waitUntilEntered()
        coordinator.inhibitMotion()
        _ = await coordinator.stop()
        let fenced = try XCTUnwrap(records("follow_recovery.fenced").first)
        XCTAssertEqual(fenced["reason"] as? String, "lifecycle_inhibition")
        XCTAssertEqual(fenced["stop_outcome"] as? String, "pending")
        let terminal = try XCTUnwrap(records("follow_recovery.terminated").first)
        XCTAssertEqual(terminal["episode_id"] as? String, fenced["episode_id"] as? String)
        XCTAssertEqual(terminal["stop_outcome"] as? String, "confirmed")
        _ = await coordinator.start()
        perception.send(frame(10, people: [person(10)]))
        await drain()
        perception.send(frame(11))
        await drain()
        let started = try records("follow_recovery.started")
        XCTAssertEqual(started.count, 2)
        XCTAssertNotEqual(started[0]["episode_id"] as? String, started[1]["episode_id"] as? String)
        let before = sink.records.count
        gate.release()
        await drain()
        XCTAssertEqual(sink.records.count, before, "Old completion cannot describe or clear current recovery")
        XCTAssertTrue(try records("follow_recovery.cleared").isEmpty)
        let phase = try XCTUnwrap(records("follow_phase").last)
        XCTAssertEqual(phase["recovery_active"] as? Bool, true)
        XCTAssertEqual(phase["episode_id"] as? String, started[1]["episode_id"] as? String)
        _ = await coordinator.stop()
    }

    func testRecoveryDiagnosticsReportExhaustionExpiryRestorationAndFailedStop() async throws {
        for ending in ["expired", "cleared", "failed_stop", "stopped"] {
            let perception = FollowPerceptionFake()
            let motion = AbsoluteRecoveryMotionFake()
            motion.autoArrive = true
            let clock = ManualFollowClock()
            motion.legacy.useSourceClock(clock)
            var config = FollowMeConfiguration()
            config.scanObservationSeconds = 0 // This matrix isolates episode/diagnostic boundaries.
            config.scanIncrement = .pi / 6
            config.stationaryPauseSeconds = 0
            let sink = FollowDiagnosticRecordingSink()
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                configuration: config, eventSink: sink.append)
            _ = await coordinator.start()
            perception.send(frame(1, people: [person(1, x: 4, y: 0)]))
            await drain()
            perception.send(frame(2))
            await drain()
            func records(_ name: String) throws -> [[String: Any]] {
                try sink.records.filter { $0.event == name }.map {
                    try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap($0.fields["payload"]).utf8)) as? [String: Any])
                }
            }
            let completed = try records("follow_recovery.segment_completed")
            XCTAssertFalse(completed.isEmpty)
            XCTAssertEqual(completed.first?["reason"] as? String, "stage_skipped_within_tolerance")
            XCTAssertEqual(completed.first?["stage_index"] as? Int, 0, "Completion describes the stage actually evaluated")
            XCTAssertEqual(completed.first?["next_stage_index"] as? Int, 1)
            XCTAssertEqual(completed.first?["requested_movement_rad"] as? Double, 0)
            XCTAssertEqual(completed.first?["measured_movement_rad"] as? Double, 0)
            XCTAssertEqual(completed.first?["requested_movement_rad_availability"] as? String, "available")
            XCTAssertEqual(completed.first?["measured_movement_rad_availability"] as? String, "available")
            let exhausted = try XCTUnwrap(records("follow_recovery.exhausted").first)
            XCTAssertEqual(exhausted["stage_index"] as? Int, 7)
            XCTAssertEqual(exhausted["pass_exhausted"] as? Bool, true)
            XCTAssertTrue(exhausted["stage_target_rad"] is NSNull)
            let count = motion.headings.count
            if ending == "cleared" {
                perception.send(frame(3, people: [person(3, x: 4, y: 0)]))
                await drain()
                await matchedBoundaryFrame(4, perception, clock, yaw: 0, x: 4, y: 0)
                await matchedBoundaryFrame(5, perception, clock, yaw: 0, x: 4, y: 0)
                let signaling = try XCTUnwrap(records("follow_phase").last)
                XCTAssertEqual(signaling["phase"] as? String, "signalingReady")
                XCTAssertEqual(signaling["recovery_active"] as? Bool, true)
                XCTAssertEqual(signaling["deadline_s"] as? Double, 10)
                let ready = try XCTUnwrap(records("follow_ready.completed").last)
                XCTAssertEqual(ready["deadline_s"] as? Double, 10)
                XCTAssertEqual(ready["recovery_active"] as? Bool, true)
                await matchedBoundaryFrame(6, perception, clock, yaw: 0, x: 4, y: 0)
                XCTAssertEqual(coordinator.state, .waitingForMovement)
                let cleared = try XCTUnwrap(records("follow_recovery.cleared").first)
                XCTAssertEqual(cleared["restored_frame_id"] as? String, "1:6")
                XCTAssertEqual(cleared["phase"] as? String, "waitingForMovement")
                XCTAssertEqual(cleared["anchor_frame_id"] as? String, "1:1")
                XCTAssertEqual(cleared["stop_outcome"] as? String, "confirmed")
                XCTAssertEqual(cleared["recovery_active"] as? Bool, false)
                XCTAssertEqual(cleared["deadline_active"] as? Bool, false)
                XCTAssertTrue(cleared["provisional_frame_id"] is NSNull)
                await matchedBoundaryFrame(7, perception, clock, yaw: 0, x: 4, y: 0)
                XCTAssertEqual(try records("follow_recovery.cleared").count, 1)
            } else if ending == "expired" {
                clock.advance(to: 10, wakeSleepers: false)
                perception.send(frame(3, at: 10))
                await drain()
                let expired = try XCTUnwrap(records("follow_recovery.expired").first)
                XCTAssertEqual(expired["remaining_s"] as? Double, 0)
                XCTAssertEqual(expired["deadline_s"] as? Double, 10)
                XCTAssertEqual(motion.headings.count, count)
            } else {
                motion.legacy.stopError = ending == "failed_stop"
                _ = await coordinator.stop()
                let terminal = try XCTUnwrap(records("follow_recovery.terminated").first)
                XCTAssertEqual(terminal["stop_outcome"] as? String, ending == "failed_stop" ? "failed" : "confirmed")
                XCTAssertEqual(terminal["reason"] as? String, ending == "failed_stop" ? "stop_not_confirmed" : "local_stop")
            }
            _ = await coordinator.stop()
        }
    }

    func testRecoveryDiagnosticsFreezePairedMemoryAndSelectCenterExactlyOnce() async throws {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        let sink = FollowDiagnosticRecordingSink()
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: sink.append)
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        motion.legacy.suspendStop = true
        perception.send(frame(2))
        await drain()
        func records(_ name: String) throws -> [[String: Any]] {
            try sink.records.filter { $0.event == name }.map {
                try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap($0.fields["payload"]).utf8)) as? [String: Any])
            }
        }
        let frozen = try XCTUnwrap(records("follow_recovery.started").first)
        XCTAssertEqual(frozen["anchor_person_z"] as? Double, 4)
        XCTAssertEqual(frozen["anchor_timestamp_s"] as? Double, 0)
        XCTAssertEqual(frozen["anchor_bearing_rad"] as? Double, .pi / 2)
        XCTAssertEqual(frozen["stop_outcome"] as? String, "pending")
        XCTAssertTrue(frozen["anchor_raw_person_id"] is NSNull)
        XCTAssertTrue(frozen["center_heading_rad"] is NSNull)
        clock.advance(to: 0.2, wakeSleepers: false)
        motion.sample = .init(pose: Pose2D(position: Vec2(1, 0), yaw: 0.3),
            frameID: ARFrameID(generation: 1, sequence: 101), sourceTimestamp: 0.2,
            trackingQuality: .normal, source: "synthetic")
        motion.legacy.suspendStop = false
        motion.legacy.releaseStop()
        await drain()
        let selected = try XCTUnwrap(records("follow_recovery.center_selected").first)
        XCTAssertEqual(selected["episode_id"] as? String, frozen["episode_id"] as? String)
        XCTAssertEqual(selected["deadline_s"] as? Double, 10)
        XCTAssertEqual(selected["remaining_s"] as? Double, 9.8)
        XCTAssertEqual(selected["center_source"] as? String, "world_from_post_stop_pose")
        XCTAssertEqual(try XCTUnwrap(selected["initial_return_delta_rad"] as? Double), 1.515774989921761, accuracy: 1e-12)
        XCTAssertEqual(selected["center_source_frame_id"] as? String, "1:101")
        XCTAssertEqual(selected["center_source_identity"] as? String, "synthetic")
        XCTAssertEqual(selected["center_initial_yaw_rad"] as? Double, 0.3)
        perception.send(frame(3, at: 0.2, people: [person(3, at: 0.2, y: 4.2)]))
        await drain()
        perception.send(frame(4, at: 0.2))
        await drain()
        XCTAssertEqual(try records("follow_recovery.center_selected").count, 1)
        let retained = try XCTUnwrap(records("follow_recovery.retained").last)
        XCTAssertEqual(retained["anchor_frame_id"] as? String, "1:1")
        XCTAssertEqual(retained["deadline_s"] as? Double, 10)
        XCTAssertEqual(retained["stage_index"] as? Int, 0)
        _ = await coordinator.stop()
    }

    func testProvisionalAlignmentLossKeepsOriginalEpisodeAndDeadline() async {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        var anchors: [String] = []
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config) { event, fields in
            if event == "follow_recovery.started", let payload = fields["payload"],
               let record = try? JSONSerialization.jsonObject(with: Data(payload.utf8)) as? [String: Any],
               let anchor = record["anchor_frame_id"] as? String { anchors.append(anchor) }
        }
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        perception.send(frame(2))
        await drain()
        motion.legacy.suspendAlignment = true
        perception.send(frame(3, people: [person(3, y: 4.2)]))
        await drain()
        XCTAssertEqual(coordinator.state, .aligning)
        perception.send(frame(4))
        await drain()
        motion.legacy.releaseAlignment()
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring)
        XCTAssertEqual(anchors, ["1:1"])
        XCTAssertEqual(motion.headings, [.pi / 2, .pi / 2], "Interrupted recovery resumes the frozen stage")
        clock.advance(to: 10, wakeSleepers: false)
        perception.send(frame(5, at: 10, people: [person(5, at: 10)]))
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Person lost."))
        _ = await coordinator.stop()
    }

    func testUnusedRecoveryReadyStaysBoundUntilDistinctBaselineAndNeverRetries() async {
        for completeBaseline in [false, true] {
            let (coordinator, perception, motion, clock) = sourcedRecoverySetup()
            _ = await coordinator.start()
            await drain()
            clock.advance(to: 5)
            perception.send(frame(1, at: 5, people: [person(1, at: 5)]))
            await drain()
            perception.send(frame(2, at: 5))
            await drain()
            perception.send(frame(3, at: 5, pose: Pose2D(position: .zero, yaw: .pi / 2), people: [person(3, at: 5)]))
            await drain()
            await matchedBoundaryFrame(4, perception, clock)
            await matchedBoundaryFrame(5, perception, clock)
            XCTAssertEqual(coordinator.state, .signalingReady, "A successful move is still baseline-pending recovery")
            XCTAssertEqual(motion.readySignals, 1)
            if completeBaseline {
                await matchedBoundaryFrame(6, perception, clock)
                XCTAssertEqual(coordinator.state, .waitingForMovement)
            }
            clock.advance(to: 15, wakeSleepers: false)
            perception.send(frame(7, at: 15, pose: Pose2D(position: .zero, yaw: .pi / 2), people: [person(7, at: 15)]))
            await drain()
            XCTAssertEqual(coordinator.state, completeBaseline ? .waitingForMovement : .failed("Person lost."))
            XCTAssertEqual(motion.readySignals, 1)
            _ = await coordinator.stop()
        }
    }

    func testRecoveryAlignmentInclusivePointZeroFiveGateNeedsNewFrameAndRejectsOutsideGate() async {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        let clock = ManualFollowClock()
        motion.legacy.useSourceClock(clock)
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config)
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, x: 0.6, y: 0)]))
        await drain()
        perception.send(frame(2))
        await drain()
        perception.send(frame(3, people: [person(3, x: 0.6, y: 0)]))
        await drain()
        XCTAssertEqual(coordinator.state, .aligning)
        await matchedBoundaryFrame(4, perception, clock, yaw: 0, x: 0.6, y: 0)
        await matchedBoundaryFrame(5, perception, clock, yaw: -0.050001, x: 0.6, y: 0)
        XCTAssertEqual(coordinator.state, .aligning)
        await matchedBoundaryFrame(6, perception, clock, yaw: -0.050001, x: 0.6, y: 0)
        await matchedBoundaryFrame(7, perception, clock, yaw: -0.05, x: 0.6, y: 0)
        XCTAssertEqual(coordinator.state, .waitingForClearance)
        XCTAssertEqual(motion.legacy.readySignals, 0)
        _ = await coordinator.stop()
    }

    func testRecoveryFinalAlignmentAndReadyStopsEnforceDeadlineWithoutTimerDelivery() async {
        for ready in [false, true] {
            let (coordinator, perception, motion, clock) = sourcedRecoverySetup()
            _ = await coordinator.start()
            await drain()
            clock.advance(to: 5)
            perception.send(frame(1, at: 5, people: [person(1, at: 5)]))
            await drain()
            perception.send(frame(2, at: 5))
            await drain()
            motion.suspendAlignment = !ready
            motion.suspendReadySignal = ready
            perception.send(frame(3, at: 5, pose: Pose2D(position: .zero, yaw: .pi / 2), people: [person(3, at: 5)]))
            await drain()
            await matchedBoundaryFrame(4, perception, clock)
            if ready {
                await matchedBoundaryFrame(5, perception, clock)
                XCTAssertEqual(coordinator.state, .signalingReady)
            }
            motion.suspendStop = true
            if ready { motion.releaseReadySignal() } else { motion.releaseAlignment() }
            await drain()
            clock.advance(to: 15, wakeSleepers: false)
            motion.suspendStop = false
            motion.releaseStop()
            await drain()
            XCTAssertEqual(coordinator.state, .failed("Person lost."), "Final stop callbacks enforce the original episode deadline before recording success")
            _ = await coordinator.stop()
        }
    }

    func testRecoveryDetectionStopRevalidatesNewestAssociationAndExcludesConsumedStopFrame() async {
        for conflict in [false, true] {
            let (coordinator, perception, motion, clock) = sourcedRecoverySetup()
            await acquireWaiting(coordinator, perception, clock)
            perception.send(frame(4, at: clock.now))
            await drain()
            let alignments = motion.alignments.count
            motion.suspendStop = true
            perception.send(frame(5, at: clock.now, pose: Pose2D(position: .zero, yaw: .pi / 2), people: [person(5, at: clock.now)]))
            await drain()
            perception.send(frame(6, at: clock.now, pose: Pose2D(position: .zero, yaw: .pi / 2),
                people: conflict ? [person(6, at: clock.now), person(6, at: clock.now, y: 4.1)] : [person(6, at: clock.now)]))
            await drain()
            motion.suspendStop = false
            motion.releaseStop()
            await drain()
            if conflict {
                XCTAssertEqual(coordinator.state, .reacquiring)
                XCTAssertEqual(motion.alignments.count, alignments, "Conflicting pending targets cannot authorize alignment after scan stop")
            } else {
                XCTAssertEqual(coordinator.state, .reacquiring, "Frame consumed before stop confirmation cannot restore normal control")
                await matchedBoundaryFrame(7, perception, clock)
                XCTAssertEqual(coordinator.state, .waitingForMovement, "Completed readiness needs fresh continuity, not another alignment")
                await matchedBoundaryFrame(8, perception, clock)
                XCTAssertEqual(coordinator.state, .waitingForMovement)
            }
            _ = await coordinator.stop()
        }
    }

    func testBeforeDepartureRestorationClearsOnlyAtAlignedClearanceOrRetainedBaselinePhase() async {
        for tooClose in [false, true] {
            let perception = FollowPerceptionFake()
            let motion = AbsoluteRecoveryMotionFake()
            let clock = ManualFollowClock()
            motion.legacy.useSourceClock(clock)
            motion.sample = .init(pose: Pose2D(position: .zero, yaw: .pi / 2),
                frameID: ARFrameID(generation: 1, sequence: 100), sourceTimestamp: 5,
                trackingQuality: .normal, source: "synthetic")
            var anchors: [String] = []
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock) { event, fields in
                if event == "follow_recovery.started", let payload = fields["payload"],
                   let record = try? JSONSerialization.jsonObject(with: Data(payload.utf8)) as? [String: Any],
                   let anchor = record["anchor_frame_id"] as? String { anchors.append(anchor) }
            }
            if !tooClose { await acquireWaiting(coordinator, perception, clock) }
            else {
                _ = await coordinator.start()
                await drain()
                clock.advance(to: 5)
                perception.send(frame(0, at: 5, people: [person(0, at: 5, y: 0.6)]))
                await drain()
                await matchedBoundaryFrame(1, perception, clock, yaw: 0, y: 0.6)
                await matchedBoundaryFrame(2, perception, clock, y: 0.6)
                XCTAssertEqual(coordinator.state, .waitingForClearance)
            }
            let oldAnchor = tooClose ? "1:2" : "1:3"
            perception.send(frame(4, at: clock.now))
            await drain()
            let range = tooClose ? 0.6 : 4.2
            perception.send(frame(5, at: clock.now, pose: Pose2D(position: .zero, yaw: .pi / 2), people: [person(5, at: clock.now, y: range)]))
            await drain()
            XCTAssertEqual(coordinator.state, tooClose ? .aligning : .reacquiring)
            await matchedBoundaryFrame(6, perception, clock, y: range)
            await matchedBoundaryFrame(7, perception, clock, y: range)
            XCTAssertEqual(coordinator.state, tooClose ? .waitingForClearance : .waitingForMovement)
            XCTAssertEqual(motion.legacy.readySignals, tooClose ? 0 : 1)
            perception.send(frame(8, at: clock.now))
            await drain()
            XCTAssertEqual(anchors, [oldAnchor, "1:7"])
            _ = await coordinator.stop()
        }
    }

    func testPostDepartureRecoveryNeedsDistinctContinuityThenNewLossFreezesRestoredMemory() async {
        for range in [1.5, 4.0] {
            let perception = FollowPerceptionFake()
            let motion = AbsoluteRecoveryMotionFake()
            motion.legacy.suspendRotation = true
            let clock = ManualFollowClock()
            var config = FollowMeConfiguration()
            config.stationaryPauseSeconds = 0
            config.departureRangeIncrease = 0
            var anchors: [String] = []
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config) { event, fields in
                if event == "follow_recovery.started", let payload = fields["payload"],
                   let record = try? JSONSerialization.jsonObject(with: Data(payload.utf8)) as? [String: Any],
                   let anchor = record["anchor_frame_id"] as? String { anchors.append(anchor) }
            }
            _ = await coordinator.start()
            perception.send(frame(1, people: [person(1, y: range)]))
            await drain()
            perception.send(frame(2))
            await drain()
            perception.send(frame(3, people: [person(3, y: range + 0.1)]))
            await drain()
            XCTAssertEqual(coordinator.state, .reacquiring, "Provisional detection is not normal continuity")
            perception.send(frame(4, people: [person(4, y: range + 0.2)]))
            await drain()
            XCTAssertEqual(coordinator.state, range == 1.5 ? .holdingDistance : .following)
            perception.send(frame(5))
            await drain()
            XCTAssertEqual(anchors, ["1:1", "1:4"], "Actual normal restoration commits the new paired memory and ends the old episode")
            _ = await coordinator.stop()
            motion.legacy.releaseRotation()
        }
    }

    func testRecoveryCommandSuccessWithoutAuthoritativeArrivalCannotAdvanceStage() async {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        motion.resultWithoutEvidence = .arrived
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        config.scanObservationSeconds = 1
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config)
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        perception.send(frame(2))
        await drain()
        for (index, time) in [0.3, 0.6, 0.9, 1.01].enumerated() {
            clock.advance(to: time)
            perception.send(frame(UInt64(index + 3), at: time))
            await drain()
        }
        XCTAssertEqual(motion.headings, [.pi / 2, .pi / 2], "Success labels cannot advance the measured stage cursor")
        _ = await coordinator.stop()
    }

    func testRejectedObservationsNeverEraseLastValidMemory() async {
        for rejection in ["confidence", "jump", "ambiguous", "stale", "future", "unhealthy", "frame"] {
            let perception = FollowPerceptionFake()
            let motion = FollowMotionFake()
            let clock = ManualFollowClock()
            var config = FollowMeConfiguration()
            config.stationaryPauseSeconds = 0
            config.departureRangeIncrease = 0
            var anchor: String?
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config) { event, fields in
                if event == "follow_recovery.started", let payload = fields["payload"],
                   let record = try? JSONSerialization.jsonObject(with: Data(payload.utf8)) as? [String: Any] {
                    anchor = record["anchor_frame_id"] as? String
                }
            }
            _ = await coordinator.start()
            perception.send(frame(1, people: [person(1, y: 1.5)]))
            await drain()
            let candidate = FollowPersonObservation(frameID: ARFrameID(generation: 1, sequence: rejection == "frame" ? 99 : 2),
                timestamp: rejection == "stale" ? -0.501 : rejection == "future" ? 0.001 : 0,
                confidence: rejection == "confidence" ? 0.49 : 0.9,
                boundingBox: CGRect(x: 0.45, y: 0.4, width: 0.1, height: 0.2),
                position: Vec2(0, rejection == "jump" ? 5 : 1.6), pose: Pose2D(position: .zero, yaw: 0))
            perception.send(frame(2, people: rejection == "ambiguous" ? [candidate, person(2, y: 1.7)] : [candidate],
                quality: rejection == "unhealthy" ? .limited : .normal))
            await drain()
            perception.send(frame(3))
            await drain()
            XCTAssertEqual(anchor, "1:1", rejection)
            _ = await coordinator.stop()
        }
    }

    func testRealRecoveryWatchdogFailureRemainsTerminalInsteadOfRetryingNextArc() async {
        let perception = FollowPerceptionFake()
        let clock = ManualFollowClock()
        let source = CoordinatorTurnSource(clock: clock)
        var controllerTime = Date(timeIntervalSince1970: 100)
        var sends = 0
        let controller = NavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { controllerTime },
            sendCommand: { _ in sends += 1 }, stopRover: {}, sleep: { duration in
                for _ in 0..<5 { await Task.yield() }
                guard !Task.isCancelled else { return }
                controllerTime = controllerTime.addingTimeInterval(0.5)
                source.advance(by: duration.secondsValue)
            }, now: { controllerTime }, poseSample: {
                source.sample
            }, sourceNow: { clock.now }, sourceStopSnapshot: { source.sample })
        source.onCapture = { sample in
            controller.ingestFollowTurnSource(sample)
            perception.send(self.frame(sample.frameID!.sequence, at: sample.sourceTimestamp!))
        }
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: NavigationFollowMeMotion(navigation: controller),
            clock: clock, configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        perception.send(frame(2))
        for _ in 0..<20 { await drain() }
        XCTAssertGreaterThan(sends, 0, "Reach measured-progress failure rather than missing-source preflight")
        XCTAssertEqual(coordinator.state, .failed("Search rotation stopped: insufficient measured yaw progress. Stop confirmed. Restart following to try again."))
        let count = sends
        clock.advance(to: 10)
        perception.send(frame(source.sequence + 1, at: 10))
        await drain()
        XCTAssertEqual(sends, count)
        _ = await coordinator.stop()
    }

    func testOldRecoveryCompletionAndTimerCannotDeleteNewSessionsScan() async {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        motion.autoArrive = true
        let oldGate = FollowDiagnosticSuspension()
        let newGate = FollowDiagnosticSuspension()
        motion.recoveryGates = [1: oldGate, 2: newGate]
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config)
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        perception.send(frame(2))
        await oldGate.waitUntilEntered()
        _ = await coordinator.stop()
        clock.advance(to: 1)
        motion.sample = .init(pose: Pose2D(position: .zero, yaw: 0), frameID: ARFrameID(generation: 1, sequence: 100),
            sourceTimestamp: 1, trackingQuality: .normal, source: "synthetic")
        _ = await coordinator.start()
        perception.send(frame(1, at: 1, people: [person(1, at: 1, x: 1.5, y: 0)]))
        await drain()
        perception.send(frame(2, at: 1))
        await newGate.waitUntilEntered()
        XCTAssertEqual(motion.headings, [.pi / 2, 0])
        XCTAssertEqual(Set(motion.authorizations.map(\.episodeID)).count, 2)
        oldGate.release()
        await drain()
        for step in 1...23 {
            let time = min(10, 1 + Double(step) * 0.4)
            clock.advance(to: time)
            perception.send(frame(UInt64(step + 2), at: time))
            await drain()
        }
        XCTAssertEqual(coordinator.state, .reacquiring, "Old ten-second timer cannot expire the new eleven-second episode")
        XCTAssertEqual(motion.headings.count, 2, "Old callback cannot clear the newer task/scan flag")
        _ = await coordinator.stop()
        newGate.release()
        await drain()
    }

    func testRecoveryStopCompletionAtOriginalDeadlineCannotLaunchForwardGoal() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        perception.send(frame(2))
        await drain()
        for step in 1...24 {
            let time = Double(step) * 0.4
            clock.advance(to: time)
            perception.send(frame(UInt64(step + 2), at: time))
            await drain()
        }
        motion.suspendStop = true
        perception.send(frame(30, at: 9.6, people: [person(30, at: 9.6, y: 2)]))
        await drain()
        clock.advance(to: 10, wakeSleepers: false)
        motion.suspendStop = false
        motion.releaseStop()
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Person lost."))
        XCTAssertTrue(motion.goals.isEmpty)
        _ = await coordinator.stop()
    }

    func testRealControllerRecoveryUsesNegativeWorldHeadingAndMeasuredOvershootThroughOnePass() async {
        let perception = FollowPerceptionFake()
        let clock = ManualFollowClock()
        let source = CoordinatorTurnSource(clock: clock, position: Vec2(0, 1))
        var yaw = 0.0
        var headings: [Double] = []
        var commands: [Double] = []
        let controller = NavigationController(currentPose: { Pose2D(position: Vec2(0, 1), yaw: yaw) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: {
                if let heading = FollowRecoveryScope.heading?.stageHeading, headings.last != heading { headings.append(heading) }
                return Date()
            }, sendCommand: { command in
                commands.append(command.right)
                // A real controller must correct this negative overshoot using a positive pulse.
                if commands.count == 1 { yaw = -0.7 }
                else { yaw = FollowMotionTaskScope.evidence!.targetYaw! }
                source.yaw = yaw
                source.advance(by: 0.001) // Actual sourced crossing while the command response is pending.
            }, stopRover: {}, sleep: { duration in
                for _ in 0..<5 { await Task.yield() }
                guard !Task.isCancelled else { return }
                source.advance(by: duration.secondsValue, captureLag: 0.020)
            },
            poseSample: { source.sample }, sourceNow: { clock.now }, sourceStopSnapshot: { source.sample })
        source.onCapture = { sample in
            controller.ingestFollowTurnSource(sample)
            perception.send(self.frame(sample.frameID!.sequence, at: sample.sourceTimestamp!))
        }
        var config = FollowMeConfiguration()
        config.scanObservationSeconds = 0
        config.scanIncrement = .pi / 6
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: NavigationFollowMeMotion(navigation: controller),
            clock: clock, configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: -1.5)]))
        await drain()
        perception.send(frame(2))
        for _ in 0..<30 { await drain() }
        XCTAssertEqual(headings.map { Int(($0 * 180 / .pi).rounded()) }, [-90, -75, -105, -60, -120, -45, -135])
        XCTAssertEqual(Array(commands.prefix(2)), [-0.25, 0.25])
        XCTAssertLessThanOrEqual(abs(normalizeAngle(-135 * .pi / 180 - yaw)), 7 * .pi / 180)
        let count = commands.count
        perception.send(frame(source.sequence + 1, at: clock.now))
        await drain()
        XCTAssertEqual(commands.count, count)
        _ = await coordinator.stop()
    }

    func testGenerationReplacementFencesRecoveryBeforeOldResultCanAdvance() async {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        motion.autoArrive = true
        let gate = FollowDiagnosticSuspension()
        motion.recoveryGates = [1: gate]
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config)
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        perception.send(frame(2))
        await gate.waitUntilEntered()
        perception.send(.frame(.init(frameID: ARFrameID(generation: 2, sequence: 1), timestamp: 0,
            pose: Pose2D(position: .zero, yaw: 0), depthAvailable: true, people: [], trackingQuality: .normal)))
        await drain()
        XCTAssertFalse(coordinator.isActive, "An AR generation change terminates historical recovery")
        gate.release()
        await drain()
        XCTAssertEqual(motion.headings.count, 1)
        _ = await coordinator.stop()
    }

    func testInitialSelectionCannotAuthorizeCandidateFromAnotherBatch() async {
        let (coordinator, perception, motion, _) = setup()
        motion.suspendRotation = true
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(99)]))
        await drain()
        XCTAssertTrue(motion.goals.isEmpty, "Initial matched memory and authority require batch pairing")
        XCTAssertEqual(coordinator.state, .searching, "Unpaired candidate cannot become the selected lock")
        _ = await coordinator.stop()
        motion.releaseRotation()
    }

    func testInterruptedStageResumesWithoutRecenteringAndOldCompletionCannotClearNewScan() async {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        motion.autoArrive = true
        let oldGate = FollowDiagnosticSuspension()
        let newGate = FollowDiagnosticSuspension()
        motion.recoveryGates = [4: oldGate, 5: newGate]
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.scanObservationSeconds = 0
        config.scanIncrement = .pi / 6
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config)
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        perception.send(frame(2))
        await oldGate.waitUntilEntered()
        perception.send(frame(3, people: [person(3, x: 1, y: 1.5)]))
        await drain()
        perception.send(frame(4))
        await newGate.waitUntilEntered()
        XCTAssertEqual(motion.headings.last ?? 0, 105 * .pi / 180, accuracy: 1e-12)
        XCTAssertEqual(Set(motion.authorizations.map(\.episodeID)).count, 1)
        XCTAssertEqual(Set(motion.authorizations.map(\.deadline)), [10])
        oldGate.release()
        await drain()
        perception.send(frame(5))
        await drain()
        XCTAssertEqual(motion.headings.count, 5, "Old completion cannot clear the newer scan flag/task or advance its cursor")
        _ = await coordinator.stop()
        newGate.release()
        await drain()
    }

    func testRejectedClippedContinuationCannotShadowReliablePairedMemory() async throws {
        let perception = FollowPerceptionFake()
        let motion = FollowMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        var anchor: String?
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config) { event, fields in
            if event == "follow_recovery.started", let payload = fields["payload"],
               let record = try? JSONSerialization.jsonObject(with: Data(payload.utf8)) as? [String: Any] {
                anchor = record["anchor_frame_id"] as? String
            }
        }
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        let clipped = FollowPersonObservation(frameID: ARFrameID(generation: 1, sequence: 2), timestamp: 0,
            confidence: 0.9, boundingBox: CGRect(x: 0, y: 0.4, width: 0.5, height: 0.2), position: Vec2(0, 1.6),
            pose: Pose2D(position: Vec2(0.1, 0.2), yaw: -0.5))
        perception.send(frame(2, people: [clipped]))
        await drain()
        perception.send(frame(3))
        await drain()
        XCTAssertEqual(anchor, "1:1", "Rejected clipped geometry cannot become reliable memory")
        _ = await coordinator.stop()
    }

    func testRecoveryVisitsFixedOffsetsOnceUsingMeasuredSegmentsAndOvershoot() async {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        motion.autoArrive = true
        motion.overshootAt = 2
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.scanObservationSeconds = 0
        config.scanIncrement = .pi / 6
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config)
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        perception.send(frame(2))
        for _ in 0..<20 { await drain() }
        var stages: [Double] = []
        for heading in motion.headings where stages.last != heading { stages.append(heading) }
        XCTAssertEqual(stages.map { Int(($0 * 180 / .pi).rounded()) }, [90, 105, 75, 120, 60, 135, 45])
        XCTAssertTrue(motion.deltas.allSatisfy { abs($0) <= .pi / 6 + 1e-12 })
        XCTAssertTrue(motion.deltas.contains { $0 < 0 }, "Actual positive overshoot requires a negative correction")
        let requests = motion.headings.count
        perception.send(frame(3))
        await drain()
        XCTAssertEqual(motion.headings.count, requests, "Exhausted pass observes stationary")
        XCTAssertTrue(motion.legacy.goals.isEmpty, "Remembered geometry never creates a forward goal")
        _ = await coordinator.stop()
    }

    func testOriginalLossDeadlineIncludesPendingStopAndFreshDepartureHandoff() async {
        for pendingStop in [true, false] {
            let (coordinator, perception, motion, clock) = productionSetup()
            await acquireWaiting(coordinator, perception, clock)
            motion.suspendStop = pendingStop
            let firstLoss = clock.now
            perception.send(frame(4, at: firstLoss))
            await drain()
            if pendingStop {
                clock.advance(to: firstLoss + 10)
                await drain()
                XCTAssertEqual(coordinator.state, .failed("Person lost."), "Stop latency consumes the original budget")
                motion.suspendStop = false
                motion.releaseStop()
                await drain()
            } else {
                for step in 1...24 {
                    let time = firstLoss + Double(step) * 0.4
                    clock.advance(to: time)
                    perception.send(frame(UInt64(step + 4), at: time))
                    await drain()
                }
                perception.send(frame(30, at: clock.now, people: [person(30, at: clock.now)]))
                await drain()
                XCTAssertEqual(coordinator.state, .reacquiring)
                clock.advance(to: firstLoss + 10, wakeSleepers: false)
                perception.send(frame(31, at: clock.now, people: [person(31, at: clock.now)]))
                await drain()
                XCTAssertEqual(coordinator.state, .failed("Person lost."), "A frame exactly at expiry cannot rescue the departure handoff")
            }
            _ = await coordinator.stop()
        }
    }

    func testRecoveryRejectsUnpairedCandidateBeforeSpatialAcceptance() async {
        for candidateFrame in [ARFrameID(generation: 2, sequence: 3), ARFrameID(generation: 1, sequence: 99)] {
            let (coordinator, perception, _, _) = setup()
            _ = await coordinator.start()
            perception.send(frame(1, people: [person(1)]))
            await drain()
            perception.send(frame(2))
            await drain()
            let candidate = FollowPersonObservation(frameID: candidateFrame, timestamp: 0, confidence: 0.9,
                boundingBox: CGRect(x: 0.45, y: 0.4, width: 0.1, height: 0.2), position: Vec2(0, 4), pose: Pose2D(position: .zero, yaw: 0))
            perception.send(frame(3, people: [candidate]))
            await drain()
            XCTAssertEqual(coordinator.state, .reacquiring, "Candidate must belong to this batch and frozen generation")
            _ = await coordinator.stop()
        }
    }

    func testProvisionalReacquisitionCannotMoveFrozenSpatialAnchor() async {
        let (coordinator, perception, _, _) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 4)]))
        await drain()
        perception.send(frame(2))
        await drain()
        perception.send(frame(3, people: [person(3, y: 5.4)]))
        await drain()
        perception.send(frame(4))
        await drain()
        perception.send(frame(5, people: [person(5, y: 6.7)]))
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring, "1.5 m gate remains around the reliable 4 m anchor")
        _ = await coordinator.stop()
    }

    func testRecoveryReturnsToLatestMatchedWorldPointFromActualPostStopPosition() async {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config)
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        perception.send(frame(2, people: [person(2, y: 4.5)]))
        await drain()
        motion.legacy.suspendStop = true
        perception.send(frame(3))
        await drain()
        motion.sample = .init(pose: Pose2D(position: Vec2(1, 1), yaw: -0.4),
            frameID: ARFrameID(generation: 1, sequence: 30), sourceTimestamp: 0, trackingQuality: .normal, source: "synthetic")
        motion.legacy.suspendStop = false
        motion.legacy.releaseStop()
        await drain()
        XCTAssertEqual(motion.headings, [atan2(3.5, -1)], "Use latest accepted world point and real post-stop position")
        _ = await coordinator.stop()
    }

    func testLegacyRecoveryObservesStationaryWithoutInventedFreshPose() async {
        let (coordinator, perception, motion, _) = setup()
        motion.suspendRotation = true
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        perception.send(frame(2))
        await drain()
        XCTAssertEqual(motion.rotations, [], "Relative-only motion cannot certify recovery provenance")
        _ = await coordinator.stop()
        motion.releaseRotation()
    }

    func testHealthyFrameAndAssociationUseOneSessionBudgetAndPreferCombinedSummary() async throws {
        let perception = FollowPerceptionFake()
        let motion = FollowMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0.5
        config.departureRangeIncrease = 0
        var records: [(String, Double, [String: Any])] = []
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config) { name, fields in
            if name == "follow_frame" || name == "follow_person.association" {
                let payload = fields["payload"].flatMap { $0.data(using: .utf8) }
                let record = payload.flatMap { try? JSONSerialization.jsonObject(with: $0) as? [String: Any] } ?? [:]
                records.append((name, clock.now, record))
            }
        }
        _ = await coordinator.start()
        await drain()
        let times = [0.0, 0.499, 0.5, 0.6, 0.7, 1.0, 1.499, 1.5, 1.6, 2.499, 2.5]
        for (index, time) in times.enumerated() {
            clock.advance(to: time, wakeSleepers: false)
            // Resume the pause at its exact boundary; later sleeps cannot create an outage.
            if time == 0.5 { clock.advance(to: time); await drain() }
            let sequence = UInt64(index + 1)
            perception.send(frame(sequence, at: time, people: [person(sequence, at: time, y: 1.5)]))
            await drain()
        }
        XCTAssertEqual(records.filter { $0.0 == "follow_frame" }.map { $0.1 }, [0])
        // Pause completion revalidates its latest frame, selecting initially;
        // the new boundary frame then transitions to continued at the same time.
        XCTAssertEqual(records.filter { $0.0 == "follow_person.association" }.map { $0.1 }, [0.5, 0.5, 1.5, 2.5])
        let periodic = try XCTUnwrap(records.first { $0.1 == 1.5 }?.2)
        XCTAssertEqual(periodic["tracking_state"] as? String, "normal")
        XCTAssertEqual(periodic["observation_age_s"] as? Double, 0)
        XCTAssertEqual(periodic["depth_available"] as? Bool, true)
        _ = await coordinator.stop()
        _ = await coordinator.start()
        await drain()
        perception.send(frame(20, at: 2.5))
        await drain()
        XCTAssertEqual(records.last?.0, "follow_frame")
        XCTAssertEqual(records.last?.2["session_generation"] as? Int, 3)
        _ = await coordinator.stop()
    }

    func testAssociationTransitionsEmitImmediatelyAndIdenticalLostAmbiguousRepeatOnlyOnce() async throws {
        let perception = FollowPerceptionFake()
        let motion = FollowMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        motion.suspendRotation = true
        var records: [[String: Any]] = []
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config) { name, fields in
            if name == "follow_person.association", let data = fields["payload"]?.data(using: .utf8),
               let record = try? JSONSerialization.jsonObject(with: data) as? [String: Any] { records.append(record) }
        }
        _ = await coordinator.start()
        await drain()
        for sequence: UInt64 in 1...8 {
            let people: [FollowPersonObservation]
            switch sequence {
            case 1, 2, 7, 8: people = [person(sequence, y: 1.5)]
            case 5, 6: people = [person(sequence, y: 1.5), person(sequence, y: 1.6)]
            default: people = []
            }
            perception.send(frame(sequence, people: people))
            await drain()
        }
        XCTAssertEqual(records.compactMap { $0["association_outcome"] as? String },
                       ["initial", "continued", "lost", "ambiguous", "reacquired", "continued"])
        XCTAssertEqual(records.compactMap { $0["previous_outcome"] as? String },
                       ["initial", "continued", "lost", "ambiguous", "reacquired"])
        XCTAssertTrue(records.allSatisfy { $0["schema_version"] as? Int == 1 && $0["session_generation"] as? Int == 1 })
        XCTAssertEqual(records.first?["phase"] as? String, "searching")
        XCTAssertEqual(records.first?["operation_id"] is NSNull, true)
        motion.releaseRotation()
        _ = await coordinator.stop()
    }

    func testCapturedFailedStopCannotBeDowngradedByCleanupSuccessOrAuthorizeRestart() async throws {
        let perception = FollowPerceptionFake()
        let motion = ContextualFollowMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        motion.legacy.suspendRotation = true
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config)
        _ = await coordinator.start()
        await drain()
        perception.send(frame(1))
        await drain()
        let context = FollowMotionOperationContext(request: try XCTUnwrap(motion.contexts.first),
            controllerOperationID: 91, purpose: .followScan, profile: nil)
        motion.sendFailure(.init(context: context, reason: .stalled, stopOutcome: .failed, source: .stream))
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Motor stop could not be confirmed. Motion is blocked."))
        let restarted = await coordinator.start()
        XCTAssertFalse(restarted)
        // An explicit operator retry ends this terminal delivery window. A late result
        // cannot resurrect its blocked UI even if it did not mark itself stale.
        let recovered = await coordinator.stop()
        XCTAssertTrue(recovered)
        XCTAssertEqual(coordinator.state, .stopped)
        motion.resultOverride = .init(result: .cancelled, context: context,
            failure: .init(context: context, reason: .commandFailed, stopOutcome: .confirmed,
                source: .result))
        motion.legacy.releaseRotation()
        await drain()
        XCTAssertEqual(coordinator.state, .stopped)
    }

    func testCapturedSearchFailureSurvivesDetectionPhaseChangeAndOnePendingStopOwner() async throws {
        let perception = FollowPerceptionFake()
        let motion = ContextualFollowMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        motion.legacy.suspendRotation = true
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config)
        _ = await coordinator.start()
        await drain()
        perception.send(frame(1))
        await drain()
        let request = try XCTUnwrap(motion.contexts.first)
        let context = FollowMotionOperationContext(request: request, controllerOperationID: 123,
            purpose: .followScan, profile: RoverConfig.followScanRotationProfile)
        motion.legacy.suspendStop = true
        perception.send(frame(2, people: [person(2)]))
        await drain()
        XCTAssertEqual(coordinator.state, .aligning)
        motion.sendFailure(.init(context: context, reason: .stalled, stopOutcome: .pending, source: .stream))
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Search rotation stopped: insufficient measured yaw progress. Confirming motor stop…"))
        motion.resultOverride = .init(result: .cancelled, context: context,
            failure: .init(context: context, reason: .cancelled, stopOutcome: .pending, source: .result, stale: true))
        motion.legacy.releaseRotation()
        await drain()
        XCTAssertEqual(motion.legacy.stops, 1)
        motion.legacy.suspendStop = false
        motion.legacy.releaseStop()
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Search rotation stopped: insufficient measured yaw progress. Stop confirmed. Restart following to try again."))
        XCTAssertEqual(motion.legacy.stops, 2, "Detection confirmation drains before the single terminal cleanup")
        XCTAssertTrue(motion.legacy.alignments.isEmpty)
    }

    func testOldSessionStreamAndSuspendedResultAfterRestartOnlyLogStaleEvidence() async throws {
        let perception = FollowPerceptionFake()
        let motion = ContextualFollowMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        motion.legacy.suspendRotation = true
        var events: [[String: Any]] = []
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { name, fields in
                if name == "follow_motion.failure_resolution", let data = fields["payload"]?.data(using: .utf8),
                   let event = try? JSONSerialization.jsonObject(with: data) as? [String: Any] { events.append(event) }
            })
        _ = await coordinator.start()
        await drain()
        perception.send(frame(1))
        await drain()
        let request = try XCTUnwrap(motion.contexts.first)
        let context = FollowMotionOperationContext(request: request, controllerOperationID: 91,
            purpose: .followScan, profile: nil)
        motion.sendFailure(.init(context: context, reason: .stalled, stopOutcome: .pending, source: .stream))
        await drain()
        let restarted = await coordinator.start()
        XCTAssertTrue(restarted)
        await drain()
        motion.resultOverride = .init(result: .failed(.stalled), context: context,
            failure: .init(context: context, reason: .stalled, stopOutcome: .failed, source: .result))
        motion.legacy.releaseRotation()
        motion.sendFailure(.init(context: context, reason: .commandFailed, stopOutcome: .failed, source: .stream))
        await drain()
        XCTAssertEqual(coordinator.state, .searching)
        XCTAssertEqual(motion.legacy.stops, 1)
        XCTAssertTrue(events.suffix(2).allSatisfy { $0["stale"] as? Bool == true })
        XCTAssertTrue(events.suffix(2).allSatisfy { $0["session_generation"] as? Int == 1 })
        _ = await coordinator.stop()
    }

    func testLegacyStallDoesNotInventControllerPurposeOrNoYawProgress() async {
        let (coordinator, perception, motion, _) = setup()
        motion.suspendRotation = true
        _ = await coordinator.start()
        perception.send(frame(1))
        await drain()
        motion.safety(.failed(.stalled))
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Navigation stopped: insufficient measured progress."))
        XCTAssertEqual(motion.stops, 1)
        motion.releaseRotation()
        await drain()
    }

    func testCorrelatedFailureOrdersPublishPendingThenOnlyAcknowledgedStopAndRetainFailedStop() async throws {
        for streamFirst in [true, false] {
            for stopFails in [false, true] {
                let perception = FollowPerceptionFake()
                let motion = ContextualFollowMotionFake()
                let clock = ManualFollowClock()
                var config = FollowMeConfiguration()
                config.stationaryPauseSeconds = 0
                motion.legacy.suspendRotation = true
                motion.legacy.suspendStop = true
                motion.legacy.stopError = stopFails
                var events: [[String: Any]] = []
                let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                    configuration: config, eventSink: { name, fields in
                        if name == "follow_motion.failure_resolution", let json = fields["payload"],
                           let data = json.data(using: .utf8),
                           let event = try? JSONSerialization.jsonObject(with: data) as? [String: Any] {
                            events.append(event)
                        }
                    })
                _ = await coordinator.start()
                await drain()
                perception.send(frame(1))
                await drain()
                let request = try XCTUnwrap(motion.contexts.first)
                let context = FollowMotionOperationContext(request: request, controllerOperationID: 91,
                    purpose: .followScan, profile: RoverConfig.followScanRotationProfile)
                let stream = FollowMotionFailureDelivery(context: context, reason: .stalled,
                    stopOutcome: .pending, source: .stream)
                let resultFailure = FollowMotionFailureDelivery(context: context, reason: .stalled,
                    stopOutcome: .pending, source: .result)
                motion.resultOverride = .init(result: .failed(.commandFailed), context: context, failure: resultFailure)
                if streamFirst { motion.sendFailure(stream) } else { motion.legacy.releaseRotation() }
                await drain()
                XCTAssertEqual(coordinator.state, .failed("Search rotation stopped: insufficient measured yaw progress. Confirming motor stop…"))
                XCTAssertEqual(motion.legacy.stops, 1)
                if streamFirst { motion.legacy.releaseRotation() } else { motion.sendFailure(stream) }
                await drain()
                XCTAssertEqual(motion.legacy.stops, 1, "The second delivery cannot own cleanup")
                let concurrentStop = Task { await coordinator.stop() }
                await drain()
                motion.legacy.releaseStop()
                _ = await concurrentStop.value
                await drain()
                let expected = stopFails ? "Motor stop could not be confirmed. Motion is blocked."
                    : "Search rotation stopped: insufficient measured yaw progress. Stop confirmed. Restart following to try again."
                XCTAssertEqual(coordinator.state, .failed(expected))
                XCTAssertEqual(events.filter { $0["source"] as? String == "stream" }.count, 1)
                XCTAssertEqual(events.filter { $0["source"] as? String == "result" }.count, 1)
                XCTAssertTrue(events.contains { $0["deduplicated"] as? Bool == true })
                XCTAssertTrue(events.allSatisfy { $0["operation_id"] as? Int == 91 })
                XCTAssertEqual(events.last?["reason"] as? String, "no_yaw_progress")
                motion.sendFailure(.init(context: context, reason: .commandFailed, stopOutcome: .confirmed,
                    source: .stream, stale: true))
                await drain()
                XCTAssertEqual(coordinator.state, .failed(expected))
                XCTAssertEqual(motion.legacy.stops, 1)
                XCTAssertEqual(events.last?["stale"] as? Bool, true)
                XCTAssertEqual(events.last?["deduplicated"] as? Bool, false,
                    "After both deliveries and cleanup drain, stale telemetry must not retain a historical reducer record")
                if stopFails {
                    let started = await coordinator.start()
                    XCTAssertFalse(started)
                }
            }
        }
    }

    func testContextualRuntimeCapturesSearchFollowAndReacquisitionAndUsesOneFailureChannel() async {
        let perception = FollowPerceptionFake()
        let motion = ContextualFollowMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.scanObservationSeconds = 0
        config.scanIncrement = .pi / 6
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        motion.legacy.suspendRotation = true
        motion.legacy.suspendNavigation = true
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config)
        _ = await coordinator.start()
        await drain()
        perception.send(frame(1))
        await drain()
        XCTAssertEqual(motion.contextualSubscriptions, 1)
        XCTAssertEqual(motion.legacySubscriptions, 0)
        XCTAssertEqual(motion.contexts.first?.phase, "searching")
        XCTAssertEqual(motion.contexts.first?.sessionGeneration, 1)
        XCTAssertEqual(motion.contexts.first?.scanUsed ?? 0, .pi / 6, accuracy: 1e-12)
        XCTAssertEqual(motion.contexts.first?.scanRemaining ?? 0, 2 * .pi - .pi / 6, accuracy: 1e-12)
        perception.send(frame(2, people: [person(2)]))
        await drain()
        motion.legacy.releaseRotation()
        await drain()
        perception.send(frame(3))
        await drain()
        XCTAssertEqual(motion.contexts.map(\.phase), ["searching", "following"])
        XCTAssertEqual(motion.requests.map(\.purpose), [.followScan, .followGoal])
        XCTAssertEqual(coordinator.state, .reacquiring, "Unknown-provenance contextual providers observe stopped")
        XCTAssertEqual(Set(motion.contexts.map(\.requestToken)).count, 2)
        _ = await coordinator.stop()
        motion.legacy.releaseRotation()
        motion.legacy.releaseNavigation()
        await drain()
    }

    func testContextualRuntimeCapturesAlignmentReadyAndDepartureRequests() async {
        let perception = FollowPerceptionFake()
        let motion = ContextualFollowMotionFake()
        let clock = ManualFollowClock()
        motion.legacy.useSourceClock(clock)
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config)
        _ = await coordinator.start()
        await drain()
        perception.send(frame(0, people: [person(0)]))
        await drain()
        await matchedBoundaryFrame(1, perception, clock, yaw: 0)
        await matchedBoundaryFrame(2, perception, clock)
        await matchedBoundaryFrame(3, perception, clock)
        await matchedBoundaryFrame(4, perception, clock, y: 4.3)
        XCTAssertEqual(motion.requests.map(\.purpose), [.followAlignment, .followReady, .followGoal])
        XCTAssertEqual(motion.contexts.map(\.phase), ["aligning", "aligning", "following"],
                       "Capture the actual pending request phase, not signaling before admission")
        XCTAssertEqual(Set(motion.contexts.map(\.sessionGeneration)), [1])
        XCTAssertEqual(Set(motion.contexts.map(\.requestToken)).count, 3)
        _ = await coordinator.stop()
    }

    func testPersonApproachingDuringReadySignalCancelsBeforeHoldClearanceIsCrossed() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        motion.suspendReadySignal = true
        await acquireSignaling(coordinator, perception, clock, range: 1.6)
        let stops = motion.stops
        perception.send(frame(3, at: clock.now, pose: Pose2D(position: .zero, yaw: .pi / 2), people: [person(3, at: clock.now, y: 1.3)]))
        await drain()
        XCTAssertGreaterThan(motion.stops, stops)
        XCTAssertFalse(coordinator.isActive)
        motion.releaseReadySignal()
        await drain()
        XCTAssertNotEqual(coordinator.state, .waitingForMovement)
        XCTAssertEqual(motion.readySignals, 1)
    }

    func testTooClosePersonCannotAuthorizeReadySignalAcrossHoldClearance() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(0, at: 5, people: [person(0, at: 5, y: 1.3)]))
        await drain()
        await matchedBoundaryFrame(1, perception, clock, yaw: 0, y: 1.3)
        await matchedBoundaryFrame(2, perception, clock, y: 1.3)
        XCTAssertEqual(motion.readySignals, 0)
        XCTAssertEqual(String(describing: coordinator.state), "waitingForClearance")
        XCTAssertTrue(coordinator.isActive)
        XCTAssertTrue(motion.goals.isEmpty)
        let alignments = motion.alignments.count
        await matchedBoundaryFrame(3, perception, clock, y: 1.31)
        XCTAssertEqual(motion.alignments.count, alignments)
        XCTAssertEqual(motion.readySignals, 0)
        _ = await coordinator.stop()
    }

    func testClearanceWaitHeadingDriftReturnsToAlignmentAndNeedsNewPostStopFrame() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(0, at: 5, people: [person(0, at: 5, y: 1.3)]))
        await drain()
        await matchedBoundaryFrame(1, perception, clock, yaw: 0, y: 1.3)
        await matchedBoundaryFrame(2, perception, clock, y: 1.3)
        XCTAssertEqual(coordinator.state, .waitingForClearance)
        motion.suspendAlignment = true
        await matchedBoundaryFrame(3, perception, clock, yaw: .pi / 2 - 0.06, y: 1.4)
        XCTAssertEqual(coordinator.state, .aligning)
        XCTAssertEqual(motion.alignments.count, 1, "Drift detection must clear its new stop before turning")
        await matchedBoundaryFrame(4, perception, clock, yaw: .pi / 2 - 0.06, y: 1.4)
        XCTAssertEqual(motion.alignments.count, 2)
        motion.releaseAlignment()
        await drain()
        XCTAssertEqual(motion.readySignals, 0)
        await matchedBoundaryFrame(5, perception, clock, y: 1.4)
        XCTAssertEqual(motion.readySignals, 1)
        _ = await coordinator.stop()
    }

    func testCachedOrFutureStepBackFrameCannotReleaseClearanceWait() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(0, at: 5, people: [person(0, at: 5, y: 1.3)]))
        await drain()
        await matchedBoundaryFrame(1, perception, clock, yaw: 0, y: 1.3)
        await matchedBoundaryFrame(2, perception, clock, y: 1.3)
        perception.send(frame(2, at: clock.now, pose: Pose2D(position: .zero, yaw: .pi / 2), people: [person(2, at: clock.now, y: 1.6)]))
        await drain()
        XCTAssertEqual(coordinator.state, .waitingForClearance)
        perception.send(frame(3, at: clock.now + 0.1, pose: Pose2D(position: .zero, yaw: .pi / 2), people: [person(3, at: clock.now + 0.1, y: 1.6)]))
        await drain()
        XCTAssertEqual(motion.readySignals, 0)
        XCTAssertTrue(motion.goals.isEmpty)
        XCTAssertEqual(coordinator.perceptionIssue, .staleFrame)
        clock.advance(to: clock.now + 2)
        await drain()
        XCTAssertFalse(coordinator.isActive, "Clearance waiting cannot extend the existing outage deadline")
        _ = await coordinator.stop()
    }

    func testHealthyMatchedClearanceWaitHasNoNewTimeoutOrMotion() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(0, at: 5, people: [person(0, at: 5, y: 1.3)]))
        await drain()
        await matchedBoundaryFrame(1, perception, clock, yaw: 0, y: 1.3)
        await matchedBoundaryFrame(2, perception, clock, y: 1.3)
        for sequence in UInt64(3)...33 {
            let time = clock.now + 0.4
            clock.advance(to: time)
            perception.send(frame(sequence, at: time, pose: Pose2D(position: .zero, yaw: .pi / 2),
                                  people: [person(sequence, at: time, y: 1.3)]))
            await drain()
        }
        XCTAssertEqual(coordinator.state, .waitingForClearance)
        XCTAssertTrue(coordinator.isActive)
        XCTAssertEqual(motion.readySignals, 0)
        XCTAssertEqual(motion.alignments.count, 1)
        XCTAssertTrue(motion.rotations.isEmpty)
        XCTAssertTrue(motion.goals.isEmpty)
        _ = await coordinator.stop()
    }

    func testLossAfterSignalStopBeforeBaselineReacquiresWithoutSecondMoveAndCanDepart() async {
        let (coordinator, perception, motion, clock) = sourcedRecoverySetup()
        motion.suspendRotation = true
        await acquireSignaling(coordinator, perception, clock)
        perception.send(frame(3, at: clock.now))
        await drain()
        for sequence in UInt64(4)...6 {
            await matchedBoundaryFrame(sequence, perception, clock)
        }
        XCTAssertEqual(coordinator.state, .waitingForMovement)
        await matchedBoundaryFrame(7, perception, clock, y: 4.3)
        XCTAssertEqual(motion.goals.count, 1, "A confirmed signal without its baseline must not leave departure permanently gated")
        XCTAssertEqual(motion.readySignals, 1)
        motion.releaseRotation()
        _ = await coordinator.stop()
    }

    func testLossDuringReadySignalCannotRepeatMoveOrBecomeReadyAfterReacquisition() async {
        let (coordinator, perception, motion, clock) = sourcedRecoverySetup()
        motion.suspendReadySignal = true
        motion.suspendRotation = true
        await acquireSignaling(coordinator, perception, clock)
        perception.send(frame(3, at: clock.now))
        await drain()
        motion.releaseReadySignal()
        await matchedBoundaryFrame(4, perception, clock)
        await matchedBoundaryFrame(5, perception, clock)
        await matchedBoundaryFrame(6, perception, clock)
        XCTAssertEqual(motion.readySignals, 1)
        XCTAssertEqual(coordinator.state, .failed("Ready signal interrupted. Stop and start following again."))
        XCTAssertTrue(motion.goals.isEmpty)
        motion.releaseRotation()
        _ = await coordinator.stop()
    }

    func testCancelledUnsafeStaleAndOperatorStoppedReadySignalNeverMarksReady() async {
        for scenario in 0..<4 {
            let (coordinator, perception, motion, clock) = productionSetup()
            motion.suspendReadySignal = true
            await acquireSignaling(coordinator, perception, clock)
            if scenario == 0 { motion.readySignalResult = .cancelled }
            if scenario == 1 { motion.readySignalResult = .failed(.obstacle) }
            if scenario == 2 { clock.advance(to: clock.now + 0.501); await drain() }
            if scenario == 3 { coordinator.inhibitMotion(); _ = await coordinator.stop() }
            motion.releaseReadySignal()
            await drain()
            perception.send(frame(3, at: clock.now, pose: Pose2D(position: .zero, yaw: .pi / 2), people: [person(3, at: clock.now)]))
            await drain()
            XCTAssertNotEqual(coordinator.state, .waitingForMovement)
            XCTAssertEqual(motion.readySignals, 1)
            XCTAssertTrue(motion.goals.isEmpty)
            _ = await coordinator.stop()
        }
    }

    func testReadySignalOnceThenFreshPostStopBaselineBeforeWaiting() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        motion.suspendReadySignal = true
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(0, at: 5, people: [person(0, at: 5)]))
        await drain()
        await matchedBoundaryFrame(1, perception, clock, yaw: 0)
        await matchedBoundaryFrame(2, perception, clock)
        XCTAssertEqual(motion.readySignals, 1)
        XCTAssertEqual(String(describing: coordinator.state), "signalingReady")
        XCTAssertTrue(motion.goals.isEmpty)
        motion.releaseReadySignal()
        await drain()
        XCTAssertEqual(String(describing: coordinator.state), "signalingReady", "Completion needs a new post-stop frame")
        await matchedBoundaryFrame(3, perception, clock, position: Vec2(0, 0.1))
        XCTAssertEqual(coordinator.state, .waitingForMovement)
        await matchedBoundaryFrame(4, perception, clock, position: Vec2(0, 0.1), y: 4.299)
        XCTAssertTrue(motion.goals.isEmpty)
        await matchedBoundaryFrame(5, perception, clock, position: Vec2(0, 0.1), y: 4.3)
        XCTAssertEqual(motion.goals.count, 1)
        XCTAssertEqual(motion.readySignals, 1)
        _ = await coordinator.stop()
    }

    func testAlignmentProgressesWithNewMatchedFramesDuringEveryStopAcknowledgement() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        motion.suspendStop = true
        motion.suspendAlignment = true
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(1, at: 5, people: [person(1, at: 5, x: 1)]))
        await drain()
        XCTAssertEqual(motion.stops, 1)
        XCTAssertTrue(motion.alignments.isEmpty)
        for sequence in UInt64(2)...3 {
            let time = 5 + Double(sequence - 1) / 10
            clock.advance(to: time)
            perception.send(frame(sequence, at: time,
                                  pose: Pose2D(position: Vec2(1, time - 5), yaw: sequence == 2 ? .pi / 4 : .pi / 3),
                                  people: [person(sequence, at: time, x: 1, y: 4 + time - 5)]))
            await drain()
        }
        motion.releaseStop()
        await drain()
        XCTAssertTrue(motion.alignments.isEmpty, "All frames captured before ACK remain fenced")
        await matchedBoundaryFrame(4, perception, clock, yaw: .pi / 3, position: Vec2(1, 0.501), x: 1, y: 4.2)
        XCTAssertEqual(motion.alignments.count, 1, "A deliberate settled post-ACK match resumes alignment")
        if let angle = motion.alignments.first { XCTAssertEqual(angle, .pi / 6, accuracy: 0.0001, "Use newest matched person and its same-snapshot rover pose") }
        await matchedBoundaryFrame(5, perception, clock, position: Vec2(1, 0.6), x: 1, y: 4.2, at: clock.now + 0.1)
        motion.releaseAlignment()
        await drain()
        XCTAssertEqual(motion.stops, 2)
        await matchedBoundaryFrame(6, perception, clock, position: Vec2(1, 0.7), x: 1, y: 4.2, at: clock.now + 0.1)
        clock.advance(to: clock.now + 0.1)
        motion.suspendStop = false
        motion.releaseStop()
        await drain()
        await matchedBoundaryFrame(7, perception, clock, position: Vec2(1, 0.8), x: 1, y: 4.2, at: clock.now)
        XCTAssertEqual(motion.readySignals, 0, "Equal ACK timestamp cannot hand off")
        await matchedBoundaryFrame(8, perception, clock, position: Vec2(1, 0.8), x: 1, y: 4.2)
        await matchedBoundaryFrame(9, perception, clock, position: Vec2(1, 0.9), x: 1, y: 4.2)
        XCTAssertEqual(coordinator.state, .waitingForMovement)
        XCTAssertEqual(motion.alignments.count, 1, "Repeated arrivals across both acknowledgements must allow alignment to complete")
        XCTAssertTrue(motion.goals.isEmpty)
        _ = await coordinator.stop()
        motion.releaseAlignment()
    }

    func testHealthyFrameAtReadinessDeadlineCannotBeatTimeoutTask() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        await drain()
        clock.advance(to: 10, wakeSleepers: false)
        perception.send(frame(1, at: 10, people: [person(1, at: 10)]))
        await drain()
        XCTAssertEqual(coordinator.state, .failed(FollowPerceptionIssue.noFrames.message))
        XCTAssertTrue(motion.alignments.isEmpty)
        XCTAssertTrue(motion.goals.isEmpty)
        clock.advance(to: 10)
        await drain()
    }

    func testCancelledAlignmentStillConfirmsStopAndCannotEstablishBaseline() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        motion.alignmentResult = .cancelled
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(1, at: 5, people: [person(1, at: 5)]))
        await drain()
        await matchedBoundaryFrame(2, perception, clock, yaw: 0)
        XCTAssertEqual(motion.stops, 2, "Even a cancelled alignment must confirm stationary motors afterward")
        XCTAssertEqual(coordinator.state, .aligning)
        XCTAssertTrue(motion.goals.isEmpty)
        _ = await coordinator.stop()
    }

    func testProductionReadinessWindowStartsAfterPauseAndFailsAtTenSeconds() async {
        let (coordinator, _, motion, clock) = productionSetup()
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 4.999)
        await drain()
        XCTAssertEqual(coordinator.state, .pausing)
        clock.advance(to: 5)
        await drain()
        XCTAssertEqual(coordinator.state, .searching)
        clock.advance(to: 9.999)
        await drain()
        XCTAssertTrue(coordinator.isActive)
        clock.advance(to: 10)
        await drain()
        XCTAssertEqual(coordinator.state, .failed(FollowPerceptionIssue.noFrames.message))
        XCTAssertTrue(motion.rotations.isEmpty)
        XCTAssertTrue(motion.alignments.isEmpty)
        XCTAssertTrue(motion.goals.isEmpty)
    }

    func testExpiredPauseFrameCannotStartScanAlignmentOrTranslation() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        clock.advance(to: 5)
        await drain()
        XCTAssertEqual(coordinator.state, .searching)
        XCTAssertEqual(coordinator.perceptionIssue, .staleFrame)
        XCTAssertTrue(motion.rotations.isEmpty)
        XCTAssertTrue(motion.alignments.isEmpty)
        XCTAssertTrue(motion.goals.isEmpty)
        perception.send(frame(2, at: 5, people: [person(2, at: 5)]))
        await drain()
        XCTAssertTrue(motion.alignments.isEmpty, "Detection capture is before its new stop ACK")
        await matchedBoundaryFrame(3, perception, clock, yaw: 0)
        XCTAssertEqual(motion.alignments, [.pi / 2])
        XCTAssertTrue(motion.goals.isEmpty)
        _ = await coordinator.stop()
    }

    func testStopDuringPauseAndWaitingPreventsCallbacksAndTranslation() async {
        for waiting in [false, true] {
            let (coordinator, perception, motion, clock) = productionSetup()
            if waiting { await acquireWaiting(coordinator, perception, clock) }
            else { _ = await coordinator.start(); await drain() }
            coordinator.inhibitMotion()
            _ = await coordinator.stop()
            clock.advance(to: 10)
            perception.send(frame(99, at: 10, people: [person(99, at: 10, y: 4.5)]))
            await drain()
            XCTAssertEqual(coordinator.state, .stopped)
            XCTAssertTrue(motion.goals.isEmpty)
            XCTAssertEqual(motion.alignments.count, waiting ? 1 : 0)
            XCTAssertTrue(motion.rotations.isEmpty)
        }
    }

    func testOutdatedAlignmentStopAcknowledgementCannotTurnOrTranslate() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        motion.suspendStop = true
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(1, at: 5, people: [person(1, at: 5)]))
        await drain()
        XCTAssertEqual(motion.stops, 1)
        clock.advance(to: 5.501)
        await drain()
        motion.suspendStop = false
        motion.releaseStop()
        await drain()
        XCTAssertTrue(motion.alignments.isEmpty)
        XCTAssertTrue(motion.goals.isEmpty)
        XCTAssertEqual(coordinator.perceptionIssue, .staleFrame)
        _ = await coordinator.stop()
    }

    func testWaitingTracksSamePersonAndLateralOrTowardMotionDoesNotDepart() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        await acquireWaiting(coordinator, perception, clock)
        for (sequence, x, y) in [(UInt64(4), 0.5, sqrt(16 - 0.25)), (5, 0.0, 3.5), (6, 0.0, 4.0)] {
            perception.send(frame(sequence, at: clock.now,
                                  people: [person(sequence, at: clock.now, x: x, y: y, screen: 0.65),
                                           person(sequence, at: clock.now, x: 6, y: 6, screen: 0.5)]))
            await drain()
            XCTAssertEqual(coordinator.state, .waitingForMovement)
            XCTAssertTrue(motion.goals.isEmpty)
        }
        _ = await coordinator.stop()
    }

    func testWaitingOutageStopsOnceAndFailsAfterTwoSecondsWithoutDeparture() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        await acquireWaiting(coordinator, perception, clock)
        let stops = motion.stops
        let outage = clock.now
        perception.send(frame(4, at: outage, depth: false))
        await drain()
        clock.advance(to: outage + 1)
        perception.send(frame(5, at: clock.now, pose: nil))
        await drain()
        XCTAssertEqual(motion.stops, stops + 1)
        clock.advance(to: outage + 2)
        await drain()
        XCTAssertEqual(coordinator.state, .failed(FollowPerceptionIssue.poseUnavailable.message))
        XCTAssertTrue(motion.goals.isEmpty)
    }

    func testProductionSearchNeverExceedsOneRoundEvenWithNonDividingScanIncrement() async {
        for increment in [Double.pi / 18, Double.pi / 6, 0.7] {
            let perception = FollowPerceptionFake()
            let motion = FollowMotionFake()
            let clock = ManualFollowClock()
            var config = FollowMeConfiguration()
            config.scanIncrement = increment
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                                                  configuration: config, eventSink: { _, _ in })
            _ = await coordinator.start()
            await drain()
            clock.advance(to: 5)
            perception.send(frame(1, at: 5))
            await drain()
            for index in 0..<200 where coordinator.isActive {
                clock.advance(to: clock.now + 0.25)
                perception.send(frame(UInt64(index + 2), at: clock.now))
                await drain()
            }
            XCTAssertEqual(coordinator.state, .failed("No person found."))
            XCTAssertEqual(motion.rotations.reduce(0, +), 2 * .pi, accuracy: 0.00001)
            var requested = 0.0
            for angle in motion.rotations {
                requested += angle
                XCTAssertLessThanOrEqual(requested, 2 * .pi + 1e-12)
            }
            if increment == 0.7 {
                XCTAssertEqual(motion.rotations.count, 9)
                XCTAssertEqual(motion.rotations.last!, 0.6831853071795862, accuracy: 1e-12)
            }
            XCTAssertTrue(motion.goals.isEmpty)
            XCTAssertGreaterThan(motion.stops, 0)
        }
    }

    func testBaselineRejectsDelayedPreCompletionFrameAndIncorrectHeading() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        motion.suspendAlignment = true
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(0, at: 5, people: [person(0, at: 5)]))
        await drain()
        await matchedBoundaryFrame(1, perception, clock, yaw: 0)
        motion.releaseAlignment()
        await drain()
        perception.send(frame(2, at: 5.2, pose: Pose2D(position: .zero, yaw: .pi / 2),
                              people: [person(2, at: 5.2)]))
        await drain()
        XCTAssertEqual(coordinator.state, .aligning, "Delayed in-turn observation cannot establish baseline")
        XCTAssertEqual(motion.alignments.count, 1, "Wait for a post-completion frame")
        await matchedBoundaryFrame(3, perception, clock, yaw: 0)
        XCTAssertEqual(coordinator.state, .aligning, "Arrival alone cannot certify heading")
        await matchedBoundaryFrame(4, perception, clock, yaw: 0)
        XCTAssertEqual(motion.alignments.count, 2)
        motion.releaseAlignment()
        await drain()
        await matchedBoundaryFrame(5, perception, clock)
        await matchedBoundaryFrame(6, perception, clock)
        XCTAssertEqual(coordinator.state, .waitingForMovement)
        XCTAssertTrue(motion.goals.isEmpty)
        _ = await coordinator.stop()
    }

    private func productionSetup(scanObservationSeconds: Double? = nil) -> (FollowMeCoordinator, FollowPerceptionFake, FollowMotionFake, ManualFollowClock) {
        let perception = FollowPerceptionFake()
        let motion = FollowMotionFake()
        let clock = ManualFollowClock()
        motion.useSourceClock(clock)
        var config = FollowMeConfiguration()
        if let scanObservationSeconds { config.scanObservationSeconds = scanObservationSeconds }
        return (FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                                    configuration: config, eventSink: { _, _ in }), perception, motion, clock)
    }

    private func sourcedRecoverySetup() -> (FollowMeCoordinator, FollowPerceptionFake, FollowMotionFake, ManualFollowClock) {
        let perception = FollowPerceptionFake()
        let motion = AbsoluteRecoveryMotionFake()
        let clock = ManualFollowClock()
        motion.legacy.useSourceClock(clock)
        motion.sample = .init(pose: Pose2D(position: .zero, yaw: .pi / 2),
            frameID: ARFrameID(generation: 1, sequence: 100), sourceTimestamp: 5,
            trackingQuality: .normal, source: "synthetic")
        return (FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            eventSink: { _, _ in }), perception, motion.legacy, clock)
    }

    private func acquireWaiting(_ coordinator: FollowMeCoordinator, _ perception: FollowPerceptionFake,
                                _ clock: ManualFollowClock) async {
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(0, at: 5, people: [person(0, at: 5)]))
        await drain()
        await matchedBoundaryFrame(1, perception, clock, yaw: 0)
        await matchedBoundaryFrame(2, perception, clock)
        await matchedBoundaryFrame(3, perception, clock)
        XCTAssertEqual(coordinator.state, .waitingForMovement)
    }

    private func acquireSignaling(_ coordinator: FollowMeCoordinator, _ perception: FollowPerceptionFake,
                                  _ clock: ManualFollowClock, range: Double = 4) async {
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(0, at: 5, people: [person(0, at: 5, y: range)]))
        await drain()
        await matchedBoundaryFrame(1, perception, clock, yaw: 0, y: range)
        await matchedBoundaryFrame(2, perception, clock, y: range)
        XCTAssertEqual(coordinator.state, .signalingReady)
    }

    /// One deliberate capture event; never auto-pumps while motion/outage is pending.
    private func matchedBoundaryFrame(_ sequence: UInt64, _ perception: FollowPerceptionFake,
                                      _ clock: ManualFollowClock, yaw: Double = .pi / 2,
                                      position: Vec2 = .zero, x: Double = 0, y: Double = 4,
                                      at timestamp: TimeInterval? = nil) async {
        let time = timestamp ?? clock.now + 0.301
        clock.advance(to: time)
        perception.send(frame(sequence, at: time, pose: Pose2D(position: position, yaw: yaw),
            people: [person(sequence, at: time, x: x, y: y)]))
        await drain()
    }

    func testLossBeforeDepartureReacquiresAndRetainsFixedBaseline() async {
        let (coordinator, perception, motion, clock) = sourcedRecoverySetup()
        motion.suspendRotation = true
        await acquireWaiting(coordinator, perception, clock)
        perception.send(frame(4, at: clock.now))
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring)
        perception.send(frame(5, at: clock.now, pose: Pose2D(position: .zero, yaw: .pi / 2),
                              people: [person(5, at: clock.now, y: 4.2)]))
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring)
        XCTAssertTrue(motion.goals.isEmpty, "Reacquisition cannot bypass departure")
        await matchedBoundaryFrame(6, perception, clock, y: 4.2)
        XCTAssertEqual(coordinator.state, .waitingForMovement)
        await matchedBoundaryFrame(7, perception, clock, y: 4.299)
        XCTAssertEqual(coordinator.state, .waitingForMovement)
        XCTAssertTrue(motion.goals.isEmpty)
        await matchedBoundaryFrame(8, perception, clock, y: 4.3)
        XCTAssertEqual(motion.goals.count, 1, "The original baseline survives reacquisition")
        XCTAssertEqual(motion.readySignals, 1, "Reacquisition cannot repeat the ready move")
        _ = await coordinator.stop()
        motion.releaseRotation()
    }

    func testStopDuringAlignmentAllowsNewGenerationToAlignBeforeOldTurnCompletes() async {
        let (coordinator, perception, motion, clock) = productionSetup()
        motion.suspendAlignment = true
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(1, at: 5, people: [person(1, at: 5)]))
        await drain()
        await matchedBoundaryFrame(2, perception, clock, yaw: 0)
        XCTAssertEqual(motion.alignments.count, 1)
        _ = await coordinator.stop()
        _ = await coordinator.start()
        await drain()
        clock.advance(to: clock.now + 5)
        perception.send(frame(1, at: clock.now, people: [person(1, at: clock.now)]))
        await drain()
        await matchedBoundaryFrame(2, perception, clock, yaw: 0)
        XCTAssertEqual(motion.alignments.count, 2, "An old suspended alignment cannot block a new generation")
        motion.releaseAlignment()
        await drain()
        XCTAssertTrue(motion.goals.isEmpty)
        _ = await coordinator.stop()
    }

    func testProductionSequenceRequiresFreshAlignedBaselineAndPointThreeRangeDeparture() async {
        let perception = FollowPerceptionFake()
        let motion = FollowMotionFake()
        let clock = ManualFollowClock()
        motion.useSourceClock(clock)
        motion.suspendAlignment = true
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock)
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(0, at: 5, people: [person(0, at: 5)]))
        await drain()
        await matchedBoundaryFrame(1, perception, clock, yaw: 0)
        XCTAssertEqual(motion.alignments, [.pi / 2])
        await matchedBoundaryFrame(2, perception, clock, y: 4.5)
        XCTAssertTrue(motion.goals.isEmpty)
        motion.releaseAlignment()
        await drain()
        XCTAssertEqual(coordinator.state, .aligning, "An in-turn frame cannot establish the baseline")
        await matchedBoundaryFrame(3, perception, clock, y: 4.5)
        await matchedBoundaryFrame(4, perception, clock, y: 4.5)
        XCTAssertEqual(String(describing: coordinator.state), "waitingForMovement")
        XCTAssertEqual(motion.stops, 3, "Alignment and ready signal end with confirmed stops")
        for (sequence, distance) in [(UInt64(5), 4.5), (6, 4.3), (7, 4.799)] {
            perception.send(frame(sequence, at: clock.now, pose: Pose2D(position: .zero, yaw: .pi / 2),
                                  people: [person(sequence, at: clock.now, y: distance)]))
            await drain()
            XCTAssertTrue(motion.goals.isEmpty, "Stationary, toward, and .299-away observations must hold")
        }
        perception.send(frame(8, at: clock.now, pose: Pose2D(position: .zero, yaw: .pi / 2),
                              people: [person(8, at: clock.now, y: 4.8)]))
        await drain()
        XCTAssertEqual(coordinator.state, .following)
        XCTAssertEqual(motion.goals.count, 1)
        XCTAssertEqual(motion.goals.first?.y ?? 0, 3.3, accuracy: 0.0001)
        _ = await coordinator.stop()
    }

    func testProductionDetectionStopsScanImmediatelyAndAlignsWithoutBlockingFrames() async {
        let perception = FollowPerceptionFake()
        let motion = FollowMotionFake()
        let clock = ManualFollowClock()
        motion.useSourceClock(clock)
        motion.suspendRotation = true
        motion.suspendAlignment = true
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock)
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 5)
        perception.send(frame(1, at: 5))
        await drain()
        XCTAssertEqual(motion.rotations.count, 1)
        perception.send(frame(2, at: 5, pose: Pose2D(position: Vec2(1, 1), yaw: -3 * .pi / 4),
                              people: [person(2, at: 5, x: -3, y: 1)]))
        await drain()
        XCTAssertTrue(motion.alignments.isEmpty)
        await matchedBoundaryFrame(3, perception, clock, yaw: -3 * .pi / 4, position: Vec2(1, 1), x: -3, y: 1)
        XCTAssertEqual(motion.alignments.count, 1)
        if let angle = motion.alignments.first { XCTAssertEqual(angle, -.pi / 4, accuracy: 0.0001) }
        XCTAssertEqual(String(describing: coordinator.state), "aligning")
        XCTAssertTrue(motion.goals.isEmpty)
        XCTAssertGreaterThanOrEqual(motion.stops, 1)
        XCTAssertTrue(motion.stopOrigins.contains(.detection), "Detection attribution is captured before the stop suspension")
        perception.send(frame(4, at: clock.now, depth: false))
        await drain()
        XCTAssertEqual(coordinator.perceptionIssue, .depthUnavailable, "Alignment cannot block perception")
        motion.releaseRotation()
        motion.releaseAlignment()
        await drain()
        XCTAssertEqual(motion.rotations.count, 1, "Detection ends the initial round immediately")
        XCTAssertTrue(motion.goals.isEmpty)
        _ = await coordinator.stop()
    }

    func testProductionPauseProcessesPerceptionButNeverMovesBeforeFiveSeconds() async {
        let perception = FollowPerceptionFake()
        let motion = FollowMotionFake()
        let clock = ManualFollowClock()
        motion.suspendRotation = true
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock)
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        XCTAssertNil(coordinator.perceptionIssue)
        XCTAssertEqual(String(describing: coordinator.state), "pausing")
        XCTAssertTrue(motion.goals.isEmpty)
        XCTAssertTrue(motion.rotations.isEmpty)
        clock.advance(to: 4.999)
        perception.send(frame(2, at: 4.999))
        await drain()
        XCTAssertEqual(String(describing: coordinator.state), "pausing")
        XCTAssertTrue(motion.rotations.isEmpty)
        clock.advance(to: 5)
        await drain()
        XCTAssertEqual(coordinator.state, .searching)
        XCTAssertEqual(motion.rotations.count, 1)
        XCTAssertTrue(motion.goals.isEmpty, "Old pause person must not initiate translation")
        _ = await coordinator.stop()
        motion.releaseRotation()
    }

    private func setup() -> (FollowMeCoordinator, FollowPerceptionFake, FollowMotionFake, ManualFollowClock) {
        let perception = FollowPerceptionFake()
        let motion = FollowMotionFake()
        let clock = ManualFollowClock()
        // These tests exercise downstream legacy follow/readiness policies directly.
        var config = FollowMeConfiguration()
        config.scanObservationSeconds = 0 // Legacy downstream tests; production pacing has separate coverage.
        config.scanIncrement = .pi / 6
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        return (FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                                    configuration: config,
                                    eventSink: { _, _ in }), perception, motion, clock)
    }

    private func frame(_ sequence: UInt64, at time: TimeInterval = 0, pose: Pose2D? = Pose2D(position: .zero, yaw: 0), depth: Bool = true, people: [FollowPersonObservation] = [], quality: ARTrackingQuality? = .normal) -> FollowPerceptionEvent {
        .frame(FollowFrameBatch(frameID: ARFrameID(generation: 1, sequence: sequence), timestamp: time, pose: pose, depthAvailable: depth, people: people, trackingQuality: quality))
    }

    private func person(_ sequence: UInt64, at time: TimeInterval = 0, x: Double = 0, y: Double = 4, screen: CGFloat = 0.5) -> FollowPersonObservation {
        FollowPersonObservation(frameID: ARFrameID(generation: 1, sequence: sequence), timestamp: time, confidence: 0.9, boundingBox: CGRect(x: screen - 0.05, y: 0.4, width: 0.1, height: 0.2), position: Vec2(x, y), pose: Pose2D(position: .zero, yaw: 0))
    }

    private func drain() async { for _ in 0..<30 { await Task.yield() } }

    func testStartupWaitsForHealthyFrameUntilJustBeforeFiveSeconds() async {
        let unhealthy = [frame(1, pose: nil), frame(1, depth: false),
                         frame(1, quality: .limited), frame(1, quality: .unavailable),
                         frame(1, at: -1), frame(1, quality: nil)]
        for event in unhealthy {
            let (coordinator, perception, motion, clock) = setup()
            motion.suspendRotation = true
            _ = await coordinator.start()
            perception.send(event)
            await drain()
            XCTAssertTrue(motion.rotations.isEmpty)
            XCTAssertTrue(motion.goals.isEmpty)
            XCTAssertEqual(motion.stops, 0, "Startup is already stationary")
            clock.advance(to: 4.999)
            await drain()
            XCTAssertEqual(coordinator.state, .searching)
            perception.send(frame(2, at: 4.999))
            await drain()
            XCTAssertNil(coordinator.perceptionIssue)
            XCTAssertEqual(motion.rotations.count, 1, "Readiness starts searching immediately")
            _ = await coordinator.stop()
            motion.releaseRotation()
        }
    }

    func testReadinessFailsBeforeAnyMotion() async {
        for unavailable in 0..<2 {
            let (coordinator, perception, motion, _) = setup()
            if unavailable == 0 { perception.detectorReady = false } else { perception.personLabelAvailable = false }
            let started = await coordinator.start()
            XCTAssertFalse(started)
            XCTAssertTrue(motion.rotations.isEmpty)
            XCTAssertTrue(motion.goals.isEmpty)
        }
    }

    func testStartupDeadlineReportsLatestTrackingReasonAtFiveSeconds() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, depth: false))
        await drain()
        clock.advance(to: 4.9)
        perception.send(.frame(FollowFrameBatch(
            frameID: ARFrameID(generation: 1, sequence: 2), timestamp: 4.9,
            pose: nil, depthAvailable: true, people: [],
            trackingQuality: .limited, trackingReason: .excessiveMotion)))
        await drain()
        XCTAssertEqual(coordinator.state, .searching)
        clock.advance(to: 5)
        await drain()
        XCTAssertEqual(coordinator.state, .failed("AR tracking limited (excessiveMotion). Move the camera slowly and keep it steady."))
        XCTAssertTrue(motion.rotations.isEmpty)
        XCTAssertTrue(motion.goals.isEmpty)
    }

    func testInitialAcquisitionNavigatesToStandOffGoal() async {
        let (coordinator, perception, motion, _) = setup()
        let started = await coordinator.start()
        XCTAssertTrue(started)
        perception.send(frame(1, people: [person(1)]))
        await drain()
        XCTAssertEqual(coordinator.state, .following)
        XCTAssertEqual(motion.goals, [Vec2(0, 2.5)])
    }

    func testMissingPoseOrDepthFailsBeforeMovement() async {
        for missingPose in [false, true] {
            let (coordinator, perception, motion, _) = setup()
            let started = await coordinator.start()
            XCTAssertTrue(started)
            perception.send(frame(1, pose: missingPose ? nil : Pose2D(position: .zero, yaw: 0), depth: missingPose))
            await drain()
            XCTAssertTrue(motion.goals.isEmpty)
            XCTAssertTrue(motion.rotations.isEmpty)
            XCTAssertEqual(coordinator.state, .searching)
        }
    }

    func testSearchStopsAfterTwelveThirtyDegreeRotations() async {
        let (coordinator, perception, motion, _) = setup()
        _ = await coordinator.start()
        perception.send(frame(1))
        await drain()
        for _ in 0..<20 where coordinator.isActive { await drain() }
        XCTAssertEqual(motion.rotations.count, 12)
        XCTAssertTrue(motion.rotations.allSatisfy { abs($0 - .pi / 6) < 0.0001 })
        XCTAssertEqual(coordinator.state, .failed("No person found."))
    }

    func testSearchEvaluatesFramesWhileRotationIsInFlight() async {
        let (coordinator, perception, motion, _) = setup()
        motion.suspendRotation = true
        _ = await coordinator.start()
        perception.send(frame(1))
        await drain()
        XCTAssertEqual(motion.rotations.count, 1)
        perception.send(frame(2, people: [person(2)]))
        await drain()
        XCTAssertEqual(motion.goals, [Vec2(0, 2.5)])
        XCTAssertEqual(motion.rotations.count, 1)
        motion.releaseRotation()
    }

    func testHoldBandAndTooCloseNeverNavigateBackward() async {
        for distance in [1.0, 1.25, 1.5, 1.75] {
            let (coordinator, perception, motion, _) = setup()
            _ = await coordinator.start()
            perception.send(frame(1, people: [person(1, y: distance)]))
            await drain()
            XCTAssertEqual(coordinator.state, .holdingDistance)
            XCTAssertTrue(motion.goals.isEmpty)
        }
    }

    func testGoalReplacementRequiresDisplacementAndThreeHertzAndStopsFirst() async {
        let (coordinator, perception, motion, clock) = setup()
        motion.suspendNavigation = true
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        perception.send(frame(2, people: [person(2, y: 4.5)]))
        await drain()
        XCTAssertEqual(motion.goals.count, 1)
        clock.advance(to: 0.334)
        perception.send(frame(3, at: 0.334, people: [person(3, at: 0.334, y: 4.1)]))
        await drain()
        XCTAssertEqual(motion.goals.count, 1)
        perception.send(frame(4, at: 0.334, people: [person(4, at: 0.334, y: 4.5)]))
        await drain()
        XCTAssertEqual(motion.goals, [Vec2(0, 2.5), Vec2(0, 3)])
        XCTAssertGreaterThanOrEqual(motion.stops, 2)
        motion.releaseNavigation()
    }

    func testLossStopsBeforeReacquisitionRotationAndRejectsCentralNewcomer() async {
        let (coordinator, perception, motion, clock) = setup()
        motion.suspendRotation = true
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        let stops = motion.stops
        perception.send(frame(2))
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring)
        XCTAssertGreaterThan(motion.stops, stops)
        XCTAssertEqual(motion.rotations.count, 0, "Legacy recovery observes stopped")
        clock.advance(to: 0.5)
        perception.send(frame(3, at: 0.5))
        await drain()
        clock.advance(to: 1)
        perception.send(frame(4, at: 1, people: [person(4, at: 1, x: 6, screen: 0.5)]))
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring)
        for step in 3...19 {
            let time = Double(step) / 2
            let sequence = UInt64(step + 2)
            clock.advance(to: time)
            perception.send(frame(sequence, at: time,
                                  people: [person(sequence, at: time, x: 6, screen: 0.5)]))
            await drain()
        }
        clock.advance(to: 9.999)
        perception.send(frame(22, at: 9.999, people: [person(22, at: 9.999, y: 4.2, screen: 0.8)]))
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring, "Provisional detection cannot clear the episode")
        perception.send(frame(23, at: 9.999, people: [person(23, at: 9.999, y: 4.2, screen: 0.8)]))
        await drain()
        XCTAssertEqual(coordinator.state, .following)
        motion.releaseRotation()
    }

    func testReacquisitionExpiresAtTenSeconds() async {
        let (coordinator, perception, motion, clock) = setup()
        motion.suspendRotation = true
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        perception.send(frame(2))
        await drain()
        for step in 1...19 {
            let time = Double(step) / 2
            clock.advance(to: time)
            perception.send(frame(UInt64(step + 2), at: time))
            await drain()
        }
        clock.advance(to: 9.999)
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring)
        let rotationsBeforeDeadline = motion.rotations.count
        let stopsBeforeDeadline = motion.stops
        clock.advance(to: 10)
        perception.send(frame(22, at: 10, people: [person(22, at: 10)]))
        await drain()
        XCTAssertFalse(coordinator.isActive)
        XCTAssertTrue(motion.goals.count == 1)
        XCTAssertEqual(motion.stops, stopsBeforeDeadline + 1)
        motion.releaseRotation()
        await drain()
        clock.advance(to: 10.5)
        perception.send(frame(23, at: 10.5, people: [person(23, at: 10.5)]))
        await drain()
        XCTAssertEqual(motion.rotations.count, rotationsBeforeDeadline, "Late scan completion cannot extend the deadline")
        XCTAssertEqual(motion.goals.count, 1)
    }

    func testReacquisitionScansRequireFreshPerceptionAfterStopAndCompletion() async {
        for delayedStop in [true, false] {
            let (coordinator, perception, motion, clock) = setup()
            motion.suspendRotation = true
            _ = await coordinator.start()
            perception.send(frame(1, people: [person(1)]))
            await drain()
            motion.suspendStop = delayedStop
            perception.send(frame(2))
            await drain()
            XCTAssertEqual(coordinator.state, .reacquiring)
            XCTAssertEqual(motion.rotations.count, 0)
            clock.advance(to: 0.6)
            await drain()
            if delayedStop {
                motion.suspendStop = false
                motion.releaseStop()
            } else {
                motion.releaseRotation()
            }
            await drain()
            XCTAssertEqual(motion.rotations.count, 0,
                           "Neither acknowledgement nor scan completion may launch a stale scan")
            XCTAssertEqual(coordinator.perceptionIssue, .staleFrame)
            let rotations = motion.rotations.count
            perception.send(frame(3, at: 0.6))
            await drain()
            XCTAssertNil(coordinator.perceptionIssue)
            XCTAssertEqual(motion.rotations.count, rotations, "Fresh perception does not certify legacy motion provenance")
            _ = await coordinator.stop()
            motion.releaseRotation()
        }
    }

    func testReacquisitionWithoutFramesStopsAndFailsBeforeTenSecondTargetDeadline() async {
        let (coordinator, perception, motion, clock) = setup()
        motion.suspendRotation = true
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        perception.send(frame(2))
        await drain()
        XCTAssertEqual(motion.rotations.count, 0)
        let stops = motion.stops
        clock.advance(to: 0.501)
        await drain()
        XCTAssertEqual(motion.stops, stops + 1)
        XCTAssertEqual(coordinator.perceptionIssue, .staleFrame)
        motion.releaseRotation()
        await drain()
        XCTAssertEqual(motion.rotations.count, 0)
        clock.advance(to: 2.5)
        await drain()
        XCTAssertTrue(coordinator.isActive)
        clock.advance(to: 2.501)
        await drain()
        XCTAssertEqual(coordinator.state, .failed(FollowPerceptionIssue.staleFrame.message))
        XCTAssertEqual(motion.rotations.count, 0)
        XCTAssertEqual(motion.goals.count, 1)
    }

    func testPoseRecoveryEndsAtTwoSeconds() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        perception.send(frame(2, depth: false))
        await drain()
        clock.advance(to: 1.999)
        await drain()
        XCTAssertTrue(coordinator.isActive)
        clock.advance(to: 2)
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Depth unavailable. Check LiDAR visibility and depth support."))
        XCTAssertGreaterThanOrEqual(motion.stops, 2)
    }

    func testOutageDeadlineExpiresWhileStopIsPendingAndRejectsLateHealthyFrame() async {
        for drainBeforeRelease in [true, false] {
            let (coordinator, perception, motion, clock) = setup()
            _ = await coordinator.start()
            perception.send(frame(1, people: [person(1, y: 2)]))
            await drain()
            motion.suspendStop = true
            clock.advance(to: 0.1)
            perception.send(frame(2, at: 0.1, depth: false))
            await drain()
            XCTAssertEqual(motion.stops, 2)
            clock.advance(to: 2.099)
            await drain()
            XCTAssertTrue(coordinator.isActive)
            clock.advance(to: 2.1)
            perception.send(frame(3, at: 2.1, people: [person(3, at: 2.1, y: 1.5)]))
            if drainBeforeRelease {
                await drain()
                XCTAssertEqual(coordinator.state, .failed(FollowPerceptionIssue.depthUnavailable.message),
                               "Recovery enforcement cannot wait for motor acknowledgement")
            }
            motion.suspendStop = false
            motion.releaseStop()
            await drain()
            XCTAssertEqual(coordinator.state, .failed(FollowPerceptionIssue.depthUnavailable.message),
                           "A late healthy frame cannot cancel an elapsed outage deadline")
            XCTAssertEqual(motion.goals.count, 1)
            XCTAssertTrue(motion.rotations.isEmpty)
            XCTAssertEqual(motion.stops, 3, "Terminal cleanup still confirms stopping")
        }
    }

    func testContinuousOutageStopsOnceAndKeepsOriginalRecoveryDeadline() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        clock.advance(to: 0.1)
        perception.send(frame(2, at: 0.1, depth: false))
        await drain()
        XCTAssertEqual(motion.stops, 1)
        clock.advance(to: 0.5)
        perception.send(frame(3, at: 0.5, pose: nil))
        await drain()
        clock.advance(to: 1.9)
        perception.send(frame(4, at: 1.9, quality: .unavailable))
        await drain()
        XCTAssertEqual(motion.stops, 1, "One continuous outage requests one confirmed stop")
        clock.advance(to: 2.099)
        await drain()
        XCTAssertTrue(coordinator.isActive)
        clock.advance(to: 2.1)
        await drain()
        XCTAssertEqual(coordinator.state, .failed(FollowPerceptionIssue.trackingUnavailable.message))
        XCTAssertTrue(motion.goals.isEmpty)
        XCTAssertTrue(motion.rotations.isEmpty)
    }

    func testUnhealthyFrameDuringWatchdogStopRetainsRecoveryDeadline() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        motion.suspendStop = true
        clock.advance(to: 0.501)
        await drain()
        XCTAssertEqual(motion.stops, 1)
        clock.advance(to: 0.6)
        perception.send(frame(2, at: 0.6, depth: false))
        await drain()
        motion.suspendStop = false
        motion.releaseStop()
        await drain()
        clock.advance(to: 2.5)
        await drain()
        XCTAssertTrue(coordinator.isActive)
        clock.advance(to: 2.501)
        await drain()
        XCTAssertEqual(coordinator.state, .failed(FollowPerceptionIssue.depthUnavailable.message))
        XCTAssertTrue(motion.goals.isEmpty)
    }

    func testHealthyRecoveryAllowsOneStopForASecondOutage() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        perception.send(frame(2, depth: false))
        await drain()
        XCTAssertEqual(motion.stops, 1)
        clock.advance(to: 0.5)
        perception.send(frame(3, at: 0.5, people: [person(3, at: 0.5, y: 1.5)]))
        await drain()
        XCTAssertNil(coordinator.perceptionIssue)
        clock.advance(to: 0.6)
        perception.send(frame(4, at: 0.6, depth: false))
        await drain()
        XCTAssertEqual(motion.stops, 2)
        clock.advance(to: 2)
        await drain()
        XCTAssertTrue(coordinator.isActive, "First outage's cancelled timer must not terminate the second")
        clock.advance(to: 2.6)
        await drain()
        XCTAssertEqual(coordinator.state, .failed(FollowPerceptionIssue.depthUnavailable.message))
    }

    func testHealthySearchFrameDuringWatchdogStopResumesScanOnlyAfterConfirmation() async {
        let (coordinator, perception, motion, clock) = setup()
        motion.suspendRotation = true
        _ = await coordinator.start()
        perception.send(frame(1))
        await drain()
        XCTAssertEqual(motion.rotations.count, 1)
        motion.suspendStop = true
        clock.advance(to: 0.501)
        await drain()
        motion.releaseRotation()
        perception.send(frame(2, at: 0.501))
        await drain()
        XCTAssertEqual(motion.rotations.count, 1, "Pending stop confirmation gates scanning")
        motion.suspendStop = false
        motion.releaseStop()
        await drain()
        XCTAssertEqual(motion.rotations.count, 2, "Healthy recovery resumes searching")
        _ = await coordinator.stop()
        motion.releaseRotation()
    }

    func testFailedStopBlocksFurtherGoalsAndStarts() async {
        let (coordinator, perception, motion, _) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        motion.stopError = true
        perception.send(frame(2))
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Rover stop could not be confirmed."))
        perception.send(frame(3, people: [person(3)]))
        let stopped = await coordinator.stop()
        XCTAssertFalse(stopped)
        let restarted = await coordinator.start()
        XCTAssertFalse(restarted)
        XCTAssertEqual(motion.goals.count, 1)
    }

    func testFailedStopCanBeRetriedWithoutRestartingFollow() async {
        let (coordinator, perception, motion, _) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        motion.stopError = true
        let first = await coordinator.stop()
        XCTAssertFalse(first)
        motion.stopError = false
        let retry = await coordinator.stop()
        XCTAssertTrue(retry)
        XCTAssertEqual(coordinator.state, .stopped)
        XCTAssertEqual(motion.goals.count, 1)
    }

    func testStopAndSafetyFailureInvalidateOldNavigationCompletions() async {
        for safety in [false, true] {
            let (coordinator, perception, motion, _) = setup()
            motion.suspendNavigation = true
            _ = await coordinator.start()
            perception.send(frame(1, people: [person(1)]))
            await drain()
            if safety { motion.safety(.failed(.obstacle)) }
            else { _ = await coordinator.stop() }
            await drain()
            motion.releaseNavigation()
            perception.send(frame(2, people: [person(2)]))
            await drain()
            XCTAssertFalse(coordinator.isActive)
            XCTAssertEqual(motion.goals.count, 1)
            XCTAssertGreaterThanOrEqual(motion.stops, 2)
        }
    }

    func testStopAcknowledgementGatesReplacementAndConcurrentFrames() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        motion.suspendStop = true
        clock.advance(to: 0.5)
        perception.send(frame(2, at: 0.5, people: [person(2, at: 0.5, y: 4.5)]))
        await drain()
        perception.send(frame(3, at: 0.5, people: [person(3, at: 0.5, y: 5)]))
        await drain()
        XCTAssertEqual(motion.stops, 2, "Only one stop may be outstanding")
        XCTAssertEqual(motion.goals.count, 1)
        motion.releaseStop()
        await drain()
        XCTAssertLessThanOrEqual(motion.goals.count, 2)
    }

    func testSuspendedFrameWorkProcessesOnlyNewestPendingFrame() async {
        let perception = FollowPerceptionFake()
        let motion = FollowMotionFake()
        let clock = ManualFollowClock()
        var issues: [String] = []
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config,
            eventSink: { event, fields in
                if event == "follow_perception_unavailable", let issue = fields["issue"] { issues.append(issue) }
            })
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 2)]))
        await drain()
        motion.suspendStop = true
        clock.advance(to: 0.334)
        perception.send(frame(2, at: 0.334, depth: false))
        await drain()
        XCTAssertEqual(motion.stops, 2)
        for sequence in 3...5 {
            clock.advance(to: Double(sequence) / 10 + 0.1)
            perception.send(frame(UInt64(sequence), at: clock.now, pose: nil))
            await drain()
        }
        clock.advance(to: 1.2)
        perception.send(frame(6, at: 1.2, people: [person(6, at: 1.2, y: 1.5)]))
        await drain()
        XCTAssertEqual(motion.goals.count, 1)
        motion.suspendStop = false
        motion.releaseStop()
        await drain()
        XCTAssertEqual(coordinator.state, .holdingDistance)
        XCTAssertNil(coordinator.perceptionIssue)
        XCTAssertEqual(motion.stops, 2)
        XCTAssertFalse(issues.contains("staleFrame"), "Superseded frames must not cause a stale outage")
        XCTAssertFalse(issues.contains("poseUnavailable"), "Obsolete pending frames must not be processed")
        _ = await coordinator.stop()
    }

    func testLifecycleEventsBypassSuspendedFrameStop() async {
        for (event, message) in [(FollowPerceptionEvent.interrupted, "AR session interrupted."),
                                 (.failed("AR session failed."), "AR session failed.")] {
            let (coordinator, perception, motion, clock) = setup()
            _ = await coordinator.start()
            perception.send(frame(1, people: [person(1)]))
            await drain()
            motion.suspendStop = true
            clock.advance(to: 0.334)
            perception.send(frame(2, at: 0.334, people: [person(2, at: 0.334, y: 4.5)]))
            await drain()
            perception.send(frame(3, at: 0.334))
            perception.send(event)
            perception.send(frame(4, at: 0.334, people: [person(4, at: 0.334)]))
            await drain()
            XCTAssertEqual(coordinator.state, .failed(message), "Lifecycle failure must fence motion before acknowledgement")
            XCTAssertEqual(motion.stops, 2, "Terminal cleanup waits for the in-flight confirmation")
            XCTAssertEqual(motion.goals.count, 1)
            motion.suspendStop = false
            motion.releaseStop()
            await drain()
            XCTAssertEqual(coordinator.state, .failed(message))
            XCTAssertEqual(motion.stops, 3, "Terminal stop remains confirmed")
            XCTAssertEqual(motion.goals.count, 1)
        }
    }

    func testLifecycleFailureDuringStopRemainsBlockedWhenConfirmationFails() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        motion.suspendStop = true
        clock.advance(to: 0.334)
        perception.send(frame(2, at: 0.334, people: [person(2, at: 0.334, y: 4.5)]))
        await drain()
        perception.send(.interrupted)
        perception.send(frame(3, at: 0.334, people: [person(3, at: 0.334)]))
        await drain()
        XCTAssertEqual(coordinator.state, .failed("AR session interrupted."))
        motion.stopError = true
        motion.suspendStop = false
        motion.releaseStop()
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Rover stop could not be confirmed."))
        let restarted = await coordinator.start()
        XCTAssertFalse(restarted)
        XCTAssertEqual(motion.stops, 2, "An unconfirmed pending stop cannot be treated as confirmed by terminal cleanup")
        XCTAssertEqual(motion.goals.count, 1)
        motion.stopError = false
        let stopped = await coordinator.stop()
        XCTAssertTrue(stopped)
    }

    func testRestartDiscardsOldPendingFramesAndCancelledTimers() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        motion.suspendStop = true
        clock.advance(to: 0.334)
        perception.send(frame(2, at: 0.334, people: [person(2, at: 0.334, y: 4.5)]))
        await drain()
        perception.send(frame(99, at: 0.334, depth: false))
        await drain()
        let stopping = Task { await coordinator.stop() }
        await drain()
        motion.suspendStop = false
        motion.releaseStop()
        let stopped = await stopping.value
        XCTAssertTrue(stopped)
        let restarted = await coordinator.start()
        XCTAssertTrue(restarted)
        perception.send(frame(1, at: 0.334, people: [person(1, at: 0.334, y: 1.5)]))
        await drain()
        XCTAssertEqual(coordinator.state, .holdingDistance)
        XCTAssertNil(coordinator.perceptionIssue)
        for (index, time) in [0.8, 1.3, 1.8, 2.3, 2.8, 3.3, 3.8, 4.3, 4.8, 5.0, 5.334].enumerated() {
            clock.advance(to: time)
            perception.send(frame(UInt64(index + 2), at: time,
                                  people: [person(UInt64(index + 2), at: time, y: 1.5)]))
            await drain()
        }
        XCTAssertEqual(coordinator.state, .holdingDistance)
        XCTAssertNil(coordinator.perceptionIssue)
        XCTAssertEqual(motion.goals.count, 1)
        _ = await coordinator.stop()
    }

    func testInterruptionStopsAndTeardownIsIdempotent() async {
        let (coordinator, perception, motion, _) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        perception.send(.interrupted)
        await drain()
        XCTAssertEqual(coordinator.state, .failed("AR session interrupted."))
        let stops = motion.stops
        let first = await coordinator.stop()
        let second = await coordinator.stop()
        XCTAssertTrue(first && second)
        XCTAssertEqual(motion.stops, stops)
        XCTAssertEqual(motion.goals.count, 1)
    }

    func testStopDuringSuspendedGoalReplacementPreventsStaleMotion() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        motion.suspendStop = true
        clock.advance(to: 0.5)
        perception.send(frame(2, at: 0.5, people: [person(2, at: 0.5, y: 4.5)]))
        await drain()
        let stopping = Task { await coordinator.stop() }
        await drain()
        XCTAssertEqual(motion.stops, 2, "Terminal stop waits for the outstanding acknowledgement")
        motion.releaseStop()
        await drain()
        motion.releaseStop()
        let stopped = await stopping.value
        XCTAssertTrue(stopped)
        XCTAssertEqual(coordinator.state, .stopped)
        XCTAssertEqual(motion.goals.count, 1)
    }

    func testInhibitingFollowImmediatelyPreventsNewGoalsBeforeStopCompletes() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        motion.suspendStop = true

        coordinator.inhibitMotion()
        XCTAssertFalse(coordinator.isActive)
        clock.advance(to: 1)
        perception.send(frame(2, at: 1, people: [person(2, at: 1, y: 6)]))
        await drain()
        XCTAssertEqual(motion.goals.count, 1)
        motion.releaseStop()
        let stopped = await coordinator.stop()
        XCTAssertTrue(stopped)
    }

    func testStalledCameraStreamStopsMotionBeforeDrivingOnStaleTarget() async {
        let (coordinator, perception, motion, clock) = setup()
        motion.suspendNavigation = true
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        XCTAssertEqual(motion.goals.count, 1)
        let stops = motion.stops

        clock.advance(to: 0.501)
        await drain()

        XCTAssertGreaterThan(motion.stops, stops)
        XCTAssertEqual(motion.goals.count, 1)
        motion.releaseNavigation()
    }

    func testNearAgedFrameWatchdogExpiresFromObservationTimestamp() async {
        for searching in [false, true] {
            let (coordinator, perception, motion, clock) = setup()
            motion.suspendNavigation = true
            motion.suspendRotation = true
            _ = await coordinator.start()
            await drain()
            clock.advance(to: 0.49)
            perception.send(frame(1, people: searching ? [] : [person(1)]))
            await drain()
            XCTAssertEqual(searching ? motion.rotations.count : motion.goals.count, 1)
            let stops = motion.stops
            clock.advance(to: 0.5)
            await drain()
            XCTAssertEqual(motion.stops, stops, "The inclusive freshness boundary remains usable")
            clock.advance(to: 0.501)
            await drain()
            XCTAssertEqual(motion.stops, stops + 1, "Watchdog must use frame expiry, not processing time")
            XCTAssertEqual(coordinator.perceptionIssue, .staleFrame)
            motion.releaseNavigation()
            motion.releaseRotation()
            await drain()
            XCTAssertEqual(motion.rotations.count, searching ? 1 : 0)
            _ = await coordinator.stop()
        }
    }

    func testDelayedMotorStopCannotLaunchGoalFromExpiredFrame() async {
        let (coordinator, perception, motion, clock) = setup()
        motion.suspendStop = true
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        XCTAssertTrue(motion.goals.isEmpty)
        clock.advance(to: 0.6)
        await drain()
        motion.releaseStop()
        await drain()
        XCTAssertTrue(motion.goals.isEmpty)
    }

    func testNoFirstCameraFrameFailsInsteadOfSearchingForever() async {
        let (coordinator, _, motion, clock) = setup()
        _ = await coordinator.start()
        await drain()
        clock.advance(to: 4.999)
        await drain()
        XCTAssertEqual(coordinator.state, .searching)
        clock.advance(to: 5)
        await drain()
        XCTAssertEqual(coordinator.state, .failed("No camera frames received. Check the AR session and camera access."))
        XCTAssertGreaterThanOrEqual(motion.stops, 1)
    }

    func testMissingPoseAndExpiredFrameHaveDistinctActionableFailures() async {
        for missingPose in [true, false] {
            let (coordinator, perception, motion, clock) = setup()
            _ = await coordinator.start()
            perception.send(frame(1, at: missingPose ? 0 : -1,
                                  pose: missingPose ? nil : Pose2D(position: .zero, yaw: 0)))
            await drain()
            clock.advance(to: 5)
            await drain()
            XCTAssertEqual(coordinator.state, .failed(missingPose
                ? "Rover pose unavailable. Wait for AR tracking to recover."
                : "Camera frame stale or timestamp invalid. Check camera delivery and inference latency."))
            XCTAssertTrue(motion.goals.isEmpty)
            XCTAssertTrue(motion.rotations.isEmpty)
        }
    }

    func testRecoveryDeadlineUsesLatestPersistentTrackingIssueAndReason() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        perception.send(frame(2, depth: false))
        await drain()
        clock.advance(to: 1.5)
        perception.send(.frame(FollowFrameBatch(
            frameID: ARFrameID(generation: 1, sequence: 3), timestamp: 1.5,
            pose: nil, depthAvailable: true, people: [],
            trackingQuality: .limited, trackingReason: .initializing)))
        await drain()
        XCTAssertEqual(coordinator.perceptionIssue, .trackingLimited)
        clock.advance(to: 1.9)
        perception.send(.frame(FollowFrameBatch(
            frameID: ARFrameID(generation: 1, sequence: 4), timestamp: 1.9,
            pose: nil, depthAvailable: true, people: [],
            trackingQuality: .limited, trackingReason: .excessiveMotion)))
        await drain()
        clock.advance(to: 2)
        await drain()
        XCTAssertEqual(coordinator.state, .failed("AR tracking limited (excessiveMotion). Move the camera slowly and keep it steady."))
        XCTAssertTrue(motion.goals.isEmpty)
        XCTAssertTrue(motion.rotations.isEmpty)
    }

    func testDiagnosticsAreThrottledAndReportUnavailableReasonChangesAndRecovery() async {
        let perception = FollowPerceptionFake()
        let motion = FollowMotionFake()
        let clock = ManualFollowClock()
        var logs: [(String, [String: String])] = []
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                                             configuration: config,
                                             eventSink: { logs.append(($0, $1)) })
        _ = await coordinator.start()
        for sequence in 1...3 {
            perception.send(.frame(FollowFrameBatch(
                frameID: ARFrameID(generation: 4, sequence: UInt64(sequence)), timestamp: 0,
                pose: nil, depthAvailable: true, people: [], trackingQuality: .limited,
                trackingReason: .initializing, inferenceDuration: 0)))
            await drain()
        }
        XCTAssertEqual(logs.filter { $0.0 == "follow_frame" }.count, 1)
        XCTAssertEqual(logs.filter { $0.0 == "follow_perception_unavailable" }.count, 2, "No frames then limited, not every frame")
        clock.advance(to: 0.2)
        perception.send(.frame(FollowFrameBatch(
            frameID: ARFrameID(generation: 4, sequence: 4), timestamp: 0.1,
            pose: nil, depthAvailable: true, people: [], trackingQuality: .limited,
            trackingReason: .insufficientFeatures, inferenceDuration: 0.08)))
        await drain()
        let changed = logs.last { $0.0 == "follow_perception_unavailable" }?.1
        XCTAssertEqual(changed?["issue"], "trackingLimited")
        XCTAssertEqual(changed?["frame_id"], "4:4")
        XCTAssertEqual(changed?["frame_timestamp"], "0.1")
        XCTAssertEqual(changed?["frame_age_seconds"], "0.1")
        XCTAssertEqual(changed?["tracking_quality"], "limited")
        XCTAssertEqual(changed?["tracking_reason"], "insufficientFeatures")
        XCTAssertEqual(changed?["depth_available"], "true")
        XCTAssertEqual(changed?["inference_duration_seconds"], "0.08")
        perception.send(frame(5, at: 0.2, people: [person(5, at: 0.2, y: 1.5)]))
        await drain()
        XCTAssertNil(coordinator.perceptionIssue)
        XCTAssertEqual(coordinator.state, .holdingDistance)
        XCTAssertEqual(logs.filter { $0.0 == "follow_perception_recovered" }.count, 1)
        perception.send(frame(6, at: 0.2, people: [person(6, at: 0.2, y: 1.5)]))
        await drain()
        XCTAssertEqual(logs.filter { $0.0 == "follow_state" }.count, 2, "Searching and holding, not repeated states")
        clock.advance(to: 0.3)
        perception.send(frame(7, at: 0.3, depth: false))
        await drain()
        clock.advance(to: 1.3)
        perception.send(frame(8, at: 1.3, depth: false))
        await drain()
        let pipeline = logs.filter { $0.0 == "follow_frame" }.compactMap { entry -> [String: Any]? in
            guard let data = entry.1["payload"]?.data(using: .utf8) else { return nil }
            return try? JSONSerialization.jsonObject(with: data) as? [String: Any]
        }
        XCTAssertEqual(pipeline.count, 3, "Initial tracking issue, changed reason, and changed health emit immediately")
        XCTAssertEqual(pipeline.compactMap { $0["monotonic_s"] as? Double }, [0, 0.2, 0.3],
                       "Repeated identical unhealthy evaluations do not consume the healthy periodic allowance")
        _ = await coordinator.stop()
    }

    func testFreshFrameDuringWatchdogStopCannotResurrectOldRecoveryTimer() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 2)]))
        await drain()
        motion.suspendStop = true
        clock.advance(to: 0.501)
        await drain()
        XCTAssertEqual(coordinator.perceptionIssue, .staleFrame)
        perception.send(frame(2, at: 0.501, people: [person(2, at: 0.501, y: 1.5)]))
        await drain()
        XCTAssertNil(coordinator.perceptionIssue)
        motion.suspendStop = false
        motion.releaseStop()
        await drain()
        XCTAssertEqual(coordinator.state, .holdingDistance)
        // Wake cancelled sleepers while continuing to provide usable camera frames.
        for (index, time) in [1.0, 1.5, 2.0, 2.5, 2.501].enumerated() {
            clock.advance(to: time)
            perception.send(frame(UInt64(index + 3), at: time,
                                  people: [person(UInt64(index + 3), at: time, y: 1.5)]))
            await drain()
        }
        XCTAssertEqual(coordinator.state, .holdingDistance)
        XCTAssertNil(coordinator.perceptionIssue)
        _ = await coordinator.stop()
    }

    func testCancelledSafetyFailureDuringReplacementStillStopsFollow() async {
        let (coordinator, perception, motion, clock) = setup()
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1)]))
        await drain()
        motion.suspendStop = true
        clock.advance(to: 0.334)
        perception.send(frame(2, at: 0.334, people: [person(2, at: 0.334, y: 4.5)]))
        await drain()
        motion.safety(.failed(.cancelled))
        perception.send(frame(3, at: 0.334, people: [person(3, at: 0.334, y: 5)]))
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Navigation cancelled."))
        XCTAssertEqual(motion.goals.count, 1)
        motion.suspendStop = false
        motion.releaseStop()
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Navigation cancelled."))
        XCTAssertGreaterThanOrEqual(motion.stops, 3)
    }

    func testFreshnessBoundaryAndUnavailableTrackingRemainFailClosed() async {
        for quality in [ARTrackingQuality.normal, .unavailable] {
            let (coordinator, perception, motion, clock) = setup()
            _ = await coordinator.start()
            clock.advance(to: 0.5)
            perception.send(.frame(FollowFrameBatch(
                frameID: ARFrameID(generation: 1, sequence: 1), timestamp: 0,
                pose: quality == .normal ? Pose2D(position: .zero, yaw: 0) : nil,
                depthAvailable: true, people: [person(1)], trackingQuality: quality)))
            await drain()
            if quality == .normal {
                XCTAssertEqual(coordinator.state, .following)
                XCTAssertEqual(motion.goals.count, 1, "Exactly 0.5 seconds remains usable")
                clock.advance(to: 0.501)
                perception.send(frame(2, people: [person(2)]))
                await drain()
                XCTAssertEqual(coordinator.perceptionIssue, .staleFrame)
            } else {
                XCTAssertEqual(coordinator.perceptionIssue, .trackingUnavailable)
                XCTAssertTrue(motion.goals.isEmpty)
                XCTAssertTrue(motion.rotations.isEmpty)
            }
            _ = await coordinator.stop()
        }
    }

    func testInvalidAndFutureObservationTimestampsCannotBeRescuedByDiagnostics() async {
        for timestamp in [Double.nan, Double.infinity, -Double.infinity, 0.001] {
            let perception = FollowPerceptionFake()
            let motion = FollowMotionFake()
            let clock = ManualFollowClock()
            var config = FollowMeConfiguration()
            config.stationaryPauseSeconds = 0
            config.departureRangeIncrease = 0
            let sink = FollowDiagnosticRecordingSink()
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                configuration: config, eventSink: sink.append)
            _ = await coordinator.start()
            await drain()
            perception.send(frame(1, at: timestamp, people: [person(1, at: timestamp)]))
            await drain()
            XCTAssertEqual(coordinator.perceptionIssue, .staleFrame)
            XCTAssertTrue(motion.rotations.isEmpty)
            XCTAssertTrue(motion.alignments.isEmpty)
            XCTAssertTrue(motion.goals.isEmpty)
            XCTAssertTrue(sink.records.contains { $0.event == "follow_frame" }, "Exercise enabled telemetry too")
            _ = await coordinator.stop()
        }
    }

    func testTenSecondReacquisitionDeadlineFencesRealSuspendedPulseWithoutExtension() async throws {
        let perception = FollowPerceptionFake()
        let clock = ManualFollowClock()
        let sink = FollowDiagnosticRecordingSink()
        let pulseGate = FollowDiagnosticSuspension()
        let source = CoordinatorTurnSource(clock: clock)
        let emitter = FollowDiagnosticEmitter(streamID: "reacquisition-deadline", monotonic: { clock.now },
            utc: { Date(timeIntervalSince1970: clock.now) }, sink: sink.append)
        var sends = 0
        let controller = NavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { nil },
            sendCommand: { _ in sends += 1; await pulseGate.suspend() }, stopRover: {},
            sleep: { duration in await clock.sleep(seconds: duration.secondsValue) },
            now: { Date(timeIntervalSince1970: 100) }, diagnosticEmitter: emitter,
            poseSample: { source.sample }, sourceNow: { clock.now }, sourceStopSnapshot: { source.sample })
        source.onCapture = { sample in
            controller.ingestFollowTurnSource(sample)
            perception.send(self.frame(sample.frameID!.sequence, at: sample.sourceTimestamp!))
        }
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception,
            motion: NavigationFollowMeMotion(navigation: controller), clock: clock,
            configuration: config, eventSink: sink.append)
        _ = await coordinator.start()
        perception.send(frame(1, people: [person(1, y: 1.5)]))
        await drain()
        XCTAssertEqual(coordinator.state, .holdingDistance)
        perception.send(frame(2))
        await drain()
        source.advance(by: 0.301)
        for _ in 0..<1000 where !pulseGate.entered { await Task.yield() }
        XCTAssertTrue(pulseGate.entered, "Reach the real pending burst response, not an obsolete 200ms wait")
        XCTAssertEqual(sends, 1)
        for step in 1...19 {
            let time = Double(step) / 2
            source.advance(by: time - clock.now)
            await drain()
        }
        source.advance(by: 9.999 - clock.now)
        await drain()
        XCTAssertEqual(coordinator.state, .reacquiring)
        clock.advance(to: 10)
        perception.send(frame(source.sequence + 1, at: 10, people: [person(source.sequence + 1, at: 10, y: 1.5)]))
        await drain()
        XCTAssertFalse(coordinator.isActive, "Deadline enforcement does not wait for the pulse")
        XCTAssertTrue(sink.records.contains { $0.event == "follow_scan.cancel" })
        XCTAssertEqual(sends, 1)
        pulseGate.release()
        await drain()
        _ = await coordinator.stop()
        clock.advance(to: 11)
        perception.send(frame(source.sequence + 2, at: 11, people: [person(source.sequence + 2, at: 11, y: 1.5)]))
        await drain()
        let events = try sink.records.filter { $0.fields["payload"] != nil }.map {
            try JSONSerialization.jsonObject(with: Data($0.fields["payload"]!.utf8)) as! [String: Any]
        }
        XCTAssertEqual(events.filter { $0["event"] as? String == "follow_scan.operation_begin" }.count, 1)
        XCTAssertEqual(events.first { $0["event"] as? String == "follow_scan.operation_begin" }?["phase"] as? String, "reacquiring")
        XCTAssertFalse(events.contains { $0["event"] as? String == "follow_scan.pulse_wait_begin" },
            "Expired pending response cannot add a motor wait after drain")
        XCTAssertEqual(events.filter { $0["event"] as? String == "follow_scan.settle_begin" }.count, 1,
            "Only the mandatory initial settle occurred; cancellation cannot enter a post-burst settle")
        XCTAssertEqual(sends, 1, "Neither deadline nor stale completion permits another send")
        XCTAssertEqual(controller.safetyState, .idle)
    }

    func testDetectionAtRealControllerSuspensionsFencesScanBeforeConfirmedAlignment() async throws {
        for boundary in ["send", "pulse", "stop", "settle"] {
            let perception = FollowPerceptionFake()
            let clock = ManualFollowClock()
            let sink = FollowDiagnosticRecordingSink()
            let gate = FollowDiagnosticSuspension()
            let alignmentGate = FollowDiagnosticSuspension()
            let source = CoordinatorTurnSource(clock: clock)
            var remainingWaitEntered = false
            let emitter = FollowDiagnosticEmitter(streamID: boundary, monotonic: { clock.now },
                utc: { Date(timeIntervalSince1970: clock.now) }, sink: { event, fields in
                    sink.append(event, fields: fields)
                    if event == "follow_scan.pulse_wait_begin" { remainingWaitEntered = true }
                })
            var sends = 0
            var stops = 0
            var waits: [Double] = []
            let controller = NavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date(timeIntervalSince1970: 100) },
                sendCommand: { _ in
                    sends += 1
                    if sends == 1, boundary == "send" { await gate.suspend() }
                    if sends == 2 { await alignmentGate.suspend() }
                }, stopRover: {
                    stops += 1
                    if stops == 2, boundary == "stop" { await gate.suspend() }
                }, sleep: { duration in
                    let parts = duration.components
                    let seconds = Double(parts.seconds) + Double(parts.attoseconds) / 1e18
                    waits.append(seconds)
                    if !gate.entered && ((boundary == "pulse" && remainingWaitEntered && stops == 1 && seconds <= 0.080 + 1e-12)
                        || (boundary == "settle" && stops == 2 && seconds <= 0.300)) {
                        await gate.suspend()
                    } else {
                        await clock.sleep(seconds: seconds)
                    }
                }, now: { Date(timeIntervalSince1970: 100) }, diagnosticEmitter: emitter,
                poseSample: { source.sample }, sourceNow: { clock.now }, sourceStopSnapshot: { source.sample })
            source.onCapture = { controller.ingestFollowTurnSource($0) }
            let coordinator = FollowMeCoordinator(perception: perception,
                motion: NavigationFollowMeMotion(navigation: controller), clock: clock, eventSink: sink.append)
            _ = await coordinator.start()
            await drain()
            source.advance(by: 5)
            perception.send(frame(1, at: 5))
            await drain()
            source.advance(by: 0.301)
            for _ in 0..<1000 where sends == 0 { await Task.yield() }
            XCTAssertEqual(sends, 1, boundary)
            if boundary == "stop" || boundary == "settle" {
                // SEND entry is not remaining-wait entry: let the asynchronous ACK read finish
                // before deliberately expiring the burst to reach its stop/settle boundary.
                for _ in 0..<1000 where !remainingWaitEntered { await Task.yield() }
                XCTAssertTrue(remainingWaitEntered, "Reach the original scan's remaining wait: \(boundary)")
                source.advance(by: 0.081)
            }
            for _ in 0..<1000 where !gate.entered { await Task.yield() }
            XCTAssertTrue(gate.entered, "Reach the actual \(boundary) suspension")
            source.advance(by: 0.001)
            perception.send(frame(source.sequence, at: clock.now, people: [person(source.sequence, at: clock.now)]))
            await drain()
            XCTAssertEqual(coordinator.state, .aligning, boundary)
            let captured = try sink.records.filter { $0.event == "follow_scan.cancel" }.map {
                try JSONSerialization.jsonObject(with: Data($0.fields["payload"]!.utf8)) as! [String: Any]
            }
            XCTAssertEqual(captured.count, 1, boundary)
            XCTAssertEqual(captured.first?["cancel_origin"] as? String, "detection", boundary)
            XCTAssertEqual(captured.first?["fenced"] as? Bool, true, boundary)
            XCTAssertEqual(sends, 1, "No alignment send before the suspended scan drains: \(boundary)")
            gate.release()
            await drain()
            for _ in 0..<2 {
                source.advance(by: 0.301)
                perception.send(frame(source.sequence, at: clock.now, people: [person(source.sequence, at: clock.now)]))
                await drain()
            }
            for _ in 0..<1000 where !alignmentGate.entered { await Task.yield() }
            XCTAssertTrue(alignmentGate.entered, "Two deliberate post-stop source frames permit the new alignment")
            let events = try sink.records.filter { $0.fields["payload"] != nil }.map {
                try JSONSerialization.jsonObject(with: Data($0.fields["payload"]!.utf8)) as! [String: Any]
            }
            XCTAssertTrue(events.contains {
                $0["event"] as? String == "follow_scan.stop_response"
                    && $0["stop_origin"] as? String == "detection"
                    && $0["stop_outcome"] as? String == "confirmed"
            }, "Controller confirmation must precede the new alignment: \(boundary)")
            XCTAssertEqual(events.filter { $0["event"] as? String == "follow_scan.operation_begin" && $0["purpose"] as? String == "followScan" }.count, 1)
            XCTAssertEqual(events.filter { $0["event"] as? String == "follow_scan.operation_begin" && $0["purpose"] as? String == "followAlignment" }.count, 1)
            XCTAssertEqual(sends, 2, "One scan send, then one alignment burst")
            let motorWaits = events.filter { $0["event"] as? String == "follow_scan.pulse_wait_begin" }
                .compactMap { $0["requested_wait_s"] as? Double }
            XCTAssertTrue(motorWaits.allSatisfy { $0 > 0 && $0 <= 0.080 + 1e-12 })
            if boundary == "send" { XCTAssertTrue(motorWaits.isEmpty, "Pending send cancellation adds no motor wait") }
            else { XCTAssertEqual(motorWaits.count, 1, "Only the original scan's remaining budget wait entered") }
            XCTAssertFalse(waits.contains { abs($0 - 0.200) < 1e-12 }, "No obsolete 200ms pulse wait")
            coordinator.inhibitMotion()
            let terminalStop = Task { await coordinator.stop() }
            await drain()
            alignmentGate.release()
            _ = await terminalStop.value
            await drain()
            XCTAssertEqual(sends, 2, "Old scan/alignment cannot authorize another command")
            XCTAssertEqual(controller.safetyState, .idle)
        }
    }

    func testDetectionFencesSuspendedAckBeforeQueuedAlignmentTaskCanRun() async {
        let perception = FollowPerceptionFake()
        let clock = ManualFollowClock()
        let source = CoordinatorTurnSource(clock: clock)
        let ackGate = FollowDiagnosticSuspension()
        let sink = FollowDiagnosticRecordingSink()
        var scanSends = 0
        let emitter = FollowDiagnosticEmitter(streamID: "detection-ack-race", monotonic: { clock.now },
            utc: { Date(timeIntervalSince1970: clock.now) }, sink: { event, fields in
                sink.append(event, fields: fields)
                if event == "follow_scan.send_begin" { scanSends += 1 }
            })
        let controller = NavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: {
                if !ackGate.entered { await ackGate.suspend() }
                return nil
            }, sendCommand: { _ in }, stopRover: {},
            sleep: { duration in await clock.sleep(seconds: duration.secondsValue) },
            now: { Date(timeIntervalSince1970: 100) }, diagnosticEmitter: emitter,
            poseSample: { source.sample }, sourceNow: { clock.now }, sourceStopSnapshot: { source.sample })
        source.onCapture = { controller.ingestFollowTurnSource($0) }
        let coordinator = FollowMeCoordinator(perception: perception,
            motion: NavigationFollowMeMotion(navigation: controller), clock: clock, eventSink: { event, fields in
                sink.append(event, fields: fields)
                // Queue the controller's ack return at the observed detection transition,
                // before the asynchronously scheduled alignment/confirmation owners run.
                if event == "follow_state", fields["state"] == "aligning" { ackGate.release() }
            })
        _ = await coordinator.start()
        await drain()
        source.advance(by: 5)
        perception.send(frame(1, at: 5))
        await drain()
        source.advance(by: 0.301)
        for _ in 0..<1000 where !ackGate.entered { await Task.yield() }
        XCTAssertTrue(ackGate.entered, "The queued actual ACK getter is the intended suspension")
        perception.send(frame(4, at: clock.now, people: [person(4, at: clock.now, x: 4, y: 0)]))
        await drain()
        XCTAssertEqual(scanSends, 0, "An eligible detection cannot authorize a queued scan acknowledgement to send")
        ackGate.release()
        _ = await coordinator.stop()
    }
}

/// A bounded, explicit camera capture at each simulated timer/response boundary.
/// Providers only read the stored sample; no capture is generated by a read.
@MainActor
private final class CoordinatorTurnSource {
    let clock: ManualFollowClock
    let position: Vec2
    var yaw = 0.0
    private(set) var sequence: UInt64 = 1
    private(set) var sample: NavigationPoseSample
    var onCapture: ((NavigationPoseSample) -> Void)?

    init(clock: ManualFollowClock, position: Vec2 = .zero) {
        self.clock = clock
        self.position = position
        sample = .init(pose: .init(position: position, yaw: 0), frameID: .init(generation: 1, sequence: 1),
            sourceTimestamp: clock.now, trackingQuality: .normal, source: "synthetic_ar")
    }

    func advance(by seconds: Double, captureLag: Double = 0) {
        guard sequence < 160 else { XCTFail("Finite fixture capture schedule exhausted"); return }
        clock.advance(to: clock.now + max(0.001, seconds))
        sequence += 1
        sample = .init(pose: .init(position: position, yaw: yaw), frameID: .init(generation: 1, sequence: sequence),
            sourceTimestamp: clock.now - captureLag, trackingQuality: .normal, source: "synthetic_ar")
        onCapture?(sample)
    }
}

private extension Duration {
    var secondsValue: Double { Double(components.seconds) + Double(components.attoseconds) / 1e18 }
}

@MainActor
private final class AbsoluteRecoveryMotionFake: FollowMeAbsoluteHeadingMotion, FollowMeSourceClockMotion {
    let legacy = FollowMotionFake()
    var sourceUptime: TimeInterval? { legacy.sourceUptime }
    var sample = NavigationPoseSample(pose: Pose2D(position: .zero, yaw: 0),
        frameID: ARFrameID(generation: 1, sequence: 100), sourceTimestamp: 0, trackingQuality: .normal, source: "synthetic")
    var headings: [Double] = []
    var deltas: [Double] = []
    var autoArrive = false
    var onRecoveryArrival: (() -> Void)?
    var resultWithoutEvidence: NavigationResult = .cancelled
    var overshootAt: Int?
    var recoveryGates: [Int: FollowDiagnosticSuspension] = [:]
    var authorizations: [FollowRecoveryAuthorization] = []
    func recoveryPoseSample() -> NavigationPoseSample { sample }
    func performRecovery(_ request: FollowRecoveryHeadingRequest, context: FollowMotionRequestContext) async -> FollowMotionResult {
        headings.append(request.stageHeading)
        authorizations.append(request.authorization)
        if let gate = recoveryGates[headings.count] { await gate.suspend() }
        if autoArrive, request.authorization.authorized, let pose = sample.pose {
            let error = normalizeAngle(request.stageHeading - pose.yaw)
            let delta = abs(error) <= 7 * .pi / 180 ? 0 : max(-Double.pi / 6, min(Double.pi / 6, error))
            let target = normalizeAngle(pose.yaw + delta)
            deltas.append(delta)
            if headings.count == overshootAt { deltas.append(-0.16) }
            let actual = target
            let before = sample
            sample = .init(pose: Pose2D(position: pose.position, yaw: actual), frameID: before.frameID,
                sourceTimestamp: request.authorization.now(), trackingQuality: .normal, source: "synthetic")
            onRecoveryArrival?()
            return .init(result: .arrived, context: .init(request: context, controllerOperationID: nil, purpose: .followScan, profile: nil),
                failure: nil, stopOutcome: .confirmed, recovery: .init(postStopSource: before, resolutionSource: before,
                    stageHeading: request.stageHeading, segmentHeading: target, requestedDelta: delta, arrivalSource: sample,
                    segmentArrived: abs(normalizeAngle(target - actual)) <= 7 * .pi / 180,
                    stageArrived: abs(normalizeAngle(request.stageHeading - actual)) <= 7 * .pi / 180))
        }
        return .init(result: resultWithoutEvidence, context: .init(request: context, controllerOperationID: nil, purpose: .followScan, profile: nil), failure: nil, stopOutcome: .confirmed)
    }
    func rotateForScan(by angle: Double) async -> NavigationResult { await legacy.rotateForScan(by: angle) }
    func alignTowardPerson(by angle: Double) async -> NavigationResult { await legacy.alignTowardPerson(by: angle) }
    func signalReady() async -> NavigationResult { await legacy.signalReady() }
    func navigate(to goal: Vec2, stoppingAtForwardClearance clearance: Double) async -> NavigationResult {
        await legacy.navigate(to: goal, stoppingAtForwardClearance: clearance)
    }
    func stopAndConfirm() async throws { try await legacy.stopAndConfirm() }
    func safetyStates() -> AsyncStream<NavigationSafetyState> { legacy.safetyStates() }
}
