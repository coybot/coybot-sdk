import XCTest
import RoverNav
@testable import PhroverKit

final class NavigationFollowTurnBurstTests: XCTestCase {
    func testEarlySettleWindowStartsAtActualPostStopCapture() {
        var gate = FollowTurnSourceGate()
        gate.ingest(sample(1, 9.99))
        let fence = gate.fence(at: 10, operationGeneration: 1)
        gate.ingest(sample(2, 10.08))
        gate.ingest(sample(3, 10.11))
        XCTAssertNil(gate.consume(after: fence, at: 10.12, minimumSettle: 0.100),
            "Pre-ACK history cannot turn 30 ms of post-stop captures into 100 ms of evidence")
        gate.ingest(sample(4, 10.19))
        XCTAssertNotNil(gate.consume(after: fence, at: 10.20, minimumSettle: 0.100))
    }

    func testEarlySettleBoundsTheWholePoseWindowNotOnlyDistanceFromAnchor() {
        for yawMotion in [true, false] {
            func observation(_ sequence: UInt64, _ timestamp: Double, _ displacement: Double) -> NavigationPoseSample {
                .init(pose: .init(position: .init(yawMotion ? 0 : displacement, 0), yaw: yawMotion ? displacement : 0),
                    frameID: .init(generation: 1, sequence: sequence), sourceTimestamp: timestamp, trackingQuality: .normal)
            }
            var gate = FollowTurnSourceGate()
            gate.ingest(observation(1, 9.99, 0))
            let fence = gate.fence(at: 10, operationGeneration: 1)
            gate.ingest(observation(2, 10.01, 0))
            gate.ingest(observation(3, 10.07, 0.009))
            gate.ingest(observation(4, 10.12, -0.009))
            XCTAssertNil(gate.consume(after: fence, at: 10.13, minimumSettle: 0.100),
                "Opposite excursions are 0.018 apart even though each is within 0.01 of the anchor")
            gate.ingest(observation(5, 10.18, -0.009))
            gate.ingest(observation(6, 10.24, -0.009))
            XCTAssertNotNil(gate.consume(after: fence, at: 10.25, minimumSettle: 0.100))
        }
    }

    func testEarlySearchSettleRejectsDriftFrameGapsAndDuplicateEvidence() {
        for scenario in ["moving", "gap", "duplicate"] {
            var gate = FollowTurnSourceGate()
            gate.ingest(sample(1, 10))
            let fence = gate.fence(at: 10.01, operationGeneration: 1)
            gate.ingest(sample(2, 10.03))
            if scenario == "moving" {
                gate.ingest(sample(3, 10.09, yaw: 0.02))
                gate.ingest(sample(4, 10.15, yaw: 0.04))
            } else if scenario == "gap" {
                gate.ingest(sample(5, 10.15))
            } else {
                gate.ingest(sample(2, 10.03))
            }
            XCTAssertNil(gate.consume(after: fence, at: 10.16, minimumSettle: 0.100), scenario)
            XCTAssertNotNil(gate.consume(after: fence, at: 10.32, minimumSettle: 0.100),
                "Without stable evidence retain the existing 300 ms path: \(scenario)")
        }
    }

    @MainActor
    func testSearchSettlesEarlyOnlyAfterAdvancingStablePostStopPoses() async {
        var uptime = 10.0
        var snapshot = sample(1, 9.99)
        var stops = 0
        var completed = false
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in
                uptime = FollowTurnBurstTransportScope.authorization!.deadline + 0.001
                snapshot = self.sample(3, uptime - 0.001, yaw: 0.18)
                controller.ingestFollowTurnSource(snapshot)
            }, stopRover: { stops += 1; if stops > 1 { uptime += 0.03 } },
            sleep: { try? await Task.sleep(for: $0) }, poseSample: { snapshot }, sourceNow: { uptime }, sourceStopSnapshot: { snapshot })
        let task = Task {
            let result = await NavigationFollowMeMotion(navigation: controller).perform(.scan(.pi / 18), context:
                .init(sessionGeneration: 1, requestToken: 1, purpose: .followScan, phase: "searching", isSearchSweep: true,
                    allowsAcquisitionRotationWithoutDepth: true))
            completed = true
            return result
        }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.301; snapshot = sample(2, 10.3); controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        let ack = uptime
        for (sequence, offset) in [(UInt64(4), 0.01), (5, 0.07)] {
            uptime = ack + offset; snapshot = sample(sequence, uptime, yaw: 0.18); controller.ingestFollowTurnSource(snapshot)
            for _ in 0..<30 { await Task.yield() }
            XCTAssertFalse(completed, "A single frame or less than 100 ms stability is insufficient")
        }
        uptime = ack + 0.12; snapshot = sample(6, uptime, yaw: 0.1805); controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<2000 where !completed { await Task.yield() }
        XCTAssertTrue(completed, "Observed stationary frames should release the redundant 300 ms wait")
        if !completed {
            uptime = ack + 0.301; snapshot = sample(7, uptime, yaw: 0.1805); controller.ingestFollowTurnSource(snapshot)
        }
        let result = await task.value
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    @MainActor
    func testRecoveryRepeatsSmallMeasuredResponseWithoutRebasingHeadingOrIncreasingBudget() async {
        var uptime = 10.0
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var budgets: [Double] = []
        var targets: [Double] = []
        var stops = 0
        var completed = false
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { command in
                XCTAssertEqual(command.left, -0.25)
                let authority = FollowTurnBurstTransportScope.authorization!
                budgets.append(authority.deadline - authority.sendEntryUptime)
                targets.append(FollowMotionTaskScope.evidence!.targetYaw!)
                guard budgets.count <= 2 else { throw URLError(.timedOut) }
                uptime = authority.sendEntryUptime + 0.030076541681774
                snapshot = self.sample(budgets.count == 1 ? 3 : 5, uptime - 0.001, yaw: budgets.count == 1 ? 0.01 : 0.05)
                controller.ingestFollowTurnSource(snapshot)
            }, stopRover: { stops += 1; if stops > 1 { uptime += 0.031320708303247 } },
            sleep: { try? await Task.sleep(for: $0) }, poseSample: { snapshot }, sourceNow: { uptime },
            sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
        let task = Task {
            let result = await NavigationFollowMeMotion(navigation: controller).perform(.scan(.pi / 18), context:
                .init(sessionGeneration: 1, requestToken: 1, purpose: .followScan, phase: "reacquiring"))
            completed = true
            return result
        }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.301; snapshot = sample(2, 10.3); controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        uptime += 0.301; snapshot = sample(4, uptime - 0.001, yaw: 0.021135333568015113); controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where stops < 3 && !completed { await Task.yield() }
        XCTAssertEqual(budgets.count, 2, "Small valid progress is not a measured minimum-motion failure")
        if budgets.count == 2 {
            uptime += 0.301; snapshot = sample(6, uptime - 0.001, yaw: 0.06); controller.ingestFollowTurnSource(snapshot)
        }
        for _ in 0..<1000 where !completed { await Task.yield() }
        if !completed { task.cancel() }
        let result = await task.value
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(budgets.count, 2)
        XCTAssertTrue(budgets.allSatisfy { abs($0 - 0.025) < 1e-12 })
        XCTAssertTrue(targets.allSatisfy { abs($0 - .pi / 18) < 1e-12 })
        events.continuation.finish()
    }

    @MainActor
    func testHalfTurnSearchUsesExplicitRequestedDirectionAtPiSeam() async {
        for direction in [1.0, -1.0] {
            var uptime = 10.0
            var snapshot = sample(1, 9.99)
            var stops = 0
            var sends = 0
            let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
                plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { command in
                    sends += 1
                    XCTAssertEqual(command.left, -direction * 0.25)
                    XCTAssertEqual(command.right, direction * 0.25)
                    // Only admission/direction is under test; end deterministically.
                    throw URLError(.badURL)
                }, stopRover: { stops += 1 }, sleep: { try? await Task.sleep(for: $0) },
                poseSample: { snapshot }, sourceNow: { uptime }, sourceStopSnapshot: { snapshot })
            let task = Task { await NavigationFollowMeMotion(navigation: controller).perform(.scan(direction * .pi), context:
                .init(sessionGeneration: 1, requestToken: 1, purpose: .followScan, phase: "searching", isSearchSweep: true,
                    allowsAcquisitionRotationWithoutDepth: true)) }
            for _ in 0..<1000 where stops == 0 { await Task.yield() }
            uptime = 10.301; snapshot = sample(2, 10.3); controller.ingestFollowTurnSource(snapshot)
            let result = await task.value
            XCTAssertEqual(sends, 1, "The antipodal wrap must not reject a valid explicit sweep direction")
            XCTAssertEqual(result.result, .failed(.commandFailed))
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
    }

    @MainActor
    func testInitialSearchNoProgressCannotResetOriginalWatchdogBetweenPulses() async {
        var uptime = 10.0
        var date = Date(timeIntervalSince1970: 1000)
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var stops = 0
        var sends = 0
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { date }, sendCommand: { _ in
                sends += 1
                let authority = FollowTurnBurstTransportScope.authorization!
                uptime = authority.deadline + 0.001
                snapshot = self.sample(3, uptime - 0.001, yaw: 0)
                controller.ingestFollowTurnSource(snapshot)
            }, stopRover: { stops += 1 }, sleep: { try? await Task.sleep(for: $0) }, now: { date },
            poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).perform(.scan(.pi / 18), context:
            .init(sessionGeneration: 1, requestToken: 1, purpose: .followScan, phase: "searching", isSearchSweep: true,
                allowsAcquisitionRotationWithoutDepth: true)) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.301; snapshot = sample(2, uptime - 0.001); controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        XCTAssertEqual(stops, 2)
        // Distinct, healthy, post-stop source, but the original progress window
        // expired. Another search pulse must not renew it.
        date = date.addingTimeInterval(2.501)
        uptime += 0.301; snapshot = sample(4, uptime - 0.001); controller.ingestFollowTurnSource(snapshot)
        let result = await task.value
        XCTAssertEqual(result.result, .failed(.stalled))
        XCTAssertEqual(sends, 1)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        events.continuation.finish()
    }

    @MainActor
    func testInitialSearchContinuesAfterRecordedSmallResponseDespiteHTTPOverhead() async {
        // Both 23:12 device attempts: about one degree of net motion, not a
        // coarse turn. Command/stop wall time must not end a directed search.
        for (sendTime, stopTime, firstYaw) in [
            (0.08138474999577738, 0.0341829166864045, 0.017908923542101718),
            (0.028802916669519618, 0.033509958331706, 0.018780268239300346),
            // Already inside the old seven-degree heading tolerance, but the
            // requested ten-degree sweep is not yet covered.
            (0.028802916669519618, 0.033509958331706, 0.10)
        ] {
            var uptime = 10.0
            var snapshot = sample(1, 9.99)
            let events = AsyncStream<NavigationPoseSample>.makeStream()
            var budgets: [Double] = []
            var stops = 0
            var completed = false
            var deadline = 10.0
            var controller: FollowNavigationController!
            controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
                plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { command in
                    XCTAssertEqual(command.left, -0.25)
                    let authority = FollowTurnBurstTransportScope.authorization!
                    deadline = authority.deadline
                    budgets.append(authority.deadline - authority.sendEntryUptime)
                    guard budgets.count <= 2 else { throw URLError(.timedOut) }
                    uptime = authority.sendEntryUptime + sendTime
                    snapshot = self.sample(budgets.count == 1 ? 3 : 5, uptime - 0.001,
                        yaw: budgets.count == 1 ? firstYaw : 0.10)
                    controller.ingestFollowTurnSource(snapshot)
                }, stopRover: {
                    stops += 1
                    if stops > 1 { uptime = max(uptime, deadline) + stopTime }
                }, sleep: { try? await Task.sleep(for: $0) }, poseSample: { snapshot }, sourceNow: { uptime },
                sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
            let task = Task {
                let result = await NavigationFollowMeMotion(navigation: controller).perform(.scan(.pi / 18), context:
                    .init(sessionGeneration: 1, requestToken: 1, purpose: .followScan, phase: "searching", isSearchSweep: true,
                        allowsAcquisitionRotationWithoutDepth: true))
                completed = true
                return result
            }
            for _ in 0..<1000 where stops == 0 { await Task.yield() }
            uptime = 10.301; snapshot = sample(2, uptime - 0.001); controller.ingestFollowTurnSource(snapshot)
            for _ in 0..<1000 where budgets.isEmpty && !completed { await Task.yield() }
            uptime = max(uptime, deadline)
            for _ in 0..<1000 where stops < 2 { try? await Task.sleep(for: .milliseconds(1)) }
            XCTAssertEqual(stops, 2)
            uptime += 0.301; snapshot = sample(4, uptime - 0.001, yaw: firstYaw); controller.ingestFollowTurnSource(snapshot)
            for _ in 0..<1000 where budgets.count < 2 && !completed { await Task.yield() }
            uptime = max(uptime, deadline)
            for _ in 0..<1000 where stops < 3 && !completed { try? await Task.sleep(for: .milliseconds(1)) }
            XCTAssertEqual(budgets.count, 2, "A valid small response must allow another bounded search pulse")
            if budgets.count == 2 {
                uptime += 0.301; snapshot = sample(6, uptime - 0.001, yaw: 0.18); controller.ingestFollowTurnSource(snapshot)
            }
            for _ in 0..<1000 where !completed { await Task.yield() }
            if !completed { task.cancel() }
            let result = await task.value
            XCTAssertEqual(result.result, .arrived)
            XCTAssertEqual(result.stopOutcome, .confirmed)
            XCTAssertEqual(result.searchSweepTravel ?? -1, 0.18, accuracy: 1e-12)
            XCTAssertTrue(budgets.allSatisfy { abs($0 - 0.080) < 1e-12 })
            events.continuation.finish()
        }
    }

    @MainActor
    func testRecordedInitialSearchOvershootCompletesSweepWithoutReverseCorrection() async {
        let result = await recordedSearchSweep()
        XCTAssertEqual(result.result, .arrived, "The requested search arc was swept; precise heading correction is unnecessary")
        XCTAssertNil(result.failure)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(result.searchSweepTravel ?? -1, 0.483189002088996, accuracy: 1e-12)
        XCTAssertEqual(result.searchSweepProgress ?? -1, 0.483189002088996, accuracy: 1e-12)
    }

    @MainActor
    func testSearchSweepSupportsNegativeDirectionButRejectsWrongWayProgress() async {
        let negative = await recordedSearchSweep(direction: -1)
        XCTAssertEqual(negative.result, .arrived)
        XCTAssertEqual(negative.stopOutcome, .confirmed)
        let wrongWay = await recordedSearchSweep(wrongWay: true)
        XCTAssertEqual(wrongWay.result, .failed(.rotationResolutionInsufficient))
        XCTAssertEqual(wrongWay.stopOutcome, .confirmed)
        XCTAssertNil(wrongWay.searchSweepTravel)
    }

    @MainActor
    func testSearchSweepCannotCompleteWithMissingBracketOrFailedStop() async {
        let missing = await recordedSearchSweep(missingBracket: true)
        XCTAssertEqual(missing.result, .failed(.rotationResolutionInsufficient))
        XCTAssertEqual(missing.context.failureCause, .calibrationEvidenceIncomplete)
        XCTAssertNil(missing.searchSweepTravel)
        let failedStop = await recordedSearchSweep(failStop: true)
        XCTAssertNotEqual(failedStop.result, .arrived)
        XCTAssertEqual(failedStop.stopOutcome, .failed)
        XCTAssertNil(failedStop.searchSweepTravel)
    }

    @MainActor
    private func recordedSearchSweep(direction: Double = 1, wrongWay: Bool = false,
                                     missingBracket: Bool = false, failStop: Bool = false) async -> FollowMotionResult {
        // Oct 8 22:16:41 UTC: 10 degrees requested, 27.68 degrees swept,
        // valid AR bracket, HTTP 200 and confirmed stop. Search must keep looking.
        var uptime = 10.0
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var budgets: [Double] = []
        var stops = 0
        var completed = false
        let sign = wrongWay ? -direction : direction
        var deadline = 10.0
        let missingArchive: ((ARFrameID, ARFrameID) -> [FollowTurnBurstPlanner.Sample]?)? = missingBracket ? { _, _ in nil } : nil
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { command in
                XCTAssertEqual(command.left, -direction * 0.25, "A sweep never sends a reverse correction")
                let authorization = FollowTurnBurstTransportScope.authorization!
                deadline = authorization.deadline
                budgets.append(authorization.deadline - authorization.sendEntryUptime)
                guard budgets.count == 1 || (wrongWay && budgets.count == 2) else { throw URLError(.timedOut) }
                uptime = authorization.sendEntryUptime + 0.0522037916816771
                snapshot = self.sample(budgets.count == 1 ? 3 : 5, uptime - 0.01,
                    yaw: (budgets.count == 1 ? sign : direction) * 0.1)
                controller.ingestFollowTurnSource(snapshot)
            }, stopRover: {
                stops += 1
                if stops > 1 { uptime = max(uptime, deadline) + 0.03533458334277384 }
                if failStop && stops > 1 { throw URLError(.timedOut) }
            }, sleep: { try? await Task.sleep(for: $0) }, poseSample: { snapshot },
            sourceNow: { uptime }, sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot }, turnPoseEvidence: missingArchive)
        let task = Task {
            let result = await NavigationFollowMeMotion(navigation: controller).perform(.scan(direction * .pi / 18),
                context: .init(sessionGeneration: 1, requestToken: 2, purpose: .followScan, phase: "searching",
                    scanResponseSeed: .init(sourceGeneration: 1, responseRate: 3.9459105709998488), isSearchSweep: true,
                    allowsAcquisitionRotationWithoutDepth: true))
            completed = true
            return result
        }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300
        snapshot = sample(2, 10.2)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where budgets.isEmpty && !completed { await Task.yield() }
        uptime = max(uptime, deadline)
        for _ in 0..<1000 where stops < 2 { try? await Task.sleep(for: .milliseconds(1)) }
        XCTAssertGreaterThanOrEqual(stops, 2)
        uptime += 0.310
        snapshot = sample(4, uptime - 0.01, yaw: sign * 0.483189002088996)
        controller.ingestFollowTurnSource(snapshot)
        if wrongWay {
            for _ in 0..<1000 where stops < 3 && !completed { await Task.yield() }
            if budgets.count == 2 {
                uptime += 0.310
                snapshot = sample(6, uptime - 0.01, yaw: direction * 0.483189002088996)
                controller.ingestFollowTurnSource(snapshot)
            }
        }
        for _ in 0..<1000 where !completed { await Task.yield() }
        if !completed { task.cancel() }
        let result = await task.value
        XCTAssertEqual(budgets.count, 1)
        XCTAssertEqual(budgets.first ?? -1, 0.080, accuracy: 1e-12)
        events.continuation.finish()
        return result
    }

    @MainActor
    func testRecordedSearchOverrunStopsBeforeSubmillisecondCorrection() async {
        // Oct 8 20:34:47 UTC: a 4.755 ms request took 28.712 ms to return,
        // then 33.759 ms to stop, with 0.502688 rad of observed response.
        var uptime = 10.0
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var budgets: [Double] = []
        var stops = 0
        var completed = false
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in
                let authorization = FollowTurnBurstTransportScope.authorization!
                budgets.append(authorization.deadline - authorization.sendEntryUptime)
                // Bound the old behavior: any attempted correction fails this replay.
                guard budgets.count == 1 else { throw URLError(.timedOut) }
                uptime = authorization.sendEntryUptime + 0.028712
                snapshot = self.sample(3, uptime - 0.01, yaw: 0.1)
                controller.ingestFollowTurnSource(snapshot)
            }, stopRover: {
                stops += 1
                if stops > 1 { uptime += 0.033759375 }
            }, sleep: { try? await Task.sleep(for: $0) }, poseSample: { snapshot },
            sourceNow: { uptime }, sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
        let task = Task {
            let result = await NavigationFollowMeMotion(navigation: controller).perform(.scan(.pi / 18),
                context: .init(sessionGeneration: 1, requestToken: 2, purpose: .followScan, phase: "searching",
                    scanResponseSeed: .init(sourceGeneration: 1, responseRate: 11.012353425931373)))
            completed = true
            return result
        }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300
        snapshot = sample(2, 10.2)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        XCTAssertEqual(stops, 2)
        uptime += 0.301
        snapshot = sample(4, uptime - 0.01, yaw: 0.5026881913644705)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where !completed { await Task.yield() }
        if !completed { task.cancel() }
        let result = await task.value
        XCTAssertEqual(budgets.count, 1, "Measured transport/stop overhead must rule out an ineffective tiny correction")
        XCTAssertEqual(budgets.first ?? -1, 0.004754649213903299, accuracy: 1e-12)
        XCTAssertEqual(result.result, .failed(.rotationResolutionInsufficient))
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertNotEqual(result.context.failureCause, .burstPreSendExpired,
            "Resolve while stopped, before transport scheduling fails")
        events.continuation.finish()
    }

    @MainActor
    func testHeldPreSendAdmissionAckCannotDelayWatchdogCompletion() async {
        await checkHeldPreSendAdmissionAck(scenario: "watchdog")
    }

    @MainActor
    func testHeldPreSendAdmissionAckCannotDelayRecoveryEpisodeCompletion() async {
        await checkHeldPreSendAdmissionAck(scenario: "episode")
    }

    @MainActor
    func testHeldPreSendAdmissionAckCannotDelayExplicitStopConfirmation() async {
        await checkHeldPreSendAdmissionAck(scenario: "explicit_stop")
    }

    @MainActor
    private func checkHeldPreSendAdmissionAck(scenario: String) async {
        var uptime = 10.0
        var date = Date(timeIntervalSince1970: 1000)
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var reads: [CheckedContinuation<Void, Never>] = []
        var sends = 0
        var stops = 0
        var completed = false
        var stopCompleted = false
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: {
                if controller.state == .driving, sends == 0 {
                    await withCheckedContinuation { reads.append($0) }
                }
                return date
            }, sendCommand: { _ in sends += 1 }, stopRover: { stops += 1 },
            sleep: { try? await Task.sleep(for: $0) }, now: { date }, poseSample: { snapshot },
            sourceNow: { uptime }, sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
        let motion = NavigationFollowMeMotion(navigation: controller)
        let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 1,
            deadline: 10.5, now: { uptime }, canContinue: { true })
        let task = Task {
            let result = await FollowRecoveryScope.$authorization.withValue(scenario == "episode" ? authorization : nil) {
                await motion.alignTowardPerson(by: 0.5)
            }
            completed = true
            return result
        }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300; snapshot = sample(2, 10.2); controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where reads.isEmpty { await Task.yield() }
        XCTAssertEqual(reads.count, 1, "Hold the admission read after initial stopped-source selection")
        XCTAssertEqual(controller.state, .driving)
        XCTAssertEqual(sends, 0, "This is PRE-SEND admission, not an active remaining-budget read")
        XCTAssertNil(controller.followTurnBurstPendingStatus)
        let stop: Task<Void, Never>?
        if scenario == "explicit_stop" {
            stop = Task { try? await controller.stopAndConfirm(); stopCompleted = true }
        } else {
            stop = nil
            if scenario == "watchdog" { date = Date(timeIntervalSince1970: 1002.5) }
            uptime = scenario == "episode" ? 10.501 : 10.340
            snapshot = sample(3, uptime - 0.001) // Healthy flat source cannot renew target progress.
            controller.ingestFollowTurnSource(snapshot)
        }
        for _ in 0..<1000 where !completed || (stop != nil && !stopCompleted) { await Task.yield() }
        XCTAssertTrue(completed, "\(scenario): terminal completion must precede getter release")
        XCTAssertEqual(stops, scenario == "explicit_stop" ? 2 : 1)
        if stop != nil { XCTAssertTrue(stopCompleted, "Explicit STOP must not drain read-only ACK work") }
        XCTAssertEqual(sends, 0)

        // RED cleanup; GREEN deliberately retains the old read into a new owner's admission.
        var oldReleased = false
        if !completed || (stop != nil && !stopCompleted) {
            reads.first?.resume(); oldReleased = true
        }
        let result = await task.value
        await stop?.value
        XCTAssertEqual(result, scenario == "watchdog" ? .failed(.stalled) : .cancelled)
        date = Date(timeIntervalSince1970: 1003)
        var replacementCompleted = false
        let replacement = Task {
            let result = await motion.alignTowardPerson(by: 0.5)
            replacementCompleted = true
            return result
        }
        let replacementStops = scenario == "explicit_stop" ? 3 : 2
        for _ in 0..<1000 where stops < replacementStops { await Task.yield() }
        uptime += 0.301; snapshot = sample(4, uptime - 0.001); controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where reads.count < 2 { await Task.yield() }
        XCTAssertEqual(reads.count, 2)
        let newFence = controller.followTurnStopFence?.identity
        if !oldReleased { reads.first?.resume() }
        for _ in 0..<40 { await Task.yield() }
        XCTAssertFalse(replacementCompleted, "Old completion cannot complete the new owner's held read")
        XCTAssertEqual(controller.state, .driving)
        XCTAssertEqual(controller.followTurnStopFence?.identity, newFence)
        XCTAssertEqual(stops, replacementStops)
        XCTAssertEqual(sends, 0, "Late old ACK cannot admit motion")
        replacement.cancel()
        for _ in 0..<1000 where !replacementCompleted { await Task.yield() }
        XCTAssertTrue(replacementCompleted, "New owner cancellation also must not join its ACK read")
        reads.dropFirst().first?.resume()
        let replacementResult = await replacement.value
        XCTAssertEqual(replacementResult, .cancelled)
        XCTAssertEqual(stops, replacementStops + 1)
        XCTAssertEqual(sends, 0)
        events.continuation.finish()
    }

    @MainActor
    func testStoppedSourceWatchdogAndRecoveryExpiryFinishWhileAckReadRemainsHeld() async throws {
        for recovery in [false, true] {
            var uptime = 10.0
            var date = Date(timeIntervalSince1970: 1000)
            let snapshot = sample(1, 9.99)
            var feedback: CheckedContinuation<Void, Never>?
            var completed = false
            var stops = 0
            let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
                plan: { _, _ in nil }, lastAckAt: {
                    await withCheckedContinuation { feedback = $0 }; return date
                }, sendCommand: { _ in XCTFail("Stopped source cannot send") }, stopRover: { stops += 1 },
                sleep: { try? await Task.sleep(for: $0) }, now: { date }, poseSample: { snapshot },
                sourceNow: { uptime }, sourceStopSnapshot: { snapshot })
            _ = try await controller.executeFollowTurnBurst(.init(left: -0.25, right: 0.25),
                requestedBudget: 0, purpose: .followScan)
            let fence = try XCTUnwrap(controller.followTurnStopFence)
            var progress = FollowTurnWaitingProgress()
            _ = progress.watchdog.observe(distanceToGoal: 0.5, now: date, commanded: true)
            let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 1,
                deadline: 10.2, now: { uptime }, canContinue: { true })
            let task = Task {
                let result = await FollowRecoveryScope.$authorization.withValue(recovery ? authorization : nil) {
                    await controller.awaitFollowTurnSource(after: fence, progress: progress)
                }
                completed = true
                return result
            }
            for _ in 0..<1000 where feedback == nil { await Task.yield() }
            XCTAssertNotNil(feedback)
            uptime = 10.21
            if !recovery { date = Date(timeIntervalSince1970: 1002.5) }
            controller.ingestFollowTurnSource(sample(2, 10.20))
            for _ in 0..<1000 where !completed { await Task.yield() }
            XCTAssertTrue(completed, "Deadline must finish while the read is still held: recovery=\(recovery)")
            feedback?.resume(); feedback = nil
            let result = await task.value
            switch result {
            case .cancelled: XCTAssertTrue(recovery)
            case .failed(let reason): XCTAssertFalse(recovery); XCTAssertEqual(reason, .stalled)
            case .sample: XCTFail("Expired source wait cannot arrive")
            }
            XCTAssertEqual(stops, 1)
        }
    }

    @MainActor
    func testHeldActiveAckReadCannotDelayCrossingExpiryWatchdogOrCancellationStop() async {
        for scenario in ["crossing", "expiry", "watchdog", "cancellation"] {
            var uptime = 10.0
            var date = Date(timeIntervalSince1970: 1000)
            var snapshot = sample(1, 9.99)
            let events = AsyncStream<NavigationPoseSample>.makeStream()
            var feedback: CheckedContinuation<Void, Never>?
            var held = false
            var sends = 0
            var stops = 0
            var budgetTimers: [CheckedContinuation<Void, Never>] = []
            var controller: FollowNavigationController!
            controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
                plan: { _, _ in nil }, lastAckAt: {
                    if sends == 1, !held {
                        held = true
                        await withCheckedContinuation { feedback = $0 }
                    }
                    return date
                }, sendCommand: { _ in sends += 1; uptime = 10.330 }, stopRover: { stops += 1 },
                sleep: { duration in
                    if scenario == "expiry", sends == 1, stops == 1 {
                        await withCheckedContinuation { budgetTimers.append($0) }
                    } else { try? await Task.sleep(for: duration) }
                }, now: { date }, poseSample: { snapshot },
                sourceNow: { uptime }, sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
            let task = Task { await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.5) }
            for _ in 0..<1000 where stops == 0 { await Task.yield() }
            uptime = 10.300; snapshot = sample(2, 10.2); controller.ingestFollowTurnSource(snapshot)
            for _ in 0..<1000 where feedback == nil { await Task.yield() }
            XCTAssertNotNil(feedback, scenario)
            XCTAssertNil(controller.followTurnBurstPendingStatus, "Actual SEND has returned")
            if scenario == "expiry" {
                for _ in 0..<1000 where budgetTimers.isEmpty { await Task.yield() }
                XCTAssertFalse(budgetTimers.isEmpty)
            }
            uptime = scenario == "expiry" ? 10.381 : 10.340
            if scenario == "watchdog" { date = Date(timeIntervalSince1970: 1002.5) }
            if scenario == "cancellation" { task.cancel() }
            if scenario == "expiry" {
                let timers = budgetTimers; budgetTimers.removeAll()
                for timer in timers { timer.resume() }
            } else {
                snapshot = sample(3, uptime - 0.001, yaw: scenario == "crossing" ? 0.5 : 0)
                controller.ingestFollowTurnSource(snapshot)
            }
            for _ in 0..<1000 where stops < 2 { await Task.yield() }
            XCTAssertEqual(stops, 2, "\(scenario): stop must be admitted BEFORE releasing the unrelated read")
            // RED cleanup also releases the uncooperative getter, then supplies strict stopped source.
            uptime = 10.600; feedback?.resume(); feedback = nil
            for timer in budgetTimers { timer.resume() }; budgetTimers.removeAll()
            for _ in 0..<1000 where stops < 2 { await Task.yield() }
            uptime = 10.901; snapshot = sample(4, 10.90, yaw: 0.5); controller.ingestFollowTurnSource(snapshot)
            let result = await task.value
            XCTAssertEqual(result, scenario == "watchdog" ? .failed(.stalled) :
                (scenario == "cancellation" ? .cancelled : .arrived), scenario)
            XCTAssertEqual(sends, 1, "Late read cannot resend")
            XCTAssertEqual(stops, 2, "Read completion cannot append another stop")
            events.continuation.finish()
        }
    }

    @MainActor
    func testDetectionSynchronouslyFencesAlignmentDuringPendingResponse() async throws {
        var uptime = 10.0
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var response: CheckedContinuation<Void, Never>?
        var sends = 0
        var stops = 0
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in
                sends += 1
                await withCheckedContinuation { response = $0 }
            }, stopRover: { stops += 1 }, sleep: { try? await Task.sleep(for: $0) },
            poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream },
            sourceStopSnapshot: { snapshot })
        let motion = NavigationFollowMeMotion(navigation: controller)
        let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 1,
            purpose: .followAlignment, phase: "aligning")
        let task = Task { await motion.perform(.alignment(0.4), context: context) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.31
        snapshot = sample(2, 10.30)
        events.continuation.yield(snapshot)
        for _ in 0..<1000 where response == nil { await Task.yield() }
        XCTAssertEqual(sends, 1)
        motion.inhibitScanContinuation(origin: .detection)
        uptime = 10.32
        response?.resume()
        response = nil
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        uptime = 10.63
        snapshot = sample(3, 10.62, yaw: 0.4)
        events.continuation.yield(snapshot)
        let result = await task.value
        XCTAssertEqual(result.result, .cancelled, "Detection during alignment cannot publish old arrival")
        XCTAssertEqual(sends, 1)
        try await motion.stopAndConfirm()
        events.continuation.finish()
    }

    @MainActor
    func testPendingFeedbackSuspensionCannotPauseIndependentBudgetAndWallDeadline() async {
        var uptime = 10.0
        var date = Date(timeIntervalSince1970: 1000)
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var response: CheckedContinuation<Void, Never>?
        var feedback: CheckedContinuation<Void, Never>?
        var timers: [CheckedContinuation<Void, Never>] = []
        var held = false
        var stops = 0
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: {
                if controller.followTurnBurstPendingStatus != nil, !held {
                    held = true
                    await withCheckedContinuation { feedback = $0 }
                }
                return date
            }, sendCommand: { _ in await withCheckedContinuation { response = $0 } },
            stopRover: { stops += 1 }, sleep: { duration in
                let seconds = Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
                if controller.followTurnBurstPendingStatus != nil, seconds <= 0.0800001 {
                    await withCheckedContinuation { timers.append($0) }
                } else { try? await Task.sleep(for: duration) }
            }, now: { date }, poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream },
            sourceStopSnapshot: { snapshot })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.5) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300
        snapshot = sample(2, 10.2)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where feedback == nil || response == nil || timers.isEmpty { await Task.yield() }
        XCTAssertNotNil(feedback)
        XCTAssertNotNil(response)
        XCTAssertFalse(timers.isEmpty, "A suspended actual getter cannot own the budget/watchdog wake task")
        date = Date(timeIntervalSince1970: 1002.5)
        uptime = 10.400
        let pending = timers; timers.removeAll()
        for timer in pending { timer.resume() }
        for _ in 0..<30 { await Task.yield() }
        XCTAssertEqual(controller.followTurnBurstPendingStatus?.stopObligation, true,
            "Deadline must inhibit while both sender and actual feedback getter remain suspended")
        XCTAssertEqual(stops, 1)
        feedback?.resume()
        response?.resume()
        for timer in timers { timer.resume() }; timers.removeAll()
        let result = await task.value
        XCTAssertEqual(result, .failed(.stalled))
        XCTAssertEqual(stops, 2)
        events.continuation.finish()
    }

    @MainActor
    func testActiveFeedbackAwaitCannotResetWallCheckpointOrAddRemainingMotorWait() async {
        var uptime = 10.0
        var date = Date(timeIntervalSince1970: 1000)
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var feedback: CheckedContinuation<Void, Never>?
        var held = false
        var sends = 0
        var stops = 0
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: {
                if sends == 1, !held {
                    held = true
                    await withCheckedContinuation { feedback = $0 }
                }
                return Date(timeIntervalSince1970: 1000)
            }, sendCommand: { _ in sends += 1; uptime = 10.330 }, stopRover: { stops += 1 },
            sleep: { try? await Task.sleep(for: $0) }, now: { date }, poseSample: { snapshot },
            sourceNow: { uptime }, sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.5) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300
        snapshot = sample(2, 10.2)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where feedback == nil { await Task.yield() }
        XCTAssertNotNil(feedback)
        date = Date(timeIntervalSince1970: 1002.5)
        snapshot = sample(3, 10.32)
        controller.ingestFollowTurnSource(snapshot)
        feedback?.resume()
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        XCTAssertEqual(stops, 2)
        XCTAssertEqual(controller.followTurnStopFence?.acknowledgementUptime, 10.330,
            "Guard suspension consumes the original wall epoch; no remaining 50ms motor wait")
        uptime = 10.400 // Ensure any failed fixture path can drain its synthetic timer.
        controller.ingestFollowTurnSource(snapshot)
        let result = await task.value
        XCTAssertEqual(result, .failed(.stalled), "Watchdog expiry precedes post-await stale ACK evaluation")
        XCTAssertEqual(sends, 1)
        events.continuation.finish()
    }

    @MainActor
    func testAttemptGateRejectsHealthyCaptureRegressingToStopAcknowledgement() async {
        var uptime = 10.0
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var stops = 0
        var attempted = false
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in
                // A queue/actor suspension can replace latest source after admission.
                // This capture is healthy/age-valid, but equal to the stop ACK.
                snapshot = self.sample(3, 10.0)
                controller.ingestFollowTurnSource(snapshot)
                let authority = FollowTurnBurstTransportScope.authorization!
                XCTAssertNil(authority.authorizeAttempt, "Production attempts must not hop back to MainActor")
                attempted = authority.isAuthorized()
                XCTAssertFalse(attempted, "Attempt authority must retain the strict stop capture fence")
                throw FollowTurnBurstTransportDenial.fenced
            }, stopRover: { stops += 1 }, sleep: { try? await Task.sleep(for: $0) },
            poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream },
            sourceStopSnapshot: { snapshot })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.5) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300
        snapshot = sample(2, 10.2)
        controller.ingestFollowTurnSource(snapshot)
        let result = await task.value
        XCTAssertEqual(result, .failed(.trackingLost))
        XCTAssertFalse(attempted)
        XCTAssertEqual(stops, 2)
        events.continuation.finish()
    }

    @MainActor
    func testPendingProgressUsesActualTargetErrorAndRepeatedFramesCannotRenewCheckpoint() async {
        var uptime = 10.0
        var date = Date(timeIntervalSince1970: 1000)
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var response: CheckedContinuation<Void, Never>?
        var stops = 0
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { date }, sendCommand: { _ in
                await withCheckedContinuation { response = $0 }
            }, stopRover: { stops += 1 }, sleep: { try? await Task.sleep(for: $0) }, now: { date },
            poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream },
            sourceStopSnapshot: { snapshot })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.5) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300
        snapshot = sample(2, 10.2)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where response == nil { await Task.yield() }
        date = Date(timeIntervalSince1970: 1002)
        uptime = 10.320
        snapshot = sample(3, 10.31, yaw: 0.1)
        controller.ingestFollowTurnSource(snapshot) // Genuine 0.1-rad goal progress at Date 1002.
        date = Date(timeIntervalSince1970: 1003)
        controller.ingestFollowTurnSource(snapshot) // Same capture, not a new progress checkpoint.
        for _ in 0..<30 { await Task.yield() }
        XCTAssertEqual(controller.followTurnBurstPendingStatus?.stopObligation, false,
            "Genuine progress retains its own epoch beyond the initial 2.5s boundary")
        date = Date(timeIntervalSince1970: 1004.5)
        uptime = 10.340
        snapshot = sample(4, 10.33, yaw: -0.1) // Actual error grows; sampled travel is not goal progress.
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<30 { await Task.yield() }
        XCTAssertEqual(controller.followTurnBurstPendingStatus?.stopObligation, true)
        uptime = 10.400
        response?.resume()
        let result = await task.value
        XCTAssertEqual(result, .failed(.stalled))
        XCTAssertEqual(stops, 2)
        events.continuation.finish()
    }

    @MainActor
    func testFirstFollowBurstRetainsExistingNoFreshAckRequirementUntilSenderEntry() async {
        var uptime = 10.0
        let date = Date(timeIntervalSince1970: 1000)
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var sends = 0
        var stops = 0
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { 0 },
            plan: { _, _ in nil }, lastAckAt: { sends == 0 ? date.addingTimeInterval(-3) : date },
            sendCommand: { _ in
                sends += 1
                uptime = 10.400
                snapshot = self.sample(3, 10.39, yaw: 0.5)
                controller.ingestFollowTurnSource(snapshot)
            }, stopRover: { stops += 1 }, sleep: { try? await Task.sleep(for: $0) }, now: { date },
            poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream },
            sourceStopSnapshot: { snapshot })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.5) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300
        snapshot = sample(2, 10.2)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        XCTAssertEqual(sends, 1, "Rotation's first command historically permits a stale ACK")
        uptime = 10.701
        snapshot = sample(4, 10.6, yaw: 0.5)
        controller.ingestFollowTurnSource(snapshot)
        let result = await task.value
        XCTAssertEqual(result, .arrived)
        events.continuation.finish()
    }

    @MainActor
    func testActualAckGetterRevalidatesCommsBeforeRemainingBudgetWait() async {
        var uptime = 10.0
        let date = Date(timeIntervalSince1970: 1000)
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var sends = 0
        var stops = 0
        var activeAckReads = 0
        var stopTimes: [Double] = []
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { 0 },
            plan: { _, _ in nil }, lastAckAt: {
                if sends > 0 { activeAckReads += 1; return date.addingTimeInterval(-3) }
                return date
            }, sendCommand: { _ in sends += 1; uptime = 10.330 },
            stopRover: { stops += 1; stopTimes.append(uptime) }, sleep: { try? await Task.sleep(for: $0) },
            now: { date }, poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream },
            sourceStopSnapshot: { snapshot })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.5) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300
        snapshot = sample(2, 10.2)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        XCTAssertGreaterThan(activeAckReads, 0, "Active burst safety must consult the actual ACK getter")
        XCTAssertEqual(stops, 2, "Stale comms must stop at 30ms, without adding the remaining 50ms")
        XCTAssertEqual(stopTimes.last ?? -1, 10.330, accuracy: 1e-12)
        // Release the old path's synthetic budget without weakening the assertions.
        uptime = 10.400
        snapshot = sample(3, 10.39)
        controller.ingestFollowTurnSource(snapshot)
        let result = await task.value
        XCTAssertEqual(result, .failed(.commsLost))
        XCTAssertEqual(sends, 1)
        events.continuation.finish()
    }

    @MainActor
    func testOriginalWallWatchdogInhibitsPendingSendBeforeResponseAndDrainsStop() async {
        var uptime = 10.0
        var date = Date(timeIntervalSince1970: 1000)
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var response: CheckedContinuation<Void, Never>?
        var sends = 0
        var stops = 0
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { date }, sendCommand: { _ in
                sends += 1
                await withCheckedContinuation { response = $0 }
            }, stopRover: { stops += 1 }, sleep: { try? await Task.sleep(for: $0) }, now: { date },
            poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream },
            sourceStopSnapshot: { snapshot })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.5) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300
        snapshot = sample(2, 10.2)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where response == nil { await Task.yield() }
        XCTAssertNotNil(response)
        // Independent injected Date clock: preserve the existing wall watchdog,
        // never compare its epoch with the source/budget uptime clock.
        date = Date(timeIntervalSince1970: 1002.5)
        uptime = 10.330
        snapshot = sample(3, 10.32)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<30 { await Task.yield() }
        XCTAssertEqual(controller.followTurnBurstPendingStatus?.stopObligation, true,
            "2.5s without 0.05rad progress must inhibit pending transport even before its uptime budget expires")
        XCTAssertEqual(stops, 1, "Watchdog cannot overtake the actual pending sender")
        uptime = 10.400 // Drain both RED and GREEN without leaving a synthetic budget timer running.
        response?.resume()
        let result = await task.value
        XCTAssertEqual(result, .failed(.stalled))
        XCTAssertEqual(stops, 2)
        XCTAssertEqual(sends, 1)
        XCTAssertEqual(controller.followTurnStopFence?.acknowledgementUptime, 10.400)
        events.continuation.finish()
    }

    @MainActor
    func testRecoveredHealthyEndpointCannotHideInvalidPendingCalibrationSource() async {
        var uptime = 10.0
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var sends = 0
        var stops = 0
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in
                sends += 1
                uptime = 10.340
                snapshot = self.sample(3, 10.33, yaw: 0.1, quality: .unavailable)
                controller.ingestFollowTurnSource(snapshot)
                uptime = 10.380
                snapshot = self.sample(4, 10.37, yaw: 0.17)
                controller.ingestFollowTurnSource(snapshot)
            }, stopRover: { stops += 1; if stops > 1 { uptime += 0.01 } },
            sleep: { try? await Task.sleep(for: $0) }, poseSample: { snapshot }, sourceNow: { uptime },
            sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.5) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300
        snapshot = sample(2, 10.2)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        uptime = 10.701
        snapshot = sample(5, 10.6, yaw: 0.18)
        controller.ingestFollowTurnSource(snapshot)
        let result = await task.value
        // Runtime health now fails at the actual invalid ingress, before the
        // calibration reducer. Recovery of the cache cannot erase that failure.
        XCTAssertEqual(result, .failed(.trackingLost))
        XCTAssertEqual(sends, 1, "A later valid stopped frame cannot authorize learning from an unhealthy bracket")
        XCTAssertEqual(stops, 2)
        events.continuation.finish()
    }

    @MainActor
    func testMeasuredOvershootAllowsOnlySmallerCorrectionAfterConfirmedStop() async {
        var uptime = 10.0
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var sends = 0
        var budgets: [Double] = []
        var stops = 0
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { command in
                sends += 1
                let authorization = FollowTurnBurstTransportScope.authorization!
                budgets.append(authorization.deadline - authorization.sendEntryUptime)
                XCTAssertEqual(stops, sends, "Each command follows confirmed stopping")
                XCTAssertEqual(command.left, sends == 1 ? -0.25 : 0.25)
                uptime = authorization.deadline
                snapshot = self.sample(sends == 1 ? 3 : 5, uptime - 0.01, yaw: sends == 1 ? 0.25 : 0.1)
                controller.ingestFollowTurnSource(snapshot)
            }, stopRover: { stops += 1; if stops > 1 { uptime += 0.001 } },
            sleep: { try? await Task.sleep(for: $0) }, poseSample: { snapshot }, sourceNow: { uptime },
            sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.1) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300
        snapshot = sample(2, 10.2)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        XCTAssertEqual(stops, 2)
        uptime = 10.711
        snapshot = sample(4, 10.6, yaw: 0.25)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where stops < 3 { await Task.yield() }
        XCTAssertEqual(sends, 2)
        XCTAssertLessThan(budgets.last ?? .infinity, (budgets.first ?? 0) / 2)
        uptime = 11.122
        snapshot = sample(6, 11.02, yaw: 0.1)
        controller.ingestFollowTurnSource(snapshot)
        let result = await task.value
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(sends, 2)
        events.continuation.finish()
    }

    @MainActor
    func testSecondAlignmentBurstUsesWholeResponseOnceAndKeepsTighterTarget() async {
        var uptime = 10.0
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var budgets: [Double] = []
        var targets: [Double] = []
        var stops = 0
        var completed = false
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { command in
                XCTAssertEqual(command.left, -0.25)
                XCTAssertEqual(command.right, 0.25)
                let authorization = FollowTurnBurstTransportScope.authorization!
                budgets.append(authorization.deadline - authorization.sendEntryUptime)
                targets.append(FollowMotionTaskScope.evidence!.context.targetYaw!)
                uptime = authorization.sendEntryUptime + (budgets.count == 1 ? 0.080 : 0.100)
                snapshot = self.sample(budgets.count == 1 ? 3 : 6, uptime - 0.02,
                    yaw: budgets.count == 1 ? 0.30 : 0.5)
                controller.ingestFollowTurnSource(snapshot)
            }, stopRover: {
                stops += 1
                if stops > 1 { uptime += 0.010 }
            }, sleep: { try? await Task.sleep(for: $0) }, poseSample: { snapshot },
            sourceNow: { uptime }, sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
        let task = Task {
            let result = await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.5)
            completed = true
            return result
        }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        uptime = 10.300
        snapshot = sample(2, 10.2)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        XCTAssertEqual(stops, 2)
        // Both endpoints are genuinely captured after the 10.390 stop ACK.
        uptime = 10.430
        snapshot = sample(4, 10.42, yaw: 0.30)
        controller.ingestFollowTurnSource(snapshot)
        uptime = 10.701
        snapshot = sample(5, 10.65, yaw: 0.31)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<1000 where budgets.count < 2 && !completed { await Task.yield() }
        XCTAssertEqual(budgets.count, 2, "A valid completed bracket must enable a corrective burst")
        XCTAssertEqual(budgets.first ?? -1, 0.080, accuracy: 1e-12)
        // .31 rad across a .09 s command/stop window, with .01 s overhead.
        XCTAssertEqual(budgets.dropFirst().first ?? -1, 0.0306451612903226, accuracy: 1e-12)
        XCTAssertEqual(targets, [0.5, 0.5], "Calibration must not rebase the frozen target")
        if budgets.count == 2 {
            for _ in 0..<1000 where stops < 3 { await Task.yield() }
            let acknowledgement = uptime
            uptime = acknowledgement + 0.301
            snapshot = sample(7, acknowledgement + 0.2, yaw: 0.5)
            controller.ingestFollowTurnSource(snapshot)
        } else { task.cancel() }
        let result = await task.value
        XCTAssertEqual(result, .arrived)
        events.continuation.finish()
    }

    @MainActor
    func testProductionAbsoluteRecoveryResolvesThirtyDegreeSegmentAfterStrictSourceGate() async {
        var uptime = 10.0
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var response: CheckedContinuation<Void, Never>?
        var seenTarget: Double?
        var budget: Double?
        var wheels = 0
        var stops = 0
        let expectedTarget = 0.9235987755982988 // real stopped yaw 0.4 + a maximum 30-degree segment
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in
                wheels += 1
                seenTarget = FollowMotionTaskScope.evidence?.context.targetYaw
                budget = FollowTurnBurstTransportScope.authorization.map { $0.deadline - $0.sendEntryUptime }
                await withCheckedContinuation { response = $0 }
            }, stopRover: { stops += 1 }, sleep: { try? await Task.sleep(for: $0) },
            poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream },
            sourceStopSnapshot: { snapshot })
        let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 1,
            deadline: 20, now: { uptime }, canContinue: { true })
        let request = FollowRecoveryHeadingRequest(stageHeading: 1.2, authorization: authorization)
        let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 1,
            purpose: .followScan, phase: "reacquiring")
        let task = Task { await NavigationFollowMeMotion(navigation: controller).performRecovery(request, context: context) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        for _ in 0..<30 { await Task.yield() }
        XCTAssertEqual(wheels, 0)
        uptime = 10.300
        snapshot = sample(2, 10.2, yaw: 0.4)
        events.continuation.yield(snapshot)
        for _ in 0..<1000 where response == nil { await Task.yield() }
        XCTAssertNotNil(response)
        XCTAssertEqual(seenTarget ?? -1, expectedTarget, accuracy: 1e-12)
        XCTAssertEqual(budget ?? -1, 0.080, accuracy: 1e-12)
        uptime = 10.400
        snapshot = sample(3, 10.39, yaw: seenTarget ?? expectedTarget)
        events.continuation.yield(snapshot)
        response?.resume()
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        uptime = 10.701
        snapshot = sample(4, 10.6, yaw: seenTarget ?? expectedTarget)
        events.continuation.yield(snapshot)
        let result = await task.value
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(result.context.targetYaw ?? -1, expectedTarget, accuracy: 1e-12)
        XCTAssertEqual(result.recovery?.requestedDelta ?? -1, .pi / 6, accuracy: 1e-12)
        XCTAssertEqual(result.recovery?.resolutionSource?.frameID?.sequence, 2)
        XCTAssertEqual(result.recovery?.arrivalSource?.frameID?.sequence, 4)
        XCTAssertEqual(result.recovery?.segmentArrived, true)
        XCTAssertEqual(result.recovery?.stageArrived, false)
        XCTAssertEqual(result.turnStopFence?.acknowledgementUptime, 10.400)
        XCTAssertEqual(wheels, 1)
        XCTAssertEqual(stops, 2)
        events.continuation.finish()
    }
    @MainActor
    func testProductionScanUsesEightyMillisecondBudgetSevenDegreeToleranceAndFreshStop() async {
        let angle = Double.pi / 6
        var uptime = 10.0
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var response: CheckedContinuation<Void, Never>?
        var budget: Double?
        var wheels: [WheelCommand] = []
        var stops = 0
        var ackReturned = false
        var motorWaitsAfterAck: [Double] = []
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { command in
                wheels.append(command)
                budget = FollowTurnBurstTransportScope.authorization.map { $0.deadline - $0.sendEntryUptime }
                await withCheckedContinuation { response = $0 }
                ackReturned = true
            }, stopRover: { stops += 1 }, sleep: { duration in
                let seconds = Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
                if ackReturned, stops < 2 { motorWaitsAfterAck.append(seconds) }
                try? await Task.sleep(for: duration)
            }, poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream },
            sourceStopSnapshot: { snapshot })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: angle) }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        for _ in 0..<30 { await Task.yield() }
        XCTAssertTrue(wheels.isEmpty)
        uptime = 10.300
        snapshot = sample(2, 10.2)
        events.continuation.yield(snapshot)
        for _ in 0..<1000 where response == nil { await Task.yield() }
        XCTAssertNotNil(response)
        XCTAssertEqual(budget ?? -1, 0.080, accuracy: 1e-12)
        XCTAssertEqual(wheels.first?.left, -0.25)
        XCTAssertEqual(wheels.first?.right, 0.25)
        uptime = 10.400
        snapshot = sample(3, 10.39, yaw: angle + 0.1)
        events.continuation.yield(snapshot)
        response?.resume()
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        XCTAssertEqual(stops, 2)
        XCTAssertEqual(motorWaitsAfterAck, [])
        uptime = 10.701
        snapshot = sample(4, 10.6, yaw: angle + 0.1)
        events.continuation.yield(snapshot)
        let result = await task.value
        XCTAssertEqual(result, .arrived, "0.1 rad is within scan's unchanged 7 degrees, not alignment's 0.05 rad")
        XCTAssertEqual(wheels.count, 1)
        XCTAssertEqual(stops, 2)
        events.continuation.finish()
    }
    @MainActor
    func testProductionAlignmentNearThreePointSevenDegreesUsesShortBurstAndFreshStoppedArrival() async {
        let angle = 3.7 * Double.pi / 180
        var uptime = 10.0
        var snapshot = sample(1, 9.99)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var response: CheckedContinuation<Void, Never>?
        var budgets: [Double?] = []
        var wheels: [WheelCommand] = []
        var stops = 0
        var ackReturned = false
        var motorWaitsAfterAck: [Double] = []
        var completed = false
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { command in
                wheels.append(command)
                budgets.append(FollowTurnBurstTransportScope.authorization.map { $0.deadline - $0.sendEntryUptime })
                await withCheckedContinuation { response = $0 }
                ackReturned = true
            }, stopRover: { stops += 1 }, sleep: { duration in
                let seconds = Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
                if ackReturned, stops < 2 { motorWaitsAfterAck.append(seconds) }
                try? await Task.sleep(for: duration)
            }, poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream },
            sourceStopSnapshot: { snapshot })
        let motion = NavigationFollowMeMotion(navigation: controller)
        let task = Task { let result = await motion.alignTowardPerson(by: angle); completed = true; return result }
        for _ in 0..<1000 where stops == 0 { await Task.yield() }
        for _ in 0..<30 { await Task.yield() }
        XCTAssertTrue(wheels.isEmpty, "Initial confirmed stop must settle and consume actual post-ACK source before alignment")
        uptime = 10.300
        snapshot = sample(2, 10.2)
        events.continuation.yield(snapshot)
        for _ in 0..<1000 where response == nil { await Task.yield() }
        XCTAssertNotNil(response)
        XCTAssertEqual(wheels.count, 1)
        XCTAssertEqual(wheels.first?.left, -0.25)
        XCTAssertEqual(wheels.first?.right, 0.25)
        XCTAssertEqual(budgets.first.flatMap { $0 } ?? -1, 0.00696009186955, accuracy: 1e-12,
            "A real 3.7-degree follow alignment uses the provisional excess budget, without a 20ms floor")
        uptime = 10.400
        snapshot = sample(3, 10.39, yaw: angle)
        events.continuation.yield(snapshot)
        response?.resume()
        for _ in 0..<1000 where stops < 2 { await Task.yield() }
        XCTAssertEqual(stops, 2, "100ms ACK must immediately enter serialized stop")
        XCTAssertEqual(motorWaitsAfterAck, [], "No continuous cadence or 200ms motor wait after expired ACK")
        XCTAssertFalse(completed, "Crossing/in-tolerance pending evidence cannot certify arrival")
        uptime = 10.701
        snapshot = sample(4, 10.6, yaw: angle)
        events.continuation.yield(snapshot)
        let result = await task.value
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(wheels.count, 1)
        XCTAssertEqual(stops, 2, "Arrival retains its strict post-stop source; no invalidating wrapper stop")
        events.continuation.finish()
    }
    @MainActor
    func testBurstReturnCarriesItsActualConfirmedStopBoundaryThroughLaterWrapperStop() async throws {
        var uptime = 10.0
        var snapshot = sample(10, 9.99)
        var stops = 0
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in uptime = 10.100 },
            stopRover: {
                stops += 1
                uptime = stops == 1 ? 10.125 : 10.400
                snapshot = self.sample(UInt64(10 + stops), uptime - 0.01)
            }, sleep: { _ in XCTFail("Expired burst adds no wait") }, poseSample: { snapshot },
            sourceNow: { uptime }, sourceStopSnapshot: { snapshot })
        let receipt = try await controller.executeFollowTurnBurst(.init(left: -0.25, right: 0.25),
            requestedBudget: 0.080, purpose: .followScan)
        XCTAssertEqual(receipt.confirmedStopFence?.acknowledgementUptime, 10.125)
        XCTAssertEqual(receipt.confirmedStopFence?.highestSequence, 11)
        XCTAssertEqual(receipt.confirmedStopFence?.highestSourceTimestamp, 10.115)
        let identity = receipt.confirmedStopFence?.identity
        try await FollowTurnSourceScope.$required.withValue(true) { try await controller.stopAndConfirm() }
        XCTAssertEqual(controller.followTurnStopFence?.acknowledgementUptime, 10.400)
        XCTAssertNotEqual(controller.followTurnStopFence?.identity, identity)
        XCTAssertEqual(receipt.confirmedStopFence?.acknowledgementUptime, 10.125,
            "A returned confirmation is immutable, never a lookup of the later latest stop")
    }
    @MainActor
    func testUnhealthyPendingSourceInhibitsBurstWithoutConcurrentMotorCommand() async {
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        let initial = sample(1, 9.99)
        var response: CheckedContinuation<Void, Never>?
        var stops = 0
        let context = FollowMotionOperationContext(request: nil, controllerOperationID: 1,
            purpose: .followScan, profile: RoverConfig.followScanRotationProfile, requestedRotation: 0.5)
        let evidence = FollowMotionOperationEvidence(context: context)
        evidence.targetYaw = 0.5
        let controller = FollowNavigationController(currentPose: { initial.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in
                await withCheckedContinuation { response = $0 }
            }, stopRover: { stops += 1 }, sleep: { try? await Task.sleep(for: $0) },
            poseSample: { initial }, sourceNow: { 10 }, sourceEvents: { events.stream })
        let send = Task {
            await FollowMotionTaskScope.$evidence.withValue(evidence) {
                await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25),
                    requestedBudget: 0.080, purpose: .followScan)
            }
        }
        for _ in 0..<1000 where response == nil { await Task.yield() }
        controller.ingestFollowTurnSource(sample(2, 10, quality: .unavailable))
        XCTAssertEqual(controller.followTurnBurstPendingStatus?.stopObligation, true,
            "Actual health loss must synchronously inhibit further transport attempts")
        XCTAssertEqual(stops, 0)
        response?.resume()
        let receipt = await send.value
        XCTAssertTrue(receipt.stopObligation)
        events.continuation.finish()
    }
    @MainActor
    func testAdvancingSourceDuringRemainingWaitStopsWithoutWaitingForBudgetTimer() async throws {
        var uptime = 10.0
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        let initial = sample(1, 9.99)
        var response: CheckedContinuation<Void, Never>?
        var budgetTimers: [CheckedContinuation<Void, Never>] = []
        var responded = false
        var stopTimes: [Double] = []
        let context = FollowMotionOperationContext(request: nil, controllerOperationID: 1,
            purpose: .followAlignment, profile: nil, requestedRotation: 0.3)
        let evidence = FollowMotionOperationEvidence(context: context)
        evidence.targetYaw = 0.3
        let controller = FollowNavigationController(currentPose: { initial.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in
                await withCheckedContinuation { response = $0 }
            }, stopRover: { stopTimes.append(uptime) }, sleep: { duration in
                if responded, !Task.isCancelled {
                    await withCheckedContinuation { budgetTimers.append($0) }
                } else { try? await Task.sleep(for: duration) }
            }, poseSample: { initial }, sourceNow: { uptime }, sourceEvents: { events.stream })
        let turn = Task {
            try await FollowMotionTaskScope.$evidence.withValue(evidence) {
                try await controller.executeFollowTurnBurst(.init(left: -0.25, right: 0.25),
                    requestedBudget: 0.080, purpose: .followAlignment)
            }
        }
        for _ in 0..<1000 where response == nil { await Task.yield() }
        for _ in 0..<30 { await Task.yield() }
        uptime = 10.030
        responded = true
        response?.resume()
        for _ in 0..<1000 where budgetTimers.isEmpty { await Task.yield() }
        XCTAssertFalse(budgetTimers.isEmpty)
        uptime = 10.040
        controller.ingestFollowTurnSource(sample(2, 10.035, yaw: 0.28))
        controller.ingestFollowTurnSource(sample(3, 10.036, yaw: 0.1))
        for _ in 0..<1000 where stopTimes.isEmpty { await Task.yield() }
        XCTAssertEqual(stopTimes, [10.040], "A source trigger must stop before the deadline timer returns")
        // Release the fake timer even on RED; no leaked continuation or timed-out test.
        uptime = 10.080
        for timer in budgetTimers { timer.resume() }
        _ = try await turn.value
        events.continuation.finish()
    }
    @MainActor
    func testPendingSourceTriggerIsRetainedWhenNextFrameLeavesToleranceBeforeAck() async {
        var uptime = 10.0
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var response: CheckedContinuation<Void, Never>?
        let initial = sample(1, 9.99)
        let context = FollowMotionOperationContext(request: nil, controllerOperationID: 1,
            purpose: .followAlignment, profile: nil, requestedRotation: 0.3)
        let evidence = FollowMotionOperationEvidence(context: context)
        evidence.targetYaw = 0.3
        let controller = FollowNavigationController(currentPose: { initial.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in
                await withCheckedContinuation { response = $0 }
            }, stopRover: { XCTFail("A source observer cannot send a motor command") },
            sleep: { try? await Task.sleep(for: $0) }, poseSample: { initial },
            sourceNow: { uptime }, sourceEvents: { events.stream })
        let send = Task {
            await FollowMotionTaskScope.$evidence.withValue(evidence) {
                await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25),
                    requestedBudget: 0.080, purpose: .followAlignment)
            }
        }
        for _ in 0..<1000 where response == nil { await Task.yield() }
        for _ in 0..<30 { await Task.yield() }
        uptime = 10.030
        controller.ingestFollowTurnSource(sample(2, 10.01, yaw: 0.28))
        controller.ingestFollowTurnSource(sample(3, 10.02, yaw: 0.1))
        for _ in 0..<1000 where controller.followTurnBurstPendingStatus?.stopObligation != true { await Task.yield() }
        XCTAssertEqual(controller.followTurnBurstPendingStatus?.stopObligation, true,
            "All ingested events, including a brief valid trigger, must be consumed before cache replacement")
        response?.resume()
        let receipt = await send.value
        XCTAssertTrue(receipt.stopObligation)
        events.continuation.finish()
    }
    @MainActor
    func testThirtyMillisecondAckWaitsOnlyRemainingFiftyMillisecondsBeforeStop() async throws {
        var uptime = 10.0
        var response: CheckedContinuation<Void, Never>?
        var responded = false
        var remainingWaits: [Double] = []
        var stopTimes: [Double] = []
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in
                await withCheckedContinuation { response = $0 }
            }, stopRover: { stopTimes.append(uptime) }, sleep: { duration in
                let seconds = Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
                if responded, !Task.isCancelled {
                    remainingWaits.append(seconds)
                    uptime += seconds
                } else {
                    try? await Task.sleep(for: duration)
                }
            }, sourceNow: { uptime })
        let turn = Task { try await controller.executeFollowTurnBurst(.init(left: -0.25, right: 0.25),
            requestedBudget: 0.080, purpose: .followAlignment) }
        for _ in 0..<1000 where response == nil { await Task.yield() }
        for _ in 0..<30 { await Task.yield() }
        uptime = 10.030
        responded = true
        response?.resume()
        _ = try await turn.value
        XCTAssertEqual(remainingWaits.count, 1)
        XCTAssertEqual(remainingWaits.first ?? 0, 0.050, accuracy: 1e-12,
            "ACK-relative full 80ms or historical 200ms waits are forbidden")
        XCTAssertEqual(stopTimes.count, 1)
        XCTAssertEqual(stopTimes.first ?? 0, 10.080, accuracy: 1e-12)
    }
    @MainActor
    func testHundredMillisecondAckForEightyMillisecondBurstStopsWithZeroAddedWait() async throws {
        var uptime = 10.0
        var response: CheckedContinuation<Void, Never>?
        var responded = false
        var waitsAfterAck: [Double] = []
        var stops = 0
        var commands = 0
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in
                commands += 1
                await withCheckedContinuation { response = $0 }
            }, stopRover: {
                XCTAssertTrue(responded, "Stop cannot overtake the real sender return")
                stops += 1
            }, sleep: { duration in
                if responded {
                    waitsAfterAck.append(Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18)
                }
                try? await Task.sleep(for: duration)
            }, sourceNow: { uptime })
        let turn = Task { try await controller.executeFollowTurnBurst(.init(left: -0.25, right: 0.25),
            requestedBudget: 0.080, purpose: .followScan) }
        for _ in 0..<1000 where response == nil { await Task.yield() }
        XCTAssertNotNil(response)
        XCTAssertEqual(stops, 0)
        uptime = 10.100
        responded = true
        response?.resume()
        let receipt = try await turn.value
        XCTAssertEqual(receipt.send.sendEntryUptime, 10)
        XCTAssertEqual(receipt.send.deadline, 10.080)
        XCTAssertEqual(receipt.send.responseUptime, 10.100)
        XCTAssertEqual(waitsAfterAck, [], "Expired budget adds no deliberate motor wait after ACK")
        XCTAssertEqual(stops, 1, "The burst executor must confirm the serialized stop")
        XCTAssertEqual(commands, 1)
    }
    func testDirectedCrossingUnwrapsBothWaysWithoutFalsePiSeamTrigger() throws {
        for sign in [-1.0, 1.0] {
            let startYaw = sign * 3.0
            let target = FollowReacquisitionPlanner.wrap(startYaw + sign * 0.5)
            var observer = try XCTUnwrap(FollowTurnBurstObservation(targetYaw: target, tolerance: 0.05,
                start: sample(1, 10, yaw: startYaw), uptime: 10))
            XCTAssertFalse(observer.observe(sample(2, 10.01, yaw: -sign * 3.1), at: 10.01),
                "Crossing the yaw representation seam is not crossing the target")
            XCTAssertTrue(observer.observe(sample(3, 10.02,
                yaw: FollowReacquisitionPlanner.wrap(startYaw + sign * 0.65)), at: 10.02),
                "An outside-tolerance overshoot in either direction must end the burst")
        }
        var exactPi = try XCTUnwrap(FollowTurnBurstObservation(targetYaw: .pi, tolerance: 0.05,
            start: sample(1, 10), uptime: 10))
        XCTAssertFalse(exactPi.observe(sample(2, 10.01, yaw: -2.9), at: 10.01))
        XCTAssertTrue(exactPi.observe(sample(3, 10.02, yaw: 3.0), at: 10.02),
            "Exact pi selects the negative directed target, then unwraps across -pi")
        var positivePi = try XCTUnwrap(FollowTurnBurstObservation(targetYaw: .pi, tolerance: 0,
            start: sample(1, 10), uptime: 10, preferPositiveHalfTurn: true))
        XCTAssertFalse(positivePi.observe(sample(2, 10.01, yaw: 1.6), at: 10.01))
        XCTAssertTrue(positivePi.observe(sample(3, 10.02, yaw: -3.0), at: 10.02),
            "An explicitly positive half-turn must stop on its directed source crossing")
    }
    @MainActor
    func testAdvancingToleranceSourceWhileSendPendingMarksObligationWithoutConcurrentStop() async {
        var uptime = 10.0
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var response: CheckedContinuation<Void, Never>?
        var commands = 0
        var stops = 0
        let initial = sample(1, 9.99)
        let context = FollowMotionOperationContext(request: nil, controllerOperationID: 1,
            purpose: .followAlignment, profile: nil, requestedRotation: 0.3)
        let evidence = FollowMotionOperationEvidence(context: context)
        evidence.targetYaw = 0.3
        let controller = FollowNavigationController(currentPose: { initial.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { command in
                commands += 1
                XCTAssertEqual(command.left, -0.25)
                XCTAssertEqual(command.right, 0.25)
                await withCheckedContinuation { response = $0 }
            }, stopRover: { stops += 1 }, sleep: { try? await Task.sleep(for: $0) },
            poseSample: { initial }, sourceNow: { uptime }, sourceEvents: { events.stream })
        let send = Task {
            await FollowMotionTaskScope.$evidence.withValue(evidence) {
                await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25),
                    requestedBudget: 0.080, purpose: .followAlignment)
            }
        }
        for _ in 0..<1000 where response == nil { await Task.yield() }
        XCTAssertNotNil(response)
        uptime = 10.030
        events.continuation.yield(sample(2, 10.02, yaw: 0.26))
        for _ in 0..<1000 where controller.followTurnBurstPendingStatus?.stopObligation != true { await Task.yield() }
        XCTAssertEqual(controller.followTurnBurstPendingStatus?.stopObligation, true,
            "An advancing in-tolerance source must inhibit pending-send retries before ACK")
        XCTAssertEqual(stops, 0, "Source monitor cannot own motors or overtake the pending send")
        XCTAssertEqual(commands, 1)
        let stop = Task { try? await controller.stopAndConfirm() }
        for _ in 0..<30 { await Task.yield() }
        XCTAssertEqual(stops, 0, "Authoritative stop drains the actual pending sender")
        response?.resume()
        let receipt = await send.value
        _ = await stop.value
        XCTAssertTrue(receipt.stopObligation)
        XCTAssertEqual(receipt.responseUptime, 10.030)
        XCTAssertEqual(stops, 1)
        XCTAssertEqual(commands, 1)
        events.continuation.finish()
    }
    @MainActor
    func testNewWrapperStopInvalidatesEarlierFenceEvenWithSameOperationOwner() async {
        var uptime = 10.0
        var stops = 0
        var snapshot = sample(1, 9.9)
        var earlier: FollowTurnStopFence?
        var controller: FollowNavigationController!
        controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in XCTFail("Zero turn cannot send") },
            stopRover: {
                stops += 1
                if stops == 3 { uptime = 10.3; snapshot = self.sample(2, 10.2) }
            }, sleep: { _ in XCTFail("Stale stop identity must exit before waiting") },
            poseSample: { snapshot }, sourceNow: { uptime }, sourceStopSnapshot: {
                if stops == 3 { earlier = controller.followTurnStopFence }
                return snapshot
            })
        // Exercise actual wrapper confirmations directly: a zero-angle follow
        // operation now correctly waits for fresh post-stop source before arrival.
        for _ in 0..<3 {
            _ = try? await controller.executeFollowTurnBurst(.init(left: -0.25, right: 0.25),
                requestedBudget: 0, purpose: .followScan)
        }
        guard let earlier else { XCTFail("Missing earlier actual confirmation"); return }
        XCTAssertEqual(earlier.operationGeneration, controller.followTurnStopFence?.operationGeneration)
        XCTAssertNotEqual(earlier.identity, controller.followTurnStopFence?.identity)
        let result = await controller.awaitFollowTurnSource(after: earlier, progress: .init())
        guard case .cancelled = result else { XCTFail("A later actual stop must invalidate earlier source authority"); return }
    }
    @MainActor
    func testStoppedWaitUsesOriginalWatchdogDeadlineBeforeSettleCompletes() async {
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        let snapshot = sample(1, 9.9)
        var uptime = 10.0
        var date = Date(timeIntervalSince1970: 2.4)
        var waits: [Double] = []
        var progress = FollowTurnWaitingProgress()
        progress.distanceToGoal = 1
        progress.hasSentCommand = true
        _ = progress.watchdog.observe(distanceToGoal: 1, now: Date(timeIntervalSince1970: 0), commanded: true)
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { date }, sendCommand: { _ in XCTFail("Stopped wait cannot send") },
            stopRover: {}, sleep: { duration in
                let seconds = Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
                waits.append(seconds)
                uptime += seconds
                date = Date(timeIntervalSince1970: 2.5)
            }, now: { date }, poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream })
        let result = await controller.prepareFollowTurnSource(progress: progress)
        guard case .failed(let reason) = result else { XCTFail("Waiting must consume original checkpoint"); return }
        XCTAssertEqual(reason, .stalled)
        XCTAssertEqual(waits.count, 1)
        XCTAssertEqual(waits.first ?? 0, 0.1, accuracy: 1e-6) // Date retains its existing wall-clock precision.
        events.continuation.finish()
    }

    @MainActor
    func testOriginalRecoveryDeadlineWakesSourceWaitWithoutNewFrame() async {
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        let snapshot = sample(1, 9.9)
        var uptime = 10.0
        var waits: [Double] = []
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in XCTFail("Expired episode cannot send") },
            stopRover: {}, sleep: { duration in
                let seconds = Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
                waits.append(seconds)
                uptime = 10.15
            }, poseSample: { snapshot }, sourceNow: { uptime }, sourceEvents: { events.stream })
        let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 1,
            deadline: 10.15, now: { uptime }, canContinue: { true })
        let result = await FollowRecoveryScope.$authorization.withValue(authorization) { await controller.prepareFollowTurnSource() }
        guard case .cancelled = result else { XCTFail("First-loss deadline must remain active without a new frame"); return }
        XCTAssertEqual(waits.count, 1)
        XCTAssertEqual(waits.first ?? 0, 0.15, accuracy: 1e-12)
        events.continuation.finish()
    }

    @MainActor
    func testCallerCancellationInterruptsSourceWaitAndRetainsActualStopConfirmation() async {
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        let snapshot = sample(1, 9.9)
        var completed = false
        var result: FollowTurnSourceResult?
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in XCTFail("Cancelled wait cannot send") },
            stopRover: {}, sleep: { try? await Task.sleep(for: $0) }, poseSample: { snapshot },
            sourceNow: { 10 }, sourceEvents: { events.stream })
        let caller = Task { result = await controller.prepareFollowTurnSource(); completed = true }
        for _ in 0..<100 where controller.followTurnStopFence == nil { await Task.yield() }
        for _ in 0..<20 { await Task.yield() }
        let identity = controller.followTurnStopFence?.identity
        caller.cancel()
        for _ in 0..<100 where !completed { await Task.yield() }
        XCTAssertTrue(completed)
        guard case .cancelled? = result else { XCTFail("Wait cancellation must stay linked to caller"); return }
        XCTAssertEqual(controller.followTurnStopFence?.identity, identity)
        _ = await caller.value
        events.continuation.finish()
    }
    @MainActor
    func testStopEvidenceCarriesActualLastAcknowledgedFenceThroughFailedWrapperStop() async {
        var uptime = 10.0
        var stops = 0
        let snapshot = sample(5, 9.9)
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in XCTFail("Zero turn must not send") },
            stopRover: {
                stops += 1
                uptime = 10 + Double(stops) * 0.1
                if stops == 3 { throw URLError(.timedOut) }
            }, sleep: { _ in }, poseSample: { snapshot }, sourceNow: { uptime },
            sourceStopSnapshot: { snapshot })
        let evidence = FollowMotionOperationEvidence(context: .init(request: nil,
            controllerOperationID: 1, purpose: .followScan,
            profile: RoverConfig.followScanRotationProfile, requestedRotation: 0))
        await FollowMotionTaskScope.$evidence.withValue(evidence) {
            await FollowTurnSourceScope.$required.withValue(true) {
                for _ in 0..<3 { try? await controller.stopAndConfirm() }
            }
        }
        let result = evidence.result(.failed(.commandFailed))
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertEqual(result.stopOutcome, .failed)
        XCTAssertEqual(result.turnStopFence?.acknowledgementUptime, 10.2)
        XCTAssertEqual(result.turnStopFence?.context.controllerOperationID, result.context.controllerOperationID)
        XCTAssertEqual(result.turnStopFence?.highestSequence, 5)
        XCTAssertEqual(result.turnStopFence?.highestSourceTimestamp, 9.9)
    }
    @MainActor
    func testRecoverySourceWaitCannotAdoptGenerationChangedBeforeStop() async {
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        let snapshot = sample(1, 9.9, generation: 2)
        var uptime = 10.0
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: {
                uptime = 10.3
                events.continuation.yield(self.sample(2, 10.2, generation: 2))
                for _ in 0..<20 { await Task.yield() }
                return Date()
            }, sendCommand: { _ in XCTFail("No send") }, stopRover: {},
            sleep: { try? await Task.sleep(for: $0) }, poseSample: { snapshot },
            sourceNow: { uptime }, sourceEvents: { events.stream })
        let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 1,
            deadline: 11, now: { uptime }, canContinue: { true })
        let result = await FollowRecoveryScope.$authorization.withValue(authorization) {
            await controller.prepareFollowTurnSource()
        }
        guard case .failed(let reason) = result else { XCTFail("Stop fence cannot adopt another AR generation"); return }
        XCTAssertEqual(reason, .trackingLost)
        events.continuation.finish()
    }
    func testNewGenerationUsesItsOwnConsumedSequenceWithoutAdmittingOldGeneration() {
        var gate = FollowTurnSourceGate()
        gate.ingest(sample(100, 9.9))
        let old = gate.fence(at: 10, operationGeneration: 1)
        gate.ingest(sample(101, 10.2))
        XCTAssertNotNil(gate.consume(after: old, at: 10.3))
        gate.ingest(sample(1, 10.35, generation: 2))
        let newer = gate.fence(at: 10.4, operationGeneration: 2)
        gate.ingest(sample(102, 10.5))
        XCTAssertNil(gate.consume(after: newer, at: 10.701))
        gate.ingest(sample(2, 10.5, generation: 2))
        XCTAssertEqual(gate.consume(after: newer, at: 10.701)?.frameID, .init(generation: 2, sequence: 2))
    }
    @MainActor
    func testReplacingStoppedSourceWaitWakesOldCallerWithoutDeletingNewFence() async {
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        let snapshot = sample(1, 9.9)
        var oldResult: FollowTurnSourceResult?
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in XCTFail("No send") },
            stopRover: {}, sleep: { try? await Task.sleep(for: $0) }, poseSample: { snapshot },
            sourceNow: { 10 }, sourceEvents: { events.stream })
        let old = Task { oldResult = await controller.prepareFollowTurnSource() }
        for _ in 0..<100 where controller.followTurnStopFence == nil { await Task.yield() }
        for _ in 0..<20 { await Task.yield() }
        let oldFence = controller.followTurnStopFence?.identity
        try? await controller.stopAndConfirm()
        let newFence = controller.followTurnStopFence?.identity
        for _ in 0..<100 where oldResult == nil { await Task.yield() }
        XCTAssertNotEqual(oldFence, newFence)
        guard case .cancelled? = oldResult else {
            XCTFail("Ownership replacement must wake source wait immediately")
            old.cancel(); _ = await old.value; events.continuation.finish(); return
        }
        XCTAssertEqual(controller.followTurnStopFence?.identity, newFence)
        _ = await old.value
        events.continuation.finish()
    }
    @MainActor
    func testSourceWaitGuardAwaitConsumesExistingProgressCheckpoint() async {
        var uptime = 10.0
        var date = Date(timeIntervalSince1970: 0)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        let initial = sample(1, 9.9)
        let advancing = sample(2, 10.2)
        var progress = FollowTurnWaitingProgress()
        progress.distanceToGoal = 1
        progress.hasSentCommand = true
        _ = progress.watchdog.observe(distanceToGoal: 1, now: date, commanded: true)
        let controller = FollowNavigationController(currentPose: { initial.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: {
                uptime = 10.3
                date = Date(timeIntervalSince1970: 2.5)
                events.continuation.yield(advancing)
                await Task.yield()
                return date
            }, sendCommand: { _ in XCTFail("No motion during stopped source gate") }, stopRover: {},
            sleep: { try? await Task.sleep(for: $0) }, now: { date }, poseSample: { initial },
            sourceNow: { uptime }, sourceEvents: { events.stream })
        let result = await controller.prepareFollowTurnSource(progress: progress)
        guard case .failed(let reason) = result else { XCTFail("Guard read cannot pause/reset watchdog"); return }
        XCTAssertEqual(reason, .stalled)
        events.continuation.finish()
    }
    @MainActor
    func testControllerCapturesAckReturnFenceAndAwaitsSourceWithoutProviderPolling() async {
        var uptime = 9.9
        var providerCalls = 0
        var snapshot = sample(40, 9.9)
        var stop: CheckedContinuation<Void, Never>?
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        var selected: NavigationPoseSample?
        var completed = false
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in XCTFail("Source wait must not send") },
            stopRover: { await withCheckedContinuation { stop = $0 } },
            sleep: { try? await Task.sleep(for: $0) }, poseSample: { providerCalls += 1; return snapshot },
            sourceNow: { uptime }, sourceEvents: { events.stream },
            sourceStopSnapshot: { providerCalls += 1; return snapshot })
        let task = Task { @MainActor in
            if case .sample(let value) = await controller.prepareFollowTurnSource() { selected = value }
            completed = true
        }
        for _ in 0..<100 where stop == nil { await Task.yield() }
        XCTAssertNotNil(stop)
        snapshot = sample(45, 9.99)
        events.continuation.yield(snapshot)
        for _ in 0..<20 { await Task.yield() }
        uptime = 10
        stop?.resume()
        for _ in 0..<100 where controller.followTurnStopFence == nil { await Task.yield() }
        XCTAssertEqual(controller.followTurnStopFence?.acknowledgementUptime, 10)
        XCTAssertEqual(controller.followTurnStopFence?.highestSequence, 45)
        events.continuation.yield(sample(46, 9.99)) // delivered late, captured before ack
        for _ in 0..<20 { await Task.yield() }
        XCTAssertFalse(completed)
        uptime = 10.299
        events.continuation.yield(sample(47, 10.2))
        for _ in 0..<20 { await Task.yield() }
        XCTAssertFalse(completed)
        uptime = 10.3
        events.continuation.yield(sample(47, 10.2))
        for _ in 0..<100 where !completed { await Task.yield() }
        XCTAssertTrue(completed)
        XCTAssertEqual(selected?.frameID?.sequence, 47)
        XCTAssertEqual(providerCalls, 2, "One subscription seed and one synchronous stop snapshot; no later reads")
        task.cancel()
        _ = await task.value
        events.continuation.finish()
    }
    @MainActor
    func testOrdinaryFollowAlignmentRejectsLegacyWhileStopped() async {
        var commands = 0
        var stops = 0
        let controller = FollowNavigationController(currentPose: { .init(position: .zero, yaw: 0) },
            forwardClearance: { .infinity }, plan: { _, _ in nil }, lastAckAt: { Date() },
            sendCommand: { _ in commands += 1 }, stopRover: { stops += 1 },
            sleep: { _ in }, now: { Date(timeIntervalSince1970: 0) })
        let result = await controller.rotateForFollowAlignment(by: 0)
        XCTAssertEqual(result, .failed(.trackingLost))
        XCTAssertEqual(commands, 0)
        XCTAssertGreaterThanOrEqual(stops, 1)
    }
    func testPostStopRejectsInvalidSourceWithoutChangingGenericLegacyCompatibility() {
        let legacy = NavigationPoseSample.legacy(.init(position: .zero, yaw: 0))
        XCTAssertNil(legacy.rejection(at: 10.3))
        var gate = FollowTurnSourceGate()
        gate.ingest(sample(1, 9.9))
        let fence = gate.fence(at: 10, operationGeneration: 1)
        let invalid = [legacy, sample(2, 10.2, generation: 2), sample(2, 10.4),
                       sample(2, 10.1), sample(2, .nan), sample(2, 10.2, yaw: .infinity),
                       sample(2, 10.2, quality: .unavailable)]
        for (index, value) in invalid.enumerated() {
            var isolated = gate
            isolated.ingest(value)
            XCTAssertNil(isolated.consume(after: fence, at: index == 3 ? 10.7 : 10.3), "Invalid source \(index)")
        }
        gate.ingest(sample(2, 10.2))
        XCTAssertNotNil(gate.consume(after: fence, at: 10.7), "500ms age is inclusive")
    }
    private func sample(_ sequence: UInt64, _ time: Double, generation: UInt64 = 1,
                        yaw: Double = 0, quality: ARTrackingQuality = .normal) -> NavigationPoseSample {
        .init(pose: .init(position: .zero, yaw: yaw), frameID: .init(generation: generation, sequence: sequence),
              sourceTimestamp: time, trackingQuality: quality)
    }

    func testConfirmedStopRequiresSettleAndActualAdvancingPostAckSource() {
        var gate = FollowTurnSourceGate()
        gate.ingest(sample(40, 9.99))
        // A pending frame with a higher sequence must be fenced even if subsequently delivered out of order.
        gate.ingest(sample(45, 10.05))
        gate.ingest(sample(41, 9.98))
        let fence = gate.fence(at: 10, operationGeneration: 7)
        XCTAssertEqual(fence.highestSequence, 45)
        XCTAssertEqual(fence.highestSourceTimestamp, 10.05)
        XCTAssertNil(gate.consume(after: fence, at: 10.3), "Cached pre-ack source cannot authorize")
        gate.ingest(sample(46, 10))
        XCTAssertNil(gate.consume(after: fence, at: 10.3), "Equality is not after acknowledgement")
        gate.ingest(sample(46, 10.05))
        XCTAssertNil(gate.consume(after: fence, at: 10.3), "New ID with repeated source time is not advancement")
        gate.ingest(sample(44, 10.2))
        XCTAssertNil(gate.consume(after: fence, at: 10.3), "Sequence must exceed highest ingested fence")
        gate.ingest(sample(47, 10.2))
        XCTAssertNil(gate.consume(after: fence, at: 10.299), "Settle starts at acknowledgement return")
        XCTAssertEqual(gate.consume(after: fence, at: 10.3)?.frameID?.sequence, 47)
        XCTAssertNil(gate.consume(after: fence, at: 10.3), "Repeated reads cannot authorize a second decision")
        gate.ingest(sample(48, 10.21))
        XCTAssertEqual(gate.consume(after: fence, at: 10.3)?.frameID?.sequence, 48)
    }
}
