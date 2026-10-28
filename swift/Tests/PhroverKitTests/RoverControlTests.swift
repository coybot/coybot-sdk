import XCTest
import ARKit
import CoreVideo
@testable import PhroverKit

final class RoverControlTests: XCTestCase {
    @MainActor
    func testReadyPulseExpiringInSenderQueueNeverEntersMotorHTTP() async throws {
        StubURLProtocol.reset()
        StubURLProtocol.results = Array(repeating: .success((Data(), HTTPURLResponse(
            url: URL(string: "http://192.168.4.1/js")!, statusCode: 200, httpVersion: nil, headerFields: nil)!)), count: 4)
        let clock = BurstTestClock(10)
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        var receipt: RoverCommandDiagnosticResult?
        let controller = NavigationController(currentPose: { .init(position: .zero, yaw: 0) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { await control.lastAckAt },
            sendCommand: { _ in XCTFail("Real receipt sender must be used") },
            stopRover: { _ = try await control.stopWithReceipt().get() },
            sleep: { try? await Task.sleep(for: $0) }, sendCommandReceipt: { command in
                clock.set(clock.now + 0.041)
                let result = await control.sendNavigationWithReceipt(command)
                receipt = result
                return result
            }, poseSample: {
                .init(pose: .init(position: .zero, yaw: 0), frameID: .init(generation: 1, sequence: 1),
                    sourceTimestamp: 9.99, trackingQuality: .normal)
            }, sourceNow: { clock.now }, transportUptime: { clock.now })
        let result = await NavigationFollowMeMotion(navigation: controller).perform(.ready, context:
            .init(sessionGeneration: 1, requestToken: 1, purpose: .followReady, phase: "aligning"))
        XCTAssertEqual(receipt?.receipt.attempts, 0)
        XCTAssertEqual(receipt?.receipt.outcome, "expired")
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertNil(result.context.failureCause, "Ready expiry is not a rotation-resolution failure")
        XCTAssertEqual(result.stopOutcome, .confirmed)
        for request in StubURLProtocol.requests {
            let json = try XCTUnwrap(URLComponents(url: try XCTUnwrap(request.url), resolvingAgainstBaseURL: false)?
                .queryItems?.first?.value)
            let payload = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(json.utf8)) as? [String: Any])
            XCTAssertEqual(payload["T"] as? Int, 1)
            XCTAssertEqual(payload["L"] as? Double, 0)
            XCTAssertEqual(payload["R"] as? Double, 0)
        }
    }

    func testStopClearsWaveRoverWheelOutputsEvenWhenUnknownOpcodesReturnHTTP200() async throws {
        // Vendor WAVE_ROVER_V0.9: /js returns 200 after dispatch even for
        // unhandled opcodes. Only T:1 with L/R updates the chassis outputs.
        StubURLProtocol.reset()
        StubURLProtocol.results = Array(repeating: .success((Data("{}".utf8), HTTPURLResponse(
            url: URL(string: "http://192.168.4.1/js")!, statusCode: 200, httpVersion: nil, headerFields: nil)!)), count: 3)
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        try await control.sendNavigation(.init(left: -0.25, right: 0.25))
        try await control.stop()
        try await control.probeLink()

        var left = 0.0
        var right = 0.0
        var wheelHistory: [[Double]] = []
        for request in StubURLProtocol.requests {
            let url = try XCTUnwrap(request.url)
            let json = try XCTUnwrap(URLComponents(url: url, resolvingAgainstBaseURL: false)?
                .queryItems?.first(where: { $0.name == "json" })?.value)
            let command = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(json.utf8)) as? [String: Any])
            // Protocol fixture deliberately ignores unknown T values like firmware;
            // an HTTP-success-only stub would miss the actual stop defect.
            if command["T"] as? Int == 1,
               let l = command["L"] as? Double, let r = command["R"] as? Double {
                left = l; right = r
            }
            wheelHistory.append([left, right])
        }
        XCTAssertEqual(wheelHistory, [[0.25, -0.25], [0, 0], [0, 0]],
            "Stop must clear both wheel outputs before observation/heartbeat; HTTP 200 alone cannot stop the base")
    }

    @MainActor
    func testPartialSearchSweepCannotCompleteFromLaterUnsentTargetPose() async throws {
        StubURLProtocol.reset()
        StubURLProtocol.results = Array(repeating: .success((Data(), HTTPURLResponse(
            url: URL(string: "http://192.168.4.1/js")!, statusCode: 200, httpVersion: nil, headerFields: nil)!)), count: 8)
        let clock = BurstTestClock(10)
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        func sample(_ id: UInt64, _ yaw: Double) -> NavigationPoseSample {
            .init(pose: .init(position: .zero, yaw: yaw), frameID: .init(generation: 1, sequence: id),
                sourceTimestamp: clock.now - 0.001, trackingQuality: .normal)
        }
        var snapshot = sample(1, 0)
        var sends = 0
        var stops = 0
        var denied: RoverCommandDiagnosticResult?
        var controller: NavigationController!
        controller = NavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { await control.lastAckAt }, sendCommand: { _ in XCTFail() },
            stopRover: { _ = try await control.stopWithReceipt().get(); stops += 1 },
            sleep: { try? await Task.sleep(for: $0) }, sendCommandReceipt: { command in
                sends += 1
                let entry = FollowTurnBurstTransportScope.authorization!.sendEntryUptime
                if sends == 1 {
                    let receipt = await control.sendNavigationWithReceipt(command)
                    clock.set(entry + 0.081)
                    snapshot = sample(3, 0.02)
                    controller.ingestFollowTurnSource(snapshot)
                    return receipt
                }
                clock.set(entry + 0.002)
                snapshot = sample(5, 0.31)
                controller.ingestFollowTurnSource(snapshot)
                let receipt = await control.sendNavigationWithReceipt(command)
                denied = receipt
                return receipt
            }, poseSample: { snapshot }, sourceNow: { clock.now }, sourceStopSnapshot: { snapshot }, transportUptime: { clock.now })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).perform(.scan(0.3), context:
            .init(sessionGeneration: 1, requestToken: 1, purpose: .followScan, phase: "searching", isSearchSweep: true,
                allowsAcquisitionRotationWithoutDepth: true)) }
        for _ in 0..<2000 where stops < 1 { await Task.yield() }
        clock.set(10.301); snapshot = sample(2, 0); controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<4000 where stops < 2 { await Task.yield() }
        XCTAssertEqual(stops, 2)
        clock.set(clock.now + 0.301); snapshot = sample(4, 0.02); controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<4000 where stops < 3 { await Task.yield() }
        XCTAssertEqual(stops, 3)
        XCTAssertEqual(denied?.receipt.attempts, 0)
        XCTAssertEqual(denied?.receipt.outcome, "fenced")
        clock.set(clock.now + 0.301); snapshot = sample(6, 0.3); controller.ingestFollowTurnSource(snapshot)
        let result = await task.value
        XCTAssertEqual(result.result, .failed(.rotationResolutionInsufficient),
            "A later target pose cannot fill the unmeasured remainder of a partial sweep")
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(result.searchSweepTravel ?? -1, 0.02, accuracy: 1e-12)
        XCTAssertEqual(StubURLProtocol.requestCount, 4, "One motor request and three confirmed stops")
    }

    func testTargetFenceCannotRelabelEarlierExpiryOrOwnershipInhibition() {
        for earlier in ["expiry", "owner", "stale_snapshot"] {
            let fence = FollowTurnBurstFence()
            XCTAssertTrue(fence.publishValidity(from: 10, untilExclusive: earlier == "stale_snapshot" ? 10.010 : 11))
            XCTAssertNotNil(fence.arm(entry: 10, budget: 0.080))
            if earlier == "owner" { fence.inhibit() }
            fence.inhibitForTarget(.tolerance, at: earlier == "expiry" ? 10.080 : 10.020)
            XCTAssertNil(fence.targetStop, earlier)
            XCTAssertFalse(fence.authorized(at: 10.021))
        }
    }

    @MainActor
    func testTargetReachedBeforeHTTPCompletesOnlyAfterConfirmedStopAndFreshPose() async throws {
        for purpose in [FollowMotionPurpose.followScan, .followAlignment] {
            try await runUnsentTargetFence(purpose: purpose, scenario: "within_tolerance")
            try await runUnsentTargetFence(purpose: purpose, scenario: "crossed")
        }
    }

    @MainActor
    func testUnsentTargetFenceDoesNotMaskLostAuthorityOrUncertainTransport() async throws {
        for scenario in ["drifted", "stale", "stop_failed", "entered_attempt", "expired", "unknown_fence", "tracking_lost"] {
            try await runUnsentTargetFence(purpose: .followScan, scenario: scenario)
        }
    }

    @MainActor
    private func runUnsentTargetFence(purpose: FollowMotionPurpose, scenario: String) async throws {
            StubURLProtocol.reset()
            StubURLProtocol.results = Array(repeating: .success((Data(), HTTPURLResponse(
                url: URL(string: "http://192.168.4.1/js")!, statusCode: 200, httpVersion: nil, headerFields: nil)!)), count: 5)
            let clock = BurstTestClock(10)
            let control = RoverControl(session: URLSession(configuration: .stubbed))
            if scenario == "stop_failed" {
                StubURLProtocol.results = [.success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
                    statusCode: 200, httpVersion: nil, headerFields: nil)!)), .failure(URLError(.badURL))]
            }
            var snapshot = NavigationPoseSample(pose: .init(position: .zero, yaw: 0),
                frameID: .init(generation: 1, sequence: 1), sourceTimestamp: 9.99, trackingQuality: .normal)
            var stops = 0
            var completed = false
            var denied: RoverCommandDiagnosticResult?
            let sink = FollowDiagnosticRecordingSink()
            var controller: NavigationController!
            controller = NavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
                plan: { _, _ in nil }, lastAckAt: { await control.lastAckAt }, sendCommand: { _ in XCTFail() },
                stopRover: { _ = try await control.stopWithReceipt().get(); stops += 1 },
                sleep: { try? await Task.sleep(for: $0) }, sendCommandReceipt: { command in
                    // A real fresh ingress event reaches tolerance while the
                    // transport is queued, before its first HTTP attempt.
                    clock.set(scenario == "expired" ? 10.400 : 10.302)
                    snapshot = .init(pose: .init(position: .zero, yaw: scenario == "crossed" ? 0.31 : 0.29),
                        frameID: .init(generation: 1, sequence: 3), sourceTimestamp: 10.301,
                        trackingQuality: scenario == "tracking_lost" ? .limited : .normal)
                    if scenario != "unknown_fence" { controller.ingestFollowTurnSource(snapshot) }
                    let receipt: RoverCommandDiagnosticResult
                    if scenario == "unknown_fence" {
                        receipt = .init(receipt: .init(httpStatus: nil, acknowledged: false,
                            acknowledgementUTC: nil, attempts: 0, outcome: "fenced"), failure: FollowTurnBurstTransportDenial.fenced)
                    } else {
                        receipt = await control.sendNavigationWithReceipt(command)
                    }
                    if scenario == "entered_attempt", let authority = FollowTurnBurstTransportScope.authorization {
                        authority.didEnterAttempt?(.init(operationID: authority.operationID, attempt: 1, entryUptime: clock.now))
                    }
                    denied = receipt
                    return receipt
                }, diagnosticEmitter: .init(streamID: "target-before-http", monotonic: { clock.now }, utc: { Date() }, sink: sink.append),
                poseSample: { snapshot }, sourceNow: { clock.now }, sourceStopSnapshot: { snapshot }, transportUptime: { clock.now })
            let motion = NavigationFollowMeMotion(navigation: controller)
            let task = Task {
                let result = await motion.perform(purpose == .followScan ? .scan(0.3) : .alignment(0.3), context:
                    .init(sessionGeneration: 1, requestToken: 1, purpose: purpose, phase: "searching"))
                completed = true
                return result
            }
            for _ in 0..<2000 where stops < 1 { await Task.yield() }
            clock.set(10.301)
            snapshot = .init(pose: .init(position: .zero, yaw: 0), frameID: .init(generation: 1, sequence: 2),
                sourceTimestamp: 10.2, trackingQuality: .normal)
            controller.ingestFollowTurnSource(snapshot)
            for _ in 0..<4000 where stops < 2 && !completed { await Task.yield() }
            for _ in 0..<100 { await Task.yield() }
            if ["within_tolerance", "crossed", "drifted", "stale"].contains(scenario) {
                XCTAssertFalse(completed, "A pre-send tolerance trigger is not stopped arrival")
            }
            XCTAssertEqual(denied?.receipt.outcome, scenario == "expired" ? "expired" : "fenced", scenario)
            XCTAssertEqual(denied?.receipt.attempts, 0)
            clock.set(10.603)
            snapshot = .init(pose: .init(position: .zero, yaw: 0.3), frameID: .init(generation: 1, sequence: 4),
                sourceTimestamp: scenario == "stale" ? 10.0 : 10.602, trackingQuality: .normal)
            if scenario == "drifted" {
                snapshot = .init(pose: .init(position: .zero, yaw: 0), frameID: .init(generation: 1, sequence: 4),
                    sourceTimestamp: 10.602, trackingQuality: .normal)
            }
            controller.ingestFollowTurnSource(snapshot)
            let result = await task.value
            let expected: NavigationResult
            switch scenario {
            case "within_tolerance", "crossed": expected = .arrived
            case "drifted", "expired": expected = .failed(.rotationResolutionInsufficient)
            case "stale", "tracking_lost": expected = .failed(.trackingLost)
            default: expected = .failed(.commandFailed)
            }
            XCTAssertEqual(result.result, expected, scenario)
            if scenario == "within_tolerance" || scenario == "crossed" { XCTAssertNil(result.failure) }
            XCTAssertEqual(result.stopOutcome, scenario == "stop_failed" ? .failed : .confirmed, scenario)
            XCTAssertEqual(stops, scenario == "stop_failed" ? 1 : 2, scenario)
            XCTAssertEqual(StubURLProtocol.requestCount, 2, "Only initial and final stop HTTP requests")
            XCTAssertFalse(sink.records.contains { $0.event == "follow_scan.burst_response" }, "Unsent commands cannot train calibration")
    }
    func testPreparedSnapshotRechecksBurstExpiryAtItsFinalClockRead() async {
        StubURLProtocol.reset()
        StubURLProtocol.results = [.success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!))]
        let clock = BurstScriptedClock([10.001, 10.002, 10.003, 10.004, 10.079, 10.081])
        let fence = FollowTurnBurstFence()
        XCTAssertTrue(fence.publishValidity(from: 10, untilExclusive: 10.5))
        XCTAssertNotNil(fence.arm(entry: 10, budget: 0.080))
        let authority = FollowTurnBurstAuthorization(operationID: 1, preparedFence: fence,
            uptime: { clock.now }, didEnterAttempt: { _ in }, transportCapture: .init())
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let result = await control.sendNavigationWithReceipt(.init(left: -0.25, right: 0.25), authorization: authority)
        XCTAssertEqual(StubURLProtocol.requestCount, 0)
        XCTAssertEqual(result.receipt.attempts, 0)
        XCTAssertEqual(result.receipt.outcome, "expired")
    }

    @MainActor
    func testRevocationPrecedesCancellationDiagnosticCallbacks() async {
        for origin in ["detection", "stop", "cancel"] {
            let clock = BurstTestClock(10)
            var snapshot = NavigationPoseSample(pose: .init(position: .zero, yaw: 0),
                frameID: .init(generation: 1, sequence: 1), sourceTimestamp: 9.99, trackingQuality: .normal)
            var authority: FollowTurnBurstAuthorization?
            var sendWaiter: CheckedContinuation<Void, Never>?
            var checkedRevocation = false
            let emitter = FollowDiagnosticEmitter(streamID: "revoke-\(origin)", monotonic: { clock.now }, utc: { Date() }) { event, _ in
                if event == "follow_scan.cancel", let authority {
                    // The transport may concurrently execute while this synchronous
                    // diagnostic callback performs formatting or disk writes.
                    checkedRevocation = true
                    XCTAssertFalse(authority.isAuthorized(), "Revoke before logging: \(origin)")
                }
            }
            let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
                plan: { _, _ in nil }, lastAckAt: { Date() }, sendCommand: { _ in }, stopRover: {},
                sleep: { try? await Task.sleep(for: $0) }, sendCommandReceipt: { _ in
                    authority = FollowTurnBurstTransportScope.authorization
                    await withCheckedContinuation { sendWaiter = $0 }
                    return .init(receipt: .unknown, failure: CancellationError())
                }, diagnosticEmitter: emitter, poseSample: { snapshot }, sourceNow: { clock.now },
                sourceStopSnapshot: { snapshot }, transportUptime: { clock.now })
            let motion = NavigationFollowMeMotion(navigation: controller)
            let operation = Task { await motion.perform(.alignment(0.5), context:
                .init(sessionGeneration: 1, requestToken: 1, purpose: .followAlignment, phase: "aligning")) }
            for _ in 0..<2000 where controller.followTurnStopFence == nil { await Task.yield() }
            XCTAssertNotNil(controller.followTurnStopFence)
            clock.set(10.301)
            snapshot = .init(pose: .init(position: .zero, yaw: 0), frameID: .init(generation: 1, sequence: 2),
                sourceTimestamp: 10.2, trackingQuality: .normal)
            controller.ingestFollowTurnSource(snapshot)
            for _ in 0..<2000 where sendWaiter == nil { await Task.yield() }
            XCTAssertNotNil(sendWaiter)
            if origin == "detection" { controller.inhibitFollowScanContinuation(origin: .detection) }
            if origin == "cancel" { controller.cancel() }
            let stop = Task { try? await controller.stopAndConfirm() }
            for _ in 0..<2000 where !checkedRevocation { await Task.yield() }
            XCTAssertTrue(checkedRevocation)
            sendWaiter?.resume(); sendWaiter = nil
            _ = await stop.value
            _ = await operation.value
        }
    }

    func testSynchronousSnapshotRefusesExpiredOrInhibitedAuthorityBeforeHTTP() async {
        for scenario in ["freshness", "inhibited", "unvalidated", "retry"] {
            StubURLProtocol.reset()
            StubURLProtocol.results = scenario == "retry"
                ? [.failure(URLError(.timedOut))]
                : [.success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
                    statusCode: 200, httpVersion: nil, headerFields: nil)!))]
            let clock = BurstTestClock(10)
            let fence = FollowTurnBurstFence()
            if scenario != "unvalidated" { XCTAssertTrue(fence.publishValidity(from: 10, untilExclusive: 10.020)) }
            XCTAssertNotNil(fence.arm(entry: 10, budget: 0.080))
            let authorization = FollowTurnBurstAuthorization(operationID: 7, preparedFence: fence,
                uptime: { clock.now }, didEnterAttempt: { _ in }, transportCapture: .init())
            if scenario == "inhibited" { fence.inhibit() }
            if scenario == "freshness" { clock.set(10.020) }
            let control = RoverControl(session: URLSession(configuration: .stubbed), retrySleep: { _ in clock.set(10.020) })
            let result = await control.sendNavigationWithReceipt(.init(left: -0.25, right: 0.25), authorization: authorization)
            XCTAssertEqual(result.receipt.outcome, "fenced", scenario)
            XCTAssertEqual(result.receipt.attempts, scenario == "retry" ? 1 : 0, scenario)
            XCTAssertEqual(StubURLProtocol.requestCount, scenario == "retry" ? 1 : 0, scenario)
            XCTAssertLessThan(clock.now, authorization.deadline, "This is authority expiry, not burst-budget expiry")
        }
    }

    @MainActor
    func testControllerSnapshotExpiresWithSourceWithoutWaitingForMainActorMonitor() async {
        StubURLProtocol.reset()
        StubURLProtocol.results = Array(repeating: .success((Data(), HTTPURLResponse(
            url: URL(string: "http://192.168.4.1/js")!, statusCode: 200, httpVersion: nil, headerFields: nil)!)), count: 4)
        let clock = BurstTestClock(10)
        var snapshot = NavigationPoseSample(pose: .init(position: .zero, yaw: 0),
            frameID: .init(generation: 1, sequence: 1), sourceTimestamp: 9.99, trackingQuality: .normal)
        var motionResult: RoverCommandDiagnosticResult?
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { await control.lastAckAt }, sendCommand: { _ in },
            stopRover: { _ = try await control.stopWithReceipt().get() }, sleep: { try? await Task.sleep(for: $0) },
            sendCommandReceipt: { command in
                // Still inside the 80 ms burst, but the validated source becomes
                // stale without delivering an event to the MainActor observer.
                clock.set(10.701)
                let result = await control.sendNavigationWithReceipt(command)
                motionResult = result
                return result
            }, poseSample: { snapshot }, sourceNow: { clock.now }, sourceStopSnapshot: { snapshot },
            transportUptime: { clock.now })
        let task = Task { await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.5) }
        for _ in 0..<2000 where controller.followTurnStopFence == nil { await Task.yield() }
        XCTAssertNotNil(controller.followTurnStopFence)
        clock.set(10.690)
        snapshot = .init(pose: .init(position: .zero, yaw: 0), frameID: .init(generation: 1, sequence: 2),
            sourceTimestamp: 10.200, trackingQuality: .normal)
        controller.ingestFollowTurnSource(snapshot)
        _ = await task.value
        XCTAssertEqual(motionResult?.receipt.outcome, "fenced")
        XCTAssertEqual(motionResult?.receipt.attempts, 0)
        XCTAssertEqual(StubURLProtocol.requestCount, 2, "Initial and final STOP only")
    }

    @MainActor
    func testPreparedControllerAuthorityDoesNotSpendBurstBudgetOnActorReauthorization() async throws {
        StubURLProtocol.reset()
        StubURLProtocol.results = [.success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!))]
        let clock = BurstTestClock(10)
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in }, stopRover: {},
            sleep: { try? await Task.sleep(for: $0) }, sendCommandReceipt: { command in
                let prepared = try! XCTUnwrap(FollowTurnBurstTransportScope.authorization)
                let delayedCheck: (@Sendable () async -> Bool)?
                if let check = prepared.authorizeAttempt {
                    delayedCheck = { clock.set(10.065); return await check() }
                } else {
                    delayedCheck = nil
                }
                // Model the measured 65 ms MainActor round trip only when the
                // production authority still requires asynchronous revalidation.
                let simulated = FollowTurnBurstAuthorization(operationID: prepared.operationID,
                    sendEntryUptime: prepared.sendEntryUptime,
                    requestedBudget: prepared.deadline - prepared.sendEntryUptime,
                    uptime: prepared.uptime, isAuthorized: prepared.isAuthorized,
                    authorizeAttempt: delayedCheck,
                    didEnterAttempt: prepared.didEnterAttempt, transportCapture: prepared.transportCapture)
                return await control.sendNavigationWithReceipt(command, authorization: simulated)
            }, sourceNow: { clock.now }, transportUptime: { clock.now })
        let receipt = await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25),
            requestedBudget: 0.015229, purpose: .followAlignment)
        XCTAssertEqual(receipt.result.receipt.outcome, "acknowledged")
        XCTAssertEqual(receipt.result.receipt.attempts, 1)
        XCTAssertEqual(StubURLProtocol.requestCount, 1,
            "Prepared authority must be checked synchronously on the transport actor")
        XCTAssertEqual(receipt.deadline - receipt.sendEntryUptime, 0.015229, accuracy: 1e-12)
    }

    @MainActor
    func testHealthFailureRetainsPrecedenceOverDefiniteZeroAttemptExpiry() async throws {
        let result = try await classifiedTurnFailure(holdAuthorization: true, healthLoss: true)
        XCTAssertEqual(result.result, .failed(.trackingLost))
        XCTAssertEqual(result.failure?.reason, .trackingLost)
        XCTAssertNil(result.context.failureCause)
        XCTAssertEqual(result.commandReceipt?.outcome, "expired")
        XCTAssertEqual(result.commandReceipt?.attempts, 0)
    }

    func testRequestStartClockExpiryCannotEnterHTTPAfterEarlierEligibility() async {
        StubURLProtocol.reset()
        StubURLProtocol.results = [.success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!))]
        let clock = BurstScriptedClock([10.001, 10.002, 10.003, 10.004, 10.080])
        let capture = FollowTurnTransportCapture()
        let authorization = FollowTurnBurstAuthorization(operationID: 1, sendEntryUptime: 10,
            requestedBudget: 0.080, uptime: { clock.now }, didEnterAttempt: capture.record,
            transportCapture: capture)
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let result = await control.sendNavigationWithReceipt(.init(left: -0.25, right: 0.25), authorization: authorization)
        XCTAssertEqual(result.receipt.outcome, "expired")
        XCTAssertEqual(result.receipt.attempts, 0)
        XCTAssertEqual(StubURLProtocol.requestCount, 0, "The last request-start read must still be gated")
        XCTAssertTrue(capture.attempts.isEmpty, "Denied request creation cannot manufacture an entered attempt")
    }

    @MainActor
    func testRealCoordinatorRetainsPreSendCauseAndTimingThroughPendingResultAndConfirmation() async throws {
        StubURLProtocol.reset()
        StubURLProtocol.results = Array(repeating: .success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!)), count: 12)
        StubURLProtocol.suspendRequestNumber = 2
        let clock = BurstTestClock(10)
        let followClock = ManualFollowClock()
        followClock.advance(to: 10)
        let gate = BurstAttemptAdmissionGate()
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let perception = FollowPerceptionFake()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "coordinator-transport", monotonic: { clock.now }, utc: { Date() }, sink: sink.append)
        var snapshot = NavigationPoseSample(pose: .init(position: .zero, yaw: 0), frameID: .init(generation: 1, sequence: 1),
            sourceTimestamp: 9.99, trackingQuality: .normal)
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in XCTFail("Scheduling failure cannot enter readiness/departure"); return nil },
            lastAckAt: { await control.lastAckAt }, sendCommand: { _ in XCTFail("Use real receipt sender") },
            stopRover: { _ = try await control.stopWithReceipt().get() }, sleep: { try? await Task.sleep(for: $0) },
            sendCommandReceipt: { command in
                let original = FollowTurnBurstTransportScope.authorization!
                let held = FollowTurnBurstAuthorization(operationID: original.operationID,
                    sendEntryUptime: original.sendEntryUptime, requestedBudget: original.deadline - original.sendEntryUptime,
                    uptime: original.uptime, isAuthorized: original.isAuthorized, authorizeAttempt: {
                        await gate.wait()
                        return await original.authorizeAttempt?() ?? true
                    }, didEnterAttempt: original.didEnterAttempt, transportCapture: original.transportCapture)
                return await control.sendNavigationWithReceipt(command, authorization: held)
            }, diagnosticEmitter: emitter, poseSample: { snapshot }, sourceNow: { clock.now },
            sourceStopSnapshot: { snapshot }, transportUptime: { clock.now })
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: NavigationFollowMeMotion(navigation: controller),
            clock: followClock, configuration: config, eventSink: sink.append)
        let started = await coordinator.start()
        XCTAssertTrue(started)
        for _ in 0..<100 { await Task.yield() }
        perception.send(.frame(.init(frameID: .init(generation: 1, sequence: 1), timestamp: 10,
            pose: snapshot.pose, depthAvailable: true, people: [], trackingQuality: .normal)))
        for _ in 0..<2000 where controller.followTurnStopFence == nil { await Task.yield() }
        XCTAssertNotNil(controller.followTurnStopFence)
        clock.set(10.301); followClock.advance(to: 10.301)
        snapshot = .init(pose: .init(position: .zero, yaw: 0), frameID: .init(generation: 1, sequence: 2),
            sourceTimestamp: 10.2, trackingQuality: .normal)
        controller.ingestFollowTurnSource(snapshot)
        for _ in 0..<4000 {
            if await gate.isWaiting { break }
            await Task.yield()
        }
        let waiting = await gate.isWaiting
        XCTAssertTrue(waiting)
        clock.set(10.434); followClock.advance(to: 10.434)
        await gate.release()
        let pendingText = "Turn not started: command scheduling exceeded burst budget. Confirming motor stop…"
        for _ in 0..<4000 where coordinator.state != .failed(pendingText) { await Task.yield() }
        XCTAssertEqual(coordinator.state, .failed(pendingText))
        XCTAssertNotNil(StubURLProtocol.suspended)
        StubURLProtocol.releaseFirst()
        let confirmedText = "Turn not started: command scheduling exceeded burst budget. Stop confirmed. Restart following to try again."
        for _ in 0..<4000 where coordinator.state != .failed(confirmedText) { await Task.yield() }
        XCTAssertEqual(coordinator.state, .failed(confirmedText))
        let resolutions = try sink.records.filter { $0.event == "follow_motion.failure_resolution" }.map {
            try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap($0.fields["payload"]).utf8)) as? [String: Any])
        }
        XCTAssertTrue(resolutions.contains { $0["source"] as? String == "stream" })
        XCTAssertTrue(resolutions.contains { $0["source"] as? String == "result" })
        XCTAssertTrue(resolutions.contains { $0["source"] as? String == "confirmation" })
        XCTAssertTrue(resolutions.allSatisfy { $0["reason"] as? String == "burst_pre_send_expired" })
        XCTAssertTrue(resolutions.allSatisfy { $0["primary_typed_reason"] as? String == "rotationResolutionInsufficient" })
        XCTAssertTrue(resolutions.allSatisfy { $0["sender_outcome"] as? String == "expired" },
            "The actual coordinator must retain controller timing/context, not drop it while copying deliveries")
        XCTAssertTrue(resolutions.allSatisfy { $0["send_entry_uptime_s"] as? Double == 10.301 })
        XCTAssertFalse(sink.records.contains { $0.event == "follow_scan.burst_response" || $0.event == "follow_ready.admission_authorized" })
        XCTAssertEqual(StubURLProtocol.requests.count, StubURLProtocol.requestCount)
        for request in StubURLProtocol.requests {
            let json = try XCTUnwrap(URLComponents(url: try XCTUnwrap(request.url), resolvingAgainstBaseURL: false)?.queryItems?.first { $0.name == "json" }?.value)
            let payload = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(json.utf8)) as? [String: Any])
            XCTAssertEqual(payload["T"] as? Int, 1)
            XCTAssertEqual(payload["L"] as? Double, 0, "Every actual HTTP command clears the wheels; no nonzero motion/retry")
            XCTAssertEqual(payload["R"] as? Double, 0)
        }
        _ = await coordinator.stop()
    }

    @MainActor
    func testFailedStopAndContradictoryAttemptEvidenceCannotPublishSafeNotStartedOutcome() async throws {
        let failedStop = try await classifiedTurnFailure(holdAuthorization: true, failStop: true)
        XCTAssertEqual(failedStop.result, .failed(.commandFailed))
        XCTAssertEqual(failedStop.stopOutcome, .failed)
        XCTAssertEqual(FollowMotionFailureResolution(try XCTUnwrap(failedStop.failure)).message,
            "Motor stop could not be confirmed. Motion is blocked.")
        let contradictory = try await classifiedTurnFailure(holdAuthorization: true, contradictoryEntry: true)
        XCTAssertEqual(contradictory.result, .failed(.commandFailed))
        XCTAssertNil(contradictory.context.failureCause)
        XCTAssertEqual(FollowMotionFailureResolution(try XCTUnwrap(contradictory.failure)).diagnosticReason,
            "transport_failed")
    }

    @MainActor
    func testCancellationAndOwnerReplacementDuringPreparationNeverArmOrInvokeSender() async throws {
        for interruption in ["cancel", "replacement", "clock_replacement"] {
            let clock = BurstTestClock(10)
            var controller: FollowNavigationController!
            var sends = 0
            var stops = 0
            var replaceAtClockRead = false
            let sink = FollowDiagnosticRecordingSink()
            let emitter = FollowDiagnosticEmitter(streamID: "abandoned-preparation", monotonic: { clock.now }, utc: { Date() }) { event, fields in
                sink.append(event, fields: fields)
                if event == "follow_scan.send_begin" {
                    if interruption == "clock_replacement" { replaceAtClockRead = true }
                    else if interruption == "replacement" { controller.cancel() }
                    else { withUnsafeCurrentTask { $0?.cancel() } }
                }
            }
            let evidence = FollowMotionOperationEvidence(context: .init(request: nil, controllerOperationID: 1,
                purpose: .followAlignment, profile: .turnBurst(purpose: .followAlignment)))
            evidence.scanTrace = .init(emitter: emitter)
            evidence.burstTrace = .init()
            controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
                plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in }, stopRover: { stops += 1 },
                sleep: { try? await Task.sleep(for: $0) }, sendCommandReceipt: { _ in
                    sends += 1
                    XCTFail("Abandoned preparation cannot enter sender")
                    return .init(receipt: .unknown, failure: nil)
                }, sourceNow: {
                    if replaceAtClockRead { replaceAtClockRead = false; controller.cancel() }
                    return clock.now
                }, transportUptime: { clock.now })
            let task = Task {
                await FollowMotionTaskScope.$evidence.withValue(evidence) {
                    try? await controller.executeFollowTurnBurst(.init(left: -0.25, right: 0.25),
                        requestedBudget: 0.002374, purpose: .followAlignment)
                }
            }
            _ = await task.value
            for _ in 0..<1000 where stops == 0 { await Task.yield() }
            XCTAssertEqual(sends, 0)
            XCTAssertGreaterThanOrEqual(stops, 1, "Serialized stop still drains abandoned work")
            XCTAssertNil(controller.followTurnBurstPendingStatus)
            XCTAssertNil(evidence.context.failureCause)
            let begin = try XCTUnwrap(sink.records.first { $0.event == "follow_scan.send_begin" })
            let json = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap(begin.fields["payload"]).utf8)) as? [String: Any])
            XCTAssertTrue(json["send_entry_uptime_s"] is NSNull)
            XCTAssertTrue(json["burst_deadline_uptime_s"] is NSNull)
        }
    }

    @MainActor
    func testRealTransportReportsActorAuthorizationEligibilityAndRequestStartSeparately() async throws {
        StubURLProtocol.reset()
        StubURLProtocol.results = [.success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!))]
        let clock = BurstTestClock(10)
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "transport-timing", monotonic: { clock.now }, utc: { Date() }, sink: sink.append)
        let evidence = FollowMotionOperationEvidence(context: .init(request: nil, controllerOperationID: 1,
            purpose: .followAlignment, profile: .turnBurst(purpose: .followAlignment)))
        evidence.scanTrace = .init(emitter: emitter)
        evidence.burstTrace = .init()
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in }, stopRover: {},
            sleep: { try? await Task.sleep(for: $0) }, sendCommandReceipt: { command in
                clock.set(10.003) // Sender-to-control actor queue/setup cost remains inside budget.
                return await control.sendNavigationWithReceipt(command)
            }, sourceNow: { clock.now }, transportUptime: { clock.now })
        let receipt = await FollowMotionTaskScope.$evidence.withValue(evidence) {
            await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25), requestedBudget: 0.080, purpose: .followAlignment)
        }
        XCTAssertEqual(receipt.sendEntryUptime, 10)
        XCTAssertEqual(receipt.deadline, 10.080)
        let ack = try XCTUnwrap(sink.records.first { $0.event == "follow_scan.send_ack" })
        let json = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap(ack.fields["payload"]).utf8)) as? [String: Any])
        XCTAssertEqual(json["transport_actor_entry_uptime_s"] as? Double, 10.003)
        XCTAssertEqual(json["transport_timing_availability"] as? String, "available")
        let timeline = try XCTUnwrap(json["transport_timing_entries"] as? [[String: Any]])
        XCTAssertEqual(timeline.compactMap { $0["boundary"] as? String },
            ["actor_entry", "authorization_start", "authorization_end", "eligibility", "request_start"])
        XCTAssertEqual(timeline.compactMap { $0["uptime_s"] as? Double }, Array(repeating: 10.003, count: 5))
        XCTAssertEqual(timeline.last?["attempt"] as? Int, 1)
    }

    @MainActor
    func testPreSendExpiryPublishesTypedCauseWhileRealStopConfirmationIsHeld() async throws {
        let result = try await classifiedTurnFailure(holdAuthorization: true, holdStop: true)
        XCTAssertEqual(result.context.failureCause, .burstPreSendExpired)
        XCTAssertEqual(result.failure?.context.failureCause, .burstPreSendExpired)
    }

    @MainActor
    func testDiagnosticSetupCostPrecedesActualSenderBudgetOrigin() async throws {
        StubURLProtocol.reset()
        let accepted = (Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!)
        StubURLProtocol.results = [.success(accepted), .success(accepted)]
        let clock = BurstTestClock(10)
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "budget-origin", monotonic: { clock.now }, utc: { Date() },
            sink: { event, fields in
                sink.append(event, fields: fields)
                if event == "follow_scan.send_begin" { clock.set(10.100) }
            })
        let evidence = FollowMotionOperationEvidence(context: .init(request: nil, controllerOperationID: 99,
            purpose: .followAlignment, profile: .turnBurst(purpose: .followAlignment)))
        evidence.scanTrace = .init(emitter: emitter)
        evidence.burstTrace = .init()
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        var invokedAt: Double?
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in },
            stopRover: { _ = try await control.stopWithReceipt().get() },
            sleep: { try? await Task.sleep(for: $0) }, sendCommandReceipt: { command in
                invokedAt = clock.now
                let result = await control.sendNavigationWithReceipt(command)
                clock.set(10.200) // Response/drain is late, so no remaining motor wait is permitted.
                return result
            }, sourceNow: { clock.now }, transportUptime: { clock.now })
        let receipt = try await FollowMotionTaskScope.$evidence.withValue(evidence) {
            try await controller.executeFollowTurnBurst(.init(left: -0.25, right: 0.25),
                requestedBudget: 0.0023742724180478418, purpose: .followAlignment)
        }
        XCTAssertEqual(invokedAt, 10.100)
        XCTAssertEqual(receipt.send.sendEntryUptime, 10.100,
            "Synchronous formatting/sink work is pre-entry setup, not sender queue time")
        XCTAssertEqual(receipt.send.deadline, 10.1023742724180478418, accuracy: 1e-12)
        XCTAssertEqual(receipt.send.transportAttempts.count, 1)
        XCTAssertEqual(receipt.send.transportAttempts.first?.entryUptime, 10.100)
        XCTAssertEqual(receipt.send.result.receipt.outcome, "acknowledged")
        XCTAssertEqual(StubURLProtocol.requestCount, 2, "One real motion request, then confirmed stop")
        XCTAssertNotNil(receipt.confirmedStopFence)
        let ack = try XCTUnwrap(sink.records.first { $0.event == "follow_scan.send_ack" })
        let payload = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap(ack.fields["payload"]).utf8)) as? [String: Any])
        XCTAssertEqual(payload["send_preparation_start_uptime_s"] as? Double, 10)
        XCTAssertEqual(try XCTUnwrap(payload["send_preparation_duration_s"] as? Double), 0.100, accuracy: 1e-12)
        XCTAssertEqual(payload["send_entry_uptime_s"] as? Double, 10.100)
        XCTAssertEqual(payload["send_entry_availability"] as? String, "captured_sender_invocation")
    }

    @MainActor
    func testPreSendExpiryReportsSchedulingResolutionWithoutMotionLearningOrRetry() async throws {
        let result = try await classifiedTurnFailure(holdAuthorization: true)
        XCTAssertEqual(result.result, .failed(.rotationResolutionInsufficient))
        let failure = try XCTUnwrap(result.failure)
        XCTAssertEqual(failure.reason, .rotationResolutionInsufficient)
        XCTAssertEqual(failure.commandReceipt?.outcome, "expired")
        XCTAssertEqual(failure.commandReceipt?.attempts, 0)
        XCTAssertEqual(failure.commandReceipt?.acknowledged, false)
        var resolution = FollowMotionFailureResolution(failure)
        XCTAssertNotNil(resolution.key)
        XCTAssertEqual(resolution.diagnosticReason, "burst_pre_send_expired")
        XCTAssertEqual(resolution.message, "Turn not started: command scheduling exceeded burst budget. Stop confirmed. Restart following to try again.")
        resolution.consume(.init(context: result.context, reason: .commandFailed,
            stopOutcome: .pending, source: .stream))
        resolution.consume(.init(context: result.context, reason: .cancelled,
            stopOutcome: .confirmed, source: .confirmation))
        XCTAssertEqual(resolution.primaryReason, .rotationResolutionInsufficient)
        XCTAssertEqual(resolution.diagnosticReason, "burst_pre_send_expired")
        var pending = FollowMotionFailureResolution(.init(context: result.context,
            reason: failure.reason, stopOutcome: .pending, source: .stream,
            commandReceipt: failure.commandReceipt, turnDiagnosticFields: failure.turnDiagnosticFields))
        XCTAssertEqual(pending.message, "Turn not started: command scheduling exceeded burst budget. Confirming motor stop…")
        pending.consume(failure)
        XCTAssertEqual(pending.message, resolution.message)
        pending.consume(.init(context: result.context, reason: .commandFailed,
            stopOutcome: .failed, source: .confirmation))
        XCTAssertEqual(pending.message, "Motor stop could not be confirmed. Motion is blocked.")
    }

    @MainActor
    func testActualHTTPFailureRetainsTransportClassificationAndConfirmedSafetyStop() async throws {
        for expiryAfterAttempt in [false, true] {
            let result = try await classifiedTurnFailure(holdAuthorization: false,
                expiryAfterAttempt: expiryAfterAttempt)
            XCTAssertEqual(result.result, .failed(.commandFailed))
            let failure = try XCTUnwrap(result.failure)
            XCTAssertEqual(failure.reason, .commandFailed)
            XCTAssertEqual(failure.commandReceipt?.outcome, expiryAfterAttempt ? "expired" : "failed")
            XCTAssertEqual(failure.commandReceipt?.attempts, 1)
            XCTAssertEqual(failure.commandReceipt?.httpStatus, expiryAfterAttempt ? nil : 503)
            let resolution = FollowMotionFailureResolution(failure)
            XCTAssertEqual(resolution.diagnosticReason, "transport_failed",
                "Expiry after an entered request retains uncertain-motion transport failure")
            XCTAssertEqual(resolution.message, "Navigation command failed.")
        }
    }

    @MainActor
    private func classifiedTurnFailure(holdAuthorization: Bool,
                                       expiryAfterAttempt: Bool = false, holdStop: Bool = false,
                                       failStop: Bool = false, contradictoryEntry: Bool = false,
                                       healthLoss: Bool = false) async throws -> FollowMotionResult {
        func http(_ status: Int) -> Result<(Data, URLResponse), Error> {
            .success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
                statusCode: status, httpVersion: nil, headerFields: nil)!))
        }
        StubURLProtocol.reset()
        StubURLProtocol.results = holdAuthorization ? [http(200), http(200)] : [http(200), http(503), http(200)]
        if failStop { StubURLProtocol.results = [http(200), .failure(URLError(.badURL))] }
        let clock = BurstTestClock(10)
        if expiryAfterAttempt {
            StubURLProtocol.results = [http(200), .failure(URLError(.timedOut)), http(200)]
            StubURLProtocol.onRequest = {
                if StubURLProtocol.requestCount == 2 { clock.set(10.434) }
            }
        }
        let backoffs = BurstTestClock(0)
        let control = RoverControl(session: URLSession(configuration: .stubbed), retrySleep: { _ in
            backoffs.set(backoffs.now + 1)
        })
        let gate = BurstAttemptAdmissionGate()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "classification", monotonic: { clock.now },
            utc: { Date() }, sink: sink.append)
        var snapshot = NavigationPoseSample(pose: .init(position: .zero, yaw: 0),
            frameID: .init(generation: 1, sequence: 1), sourceTimestamp: 9.99, trackingQuality: .normal)
        var stops = 0
        var senderCalls = 0
        var completed = false
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { await control.lastAckAt }, sendCommand: { _ in
                XCTFail("Follow turn must use receipt transport")
            }, stopRover: {
                stops += 1
                _ = try await control.stopWithReceipt().get()
            }, sleep: { try? await Task.sleep(for: $0) }, sendCommandReceipt: { command in
                senderCalls += 1
                let original = FollowTurnBurstTransportScope.authorization!
                if holdAuthorization {
                    let held = FollowTurnBurstAuthorization(operationID: original.operationID,
                        sendEntryUptime: original.sendEntryUptime,
                        requestedBudget: original.deadline - original.sendEntryUptime,
                        uptime: original.uptime, isAuthorized: original.isAuthorized,
                        authorizeAttempt: {
                            await gate.wait()
                            return await original.authorizeAttempt?() ?? true
                        }, didEnterAttempt: original.didEnterAttempt, transportCapture: original.transportCapture)
                    let result = await control.sendNavigationWithReceipt(command, authorization: held)
                    if contradictoryEntry {
                        original.didEnterAttempt?(.init(operationID: original.operationID, attempt: 1, entryUptime: clock.now))
                    }
                    return result
                }
                return await control.sendNavigationWithReceipt(command)
            }, diagnosticEmitter: emitter, poseSample: { snapshot }, sourceNow: { clock.now },
            sourceStopSnapshot: { snapshot }, transportUptime: { clock.now })
        let motion = NavigationFollowMeMotion(navigation: controller)
        var deliveries: [FollowMotionFailureDelivery] = []
        let failures = motion.motionFailures()
        let collector = Task { for await delivery in failures { deliveries.append(delivery) } }
        defer { collector.cancel() }
        if holdStop { StubURLProtocol.suspendRequestNumber = 2 }
        let request = FollowMotionRequestContext(sessionGeneration: 7, requestToken: 2,
            purpose: .followAlignment, phase: "aligning")
        // With the five-degree window, this worked angle preserves the original
        // 2.374ms provisional budget and still exercises transport admission.
        let completion = expectation(description: "Classified turn and confirmed safety stop completed")
        let task = Task {
            defer { completion.fulfill() }
            let result = await motion.perform(.alignment(0.09223912712382312), context: request)
            completed = true
            return result
        }
        let admissionDeadline = ContinuousClock.now + .seconds(2)
        while controller.followTurnStopFence == nil, ContinuousClock.now < admissionDeadline {
            try? await Task.sleep(for: .milliseconds(1))
        }
        XCTAssertNotNil(controller.followTurnStopFence, "Initial stop fence must be captured before advancing source time")
        XCTAssertEqual(stops, 1, "Initial stop must acknowledge before source admission")
        clock.set(10.301)
        snapshot = .init(pose: .init(position: .zero, yaw: 0), frameID: .init(generation: 1, sequence: 2),
            sourceTimestamp: 10.2, trackingQuality: .normal)
        controller.ingestFollowTurnSource(snapshot)
        if holdAuthorization {
            let authorizationDeadline = ContinuousClock.now + .seconds(2)
            while ContinuousClock.now < authorizationDeadline {
                if await gate.isWaiting { break }
                try? await Task.sleep(for: .milliseconds(1))
            }
            let waiting = await gate.isWaiting
            XCTAssertTrue(waiting, "Suspend inside real RoverControl before the MainActor attempt check")
            XCTAssertEqual(StubURLProtocol.requestCount, 1, "No nonzero HTTP attempt before eligibility")
            clock.set(10.434) // 133ms scheduling delay consumes the original 2.374ms budget.
            if healthLoss { controller.ingestFollowTurnSource(.unavailable) }
            await gate.release()
        }
        if holdStop {
            let stopDeadline = ContinuousClock.now + .seconds(2)
            while StubURLProtocol.suspended == nil, ContinuousClock.now < stopDeadline {
                try? await Task.sleep(for: .milliseconds(1))
            }
            XCTAssertNotNil(StubURLProtocol.suspended)
            for _ in 0..<100 { await Task.yield() }
            XCTAssertFalse(completed, "Stop response is held")
            XCTAssertEqual(deliveries.count, 1, "Specific failure must be published before stop returns")
            if let pending = deliveries.first {
                XCTAssertEqual(pending.reason, .rotationResolutionInsufficient)
                XCTAssertEqual(pending.context.failureCause, .burstPreSendExpired)
                XCTAssertEqual(pending.stopOutcome, .pending)
                XCTAssertEqual(FollowMotionFailureResolution(pending).message,
                    "Turn not started: command scheduling exceeded burst budget. Confirming motor stop…")
            }
            StubURLProtocol.releaseFirst()
        }
        // Yield counts are not a deadline for real URLSession and actor work.
        // Cancelling too early consumes the scripted motor failure as the STOP
        // response and tests a different failure path entirely.
        await fulfillment(of: [completion], timeout: 2)
        if !completed { task.cancel(); await gate.release() }
        let result = await task.value
        XCTAssertTrue(completed)
        XCTAssertEqual(senderCalls, 1, "Terminal failure cannot retry or launch another burst")
        XCTAssertEqual(stops, 2, "Real transport confirms serialized safety stop")
        XCTAssertEqual(StubURLProtocol.requestCount, holdAuthorization ? 2 : 3)
        XCTAssertEqual(backoffs.now, 0)
        XCTAssertEqual(result.stopOutcome, failStop ? .failed : .confirmed)
        if holdStop, let pending = deliveries.first, let terminal = result.failure {
            var resolution = FollowMotionFailureResolution(pending)
            resolution.consume(terminal)
            XCTAssertEqual(resolution.context.failureCause, .burstPreSendExpired)
            XCTAssertEqual(resolution.stopOutcome, .confirmed)
        }
        XCTAssertFalse(sink.records.contains { $0.event == "follow_scan.burst_response" },
            "Failed/unexecuted work must not learn a response")
        let ack = try XCTUnwrap(sink.records.first { $0.event == "follow_scan.send_ack" })
        let payload = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap(ack.fields["payload"]).utf8)) as? [String: Any])
        XCTAssertEqual(payload["sender_outcome"] as? String,
            holdAuthorization || expiryAfterAttempt ? "expired" : "failed")
        XCTAssertEqual(payload["attempts"] as? Int, holdAuthorization ? 0 : 1)
        XCTAssertEqual((payload["transport_attempt_entries"] as? [Any])?.count, holdAuthorization && !contradictoryEntry ? 0 : 1)
        XCTAssertEqual(try XCTUnwrap(payload["requested_host_budget_s"] as? Double),
            0.0023742724180478418, accuracy: 1e-12)
        XCTAssertEqual(payload["requested_additional_wait_s"] as? Double, 0)
        if holdAuthorization {
            XCTAssertEqual(try XCTUnwrap(payload["send_entry_to_response_s"] as? Double), 0.133, accuracy: 1e-12)
            XCTAssertEqual(payload["sender_failure_reason"] as? String, "expired")
        }
        return result
    }

    @MainActor
    func testControllerTraceRecordsActualAttemptAndExpiredBackoffWithoutRetryOrInventedAck() async throws {
        StubURLProtocol.results = Array(repeating: .failure(URLError(.timedOut)), count: 3)
        let clock = BurstTestClock(20)
        let control = RoverControl(session: URLSession(configuration: .stubbed), retrySleep: { _ in clock.set(20.080) })
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "backoff", monotonic: { 1000 }, utc: { Date() }, sink: sink.append)
        let evidence = FollowMotionOperationEvidence(context: .init(request:
            .init(sessionGeneration: 23, requestToken: 47, purpose: .followScan, phase: "scanning"),
            controllerOperationID: 99, purpose: .followScan, profile: .turnBurst(purpose: .followScan)))
        evidence.scanTrace = .init(emitter: emitter)
        evidence.burstTrace = .init()
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in }, stopRover: {},
            sleep: { try? await Task.sleep(for: $0) }, sendCommandReceipt: { command in
                clock.set(20.005)
                return await control.sendNavigationWithReceipt(command)
            }, diagnosticEmitter: emitter, sourceNow: { clock.now }, transportUptime: { clock.now })
        let receipt = try await FollowMotionTaskScope.$evidence.withValue(evidence) {
            try await controller.executeFollowTurnBurst(.init(left: -0.25, right: 0.25), requestedBudget: 0.080, purpose: .followScan)
        }
        XCTAssertEqual(receipt.send.result.receipt.outcome, "expired")
        XCTAssertEqual(receipt.send.result.receipt.acknowledged, false)
        XCTAssertEqual(StubURLProtocol.requestCount, 1)
        let record = try XCTUnwrap(sink.records.first { $0.event == "follow_scan.send_ack" })
        let ack = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap(record.fields["payload"]).utf8)) as? [String: Any])
        XCTAssertEqual(ack["sender_outcome"] as? String, "expired")
        XCTAssertEqual(ack["sender_failure_reason"] as? String, "expired")
        XCTAssertEqual(ack["transport_entry_uptime_s"] as? Double, 20.005)
        XCTAssertEqual(ack["attempts"] as? Int, 1)
        XCTAssertEqual(ack["operation_id"] as? Int, 99)
        XCTAssertEqual(ack["session_generation"] as? Int, 23)
        XCTAssertEqual((ack["transport_attempt_entries"] as? [Any])?.count, 1)
        XCTAssertEqual(ack["requested_additional_wait_s"] as? Double, 0)
    }

    @MainActor
    func testProductionSourceWaitCannotUseCachedNormalFrameAfterARInterruption() async throws {
        let ar = ARSessionManager()
        ar.resetForTesting()
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 8, 6, kCVPixelFormatType_32BGRA, nil, &image)
        let pixels = try XCTUnwrap(image)
        func ingest() {
            ar.ingestForTesting(image: pixels, timestamp: ProcessInfo.processInfo.systemUptime,
                cameraTransform: matrix_identity_float4x4, intrinsics: matrix_identity_float3x3,
                imageResolution: CGSize(width: 8, height: 6), depthMap: nil, trackingQuality: .normal)
        }
        ingest()
        StubURLProtocol.results = [.success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!))]
        let controller = FollowNavigationController(ar: ar, control: RoverControl(session: URLSession(configuration: .stubbed)))
        let wait = Task { await controller.prepareFollowTurnSource() }
        for _ in 0..<1000 where controller.followTurnStopFence == nil { await Task.yield() }
        XCTAssertNotNil(controller.followTurnStopFence)
        ingest() // A genuine post-ack capture during settle, still normal/fresh in the cache.
        for _ in 0..<30 { await Task.yield() }
        ar.interruptionBeganForTesting()
        let result = await wait.value
        guard case .failed(let reason) = result else { XCTFail("Current AR interruption must invalidate cached normal evidence"); return }
        XCTAssertEqual(reason, .trackingLost)
        XCTAssertEqual(StubURLProtocol.requestCount, 1, "Only the initial authoritative stop, no motion")
    }
    func testExpiryDuringRetryBackoffPreventsTheSecondHTTPAttempt() async {
        StubURLProtocol.results = Array(repeating: .failure(URLError(.timedOut)), count: 3)
        let clock = BurstTestClock(10)
        let backoffs = BurstTestClock(0)
        let control = RoverControl(session: URLSession(configuration: .stubbed), retrySleep: { _ in
            backoffs.set(backoffs.now + 1)
            clock.set(10.080)
        })
        let authorization = FollowTurnBurstAuthorization(operationID: 13, sendEntryUptime: 10,
            requestedBudget: 0.080, uptime: { clock.now })
        let result = await control.sendNavigationWithReceipt(.init(left: -0.25, right: 0.25), authorization: authorization)
        XCTAssertEqual(backoffs.now, 1)
        XCTAssertEqual(result.receipt.outcome, "expired")
        XCTAssertEqual(result.receipt.attempts, 1)
        XCTAssertEqual(StubURLProtocol.requestCount, 1)
    }
    func testExpiredResponseDrainsWithoutAddingRetryBackoffBeforeStopAdmission() async {
        StubURLProtocol.results = Array(repeating: .failure(URLError(.timedOut)), count: 3)
        let clock = BurstTestClock(10)
        let backoffs = BurstTestClock(0)
        StubURLProtocol.onRequest = { clock.set(10.100) }
        let control = RoverControl(session: URLSession(configuration: .stubbed), retrySleep: { _ in
            backoffs.set(backoffs.now + 1)
        })
        let authorization = FollowTurnBurstAuthorization(operationID: 12, sendEntryUptime: 10,
            requestedBudget: 0.080, uptime: { clock.now })
        let result = await control.sendNavigationWithReceipt(.init(left: -0.25, right: 0.25), authorization: authorization)
        XCTAssertEqual(backoffs.now, 0, "Known expiry adds no deliberate backoff before serialized stop")
        XCTAssertEqual(result.receipt.outcome, "expired")
        XCTAssertEqual(result.receipt.attempts, 1)
        XCTAssertEqual(StubURLProtocol.requestCount, 1)
    }
    func testCancellationAtAuthorizationBoundaryIsCancelledRatherThanFenced() async {
        let entered = expectation(description: "authorization actor boundary")
        let suspension = AsyncStream<Void>.makeStream()
        let authorization = FollowTurnBurstAuthorization(operationID: 11, sendEntryUptime: 10,
            requestedBudget: 0.080, uptime: { 10.01 }, authorizeAttempt: {
                entered.fulfill()
                var iterator = suspension.stream.makeAsyncIterator()
                _ = await iterator.next()
                return false
            })
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let task = Task { await control.sendNavigationWithReceipt(.init(left: -0.25, right: 0.25), authorization: authorization) }
        await fulfillment(of: [entered], timeout: 2)
        task.cancel()
        suspension.continuation.yield(())
        let result = await task.value
        XCTAssertEqual(result.receipt.outcome, "cancelled")
        XCTAssertEqual(result.receipt.attempts, 0)
        XCTAssertEqual(result.receipt.acknowledged, false)
        XCTAssertEqual(StubURLProtocol.requestCount, 0)
        suspension.continuation.finish()
    }
    @MainActor
    func testPublicCancelWaitsForPendingBurstBeforeItsIndependentStop() async {
        let accepted = (Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!)
        StubURLProtocol.results = [.success(accepted), .success(accepted)]
        StubURLProtocol.suspendFirst = true
        let entered = expectation(description: "pending send")
        entered.assertForOverFulfill = false
        StubURLProtocol.onRequest = { entered.fulfill() }
        var stopReturned = false
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in },
            stopRover: { _ = try await control.stopWithReceipt().get(); stopReturned = true },
            sleep: { try? await Task.sleep(for: $0) },
            sendCommandReceipt: { await control.sendNavigationWithReceipt($0) },
            sourceNow: { 10 }, transportUptime: { 10 })
        let send = Task { await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25),
            requestedBudget: 0.080, purpose: .followScan) }
        await fulfillment(of: [entered], timeout: 2)
        controller.cancel()
        for _ in 0..<100 { await Task.yield() }
        XCTAssertFalse(stopReturned)
        XCTAssertEqual(StubURLProtocol.requestCount, 1)
        StubURLProtocol.releaseFirst()
        _ = await send.value
        for _ in 0..<1000 where !stopReturned { await Task.yield() }
        XCTAssertTrue(stopReturned)
        XCTAssertEqual(StubURLProtocol.requestCount, 2)
    }

    func testGenericAndStopStillUseThreeAttemptsDespiteExpiredFollowContext() async {
        let authorization = FollowTurnBurstAuthorization(operationID: 9, sendEntryUptime: 10,
            requestedBudget: 0.080, uptime: { 11 }, isAuthorized: { false })
        for stopping in [false, true] {
            StubURLProtocol.reset()
            StubURLProtocol.results = [.failure(URLError(.timedOut)), .failure(URLError(.timedOut)),
                .success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
                    statusCode: 200, httpVersion: nil, headerFields: nil)!))]
            let control = RoverControl(session: URLSession(configuration: .stubbed))
            await FollowTurnBurstTransportScope.$authorization.withValue(authorization) {
                if stopping {
                    let result = await control.stopWithReceipt()
                    XCTAssertEqual(result.receipt.attempts, 3)
                    XCTAssertEqual(result.receipt.acknowledged, true)
                } else {
                    do { try await control.send(.init(left: 0.1, right: 0.1)) }
                    catch { XCTFail("Generic retry behavior changed: \(error)") }
                }
            }
            XCTAssertEqual(StubURLProtocol.requestCount, 3)
        }
    }
    @MainActor
    func testControllerRecordsActualTransportEntrySeparatelyFromSenderQueueAndLateAck() async {
        StubURLProtocol.results = [.success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!))]
        let clock = BurstTestClock(10)
        StubURLProtocol.onRequest = { clock.set(10.100) }
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in }, stopRover: {},
            sleep: { try? await Task.sleep(for: $0) }, sendCommandReceipt: { command in
                clock.set(10.030)
                return await control.sendNavigationWithReceipt(command)
            }, sourceNow: { clock.now }, transportUptime: { clock.now })
        let receipt = await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25),
            requestedBudget: 0.080, purpose: .followAlignment)
        XCTAssertEqual(receipt.sendEntryUptime, 10)
        XCTAssertEqual(receipt.deadline, 10.080)
        XCTAssertEqual(receipt.transportAttempts.count, 1)
        XCTAssertEqual(receipt.transportAttempts.first?.entryUptime, 10.030)
        XCTAssertEqual(receipt.transportAttempts.first?.attempt, 1)
        XCTAssertEqual(receipt.responseUptime, 10.100)
        XCTAssertEqual(receipt.result.receipt.acknowledged, true)
        XCTAssertEqual(receipt.result.receipt.outcome, "acknowledged")
        XCTAssertTrue(receipt.stopObligation)
        XCTAssertEqual(StubURLProtocol.requestCount, 1)
    }
    @MainActor
    func testPendingBurstCannotBeOverwrittenByNewSenderEntry() async {
        let accepted = (Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!)
        StubURLProtocol.results = [.success(accepted), .success(accepted)]
        StubURLProtocol.suspendFirst = true
        let entered = expectation(description: "first sender entered")
        entered.assertForOverFulfill = false
        StubURLProtocol.onRequest = { entered.fulfill() }
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in }, stopRover: {},
            sleep: { try? await Task.sleep(for: $0) },
            sendCommandReceipt: { await control.sendNavigationWithReceipt($0) },
            sourceNow: { 10 }, transportUptime: { 10 })
        let first = Task { await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25),
            requestedBudget: 0.080, purpose: .followScan) }
        await fulfillment(of: [entered], timeout: 2)
        let identity = controller.followTurnBurstPendingStatus?.operationID
        let second = await controller.sendFollowTurnBurst(.init(left: 0.25, right: -0.25),
            requestedBudget: 0.080, purpose: .followAlignment)
        XCTAssertEqual(second.result.receipt.outcome, "fenced")
        XCTAssertEqual(second.result.receipt.attempts, 0)
        XCTAssertEqual(StubURLProtocol.requestCount, 1)
        XCTAssertEqual(controller.followTurnBurstPendingStatus?.operationID, identity)
        StubURLProtocol.releaseFirst()
        _ = await first.value
        XCTAssertNil(controller.followTurnBurstPendingStatus)
    }
    @MainActor
    func testControllerRejectsInvalidOrUnrepresentableBudgetWithoutRoundingToFloor() async {
        let accepted = (Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!)
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in }, stopRover: {},
            sleep: { _ in }, sendCommandReceipt: { await control.sendNavigationWithReceipt($0) },
            sourceNow: { 10 }, transportUptime: { 10 })
        // Zero first: it already has a valid Duration and exposes the missing validation without a crash.
        for budget in [0.0, -0.01, Double.leastNonzeroMagnitude, 0.081, .nan, .infinity] {
            StubURLProtocol.reset()
            StubURLProtocol.results = [.success(accepted)]
            let receipt = await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25),
                requestedBudget: budget, purpose: .followAlignment)
            XCTAssertEqual(StubURLProtocol.requestCount, 0)
            XCTAssertEqual(receipt.result.receipt.outcome, "invalid_budget")
            XCTAssertEqual(receipt.result.receipt.acknowledged, false)
        }
        StubURLProtocol.reset()
        StubURLProtocol.results = [.success(accepted)]
        let tiny = await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25),
            requestedBudget: 0.001, purpose: .followAlignment)
        XCTAssertEqual(tiny.deadline, 10.001)
        XCTAssertEqual(tiny.result.receipt.attempts, 1)
        XCTAssertEqual(StubURLProtocol.requestCount, 1)
    }
    @MainActor
    func testPendingBurstExpiryOnlyMarksObligationAndStopDrainsActualHTTP() async {
        let accepted = (Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!)
        StubURLProtocol.results = [.success(accepted), .success(accepted)]
        StubURLProtocol.suspendFirst = true
        let entered = expectation(description: "pending first HTTP")
        entered.assertForOverFulfill = false
        StubURLProtocol.onRequest = { entered.fulfill() }
        let clock = BurstTestClock(10)
        var budgetWait: CheckedContinuation<Void, Never>?
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in },
            stopRover: { _ = try await control.stopWithReceipt().get() },
            sleep: { _ in await withCheckedContinuation { budgetWait = $0 } },
            sendCommandReceipt: { await control.sendNavigationWithReceipt($0) },
            sourceNow: { clock.now }, transportUptime: { clock.now })
        let send = Task { await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25),
            requestedBudget: 0.080, purpose: .followScan) }
        await fulfillment(of: [entered], timeout: 2)
        clock.set(10.100)
        for _ in 0..<100 where budgetWait == nil { await Task.yield() }
        budgetWait?.resume(); budgetWait = nil
        for _ in 0..<100 where controller.followTurnBurstPendingStatus?.stopObligation != true { await Task.yield() }
        XCTAssertEqual(controller.followTurnBurstPendingStatus?.stopObligation, true)
        let stop = Task { try? await controller.stopAndConfirm() }
        for _ in 0..<50 { await Task.yield() }
        XCTAssertEqual(StubURLProtocol.requestCount, 1, "No stop may overtake pending nonzero HTTP")
        StubURLProtocol.releaseFirst()
        let receipt = await send.value
        _ = await stop.value
        XCTAssertTrue(receipt.stopObligation)
        XCTAssertEqual(receipt.result.receipt.outcome, "acknowledged", "Real late ACK remains truthful")
        XCTAssertEqual(receipt.result.receipt.attempts, 1)
        XCTAssertEqual(StubURLProtocol.requestCount, 2)
    }
    @MainActor
    func testControllerSenderQueueConsumesWhole80msBudgetBeforeHTTPEntry() async {
        StubURLProtocol.results = [.success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
            statusCode: 200, httpVersion: nil, headerFields: nil)!))]
        let clock = BurstTestClock(10)
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { .infinity },
            plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in }, stopRover: {},
            sleep: { try? await Task.sleep(for: $0) }, sendCommandReceipt: { command in
                clock.set(10.080)
                return await control.sendNavigationWithReceipt(command)
            }, sourceNow: { clock.now }, transportUptime: { clock.now })
        let receipt = await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25),
            requestedBudget: 0.080, purpose: .followAlignment)
        XCTAssertEqual(receipt.sendEntryUptime, 10)
        XCTAssertEqual(receipt.deadline, 10.080)
        XCTAssertEqual(StubURLProtocol.requestCount, 0)
        XCTAssertEqual(receipt.result.receipt.attempts, 0)
        XCTAssertEqual(receipt.result.receipt.outcome, "expired")
        XCTAssertTrue(receipt.stopObligation)
        XCTAssertEqual(receipt.result.receipt.acknowledged, false)
    }
    func testAttemptRechecksDeadlineAndFenceAfterAuthorizationActorBoundary() async {
        for expires in [true, false] {
            StubURLProtocol.reset()
            StubURLProtocol.results = [.failure(URLError(.timedOut))]
            let clock = BurstTestClock(10)
            let authority = BurstTestClock(1)
            let authorization = FollowTurnBurstAuthorization(operationID: 8, sendEntryUptime: 10,
                requestedBudget: 0.080, uptime: { clock.now }, isAuthorized: { authority.now == 1 },
                authorizeAttempt: {
                    if expires { clock.set(10.080) } else { authority.set(0) }
                    return true
                })
            let control = RoverControl(session: URLSession(configuration: .stubbed))
            let result = await control.sendNavigationWithReceipt(.init(left: -0.25, right: 0.25), authorization: authorization)
            XCTAssertEqual(StubURLProtocol.requestCount, 0)
            XCTAssertEqual(result.receipt.attempts, 0)
            XCTAssertEqual(result.receipt.outcome, expires ? "expired" : "fenced")
            XCTAssertEqual(result.receipt.acknowledged, false)
        }
    }
    func testFollowBurstCancellationDuringBackoffCannotBeSwallowedIntoRetry() async {
        StubURLProtocol.results = Array(repeating: .failure(URLError(.timedOut)), count: 3)
        let entered = expectation(description: "first HTTP attempt")
        entered.assertForOverFulfill = false
        StubURLProtocol.onRequest = { entered.fulfill() }
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let authorization = FollowTurnBurstAuthorization(operationID: 4, sendEntryUptime: 10,
            requestedBudget: 0.080, uptime: { 10.01 })
        let task = Task {
            await control.sendNavigationWithReceipt(.init(left: -0.25, right: 0.25), authorization: authorization)
        }
        await fulfillment(of: [entered], timeout: 2)
        try? await Task.sleep(for: .milliseconds(20))
        task.cancel()
        let result = await task.value
        XCTAssertEqual(StubURLProtocol.requestCount, 1)
        XCTAssertEqual(result.receipt.attempts, 1)
        XCTAssertEqual(result.receipt.outcome, "cancelled")
        XCTAssertEqual(result.receipt.acknowledged, false)
    }
    func testFollowBurstAuthorityLossBeforeFirstAttemptAndDuringBackoff() async {
        for beforeFirst in [true, false] {
            StubURLProtocol.reset()
            StubURLProtocol.results = Array(repeating: .failure(URLError(.timedOut)), count: 3)
            let authority = BurstTestClock(beforeFirst ? 0 : 1)
            StubURLProtocol.onRequest = { authority.set(0) }
            let control = RoverControl(session: URLSession(configuration: .stubbed))
            let authorization = FollowTurnBurstAuthorization(operationID: 3, sendEntryUptime: 10,
                requestedBudget: 0.080, uptime: { 10.01 }, isAuthorized: { authority.now == 1 })
            let result = await control.sendNavigationWithReceipt(.init(left: -0.25, right: 0.25), authorization: authorization)
            XCTAssertEqual(StubURLProtocol.requestCount, beforeFirst ? 0 : 1)
            XCTAssertEqual(result.receipt.attempts, beforeFirst ? 0 : 1)
            XCTAssertEqual(result.receipt.outcome, "fenced")
            XCTAssertEqual(result.receipt.acknowledged, false)
        }
    }
    func testFollowBurstFirstTimeoutAt100msCannotRetryAn80msDeadline() async {
        StubURLProtocol.results = [.failure(URLError(.timedOut)), .failure(URLError(.timedOut)), .failure(URLError(.timedOut))]
        let clock = BurstTestClock(10)
        StubURLProtocol.onRequest = { clock.set(10.100) }
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let authorization = FollowTurnBurstAuthorization(operationID: 1, sendEntryUptime: 10,
            requestedBudget: 0.080, uptime: { clock.now })
        let result = await control.sendNavigationWithReceipt(.init(left: -0.25, right: 0.25), authorization: authorization)
        XCTAssertEqual(StubURLProtocol.requestCount, 1)
        XCTAssertEqual(result.receipt.attempts, 1)
        XCTAssertEqual(result.receipt.acknowledged, false)
        XCTAssertEqual(result.receipt.outcome, "expired")
        XCTAssertTrue(result.failure is FollowTurnBurstTransportDenial)
        let ack = await control.lastAckAt
        XCTAssertNil(ack)
    }
    func testUnknownReceiptDoesNotClaimTransportFacts() {
        XCTAssertNil(RoverCommandDiagnosticReceipt.unknown.httpStatus)
        XCTAssertNil(RoverCommandDiagnosticReceipt.unknown.acknowledged)
        XCTAssertNil(RoverCommandDiagnosticReceipt.unknown.acknowledgementUTC)
        XCTAssertNil(RoverCommandDiagnosticReceipt.unknown.attempts)
        XCTAssertEqual(RoverCommandDiagnosticReceipt.unknown.outcome, "unknown")
    }

    func testReceiptsPreserveRealAcceptedStatusAndAckClock() async throws {
        for status in [200, 204, 299] {
            StubURLProtocol.reset()
            StubURLProtocol.results = [.success((Data(), HTTPURLResponse(
                url: URL(string: "http://192.168.4.1/js")!, statusCode: status,
                httpVersion: nil, headerFields: nil)!))]
            let control = RoverControl(session: URLSession(configuration: .stubbed))
            let result = await control.sendNavigationWithReceipt(.init(left: -0.1, right: 0.1))
            XCTAssertNil(result.failure)
            XCTAssertEqual(result.receipt.httpStatus, status)
            XCTAssertEqual(result.receipt.acknowledged, true)
            XCTAssertEqual(result.receipt.attempts, 1)
            let ack = await control.lastAckAt
            XCTAssertEqual(result.receipt.acknowledgementUTC, ack)
            XCTAssertNotNil(ack)
        }
    }

    override func tearDown() {
        StubURLProtocol.reset()
        super.tearDown()
    }

    func testFailureReceiptsKeepActualStatusAttemptsAndOriginalErrors() async throws {
        let url = URL(string: "http://192.168.4.1/js")!
        let cases: [(Result<(Data, URLResponse), Error>, Int?, Int, String)] = [
            (.success((Data(), HTTPURLResponse(url: url, statusCode: 503, httpVersion: nil, headerFields: nil)!)), 503, 1, "failed"),
            (.success((Data(), URLResponse(url: url, mimeType: nil, expectedContentLength: 0, textEncodingName: nil))), nil, 1, "failed"),
            (.failure(URLError(.timedOut)), nil, 3, "failed"),
            (.failure(URLError(.cancelled)), nil, 1, "cancelled")
        ]
        for (response, status, attempts, outcome) in cases {
            StubURLProtocol.reset()
            StubURLProtocol.results = Array(repeating: response, count: attempts)
            let control = RoverControl(session: URLSession(configuration: .stubbed))
            let result = await control.sendNavigationWithReceipt(.init(left: -0.1, right: 0.1))
            XCTAssertNotNil(result.failure)
            XCTAssertThrowsError(try result.get())
            XCTAssertEqual(result.receipt.httpStatus, status)
            XCTAssertEqual(result.receipt.attempts, attempts)
            XCTAssertEqual(result.receipt.outcome, outcome)
            XCTAssertEqual(result.receipt.acknowledged, false)
            XCTAssertNil(result.receipt.acknowledgementUTC)
            XCTAssertEqual(StubURLProtocol.requestCount, attempts)
            let ack = await control.lastAckAt
            XCTAssertNil(ack)
        }
    }

    func testStopAndRetryReceiptsDescribeTheirOwnAcknowledgement() async throws {
        StubURLProtocol.results = [.failure(URLError(.timedOut)), .success((Data(), HTTPURLResponse(
            url: URL(string: "http://192.168.4.1/js")!, statusCode: 204, httpVersion: nil, headerFields: nil)!))]
        let control = RoverControl(session: URLSession(configuration: .stubbed))
        let result = await control.stopWithReceipt()
        XCTAssertNil(result.failure)
        XCTAssertEqual(result.receipt.httpStatus, 204)
        XCTAssertEqual(result.receipt.attempts, 2)
        XCTAssertEqual(result.receipt.acknowledged, true)
        XCTAssertEqual(result.receipt.outcome, "acknowledged")
        let url = try XCTUnwrap(StubURLProtocol.lastRequest?.url)
        let json = try XCTUnwrap(URLComponents(url: url, resolvingAgainstBaseURL: false)?.queryItems?.first?.value)
        let payload = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(json.utf8)) as? [String: Any])
        XCTAssertEqual(payload["T"] as? Int, 1)
        XCTAssertEqual(payload["L"] as? Double, 0)
        XCTAssertEqual(payload["R"] as? Double, 0)
    }

    func testRetriesTransientCommandTimeoutBeforeFailingNavigationLink() async throws {
        StubURLProtocol.results = [
            .failure(URLError(.timedOut)),
            .success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
                                              statusCode: 200,
                                              httpVersion: nil,
                                              headerFields: nil)!))
        ]
        let session = URLSession(configuration: .stubbed)
        let control = RoverControl(session: session)

        try await control.send(.init(left: 0.1, right: 0.1))

        XCTAssertEqual(StubURLProtocol.requestCount, 2)
        let lastAckAt = await control.lastAckAt
        XCTAssertNotNil(lastAckAt)
    }

    func testDoesNotRetryServerErrors() async {
        StubURLProtocol.results = [
            .success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
                                              statusCode: 500,
                                              httpVersion: nil,
                                              headerFields: nil)!))
        ]
        let session = URLSession(configuration: .stubbed)
        let control = RoverControl(session: session)

        do {
            try await control.send(.init(left: 0.1, right: 0.1))
            XCTFail("Expected server error")
        } catch RoverControlError.serverError(let code) {
            XCTAssertEqual(code, 500)
        } catch {
            XCTFail("Expected server error, got \(error)")
        }

        XCTAssertEqual(StubURLProtocol.requestCount, 1)
    }

    func testRetriesTwoTransientTimeoutsBeforeFailingNavigationLink() async throws {
        StubURLProtocol.results = [
            .failure(URLError(.timedOut)),
            .failure(URLError(.timedOut)),
            .success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
                                              statusCode: 200,
                                              httpVersion: nil,
                                              headerFields: nil)!))
        ]
        let session = URLSession(configuration: .stubbed)
        let control = RoverControl(session: session)

        try await control.send(.init(left: 0.1, right: 0.1))

        XCTAssertEqual(StubURLProtocol.requestCount, 3)
        let lastAckAt = await control.lastAckAt
        XCTAssertNotNil(lastAckAt)
    }

    func testRequestLogFieldsIncludeURLAndHTTPStatus() {
        let fields = RoverControl.requestLogFields(url: URL(string: "http://192.168.4.1/js?json=%7B%7D")!,
                                                   attempt: 2,
                                                   maxAttempts: 3,
                                                   statusCode: 200,
                                                   error: nil)

        XCTAssertEqual(fields["url"], "http://192.168.4.1/js?json=%7B%7D")
        XCTAssertEqual(fields["attempt"], "2")
        XCTAssertEqual(fields["max"], "3")
        XCTAssertEqual(fields["status"], "200")
        XCTAssertNil(fields["error"])
    }

    func testCommandRequestsDisableStaleConnectionReuse() async throws {
        StubURLProtocol.results = [
            .success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
                                              statusCode: 200,
                                              httpVersion: nil,
                                              headerFields: nil)!))
        ]
        let session = URLSession(configuration: .stubbed)
        let control = RoverControl(session: session)

        try await control.send(.init(left: 0.1, right: 0.1))

        XCTAssertEqual(StubURLProtocol.lastRequest?.value(forHTTPHeaderField: "Connection"), "close")
        XCTAssertEqual(StubURLProtocol.lastRequest?.cachePolicy, .reloadIgnoringLocalCacheData)
    }

    func testCommandMapsNavigationYawToWaveRoverWheelDirection() async throws {
        StubURLProtocol.results = [
            .success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
                                              statusCode: 200,
                                              httpVersion: nil,
                                              headerFields: nil)!))
        ]
        let session = URLSession(configuration: .stubbed)
        let control = RoverControl(session: session)

        try await control.sendNavigation(.init(left: -0.25, right: 0.25))

        let requestURL = try XCTUnwrap(StubURLProtocol.lastRequest?.url)
        let json = try XCTUnwrap(URLComponents(url: requestURL, resolvingAgainstBaseURL: false)?
            .queryItems?.first(where: { $0.name == "json" })?.value)
        let payload = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(json.utf8)) as? [String: Any])
        XCTAssertEqual(payload["L"] as? Double, 0.25)
        XCTAssertEqual(payload["R"] as? Double, -0.25)
    }

    func testManualCommandPreservesPhysicalWheelDirection() async throws {
        StubURLProtocol.results = [
            .success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
                                              statusCode: 200,
                                              httpVersion: nil,
                                              headerFields: nil)!))
        ]
        let session = URLSession(configuration: .stubbed)
        let control = RoverControl(session: session)

        try await control.send(.init(left: -0.25, right: 0.25))

        let requestURL = try XCTUnwrap(StubURLProtocol.lastRequest?.url)
        let json = try XCTUnwrap(URLComponents(url: requestURL, resolvingAgainstBaseURL: false)?
            .queryItems?.first(where: { $0.name == "json" })?.value)
        let payload = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(json.utf8)) as? [String: Any])
        XCTAssertEqual(payload["L"] as? Double, -0.25)
        XCTAssertEqual(payload["R"] as? Double, 0.25)
    }

    func testProbeLinkUsesFeedbackFlowCommandAndRefreshesAcknowledgement() async throws {
        StubURLProtocol.results = [
            .success((Data(), HTTPURLResponse(url: URL(string: "http://192.168.4.1/js")!,
                                              statusCode: 204,
                                              httpVersion: nil,
                                              headerFields: nil)!))
        ]
        let control = RoverControl(session: URLSession(configuration: .stubbed))

        try await control.probeLink()

        let requestURL = try XCTUnwrap(StubURLProtocol.lastRequest?.url)
        let json = try XCTUnwrap(URLComponents(url: requestURL, resolvingAgainstBaseURL: false)?
            .queryItems?.first(where: { $0.name == "json" })?.value)
        let payload = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(json.utf8)) as? [String: Any])
        XCTAssertEqual(payload["T"] as? Int, 131)
        XCTAssertEqual(payload["cmd"] as? Int, 1)
        XCTAssertNil(payload["L"])
        XCTAssertNil(payload["R"])
        let lastAckAt = await control.lastAckAt
        XCTAssertNotNil(lastAckAt)
    }
}

private extension URLSessionConfiguration {
    static var stubbed: URLSessionConfiguration {
        let config = URLSessionConfiguration.ephemeral
        config.protocolClasses = [StubURLProtocol.self]
        return config
    }
}

private final class StubURLProtocol: URLProtocol, @unchecked Sendable {
    private final class State: @unchecked Sendable {
        let lock = NSLock()
        var suspendFirst = false
        var suspendRequestNumber: Int?
        var suspended: StubURLProtocol?
        var onRequest: (@Sendable () -> Void)?
        var results: [Result<(Data, URLResponse), Error>] = []
        var requestCount = 0
        var lastRequest: URLRequest?
        var requests: [URLRequest] = []
    }
    private static let state = State()
    static var suspendFirst: Bool {
        get { state.lock.withLock { state.suspendFirst } }
        set { state.lock.withLock { state.suspendFirst = newValue } }
    }
    static var suspendRequestNumber: Int? {
        get { state.lock.withLock { state.suspendRequestNumber } }
        set { state.lock.withLock { state.suspendRequestNumber = newValue } }
    }
    static var suspended: StubURLProtocol? { state.lock.withLock { state.suspended } }
    static var onRequest: (@Sendable () -> Void)? {
        get { state.lock.withLock { state.onRequest } }
        set { state.lock.withLock { state.onRequest = newValue } }
    }
    static var results: [Result<(Data, URLResponse), Error>] {
        get { state.lock.withLock { state.results } }
        set { state.lock.withLock { state.results = newValue } }
    }
    static var requestCount: Int { state.lock.withLock { state.requestCount } }
    static var lastRequest: URLRequest? { state.lock.withLock { state.lastRequest } }
    static var requests: [URLRequest] { state.lock.withLock { state.requests } }

    static func reset() {
        state.lock.withLock {
            state.suspendFirst = false
            state.suspendRequestNumber = nil
            state.suspended = nil
            state.onRequest = nil
            state.results = []
            state.requestCount = 0
            state.lastRequest = nil
            state.requests = []
        }
    }

    override class func canInit(with request: URLRequest) -> Bool { true }

    override class func canonicalRequest(for request: URLRequest) -> URLRequest { request }

    override func startLoading() {
        let (callback, held) = Self.state.lock.withLock {
            Self.state.requestCount += 1
            Self.state.lastRequest = request
            Self.state.requests.append(request)
            let held = (Self.state.suspendFirst && Self.state.requestCount == 1) ||
                Self.state.suspendRequestNumber == Self.state.requestCount
            if held { Self.state.suspended = self }
            return (Self.state.onRequest, held)
        }
        callback?() // Client/injected callbacks never execute under the fixture lock.
        if held { return }
        deliverNextResult()
    }

    static func releaseFirst() {
        let first = state.lock.withLock {
            let first = state.suspended
            state.suspended = nil
            return first
        }
        first?.deliverNextResult()
    }

    private func deliverNextResult() {
        let next = Self.state.lock.withLock {
            Self.state.results.isEmpty ? nil : Self.state.results.removeFirst()
        }
        guard let next else {
            client?.urlProtocol(self, didFailWithError: URLError(.badServerResponse))
            return
        }

        switch next {
        case .success(let result):
            client?.urlProtocol(self, didReceive: result.1, cacheStoragePolicy: .notAllowed)
            client?.urlProtocol(self, didLoad: result.0)
            client?.urlProtocolDidFinishLoading(self)
        case .failure(let error):
            client?.urlProtocol(self, didFailWithError: error)
        }
    }

    override func stopLoading() {}
}

private final class BurstTestClock: @unchecked Sendable {
    private let lock = NSLock()
    private var value: Double
    init(_ value: Double) { self.value = value }
    var now: Double { lock.withLock { value } }
    func set(_ value: Double) { lock.withLock { self.value = value } }
}

private actor BurstAttemptAdmissionGate {
    private var continuation: CheckedContinuation<Void, Never>?
    private var released = false
    var isWaiting: Bool { continuation != nil }
    func wait() async {
        if released { return }
        await withCheckedContinuation { continuation = $0 }
    }
    func release() {
        released = true
        continuation?.resume()
        continuation = nil
    }
}

private final class BurstScriptedClock: @unchecked Sendable {
    private let lock = NSLock()
    private var values: [Double]
    init(_ values: [Double]) { self.values = values }
    var now: Double { lock.withLock { values.count > 1 ? values.removeFirst() : values[0] } }
}
