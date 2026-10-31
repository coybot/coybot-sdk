import CoreVideo
import simd
import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class NavigationFollowDepthRecoveryTests: XCTestCase {
    func testUnavailableDepthPreservesCauseInsteadOfInventingObstacle() async {
        let fixture = DepthRecoveryFixture()
        fixture.depth = { command in .unavailable(.invalidCalibration, sampleAge: 0.1, motionClass: .classify(command)) }
        let result = await fixture.run(.scan(0.5))
        XCTAssertNotEqual(result.result, .failed(.obstacle))
        XCTAssertEqual(result.failure.map { FollowMotionFailureResolution($0).diagnosticReason }, "invalid_calibration")
        XCTAssertTrue(fixture.commands.isEmpty)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testInitialSearchBypassesEveryDepthStateWithoutReadingProvider() async {
        for state in [DepthSafetyState.stop, .caution, .unavailable(.missingRawDepth),
                      .unavailable(.staleRawDepth), .unavailable(.blindSweptVolume), .unavailable(.malformedDepth)] {
            let fixture = DepthRecoveryFixture()
            var reads = 0
            fixture.depth = { command in
                reads += 1
                return .init(state: state, clearance: 0.1, supportCount: 5, sampleAge: .infinity,
                    requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: nil)
            }
            fixture.sendFailure = URLError(.timedOut)
            let result = await fixture.run(.scan(.pi), initialSearch: true)
            XCTAssertEqual(fixture.commands, [.init(left: -0.25, right: 0.25)], "\(state)")
            XCTAssertEqual(reads, 0, "Initial pure rotation must never consult depth: \(state)")
            XCTAssertEqual(result.result, .failed(.commandFailed))
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
    }

    func testInitialSearchKeepsDepthBypassedAcrossBoundedPureTurnPulses() async {
        let fixture = DepthRecoveryFixture()
        fixture.beforeTransportEntry = {
            fixture.sentPose = .init(position: .zero, yaw: fixture.pose.yaw + 0.2)
        }
        _ = await fixture.run(.scan(.pi), initialSearch: true)
        XCTAssertGreaterThan(fixture.commands.count, 1)
        XCTAssertLessThan(fixture.commands.count, 30, "The initial search sweep remains bounded")
        XCTAssertTrue(fixture.commands.allSatisfy { $0.left == -$0.right && abs($0.left) <= 0.25 })
        XCTAssertEqual(fixture.depthReads, 0)
    }

    func testPreReadyAcquisitionAlignmentAndRecoveryBypassDepth() async {
        for request in [FollowMotionRequest.alignment(0.5), .scan(0.5)] {
            for state in [DepthSafetyState.stop, .unavailable(.blindSweptVolume), .unavailable(.missingRawDepth)] {
                let fixture = DepthRecoveryFixture()
                fixture.depth = { command in .init(state: state, clearance: 0.1, supportCount: 0,
                    sampleAge: 0, requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: nil) }
                fixture.sendFailure = URLError(.timedOut)
                let result = await fixture.run(request, acquiring: true)
                XCTAssertEqual(fixture.commands, [.init(left: -0.25, right: 0.25)])
                XCTAssertEqual(fixture.depthReads, 0)
                XCTAssertEqual(result.result, .failed(.commandFailed))
                XCTAssertEqual(result.stopOutcome, .confirmed)
            }
        }
    }

    func testAcquisitionFlagNeverBypassesDepthForTranslation() async {
        for request in [FollowMotionRequest.ready, .following(.init(4, 0), 1.25)] {
            let fixture = DepthRecoveryFixture()
            fixture.depth = { command in .init(state: .stop, clearance: 0.1, supportCount: 5,
                sampleAge: 0, requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: nil) }
            let result = await fixture.run(request, acquiring: true)
            XCTAssertTrue(fixture.commands.isEmpty)
            XCTAssertNotEqual(result.result, .arrived)
        }
    }

    func testPreReadyAbsoluteRecoveryBypassesBlindDepthButKeepsDeadline() async {
        for expired in [false, true] {
            let fixture = DepthRecoveryFixture()
            fixture.sendFailure = URLError(.timedOut)
            let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 1,
                deadline: expired ? 9 : 30, now: { fixture.uptime }, canContinue: { true })
            let heading = FollowRecoveryHeadingRequest(stageHeading: 0.5, authorization: authorization,
                stageIndex: 0, segmentIndex: 0, maximumSegment: .pi / 6)
            _ = await fixture.run(.scan(0.5), acquiring: true, recovery: heading)
            XCTAssertEqual(fixture.commands.count, expired ? 0 : 1)
            XCTAssertTrue(fixture.commands.allSatisfy { $0.left == -$0.right })
            XCTAssertEqual(fixture.depthReads, 0)
        }
    }

    func testQueuedInitialSearchStillRejectsTrackingStaleSourceAndOwnershipLoss() async {
        for cause in 0..<3 {
            let fixture = DepthRecoveryFixture()
            fixture.beforeTransportEntry = {
                if cause == 2 { fixture.controller.inhibitForHandoff() }
                else {
                    fixture.controller.ingestFollowTurnSource(.init(pose: fixture.pose,
                        frameID: .init(generation: 1, sequence: fixture.sequence + 1),
                        sourceTimestamp: cause == 1 ? fixture.uptime - 1 : fixture.uptime,
                        trackingQuality: cause == 0 ? .unavailable : .normal))
                }
            }
            fixture.queuedTransportDelay = 0.001
            let result = await fixture.run(.scan(.pi), initialSearch: true)
            XCTAssertTrue(fixture.commands.isEmpty, "Non-depth fence \(cause) must remain active")
            XCTAssertNotEqual(result.result, .arrived)
            XCTAssertEqual(fixture.depthReads, 0)
        }
    }

    func testInitialSearchStopFailureBlocksLaterRequests() async {
        let fixture = DepthRecoveryFixture()
        fixture.failStopAfterMovement = true
        let result = await fixture.run(.scan(.pi), initialSearch: true)
        XCTAssertEqual(result.stopOutcome, .failed)
        let sent = fixture.commands.count
        let next = await fixture.run(.scan(.pi), token: 2, initialSearch: true)
        XCTAssertEqual(fixture.commands.count, sent)
        XCTAssertEqual(next.stopOutcome, .failed)
    }

    func testInitialSearchAuthorizationDoesNotLeakIntoAlignmentOrReacquisition() async {
        let fixture = DepthRecoveryFixture()
        fixture.sendFailure = URLError(.timedOut)
        _ = await fixture.run(.scan(.pi), initialSearch: true)
        XCTAssertEqual(fixture.commands.count, 1)
        XCTAssertEqual(fixture.depthReads, 0)
        fixture.depth = { command in .init(state: .stop, clearance: 0.1, supportCount: 5,
            sampleAge: 0, requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: nil) }
        for (index, request) in [FollowMotionRequest.alignment(0.5), .scan(0.5)].enumerated() {
            let result = await fixture.run(request, token: UInt64(index + 2), initialSearch: false)
            XCTAssertEqual(result.result, .failed(.obstacle))
            XCTAssertEqual(fixture.commands.count, 1)
        }
        XCTAssertGreaterThan(fixture.depthReads, 0)
    }

    func testUnownedSenderNeverGetsInitialSearchExceptionForAnyWheelShapeOrPurpose() async {
        for purpose in [FollowMotionPurpose.followScan, .followAlignment, .followReady, .followGoal] {
            for command in [WheelCommand(left: -0.25, right: 0.25), .init(left: 0.25, right: 0.20),
                            .init(left: -0.25, right: -0.25), .init(left: 0.25, right: 0.25),
                            .init(left: .nan, right: 0.25), .init(left: -.infinity, right: .infinity)] {
                let fixture = DepthRecoveryFixture()
                fixture.depth = { command in .unavailable(.malformedDepth, sampleAge: 0, motionClass: .classify(command)) }
                let receipt = await fixture.controller.sendFollowTurnBurst(command, requestedBudget: 0.080, purpose: purpose)
                XCTAssertNotNil(receipt.result.failure)
                XCTAssertTrue(fixture.commands.isEmpty)
                XCTAssertGreaterThan(fixture.depthReads, 0)
            }
        }
    }

    func testApprovedInitialSearchUsesOnlyPureRotationWithRealBlindDepth() async {
        let fixture = DepthRecoveryFixture()
        fixture.grant = FollowSearchClearanceConfirmation(sourceGeneration: 1, origin: .zero, confirmedAt: 10).consume(session: 1)
        fixture.depth = { command in
            DepthSafetyEvaluator.evaluate(Self.visibleFloor(timestamp: fixture.uptime, ordinary: true), command: command,
                now: fixture.uptime, operatorClearedRotation: FollowSearchRotationScope.authorized)
        }
        fixture.sendFailure = URLError(.timedOut)
        let result = await fixture.run(.scan(0.5), initialSearch: true)
        XCTAssertEqual(fixture.commands, [.init(left: -0.25, right: 0.25)])
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testExpiredGrantReportsClearanceExpiryInsteadOfCameraCoverage() async throws {
        let fixture = DepthRecoveryFixture()
        fixture.grant = FollowSearchClearanceConfirmation(sourceGeneration: 1, origin: .zero, confirmedAt: -51).consume(session: 1)
        fixture.depth = { command in .init(state: .clear, clearance: .infinity, supportCount: 0,
            sampleAge: 0.01, requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: nil) }
        let result = await fixture.run(.scan(0.5))
        let resolution = FollowMotionFailureResolution(try XCTUnwrap(result.failure))
        XCTAssertEqual(resolution.diagnosticReason, "search_clearance_expired")
        XCTAssertTrue(resolution.message.contains("expired"))
        XCTAssertFalse(resolution.message.contains("camera cannot see"))
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testClearanceExcursionAfterStoppedSettleReportsLatchedObservedBudget() async throws {
        let fixture = DepthRecoveryFixture()
        fixture.grant = FollowSearchClearanceConfirmation(sourceGeneration: 1, origin: .zero, confirmedAt: 10).consume(session: 1)
        fixture.pose = .init(position: .init(0.201, 0), yaw: 0)
        let result = await fixture.run(.scan(0.5))
        let failure = try XCTUnwrap(result.failure)
        let resolution = FollowMotionFailureResolution(failure)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(resolution.diagnosticReason, "search_clearance_space_exceeded")
        XCTAssertTrue(resolution.message.contains("observed movement"))
        XCTAssertEqual(failure.turnDiagnosticFields["search_clearance_observed_displacement_m"], .number(0.201))
        XCTAssertEqual(failure.turnDiagnosticFields["search_clearance_displacement_limit_m"], .number(0.20))
        XCTAssertNotNil(failure.turnDiagnosticFields["search_clearance_observed_frame"])
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testHealthyObservedShiftCanSendPureTurnWithinBudget() async {
        let fixture = DepthRecoveryFixture()
        fixture.grant = FollowSearchClearanceConfirmation(sourceGeneration: 1, origin: .zero, confirmedAt: 10).consume(session: 1)
        fixture.pose = .init(position: .init(0.20, 0), yaw: 0)
        fixture.depth = { command in
            DepthSafetyEvaluator.evaluate(Self.visibleFloor(timestamp: fixture.uptime, ordinary: true), command: command,
                now: fixture.uptime, operatorClearedRotation: FollowSearchRotationScope.authorized)
        }
        fixture.sendFailure = URLError(.timedOut)
        _ = await fixture.run(.scan(0.5))
        XCTAssertEqual(fixture.commands, [.init(left: -0.25, right: 0.25)])
    }

    func testConfirmedRotationCautionRechecksAndSendsExactScaledCommand() async {
        let fixture = DepthRecoveryFixture()
        fixture.grant = FollowSearchClearanceConfirmation(sourceGeneration: 1, origin: .zero, confirmedAt: 10).consume(session: 1)
        var reads: [WheelCommand] = []
        fixture.depth = { command in
            reads.append(command)
            let snapshot = DepthSafetySnapshot(timestamp: fixture.uptime,
                cells: Set((2...4).map { DepthSafetyCell(lateralIndex: 0, heightIndex: $0, forwardIndex: 18) }),
                cellResolution: 0.04, validSampleRatio: 1, geometry: RoverConfig.collisionGeometry, projection: nil, failureReason: nil)
            return DepthSafetyEvaluator.evaluate(snapshot, command: command, now: fixture.uptime,
                operatorClearedRotation: FollowSearchRotationScope.authorized)
        }
        fixture.sendFailure = URLError(.timedOut)
        _ = await fixture.run(.scan(0.5))
        XCTAssertTrue(reads.contains(.init(left: -0.25, right: 0.25)))
        XCTAssertTrue(reads.contains(.init(left: -0.12, right: 0.12)))
        XCTAssertEqual(fixture.commands, [.init(left: -0.12, right: 0.12)])
    }

    func testQueuedTypedExpiryReportsGrantDeadlineWithoutAnotherSourceCallback() async throws {
        try await assertQueuedTypedExpiry(grantRemaining: 0.040, queueDelay: 0.050,
            depthAge: 0, expectedCause: "search_clearance_expired")
    }

    func testQueuedTypedExpiryPreservesEarlierBurstBudgetCause() async throws {
        try await assertQueuedTypedExpiry(grantRemaining: 0.200, queueDelay: 0.090,
            depthAge: 0, expectedCause: "burst_pre_send_expired")
    }

    func testQueuedTypedExpiryPreservesEarlierDepthDeadlineCause() async throws {
        try await assertQueuedTypedExpiry(grantRemaining: 0.040, queueDelay: 0.050,
            depthAge: 0.230, expectedCause: "stale_raw_depth")
    }

    private func assertQueuedTypedExpiry(grantRemaining: Double, queueDelay: Double,
                                        depthAge: Double, expectedCause: String) async throws {
        let clock = DepthRecoveryTransportClock()
        var date = Date(timeIntervalSince1970: 1000)
        var snapshot = NavigationPoseSample(pose: .init(position: .zero, yaw: 0),
            frameID: .init(generation: 1, sequence: 1), sourceTimestamp: 9.999, trackingQuality: .normal)
        let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 1, origin: .zero,
            confirmedAt: 10.301 + grantRemaining - 60).consume(session: 1))
        var stops = 0
        var transports = 0
        var physicalSends = 0
        let controller = FollowNavigationController(currentPose: { snapshot.pose }, forwardClearance: { 5 },
            depthSafetyObservation: { command in
                .init(state: .clear, clearance: .infinity, supportCount: 0, sampleAge: depthAge,
                    requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: nil)
            }, plan: { _, goal in [goal] }, lastAckAt: { date }, sendCommand: { _ in physicalSends += 1 },
            stopRover: { stops += 1 }, sleep: { _ in await Task.yield() }, now: { date },
            sendCommandReceipt: { _ in
                transports += 1
                let authority = try! XCTUnwrap(FollowTurnBurstTransportScope.authorization)
                XCTAssertEqual(clock.value, 10.301, accuracy: 0.000001)
                XCTAssertEqual(authority.deadline, 10.301 + min(grantRemaining, 0.080, 0.250 - depthAge), accuracy: 0.000001)
                // Advance only the queued clock without a pose/depth callback.
                // Non-suspending return must reconcile the earliest authority;
                // later-expiring grants cannot replace a depth/burst cause.
                clock.value += queueDelay
                date = date.addingTimeInterval(queueDelay)
                XCTAssertFalse(authority.isAuthorized())
                return .init(receipt: .init(httpStatus: nil, acknowledged: false,
                    acknowledgementUTC: nil, attempts: 0, outcome: "expired"),
                    failure: FollowTurnBurstTransportDenial.expired)
            }, poseSample: { snapshot }, sourceNow: { clock.value }, sourceStopSnapshot: { snapshot },
            transportUptime: { clock.value })
        let task = Task { await controller.performFollowMotion(.scan(0.5), context:
            .init(sessionGeneration: 1, requestToken: 1, purpose: .followScan, phase: "searching",
                isSearchSweep: false, searchClearance: grant)) }
        for _ in 0..<2000 where stops == 0 { await Task.yield() }
        XCTAssertEqual(stops, 1)
        clock.value = 10.301
        snapshot = .init(pose: .init(position: .zero, yaw: 0), frameID: .init(generation: 1, sequence: 2),
            sourceTimestamp: 10.300, trackingQuality: .normal)
        controller.ingestFollowTurnSource(snapshot)
        let result = await task.value
        XCTAssertEqual(transports, 1)
        XCTAssertEqual(physicalSends, 0)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        let failure = try XCTUnwrap(result.failure)
        let resolution = FollowMotionFailureResolution(failure)
        XCTAssertEqual(resolution.diagnosticReason, expectedCause)
        if expectedCause == "search_clearance_expired" {
            XCTAssertTrue(resolution.message.contains("turning space expired"))
            XCTAssertFalse(resolution.message.contains("scheduling"))
            XCTAssertEqual(failure.turnDiagnosticFields["search_clearance_revoked_at_s"], .number(10.351))
            XCTAssertEqual(failure.turnDiagnosticFields["search_clearance_observed_displacement_m"], .number(0))
            XCTAssertEqual(failure.turnDiagnosticFields["search_clearance_observed_frame"], .string("1:2"))
        } else if expectedCause == "burst_pre_send_expired" {
            XCTAssertTrue(resolution.message.contains("scheduling"))
        } else {
            XCTAssertTrue(resolution.message.contains("Depth is stale"))
        }
    }

    func testApprovalExpiryWhileTransportQueuedCannotSend() async {
        let fixture = DepthRecoveryFixture()
        fixture.grant = FollowSearchClearanceConfirmation(sourceGeneration: 1, origin: .zero, confirmedAt: -49).consume(session: 1)
        fixture.depth = { command in
            DepthSafetyEvaluator.evaluate(Self.visibleFloor(timestamp: fixture.uptime, ordinary: true), command: command,
                now: fixture.uptime, operatorClearedRotation: FollowSearchRotationScope.authorized)
        }
        fixture.queuedTransportDelay = 2
        let result = await fixture.run(.scan(0.5))
        XCTAssertTrue(fixture.commands.isEmpty)
        XCTAssertNotEqual(result.result, .arrived)
    }

    func testQueuedApprovedTurnRejectsRevocationSourceExcursionAndNewVisibleHazard() async {
        for cause in 0..<3 {
            let fixture = DepthRecoveryFixture()
            fixture.grant = FollowSearchClearanceConfirmation(sourceGeneration: 1, origin: .zero, confirmedAt: 10).consume(session: 1)
            fixture.depth = { command in
                DepthSafetyEvaluator.evaluate(Self.visibleFloor(timestamp: fixture.uptime, ordinary: true), command: command,
                    now: fixture.uptime, operatorClearedRotation: FollowSearchRotationScope.authorized)
            }
            fixture.beforeTransportEntry = {
                switch cause {
                case 0: fixture.grant?.revoke()
                case 1:
                    fixture.pose = .init(position: .init(0.201, 0), yaw: 0)
                    fixture.controller.ingestFollowTurnSource(fixture.sample)
                    fixture.pose = .init(position: .zero, yaw: 0)
                default:
                    fixture.depth = { command in .init(state: .stop, clearance: 0.1, supportCount: 5,
                        sampleAge: 0.01, requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: nil) }
                    fixture.controller.ingestFollowTurnSource(fixture.sample)
                }
            }
            fixture.queuedTransportDelay = 0.001
            let result = await fixture.run(.scan(0.5))
            XCTAssertTrue(fixture.commands.isEmpty, "Queued invalidation must reach actual transport: \(cause)")
            XCTAssertNotEqual(result.result, .arrived)
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
    }

    func testAsyncDepthPublicationFencesQueuedTurnWithoutAnotherPoseEvent() async throws {
        try await assertAsyncDepthPublication(initialSearch: false)
    }

    func testAsyncDepthPublicationCannotFenceQueuedInitialSearch() async throws {
        try await assertAsyncDepthPublication(initialSearch: true)
    }

    private func assertAsyncDepthPublication(initialSearch: Bool) async throws {
        for scenario in 0..<5 {
            let hazard = Self.visibleFloor(timestamp: 10.301, ordinary: true, obstacle: true)
            let caution = DepthSafetySnapshot(timestamp: 10.301,
                cells: Set((2...4).map { DepthSafetyCell(lateralIndex: 0, heightIndex: $0, forwardIndex: 18) }),
                cellResolution: 0.04, validSampleRatio: 1, geometry: RoverConfig.collisionGeometry, projection: nil, failureReason: nil)
            let published = scenario == 4 ? caution : scenario == 1 ? DepthSafetySnapshot(timestamp: 10.301, cells: [], cellResolution: 0.04,
                validSampleRatio: 0, geometry: RoverConfig.collisionGeometry, projection: nil, failureReason: .malformedDepth) : hazard
            let ar = ARSessionManager(relativeHeadingTelemetry: { _, _ in }, depthProjection: { input in
                .init(timestamp: input.timestamp, cells: published.cells, cellResolution: published.cellResolution,
                    validSampleRatio: published.validSampleRatio, geometry: published.geometry,
                    projection: published.projection, failureReason: published.failureReason)
            })
            ar.resetTracking(generation: 1, runSession: false)
            let fixture = DepthRecoveryFixture(depthPublisher: ar)
            fixture.grant = FollowSearchClearanceConfirmation(sourceGeneration: 1, origin: .zero, confirmedAt: 10).consume(session: 1)
            fixture.depth = { command in
                DepthSafetyEvaluator.evaluate(Self.visibleFloor(timestamp: fixture.uptime, ordinary: true), command: command,
                    now: fixture.uptime, operatorClearedRotation: FollowSearchRotationScope.authorized)
            }
            var authorizedAfterPublication = true
            var publications = 0
            fixture.beforeTransportEntry = {
                fixture.holdSourceAdvancement = true
                defer { fixture.holdSourceAdvancement = false }
                var buffer: CVPixelBuffer?
                CVPixelBufferCreate(kCFAllocatorDefault, 2, 2, kCVPixelFormatType_DepthFloat32, nil, &buffer)
                let version = ar.depthSnapshotVersion
                let capturedSequence = fixture.sequence
                let capturedTime = fixture.uptime
                let authorization = try XCTUnwrap(FollowTurnBurstTransportScope.authorization)
                let originalDeadline = authorization.deadline
                if scenario == 2 { ar.pause() }
                else { ar.enqueueDepthSafety(rawDepthMap: buffer!, intrinsics: matrix_identity_float3x3,
                    cameraTransform: matrix_identity_float4x4, timestamp: capturedTime) }
                // The real asynchronous projector completion publishes directly to
                // AR. No source frame is ingested after the transport fence arms.
                if scenario != 2 {
                    for _ in 0..<20000 where ar.depthSnapshotVersion == version { await Task.yield() }
                    XCTAssertGreaterThan(ar.depthSnapshotVersion, version)
                    let observation = DepthSafetyEvaluator.evaluate(try XCTUnwrap(ar.latestDepthSafetySnapshot),
                        command: .init(left: -0.25, right: 0.25), now: capturedTime, operatorClearedRotation: true)
                    XCTAssertEqual(observation.state, scenario == 4 ? .caution : scenario == 1 ? .unavailable(.malformedDepth) : .stop)
                }
                if scenario == 3 {
                    // A later clear publication must not rescue the queued turn.
                    CVPixelBufferLockBaseAddress(buffer!, [])
                    let stride = CVPixelBufferGetBytesPerRow(buffer!) / MemoryLayout<Float>.size
                    let values = CVPixelBufferGetBaseAddress(buffer!)!.assumingMemoryBound(to: Float.self)
                    for row in 0..<2 { for column in 0..<2 { values[row * stride + column] = 3 } }
                    CVPixelBufferUnlockBaseAddress(buffer!, [])
                    ar.ingestDepthSafety(rawDepthMap: buffer!, intrinsics: matrix_identity_float3x3,
                        cameraTransform: matrix_identity_float4x4, timestamp: capturedTime)
                    XCTAssertEqual(DepthSafetyEvaluator.evaluate(try XCTUnwrap(ar.latestDepthSafetySnapshot),
                        command: .init(left: -0.25, right: 0.25), now: capturedTime, operatorClearedRotation: true).state, .clear)
                }
                publications += 1
                XCTAssertEqual(fixture.sequence, capturedSequence)
                XCTAssertEqual(fixture.uptime, capturedTime, "Do not accidentally prove expiry instead of publication fencing")
                XCTAssertEqual(authorization.deadline, originalDeadline)
                authorizedAfterPublication = authorization.isAuthorized()
                if !authorizedAfterPublication { throw FollowTurnBurstTransportDenial.fenced }
                fixture.sendFailure = URLError(.timedOut)
            }
            let result = await fixture.run(.scan(0.5), initialSearch: initialSearch)
            if scenario == 4 && !initialSearch {
                XCTAssertEqual(FollowMotionFailureResolution(try XCTUnwrap(result.failure)).diagnosticReason, "obstacle")
            }
            XCTAssertEqual(publications, 1)
            XCTAssertEqual(authorizedAfterPublication, initialSearch)
            XCTAssertEqual(fixture.commands, initialSearch ? [.init(left: -0.25, right: 0.25)] : [])
            if initialSearch { XCTAssertEqual(fixture.depthReads, 0) }
        }
    }

    func testOrdinaryCameraKeepsBlindArcStopped() async {
        let fixture = DepthRecoveryFixture()
        fixture.depth = { command in
            DepthSafetyEvaluator.evaluate(Self.visibleFloor(timestamp: fixture.uptime, ordinary: true), command: command, now: fixture.uptime)
        }
        let result = await fixture.run(.scan(.pi))
        XCTAssertEqual(result.result, .failed(.depthUnavailable(.blindSweptVolume)))
        XCTAssertTrue(fixture.commands.isEmpty)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testActualObstacleNeverAttemptsArc() async {
        let fixture = DepthRecoveryFixture()
        var arcs = 0
        fixture.depth = { command in
            if command.left > 0 && command.right > 0 { arcs += 1 }
            return .init(state: .stop, clearance: 0.10, supportCount: 12, sampleAge: 0.02,
                requiredStoppingDistance: 0.4, motionClass: .classify(command), speedLimit: nil)
        }
        let result = await fixture.run(.scan(0.5))
        XCTAssertEqual(result.result, .failed(.obstacle))
        XCTAssertEqual(arcs, 0)
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testMissingAndStaleDepthRecoverOnlyWhileStopped() async {
        for reason: DepthSafetyUnavailableReason in [.missingRawDepth, .staleRawDepth] {
            let fixture = DepthRecoveryFixture()
            var reads = 0
            fixture.depth = { command in
                reads += 1
                if reads < 3 {
                    XCTAssertTrue(fixture.commands.isEmpty)
                    XCTAssertGreaterThan(fixture.stops, 0)
                    return .unavailable(reason, sampleAge: 1, motionClass: .classify(command))
                }
                return .init(state: .clear, clearance: 5, supportCount: 20, sampleAge: 0.01,
                    requiredStoppingDistance: 0.4, motionClass: .classify(command), speedLimit: nil)
            }
            fixture.sendFailure = URLError(.timedOut)
            let result = await fixture.run(.alignment(0.5))
            XCTAssertEqual(fixture.commands, [.init(left: -0.25, right: 0.25)])
            XCTAssertEqual(result.result, .failed(.commandFailed))
        }
    }

    func testMissingDepthWaitTerminatesWithoutRenewingObservation() async {
        let fixture = DepthRecoveryFixture()
        var reads = 0
        fixture.depth = { command in
            reads += 1
            return .unavailable(.missingRawDepth, sampleAge: .infinity, motionClass: .classify(command))
        }
        let result = await fixture.run(.scan(0.5))
        XCTAssertEqual(result.result, .failed(.depthUnavailable(.missingRawDepth)))
        XCTAssertGreaterThan(reads, 1)
        XCTAssertLessThanOrEqual(reads, 14)
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testRecoveryBudgetSurvivesNewOperationsAndResetsOnlyForNewSession() async {
        let fixture = DepthRecoveryFixture()
        fixture.depth = Self.blindSpinClearArc
        let first = await fixture.run(.scan(0.5), token: 1, initialSearch: false)
        let second = await fixture.run(.alignment(0.5), token: 2)
        let third = await fixture.run(.scan(0.5), token: 3, initialSearch: false)
        XCTAssertEqual(first.result, .reobservationRequired)
        XCTAssertEqual(second.result, .reobservationRequired)
        XCTAssertEqual(third.context.failureCause, .depthRecoveryExhausted)
        XCTAssertEqual(fixture.commands.count, 2)
        let restarted = await fixture.run(.scan(0.5), token: 1, session: 2, initialSearch: false)
        XCTAssertEqual(restarted.result, .reobservationRequired)
        XCTAssertEqual(fixture.commands.count, 3)
    }

    func testScaledArcMustPassExactCommandDepthAgain() async {
        let fixture = DepthRecoveryFixture()
        var scaledReads = 0
        fixture.depth = { command in
            if command.left < 0 { return .unavailable(.blindSweptVolume, sampleAge: 0, motionClass: .rotating) }
            if command.right <= 0.1 {
                scaledReads += 1
                return .unavailable(.blindSweptVolume, sampleAge: 0, motionClass: .curved)
            }
            return .init(state: .caution, clearance: 0.6, supportCount: 5, sampleAge: 0.01,
                requiredStoppingDistance: 0.4, motionClass: .curved, speedLimit: 0.1)
        }
        let result = await fixture.run(.scan(0.5), initialSearch: false)
        XCTAssertEqual(result.result, .failed(.depthUnavailable(.blindSweptVolume)))
        XCTAssertGreaterThan(scaledReads, 0)
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testDepthProviderReplacementCannotSendArc() async {
        let fixture = DepthRecoveryFixture()
        fixture.depth = { command in
            if command.left > 0 { fixture.controller.inhibitForHandoff() }
            return Self.blindSpinClearArc(command)
        }
        let result = await fixture.run(.scan(0.5), initialSearch: false)
        XCTAssertEqual(result.result, .cancelled)
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testCautionArcSendsOnlyExactAuthorizedLimitedWheels() async {
        let fixture = DepthRecoveryFixture()
        var checked: [WheelCommand] = []
        fixture.depth = { command in
            checked.append(command)
            if command.left < 0 { return .unavailable(.blindSweptVolume, sampleAge: 0, motionClass: .rotating) }
            return .init(state: .caution, clearance: 0.6, supportCount: 5, sampleAge: 0.01,
                requiredStoppingDistance: 0.4, motionClass: .curved, speedLimit: 0.1)
        }
        let result = await fixture.run(.alignment(0.5))
        XCTAssertEqual(result.result, .reobservationRequired)
        XCTAssertEqual(fixture.commands.count, 1)
        if let command = fixture.commands.first {
            XCTAssertEqual(command.left, 0.08, accuracy: 1e-12)
            XCTAssertEqual(command.right, 0.1, accuracy: 1e-12)
            XCTAssertGreaterThanOrEqual(checked.filter { $0 == command }.count, 2,
                "Scaled candidate must be checked during selection and at actual sender entry")
        }
    }

    func testFailedArcStopBlocksSubsequentMotion() async {
        let fixture = DepthRecoveryFixture()
        fixture.depth = Self.blindSpinClearArc
        fixture.failStopAfterMovement = true
        let first = await fixture.run(.scan(0.5), initialSearch: false)
        XCTAssertEqual(first.result, .failed(.commandFailed))
        XCTAssertEqual(first.stopOutcome, .failed)
        let second = await fixture.run(.scan(0.5), token: 2, initialSearch: false)
        XCTAssertEqual(second.result, .failed(.commandFailed))
        XCTAssertEqual(fixture.commands.count, 1)
    }

    func testObservedArcExcursionFailsAndNeverClaimsArrival() async {
        let fixture = DepthRecoveryFixture()
        fixture.depth = Self.blindSpinClearArc
        fixture.sentPose = .init(position: .init(0.2, 0), yaw: 0.5)
        let result = await fixture.run(.alignment(0.5))
        XCTAssertEqual(result.result, .failed(.depthUnavailable(.blindSweptVolume)))
        XCTAssertEqual(result.context.failureCause, .depthRecoveryExcursion)
        XCTAssertNil(result.measuredScanResponse)
        XCTAssertEqual(fixture.commands.count, 1)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testCancellationDuringStoppedDepthWaitSendsNoMovement() async {
        let fixture = DepthRecoveryFixture()
        var retryStarted = false
        var depthQueried = false
        fixture.depth = { command in
            depthQueried = true
            return .unavailable(.staleRawDepth, sampleAge: 1, motionClass: .classify(command))
        }
        fixture.onSleep = {
            if depthQueried { retryStarted = true; fixture.controller.inhibitForHandoff() }
        }
        let result = await fixture.run(.alignment(0.5))
        XCTAssertTrue(retryStarted)
        XCTAssertEqual(result.result, .cancelled)
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testActualDepthObstacleRejectsRecoveryWithoutSending() async {
        let fixture = DepthRecoveryFixture()
        fixture.depth = { command in
            DepthSafetyEvaluator.evaluate(Self.visibleFloor(timestamp: fixture.uptime, obstacle: true), command: command, now: fixture.uptime)
        }
        XCTAssertEqual(fixture.depth(.init(left: -0.25, right: 0.25)).state, .stop)
        let result = await fixture.run(.scan(0.5))
        XCTAssertEqual(result.result, .failed(.obstacle))
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testFinalArcDepthProviderCannotReplaceOwnerThenSend() async {
        let fixture = DepthRecoveryFixture()
        var arcReads = 0
        fixture.depth = { command in
            if command.left > 0 && command.right > 0 {
                arcReads += 1
                if arcReads == 2 { fixture.controller.inhibitForHandoff() }
            }
            return Self.blindSpinClearArc(command)
        }
        let result = await fixture.run(.scan(0.5), initialSearch: false)
        XCTAssertEqual(arcReads, 2)
        XCTAssertEqual(result.result, .cancelled)
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testTransportCannotReuseDepthPastItsOriginalFreshnessDeadline() async throws {
        for policy: FollowDepthPolicy in [.fullVolume, .astralCompatible] {
            for delayProvider in [false, true] {
                let time = DepthRecoveryTransportClock()
                let sink = FollowDiagnosticRecordingSink()
                var transportCalls = 0
                let controller = FollowNavigationController(currentPose: { .init(position: .zero, yaw: 0) },
                    forwardClearance: { 5 }, depthSafetyObservation: { command in
                        if delayProvider { time.value += 0.02 }
                        return .init(state: .clear, clearance: 5, supportCount: 5, sampleAge: 0.24,
                            requiredStoppingDistance: 0.4, motionClass: .classify(command), speedLimit: nil,
                            visibilityPolicy: policy)
                    }, plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in XCTFail("No unchecked sender") },
                    stopRover: {}, sleep: { _ in }, sendCommandReceipt: { _ in
                        transportCalls += 1
                        time.value += 0.02
                        XCTAssertFalse(FollowTurnBurstTransportScope.authorization!.isAuthorized(),
                            "A fresh pose cannot renew the depth snapshot used for motor authorization")
                        return .init(receipt: .init(httpStatus: nil, acknowledged: false,
                            acknowledgementUTC: nil, attempts: 0, outcome: "expired"),
                            failure: FollowTurnBurstTransportDenial.expired)
                    }, diagnosticEmitter: .init(streamID: "depth-expiry-policy", monotonic: { time.value }, utc: { Date() }, sink: sink.append),
                    sourceNow: { time.value }, transportUptime: { time.value }, depthPolicy: policy)
                let receipt = await controller.sendFollowTurnBurst(.init(left: -0.25, right: 0.25), requestedBudget: 0.080, purpose: .followScan)
                XCTAssertEqual(receipt.result.receipt.outcome, "depth_safety_stop")
                XCTAssertEqual(transportCalls, delayProvider ? 0 : 1)
                let records = try sink.records.compactMap { record -> [String: Any]? in
                    guard record.event == "follow_depth.candidate", let text = record.fields["payload"] else { return nil }
                    return try JSONSerialization.jsonObject(with: Data(text.utf8)) as? [String: Any]
                }
                let expired = try XCTUnwrap(records.first {
                    $0["depth_candidate_disposition"] as? String == (delayProvider ? "expired_before_send" : "expired_at_transport")
                })
                XCTAssertEqual(expired["depth_visibility_policy"] as? String, policy.rawValue)
                XCTAssertEqual(expired["depth_state"] as? String, "stale_raw_depth")
                XCTAssertTrue(expired["depth_full_volume_would_block"] is NSNull)
            }
        }
    }

    func testDepthExpiryShortensActiveBurstAndConfirmsStop() async throws {
        let time = DepthRecoveryTransportClock()
        var budget = 0.0
        var stops = 0
        let controller = FollowNavigationController(currentPose: { .init(position: .zero, yaw: 0) },
            forwardClearance: { 5 }, depthSafetyObservation: { command in
                .init(state: .clear, clearance: 5, supportCount: 5, sampleAge: 0.24,
                    requiredStoppingDistance: 0.4, motionClass: .classify(command), speedLimit: nil)
            }, plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in
                let authority = FollowTurnBurstTransportScope.authorization!
                budget = authority.deadline - authority.sendEntryUptime
                time.value += 0.081
            }, stopRover: { stops += 1 }, sleep: { _ in }, sourceNow: { time.value }, transportUptime: { time.value })
        let receipt = try await controller.executeFollowTurnBurst(.init(left: -0.25, right: 0.25),
            requestedBudget: 0.080, purpose: .followScan)
        XCTAssertEqual(budget, 0.01, accuracy: 1e-9)
        XCTAssertTrue(receipt.send.stopObligation)
        XCTAssertEqual(receipt.send.deadline, 10.01, accuracy: 1e-9)
        XCTAssertEqual(stops, 1)
        XCTAssertNotNil(receipt.confirmedStopFence)
    }

    func testFirstArcQueueCannotOutliveItsAcknowledgement() async {
        let fixture = DepthRecoveryFixture()
        fixture.depth = Self.blindSpinClearArc
        fixture.ackProvider = { fixture.wallTime.addingTimeInterval(-1.99) }
        fixture.queuedTransportDelay = 0.02
        let result = await fixture.run(.alignment(0.5))
        XCTAssertEqual(fixture.queuedAuthorization, false,
            "An ACK with 10ms remaining cannot authorize a queued first arc 20ms later")
        XCTAssertTrue(fixture.commands.isEmpty)
        XCTAssertNotEqual(result.result, .reobservationRequired)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testFirstArcRejectsMissingAndFutureAcknowledgements() async {
        for future in [false, true] {
            let fixture = DepthRecoveryFixture()
            fixture.depth = Self.blindSpinClearArc
            fixture.ackProvider = { future ? fixture.wallTime.addingTimeInterval(1) : nil }
            let result = await fixture.run(.alignment(0.5))
            XCTAssertEqual(result.result, .failed(.commsLost))
            XCTAssertTrue(fixture.commands.isEmpty)
        }
    }

    func testFirstArcMonitorRejectsMissingOrFutureAcknowledgementWhileQueued() async {
        for future in [false, true] {
            let fixture = DepthRecoveryFixture()
            fixture.depth = Self.blindSpinClearArc
            var replaceAck = false
            fixture.ackProvider = {
                replaceAck ? (future ? fixture.wallTime.addingTimeInterval(1) : nil) : fixture.wallTime
            }
            fixture.beforeTransportEntry = {
                replaceAck = true
                for _ in 0..<2000 where fixture.controller.followTurnBurstPendingStatus?.stopObligation != true {
                    await Task.yield()
                }
                XCTAssertEqual(fixture.controller.followTurnBurstPendingStatus?.stopObligation, true)
                XCTAssertFalse(FollowTurnBurstTransportScope.authorization!.isAuthorized())
                throw FollowTurnBurstTransportDenial.fenced
            }
            let result = await fixture.run(.alignment(0.5))
            XCTAssertEqual(result.result, .failed(.commsLost))
            XCTAssertTrue(fixture.commands.isEmpty)
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
    }

    func testDepthCleanupFailureLatchesLegacyAndPursuitUntilExplicitConfirmedRecovery() async throws {
        for legacy in [true, false] {
            var sends = 0
            var depthBlocked = true
            var failStops = true
            let controller = FollowNavigationController(currentPose: { .init(position: .zero, yaw: 0) },
                forwardClearance: { 5 }, depthSafetyObservation: { command in
                    if depthBlocked, sends > 0 {
                        return .unavailable(.invalidCalibration, sampleAge: 0.01, motionClass: .classify(command))
                    }
                    return .init(state: .clear, clearance: 5, supportCount: 5, sampleAge: 0.01,
                        requiredStoppingDistance: 0.4, motionClass: .classify(command), speedLimit: nil)
                }, plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in
                    sends += 1
                    if sends > 1 { throw URLError(.timedOut) }
                }, stopRover: {
                    if sends > 0, failStops { throw URLError(.timedOut) }
                }, sleep: { _ in })
            let result: NavigationResult
            if legacy { result = await controller.rotateAndWait(by: 0.5) }
            else { result = await controller.performFollowMotion(.following(.init(4, 0), 1.25), context: nil).result }
            XCTAssertEqual(result, .failed(.commandFailed))
            XCTAssertTrue(controller.hasUnconfirmedStop, "\(legacy)")
            XCTAssertEqual(controller.safetyState, .failed(.commandFailed), "\(legacy)")
            if case .failed = controller.state {} else { XCTFail("Failed stop must publish terminal state") }
            XCTAssertEqual(sends, 1)
            failStops = false
            depthBlocked = false
            let blocked = await controller.performFollowMotion(.following(.init(4, 0), 1.25), context: nil)
            XCTAssertEqual(blocked.result, .failed(.commandFailed))
            XCTAssertEqual(sends, 1, "A later request must not recover the failed stop implicitly")
            try await controller.stopAndConfirm()
            XCTAssertFalse(controller.hasUnconfirmedStop)
            _ = await controller.performFollowMotion(.following(.init(4, 0), 1.25), context: nil)
            XCTAssertEqual(sends, 2, "Explicit confirmed recovery can admit motion again")
        }
    }

    private static func blindSpinClearArc(_ command: WheelCommand) -> DepthSafetyObservation {
        if command.left < 0 || command.right < 0 {
            return .unavailable(.blindSweptVolume, sampleAge: 0.01, motionClass: .rotating)
        }
        return .init(state: .clear, clearance: 5, supportCount: 20, sampleAge: 0.01,
            requiredStoppingDistance: 0.4, motionClass: .classify(command), speedLimit: nil)
    }

    // The wide synthetic frustum exercises authorization, not device FOV capability.
    private static func visibleFloor(timestamp: Double, ordinary: Bool = false, obstacle: Bool = false) -> DepthSafetySnapshot {
        var buffer: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 80, 60, kCVPixelFormatType_DepthFloat32, nil, &buffer)
        let map = buffer!
        CVPixelBufferLockBaseAddress(map, [])
        let stride = CVPixelBufferGetBytesPerRow(map) / MemoryLayout<Float>.size
        let base = CVPixelBufferGetBaseAddress(map)!.assumingMemoryBound(to: Float.self)
        for row in 0..<60 {
            for column in 0..<80 { base[row * stride + column] = row > 30 ? 0.30 * (ordinary ? 40 : 8) / Float(row - 30) : 3 }
        }
        if obstacle {
            for row in 31...37 { for column in 35...44 { base[row * stride + column] = 0.20 } }
        }
        CVPixelBufferUnlockBaseAddress(map, [])
        var transform = matrix_identity_float4x4
        transform.columns.3 = SIMD4<Float>(0, 0.30, 0, 1)
        return DepthSafetyEvaluator.ingest(rawDepthMap: map,
            intrinsics: .init(columns: (SIMD3<Float>(ordinary ? 50 : 20, 0, 0), SIMD3<Float>(0, ordinary ? 40 : 8, 0), SIMD3<Float>(40, 30, 1))),
            cameraTransform: transform, timestamp: timestamp, calibration: RoverConfig.cameraMountCalibration,
            geometry: .init(length: 0.30, width: 0.25, minimumCollisionHeight: 0.04, maximumCollisionHeight: 0.50))
    }
}

@MainActor
private final class DepthRecoveryFixture {
    private let transportClock = DepthRecoveryTransportClock()
    var uptime: Double {
        get { transportClock.value }
        set { transportClock.value = newValue }
    }
    var wallTime = Date(timeIntervalSince1970: 1000)
    var ackProvider: (() -> Date?)?
    var queuedTransportDelay = 0.0
    var queuedAuthorization: Bool?
    var beforeTransportEntry: (() async throws -> Void)?
    var sequence: UInt64 = 1
    var pose = Pose2D(position: .zero, yaw: 0)
    var commands: [WheelCommand] = []
    var stops = 0
    var depthReads = 0
    var grant: FollowSearchClearanceGrant?
    var sendFailure: Error?
    var failStopAfterMovement = false
    var sentPose = Pose2D(position: .init(0.004, 0), yaw: 0.01)
    var onSleep: (() -> Void)?
    var holdSourceAdvancement = false
    var depth: (WheelCommand) -> DepthSafetyObservation = { command in
        .unavailable(.blindSweptVolume, sampleAge: 0, motionClass: .classify(command))
    }
    var sample: NavigationPoseSample {
        .init(pose: pose, frameID: .init(generation: 1, sequence: sequence), sourceTimestamp: uptime - 0.001, trackingQuality: .normal)
    }
    var controller: FollowNavigationController!
    init(depthPublisher: ARSessionManager? = nil) {
        let transportClock = self.transportClock
        controller = FollowNavigationController(currentPose: { self.pose }, forwardClearance: { 5 },
        depthSafetyObservation: { self.depthReads += 1; return self.depth($0) },
        observeDepthSafety: depthPublisher.map { ar in { ar.observeDepthSafety($0) } }, plan: { _, goal in [goal] }, lastAckAt: {
            if let provider = self.ackProvider { return provider() }
            return self.wallTime
        },
        sendCommand: { @MainActor command in
            try await self.beforeTransportEntry?()
            if self.queuedTransportDelay > 0 {
                self.uptime += self.queuedTransportDelay
                self.wallTime = self.wallTime.addingTimeInterval(self.queuedTransportDelay)
                self.queuedAuthorization = FollowTurnBurstTransportScope.authorization!.isAuthorized()
                if self.queuedAuthorization == false { throw FollowTurnBurstTransportDenial.fenced }
            }
            self.commands.append(command)
            if let error = self.sendFailure { throw error }
            self.uptime += 0.081
            self.sequence += 1
            self.pose = self.sentPose
            self.controller.ingestFollowTurnSource(self.sample)
        }, stopRover: { @MainActor in
            self.stops += 1
            if self.failStopAfterMovement, !self.commands.isEmpty { throw URLError(.timedOut) }
        }, sleep: { @MainActor _ in self.onSleep?(); await Task.yield() }, now: { self.wallTime },
        poseSample: { self.sample }, sourceNow: { self.uptime }, sourceStopSnapshot: { self.sample },
        transportUptime: { transportClock.value })
    }

    func run(_ request: FollowMotionRequest, token: UInt64 = 1, session: UInt64 = 1, initialSearch: Bool = false,
             acquiring: Bool = false, recovery: FollowRecoveryHeadingRequest? = nil) async -> FollowMotionResult {
        var completed = false
        let previousStops = stops
        let task = Task {
            let context = FollowMotionRequestContext(sessionGeneration: session, requestToken: token, purpose: request.purpose,
                    phase: request.purpose == .followAlignment ? "aligning" : (initialSearch ? "searching" : "reacquiring"),
                    isSearchSweep: initialSearch && request.purpose == .followScan, searchClearance: self.grant,
                    allowsAcquisitionRotationWithoutDepth: initialSearch || acquiring)
            let result = await FollowRecoveryScope.$authorization.withValue(recovery?.authorization) {
                await FollowRecoveryScope.$heading.withValue(recovery) {
                    await self.controller.performFollowMotion(request, context: context)
                }
            }
            completed = true
            return result
        }
        for _ in 0..<2000 where stops == previousStops && !completed { await Task.yield() }
        // Advance actual capture identity/time; never renew an old depth or pose timestamp.
        for _ in 0..<50 where !completed {
            while holdSourceAdvancement && !completed { await Task.yield() }
            uptime += 0.301
            sequence += 1
            controller.ingestFollowTurnSource(sample)
            for _ in 0..<100 { await Task.yield() }
        }
        if !completed { task.cancel() }
        return await task.value
    }
}


private final class DepthRecoveryTransportClock: @unchecked Sendable {
    private let lock = NSLock()
    private var storage = 10.0
    var value: Double {
        get { lock.withLock { storage } }
        set { lock.withLock { storage = newValue } }
    }
}
