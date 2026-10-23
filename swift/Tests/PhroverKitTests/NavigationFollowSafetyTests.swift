import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class NavigationFollowSafetyTests: XCTestCase {
    func testFollowPursuitAndReadyRejectUnavailableDepthAndLimitCaution() async {
        for request: FollowMotionRequest in [.following(Vec2(4, 0), 1.25), .ready] {
            for state: DepthSafetyState in [.unavailable(.missingRawDepth), .unavailable(.staleRawDepth), .caution] {
                var commands: [WheelCommand] = []
                var stops = 0
                let nav = NavigationController(currentPose: { .init(position: .zero, yaw: 0) },
                    forwardClearance: { 5 }, depthSafetyObservation: { command in
                        .init(state: state, clearance: 5, supportCount: 10, sampleAge: 0,
                            requiredStoppingDistance: 0.4, motionClass: .classify(command), speedLimit: 0.05)
                    }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
                    sendCommand: { commands.append($0); throw URLError(.timedOut) },
                    stopRover: { stops += 1 }, sleep: { _ in })
                let result = await nav.performFollowMotion(request, context: nil)
                if state == .caution {
                    XCTAssertFalse(commands.isEmpty, "\(request)")
                    XCTAssertTrue(commands.allSatisfy { max(abs($0.left), abs($0.right)) <= 0.05 }, "\(request): \(commands)")
                } else {
                    XCTAssertTrue(commands.isEmpty, "\(request): \(state)")
                    XCTAssertEqual(result.result, .failed(.obstacle), "\(request): \(state)")
                    XCTAssertEqual(nav.safetyState, .failed(.obstacle))
                }
                XCTAssertGreaterThanOrEqual(stops, 2)
            }
        }
    }

    func testSharedBurstAndLegacyRotationApplyDepthSafetyAtSender() async {
        for purpose: FollowMotionPurpose in [.followAlignment, .followScan, .followReady] {
            for state: DepthSafetyState in [.unavailable(.missingRawDepth), .unavailable(.staleRawDepth), .caution] {
                var commands: [WheelCommand] = []
                let nav = FollowNavigationController(currentPose: { .init(position: .zero, yaw: 0) },
                    forwardClearance: { 5 }, depthSafetyObservation: { command in
                        .init(state: state, clearance: 5, supportCount: 10, sampleAge: 0,
                            requiredStoppingDistance: 0.4, motionClass: .classify(command), speedLimit: 0.05)
                    }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
                    sendCommand: { commands.append($0); throw URLError(.timedOut) },
                    stopRover: {}, sleep: { _ in })
                let burst = await nav.sendFollowTurnBurst(.init(left: -0.25, right: 0.25), requestedBudget: 0.025, purpose: purpose)
                if state == .caution {
                    XCTAssertEqual(commands, [.init(left: -0.05, right: 0.05)])
                } else {
                    XCTAssertTrue(commands.isEmpty)
                    XCTAssertEqual(burst.result.receipt.outcome, "depth_safety_stop")
                }
                commands.removeAll()
                let rotation = await nav.rotateAndWait(by: 0.5)
                if state == .caution {
                    XCTAssertEqual(commands, [.init(left: -0.05, right: 0.05)])
                } else {
                    XCTAssertTrue(commands.isEmpty)
                    XCTAssertEqual(rotation, .failed(.obstacle))
                }
            }
        }
    }

    func testEnrichedAlignmentAndScanPreserveTypedDepthDenialThroughPublicController() async {
        for request: FollowMotionRequest in [.alignment(0.5), .scan(0.5)] {
            var uptime = 10.0
            var sample = NavigationPoseSample(pose: .init(position: .zero, yaw: 0),
                frameID: .init(generation: 1, sequence: 1), sourceTimestamp: 9.99, trackingQuality: .normal)
            let stopSnapshot = expectation(description: "admission stop captured")
            var captured = false
            var sends = 0
            let nav = NavigationController(currentPose: { sample.pose }, forwardClearance: { 5 },
                depthSafetyObservation: { command in .unavailable(.missingRawDepth, sampleAge: .infinity, motionClass: .classify(command)) },
                plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in sends += 1 },
                stopRover: {}, sleep: { _ in await Task.yield() }, poseSample: { sample }, sourceNow: { uptime },
                sourceStopSnapshot: {
                    if !captured { captured = true; stopSnapshot.fulfill() }
                    return sample
                })
            let motion = Task { await nav.performFollowMotion(request, context: nil) }
            await fulfillment(of: [stopSnapshot], timeout: 2)
            uptime = 10.301
            sample = .init(pose: .init(position: .zero, yaw: 0), frameID: .init(generation: 1, sequence: 2),
                sourceTimestamp: 10.3, trackingQuality: .normal)
            nav.ingestFollowTurnSource(sample)
            let result = await motion.value
            XCTAssertEqual(result.result, .failed(.obstacle))
            XCTAssertEqual(nav.safetyState, .failed(.obstacle))
            XCTAssertEqual(sends, 0)
        }
    }

    func testFollowTypedFailureMirroringNeverPublishesParsedDisplayReason() {
        let nav = NavigationController(currentPose: { nil }, forwardClearance: { 5 },
            plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in }, stopRover: {}, sleep: { _ in })
        for failure: NavigationFailure in [.pathRejected(.outsideSector(pointIndex: 3)), .cancelled] {
            nav.mirrorFollowSafetyState(.moving)
            nav.mirrorFollowState(.failed(FollowNavigationController.message(for: failure)), path: [])
            XCTAssertEqual(nav.safetyState, .moving, "Display strings must not emit an intermediate guessed failure")
            nav.mirrorFollowSafetyState(.failed(failure))
            XCTAssertEqual(nav.safetyState, .failed(failure))
        }
    }

    func testGenericStopUsesInjectedTransportBeforeAnyFollowOperation() async {
        var stops = 0
        let nav = NavigationController(currentPose: { nil }, forwardClearance: { 5 },
            plan: { _, _ in nil }, lastAckAt: { nil }, sendCommand: { _ in },
            stopRover: { stops += 1 }, sleep: { _ in })
        let result = await nav.rotateAndWait(by: 0.5)
        XCTAssertEqual(result, .failed(.noPose))
        XCTAssertGreaterThanOrEqual(stops, 1)
    }

    func testBurstDepthProviderCannotSendAfterSynchronouslyReplacingOwner() async {
        var sends = 0
        var nav: FollowNavigationController!
        nav = FollowNavigationController(currentPose: { .init(position: .zero, yaw: 0) },
            forwardClearance: { 5 }, depthSafetyObservation: { command in
                nav.inhibitForHandoff()
                return .init(state: .clear, clearance: 5, supportCount: 1, sampleAge: 0,
                    requiredStoppingDistance: 0, motionClass: .classify(command), speedLimit: nil)
            }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in sends += 1 }, stopRover: {}, sleep: { _ in })
        let result = await nav.sendFollowTurnBurst(.init(left: -0.25, right: 0.25), requestedBudget: 0.025,
            purpose: .followAlignment)
        XCTAssertEqual(sends, 0)
        XCTAssertEqual(result.result.receipt.outcome, "fenced")
        XCTAssertNil(nav.followTurnBurstPendingStatus)
    }

    func testPublicConfirmedStopKeepsMovingStateUntilTransportAcknowledges() async {
        let sent = expectation(description: "moving")
        let stopping = expectation(description: "terminal stop")
        var release: CheckedContinuation<Void, Never>?
        var stops = 0
        var nav: NavigationController!
        nav = NavigationController(currentPose: { .init(position: .zero, yaw: 0) },
            forwardClearance: { 5 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in sent.fulfill(); await withCheckedContinuation { release = $0 } },
            stopRover: {
                stops += 1
                if stops == 2 { await withCheckedContinuation { release = $0; stopping.fulfill() } }
            }, sleep: { _ in })
        let drive = Task { await nav.navigateForFollow(to: Vec2(4, 0), stoppingAtForwardClearance: 1.25) }
        await fulfillment(of: [sent], timeout: 2)
        let stop = Task { try await nav.stopAndConfirm() }
        release?.resume(); release = nil
        await fulfillment(of: [stopping], timeout: 2)
        XCTAssertEqual(nav.safetyState, .moving)
        release?.resume(); release = nil
        _ = try? await stop.value
        _ = await drive.value
        XCTAssertEqual(nav.safetyState, .idle)
    }

    func testStopAndConfirmCancelsSuspendedURLSendWithoutPublishingFailure() async throws {
        try await assertIntentionalSendCancellation(error: URLError(.cancelled))
    }

    func testStopAndConfirmCancelsSuspendedTaskSendWithoutPublishingFailure() async throws {
        try await assertIntentionalSendCancellation(error: CancellationError())
    }

    private func assertIntentionalSendCancellation(error: Error) async throws {
        let send = SuspendedNavigationSend(error: error)
        var stops = 0
        let controller = NavigationController(
            currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 },
            plan: { _, goal in [goal] },
            lastAckAt: { nil },
            sendCommand: { _ in try await send.send() },
            stopRover: { stops += 1 },
            sleep: { _ in }
        )
        let states = controller.safetyStates()
        let received = Task { () -> [NavigationSafetyState] in
            var result: [NavigationSafetyState] = []
            for await state in states {
                result.append(state)
                if result.contains(.moving), state == .idle { break }
            }
            return result
        }
        let navigation = Task { await controller.navigateForFollow(to: Vec2(2, 0), stoppingAtForwardClearance: 1.25) }
        await send.waitUntilRequested()

        try await controller.stopAndConfirm()

        let result = await navigation.value
        let safetyStates = await received.value
        XCTAssertEqual(result, .cancelled)
        XCTAssertEqual(safetyStates, [.idle, .moving, .idle])
        XCTAssertEqual(controller.state, .idle)
        XCTAssertGreaterThanOrEqual(stops, 2, "Cancellation still requires an acknowledged motor stop")
    }

    func testUnrequestedURLSendCancellationRemainsCommandFailure() async {
        let controller = NavigationController(
            currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 },
            plan: { _, goal in [goal] },
            lastAckAt: { nil },
            sendCommand: { _ in throw URLError(.cancelled) },
            stopRover: {},
            sleep: { _ in }
        )

        let result = await controller.navigateForFollow(to: Vec2(2, 0), stoppingAtForwardClearance: 1.25)

        XCTAssertEqual(result, .failed(.commandFailed))
        XCTAssertEqual(controller.safetyState, .failed(.commandFailed))
    }

    func testIntentionalSendCancellationDoesNotHideUnconfirmedMotorStop() async {
        for error: Error in [FakeCommandError.timedOut, URLError(.cancelled)] {
            let send = SuspendedNavigationSend(error: URLError(.cancelled))
            var stops = 0
            var plans = 0
            let controller = NavigationController(
                currentPose: { Pose2D(position: .zero, yaw: 0) },
                forwardClearance: { 2 },
                plan: { _, goal in plans += 1; return [goal] },
                lastAckAt: { nil },
                sendCommand: { _ in try await send.send() },
                stopRover: {
                    stops += 1
                    if stops > 1 { throw error }
                },
                sleep: { _ in }
            )
            let navigation = Task { await controller.navigateForFollow(to: Vec2(2, 0), stoppingAtForwardClearance: 1.25) }
            await send.waitUntilRequested()

            do {
                try await controller.stopAndConfirm()
                XCTFail("An unconfirmed motor stop must throw")
            } catch let stopError {
                XCTAssertEqual((stopError as NSError).domain, (error as NSError).domain)
                XCTAssertEqual((stopError as NSError).code, (error as NSError).code)
            }

            let result = await navigation.value
            XCTAssertEqual(result, .cancelled, "The motion task is cancelled, not arrived")
            XCTAssertEqual(controller.state, .failed("Rover command failed: stop could not be confirmed."))
            XCTAssertEqual(controller.safetyState, .failed(.commandFailed))
            let nextGoal = await controller.navigateForFollow(to: Vec2(3, 0), stoppingAtForwardClearance: 1.5)
            XCTAssertEqual(nextGoal, .failed(.commandFailed))
            XCTAssertEqual(plans, 1, "Failed stop confirmation must bar subsequent motion")
        }
    }

    func testConfirmedStopPropagatesFailureAndBarsFollowGoal() async {
        var goals = 0
        let controller = NavigationController(
            currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 },
            plan: { _, goal in goals += 1; return [goal] },
            lastAckAt: { nil },
            sendCommand: { _ in },
            stopRover: { throw FakeCommandError.timedOut },
            sleep: { _ in }
        )

        do {
            try await controller.stopAndConfirm()
            XCTFail("Stop must propagate its error")
        } catch {
            XCTAssertEqual(error as? FakeCommandError, .timedOut)
        }
        let result = await controller.navigateForFollow(to: Vec2(2, 0), stoppingAtForwardClearance: 1.5)
        XCTAssertEqual(result, .failed(.commandFailed))
        XCTAssertEqual(goals, 0)
    }

    func testFollowGoalWaitsForMotorStopBeforePlanning() async {
        let stop = SuspendedNavigationStop()
        var plans = 0
        let controller = NavigationController(
            currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 },
            plan: { _, goal in plans += 1; return [goal] },
            lastAckAt: { nil },
            sendCommand: { _ in throw FakeCommandError.timedOut },
            stopRover: { await stop.confirm() },
            sleep: { _ in }
        )
        let goal = Task { await controller.navigateForFollow(to: Vec2(2, 0), stoppingAtForwardClearance: 1.5) }
        await stop.waitUntilRequested()
        XCTAssertEqual(plans, 0)
        stop.finish()
        let result = await goal.value
        XCTAssertEqual(plans, 1)
        XCTAssertEqual(result, .failed(.commandFailed))
    }


    func testFollowGoalDoesNotReportArrivalWhenMotorStopFails() async {
        var stops = 0
        let controller = NavigationController(
            currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 },
            plan: { _, goal in [goal] },
            lastAckAt: { nil },
            sendCommand: { _ in },
            stopRover: {
                stops += 1
                if stops > 1 { throw FakeCommandError.timedOut }
            },
            sleep: { _ in }
        )
        let result = await controller.navigateForFollow(to: .zero, stoppingAtForwardClearance: 1.5)
        XCTAssertEqual(result, .failed(.commandFailed))
    }

    func testFollowGoalDoesNotTreatUnrelatedNearObstacleAsPersonArrival() async {
        let controller = NavigationController(
            currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 0.30 },
            plan: { _, goal in [goal] },
            lastAckAt: { Date() },
            sendCommand: { _ in },
            stopRover: {},
            sleep: { _ in }
        )

        let result = await controller.navigateForFollow(to: Vec2(0, 1),
                                                         stoppingAtForwardClearance: 1.25)

        XCTAssertEqual(result, .failed(.obstacle))
    }

    func testFollowGoalDoesNotCountObstacleAsArrivalEvenNearWaypoint() async {
        let controller = NavigationController(
            currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 0.30 },
            plan: { _, goal in [goal] },
            lastAckAt: { Date() },
            sendCommand: { _ in }, stopRover: {}, sleep: { _ in }
        )

        let result = await controller.navigateForFollow(to: Vec2(0, 0.5),
                                                         stoppingAtForwardClearance: 1.25)
        XCTAssertEqual(result, .failed(.obstacle))
    }
    func testNavigationPublishesTypedCommandFailureDuringMovement() async {
        let controller = NavigationController(
            currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 },
            plan: { _, goal in [goal] },
            lastAckAt: { nil },
            sendCommand: { _ in throw FakeCommandError.timedOut },
            stopRover: {},
            sleep: { _ in }
        )
        let states = controller.safetyStates()
        let received = Task { () -> [NavigationSafetyState] in
            var iterator = states.makeAsyncIterator()
            var result: [NavigationSafetyState] = []
            if let state = await iterator.next() { result.append(state) }
            if let state = await iterator.next() { result.append(state) }
            if let state = await iterator.next() { result.append(state) }
            return result
        }

        _ = await controller.navigateForFollow(to: Vec2(2, 0), stoppingAtForwardClearance: 1.25)

        let safetyStates = await received.value
        XCTAssertEqual(safetyStates, [.idle, .moving, .failed(.commandFailed)])
    }

}

@MainActor
private final class SuspendedNavigationStop {
    private var requested = false
    private var requestWaiter: CheckedContinuation<Void, Never>?
    private var stopWaiter: CheckedContinuation<Void, Never>?
    func confirm() async {
        guard !requested else { return }
        requested = true
        requestWaiter?.resume()
        requestWaiter = nil
        await withCheckedContinuation { stopWaiter = $0 }
    }
    func waitUntilRequested() async {
        if requested { return }
        await withCheckedContinuation { requestWaiter = $0 }
    }
    func finish() {
        stopWaiter?.resume()
        stopWaiter = nil
    }
}

private enum FakeCommandError: LocalizedError {
    case timedOut

    var errorDescription: String? {
        "Timed out talking to rover."
    }
}

@MainActor
private final class SuspendedNavigationSend {
    private let error: Error
    private var requested = false
    private var requestWaiter: CheckedContinuation<Void, Never>?

    init(error: Error) { self.error = error }

    func send() async throws {
        requested = true
        requestWaiter?.resume()
        requestWaiter = nil
        do {
            try await Task.sleep(for: .seconds(60))
            XCTFail("Expected the suspended send to be cancelled")
        } catch {
            throw self.error
        }
    }

    func waitUntilRequested() async {
        if requested { return }
        await withCheckedContinuation { requestWaiter = $0 }
    }
}
