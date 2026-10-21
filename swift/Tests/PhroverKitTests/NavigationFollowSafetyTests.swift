import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class NavigationFollowSafetyTests: XCTestCase {
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
