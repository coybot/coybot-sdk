import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class NavigationFollowLiveGoalTests: XCTestCase {
    private final class Log { var commands: [WheelCommand] = []; var stops = 0 }

    private func controller(_ log: Log, plan: @escaping (Vec2, Vec2) -> [Vec2]? = { _, goal in [goal] }) -> NavigationController {
        NavigationController(
            currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 5 },
            plan: plan,
            lastAckAt: { Date() },
            sendCommand: { log.commands.append($0) },
            stopRover: { log.stops += 1 },
            sleep: { _ in await Task.yield() }
        )
    }

    private func context() -> FollowMotionRequestContext {
        .init(sessionGeneration: 1, requestToken: 1, purpose: .followGoal, phase: "following")
    }

    private func waitForCommands(_ log: Log, atLeast count: Int) async {
        for _ in 0..<2000 where log.commands.count < count { await Task.yield() }
    }

    func testLiveGoalRedirectsTheRunningDriveWithoutAStop() async {
        let log = Log()
        let nav = controller(log)
        let drive = Task { await nav.performFollowMotion(.following(Vec2(5, 0), 1.25), context: context()) }
        await waitForCommands(log, atLeast: 3)
        let stopsBefore = log.stops
        XCTAssertTrue(log.commands.suffix(3).allSatisfy { $0.left > 0 && $0.right > 0 }, "driving straight at +x")

        XCTAssertTrue(nav.updateFollowGoal(Vec2(0, 5)))
        let mark = log.commands.count
        await waitForCommands(log, atLeast: mark + 3)
        XCTAssertEqual(log.stops, stopsBefore, "the redirect itself must not stop the rover")
        let turning = log.commands.suffix(3)
        XCTAssertTrue(turning.allSatisfy { $0.right > $0.left }, "now turning left toward +y: \(Array(turning))")

        nav.cancel()
        _ = await drive.value
    }

    func testReachingTheLiveGoalEndsTheDriveAsArrived() async {
        let log = Log()
        let nav = controller(log)
        let drive = Task { await nav.performFollowMotion(.following(Vec2(5, 0), 1.25), context: context()) }
        await waitForCommands(log, atLeast: 2)
        XCTAssertTrue(nav.updateFollowGoal(Vec2(0.1, 0)))   // inside the 0.2 m goal tolerance
        let result = await drive.value
        XCTAssertEqual(result.result, .arrived)
    }

    func testLiveGoalIsRefusedWhileArrivalStopIsSuspended() async {
        let log = Log()
        var suspendNextStop = false
        var arrivalStop: CheckedContinuation<Void, Never>?
        let nav = NavigationController(
            currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 5 },
            plan: { _, goal in [goal] },
            lastAckAt: { Date() },
            sendCommand: { log.commands.append($0) },
            stopRover: {
                log.stops += 1
                if suspendNextStop {
                    suspendNextStop = false
                    await withCheckedContinuation { arrivalStop = $0 }
                }
            },
            sleep: { _ in await Task.yield() }
        )
        let drive = Task { await nav.performFollowMotion(.following(Vec2(5, 0), 1.25), context: context()) }
        await waitForCommands(log, atLeast: 2)
        XCTAssertGreaterThanOrEqual(log.commands.count, 2)

        // The initial admission stop has completed. Suspend the next stop exactly
        // when the running drive reaches this replacement goal.
        suspendNextStop = true
        XCTAssertTrue(nav.updateFollowGoal(Vec2(0.1, 0)))
        for _ in 0..<2000 where arrivalStop == nil { await Task.yield() }
        guard let stop = arrivalStop else {
            XCTFail("The drive did not enter its arrival stop")
            suspendNextStop = false
            nav.cancel()
            _ = await drive.value
            return
        }
        let commandsAtStop = log.commands.count
        XCTAssertFalse(nav.updateFollowGoal(Vec2(0, 5)),
            "An arrival stop must refuse a goal that the terminating drive will discard")

        stop.resume()
        arrivalStop = nil
        let result = await drive.value
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(log.commands.count, commandsAtStop)
    }

    func testLiveGoalIsRefusedWhenNothingIsDriving() {
        let nav = controller(Log())
        XCTAssertFalse(nav.updateFollowGoal(Vec2(1, 1)))
    }

    func testLiveGoalIsRefusedWhenTheNewGoalCannotBePlanned() async {
        let log = Log()
        var blocked = false
        let nav = controller(log, plan: { _, goal in blocked ? nil : [goal] })
        let drive = Task { await nav.performFollowMotion(.following(Vec2(5, 0), 1.25), context: context()) }
        await waitForCommands(log, atLeast: 2)
        blocked = true
        XCTAssertFalse(nav.updateFollowGoal(Vec2(0, 5)))
        nav.cancel()
        _ = await drive.value
    }

    func testLiveGoalIsRefusedAfterTheDriveHasEnded() async {
        let log = Log()
        let nav = controller(log)
        let drive = Task { await nav.performFollowMotion(.following(Vec2(0.05, 0), 1.25), context: context()) }
        _ = await drive.value
        XCTAssertFalse(nav.updateFollowGoal(Vec2(3, 3)))
    }
}
