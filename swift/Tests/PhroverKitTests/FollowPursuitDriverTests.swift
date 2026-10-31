import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowPursuitDriverTests: XCTestCase {
    func testCancelledMovingPredecessorCannotReuseAdmissionStopConfirmation() async {
        let f = ControllerFixture()
        f.autoAdvance = false
        f.arrival = .zero
        let tickGate = FollowDiagnosticSuspension()
        let stopGate = FollowDiagnosticSuspension()
        f.sleepHandler = { _ in
            if f.commands.count == 1 { await tickGate.suspend() }
            else { await Task.yield() }
        }
        f.stopHandler = { if f.stops == 2 { await stopGate.suspend() } }
        let task = Task { await f.run() }
        await wait { tickGate.entered }
        f.controller.cancel()
        tickGate.release()
        let result = await task.value
        await wait { stopGate.entered }
        XCTAssertEqual(result.result, .cancelled)
        XCTAssertNotEqual(result.stopOutcome, .confirmed, "The initial admission stop predates the moving command")
        XCTAssertEqual(result.commandReceipt?.attempts, 1)
        XCTAssertEqual(result.commandReceipt?.acknowledged, true)
        stopGate.release()
        try? await f.controller.stopAndConfirm()
    }

    func testEpisodeExpiryStillConfirmsStopForTheCurrentOwner() async {
        let f = ControllerFixture()
        f.arrival = .zero
        let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 1,
            deadline: 10.08, now: { f.clock.value }, canContinue: { true })
        let result = await FollowRecoveryScope.$authorization.withValue(authorization) { await f.run() }
        XCTAssertEqual(result.result, .cancelled)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertGreaterThanOrEqual(f.stops, 2)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testFailedInLoopStopLatchesFailureAndCannotResumeReplacementPlan() async {
        let f = ControllerFixture()
        f.autoAdvance = false
        f.holdPlanNumber = 2
        f.failStopNumber = 2
        f.afterSend = {
            if f.commands.count == 1 { XCTAssertTrue(f.controller.updateFollowGoal(Vec2(0, 5))) }
        }
        let result = await f.run()
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertEqual(result.stopOutcome, .failed)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertFalse(f.controller.updateFollowGoal(Vec2(1, 1)))
        f.planGate.release()
    }

    func testRealControllerStopsAtOldGoalWhileReplacementPlanIsHeld() async {
        let f = ControllerFixture()
        f.autoAdvance = false
        f.holdPlanNumber = 2
        let stopped = expectation(description: "in-loop stop sent without joining the pursuit task")
        f.onStop = { if f.stops == 2 { stopped.fulfill() } }
        f.afterSend = {
            if f.commands.count == 1 {
                f.arrival = Vec2(0, 5)
                XCTAssertTrue(f.controller.updateFollowGoal(f.arrival))
            }
        }
        let task = Task { await f.run() }
        await fulfillment(of: [stopped], timeout: 2)
        await wait { f.planGate.entered }
        XCTAssertEqual(f.commands.count, 1)
        f.planGate.release()
        let result = await task.value
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(f.commands.count, 2)
    }

    func testRetargetCannotHideAnEnteredFirstSendFailure() async {
        let f = ControllerFixture()
        f.failAfterAttempt = true
        f.afterAttempt = {
            f.freshAt = .infinity
            XCTAssertTrue(f.controller.updateFollowGoal(Vec2(0, 5)))
        }
        let result = await f.run()
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertLessThan(f.clock.value, 10.30, "No stopped-depth refresh is allowed after uncertain motor admission")
    }

    func testProvenZeroAttemptRetargetMayContinueWithoutInventingMotion() async {
        let f = ControllerFixture()
        f.beforeAttempt = {
            f.beforeAttempt = nil
            f.arrival = Vec2(0, 5)
            XCTAssertTrue(f.controller.updateFollowGoal(f.arrival))
        }
        let result = await f.run()
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertEqual(f.stops, 2, "Only the initial/final stops are required for a proven unsent replacement")
    }

    func testMovingReplanJoinsForwardPathAfterRoundingCorner() async {
        let f = Fixture()
        let destination = Vec2(1, 1)
        XCTAssertTrue(f.driver.updateGoal(destination))
        var map = Costmap(width: 80, height: 80, resolution: 0.1, origin: Vec2(-2, -2))
        let cell = map.worldToCell(Vec2(0.5, 0.3))
        map.setCost(cell.cx, cell.cy, 255)
        let capturedMap = map
        let route = [Vec2.zero, Vec2(1, 0), destination]
        f.planProvider = { _, _ in
            if f.plannedGoals.count == 2 { f.position = Vec2(1, 0.6) }
            return .init(path: route, map: capturedMap)
        }
        f.onSend = { _ in
            if f.commands.count == 1 { f.time += 1.01 }
            if f.paths.count == 2 { f.position = destination }
        }
        XCTAssertFalse(map.isSegmentClear(from: Vec2(1, 0.6), to: .zero, margin: 0.02))
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.paths.count, 2)
        XCTAssertEqual(f.paths.last?.first, destination)
    }

    func testRealControllerUsesAsyncPlanningAndStoppedDepthRefresh() async {
        let f = ControllerFixture()
        f.freshAt = 10.15
        let result = await f.run()
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertGreaterThanOrEqual(f.firstSendTime ?? 0, 10.15)
        XCTAssertGreaterThan(f.depthReads, 1)
        XCTAssertGreaterThanOrEqual(f.stops, 2)
    }

    func testRealControllerRejectsDepthExpiryAndNewObstacleAtTransport() async {
        for expired in [false, true] {
            let f = ControllerFixture()
            f.beforeAttempt = {
                if expired { f.clock.value += 0.30 } else { f.obstacle = true }
            }
            let result = await f.run()
            XCTAssertEqual(result.result, expired ? .failed(.depthUnavailable(.staleRawDepth)) : .failed(.obstacle))
            XCTAssertTrue(f.commands.isEmpty)
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
    }

    func testRealControllerCoalescesStartupGoalAndFencesCancelledWorker() async {
        for cancel in [false, true] {
            let f = ControllerFixture()
            f.autoAdvance = false
            f.holdFirstPlan = true
            let task = Task { await f.run() }
            await wait { f.planGate.entered }
            XCTAssertTrue(f.commands.isEmpty)
            if cancel {
                f.controller.cancel()
            } else {
                f.arrival = Vec2(0, 5)
                XCTAssertTrue(f.controller.updateFollowGoal(f.arrival))
            }
            f.planGate.release()
            let result = await task.value
            if cancel {
                XCTAssertEqual(result.result, .cancelled)
                XCTAssertTrue(f.commands.isEmpty)
            } else {
                XCTAssertEqual(result.result, .arrived)
                XCTAssertEqual(f.plannedGoals, [Vec2(5, 0), Vec2(0, 5)])
                XCTAssertEqual(f.commands.count, 1)
                XCTAssertGreaterThan(f.commands[0].right, f.commands[0].left)
            }
            XCTAssertFalse(f.controller.updateFollowGoal(Vec2(4, 4)))
        }
    }

    func testSnapshotWorkerLeavesControlActorResponsiveAndKeepsCapturedGeometry() async throws {
        let entered = expectation(description: "worker entered")
        let gate = DispatchSemaphore(value: 0)
        defer { gate.signal() }
        var vertices: [SIMD3<Float>] = [.zero, SIMD3(1, 0, 1)]
        let snapshot = CostmapBuilder.MeshSnapshot(worldVertices: vertices)
        let task = Task {
            await CostmapBuilder.planCaptured(snapshot, start: .zero, goal: Vec2(1, 0)) { mesh, _, goal in
                XCTAssertFalse(Thread.isMainThread)
                entered.fulfill()
                _ = gate.wait(timeout: .now() + 3)
                XCTAssertEqual(mesh.worldVertices.first, .zero)
                return .init(path: [goal])
            }
        }
        await fulfillment(of: [entered], timeout: 2)
        vertices[0] = SIMD3(99, 99, 99)
        XCTAssertEqual(vertices[0].x, 99, "Control work runs while the worker is held")
        gate.signal()
        let result = await task.value
        XCTAssertEqual(result?.path, [Vec2(1, 0)])
        let actual = await CostmapBuilder.planCaptured(snapshot, start: .zero, goal: Vec2(1, 0))
        XCTAssertNotNil(actual?.map)
        XCTAssertFalse(actual?.path.isEmpty ?? true)
    }

    func testRecordedStaleStartupDepthWaitsStoppedThenSendsFresh() async {
        let f = Fixture()
        f.depthProvider = { _ in f.observation(age: f.time < 10.15 ? 0.30918320833006874 : 0.01) }
        f.onSend = { _ in
            XCTAssertGreaterThanOrEqual(f.time, 10.15)
            f.position = f.driver.goal
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertEqual(f.events.filter { $0 == "depth_wait_started" }.count, 1)
        XCTAssertEqual(f.events.filter { $0 == "depth_wait_recovered" }.count, 1)
    }

    func testRepeatedStaleFramesCannotRenewFixedRefreshWindow() async {
        let f = Fixture()
        f.depthProvider = { _ in f.observation(age: 0.309) }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertTrue(f.commands.isEmpty)
        XCTAssertLessThanOrEqual(f.time, 10.70)
        XCTAssertEqual(f.events.filter { $0 == "depth_wait_started" }.count, 1)
    }

    func testFreshDepthAtWaitDeadlineDoesNotAuthorizeLateSend() async {
        let f = Fixture()
        var firstRead: Double?
        f.depthProvider = { _ in
            if firstRead == nil { firstRead = f.time }
            return f.observation(age: f.time >= firstRead! + 0.6 ? 0 : 0.309)
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertTrue(f.commands.isEmpty)
    }

    func testHardDepthFaultAndPostMovementStalenessNeverUseStartupWait() async {
        for failure in [DepthSafetyUnavailableReason.invalidCalibration, .malformedDepth, .blindSweptVolume] {
            let f = Fixture()
            f.depthProvider = { _ in .unavailable(failure, sampleAge: 0.01, motionClass: .forward) }
            let result = await f.driver.run()
            XCTAssertEqual(result, .failed(.depthUnavailable(failure)))
            XCTAssertTrue(f.commands.isEmpty)
            XCTAssertFalse(f.events.contains("depth_wait_started"))
        }
        let f = Fixture()
        f.depthProvider = { _ in f.observation(age: f.commands.isEmpty ? 0.01 : 0.309) }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertFalse(f.events.contains("depth_wait_started"))
    }

    func testCancellationSourceResetAndPlanningTimeoutCannotPublishOrSend() async {
        for kind in ["cancel", "generation", "timeout", "wrong_goal"] {
            let f = Fixture()
            f.planProvider = { _, goal in
                switch kind {
                case "cancel": f.allowed = false
                case "generation": f.generation = 2
                case "timeout": f.time += 1
                default: return .init(path: [Vec2(99, 99)])
                }
                return .init(path: [goal])
            }
            let result = await f.driver.run()
            XCTAssertNotEqual(result, .arrived, kind)
            XCTAssertTrue(f.commands.isEmpty, kind)
            XCTAssertTrue(f.paths.isEmpty, kind)
        }
    }

    func testObsoleteStartupPlanCannotSendTowardReplacedTarget() async {
        let f = Fixture()
        let gate = FollowDiagnosticSuspension()
        f.autoAdvance = false
        f.planProvider = { _, goal in
            if f.plannedGoals.count == 1 { await gate.suspend() }
            return .init(path: [goal])
        }
        f.onSend = { _ in f.position = f.driver.goal }
        let task = Task { await f.driver.run() }
        await wait { gate.entered }
        XCTAssertTrue(f.commands.isEmpty)
        XCTAssertTrue(f.driver.updateGoal(Vec2(0, 5)))
        gate.release()
        let result = await task.value
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(f.paths, [[Vec2(0, 5)]])
        XCTAssertEqual(f.plannedGoals, [Vec2(5, 0), Vec2(0, 5)])
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertGreaterThan(f.commands[0].right, f.commands[0].left)
    }

    func testPeriodicWorkerDoesNotBlockDepthSafetyAndLateCompletionCannotPublish() async {
        let f = Fixture()
        let gate = FollowDiagnosticSuspension()
        f.planProvider = { _, goal in
            if f.plannedGoals.count == 2 { f.autoAdvance = false; await gate.suspend() }
            return .init(path: [goal])
        }
        let task = Task { await f.driver.run() }
        await wait { gate.entered }
        let previousCount = f.commands.count
        await wait { f.commands.count > previousCount }
        f.depthProvider = { _ in .unavailable(.invalidCalibration, sampleAge: 0.01, motionClass: .forward) }
        let result = await task.value
        XCTAssertEqual(result, .failed(.depthUnavailable(.invalidCalibration)))
        gate.release()
        for _ in 0..<30 { await Task.yield() }
        XCTAssertEqual(f.paths.count, 1)
        XCTAssertFalse(f.driver.updateGoal(Vec2(1, 1)))
    }

    func testLiveGoalUpdatesCoalesceAndDoNotResetProgressWatchdog() async {
        let f = Fixture()
        f.onSend = { _ in
            _ = f.driver.updateGoal(Vec2(5, f.commands.count.isMultiple(of: 2) ? 0.1 : -0.1))
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.stalled))
        XCTAssertLessThan(f.time, 13)
        XCTAssertGreaterThan(f.commands.count, 5)
        XCTAssertEqual(f.stops, 0)
    }

    func testSafetyIsRecheckedAfterAcknowledgementAndBeforeFirstSend() async {
        let f = Fixture()
        f.acknowledgement = { f.clearance = 0.2; return Date(timeIntervalSince1970: f.time) }
        let result = await f.driver.run()
        XCTAssertEqual(result, .failed(.obstacle))
        XCTAssertTrue(f.commands.isEmpty)
    }

    func testCautionRechecksScaledCommandAndRetainsSpeedLimit() async {
        let f = Fixture()
        var reads = 0
        f.depthProvider = { command in
            reads += 1
            return .init(state: .caution, clearance: 0.5, supportCount: 3, sampleAge: 0.01,
                requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: 0.12)
        }
        f.onSend = { command in
            XCTAssertLessThanOrEqual(max(abs(command.left), abs(command.right)), 0.12)
            f.position = f.driver.goal
        }
        let result = await f.driver.run()
        XCTAssertEqual(result, .arrived)
        XCTAssertGreaterThanOrEqual(reads, 2)
    }

    private func wait(_ condition: () -> Bool, file: StaticString = #filePath, line: UInt = #line) async {
        for _ in 0..<4000 where !condition() { await Task.yield() }
        XCTAssertTrue(condition(), file: file, line: line)
    }

    @MainActor
    private final class ControllerFixture {
        let clock = PursuitTransportClock()
        let planGate = FollowDiagnosticSuspension()
        var holdFirstPlan = false
        var holdPlanNumber: Int?
        var autoAdvance = true
        var plannedGoals: [Vec2] = []
        var commands: [WheelCommand] = []
        var stops = 0
        var failStopNumber: Int?
        var depthReads = 0
        var freshAt = 0.0
        var obstacle = false
        var position = Vec2.zero
        var arrival = Vec2(5, 0)
        var sequence: UInt64 = 1
        var firstSendTime: Double?
        var beforeAttempt: (() -> Void)?
        var afterAttempt: (() -> Void)?
        var afterSend: (() -> Void)?
        var onStop: (() -> Void)?
        var failAfterAttempt = false
        var sleepHandler: (@MainActor (Duration) async -> Void)?
        var stopHandler: (@MainActor () async -> Void)?
        var sample: NavigationPoseSample {
            .init(pose: .init(position: position, yaw: 0), frameID: .init(generation: 1, sequence: sequence),
                sourceTimestamp: clock.value - 0.001, trackingQuality: .normal)
        }
        lazy var controller: NavigationController = makeController()
        private func makeController() -> NavigationController {
            let clock = self.clock
            return NavigationController(currentPose: { self.sample.pose }, forwardClearance: { 5 },
                depthSafetyObservation: { command in
                    self.depthReads += 1
                    if self.obstacle { return .init(state: .stop, clearance: 0.1, supportCount: 5, sampleAge: 0.01,
                        requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: nil) }
                    if clock.value < self.freshAt {
                        return .unavailable(.staleRawDepth, sampleAge: 0.30918320833006874, motionClass: .classify(command))
                    }
                    return .init(state: .clear, clearance: 5, supportCount: 0, sampleAge: 0.01,
                        requiredStoppingDistance: 0.3, motionClass: .classify(command), speedLimit: nil)
                }, plan: { _, _ in XCTFail("Production-style pursuit must not call the synchronous planner"); return nil },
                lastAckAt: { Date(timeIntervalSince1970: clock.value) }, sendCommand: { _ in XCTFail("Receipt sender required") },
                stopRover: {
                    self.stops += 1; self.onStop?()
                    if self.stops == self.failStopNumber { throw URLError(.timedOut) }
                    await self.stopHandler?()
                }, sleep: { duration in
                    if let handler = self.sleepHandler { await handler(duration); return }
                    if self.autoAdvance { clock.value += Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18 }
                    self.sequence += 1
                    await Task.yield()
                }, now: { Date(timeIntervalSince1970: clock.value) }, sendCommandReceipt: { command in
                    self.beforeAttempt?()
                    guard let authorization = FollowTurnBurstTransportScope.authorization,
                          await authorization.authorizeAttempt?() == true, authorization.isAuthorized() else {
                        let expired = FollowTurnBurstTransportScope.authorization.map { clock.value >= $0.deadline } ?? false
                        return .init(receipt: .init(httpStatus: nil, acknowledged: false, acknowledgementUTC: nil,
                            attempts: 0, outcome: expired ? "expired" : "fenced"),
                            failure: expired ? FollowTurnBurstTransportDenial.expired : .fenced)
                    }
                    authorization.didEnterAttempt?(.init(operationID: authorization.operationID, attempt: 1, entryUptime: clock.value))
                    self.commands.append(command)
                    self.firstSendTime = self.firstSendTime ?? clock.value
                    if self.failAfterAttempt {
                        self.afterAttempt?()
                        return .init(receipt: .init(httpStatus: nil, acknowledged: false, acknowledgementUTC: nil,
                            attempts: 1, outcome: "fenced"), failure: FollowTurnBurstTransportDenial.fenced)
                    }
                    self.position = self.arrival
                    self.sequence += 1
                    self.afterSend?()
                    return .init(receipt: .init(httpStatus: 200, acknowledged: true,
                        acknowledgementUTC: Date(timeIntervalSince1970: clock.value), attempts: 1, outcome: "acknowledged"), failure: nil)
                }, poseSample: { self.sample }, sourceNow: { clock.value }, sourceStopSnapshot: { self.sample },
                transportUptime: { clock.value }, pursuitPlan: { _, goal in
                    self.plannedGoals.append(goal)
                    if (self.holdFirstPlan && self.plannedGoals.count == 1) || self.plannedGoals.count == self.holdPlanNumber {
                        await self.planGate.suspend()
                    }
                    return .init(path: [goal])
                })
        }
        func run() async -> FollowMotionResult {
            await controller.performFollowMotion(.following(Vec2(5, 0), 1.25), context:
                .init(sessionGeneration: 1, requestToken: 1, purpose: .followGoal, phase: "following"))
        }
    }

    @MainActor
    private final class Fixture {
        var time = 10.0
        var generation: UInt64 = 1
        var sequence: UInt64 = 1
        var position = Vec2.zero
        var allowed = true
        var autoAdvance = true
        var clearance = 5.0
        var commands: [WheelCommand] = []
        var plannedGoals: [Vec2] = []
        var paths: [[Vec2]] = []
        var events: [String] = []
        var stops = 0
        var planProvider: (@MainActor (Vec2, Vec2) async -> FollowPursuitPlan?)?
        var depthProvider: ((WheelCommand) -> DepthSafetyObservation)?
        var onSend: ((WheelCommand) -> Void)?
        var acknowledgement: (@MainActor () async -> Date?)?
        func observation(age: Double) -> DepthSafetyObservation {
            .init(state: .clear, clearance: 5, supportCount: 0, sampleAge: age,
                requiredStoppingDistance: 0.3, motionClass: .forward, speedLimit: nil)
        }
        lazy var driver: FollowPursuitDriver = makeDriver()
        private func makeDriver() -> FollowPursuitDriver {
            FollowPursuitDriver(goal: Vec2(5, 0), environment: .init(plan: { start, goal in
            self.plannedGoals.append(goal)
            if let provider = self.planProvider { return await provider(start, goal) }
            return .init(path: [goal])
        }, pose: {
            .init(pose: .init(position: self.position, yaw: 0), frameID: .init(generation: self.generation, sequence: self.sequence),
                sourceTimestamp: self.time - 0.001, trackingQuality: .normal)
        }, owned: { self.allowed }, uptime: { self.time }, date: { Date(timeIntervalSince1970: self.time) },
        acknowledgement: {
            if let read = self.acknowledgement { return await read() }
            return Date(timeIntervalSince1970: self.time)
        }, clearance: { self.clearance }, depth: { self.depthProvider?($0) ?? self.observation(age: 0.01) },
        send: { command, expiry in
            XCTAssertLessThan(self.time, expiry)
            self.commands.append(command)
            self.onSend?(command)
        }, stop: { self.stops += 1 }, sleep: { duration in
            if self.autoAdvance {
                self.time += Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
            }
            self.sequence += 1
            await Task.yield()
        }, publishPath: { self.paths.append($0) }, moving: {}, emit: { name, _ in self.events.append(name) }))
        }
    }
}

private final class PursuitTransportClock: @unchecked Sendable {
    private let lock = NSLock()
    private var time = 10.0
    var value: Double {
        get { lock.withLock { time } }
        set { lock.withLock { time = newValue } }
    }
}
