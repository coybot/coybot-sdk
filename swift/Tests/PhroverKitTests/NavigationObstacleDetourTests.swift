import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class NavigationObstacleDetourTests: XCTestCase {
    func testNativeDetourRetainsOwnerAndReturnsConfirmedPostStopReobservation() async {
        let f = Fixture()
        let context = FollowObstacleDetourContext(request: { _ in
            .init(id: UUID(), generation: 1, deadline: f.time + 12, preferredSide: 1, target: Vec2(2, 0))
        }, began: { _ in f.detouring = true }, target: { Vec2(2, 0) })
        let result = await FollowObstacleDetourScope.$context.withValue(context) {
            await f.nav.performFollowMotion(.following(Vec2(5, 0), 1.25), context:
                .init(sessionGeneration: 1, requestToken: 1, purpose: .followGoal, phase: "following"))
        }
        XCTAssertTrue(f.detouring)
        XCTAssertEqual(result.result, .reobservationRequired)
        XCTAssertNil(result.failure)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertNotNil(result.turnStopFence)
        XCTAssertEqual(result.turnStopFence?.context, result.context)
        XCTAssertTrue(f.commands.contains { $0.left < 0 && $0.right > 0 })
        XCTAssertTrue(f.commands.contains { $0.left > 0 && $0.right < 0 })
        XCTAssertTrue(f.commands.contains { $0.left > 0 && $0.right > 0 })
        XCTAssertTrue(f.commands.filter { $0.left > 0 && $0.right > 0 }.allSatisfy { max($0.left, $0.right) <= 0.12 })
        XCTAssertLessThanOrEqual(f.position.distance(to: .zero), 0.35)
        XCTAssertEqual(f.wheels, .stop)
    }

    @MainActor
    private final class Fixture {
        let clock = DetourTestClock()
        var time: Double { get { clock.value } set { clock.value = newValue } }
        var sequence: UInt64 = 1
        var position = Vec2.zero
        var yaw = 0.0
        var wheels = WheelCommand.stop
        var commands: [WheelCommand] = []
        var detouring = false
        var deadlineDuration = 12.0
        var failDetourStop = false
        var blockAtForwardTransport = false
        var forwardTransportEntered = false
        var afterSleep: (() -> Void)?
        func run() async -> FollowMotionResult {
            let context = FollowObstacleDetourContext(request: { _ in
                .init(id: UUID(), generation: 1, deadline: self.time + self.deadlineDuration,
                    preferredSide: 1, target: Vec2(2, 0))
            }, began: { _ in self.detouring = true }, target: { Vec2(2, 0) })
            return await FollowObstacleDetourScope.$context.withValue(context) {
                await nav.performFollowMotion(.following(Vec2(5, 0), 1.25), context:
                    .init(sessionGeneration: 1, requestToken: 1, purpose: .followGoal, phase: "following"))
            }
        }
        var sample: NavigationPoseSample {
            .init(pose: .init(position: position, yaw: yaw), frameID: .init(generation: 1, sequence: sequence),
                sourceTimestamp: time - 0.001, trackingQuality: .normal)
        }
        lazy var nav: FollowNavigationController = makeNavigation()
        private func makeNavigation() -> FollowNavigationController {
            let clock = clock
            return FollowNavigationController(currentPose: { self.sample.pose }, forwardClearance: { 2 }, depthSafetyObservation: { command in
                let blocked = (!self.detouring || self.forwardTransportEntered) && command.left > 0 && command.right > 0
                return .init(state: blocked ? .stop : .clear, clearance: blocked ? 0.25 : 2,
                    supportCount: 30, sampleAge: 0.01, requiredStoppingDistance: 0.3,
                    motionClass: .classify(command), speedLimit: nil)
            }, plan: { _, _ in XCTFail("No mesh planning"); return nil }, lastAckAt: { @MainActor in Date(timeIntervalSince1970: self.time) },
                sendCommand: { _ in XCTFail("Use authorized transport") }, stopRover: { @MainActor in
                    if self.detouring && self.failDetourStop { throw URLError(.timedOut) }
                    self.wheels = .stop
                },
                sleep: { @MainActor duration in
                    let delta = Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
                    let v = (self.wheels.left + self.wheels.right) / 2
                    let w = (self.wheels.right - self.wheels.left) / RoverConfig.wheelBase
                    self.position = self.position + Vec2(cos(self.yaw), sin(self.yaw)) * (v * delta)
                    self.yaw += w * delta
                    self.time += delta
                    self.sequence += 1
                    self.nav.ingestFollowTurnSource(self.sample)
                    self.afterSleep?()
                    await Task.yield()
                }, now: { Date(timeIntervalSince1970: self.time) }, sendCommandReceipt: { @MainActor command in
                    if self.detouring && self.blockAtForwardTransport && command.left > 0 && command.right > 0 {
                        self.forwardTransportEntered = true
                    }
                    guard let authorization = FollowTurnBurstTransportScope.authorization,
                          await authorization.authorizeAttempt?() ?? true,
                          authorization.isAuthorized() else {
                        return .init(receipt: .init(httpStatus: nil, acknowledged: false, acknowledgementUTC: nil,
                            attempts: 0, outcome: "fenced"), failure: FollowTurnBurstTransportDenial.fenced)
                    }
                    authorization.didEnterAttempt?(.init(operationID: authorization.operationID, attempt: 1, entryUptime: self.time))
                    self.commands.append(command)
                    self.wheels = command
                    return .init(receipt: .init(httpStatus: 200, acknowledged: true,
                        acknowledgementUTC: Date(timeIntervalSince1970: self.time), attempts: 1, outcome: "acknowledged"), failure: nil)
                }, poseSample: { self.sample }, sourceNow: { self.time }, sourceStopSnapshot: { self.sample },
                transportUptime: { [clock] in clock.value }, pursuitPlan: { _, goal in .init(path: [goal], planningMode: .reactiveDepth) })
        }
    }

    func testObstacleAppearingAtFinalTransportAdmissionPreventsAdvance() async {
        let f = Fixture()
        f.blockAtForwardTransport = true
        let result = await f.run()
        XCTAssertTrue(f.forwardTransportEntered)
        XCTAssertEqual(result.result, .failed(.obstacle))
        XCTAssertFalse(f.commands.contains { $0.left > 0 && $0.right > 0 })
        XCTAssertEqual(f.wheels, .stop)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testFailedDetourStopCannotAuthorizeAnyMotion() async {
        let f = Fixture()
        f.failDetourStop = true
        let result = await f.run()
        XCTAssertTrue(f.detouring)
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertEqual(result.stopOutcome, .failed)
        XCTAssertTrue(f.commands.isEmpty)
    }

    func testDeadlineDuringTurnStopsWithoutAdvancing() async {
        let f = Fixture()
        f.deadlineDuration = 0.7
        let result = await f.run()
        XCTAssertNotEqual(result.result, .reobservationRequired)
        XCTAssertFalse(f.commands.contains { $0.left > 0 && $0.right > 0 })
        XCTAssertEqual(f.wheels, .stop)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testOperatorStopDuringNestedTurnFencesRemainingManeuver() async {
        let f = Fixture()
        var stop: Task<Void, Never>?
        var sendsAtStop = 0
        f.afterSleep = {
            guard f.detouring, !f.commands.isEmpty, stop == nil else { return }
            sendsAtStop = f.commands.count
            stop = Task { await f.nav.cancelAndWait() }
        }
        let result = await f.run()
        await stop?.value
        XCTAssertNotNil(stop)
        XCTAssertEqual(result.result, .cancelled)
        XCTAssertEqual(f.commands.count, sendsAtStop)
        XCTAssertEqual(f.wheels, .stop)
    }
}

private final class DetourTestClock: @unchecked Sendable {
    private let lock = NSLock()
    private var time = 10.0
    var value: Double {
        get { lock.withLock { time } }
        set { lock.withLock { time = newValue } }
    }
}
