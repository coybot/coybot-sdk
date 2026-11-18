import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowObstacleDetourTests: XCTestCase {
    func testOppositeSideClearAdvancesMeasuredDistanceThenFacesUpdatedTarget() async {
        let f = Fixture()
        f.target = Vec2(2, 1)
        let result = await f.detour.run()
        XCTAssertEqual(result, .reobservationRequired)
        XCTAssertEqual(f.headings.first ?? 0, .pi / 12, accuracy: 1e-8)
        XCTAssertGreaterThanOrEqual(f.position.distance(to: .zero), 0.23)
        XCTAssertLessThanOrEqual(f.position.distance(to: .zero), 0.30)
        XCTAssertEqual(f.headings.last ?? 0, atan2(f.target.y - f.position.y, f.target.x - f.position.x), accuracy: 1e-8)
        XCTAssertTrue(f.commands.allSatisfy { $0.left > 0 && $0.right > 0 && max($0.left, $0.right) <= 0.12 })
        XCTAssertFalse(f.moving)
    }

    func testBlockedPreferredSideInspectsOtherSideBeforeAdvancing() async {
        let f = Fixture()
        f.clear = { $0 < -0.1 }
        let result = await f.detour.run()
        XCTAssertEqual(result, .reobservationRequired)
        let expected: [Double] = [.pi / 12, .pi / 6, .pi / 4, 0, -.pi / 12]
        XCTAssertGreaterThanOrEqual(f.headings.count, expected.count)
        for (actual, expected) in zip(f.headings, expected) { XCTAssertEqual(actual, expected, accuracy: 1e-8) }
        XCTAssertLessThan(f.position.y, 0)
        XCTAssertFalse(f.moving)
    }

    func testBothSidesBlockedNeverTranslate() async {
        let f = Fixture()
        f.clear = { _ in false }
        let result = await f.detour.run()
        XCTAssertEqual(result, .failed(.obstacle))
        XCTAssertEqual(f.headings.count, 7)
        XCTAssertTrue(f.commands.isEmpty)
    }

    func testUnsafeRotationDoesNotBecomeBlindForwardRecovery() async {
        let f = Fixture()
        f.turnResult = .failed(.depthUnavailable(.blindSweptVolume))
        let result = await f.detour.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.blindSweptVolume)))
        XCTAssertEqual(f.headings.count, 1)
        XCTAssertTrue(f.commands.isEmpty)
    }

    func testStaleDepthAndFailedStopPreventDetourTranslation() async {
        for stopFails in [true, false] {
            let f = Fixture()
            f.stopFails = stopFails
            f.depthAge = 0.3
            let result = await f.detour.run()
            XCTAssertEqual(result, stopFails ? .failed(.commandFailed) : .failed(.depthUnavailable(.staleRawDepth)))
            XCTAssertTrue(f.commands.isEmpty)
        }
    }

    func testDeadlineAndMeasuredExcursionRemainBounded() async {
        let timeout = Fixture()
        timeout.deadline = timeout.time + 0.2
        let timedOut = await timeout.detour.run()
        XCTAssertEqual(timedOut, .failed(.obstacle))
        XCTAssertTrue(timeout.commands.isEmpty)
        let excursion = Fixture()
        excursion.onSend = { excursion.position = Vec2(1, 0) }
        let result = await excursion.detour.run()
        XCTAssertEqual(result, .failed(.obstacle))
        XCTAssertEqual(excursion.commands.count, 1)
        XCTAssertFalse(excursion.moving)
    }

    func testSourceResetAndOwnershipLossCannotContinueOrStopNewOwner() async {
        let reset = Fixture()
        reset.onSend = { reset.generation = 2 }
        let resetResult = await reset.detour.run()
        XCTAssertEqual(resetResult, .failed(.trackingLost))
        XCTAssertEqual(reset.commands.count, 1)
        let replaced = Fixture()
        var stopsAtReplacement = 0
        replaced.onSend = { stopsAtReplacement = replaced.stops; replaced.allowed = false }
        let replacedResult = await replaced.detour.run()
        XCTAssertEqual(replacedResult, .cancelled)
        XCTAssertEqual(replaced.commands.count, 1)
        XCTAssertEqual(replaced.stops, stopsAtReplacement, "The replacing owner owns independent stop confirmation")
    }

    func testContextChargesOneAttemptBeforeStarting() {
        let f = Fixture()
        var starts = 0
        let context = FollowObstacleDetourContext(request: { _ in f.request }, began: { _ in starts += 1 }, target: { f.target })
        XCTAssertNotNil(context.begin(at: f.source))
        XCTAssertNil(context.begin(at: f.source))
        XCTAssertEqual(starts, 1)
    }

    func testRecordedNearExpiryFrameStopsThenResumesOnFreshPostStopDepth() async {
        let f = Fixture()
        var interrupted = false
        f.onSend = {
            if !interrupted { interrupted = true; f.depthAge = 0.24324625 }
        }
        f.onSleep = {
            if interrupted && !f.moving { f.depthAge = 0.01 }
        }
        let result = await f.detour.run()
        XCTAssertEqual(result, .reobservationRequired)
        XCTAssertGreaterThan(f.commands.count, 1)
        XCTAssertFalse(f.moving)
    }

    func testNearExpiryDepthNeverRecoversBeyondBoundedWindow() async {
        let f = Fixture()
        var interruptedAt = 0.0
        f.onSend = {
            if interruptedAt == 0 { interruptedAt = f.time; f.depthAge = 0.24324625 }
        }
        let result = await f.detour.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertEqual(f.commands.count, 1)
        XCTAssertLessThanOrEqual(f.time - interruptedAt, 0.7)
        XCTAssertFalse(f.moving)
    }

    func testSecondDepthInterruptionCannotRenewDetourRecoveryBudget() async {
        let f = Fixture()
        f.onSend = { f.depthAge = 0.24324625 }
        f.onSleep = { if !f.moving { f.depthAge = 0.01 } }
        let result = await f.detour.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertEqual(f.commands.count, 2)
        XCTAssertFalse(f.moving)
    }

    func testRecoveryStopFailureAndNewObstacleRemainTerminal() async {
        for failedStop in [true, false] {
            let f = Fixture()
            f.onSend = {
                f.depthAge = 0.24324625
                if failedStop { f.stopFails = true }
            }
            f.onSleep = {
                if !f.moving && !f.commands.isEmpty {
                    f.depthAge = 0.01
                    f.clear = { _ in false }
                }
            }
            let result = await f.detour.run()
            XCTAssertEqual(result, .failed(failedStop ? .commandFailed : .obstacle))
            XCTAssertEqual(f.commands.count, 1)
        }
    }

    @MainActor
    private final class Fixture {
        var time = 10.0
        var deadline = 22.0
        var generation: UInt64 = 1
        var sequence: UInt64 = 1
        var position = Vec2.zero
        var yaw = 0.0
        var target = Vec2(2, 0)
        var allowed = true
        var stopFails = false
        var depthAge = 0.01
        var clear: (Double) -> Bool = { _ in true }
        var turnResult: NavigationResult = .arrived
        var commands: [WheelCommand] = []
        var headings: [Double] = []
        var stops = 0
        var moving = false
        var fence: FollowTurnStopFence?
        var onSend: (() -> Void)?
        var onSleep: (() -> Void)?
        var source: NavigationPoseSample {
            .init(pose: .init(position: position, yaw: yaw), frameID: .init(generation: generation, sequence: sequence),
                sourceTimestamp: time - 0.001, trackingQuality: .normal)
        }
        var request: FollowObstacleDetourRequest {
            .init(id: UUID(), generation: 1, deadline: deadline, preferredSide: 1, target: Vec2(2, 0))
        }
        func advanceTime(_ seconds: Double) { time += seconds; sequence += 1 }
        func stop() throws {
            stops += 1
            if stopFails { throw URLError(.timedOut) }
            moving = false
            fence = .init(identity: UUID(), operationGeneration: 1, context: .unknown,
                acknowledgementUptime: time, sourceGeneration: generation,
                highestSequence: sequence, highestSourceTimestamp: time - 0.001)
        }
        lazy var detour: FollowObstacleDetour = {
            let environment = FollowPursuitDriver.Environment(plan: { _, _ in XCTFail("Detour must not plan against mesh"); return nil },
                pose: { self.source }, owned: { self.allowed }, uptime: { self.time }, date: { Date(timeIntervalSince1970: self.time) },
                acknowledgement: { Date(timeIntervalSince1970: self.time) }, clearance: { 2 }, depth: { command in
                    .init(state: self.clear(self.yaw) ? .clear : .stop, clearance: self.clear(self.yaw) ? 2 : 0.25,
                        supportCount: 20, sampleAge: self.depthAge, requiredStoppingDistance: 0.3,
                        motionClass: .classify(command), speedLimit: nil)
                }, send: { command, expiry in
                    XCTAssertLessThan(self.time, expiry)
                    self.commands.append(command); self.moving = true
                    self.position = self.position + Vec2(cos(self.yaw), sin(self.yaw)) * 0.04
                    self.onSend?()
                }, stop: { try self.stop() }, sleep: { duration in
                    self.advanceTime(Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18)
                    self.onSleep?()
                    await Task.yield()
                }, publishPath: { _ in }, moving: {}, emit: { _, _ in })
            return FollowObstacleDetour(request: request, control: environment, turn: { heading in
                self.headings.append(heading)
                guard self.turnResult == .arrived else { return self.turnResult }
                self.yaw = heading
                self.advanceTime(0.1)
                do { try self.stop() } catch { return .failed(.commandFailed) }
                self.advanceTime(0.32)
                return .arrived
            }, stopFence: { self.fence }, target: { self.target })
        }()
    }
}
