import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class ReadyPathRecoveryTests: XCTestCase {
    func testRecordedStaleStartupDepthWaitsStoppedForFreshCaptureBeforeProbing() async {
        let fixture = ProbeFixture()
        // Device recovery failed at 271.521 ms against the unchanged 250 ms limit.
        fixture.rawDepthTimestamp = 10 - 0.27152120834216475
        fixture.freshDepthAt = 10.1
        fixture.onWait = {
            if fixture.time < 10.1 { XCTAssertTrue(fixture.commands.isEmpty) }
        }
        let result = await fixture.run()
        XCTAssertEqual(result, .clear)
        XCTAssertGreaterThanOrEqual(fixture.firstPulseTime ?? 0, 10.1)
        XCTAssertGreaterThan(fixture.depthReads, fixture.commands.count)
        XCTAssertEqual(fixture.stages.filter { $0 == "depth_wait_started" }.count, 1)
        XCTAssertEqual(fixture.stages.filter { $0 == "depth_wait_recovered" }.count, 1)
    }

    func testPersistentOrRepeatedStaleCapturesCannotRenewStartupWait() async {
        for replaceCapture in [false, true] {
            let fixture = ProbeFixture()
            fixture.rawDepthTimestamp = 10 - 0.27152120834216475
            fixture.onWait = {
                XCTAssertTrue(fixture.commands.isEmpty)
                if replaceCapture { fixture.rawDepthTimestamp = fixture.time - 0.27152120834216475 }
            }
            let result = await fixture.run()
            XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
            XCTAssertTrue(fixture.commands.isEmpty)
            XCTAssertGreaterThan(fixture.depthReads, 1)
            XCTAssertLessThanOrEqual(fixture.depthReads, 13)
            XCTAssertEqual(fixture.time, 10.6, accuracy: 1e-9)
            XCTAssertEqual(fixture.stages.filter { $0 == "depth_wait_started" }.count, 1)
            XCTAssertEqual(fixture.stages.filter { $0 == "depth_wait_expired" }.count, 1)
        }
    }

    func testFreshDepthAtOrAfterWaitDeadlineCannotAuthorizeProbe() async {
        for delayProvider in [false, true] {
            let fixture = ProbeFixture()
            fixture.rawDepthTimestamp = 9.7
            fixture.freshDepthAt = delayProvider ? 10.05 : 10.6
            fixture.onDepth = {
                if delayProvider, fixture.depthReads == 2 { fixture.time = 10.61 }
            }
            let result = await fixture.run()
            XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
            XCTAssertTrue(fixture.commands.isEmpty)
            XCTAssertFalse(fixture.stages.contains("depth_wait_recovered"))
        }
    }

    func testCancellationAndSourceResetDuringStartupWaitCannotMove() async {
        for reset in [false, true] {
            let fixture = ProbeFixture()
            fixture.rawDepthTimestamp = 9.7
            fixture.freshDepthAt = 10.05
            fixture.onWait = {
                if reset { fixture.generation = 2 } else { fixture.allowed = false }
            }
            let result = await fixture.run()
            XCTAssertEqual(result, reset ? .failed(.trackingLost) : .cancelled)
            XCTAssertTrue(fixture.commands.isEmpty)
        }
    }

    func testTaskCancellationDuringSuspendedDepthWaitCannotUseLateFreshCapture() async {
        let fixture = ProbeFixture()
        fixture.rawDepthTimestamp = 9.7
        fixture.freshDepthAt = 10.05
        let entered = expectation(description: "stopped depth refresh")
        let gate = FollowDiagnosticSuspension()
        fixture.waitGate = gate
        fixture.onWait = { entered.fulfill() }
        let task = Task { await fixture.run() }
        await fulfillment(of: [entered], timeout: 1)
        task.cancel()
        gate.release()
        let result = await task.value
        XCTAssertEqual(result, .cancelled)
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testStartupWaitDoesNotExtendOriginalProbeDeadline() async {
        let fixture = ProbeFixture()
        fixture.rawDepthTimestamp = 9.7
        fixture.freshDepthAt = 10.1
        fixture.onWait = {
            if fixture.commands.count == 1 { fixture.time = 13.01 }
        }
        let result = await fixture.run()
        XCTAssertEqual(result, .failed(.stalled))
        XCTAssertEqual(fixture.commands.count, 1)
    }

    func testStartupWaitDoesNotExtendOverallRecoveryDeadline() async {
        let fixture = ProbeFixture()
        fixture.rawDepthTimestamp = 9.7
        fixture.freshDepthAt = 10.1
        fixture.obstacleAfter = 0.02
        fixture.clearAfterTurn = false
        fixture.onWait = {
            if fixture.commands.count == 5 { fixture.time = 22.01 }
        }
        let result = await fixture.run()
        XCTAssertEqual(result, .failed(.noPath))
        XCTAssertEqual(fixture.commands.count, 5)
    }

    func testStaleDepthAfterFirstPulseDoesNotGetAnotherStartupWait() async {
        let fixture = ProbeFixture()
        fixture.onWait = {
            if !fixture.commands.isEmpty { fixture.rawDepthTimestamp = fixture.time - 0.27 }
        }
        let result = await fixture.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.staleRawDepth)))
        XCTAssertEqual(fixture.commands.count, 1)
        XCTAssertFalse(fixture.stages.contains("depth_wait_started"))
    }

    func testNonTransientDepthFailuresDoNotRetryOrLoseTheirCause() async {
        for reason in [DepthSafetyUnavailableReason.blindSweptVolume, .invalidCalibration, .invalidCollisionGeometry, .malformedDepth] {
            let fixture = ProbeFixture()
            fixture.unavailableReason = reason
            let result = await fixture.run()
            XCTAssertEqual(result, .failed(.depthUnavailable(reason)))
            XCTAssertTrue(fixture.commands.isEmpty)
            XCTAssertEqual(fixture.depthReads, 1)
            XCTAssertFalse(fixture.stages.contains("depth_wait_started"))
        }
    }

    func testMissingDepthWithUnknownAgeWaitsBoundedlyAndKeepsMissingReason() async {
        let fixture = ProbeFixture()
        fixture.unavailableReason = .missingRawDepth
        let result = await fixture.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.missingRawDepth)))
        XCTAssertTrue(fixture.commands.isEmpty)
        XCTAssertEqual(fixture.time, 10.6, accuracy: 1e-9)
    }

    func testClearLiveDepthProbesThenReturnsClearWithoutReverseOrTurn() async {
        let fixture = ProbeFixture()
        let result = await fixture.run()
        XCTAssertEqual(result, .clear)
        XCTAssertGreaterThan(fixture.commands.count, 0)
        XCTAssertTrue(fixture.commands.allSatisfy { $0.left > 0 && $0.left == $0.right })
        XCTAssertLessThanOrEqual(fixture.position.x, 0.05)
    }

    func testConfirmedObstacleBacktracksOnlyTravelledDistanceThenFindsClearHeading() async {
        let fixture = ProbeFixture()
        fixture.obstacleAfter = 0.02
        let result = await fixture.run()
        XCTAssertEqual(result, .turned)
        XCTAssertTrue(fixture.commands.contains { $0.left < 0 && $0.left == $0.right })
        XCTAssertTrue(fixture.commands.contains { $0.left == -$0.right })
        XCTAssertGreaterThanOrEqual(fixture.minimumX, -0.005)
        XCTAssertLessThanOrEqual(fixture.maximumX, 0.05)
    }

    func testUnknownDepthStopsWithoutAnyMovement() async {
        let fixture = ProbeFixture()
        fixture.unavailable = true
        let result = await fixture.run()
        XCTAssertEqual(result, .failed(.depthUnavailable(.missingRawDepth)))
        XCTAssertTrue(fixture.commands.isEmpty)
    }

    func testPersistentObstacleSearchIsFiniteAndDoesNotBackUpWithoutAProbe() async {
        let fixture = ProbeFixture()
        fixture.obstacleAfter = 0
        fixture.clearAfterTurn = false
        let result = await fixture.run()
        XCTAssertEqual(result, .failed(.noPath))
        XCTAssertFalse(fixture.commands.contains { $0.left < 0 && $0.left == $0.right })
        XCTAssertLessThanOrEqual(fixture.commands.count, 32)
    }

    func testBacktrackAuthorityCannotMovePastOriginOffPathAfterExpiryOrAcrossReset() {
        let authority = ReadyPathRecovery.PulseAuthority(stage: .backtrack,
            origin: .init(position: .zero, yaw: 0), generation: 1,
            maximumAlong: 0.02, expiresAt: 13, command: .init(left: -0.25, right: -0.25))
        func sample(_ point: Vec2, generation: UInt64 = 1, yaw: Double = 0) -> NavigationPoseSample {
            .init(pose: .init(position: point, yaw: yaw), frameID: .init(generation: generation, sequence: 1),
                sourceTimestamp: 10, trackingQuality: .normal)
        }
        XCTAssertTrue(authority.permits(authority.command, sample: sample(.init(0.02, 0)), now: 10))
        for point in [Vec2.zero, .init(-0.01, 0), .init(0.04, 0), .init(0.02, 0.02)] {
            XCTAssertFalse(authority.permits(authority.command, sample: sample(point), now: 10))
        }
        XCTAssertFalse(authority.permits(authority.command, sample: sample(.init(0.02, 0), generation: 2), now: 10))
        XCTAssertFalse(authority.permits(authority.command, sample: sample(.init(0.02, 0), yaw: 0.1), now: 10))
        XCTAssertFalse(authority.permits(authority.command, sample: sample(.init(0.02, 0)), now: 13))
        XCTAssertFalse(authority.permits(.init(left: -0.3, right: -0.3), sample: sample(.init(0.02, 0)), now: 10))
    }

    func testProbeOvershootStopsWithoutInventingAReturnPath() async {
        let fixture = ProbeFixture()
        fixture.step = 0.06
        let result = await fixture.run()
        XCTAssertEqual(result, .failed(.stalled))
        XCTAssertEqual(fixture.commands.count, 1)
    }

    func testLossOfAuthorityAfterProbeCannotBacktrackOrTurn() async {
        let fixture = ProbeFixture()
        fixture.cancelAfterPulse = true
        let result = await fixture.run()
        XCTAssertEqual(result, .cancelled)
        XCTAssertEqual(fixture.commands.count, 1)
    }

    func testClearDepthAfterScanOvershootCannotClaimSuccessfulRecovery() async {
        let fixture = ProbeFixture()
        fixture.obstacleAfter = 0
        fixture.turnStep = .pi / 2 + 0.01
        let result = await fixture.run()
        XCTAssertEqual(result, .failed(.noPath))
        XCTAssertEqual(fixture.commands.count, 1)
    }

    func testDepthLossAfterProbeOrDuringReturnOrScanStopsFurtherPulses() async {
        for count in [1, 3, 5] {
            let fixture = ProbeFixture()
            fixture.obstacleAfter = 0.02
            fixture.depthLossAfter = count
            let result = await fixture.run()
            XCTAssertEqual(result, .failed(.depthUnavailable(.missingRawDepth)))
            XCTAssertEqual(fixture.commands.count, count)
        }
    }
}

@MainActor
private final class ProbeFixture {
    var time = 10.0
    var sequence: UInt64 = 1
    var generation: UInt64 = 1
    var position = Vec2.zero
    var yaw = 0.0
    var commands: [WheelCommand] = []
    var minimumX = 0.0
    var maximumX = 0.0
    var obstacleAfter = Double.infinity
    var clearAfterTurn = true
    var unavailable = false
    var obstacleSeen = false
    var step = 0.01
    var cancelAfterPulse = false
    var allowed = true
    var turnStep = 0.15
    var depthLossAfter = Int.max
    var rawDepthTimestamp: Double?
    var freshDepthAt: Double?
    var depthReads = 0
    var firstPulseTime: Double?
    var onWait: (() -> Void)?
    var stages: [String] = []
    var onDepth: (() -> Void)?
    var unavailableReason: DepthSafetyUnavailableReason?
    var waitGate: FollowDiagnosticSuspension?
    var snapshot: NavigationPoseSample {
        .init(pose: .init(position: position, yaw: yaw), frameID: .init(generation: generation, sequence: sequence),
            sourceTimestamp: time - 0.001, trackingQuality: .normal)
    }
    func run() async -> ReadyPathRecovery.Outcome {
        await ReadyPathRecovery.run(start: snapshot, environment: .init(uptime: { self.time }, pose: { self.snapshot },
            depth: { command in
                self.depthReads += 1
                self.onDepth?()
                if let reason = self.unavailableReason {
                    return .unavailable(reason, sampleAge: .infinity, motionClass: .classify(command))
                }
                if let timestamp = self.rawDepthTimestamp,
                   self.freshDepthAt.map({ self.time < $0 }) ?? true {
                    return .unavailable(.staleRawDepth, sampleAge: self.time - timestamp, motionClass: .classify(command))
                }
                if self.unavailable || self.commands.count >= self.depthLossAfter {
                    return .unavailable(.missingRawDepth, sampleAge: 0, motionClass: .classify(command))
                }
                if self.position.x >= self.obstacleAfter { self.obstacleSeen = true }
                let blocked = self.obstacleSeen && !(self.clearAfterTurn && self.yaw >= 0.3)
                return .init(state: blocked ? .stop : .clear, clearance: blocked ? 0.2 : 2,
                    supportCount: blocked ? 10 : 0, sampleAge: 0.01, requiredStoppingDistance: 0.3,
                    motionClass: .classify(command), speedLimit: nil)
            }, authorized: { self.allowed }, pulse: { authority in
                guard authority.permits(authority.command, sample: self.snapshot, now: self.time) else { return .cancelled }
                self.commands.append(authority.command)
                if self.firstPulseTime == nil { self.firstPulseTime = self.time }
                if authority.command.left == authority.command.right {
                    self.position.x += authority.command.left > 0 ? self.step : -self.step
                } else { self.yaw += self.turnStep }
                self.minimumX = min(self.minimumX, self.position.x)
                self.maximumX = max(self.maximumX, self.position.x)
                self.time += 0.04
                self.sequence += 1
                if self.cancelAfterPulse { self.allowed = false }
                return .arrived
            }, wait: { duration in
                self.time += Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
                self.sequence += 1
                self.onWait?()
                if let gate = self.waitGate { await gate.suspend() }
            }, emit: { stage, _ in self.stages.append(stage) }))
    }
}
