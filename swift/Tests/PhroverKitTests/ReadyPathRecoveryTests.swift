import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class ReadyPathRecoveryTests: XCTestCase {
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
    var snapshot: NavigationPoseSample {
        .init(pose: .init(position: position, yaw: yaw), frameID: .init(generation: 1, sequence: sequence),
            sourceTimestamp: time - 0.001, trackingQuality: .normal)
    }
    func run() async -> ReadyPathRecovery.Outcome {
        await ReadyPathRecovery.run(start: snapshot, environment: .init(uptime: { self.time }, pose: { self.snapshot },
            depth: { command in
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
            }, emit: { _, _ in }))
    }
}
