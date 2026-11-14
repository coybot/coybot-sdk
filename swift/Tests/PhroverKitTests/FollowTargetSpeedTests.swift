import XCTest
import RoverNav
@testable import PhroverKit

final class FollowTargetSpeedTests: XCTestCase {
    private func sample(range: Double, velocity: Double, time: Double = 10, generation: UInt64 = 1) -> FollowTargetSpeedSample {
        .init(frameID: .init(generation: generation, sequence: 1), timestamp: time,
            position: Vec2(range, 0), velocity: Vec2(velocity, 0), standOff: 1.5, minimumDistance: 1.25)
    }
    private var pose: NavigationPoseSample {
        .init(pose: .init(position: .zero, yaw: 0), frameID: .init(generation: 1, sequence: 1),
            sourceTimestamp: 10, trackingQuality: .normal)
    }

    func testMovingTargetGetsFeedForwardAndBypassesCatchupRamp() throws {
        let moving = try XCTUnwrap(FollowTargetSpeed.evaluate(sample(range: 2.2, velocity: 0.2), pose: pose, now: 10, maximum: 0.35))
        XCTAssertEqual(moving.maximumWheelSpeed, 0.35, accuracy: 1e-9)
        XCTAssertTrue(moving.bypassesStartupRamp)
        let nearer = try XCTUnwrap(FollowTargetSpeed.evaluate(sample(range: 1.9, velocity: 0.2), pose: pose, now: 10, maximum: 0.35))
        let stopped = try XCTUnwrap(FollowTargetSpeed.evaluate(sample(range: 1.9, velocity: 0), pose: pose, now: 10, maximum: 0.35))
        XCTAssertGreaterThan(nearer.maximumWheelSpeed, stopped.maximumWheelSpeed)
        XCTAssertFalse(nearer.bypassesStartupRamp)
        XCTAssertFalse(stopped.bypassesStartupRamp)
    }

    func testSpeedFallsWithClosingGapAndStopsAtSafeDistance() throws {
        let ranges = [2.5, 1.9, 1.7, 1.6, 1.56, 1.55, 1.2]
        var previous = Double.infinity
        for range in ranges {
            let speed = try XCTUnwrap(FollowTargetSpeed.evaluate(sample(range: range, velocity: 0), pose: pose, now: 10, maximum: 0.35))
            XCTAssertLessThanOrEqual(speed.maximumWheelSpeed, previous)
            XCTAssertGreaterThanOrEqual(speed.maximumWheelSpeed, 0)
            if range <= 1.55 { XCTAssertEqual(speed.maximumWheelSpeed, 0, accuracy: 1e-9) }
            previous = speed.maximumWheelSpeed
        }
    }

    func testStaleFutureInvalidAndDifferentGenerationTargetsCannotRaiseSpeed() {
        for target in [sample(range: 3, velocity: 0.3, time: 9.49), sample(range: 3, velocity: 0.3, time: 9.5), sample(range: 3, velocity: 0.3, time: 10.1),
                       sample(range: 3, velocity: .nan), sample(range: 3, velocity: 0.3, generation: 2)] {
            XCTAssertNil(FollowTargetSpeed.evaluate(target, pose: pose, now: 10, maximum: 0.35))
        }
    }

    func testFinalCrawlReachesHoldingBoundaryInFiniteTime() throws {
        var position = 0.0
        for step in 0..<30 {
            let now = 10 + Double(step) * 0.1
            let pose = NavigationPoseSample(pose: .init(position: Vec2(position, 0), yaw: 0),
                frameID: .init(generation: 1, sequence: UInt64(step + 1)), sourceTimestamp: now, trackingQuality: .normal)
            let target = sample(range: 1.59, velocity: 0, time: now)
            let policy = try XCTUnwrap(FollowTargetSpeed.evaluate(target, pose: pose, now: now, maximum: 0.35))
            if policy.range <= policy.stopDistance { return }
            position += policy.maximumWheelSpeed * 0.1
        }
        XCTFail("Near-target braking must not approach hold asymptotically and trigger the progress watchdog")
    }
}
