import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowObstacleDetourCoordinatorTests: XCTestCase {
    private func drain() async { for _ in 0..<100 { await Task.yield() } }

    private func startDetour(_ f: Fixture) async {
        _ = await f.coordinator.start()
        f.frame(at: 0)
        await drain()
        f.frame(at: 0.1)
        await drain()
        f.request = f.context?.begin(at: f.source)
        XCTAssertNotNil(f.request)
    }

    @MainActor
    private final class Fixture {
        let clock = ManualFollowClock()
        let motion = ContextualFollowMotionFake()
        let perception = FollowPerceptionFake()
        var context: FollowObstacleDetourContext?
        var request: FollowObstacleDetourRequest?
        var sequence: UInt64 = 1
        var source: NavigationPoseSample {
            .init(pose: .init(position: .zero, yaw: 0), frameID: .init(generation: 1, sequence: sequence),
                sourceTimestamp: clock.now, trackingQuality: .normal)
        }
        lazy var coordinator = makeCoordinator()

        private func makeCoordinator() -> FollowMeCoordinator {
            motion.legacy.useSourceClock(clock)
            motion.legacy.suspendNavigation = true
            motion.legacy.suspendRotation = true
            motion.performHandler = { [self] action, requestContext in
                if requestContext.purpose == .followGoal {
                    context = FollowObstacleDetourScope.context
                    let result = await motion.legacy.performContextual(action, context: requestContext)
                    let operation = FollowMotionOperationContext(request: requestContext, controllerOperationID: 42,
                        purpose: .followGoal, profile: nil)
                    return .init(result: .reobservationRequired, context: operation, failure: result.failure,
                        stopOutcome: .confirmed, turnStopFence: .init(identity: UUID(), operationGeneration: 1,
                            context: operation, acknowledgementUptime: clock.now, sourceGeneration: 1,
                            highestSequence: sequence, highestSourceTimestamp: clock.now))
                }
                return await motion.legacy.performContextual(action, context: requestContext)
            }
            var config = FollowMeConfiguration.continuous
            config.stationaryPauseSeconds = 0
            config.departureRangeIncrease = 0
            config.planningMode = .reactiveDepth
            config.allowsObstacleDetour = true
            return .init(perception: perception, motion: motion, clock: clock,
                configuration: config, eventSink: { _, _ in })
        }

        func frame(at time: Double, depth: Bool = true) {
            clock.advance(to: time)
            sequence += 1
            let id = ARFrameID(generation: 1, sequence: sequence)
            let pose = Pose2D(position: .zero, yaw: 0)
            perception.send(.frame(.init(frameID: id, timestamp: time, pose: pose, depthAvailable: depth,
                people: [.init(frameID: id, timestamp: time, confidence: 0.99,
                    boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6),
                    position: Vec2(2, 0), pose: pose, bodyVerified: true)], trackingQuality: .normal)))
        }

        func cleanup() async {
            _ = await coordinator.stop()
            motion.legacy.releaseNavigation()
            motion.legacy.releaseRotation()
            motion.performHandler = nil
            clock.advance(to: 100)
        }
    }

    func testAvoidanceOwnsMotionAndCannotBeRenewedByNewTargetFrames() async {
        let f = Fixture()
        await startDetour(f)
        XCTAssertEqual(f.coordinator.state, .avoidingObstacle)
        XCTAssertEqual(f.request?.deadline, 12.1)
        XCTAssertEqual(f.request?.preferredSide, -1)
        f.frame(at: 0.2)
        await drain()
        XCTAssertEqual(f.motion.legacy.goals.count, 1)
        XCTAssertNil(f.context?.begin(at: f.source))
        await f.cleanup()
    }

    func testCompletionRequiresPostStopReacquisitionWithinOriginalDeadline() async {
        let f = Fixture()
        await startDetour(f)
        for step in 1...58 {
            f.frame(at: 0.1 + Double(step) * 0.2)
            await drain()
        }
        XCTAssertEqual(f.coordinator.state, .avoidingObstacle)
        f.motion.legacy.releaseNavigation()
        await drain()
        XCTAssertEqual(f.coordinator.state, .reacquiring)
        XCTAssertEqual(f.motion.legacy.goals.count, 1, "Completion cannot resume the old goal")
        // No post-stop observation was delivered; the original timer remains authoritative.
        f.clock.advance(to: 12.1)
        await drain()
        XCTAssertNotEqual(f.coordinator.state, .following)
        XCTAssertNotEqual(f.coordinator.state, .reacquiring, "Reacquisition cannot gain another ten seconds")
        XCTAssertEqual(f.motion.legacy.goals.count, 1)
        await f.cleanup()
    }

    func testStopFencesLateSuccessfulDetourCompletion() async {
        let f = Fixture()
        await startDetour(f)
        _ = await f.coordinator.stop()
        f.motion.legacy.releaseNavigation()
        f.frame(at: 0.4)
        await drain()
        XCTAssertEqual(f.coordinator.state, .stopped)
        XCTAssertEqual(f.motion.legacy.goals.count, 1)
        XCTAssertTrue(f.motion.legacy.rotations.isEmpty)
        await f.cleanup()
    }

    func testFreshPostStopTargetRestoresFollowingWithoutSecondDetour() async {
        let f = Fixture()
        await startDetour(f)
        f.motion.legacy.releaseNavigation()
        await drain()
        XCTAssertEqual(f.coordinator.state, .reacquiring)
        for time in [0.5, 0.9, 1.3, 1.7] {
            f.frame(at: time)
            await drain()
        }
        XCTAssertEqual(f.coordinator.state, .following)
        XCTAssertEqual(f.motion.legacy.goals.count, 2)
        XCTAssertNil(f.context?.begin(at: f.source))
        await f.cleanup()
    }

    func testDepthLossTerminatesAvoidance() async {
        let f = Fixture()
        await startDetour(f)
        f.frame(at: 0.2, depth: false)
        await drain()
        XCTAssertFalse(f.coordinator.isActive)
        XCTAssertEqual(f.motion.legacy.goals.count, 1)
        await f.cleanup()
    }
}
