import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowPursuitContinuityTests: XCTestCase {
    private func drain() async { for _ in 0..<60 { await Task.yield() } }
    private func frame(_ sequence: UInt64, time: Double, range: Double, angle: Double = 0) -> FollowPerceptionEvent {
        let id = ARFrameID(generation: 1, sequence: sequence)
        let pose = Pose2D(position: .zero, yaw: 0)
        return .frame(.init(frameID: id, timestamp: time, pose: pose, depthAvailable: true,
            people: [.init(frameID: id, timestamp: time, confidence: 0.99,
                boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6),
                position: Vec2(range * cos(angle), range * sin(angle)), pose: pose, bodyVerified: true)], trackingQuality: .normal))
    }

    func testStalePerceptionPausesAndResumesOriginalGoalRequest() async {
        let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
        motion.legacy.suspendNavigation = true
        motion.preservesFollowGoalOwner = true
        var pauses = 0, resumes = 0, updates: [Vec2] = []
        var boundary = 0.0
        motion.pauseHandler = { pauses += 1; boundary = clock.now; return true }
        motion.canResumeHandler = { $0 > boundary }
        motion.resumeHandler = { timestamp in
            guard timestamp > boundary else { return false }
            resumes += 1; return true
        }
        motion.goalUpdateHandler = { updates.append($0); return true }
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0; config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, range: 2.5))
        await drain()
        let stops = motion.legacy.stops
        XCTAssertEqual(motion.legacy.goals.count, 1)
        clock.advance(to: 0.51)
        await drain()
        XCTAssertEqual(pauses, 1)
        XCTAssertEqual(motion.legacy.stops, stops, "The existing driver owns the pause stop")
        clock.advance(to: 0.6)
        perception.send(frame(2, time: 0.49, range: 2.6))
        await drain()
        XCTAssertEqual(resumes, 0, "A pre-pause capture cannot resume motion")
        clock.advance(to: 0.8)
        perception.send(frame(3, time: 0.8, range: 2.9))
        await drain()
        XCTAssertEqual(resumes, 1)
        XCTAssertFalse(updates.isEmpty)
        XCTAssertEqual(motion.legacy.goals.count, 1)
        XCTAssertEqual(motion.contexts.count, 1, "No new owner or recovery budget")
        XCTAssertEqual(motion.legacy.stops, stops)
        // A terminating planned owner may refuse an update. Wait for its result
        // rather than replacing it and hiding a terminal depth failure.
        motion.goalUpdateHandler = { _ in false }
        clock.advance(to: 1.1)
        perception.send(frame(4, time: 1.1, range: 3.3))
        await drain()
        XCTAssertEqual(motion.legacy.goals.count, 1)
        _ = await coordinator.stop()
        motion.legacy.releaseNavigation()
        clock.advance(to: 100)
        await drain()
    }

    func testPrePauseFrameCannotRenewOutageDeadline() async {
        let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
        motion.legacy.suspendNavigation = true
        motion.preservesFollowGoalOwner = true
        var boundary = 0.0
        motion.pauseHandler = { boundary = clock.now; return true }
        motion.canResumeHandler = { $0 > boundary }
        motion.resumeHandler = { _ in XCTFail("No eligible post-pause frame"); return false }
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0; config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, range: 2.2))
        await drain()
        clock.advance(to: 0.51)
        await drain()
        clock.advance(to: 0.6)
        perception.send(frame(2, time: 0.49, range: 2.2))
        await drain()
        clock.advance(to: 1.01)
        await drain()
        clock.advance(to: 2.52)
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Camera frame stale or timestamp invalid. Check camera delivery and inference latency."))
        motion.legacy.releaseNavigation()
        clock.advance(to: 100)
        await drain()
    }

    func testPostPauseDirectionChangeNeverResumesOldForwardRoute() async {
        let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
        motion.legacy.suspendNavigation = true
        motion.legacy.suspendAlignment = true
        motion.legacy.useSourceClock(clock)
        motion.preservesFollowGoalOwner = true
        var boundary = 0.0, resumes = 0
        motion.pauseHandler = { boundary = clock.now; return true }
        motion.canResumeHandler = { $0 > boundary }
        motion.resumeHandler = { _ in resumes += 1; return true }
        motion.goalUpdateHandler = { _ in XCTFail("Turn first; do not release the old forward drive"); return true }
        var config = FollowMeConfiguration.continuous
        config.stationaryPauseSeconds = 0; config.departureRangeIncrease = 0
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, time: 0, range: 2.2))
        await drain()
        clock.advance(to: 0.51)
        await drain()
        clock.advance(to: 0.8)
        perception.send(frame(2, time: 0.8, range: 2.2, angle: 0.32))
        await drain()
        XCTAssertEqual(coordinator.state, .aligning)
        XCTAssertEqual(resumes, 0)
        XCTAssertEqual(motion.legacy.goals.count, 1)
        _ = await coordinator.stop()
        motion.legacy.releaseNavigation()
        motion.legacy.releaseAlignment()
        clock.advance(to: 100)
        await drain()
    }
}
