import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowReturnCoordinatorTests: XCTestCase {
    func testOnlyTransientPursuitDepthFailureWaitsAndCanDrainOldDeliveries() async throws {
        for reason: NavigationFailure in [.depthUnavailable(.staleRawDepth), .depthUnavailable(.missingRawDepth), .commsLost, .commandFailed, .stalled] {
            let perception = FollowPerceptionFake(), motion = ContextualFollowMotionFake(), clock = ManualFollowClock()
            var config = FollowMeConfiguration()
            config.stationaryPauseSeconds = 0
            config.departureRangeIncrease = 0
            config.waitForUserReturn = true
            motion.legacy.suspendNavigation = true
            let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                configuration: config, eventSink: { _, _ in })
            _ = await coordinator.start()
            perception.send(frame(1, at: 0, person: true))
            await drain()
            let context = FollowMotionOperationContext(request: try XCTUnwrap(motion.contexts.first),
                controllerOperationID: 91, purpose: .followGoal, profile: nil)
            motion.sendFailure(.init(context: context, reason: reason, stopOutcome: .confirmed, source: .stream))
            await drain()
            motion.resultOverride = .init(result: .failed(reason), context: context,
                failure: .init(context: context, reason: reason, stopOutcome: .confirmed, source: .result))
            motion.legacy.releaseNavigation()
            await drain()
            if case .depthUnavailable = reason {
                guard case .waitingForReturn = coordinator.state else {
                    XCTFail("Expected depth return wait, got \(coordinator.state)"); continue
                }
                for (sequence, time) in [(2, 0.1), (3, 0.4), (4, 0.7)] {
                    clock.advance(to: time)
                    perception.send(frame(UInt64(sequence), at: time))
                    await drain()
                }
                for (sequence, time) in [(5, 0.8), (6, 0.95), (7, 1.1)] {
                    clock.advance(to: time)
                    perception.send(frame(UInt64(sequence), at: time, person: true))
                    await drain()
                }
                XCTAssertEqual(coordinator.state, .searching, "Old stream/result/confirmation must drain before restart")
            } else {
                guard case .failed = coordinator.state else {
                    XCTFail("Noneligible failure must remain terminal: \(reason)"); continue
                }
            }
            _ = await coordinator.stop()
            clock.advance(to: 100)
            await drain()
        }
    }

    private func drain() async { for _ in 0..<50 { await Task.yield() } }

    private func frame(_ sequence: UInt64, at time: Double, person: Bool = false,
                       generation: UInt64 = 1) -> FollowPerceptionEvent {
        let id = ARFrameID(generation: generation, sequence: sequence)
        let pose = Pose2D(position: .zero, yaw: 0)
        return .frame(.init(frameID: id, timestamp: time, pose: pose, depthAvailable: true,
            people: person ? [.init(frameID: id, timestamp: time, confidence: 0.9,
                boundingBox: CGRect(x: 0.4, y: 0.3, width: 0.2, height: 0.4),
                position: Vec2(0, 2), pose: pose, bodyVerified: true)] : [], trackingQuality: .normal))
    }

    private func waiting(stopFails: Bool = false) async -> (FollowMeCoordinator, FollowPerceptionFake, FollowMotionFake, ManualFollowClock) {
        let perception = FollowPerceptionFake(), motion = FollowMotionFake(), clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        config.waitForUserReturn = true
        motion.suspendRotation = true
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: { _, _ in })
        _ = await coordinator.start()
        perception.send(frame(1, at: 0, person: true))
        await drain()
        perception.send(frame(2, at: 0))
        await drain()
        for step in 1...20 {
            let time = Double(step) / 2
            if step == 20 { motion.stopError = stopFails }
            clock.advance(to: time)
            perception.send(frame(UInt64(step + 2), at: time))
            await drain()
        }
        motion.releaseRotation()
        await drain()
        if !stopFails {
            guard case .waitingForReturn = coordinator.state else {
                XCTFail("Expected return wait, got \(coordinator.state)")
                return (coordinator, perception, motion, clock)
            }
        }
        return (coordinator, perception, motion, clock)
    }

    private func sendReturn(_ perception: FollowPerceptionFake, _ clock: ManualFollowClock) async {
        for step in 1...3 {
            let time = 10 + Double(step) * 0.15
            clock.advance(to: time)
            perception.send(frame(UInt64(30 + step), at: time, person: true))
            await drain()
        }
    }

    func testReturnWaitExpiresWithoutFurtherPerception() async {
        let (coordinator, _, motion, clock) = await waiting()
        clock.advance(to: 70)
        await drain()
        XCTAssertEqual(coordinator.state, .failed("Return wait expired. Come into view and say follow me to restart."))
        XCTAssertEqual(motion.goals.count, 1)
    }

    func testStopAndInhibitionFenceSuspendedReturnReadiness() async {
        for inhibit in [false, true] {
            let (coordinator, perception, motion, clock) = await waiting()
            var release: CheckedContinuation<Void, Never>?
            motion.returnReadinessHandler = { await withCheckedContinuation { release = $0 } }
            await sendReturn(perception, clock)
            XCTAssertNotNil(release)
            if inhibit { coordinator.inhibitMotion() }
            _ = await coordinator.stop()
            release?.resume()
            await drain()
            XCTAssertEqual(coordinator.state, .stopped)
            XCTAssertEqual(motion.goals.count, 1)
            clock.advance(to: 100)
            await drain()
            XCTAssertEqual(coordinator.state, .stopped)
        }
    }

    func testFailedStopCannotEnterOrResumeReturnWait() async {
        let (coordinator, perception, motion, clock) = await waiting(stopFails: true)
        XCTAssertEqual(coordinator.state, .failed("Rover stop could not be confirmed."))
        await sendReturn(perception, clock)
        XCTAssertEqual(coordinator.state, .failed("Rover stop could not be confirmed."))
        XCTAssertEqual(motion.goals.count, 1)
    }

    func testARResetEndsReturnWait() async {
        let (coordinator, perception, motion, clock) = await waiting()
        clock.advance(to: 10.1)
        perception.send(frame(31, at: 10.1, person: true, generation: 2))
        await drain()
        XCTAssertEqual(coordinator.state, .failed("AR session reset while waiting for your return."))
        XCTAssertEqual(motion.goals.count, 1)
    }
}
