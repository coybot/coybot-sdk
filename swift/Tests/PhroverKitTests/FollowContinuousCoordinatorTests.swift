import Foundation
import RoverNav
import XCTest
@testable import PhroverKit

/// Continuous pursuit, coasting and tolerant continuation (all off unless configured).
/// Geometry: rover at the origin facing +x on the nav plane; the person starts 4 m ahead on +y.
@MainActor
final class FollowContinuousCoordinatorTests: XCTestCase {
    private struct Rig {
        let coordinator: FollowMeCoordinator
        let perception: FollowPerceptionFake
        let motion: LiveGoalFollowMotionFake
        let clock: ManualFollowClock
    }

    private func rig(_ tune: (inout FollowMeConfiguration) -> Void = { _ in }) -> Rig {
        let perception = FollowPerceptionFake()
        let motion = LiveGoalFollowMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.scanObservationSeconds = 0
        config.stationaryPauseSeconds = 0
        config.departureRangeIncrease = 0
        tune(&config)
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
                                              configuration: config, eventSink: { _, _ in })
        return Rig(coordinator: coordinator, perception: perception, motion: motion, clock: clock)
    }

    private func frame(_ sequence: UInt64, at time: TimeInterval, people: [FollowPersonObservation] = [],
                       partial: [FollowBearingObservation] = []) -> FollowPerceptionEvent {
        .frame(FollowFrameBatch(frameID: ARFrameID(generation: 1, sequence: sequence), timestamp: time,
            pose: Pose2D(position: .zero, yaw: 0), depthAvailable: true, people: people,
            trackingQuality: .normal, partialPeople: partial))
    }

    private func person(_ sequence: UInt64, at time: TimeInterval, x: Double = 0, y: Double = 4) -> FollowPersonObservation {
        FollowPersonObservation(frameID: ARFrameID(generation: 1, sequence: sequence), timestamp: time,
            confidence: 0.9, boundingBox: CGRect(x: 0.45, y: 0.4, width: 0.1, height: 0.2),
            position: Vec2(x, y), pose: Pose2D(position: .zero, yaw: 0))
    }

    private func partial(_ sequence: UInt64, at time: TimeInterval, bearing: Double = .pi / 2) -> FollowBearingObservation {
        FollowBearingObservation(frameID: ARFrameID(generation: 1, sequence: sequence), timestamp: time,
            confidence: 0.6, boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.3),
            bearing: bearing, pose: Pose2D(position: .zero, yaw: 0))
    }

    private func drain() async { for _ in 0..<30 { await Task.yield() } }

    private func send(_ r: Rig, _ sequence: UInt64, at time: TimeInterval, people: [FollowPersonObservation] = [],
                      partial: [FollowBearingObservation] = []) async {
        r.clock.advance(to: time)
        r.perception.send(frame(sequence, at: time, people: people, partial: partial))
        await drain()
    }

    /// Acquire at t=0 and feed two more steady frames so the track is seeded and learning.
    private func following(_ r: Rig) async {
        _ = await r.coordinator.start(); await drain()
        await send(r, 1, at: 0, people: [person(1, at: 0)])
        await send(r, 2, at: 0.3, people: [person(2, at: 0.3)])
        await send(r, 3, at: 0.6, people: [person(3, at: 0.6)])
        XCTAssertEqual(r.coordinator.state, .following)
    }

    // MARK: Continuous goals

    func testContinuousGoalsSteerTheRunningDriveInsteadOfStopping() async {
        let r = rig { $0.continuousGoals = true }
        _ = await r.coordinator.start(); await drain()
        await send(r, 1, at: 0, people: [person(1, at: 0)])
        XCTAssertEqual(r.motion.legacy.goals, [Vec2(0, 2.5)])
        let stopsAfterLaunch = r.motion.legacy.stops
        await send(r, 2, at: 0.3, people: [person(2, at: 0.3, y: 4.5)])
        XCTAssertEqual(r.motion.liveGoals, [Vec2(0, 3.0)], "the moved goal should reach the running drive")
        XCTAssertEqual(r.motion.legacy.goals.count, 1, "no relaunch")
        XCTAssertEqual(r.motion.legacy.stops, stopsAfterLaunch, "no stop between goal updates")
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    func testRefusedLiveUpdateFallsBackToStopAndRelaunch() async {
        let r = rig { $0.continuousGoals = true }
        r.motion.acceptsLiveGoals = false
        _ = await r.coordinator.start(); await drain()
        await send(r, 1, at: 0, people: [person(1, at: 0)])
        let stops = r.motion.legacy.stops
        await send(r, 2, at: 0.3, people: [person(2, at: 0.3, y: 4.5)])
        XCTAssertTrue(r.motion.liveGoals.isEmpty)
        XCTAssertGreaterThan(r.motion.legacy.stops, stops, "falls back to the confirmed-stop path")
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    func testDefaultConfigurationStillStopsBetweenGoals() async {
        let r = rig()
        _ = await r.coordinator.start(); await drain()
        await send(r, 1, at: 0, people: [person(1, at: 0)])
        let stops = r.motion.legacy.stops
        await send(r, 2, at: 1.0, people: [person(2, at: 1.0, y: 5.0)])
        XCTAssertTrue(r.motion.liveGoals.isEmpty, "continuous goals are opt-in")
        XCTAssertGreaterThan(r.motion.legacy.stops, stops)
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    // MARK: Coasting

    func testDroppedFramesCoastInsteadOfLosingTheTarget() async {
        let r = rig { $0.coastSeconds = 3 }
        await following(r)
        await send(r, 4, at: 0.9)                      // a frame with nobody in it
        XCTAssertEqual(r.coordinator.state, .following, "one missed frame must not start a search")
        await send(r, 5, at: 1.8)
        await send(r, 6, at: 3.0)
        XCTAssertEqual(r.coordinator.state, .following, "still inside the 3 s coast window")
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    func testCoastExpiresIntoTheExistingSearch() async {
        let r = rig { $0.coastSeconds = 3 }
        await following(r)
        await send(r, 4, at: 1.5)
        await send(r, 5, at: 3.0)
        XCTAssertEqual(r.coordinator.state, .following)
        await send(r, 6, at: 3.8)                      // 3.2 s since the last measurement at 0.6
        XCTAssertEqual(r.coordinator.state, .reacquiring)
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    func testWithoutCoastingTheFirstMissedFrameStartsRecovery() async {
        let r = rig()
        await following(r)
        await send(r, 4, at: 0.9)
        XCTAssertEqual(r.coordinator.state, .reacquiring, "default behaviour is unchanged")
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    func testTargetReappearingNearThePredictionResumesWithoutSearching() async {
        let r = rig { $0.coastSeconds = 3 }
        await following(r)
        await send(r, 4, at: 1.2)
        await send(r, 5, at: 2.4)
        await send(r, 6, at: 3.0, people: [person(6, at: 3.0, x: 0.3, y: 4.1)])
        XCTAssertEqual(r.coordinator.state, .following)
        // Once reacquired the coast clock restarts, so a fresh gap is allowed again.
        await send(r, 7, at: 5.5)
        XCTAssertEqual(r.coordinator.state, .following)
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    func testAnotherPersonFarFromThePredictionDoesNotRescueTheTrack() async {
        let r = rig { $0.coastSeconds = 3 }
        await following(r)
        await send(r, 4, at: 1.2)
        await send(r, 5, at: 2.4, people: [person(5, at: 2.4, x: 6, y: 1)])   // someone else, 6 m away
        await send(r, 6, at: 3.8, people: [person(6, at: 3.8, x: 6, y: 1)])
        XCTAssertEqual(r.coordinator.state, .reacquiring)
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    // MARK: Tolerant continuation

    func testBearingOnlyDetectionsKeepTheTrackAliveBeyondTheCoastWindow() async {
        let r = rig { $0.coastSeconds = 3; $0.tolerantContinuation = true }
        await following(r)
        var t = 0.9
        var seq: UInt64 = 4
        while t <= 5.0 {                               // 4.4 s of only a partly visible person, dead ahead
            await send(r, seq, at: t, partial: [partial(seq, at: t)])
            XCTAssertEqual(r.coordinator.state, .following, "t=\(t)")
            t += 0.3; seq += 1
        }
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    func testBearingOnlyTrackIsBoundedAndGivesUpToTheSearch() async {
        let r = rig { $0.coastSeconds = 3; $0.tolerantContinuation = true; $0.maximumBearingOnlySeconds = 4 }
        await following(r)
        var t = 0.9
        var seq: UInt64 = 4
        while t < 4.5 { await send(r, seq, at: t, partial: [partial(seq, at: t)]); t += 0.3; seq += 1 }
        XCTAssertEqual(r.coordinator.state, .following)
        await send(r, seq, at: 4.8, partial: [partial(seq, at: 4.8)])   // > 4 s with no position
        XCTAssertEqual(r.coordinator.state, .reacquiring)
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    func testBearingTowardSomeoneElseDoesNotExtendTheTrack() async {
        let r = rig { $0.coastSeconds = 3; $0.tolerantContinuation = true }
        await following(r)
        await send(r, 4, at: 1.5, partial: [partial(4, at: 1.5, bearing: 0.6)])   // 55 degrees off
        await send(r, 5, at: 3.0, partial: [partial(5, at: 3.0, bearing: 0.6)])
        await send(r, 6, at: 3.8, partial: [partial(6, at: 3.8, bearing: 0.6)])
        XCTAssertEqual(r.coordinator.state, .reacquiring)
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    func testTwoPartialCandidatesInTheGateAreNotGuessedBetween() async {
        let r = rig { $0.coastSeconds = 3; $0.tolerantContinuation = true }
        await following(r)
        for (i, t) in [1.5, 3.0, 3.8].enumerated() {
            let seq = UInt64(4 + i)
            await send(r, seq, at: t, partial: [partial(seq, at: t, bearing: .pi / 2 + 0.01),
                                                partial(seq, at: t, bearing: .pi / 2 - 0.01)])
        }
        XCTAssertEqual(r.coordinator.state, .reacquiring, "ambiguous bearings must not refresh the track")
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    func testPartialDetectionsNeverAcquireANewTarget() async {
        let r = rig { $0.coastSeconds = 3; $0.tolerantContinuation = true; $0.continuousGoals = true }
        r.motion.legacy.suspendRotation = true
        _ = await r.coordinator.start(); await drain()
        await send(r, 1, at: 0, partial: [partial(1, at: 0)])
        await send(r, 2, at: 0.3, partial: [partial(2, at: 0.3)])
        XCTAssertNotEqual(r.coordinator.state, .following)
        XCTAssertTrue(r.motion.legacy.goals.isEmpty, "no motion toward a body that never passed the body check")
        _ = await r.coordinator.stop(); r.motion.legacy.releaseRotation()
    }

    func testBearingOnlyMustStopAtConfiguredMaximum() async {
        let r = rig { $0.coastSeconds = 3; $0.tolerantContinuation = true; $0.maximumBearingOnlySeconds = 6 }
        await following(r)
        for i in 1...21 {
            let time = 0.6 + Double(i) * 0.3
            let seq = UInt64(i + 3)
            await send(r, seq, at: time, partial: [partial(seq, at: time)])
        }
        XCTAssertEqual(r.coordinator.state, .reacquiring,
            "6.3 seconds without a full position must exceed the 6-second bearing-only limit")
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

    func testFullObservationResumesAfterBearingOnlyAndCameraTurn() async {
        let r = rig { $0.coastSeconds = 3; $0.tolerantContinuation = true; $0.continuousGoals = true }
        await following(r)
        await send(r, 4, at: 0.9, partial: [partial(4, at: 0.9)])
        await send(r, 5, at: 1.2, partial: [partial(5, at: 1.2)])
        await send(r, 6, at: 1.5, partial: [partial(6, at: 1.5)])
        // The rover turns while bearing-only evidence maintains the world position.
        let pose = Pose2D(position: .zero, yaw: .pi / 4)
        let observed = FollowPersonObservation(frameID: .init(generation: 1, sequence: 7), timestamp: 1.8,
            confidence: 0.9, boundingBox: CGRect(x: 0.05, y: 0.4, width: 0.1, height: 0.2),
            position: Vec2(0, 4), pose: pose)
        r.clock.advance(to: 1.8)
        r.perception.send(.frame(.init(frameID: observed.frameID, timestamp: 1.8, pose: pose,
            depthAvailable: true, people: [observed], trackingQuality: .normal)))
        await drain()
        // Beyond the old bearing clock, but still inside a fresh coast window from t=1.8.
        await send(r, 8, at: 4.6)
        XCTAssertEqual(r.coordinator.state, .following,
            "The full observation should refresh the track despite the pre-occlusion bounding box")
        _ = await r.coordinator.stop(); r.motion.legacy.releaseNavigation()
    }

}
