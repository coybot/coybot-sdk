import XCTest
@testable import RoverNav

final class FollowPolicyTests: XCTestCase {
    private let t0 = Date(timeIntervalSince1970: 1000)

    private func track(_ position: Vec2,
                       velocity: Vec2 = .zero,
                       seenAt: Date? = nil) -> TargetTrack {
        TargetTrack(id: 1, position: position, velocity: velocity, lastSeenAt: seenAt ?? t0)
    }

    private func facing(_ position: Vec2, from pose: Vec2) -> Pose2D {
        let d = position - pose
        return Pose2D(position: pose, yaw: atan2(d.y, d.x))
    }

    // MARK: - Stand-off geometry

    func testAdvancesToStandoffShortOfTarget() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5))
        let pose = facing(Vec2(6, 0), from: .zero)

        let out = policy.step(pose: pose, track: track(Vec2(6, 0)), now: t0)

        XCTAssertEqual(out.rangeError, 3.5, accuracy: 1e-9)
        guard case .advance(let goal, let speed) = out.motion else {
            return XCTFail("expected advance, got \(out.motion)")
        }
        // Goal sits on the line to the target, one stand-off short of it.
        XCTAssertEqual(goal.x, 3.5, accuracy: 1e-9)
        XCTAssertEqual(goal.y, 0, accuracy: 1e-9)
        XCTAssertGreaterThan(speed, 0)
        XCTAssertLessThanOrEqual(speed, 0.35)
    }

    func testSpeedIsCappedAtClassLimit() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5, maxSpeed: 0.35))
        let out = policy.step(pose: facing(Vec2(50, 0), from: .zero),
                              track: track(Vec2(50, 0)),
                              now: t0)
        guard case .advance(_, let speed) = out.motion else {
            return XCTFail("expected advance, got \(out.motion)")
        }
        XCTAssertEqual(speed, 0.35, accuracy: 1e-9)
    }

    func testGoalLeadsAMovingTarget() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5))
        // Target 6 m ahead walking away at 0.6 m/s; leadTime 0.5 s -> predicted 6.3 m.
        let out = policy.step(pose: facing(Vec2(6, 0), from: .zero),
                              track: track(Vec2(6, 0), velocity: Vec2(0.6, 0)),
                              now: t0)
        XCTAssertEqual(out.predictedTarget.x, 6.3, accuracy: 1e-9)
    }

    // MARK: - Deadband / hysteresis

    func testHoldsInsideTheBandAndDoesNotOscillate() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5))
        let pose = facing(Vec2(10, 0), from: .zero)

        // Close to exactly the stand-off: latch into hold.
        XCTAssertEqual(policy.step(pose: pose, track: track(Vec2(2.6, 0)), now: t0).motion, .hold)
        XCTAssertTrue(policy.isInBand)

        // Drifting within deadbandExit (0.60 m) must NOT re-trigger translation — this is
        // the check that the vehicle does not creep forward/back every tick.
        for x in [2.9, 3.0, 2.2, 2.05, 2.95] {
            let out = policy.step(pose: pose, track: track(Vec2(x, 0)), now: t0)
            XCTAssertEqual(out.motion, .hold, "range \(x) should stay held")
        }

        // Past the exit threshold the latch drops and it moves again.
        let out = policy.step(pose: pose, track: track(Vec2(3.2, 0)), now: t0)
        XCTAssertFalse(policy.isInBand)
        guard case .advance = out.motion else {
            return XCTFail("expected advance once outside the band, got \(out.motion)")
        }
    }

    func testEnterThresholdIsTighterThanExitThreshold() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5))
        let pose = facing(Vec2(10, 0), from: .zero)

        // 0.4 m of error is inside exit (0.60) but outside enter (0.25): approaching from
        // outside, it must keep closing rather than latching early.
        let out = policy.step(pose: pose, track: track(Vec2(2.9, 0)), now: t0)
        guard case .advance = out.motion else {
            return XCTFail("expected advance, got \(out.motion)")
        }
        XCTAssertFalse(policy.isInBand)
    }

    func testResetDropsTheLatch() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5))
        let pose = facing(Vec2(10, 0), from: .zero)
        _ = policy.step(pose: pose, track: track(Vec2(2.6, 0)), now: t0)
        XCTAssertTrue(policy.isInBand)
        policy.reset()
        XCTAssertFalse(policy.isInBand)
    }

    // MARK: - Bearing

    func testTurnsInPlaceWhenBadlyOffBearing() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5))
        // Target directly to the left (+y), rover facing +x: 90 deg of bearing error.
        let out = policy.step(pose: Pose2D(position: .zero, yaw: 0),
                              track: track(Vec2(0, 6)),
                              now: t0)
        guard case .turnInPlace(let yawError) = out.motion else {
            return XCTFail("expected turnInPlace, got \(out.motion)")
        }
        XCTAssertEqual(yawError, .pi / 2, accuracy: 1e-9)
    }

    func testHeldTargetStillGetsCentred() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5))
        // In the band, but 30 deg off boresight — keep the camera on it without translating.
        let target = Vec2(2.5 * cos(.pi / 6), 2.5 * sin(.pi / 6))
        let out = policy.step(pose: Pose2D(position: .zero, yaw: 0),
                              track: track(target),
                              now: t0)
        guard case .turnInPlace(let yawError) = out.motion else {
            return XCTFail("expected turnInPlace, got \(out.motion)")
        }
        XCTAssertEqual(yawError, .pi / 6, accuracy: 1e-9)
    }

    // MARK: - Target closing inside the stand-off

    func testGroundVehicleHoldsRatherThanReversing() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5))
        let out = policy.step(pose: facing(Vec2(1, 0), from: .zero),
                              track: track(Vec2(1, 0)),
                              now: t0)
        XCTAssertLessThan(out.rangeError, 0)
        XCTAssertEqual(out.motion, .hold, "a rover with no rear sensing must not back away")
    }

    func testMultirotorRestoresStandoff() {
        var policy = FollowPolicy(params: .multirotor(standoff: 5))
        let out = policy.step(pose: facing(Vec2(1, 0), from: .zero),
                              track: track(Vec2(1, 0)),
                              now: t0)
        guard case .retreat(let goal, _) = out.motion else {
            return XCTFail("expected retreat, got \(out.motion)")
        }
        // Retreat goal is a full stand-off on the far side of the vehicle from the target.
        XCTAssertEqual(goal.x, -4, accuracy: 1e-9)
    }

    // MARK: - Fixed-wing

    func testFixedWingAlwaysOrbitsThePredictedPosition() {
        var policy = FollowPolicy(params: .fixedWing(orbitRadius: 42))
        let pose = Pose2D(position: .zero, yaw: 0)

        // Far away, right on top of it, and stale — all three still orbit. A fixed-wing
        // has no hold state; min airspeed is 12 m/s.
        for (target, now) in [(Vec2(500, 0), t0), (Vec2(1, 0), t0), (Vec2(500, 0), t0.addingTimeInterval(30))] {
            let out = policy.step(pose: pose,
                                  track: track(target, velocity: Vec2(0, 1)),
                                  now: now)
            guard case .orbit(let center, let radius) = out.motion else {
                return XCTFail("expected orbit, got \(out.motion)")
            }
            XCTAssertEqual(radius, 42, accuracy: 1e-9)
            // Centre leads the target along its own velocity.
            XCTAssertGreaterThan(center.y, target.y)
        }
    }

    // MARK: - Staleness

    func testStaleTrackIsFlaggedButStillProducesMotion() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5))
        let out = policy.step(pose: facing(Vec2(6, 0), from: .zero),
                              track: track(Vec2(6, 0), seenAt: t0),
                              now: t0.addingTimeInterval(1.5))
        XCTAssertTrue(out.isStale)
        guard case .advance = out.motion else {
            return XCTFail("expected advance, got \(out.motion)")
        }
    }

    func testFreshTrackIsNotStale() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5))
        let out = policy.step(pose: facing(Vec2(6, 0), from: .zero),
                              track: track(Vec2(6, 0), seenAt: t0),
                              now: t0.addingTimeInterval(0.2))
        XCTAssertFalse(out.isStale)
    }

    // MARK: - Closed loop

    /// Integrates the policy against a target walking away in a straight line. Verifies the
    /// band is held and — the point of the hysteresis — that the vehicle is not switching
    /// between moving and stopping every tick.
    func testFollowsASlowWalkerWithoutChattering() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5))
        var pose = Pose2D(position: .zero, yaw: 0)
        var targetX = 2.5
        let dt = 0.1
        let walkSpeed = 0.25  // m/s — below the 0.35 m/s rover cap

        var transitions = 0
        var wasMoving = false
        var maxError = 0.0

        for i in 0..<600 {
            targetX += walkSpeed * dt
            let now = t0.addingTimeInterval(Double(i) * dt)
            let out = policy.step(pose: pose,
                                  track: track(Vec2(targetX, 0),
                                               velocity: Vec2(walkSpeed, 0),
                                               seenAt: now),
                                  now: now)
            var moving = false
            if case .advance(_, let speed) = out.motion {
                moving = true
                pose.position.x += min(speed, 0.35) * dt
            }
            if moving != wasMoving { transitions += 1 }
            wasMoving = moving
            // Ignore the first second while the loop settles from a standing start.
            if i > 10 { maxError = max(maxError, abs(targetX - pose.position.x - 2.5)) }
        }

        XCTAssertLessThan(maxError, 0.75, "stand-off drifted out of band (max error \(maxError) m)")
        XCTAssertLessThan(transitions, 30, "policy chattered between move and hold \(transitions) times in 60 s")
    }

    /// The physical ceiling, asserted so it is a documented property and not a surprise:
    /// at normal walking pace the rover cannot hold station, it falls steadily behind.
    func testFallsBehindANormalWalkingPace() {
        var policy = FollowPolicy(params: .ground(standoff: 2.5, maxSpeed: 0.35))
        var pose = Pose2D(position: .zero, yaw: 0)
        var targetX = 2.5
        let dt = 0.1
        let walkSpeed = 1.3

        for i in 0..<300 {
            targetX += walkSpeed * dt
            let now = t0.addingTimeInterval(Double(i) * dt)
            let out = policy.step(pose: pose,
                                  track: track(Vec2(targetX, 0),
                                               velocity: Vec2(walkSpeed, 0),
                                               seenAt: now),
                                  now: now)
            if case .advance(_, let speed) = out.motion {
                pose.position.x += min(speed, 0.35) * dt
            }
        }

        XCTAssertGreaterThan(targetX - pose.position.x, 25,
                             "expected the rover to fall behind a 1.3 m/s walker")
    }
}
