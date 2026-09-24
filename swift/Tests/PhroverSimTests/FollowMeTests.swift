import XCTest
import RoverNav
import PhroverKit

/// Follow mode against the Godot Depot sim's walking person actors. Exercises the real
/// `FollowController` (tracker, `FollowPolicy`, safety gate) through sim adapters — no
/// brain in the loop, same reasoning as `PersonCrossingMotionTests`: what is under test is
/// the control law and the association, not how a mission decided to invoke it.
///
/// Requires a Godot Depot sim already running and reachable — same skip-if-unreachable
/// convention as the other sim suites.
@MainActor
final class FollowMeTests: XCTestCase {
    private struct Rig {
        let link: GodotLink
        let perception: GodotPerception
        let drive: GodotDrive
        let controller: FollowController
    }

    private func makeLink() throws -> GodotLink {
        do {
            return try GodotLink()
        } catch {
            throw XCTSkip("Godot Depot sim not reachable at "
                + "\(ProcessInfo.processInfo.environment["GODOT_HOST"] ?? "127.0.0.1"):"
                + "\(ProcessInfo.processInfo.environment["GODOT_PORT"] ?? "9999")"
                + " — launch it first (eco/rover/sim/godot_launcher.py::launch_depot).")
        }
    }

    /// Rover spawned in the hallway facing north, ready to follow someone walking away.
    private func makeRig(_ link: GodotLink, rid: String, startY: Double = 1.5) -> Rig {
        link.call(["op": "reset", "seed": 7])
        link.call(["op": "phrover_spawn", "id": rid, "p": [0.0, startY], "yaw": Double.pi / 2])
        let perception = GodotPerception(link: link, rid: rid)
        let drive = GodotDrive(link: link, rid: rid)
        let controller = FollowController(
            perception: perception,
            drive: drive,
            // Stand-off deliberately above the depot person governor's own release
            // distance (PERSON_SAFE_DIST, 2.4 m) so following does not sit permanently
            // inside a guard the brain cannot see or override.
            policy: .ground(standoff: 2.5, maxSpeed: 0.45))
        return Rig(link: link, perception: perception, drive: drive, controller: controller)
    }

    private func personPosition(_ link: GodotLink, rid: String) -> Vec2? {
        let result = link.call(["op": "phrover_detect", "id": rid])
        guard let objects = result["objects"] as? [[String: Any]] else { return nil }
        for object in objects where (object["label"] as? String) == "person" {
            if let world = godotDoubleArray(object["world"]), world.count == 2 {
                return Vec2(world[0], world[1])
            }
        }
        return nil
    }

    // MARK: - Holding station

    func testHoldsStandoffBehindAWalkingPerson() async throws {
        let link = try makeLink()
        let rid = "follow-1"
        let rig = makeRig(link, rid: rid)

        // 0.25 m/s, not the actor's default 0.8: the sim phrover's own MAX_V is 0.5 m/s,
        // so at walking pace the rover simply cannot hold station — a real ceiling
        // (FollowPolicyTests.testFallsBehindANormalWalkingPace asserts it directly), not
        // something a follow controller can tune its way out of.
        link.call(["op": "inject", "name": "person_walk",
                   "params": ["on": true, "speed": 0.25,
                              "waypoints": [[0.0, 4.0], [0.0, 9.0]]]])
        try? await Task.sleep(for: .seconds(0.5))

        let startPose = rig.perception.pose
        rig.controller.follow(TargetSpec(query: "follow the person"))

        var samples: [Double] = []
        let deadline = Date().addingTimeInterval(18)
        while Date() < deadline {
            try? await Task.sleep(for: .seconds(0.25))
            if case .ended = rig.controller.state { break }
            guard let pose = rig.perception.pose, let person = personPosition(link, rid: rid) else {
                continue
            }
            samples.append(pose.position.distance(to: person))
        }
        rig.controller.cancel()

        if case .ended(let reason) = rig.controller.state {
            XCTFail("follow ended early: \(reason)")
        }
        XCTAssertGreaterThan(samples.count, 20, "not enough usable samples to judge the follow")

        // Holding a stand-off by standing still while the target walks away would show up
        // as an out-of-band distance, but assert the travel directly so the test cannot
        // pass on a rover that never moved.
        let travelled = (rig.perception.pose?.position.y ?? 0) - (startPose?.position.y ?? 0)
        XCTAssertGreaterThan(travelled, 2.5,
                             "rover only advanced \(travelled) m while the person walked away")

        // Settle-in is part of the run, not an assertion target: the rover starts 2.5 m
        // back and stationary. Judge the steady state.
        let steady = Array(samples.dropFirst(8))
        let inBand = steady.filter { $0 >= 1.8 && $0 <= 3.6 }
        let fraction = Double(inBand.count) / Double(steady.count)
        XCTAssertGreaterThan(fraction, 0.8,
                             "held the stand-off only \(Int(fraction * 100))% of the time "
                             + "(distances \(steady.map { String(format: "%.2f", $0) }))")

        let events = (link.call(["op": "get_events", "since": 0.0])["events"] as? [[String: Any]]) ?? []
        let collisions = events.filter {
            ($0["kind"] as? String) == "collision"
                && (($0["data"] as? [String: Any])?["with"] as? String) == "person"
        }
        XCTAssertTrue(collisions.isEmpty, "rover hit the person it was following: \(collisions)")
        XCTAssertEqual(rig.controller.unverifiedReacquisitions, 0,
                       "lock was re-attached to an unvouched candidate during a clean follow")
    }

    // MARK: - Identity

    /// Two people walk up the hallway abreast, one metre apart, both reported by the
    /// detector as a bare `person` — nothing in the image distinguishes them. The lock is
    /// seeded on the left-hand one and must stay there.
    func testDoesNotTransferTheLockToASecondPerson() async throws {
        let link = try makeLink()
        let rid = "follow-2"
        let rig = makeRig(link, rid: rid)

        link.call(["op": "inject", "name": "person_walk",
                   "params": ["on": true, "speed": 0.25,
                              "waypoints": [[-0.5, 4.0], [-0.5, 9.0]]]])
        link.call(["op": "inject", "name": "person_walk",
                   "params": ["on": true, "person": "DecoyActor", "speed": 0.25,
                              "waypoints": [[0.5, 4.0], [0.5, 9.0]]]])
        try? await Task.sleep(for: .seconds(0.5))

        // Seed the lock on the left-hand walker — the equivalent of the operator pointing,
        // or a brain grounding "the one on the left" into a world point.
        rig.controller.follow(TargetSpec(query: "person"), seed: Vec2(-0.5, 4.0))

        var maxDriftFromLeftLane = 0.0
        let deadline = Date().addingTimeInterval(15)
        while Date() < deadline {
            try? await Task.sleep(for: .seconds(0.25))
            if case .ended = rig.controller.state { break }
            guard let track = rig.controller.track else { continue }
            // The two lanes are 1 m apart in x; a lock that transferred shows up as the
            // tracked x crossing toward +0.5.
            maxDriftFromLeftLane = max(maxDriftFromLeftLane, abs(track.position.x - (-0.5)))
        }
        rig.controller.cancel()

        XCTAssertLessThan(maxDriftFromLeftLane, 0.5,
                          "tracked position drifted \(maxDriftFromLeftLane) m toward the decoy's lane")
        XCTAssertEqual(rig.controller.unverifiedReacquisitions, 0,
                       "lock re-attached without the association gate vouching for it")
    }

    // MARK: - Loss

    func testGivesUpWhenTheTargetIsNeverSeen() async throws {
        let link = try makeLink()
        let rid = "follow-3"
        let rig = makeRig(link, rid: rid)

        // Nobody is scenario-present at all: no person_walk inject.
        rig.controller.follow(TargetSpec(query: "person"))

        let deadline = Date().addingTimeInterval(Double(RoverConfig.followAcquireTimeout) + 6)
        while Date() < deadline {
            try? await Task.sleep(for: .seconds(0.25))
            if case .ended = rig.controller.state { break }
        }

        guard case .ended(let message) = rig.controller.state else {
            rig.controller.cancel()
            return XCTFail("expected the follow to give up, state was \(rig.controller.state)")
        }
        XCTAssertFalse(message.isEmpty, "the operator needs to be told why it stopped")
    }
}
