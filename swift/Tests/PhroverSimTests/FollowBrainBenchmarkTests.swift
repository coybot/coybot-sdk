import XCTest
import CoreGraphics
import RoverNav
@testable import PhroverKit

/// "Follow me" with each candidate brain in the loop (see `BenchBrain`). Two tiers:
///
/// - `testDecisions` — the brain alone, against hand-built mission contexts: does it pick
///   follow when asked to follow (and only then), does its description ground on the right
///   person, does it keep following on a review tick, and how long does it take. No sim.
/// - `testClosedLoop*` — the real `MissionAgent` + `FollowController` driving the Godot
///   Depot sim's walkers, with the brain deciding. Needs the sim running.
///
/// A benchmark, not a gate: results are `BENCH {json}` lines that
/// eco/rover/sim/run_follow_brain_bench.py collects and tabulates. Nothing here asserts
/// on model quality.
@MainActor
final class FollowBrainBenchmarkTests: XCTestCase {
    private var env: [String: String] { ProcessInfo.processInfo.environment }

    private func brainOrSkip() throws -> (name: String, brain: OnDeviceBrain) {
        do { return try BenchBrain.make() } catch let skip as BenchSkip {
            throw XCTSkip(skip.description)
        }
    }

    // MARK: - Tier A: decisions

    private enum Grade: String { case pass, ok, fail }
    private typealias Grader = (RoverDecision) -> (Grade, String)

    private struct DecisionCase {
        let id: String
        let group: String
        /// What the operator just said; `nil` for a follow review tick.
        let utterance: String?
        /// The utterance that started the mission, for review ticks.
        var priorUtterance: String?
        let visible: [(String, Float)]
        var followState: String?
        var recentActions: [String] = []
        let grade: Grader
    }

    /// Which visible label a description would lock onto — the same most-specific rule
    /// `TargetTracker` uses — so a query is graded by where it actually leads, not by its
    /// wording.
    static func describe(_ d: RoverDecision) -> String {
        switch d {
        case .follow(.visualQuery(let q)): "follow(\(q))"
        case .navigate(.visualQuery(let q)): "navigate(\(q))"
        case .say(let t): "say(\(t))"
        case .ask(let t): "ask(\(t))"
        case .lookAround(let a): String(format: "lookAround(%.0f°)", a * 180 / .pi)
        default: "\(d)"
        }
    }

    private static func resolves(_ query: String, among labels: [String]) -> [String] {
        MissionAgent.mostSpecificMatches(labels, query: query, label: { $0 })
    }

    private static func follows(_ want: String, among labels: [String], review: Bool = false) -> Grader {
        { decision in
            if case .follow(.visualQuery(let q)) = decision {
                let hits = resolves(q, among: labels.isEmpty ? [want] : labels)
                if hits == [want] { return (.pass, "follow(\(q))") }
                if hits.isEmpty { return (.fail, "follow(\(q)) grounds on nothing in view") }
                return (.fail, "follow(\(q)) grounds on \(hits)")
            }
            switch decision {
            case .done where review, .say where review:
                return (.ok, "\(describe(decision)) — follow keeps running, unsupervised")
            case .lookAround where labels.isEmpty, .ask where labels.isEmpty:
                return (.ok, "\(describe(decision)) — reasonable with nobody in view")
            default:
                return (.fail, describe(decision))
            }
        }
    }

    private static func navigates(to want: String, among labels: [String]) -> Grader {
        { decision in
            if case .navigate(.visualQuery(let q)) = decision {
                let hits = resolves(q, among: labels)
                return hits == [want] ? (.pass, "navigate(\(q))") : (.fail, "navigate(\(q)) grounds on \(hits)")
            }
            return (.fail, describe(decision))
        }
    }

    private static let speaks: Grader = { decision in
        switch decision {
        case .say: (.pass, describe(decision))
        case .ask, .lookAround: (.ok, describe(decision))  // checking is a fair answer too
        default: (.fail, describe(decision))
        }
    }

    private static let turnsLeft: Grader = { decision in
        if case .lookAround(let angle) = decision, angle > 0 { return (.pass, describe(decision)) }
        return (.fail, describe(decision))
    }

    private static let cases: [DecisionCase] = {
        let one = [("person", Float(0.92))]
        let hat = [("person_hat", Float(0.86)), ("person", Float(0.93))]  // decoy is louder
        let hatSwapped = [("person", Float(0.93)), ("person_hat", Float(0.86))]
        let clutter = [("person", Float(0.9)), ("red_toolbox", Float(0.88)), ("chair", Float(0.8))]
        let toolboxes = [("blue_toolbox", Float(0.95)), ("red_toolbox", Float(0.9)), ("person", Float(0.9))]
        let labels = { (v: [(String, Float)]) in v.map(\.0) }
        return [
            DecisionCase(id: "F1", group: "follow", utterance: "follow me", visible: one,
                         grade: follows("person", among: labels(one))),
            DecisionCase(id: "F2", group: "follow", utterance: "come with me", visible: one,
                         grade: follows("person", among: labels(one))),
            DecisionCase(id: "F3", group: "follow", utterance: "stay behind me while I walk to the gate",
                         visible: one, grade: follows("person", among: labels(one))),
            DecisionCase(id: "F4", group: "follow", utterance: "follow that person", visible: clutter,
                         grade: follows("person", among: labels(clutter))),
            DecisionCase(id: "F5", group: "follow-attribute", utterance: "follow the guy with the hat",
                         visible: hat, grade: follows("person_hat", among: labels(hat))),
            DecisionCase(id: "F6", group: "follow-attribute", utterance: "keep up with the man in the hat",
                         visible: hatSwapped, grade: follows("person_hat", among: labels(hatSwapped))),
            DecisionCase(id: "F7", group: "follow", utterance: "follow me", visible: [],
                         grade: follows("person", among: [])),
            DecisionCase(id: "N1", group: "not-follow", utterance: "go to the red toolbox", visible: toolboxes,
                         grade: navigates(to: "red_toolbox", among: labels(toolboxes))),
            DecisionCase(id: "N2", group: "not-follow", utterance: "is anyone there?", visible: one,
                         grade: speaks),
            DecisionCase(id: "N3", group: "not-follow", utterance: "drive over to that person", visible: one,
                         grade: navigates(to: "person", among: labels(one))),
            DecisionCase(id: "N4", group: "not-follow", utterance: "turn left", visible: one, grade: turnsLeft),
            DecisionCase(id: "R1", group: "review", utterance: nil, priorUtterance: "follow the guy with the hat",
                         visible: hat, followState: "following the guy with the hat",
                         recentActions: ["follow(the guy with the hat) → following"],
                         grade: follows("person_hat", among: labels(hat), review: true)),
            DecisionCase(id: "R2", group: "review", utterance: nil, priorUtterance: "follow me",
                         visible: one, followState: "following person",
                         recentActions: ["follow(person) → following"],
                         grade: follows("person", among: labels(one), review: true)),
            DecisionCase(id: "R3", group: "review", utterance: nil, priorUtterance: "follow me",
                         visible: [], followState: "searching person",
                         recentActions: ["follow(person) → following", "follow(person) → searching"],
                         grade: follows("person", among: [], review: true)),
        ]
    }()

    private static func context(for c: DecisionCase) -> MissionContext {
        let pose = Pose2D(position: Vec2(0, 1.5), yaw: .pi / 2)
        var memory = MissionMemory()
        if let first = c.priorUtterance ?? c.utterance { memory.record(utterance: first, at: pose) }
        let visible = c.visible.enumerated().map { i, v in
            PerceivedObject(label: v.0, confidence: v.1,
                            normalizedPoint: CGPoint(x: 0.3 + 0.2 * Double(i), y: 0.5))
        }
        return MissionContext(utterance: c.utterance, visibleObjects: visible, pose: pose,
                              navState: .idle, memory: memory, explorationCandidates: [],
                              plan: nil, lastAnswerWasInconclusive: false,
                              followState: c.followState, recentActions: c.recentActions)
    }

    func testDecisions() async throws {
        let (name, brain) = try brainOrSkip()
        let repeats = Int(env["BENCH_REPEATS"] ?? "") ?? 5

        // Untimed warm-up: the first call pays for loading the model.
        let warmStart = Date()
        _ = try? await brain.nextAction(Self.context(for: Self.cases[0]))
        benchEmit(["tier": "warmup", "brain": name, "ms": Date().timeIntervalSince(warmStart) * 1000])

        for c in Self.cases {
            for rep in 0..<repeats {
                BenchStats.shared.last = [:]
                let started = Date()
                var record: [String: Any] = ["tier": "decision", "brain": name, "case": c.id,
                                             "group": c.group, "rep": rep]
                do {
                    let output = try await brain.nextAction(Self.context(for: c))
                    let (grade, why) = c.grade(output.decision)
                    record["grade"] = grade.rawValue
                    record["why"] = why
                } catch {
                    record["grade"] = Grade.fail.rawValue
                    record["why"] = "error: \(error)"
                }
                record["ms"] = Date().timeIntervalSince(started) * 1000
                record["stats"] = BenchStats.shared.last
                benchEmit(record)
            }
        }
    }

    // MARK: - Tier B: closed loop in the Godot Depot sim

    func testClosedLoopFollowMe() async throws {
        try await runClosedLoop(scenario: "follow_me", utterance: "follow me", withHatAndDecoy: false)
    }

    func testClosedLoopFollowTheGuyWithTheHat() async throws {
        try await runClosedLoop(scenario: "hat_and_decoy", utterance: "follow the guy with the hat",
                                withHatAndDecoy: true)
    }

    /// Hallway lanes, in metres east of the centreline.
    private static let hatLane = 0.5
    private static let decoyLane = -0.5
    private static let window: TimeInterval = 38

    private func runClosedLoop(scenario: String, utterance: String, withHatAndDecoy: Bool) async throws {
        let (name, inner) = try brainOrSkip()
        let link: GodotLink
        do { link = try GodotLink() } catch {
            throw XCTSkip("Godot Depot sim not reachable — launch it first.")
        }
        let rid = "bench-\(scenario)"
        link.call(["op": "reset", "seed": 7])
        link.call(["op": "phrover_spawn", "id": rid, "p": [0.0, 1.5], "yaw": Double.pi / 2])

        // 0.15 m/s: slow enough for the sim rover (MAX_V 0.5 m/s) and long enough that the
        // walk outlasts several 10 s brain reviews before the hallway's north end.
        let walkerLane = withHatAndDecoy ? Self.hatLane : 0.0
        var walker: [String: Any] = ["on": true, "speed": 0.15,
                                     "waypoints": [[walkerLane, 4.0], [walkerLane, 9.8]]]
        if withHatAndDecoy { walker["label"] = "person_hat" }
        link.call(["op": "inject", "name": "person_walk", "params": walker])
        if withHatAndDecoy {
            link.call(["op": "inject", "name": "person_walk",
                       "params": ["on": true, "person": "DecoyActor", "speed": 0.15,
                                  "waypoints": [[Self.decoyLane, 4.0], [Self.decoyLane, 9.8]]]])
        }
        try? await Task.sleep(for: .seconds(0.5))

        let events = EventLog()
        let perception = GodotPerception(link: link, rid: rid)
        let follower = FollowController(perception: perception, drive: GodotDrive(link: link, rid: rid),
                                        policy: .ground(standoff: 2.5, maxSpeed: 0.45))
        let timed = TimedBrain(wrapping: inner)
        let recorder = RecordingBrain(wrapping: timed, events: events)
        let agent = MissionAgent(motion: GodotMotion(link: link, rid: rid), perception: perception,
                                 voice: ScriptedVoice(events: events), follower: follower) { recorder }

        let started = Date()
        let mission = Task { await agent.handle(utterance) }

        var followStartedAt: Double?
        var samples = 0, activeSamples = 0, inBand = 0, ranged = 0, onTarget = 0, identified = 0
        let targetLabel = withHatAndDecoy ? "person_hat" : "person"
        while Date().timeIntervalSince(started) < Self.window {
            try? await Task.sleep(for: .seconds(0.25))
            samples += 1
            var active = follower.followingSpec != nil
            if case .ended = follower.followState { active = false }
            if active {
                activeSamples += 1
                if followStartedAt == nil { followStartedAt = Date().timeIntervalSince(started) }
            }
            if active, let pose = perception.pose,
               let target = Self.worldPosition(of: targetLabel, link: link, rid: rid) {
                ranged += 1
                let range = pose.position.distance(to: target)
                if range >= 1.8 && range <= 3.6 { inBand += 1 }
            }
            if active, withHatAndDecoy, let track = follower.track {
                identified += 1
                if abs(track.position.x - Self.hatLane) < abs(track.position.x - Self.decoyLane) { onTarget += 1 }
            }
        }
        await agent.handle("stop")
        _ = await mission.value

        let simEvents = (link.call(["op": "get_events", "since": 0.0])["events"] as? [[String: Any]]) ?? []
        let collisions = simEvents.filter {
            ($0["kind"] as? String) == "collision"
                && (($0["data"] as? [String: Any])?["with"] as? String) == "person"
        }.count
        let decisions = events.events.filter { $0.kind == "decision" }.map { $0.data["decision"] as? String ?? "?" }

        var record: [String: Any] = [
            "tier": "loop", "brain": name, "scenario": scenario, "utterance": utterance,
            "window_s": Self.window,
            "follow_started_s": followStartedAt ?? NSNull(),
            "follow_active_frac": samples > 0 ? Double(activeSamples) / Double(samples) : 0,
            "in_band_frac": ranged > 0 ? Double(inBand) / Double(ranged) : NSNull(),
            "collisions_with_person": collisions,
            "unverified_reacquisitions": follower.unverifiedReacquisitions,
            "decisions": decisions,
            "decision_ms": timed.latenciesMs,
            "spoken": events.events.filter { $0.kind == "speak" }.map { $0.data["text"] as? String ?? "" },
        ]
        if withHatAndDecoy {
            record["on_hat_frac"] = identified > 0 ? Double(onTarget) / Double(identified) : NSNull()
        }
        benchEmit(record)
        follower.cancel()
    }

    private static func worldPosition(of label: String, link: GodotLink, rid: String) -> Vec2? {
        let result = link.call(["op": "phrover_detect", "id": rid])
        guard let objects = result["objects"] as? [[String: Any]] else { return nil }
        for object in objects where (object["label"] as? String) == label {
            if let world = godotDoubleArray(object["world"]), world.count == 2 {
                return Vec2(world[0], world[1])
            }
        }
        return nil
    }
}

/// Records how long each brain decision took inside the closed loop.
@MainActor
private final class TimedBrain: RoverBrain {
    private let inner: RoverBrain
    private(set) var latenciesMs: [Double] = []

    init(wrapping inner: RoverBrain) { self.inner = inner }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        let started = Date()
        defer { latenciesMs.append(Date().timeIntervalSince(started) * 1000) }
        return try await inner.nextAction(context)
    }
}
