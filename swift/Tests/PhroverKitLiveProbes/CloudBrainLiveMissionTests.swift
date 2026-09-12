import XCTest
import CoreGraphics
import RoverNav
import PhroverKit
import PhroverCloud

/// The strongest evidence tier for mission cognition: the REAL production loop —
/// `MissionAgent` (real), `CloudBrain` wire path (real), `/rover/act` handler + prompt +
/// tool schema (real, via the local bridge), Bedrock Claude (real) — with only the house
/// and the human simulated. No scripted brain anywhere: every decision below is made by
/// the actual model.
///
/// Requires the local bridge running (it fakes only the API Gateway authorizer, and only
/// on 127.0.0.1):
///
///   cd eco && AWS_PROFILE=coybot python3 -m e2e.harness.live_rover_act_bridge
///
/// then run with the bridge URL injected (or just use eco/e2e/run_live_mission.sh):
///
///   TEST_RUNNER_LIVE_ROVER_ACT_URL=http://127.0.0.1:<port> xcodebuild test \
///     -scheme coybot-sdk-Package -destination 'platform=iOS Simulator,name=iPhone 17' \
///     -only-testing:PhroverKitLiveProbes/CloudBrainLiveMissionTests
///
/// Skips (not fails) when the env var is absent, so it can never leak into the fast gate.
/// Assertions are on mission OUTCOMES, never on which path the model picks — a model that
/// finds the chair without ever entering the hallway is right, not lucky.
@MainActor
final class CloudBrainLiveMissionTests: XCTestCase {
    // Same ground truth as MissionCognitionTests / OnDeviceBrainLiveProbe /
    // probe_mission_cognition.py.
    static let start = Vec2(0, 0)
    static let doorway1 = Vec2(3, 2)   // hallway — nothing behind it
    static let doorway2 = Vec2(3, -2)  // the chair's room
    static let chair = Vec2(6, -3)

    private func makeBrain() throws -> CloudBrain {
        guard let url = ProcessInfo.processInfo.environment["LIVE_ROVER_ACT_URL"], !url.isEmpty else {
            throw XCTSkip("LIVE_ROVER_ACT_URL not set — live mission test only runs via eco/e2e/run_live_mission.sh (real billed Bedrock calls).")
        }
        // Only apiEndpoint is read by CloudBrain; with no token provider set, no
        // Authorization header is sent — the bridge does not check auth (it fakes the
        // authorizer context itself, locally).
        let config = PhroverCloudConfig(region: "us-west-2", apiEndpoint: url,
                                        identityPoolId: "", userPoolId: "",
                                        iotEndpoint: "", cognitoClientId: "")
        return CloudBrain(config: config)
    }

    /// Scenario 1 — the full mission, then an object-permanence follow-up.
    func testLiveGreenChairMission() async throws {
        let brain = try makeBrain()
        let world = LiveSimWorld()
        let motion = LiveSimMotion(world: world)
        let perception = LiveSimPerception(world: world)
        let voice = LiveSimVoice(world: world)
        // If the model asks which room, the human is honestly unhelpful — it has to
        // figure the doors out itself.
        voice.scriptedReplies = ["I'm not sure, sorry — somewhere through one of those doorways."]

        let recorder = TranscriptRecorder(wrapping: brain, world: world)
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { recorder }

        await agent.handle("go to the chair in the other room, then come back")
        recorder.dump(label: "mission 1: go to the chair in the other room, then come back")

        // ---- Outcome assertions (path-agnostic) ----

        XCTAssertEqual(agent.phase, .idle, "mission must terminate within the agent's tick cap")

        let arrivals = motion.navigateCalls
        let reachedChairAt = arrivals.firstIndex { $0.distance(to: Self.chair) < 0.5 }
        XCTAssertNotNil(reachedChairAt, "the rover never physically reached the chair; arrivals=\(arrivals)")
        let returnedAt = arrivals.lastIndex { $0.distance(to: Self.start) < 0.5 }
        XCTAssertNotNil(returnedAt, "the rover never returned to the start; arrivals=\(arrivals)")
        if let c = reachedChairAt, let r = returnedAt {
            XCTAssertLessThan(c, r, "chair must come before the return leg; arrivals=\(arrivals)")
        }

        // No hallucinated coordinates: every commanded goal is a place that exists.
        let knownPlaces = [Self.start, Self.doorway1, Self.doorway2, Self.chair]
        for goal in arrivals {
            XCTAssertTrue(knownPlaces.contains { $0.distance(to: goal) < 0.5 },
                          "model navigated to a coordinate that corresponds to nothing: \(goal)")
        }

        // Dead-end reasoning: visiting the hallway is a legitimate search move — going
        // back into it after having seen it's empty is not.
        let hallwayVisits = arrivals.filter { $0.distance(to: Self.doorway1) < 0.5 }.count
        XCTAssertLessThanOrEqual(hallwayVisits, 1, "re-explored the hallway it already knew was empty")

        // ---- Object permanence: chair out of sight, detections suppressed entirely ----

        world.suppressDetections = true
        let arrivalsBefore = motion.navigateCalls.count

        await agent.handle("go back to the chair")
        recorder.dump(label: "mission 2: go back to the chair (zero detections the whole leg)")

        let followUp = Array(motion.navigateCalls.dropFirst(arrivalsBefore))
        XCTAssertTrue(followUp.contains { $0.distance(to: Self.chair) < 0.5 },
                      "with no detections at all, the model had to navigate from rememberedObjects — it didn't; follow-up arrivals=\(followUp)")
    }

    /// Scenario 2 — ask-when-needed, made provably necessary: the utterance references
    /// something no context can resolve (empty memory, nothing visible), so a rational
    /// model MUST ask before it can navigate anywhere meaningful.
    func testLiveAsksWhenGoalIsUnknowable() async throws {
        let brain = try makeBrain()
        let world = LiveSimWorld()
        let motion = LiveSimMotion(world: world)
        let perception = LiveSimPerception(world: world)
        let voice = LiveSimVoice(world: world)
        voice.scriptedReplies = ["Oh right — it's the chair, in the other room."]

        let recorder = TranscriptRecorder(wrapping: brain, world: world)
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { recorder }

        await agent.handle("take this to the spot I told you about yesterday")
        recorder.dump(label: "mission 3: take this to the spot I told you about yesterday")

        XCTAssertFalse(voice.askedQuestions.isEmpty,
                       "the goal was unknowable from context — a rational model must ask, this one never did")

        // The ask must have come before any driving: navigating first would mean it
        // guessed at an unknowable goal.
        if let firstAsk = world.events.firstIndex(where: { $0.hasPrefix("ask") }) {
            let firstDrive = world.events.firstIndex { $0.hasPrefix("navigate") } ?? world.events.count
            XCTAssertLessThan(firstAsk, firstDrive,
                              "model navigated before asking about an unknowable goal; events=\(world.events)")
        }

        // And after the human's answer, the mission is completable — it should reach the
        // chair.
        XCTAssertTrue(motion.navigateCalls.contains { $0.distance(to: Self.chair) < 0.5 },
                      "after being told the goal is the chair in the other room, it never got there; arrivals=\(motion.navigateCalls)")
    }
}

// MARK: - Sim scaffolding (the house and the human — the only non-production pieces)

@MainActor
private final class LiveSimWorld {
    var roverAt = CloudBrainLiveMissionTests.start
    /// For the object-permanence leg: the chair stops being detectable no matter how
    /// close the rover gets, so only remembered coordinates can find it.
    var suppressDetections = false
    /// Ordered event log (ask/navigate) so tests can assert on sequencing across
    /// voice + motion without brittle tick counting.
    var events: [String] = []

    var chairVisible: Bool {
        !suppressDetections && roverAt.distance(to: CloudBrainLiveMissionTests.chair) < 3.5
    }
}

@MainActor
private final class LiveSimMotion: RoverMotion {
    private let world: LiveSimWorld
    var state: NavigationController.State = .idle
    private(set) var navigateCalls: [Vec2] = []

    init(world: LiveSimWorld) { self.world = world }

    func navigate(to goal: Vec2) {
        navigateCalls.append(goal)
        world.events.append("navigate (\(goal.x), \(goal.y))")
        state = .driving
        Task { @MainActor in
            try? await Task.sleep(for: .milliseconds(10))
            world.roverAt = goal
            state = .arrived
        }
    }

    func rotate(by angle: Double) async { world.events.append("rotate \(angle)") }
    func cancel() {}
}

@MainActor
private final class LiveSimPerception: RoverPerception {
    private let world: LiveSimWorld

    init(world: LiveSimWorld) { self.world = world }

    var pose: Pose2D? { Pose2D(position: world.roverAt, yaw: 0) }

    func detectObjects() -> [PerceivedObject] {
        guard world.chairVisible else { return [] }
        return [PerceivedObject(label: "chair", confidence: 0.9, normalizedPoint: CGPoint(x: 0.5, y: 0.5))]
    }

    func unproject(normalizedPoint: CGPoint) -> Vec2? {
        world.chairVisible ? CloudBrainLiveMissionTests.chair : nil
    }

    func capturedFrameJPEG() -> Data? { nil } // text-only mission — vision grounding is probed separately

    func explorationFrontiers() -> [Frontier] {
        [Frontier(centroid: CloudBrainLiveMissionTests.doorway1, widthMeters: 1.0, cellCount: 10),
         Frontier(centroid: CloudBrainLiveMissionTests.doorway2, widthMeters: 0.9, cellCount: 9)]
    }
}

@MainActor
private final class LiveSimVoice: RoverVoice {
    private let world: LiveSimWorld
    private(set) var spoken: [String] = []
    private(set) var askedQuestions: [String] = []
    var scriptedReplies: [String?] = []

    init(world: LiveSimWorld) { self.world = world }

    func speak(_ text: String) { spoken.append(text) }

    func ask(_ question: String, timeout: TimeInterval) async -> String? {
        askedQuestions.append(question)
        world.events.append("ask \(question)")
        return scriptedReplies.isEmpty ? nil : scriptedReplies.removeFirst()
    }
}

/// Pass-through brain wrapper that records every (context, real decision) pair so the
/// run leaves a gradeable transcript in the xcodebuild log.
@MainActor
private final class TranscriptRecorder: RoverBrain {
    private let inner: RoverBrain
    private let world: LiveSimWorld
    private var entries: [String] = []
    private var tick = 0

    init(wrapping inner: RoverBrain, world: LiveSimWorld) {
        self.inner = inner
        self.world = world
    }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        let candidates = context.explorationCandidates.map { "\($0.id):\($0.status)" }.joined(separator: ",")
        let remembered = context.memory.rememberedObjects.map { "\($0.label)@(\($0.worldPoint.x),\($0.worldPoint.y))" }.joined(separator: ",")
        var line = "tick \(tick) | pose=(\(world.roverAt.x),\(world.roverAt.y))"
        line += " | visible=\(context.visibleObjects.map(\.label))"
        line += " | candidates=[\(candidates)] | remembered=[\(remembered)]"
        if let u = context.utterance { line += " | heard=\"\(u)\"" }
        if let p = context.plan { line += " | plan=\"\(p)\"" }

        let output = try await inner.nextAction(context)

        line += "\n        -> \(output.decision)"
        if let plan = output.updatedPlan { line += " | new plan=\"\(plan)\"" }
        entries.append(line)
        tick += 1
        return output
    }

    func dump(label: String) {
        print("===== LIVE MISSION TRANSCRIPT: \(label) =====")
        for entry in entries { print(entry) }
        print("===== END TRANSCRIPT (\(entries.count) real model decisions) =====")
        entries.removeAll()
        tick = 0
    }
}
