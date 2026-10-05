import XCTest
import CoreGraphics
import RoverNav
@testable import PhroverKit

/// The full cognitive loop, in sim: **"go to the green chair in the other room, then come
/// back"** — planning, exploration under ambiguity, reasoning over what a visit revealed,
/// asking the human, grounding, memory, and object permanence.
///
/// A scripted brain plays the role of a rational model; a tiny `SimWorld` plays the house.
/// What these tests prove is the *machinery*: that at every tick the agent hands the brain
/// exactly the context a real model needs for each inference (candidates with visited
/// status, remembered objects, the echoed plan, the operator's answer), and that it
/// executes every decision correctly. Whether a real model *makes* those inferences well
/// is a hardware/live-tier question, deliberately out of scope here.
///
/// Runs in the existing phrover fast e2e gate (eco/e2e/harness/phrover.py runs
/// PhroverKitTests on every push).
@MainActor
final class MissionCognitionTests: XCTestCase {

    // World layout (nav-plane meters):
    //   start room at (0,0) with two doorway openings;
    //   opening_1 -> (3, 2): a hallway, nothing in it;
    //   opening_2 -> (3, -2): the other room, green chair at (6, -3).
    static let start = Vec2(0, 0)
    static let doorway1 = Vec2(3, 2)
    static let doorway2 = Vec2(3, -2)
    static let chair = Vec2(6, -3)

    func testGreenChairInOtherRoomAndBack() async {
        let world = SimWorld()
        let motion = SimMotion(world: world)
        let perception = SimPerception(world: world)
        let voice = FakeCognitionVoice()
        voice.scriptedReplies = ["I'm not sure, sorry"]

        let brain = ScriptedBrain(steps: [
            // Tick 0 — two unexplored doorways, no chair in sight: plan the mission and
            // ask the operator for a hint before picking a door.
            { _ in BrainOutput(decision: .ask("I see two doorways — do you know which room the chair is in?"),
                               updatedPlan: "1. find the green chair 2. return to start") },
            // Tick 1 — answer was no help: explore the first opening.
            { _ in BrainOutput(decision: .explore(candidateId: "opening_1")) },
            // Tick 2 — that was a hallway (context: opening_1 visited, still no chair):
            // try the other one.
            { _ in BrainOutput(decision: .explore(candidateId: "opening_2")) },
            // Tick 3 — chair in view now: drive to it by open-vocab description.
            { _ in BrainOutput(decision: .navigate(.visualQuery("green chair"))) },
        ])

        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { brain }
        await agent.handle("go to the green chair in the other room, then come back")

        // The physical mission: hallway, other room, chair, and back to the start.
        XCTAssertEqual(motion.navigateCalls, [Self.doorway1, Self.doorway2, Self.chair, Self.start])

        let ctx = brain.seenContexts
        XCTAssertEqual(ctx.count, 4,
                       "the agent should execute the return leg without asking the brain again")

        // Tick 0: the brain was offered both openings, both unexplored — the basis for
        // asking rather than guessing.
        XCTAssertEqual(ctx[0].explorationCandidates.map(\.id), ["opening_1", "opening_2"])
        XCTAssertTrue(ctx[0].explorationCandidates.allSatisfy { $0.status == .unexplored })
        XCTAssertTrue(ctx[0].visibleObjects.isEmpty)

        // Asking the human: question spoken, reply recorded in memory AND delivered as
        // the next tick's utterance.
        XCTAssertTrue(voice.spoken.contains { $0.contains("two doorways") })
        XCTAssertTrue(agent.memory.turns.contains { $0.utterance == "I'm not sure, sorry" })
        XCTAssertEqual(ctx[1].utterance, "I'm not sure, sorry")

        // Planning: the plan written at tick 0 was echoed back from tick 1 on…
        XCTAssertEqual(ctx[1].plan, "1. find the green chair 2. return to start")
        // …and the agent completed the structured return leg at the command's start pose.
        XCTAssertEqual(ctx[3].memory.missionStartPose?.position, Self.start)
        XCTAssertEqual(agent.plan, "Primary target reached; returned to mission start.")

        // Reasoning over discovery: when the brain chose opening_2, its context provably
        // showed opening_1 already visited and nothing found there.
        let atSecondChoice = ctx[2]
        XCTAssertEqual(atSecondChoice.explorationCandidates.first { $0.id == "opening_1" }?.status, .visited)
        XCTAssertEqual(atSecondChoice.explorationCandidates.first { $0.id == "opening_2" }?.status, .unexplored)
        XCTAssertTrue(atSecondChoice.visibleObjects.isEmpty, "the hallway should contain nothing")

        // Grounding: from the other room the chair was visible and remembered at its
        // world position before the navigate tick.
        XCTAssertTrue(ctx[3].visibleObjects.contains { $0.label == "chair" })
        XCTAssertTrue(ctx[3].memory.rememberedObjects.contains {
            $0.label == "chair" && $0.worldPoint.distance(to: Self.chair) < 0.1
        })
    }

    /// Object permanence: after the mission, the chair is out of sight — but a follow-up
    /// "go back to the chair" works from remembered world coordinates alone.
    func testObjectPermanenceAcrossUtterances() async {
        let world = SimWorld()
        let motion = SimMotion(world: world)
        let perception = SimPerception(world: world)
        let voice = FakeCognitionVoice()

        // Mission 1: drive within sight of the chair (via the second doorway) and finish.
        let firstBrain = ScriptedBrain(steps: [
            { _ in BrainOutput(decision: .explore(candidateId: "opening_2")) },
            { _ in BrainOutput(decision: .navigate(.visualQuery("green chair"))) },
            { _ in BrainOutput(decision: .navigate(.worldPoint(Self.start))) }, // come back
            { _ in BrainOutput(decision: .done) },
        ])
        let secondBrain = ScriptedBrain(steps: [
            { ctx in
                // The chair isn't visible from here — but memory still has it: navigate
                // straight to the remembered spot.
                guard let chair = ctx.memory.rememberedObjects.first(where: { $0.label == "chair" }) else {
                    return BrainOutput(decision: .say("I don't remember a chair."))
                }
                return BrainOutput(decision: .navigate(.worldPoint(chair.worldPoint)))
            },
            { _ in BrainOutput(decision: .done) },
        ])

        // One agent, one memory, two missions — the provider closure reads the current
        // brain each tick, so swapping scripts between utterances is just reassignment.
        var activeBrain: ScriptedBrain = firstBrain
        let agent = MissionAgent(motion: motion, perception: perception, voice: voice) { activeBrain }

        await agent.handle("go to the green chair in the other room and come back")
        XCTAssertEqual(world.roverAt, Self.start)

        activeBrain = secondBrain
        await agent.handle("now go back to the chair")

        let followUpCtx = secondBrain.seenContexts[0]
        XCTAssertTrue(followUpCtx.visibleObjects.isEmpty, "chair must not be visible from the start room")
        XCTAssertTrue(followUpCtx.memory.rememberedObjects.contains {
            $0.label == "chair" && $0.worldPoint.distance(to: Self.chair) < 0.1
        })
        XCTAssertEqual(motion.navigateCalls.last!, Self.chair)
    }

    func testRememberedObjectUpsertDedupes() {
        var memory = MissionMemory()
        memory.rememberObject(label: "chair", at: Vec2(1.0, 1.0))
        memory.rememberObject(label: "chair", at: Vec2(1.2, 1.1))   // same chair, refined fix
        memory.rememberObject(label: "chair", at: Vec2(5.0, 5.0))   // a different chair
        memory.rememberObject(label: "person", at: Vec2(1.0, 1.0))  // different label, same spot

        XCTAssertEqual(memory.rememberedObjects.count, 3)
        let first = memory.rememberedObjects[0]
        XCTAssertEqual(first.timesSeen, 2)
        XCTAssertEqual(first.worldPoint, Vec2(1.2, 1.1), "position refreshed to the latest fix")
    }

    func testExploreUnknownCandidateFailsGracefully() async {
        let world = SimWorld()
        let motion = SimMotion(world: world)
        let voice = FakeCognitionVoice()
        let brain = ScriptedBrain(steps: [
            { _ in BrainOutput(decision: .explore(candidateId: "opening_99")) },
            { _ in BrainOutput(decision: .done) },
        ])
        let agent = MissionAgent(motion: motion, perception: SimPerception(world: world), voice: voice) { brain }

        await agent.handle("go somewhere")

        XCTAssertTrue(motion.navigateCalls.isEmpty)
        XCTAssertFalse(voice.spoken.isEmpty, "should say it doesn't know that opening, not silently no-op")
        XCTAssertEqual(agent.phase, .idle)
    }
}

// MARK: - Sim scaffolding

/// Shared ground truth: where the rover is determines what it can see. Positions advance
/// when `SimMotion` "arrives".
@MainActor
private final class SimWorld {
    var roverAt = MissionCognitionTests.start
    var chairVisible: Bool { roverAt.distance(to: MissionCognitionTests.chair) < 3.5 }
}

@MainActor
private final class SimMotion: RoverMotion {
    private let world: SimWorld
    var state: NavigationController.State = .idle
    private(set) var navigateCalls: [Vec2] = []
    private(set) var rotateCalls: [Double] = []

    init(world: SimWorld) { self.world = world }

    func navigate(to goal: Vec2) {
        navigateCalls.append(goal)
        state = .driving
        Task { @MainActor in
            try? await Task.sleep(for: .milliseconds(10))
            world.roverAt = goal
            state = .arrived
        }
    }

    func rotate(by angle: Double) async { rotateCalls.append(angle) }
    func cancel() {}
}

@MainActor
private final class SimPerception: RoverPerception {
    private let world: SimWorld

    init(world: SimWorld) { self.world = world }

    var pose: Pose2D? { Pose2D(position: world.roverAt, yaw: 0) }

    func detectObjects() -> [PerceivedObject] {
        guard world.chairVisible else { return [] }
        return [
            PerceivedObject(
                label: "chair",
                confidence: 0.9,
                normalizedPoint: CGPoint(x: 0.5, y: 0.5),
                colorEvidence: [ObjectColorEvidence(color: .green, confidence: 0.70)]
            )
        ]
    }

    func unproject(normalizedPoint: CGPoint) -> Vec2? {
        world.chairVisible ? MissionCognitionTests.chair : nil
    }

    func capturedFrameJPEG() -> Data? { nil }

    // Default groundObject (substring match over detectObjects) is used deliberately.

    func explorationFrontiers() -> [Frontier] {
        [Frontier(centroid: MissionCognitionTests.doorway1, widthMeters: 1.0, cellCount: 10),
         Frontier(centroid: MissionCognitionTests.doorway2, widthMeters: 0.9, cellCount: 9)]
    }
}

/// Plays the model: each step consumes one think-tick, with full access to the context the
/// agent supplied — so a step can e.g. read the mission start pose out of memory exactly
/// as a real model would.
@MainActor
private final class ScriptedBrain: RoverBrain {
    private var steps: [(MissionContext) -> BrainOutput]
    private(set) var seenContexts: [MissionContext] = []

    init(steps: [(MissionContext) -> BrainOutput]) { self.steps = steps }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        seenContexts.append(context)
        guard !steps.isEmpty else { return BrainOutput(decision: .done) }
        return steps.removeFirst()(context)
    }
}

@MainActor
private final class FakeCognitionVoice: RoverVoice {
    private(set) var spoken: [String] = []
    var scriptedReplies: [String?] = []

    func speak(_ text: String) { spoken.append(text) }

    func ask(_ question: String, timeout: TimeInterval) async -> String? {
        spoken.append(question)
        return scriptedReplies.isEmpty ? nil : scriptedReplies.removeFirst()
    }
}
