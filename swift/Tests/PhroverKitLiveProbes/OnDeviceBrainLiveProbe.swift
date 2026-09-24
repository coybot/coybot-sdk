import XCTest
import CoreGraphics
import RoverNav
import PhroverKit

/// Live-model probe for the on-device brain — the real `OnDeviceBrain` class (not a
/// reimplementation), making real calls to Apple's on-device Foundation Model. Walks the
/// same door/hallway/chair scenario as sdk's MissionCognitionTests.swift and eco's
/// probe_mission_cognition.py, but with no scripted brain: this test drives its own world
/// state forward from whatever the real model actually decides each tick.
///
/// Deliberately its own test target (`PhroverKitLiveProbes`, see Package.swift) so this
/// never runs in the fast e2e gate — it needs Apple Intelligence ready on the host, is
/// slow, and is non-deterministic. Run explicitly:
///
///   xcodebuild test -scheme coybot-sdk-Package \
///     -destination 'platform=iOS Simulator,name=iPhone 17' \
///     -only-testing:PhroverKitLiveProbes
///
/// The on-device model has no image input (verified against the shipped SDK earlier this
/// project — not yet in the API), so this only exercises text reasoning: planning,
/// exploration, dead-end recognition, memory, return-to-start. Visual grounding accuracy
/// is a `CloudBrain`/hardware concern, not this brain's.
@MainActor
final class OnDeviceBrainLiveProbe: XCTestCase {
    // Same ground truth as the other two probes.
    static let start = Vec2(0, 0)
    static let doorway1 = Vec2(3, 2)   // hallway — nothing behind it
    static let doorway2 = Vec2(3, -2)  // the chair's room
    static let chair = Vec2(6, -3)
    static let chairVisibleRadius = 3.5
    static let maxTicks = 12

    func testGreenChairInOtherRoomAndBack_liveOnDeviceModel() async throws {
        let brain = OnDeviceBrain()
        try XCTSkipUnless(brain.isAvailable,
                          "On-device Foundation Model unavailable (Apple Intelligence not ready on this host) — skipping, not failing.")

        var pose = Self.start
        var candidates = [
            ExplorationCandidate(id: "opening_1", worldPoint: Self.doorway1, widthMeters: 1.0),
            ExplorationCandidate(id: "opening_2", worldPoint: Self.doorway2, widthMeters: 0.9),
        ]
        var memory = MissionMemory()
        var plan: String?
        var lastAnswerWasInconclusive = false
        let utterance = "go to the green chair in the other room, then come back"
        memory.record(utterance: utterance, at: Pose2D(position: pose, yaw: 0))

        func chairVisible() -> Bool { pose.distance(to: Self.chair) < Self.chairVisibleRadius }
        func visibleObjects() -> [PerceivedObject] {
            chairVisible() ? [PerceivedObject(label: "chair", confidence: 0.9, normalizedPoint: CGPoint(x: 0.5, y: 0.5))] : []
        }

        var nextUtterance: String? = utterance
        var finished = false

        for tick in 0..<Self.maxTicks {
            if chairVisible() { memory.rememberObject(label: "chair", at: Self.chair) }

            let ctx = MissionContext(utterance: nextUtterance,
                                     visibleObjects: visibleObjects(),
                                     pose: Pose2D(position: pose, yaw: 0),
                                     navState: .idle,
                                     memory: memory,
                                     explorationCandidates: candidates,
                                     plan: plan,
                                     lastAnswerWasInconclusive: lastAnswerWasInconclusive)
            nextUtterance = nil

            let visibleLabels: [String] = ctx.visibleObjects.map { $0.label }
            print("--- tick \(tick) --- pose=\(pose) candidates=\(candidates.map { "\($0.id):\($0.status)" }) visible=\(visibleLabels)")

            let output = try await brain.nextAction(ctx)
            print("  -> \(output.decision)")
            if let updatedPlan = output.updatedPlan { plan = updatedPlan; print("     plan: \(updatedPlan)") }

            switch output.decision {
            case .explore(let id):
                if let i = candidates.firstIndex(where: { $0.id == id }) {
                    candidates[i].status = .visited
                    pose = candidates[i].worldPoint
                } else {
                    print("     (unknown candidate id \(id) — real model referenced a nonexistent opening)")
                }
            case .navigate(let target):
                switch target {
                case .worldPoint(let p): pose = p
                case .imagePoint: if chairVisible() { pose = Self.chair } // no real depth sensor here
                case .visualQuery: if chairVisible() { pose = Self.chair }
                }
            case .follow(let target):
                // Following is a mode with no discrete outcome to simulate here — this
                // probe only walks a pose forward per decision.
                print("     (follow \(target) — no moving target in this static probe)")
            case .lookAround, .say:
                break // no state change
            case .claimRoom(let roomId):
                print("     (claimRoom \(roomId) — no team context in this single-rover probe)")
            case .ask(let question):
                print("     asked: \"\(question)\" — replying \"I'm not sure, sorry.\"")
                let reply = "I'm not sure, sorry."
                memory.record(utterance: reply, at: Pose2D(position: pose, yaw: 0))
                nextUtterance = reply
                lastAnswerWasInconclusive = false
            case .stop, .done:
                finished = true
            }

            for i in candidates.indices where candidates[i].worldPoint.distance(to: pose) < 1.0 {
                candidates[i].status = .visited
            }

            if finished {
                print("=== finished: \(output.decision) at tick \(tick), final pose=\(pose) ===")
                break
            }
        }

        print("Final plan: \(plan ?? "(none)")")
        print("Final remembered objects: \(memory.rememberedObjects)")
        print("Returned to start: \(pose.distance(to: Self.start) < 0.5 ? "YES" : "NO (pose=\(pose))")")

        // Not hard assertions on mission success — this is a probe, graded by reading the
        // transcript above, not a pass/fail gate. The one thing worth actually asserting:
        // it must terminate, not spin forever.
        XCTAssertTrue(finished, "model never reached done/stop within \(Self.maxTicks) ticks")
    }
}
