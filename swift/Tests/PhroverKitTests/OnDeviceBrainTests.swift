import XCTest
@testable import PhroverKit
import RoverNav

@MainActor
final class OnDeviceBrainTests: XCTestCase {
    func testReportsEveryUnavailableReasonWithoutCreatingResponder() async {
        let unavailableStates: [(OnDeviceBrainAvailability, String)] = [
            (.deviceNotEligible,
             "Apple Intelligence is not supported on this iPhone. Supported object commands can still run offline."),
            (.appleIntelligenceNotEnabled,
             "Turn on Apple Intelligence in Settings. Supported object commands can still run offline."),
            (.modelNotReady,
             "Apple Intelligence is still preparing. Keep the iPhone on Wi-Fi and power. Supported object commands can still run offline."),
        ]

        for (state, expectedOperatorMessage) in unavailableStates {
            let brain = OnDeviceBrain(availability: { state }, makeResponder: {
                XCTFail("Unavailable brain must not create a responder")
                return FakeOnDeviceBrainResponder()
            })

            XCTAssertEqual(brain.availability, state)
            XCTAssertFalse(brain.isAvailable)
            XCTAssertEqual(state.operatorMessage, expectedOperatorMessage)
            do {
                _ = try await brain.nextAction(MissionContext())
                XCTFail("Expected unavailable brain to throw")
            } catch {
                XCTAssertEqual(error as? RoverBrainError, .onDeviceUnavailable(state))
            }
        }
    }

    func testAvailabilityStatesHaveStableLogValues() {
        XCTAssertEqual(OnDeviceBrainAvailability.available.logValue, "available")
        XCTAssertEqual(OnDeviceBrainAvailability.deviceNotEligible.logValue, "device_not_eligible")
        XCTAssertEqual(
            OnDeviceBrainAvailability.appleIntelligenceNotEnabled.logValue,
            "apple_intelligence_not_enabled"
        )
        XCTAssertEqual(OnDeviceBrainAvailability.modelNotReady.logValue, "model_not_ready")
    }

    func testAvailableStateCreatesFreshResponderForEachDecision() async throws {
        let factory = RecordingResponderFactory()
        let brain = OnDeviceBrain(availability: { .available }, makeResponder: factory.makeResponder)

        XCTAssertEqual(brain.availability, .available)
        XCTAssertTrue(brain.isAvailable)
        XCTAssertNil(brain.availability.operatorMessage)

        _ = try await brain.nextAction(MissionContext())
        _ = try await brain.nextAction(MissionContext())

        XCTAssertEqual(factory.createdCount, 2)
    }

    func testCreatesFreshResponderForEachDecision() async throws {
        let factory = RecordingResponderFactory()
        let brain = OnDeviceBrain(isAvailable: { true }, makeResponder: factory.makeResponder)

        _ = try await brain.nextAction(MissionContext())
        _ = try await brain.nextAction(MissionContext())

        XCTAssertEqual(factory.createdCount, 2)
    }

    func testFollowMapsToAVisualQueryFollow() {
        let decision = OnDeviceBrain.map(Self.raw(.follow, visualQuery: "the person with the hat"),
                                         context: MissionContext())
        XCTAssertEqual(decision, .follow(.visualQuery("the person with the hat")))
    }

    func testFollowWithNoDescriptionFollowsAPerson() {
        let decision = OnDeviceBrain.map(Self.raw(.follow, visualQuery: "  "), context: MissionContext())
        XCTAssertEqual(decision, .follow(.visualQuery("person")))
    }

    func testPromptTellsTheModelAFollowIsRunning() {
        let brain = OnDeviceBrain(isAvailable: { true }, makeResponder: { FakeOnDeviceBrainResponder() })
        var context = MissionContext()
        context.followState = "following the guy with the hat"
        context.recentActions = ["follow(person) → following"]

        let prompt = brain.promptText(context)

        XCTAssertTrue(prompt.contains("currently following the guy with the hat"), prompt)
        XCTAssertTrue(prompt.contains("follow(person) → following"), prompt)
    }

    private static func raw(_ action: OnDeviceAction, visualQuery: String = "") -> OnDeviceDecision {
        OnDeviceDecision(action: action, visualQuery: visualQuery, memoryQuery: "", question: "",
                         spokenText: "", lookAroundDegrees: 0, exploreCandidateId: "", updatedPlan: "")
    }

    func testRejectsVisualTargetInventedOutsideCurrentMission() async throws {
        let responder = ScriptedOnDeviceBrainResponder(output: BrainOutput(
            decision: .navigate(.visualQuery("the green chair")),
            updatedPlan: "1. find the green chair"
        ))
        let brain = OnDeviceBrain(isAvailable: { true }, makeResponder: { responder })
        var memory = MissionMemory()
        memory.beginMission(utterance: "Find a chair", at: Pose2D(position: .zero, yaw: 0))

        let output = try await brain.nextAction(MissionContext(
            memory: memory,
            explorationCandidates: [
                ExplorationCandidate(id: "opening_1",
                                     worldPoint: Vec2(1, 0),
                                     widthMeters: 1,
                                     status: .visited),
                ExplorationCandidate(id: "opening_2",
                                     worldPoint: Vec2(2, 0),
                                     widthMeters: 1),
            ]
        ))

        XCTAssertEqual(output.decision, .explore(candidateId: "opening_2"))
        XCTAssertNil(output.updatedPlan)
    }

    func testRestoresVisibleOperatorTargetWhenModelInventsAttribute() async throws {
        let responder = ScriptedOnDeviceBrainResponder(output: BrainOutput(
            decision: .navigate(.visualQuery("the gray refrigerator")),
            updatedPlan: "1. go to the gray refrigerator"
        ))
        let brain = OnDeviceBrain(isAvailable: { true }, makeResponder: { responder })
        var memory = MissionMemory()
        memory.beginMission(utterance: "Go to refrigerator", at: Pose2D(position: .zero, yaw: 0))

        let output = try await brain.nextAction(MissionContext(
            visibleObjects: [
                PerceivedObject(
                    label: "refrigerator",
                    confidence: 0.99,
                    normalizedPoint: CGPoint(x: 0.5, y: 0.5)
                ),
            ],
            memory: memory,
            explorationCandidates: [
                ExplorationCandidate(id: "opening_1", worldPoint: Vec2(2, 0), widthMeters: 1),
            ]
        ))

        XCTAssertEqual(output.decision, .navigate(.visualQuery("refrigerator")))
        XCTAssertNil(output.updatedPlan)
    }

    func testPromptIncludesObjectAndColorConfidence() async throws {
        let responder = PromptRecordingResponder()
        let brain = OnDeviceBrain(availability: { .available }, makeResponder: { responder })

        _ = try await brain.nextAction(MissionContext(visibleObjects: [
            PerceivedObject(
                label: "chair",
                confidence: 0.96,
                normalizedPoint: CGPoint(x: 0.5, y: 0.5),
                colorEvidence: [ObjectColorEvidence(color: .black, confidence: 0.82)]
            ),
        ]))

        XCTAssertTrue(responder.lastPrompt?.contains(
            "black chair (96% object confidence, 82% color confidence)"
        ) == true)
    }

    func testPromptPreservesLegacyObjectFormatWithoutColorEvidence() async throws {
        let responder = PromptRecordingResponder()
        let brain = OnDeviceBrain(availability: { .available }, makeResponder: { responder })

        _ = try await brain.nextAction(MissionContext(visibleObjects: [
            PerceivedObject(
                label: "chair",
                confidence: 0.96,
                normalizedPoint: CGPoint(x: 0.5, y: 0.5)
            ),
        ]))

        XCTAssertTrue(responder.lastPrompt?.contains("chair (96% confidence)") == true)
    }
}

@MainActor
private final class RecordingResponderFactory {
    private(set) var createdCount = 0

    func makeResponder() -> OnDeviceBrainResponder {
        createdCount += 1
        return FakeOnDeviceBrainResponder()
    }
}

private struct FakeOnDeviceBrainResponder: OnDeviceBrainResponder {
    func nextAction(prompt: String, context: MissionContext) async throws -> BrainOutput {
        BrainOutput(decision: .done)
    }
}

private struct ScriptedOnDeviceBrainResponder: OnDeviceBrainResponder {
    let output: BrainOutput

    func nextAction(prompt: String, context: MissionContext) async throws -> BrainOutput {
        output
    }
}

@MainActor
private final class PromptRecordingResponder: OnDeviceBrainResponder {
    private(set) var lastPrompt: String?

    func nextAction(prompt: String, context: MissionContext) async throws -> BrainOutput {
        lastPrompt = prompt
        return BrainOutput(decision: .done)
    }
}
