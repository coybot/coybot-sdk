import XCTest
@testable import PhroverKit

@MainActor
final class OnDeviceBrainTests: XCTestCase {
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
