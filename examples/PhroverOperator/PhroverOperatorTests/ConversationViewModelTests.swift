import XCTest
@testable import PhroverOperator
import PhroverKit
import RoverNav

@MainActor
final class ConversationViewModelTests: XCTestCase {
    func testRuntimeInitializationIsSingleAndCancelledLoadCannotReplaceRuntime() throws {
        let model = ConversationRuntimeLifetime()
        let old = try XCTUnwrap(model.beginRuntimeInitialization())
        XCTAssertFalse(model.runtimeReady)
        XCTAssertNil(model.beginRuntimeInitialization(), "Only one load may own installation")
        model.invalidateRuntimeInitialization()
        let current = try XCTUnwrap(model.beginRuntimeInitialization())
        XCTAssertFalse(model.completeRuntimeInitialization(old), "Cancelled appearance cannot install")
        XCTAssertTrue(model.completeRuntimeInitialization(current))
        XCTAssertTrue(model.runtimeReady)
        model.invalidateRuntimeInitialization() // Leaving the tab preserves the installed runtime.
        XCTAssertNil(model.beginRuntimeInitialization(), "Returning to Talk must reuse its runtime")
    }

    func testSpeechCaptureFromPreviousAppearanceCannotSubmitAfterReturn() throws {
        let lifetime = ConversationRuntimeLifetime()
        let capture = lifetime.speechEpoch
        XCTAssertTrue(lifetime.acceptsSpeechCapture(from: capture))
        lifetime.invalidateSpeechCapture() // Disappear or background.
        XCTAssertFalse(lifetime.acceptsSpeechCapture(from: capture))
        XCTAssertTrue(lifetime.acceptsSpeechCapture(from: lifetime.speechEpoch))
    }

    func testRecreatedViewGetsFreshRuntimeLifetime() throws {
        let original = ConversationRuntimeLifetime()
        XCTAssertTrue(original.completeRuntimeInitialization(try XCTUnwrap(original.beginRuntimeInitialization())))
        let recreated = ConversationRuntimeLifetime()
        XCTAssertFalse(recreated.runtimeReady)
        XCTAssertNotNil(recreated.beginRuntimeInitialization())
    }

    func testOverlappingFollowFailureKeepsStopControlAndError() async {
        let entered = expectation(description: "mission stop")
        var release: CheckedContinuation<Void, Never>?
        let mission = HeldRouterMission(onStop: {
            await withCheckedContinuation { release = $0; entered.fulfill() }
            throw URLError(.timedOut)
        })
        let follow = IdleRouterFollow()
        let router = OperatorCommandRouter(mission: mission, follow: follow)
        let model = ConversationViewModel(submit: { await router.submit($0) }, stop: { await router.stop() })
        let first = Task { await model.submitFinalSpeech("follow me") }
        await fulfillment(of: [entered], timeout: 2)
        let secondEntered = expectation(description: "duplicate submitted")
        let second = Task { secondEntered.fulfill(); await model.submitFinalSpeech("follow me") }
        await fulfillment(of: [secondEntered], timeout: 2)
        release?.resume()
        await first.value
        await second.value
        XCTAssertEqual(model.errorMessage, "Rover stop could not be confirmed.")
        XCTAssertTrue(model.showsStopFollowing)
    }

    func testAcknowledgedTalkExitDoesNotRepeatStopOrInhibitOnDisappear() async {
        var stops = 0
        var inhibits = 0
        let model = ConversationViewModel(stop: { stops += 1; return .accepted },
                                          inhibit: { inhibits += 1 })
        let left = await model.leaveTalk()
        XCTAssertTrue(left)
        let inhibitsAtHandoff = inhibits
        model.prepareToLeave() // View disappearance follows the tab's acknowledged exit.
        let disappeared = await model.leaveTalk()
        XCTAssertTrue(disappeared)
        XCTAssertEqual(stops, 1)
        XCTAssertEqual(inhibits, inhibitsAtHandoff)
        await model.submitFinalSpeech("follow me") // A later Talk command starts a new exit epoch.
        _ = await model.leaveTalk()
        XCTAssertEqual(stops, 2)
    }

    func testOldTalkExitAcknowledgementCannotApproveExitAfterNewSubmission() async {
        var acknowledgement: CheckedContinuation<OperatorSubmission, Never>?
        let model = ConversationViewModel(submit: { _ in .accepted }, stop: {
            await withCheckedContinuation { acknowledgement = $0 }
        })
        let leaving = Task { await model.leaveTalk() }
        while acknowledgement == nil { await Task.yield() }
        await model.submitFinalSpeech("follow me")
        acknowledgement?.resume(returning: .accepted)
        let left = await leaving.value
        XCTAssertFalse(left, "A later command needs its own stop acknowledgement before handoff")
    }

    func testConcurrentTalkExitNotificationsShareOneStopAcknowledgement() async {
        var acknowledgement: CheckedContinuation<OperatorSubmission, Never>?
        var stops = 0
        let model = ConversationViewModel(stop: {
            stops += 1
            return await withCheckedContinuation { acknowledgement = $0 }
        })
        let first = Task { await model.leaveTalk() }
        while acknowledgement == nil { await Task.yield() }
        let second = Task { await model.leaveTalk() }
        await Task.yield()
        acknowledgement?.resume(returning: .accepted)
        let firstLeft = await first.value
        let secondLeft = await second.value
        XCTAssertTrue(firstLeft)
        XCTAssertTrue(secondLeft)
        XCTAssertEqual(stops, 1)
    }

    func testFinalizedSpeechPassesItsReceiptTimeBeforeAsynchronousRouting() async {
        var now = 42.0
        var receivedText: String?
        var receivedAt: TimeInterval?
        var legacySubmissions = 0
        let model = ConversationViewModel()
        model.configure(submit: { _ in legacySubmissions += 1; return .accepted },
            stop: { .accepted }, followState: { .idle }, submitFinalized: { text, time in
                receivedText = text
                receivedAt = time
                now = 42.3
                await Task.yield()
                return .accepted
            }, monotonic: { now })
        await model.submitFinalSpeech(" Follow Me! ")
        XCTAssertEqual(receivedText, " Follow Me! ")
        XCTAssertEqual(receivedAt, 42)
        XCTAssertEqual(legacySubmissions, 0)
        XCTAssertNil(model.errorMessage)
        XCTAssertEqual(model.missionPhase, .idle)
    }

    func testClearanceLabelUsesConfiguredGateRoundedUpIncludingExactTenth() {
        var gate = 1.41
        let model = ConversationViewModel(followState: { .waitingForClearance }, readySignalClearance: { gate })
        XCTAssertEqual(model.status, "Step back to at least 1.5 m — waiting to signal ready.")
        gate = 1.4
        XCTAssertEqual(model.status, "Step back to at least 1.4 m — waiting to signal ready.")
        model.configure(submit: { _ in .accepted }, stop: { .accepted }, followState: { .waitingForClearance },
                        readySignalClearance: { 2.01 })
        XCTAssertEqual(model.status, "Step back to at least 2.1 m — waiting to signal ready.")
    }
    func testOldClearanceStopCompletionCannotClearNewerSubmissionGuidance() async {
        var acknowledgement: CheckedContinuation<OperatorSubmission, Never>?
        let model = ConversationViewModel(submit: { _ in .rejected("New session guidance") },
            stop: { await withCheckedContinuation { acknowledgement = $0 } }, followState: { .waitingForClearance })
        let stopping = Task { await model.stopFollowing() }
        while acknowledgement == nil { await Task.yield() }
        await model.submitFinalSpeech("follow me")
        acknowledgement?.resume(returning: .accepted)
        await stopping.value
        XCTAssertEqual(model.errorMessage, "New session guidance")
    }
    func testClearanceWaitExplainsStepBackAndLocalStopNeverThinks() async {
        var state = FollowMeState.waitingForClearance
        let model = ConversationViewModel(stop: { state = .stopped; return .accepted }, followState: { state })
        XCTAssertEqual(model.status, "Step back to at least 1.4 m — waiting to signal ready.")
        XCTAssertTrue(model.showsStopFollowing)
        await model.stopFollowing()
        XCTAssertEqual(model.missionPhase, .idle)
        XCTAssertEqual(model.status, "Stopped")
    }
    func testRealAgentOldBrainCallbackCannotResetSecondMissionUIAfterLocalStop() async {
        let firstBrain = HeldConversationBrain()
        let secondBrain = HeldConversationBrain()
        var brain: RoverBrain = firstBrain
        let model = ConversationViewModel()
        let motion = ConversationMissionMotion()
        let agent = MissionAgent(motion: motion, perception: ConversationMissionPerception(),
                                 voice: ConversationMissionVoice(),
                                 phaseDidChange: { model.receiveMissionPhase($0) }) { brain }
        var missions: [Task<Void, Never>] = []
        let stop: () async -> OperatorSubmission = {
            do { try await agent.cancelCurrentMissionAndWait(); return .accepted }
            catch { return .rejected("Rover stop could not be confirmed.") }
        }
        model.configure(submit: { text in
            if OperatorCommandKind.classify(text) == .localStop { return await stop() }
            missions.append(Task { await agent.handle(text) })
            return .accepted
        }, stop: stop, followState: { .idle })
        await model.submitFinalSpeech("first mission")
        while !firstBrain.requested { await Task.yield() }
        await model.submitFinalSpeech("Stop!")
        XCTAssertEqual(motion.stops, 1)
        XCTAssertEqual(model.missionPhase, .idle)
        brain = secondBrain
        await model.submitFinalSpeech("second mission")
        while !secondBrain.requested { await Task.yield() }
        firstBrain.release()
        await missions[0].value
        XCTAssertEqual(model.missionPhase, .thinking)
        secondBrain.release()
        await missions[1].value
        XCTAssertEqual(model.missionPhase, .idle, "Valid current idle must reach the UI")
    }

    func testLocalStopNeverThinksAndResetsUIBeforeAndAfterAcknowledgement() async {
        var phases: [MissionAgent.Phase] = []
        var inhibits = 0
        var model: ConversationViewModel!
        model = ConversationViewModel(submit: { _ in
            phases.append(model.missionPhase)
            model.receiveMissionPhase(.thinking)
            phases.append(model.missionPhase)
            return .accepted
        }, stop: { phases.append(model.missionPhase); return .accepted },
           inhibit: { inhibits += 1 })
        model.receiveMissionPhase(.thinking)
        await model.submitFinalSpeech(" STOP FOLLOWING ME! ")
        XCTAssertEqual(phases, [.idle, .idle])
        XCTAssertEqual(model.missionPhase, .idle)
        XCTAssertEqual(inhibits, 1)
        await model.submitFinalSpeech("go to the kitchen")
        XCTAssertEqual(model.missionPhase, .thinking, "Ordinary missions retain Thinking")
        await model.stopFollowing()
        XCTAssertEqual(model.missionPhase, .idle)
        XCTAssertEqual(inhibits, 2)
    }

    func testStationaryAcquisitionPhasesExplainBehaviorAndKeepStopAvailable() async {
        for (state, label) in [(FollowMeState.pausing, "Pausing — five seconds"),
                               (.aligning, "Aligning toward you…"),
                               (.signalingReady, "Signaling ready — moving 10 cm…"),
                               (.waitingForMovement, "Ready — walk away to begin following")] {
            var stops = 0
            let model = ConversationViewModel(stop: { stops += 1; return .accepted }, followState: { state })
            XCTAssertEqual(model.status, label)
            XCTAssertTrue(model.showsStopFollowing)
            await model.stopFollowing()
            XCTAssertEqual(stops, 1)
        }
    }

    func testFinalSpeechRoutesFollowMeWithoutTextEntry() async {
        var received: [String] = []
        let model = ConversationViewModel(submit: { text in
            received.append(text)
            return .accepted
        }, stop: { .accepted }, followState: { .idle })

        await model.submitFinalSpeech("follow me")

        XCTAssertEqual(received, ["follow me"])
        XCTAssertNil(model.errorMessage)
    }

    func testRejectedFinalSpeechDisplaysGuidanceWhileStopRemainsAvailable() async {
        let model = ConversationViewModel(submit: { _ in .rejected("Stop following first.") },
                                          stop: { .accepted }, followState: { .searching })
        await model.submitFinalSpeech("go to the kitchen")
        XCTAssertEqual(model.errorMessage, "Stop following first.")
        XCTAssertTrue(model.showsStopFollowing)
    }

    func testLeavingTalkInhibitsFollowBeforeWaitingForStop() async {
        var events: [String] = []
        let model = ConversationViewModel(submit: { _ in .accepted },
                                          stop: { events.append("confirmed stop"); return .accepted },
                                          followState: { .following },
                                          inhibit: { events.append("inhibited") })
        let stopped = await model.leaveTalk()
        XCTAssertTrue(stopped)
        XCTAssertEqual(events, ["inhibited", "confirmed stop"])
    }

    func testLeavingTalkStopsAnOrdinaryMissionEvenWithoutActiveFollow() async {
        var stopped = false
        let model = ConversationViewModel(submit: { _ in .accepted },
                                          stop: { stopped = true; return .accepted },
                                          followState: { .idle })
        let confirmed = await model.leaveTalk()
        XCTAssertTrue(confirmed)
        XCTAssertTrue(stopped)
    }

    func testFollowStartPermissionRejectsOtherTabMotion() {
        XCTAssertTrue(RootView.followMayStart(navigationState: .idle, silentSearchPhase: .setup))
        XCTAssertFalse(RootView.followMayStart(navigationState: .driving, silentSearchPhase: .setup))
        XCTAssertFalse(RootView.followMayStart(navigationState: .idle, silentSearchPhase: .searching))
    }

    func testStopButtonStaysVisibleWhenMotorStopWasNotConfirmed() {
        for message in ["Rover stop could not be confirmed.", "Motor stop could not be confirmed. Motion is blocked."] {
            let model = ConversationViewModel(followState: { .failed(message) })
            XCTAssertTrue(model.showsStopFollowing)
        }
    }
}

@MainActor
private final class HeldConversationBrain: RoverBrain {
    private(set) var requested = false
    private var response: CheckedContinuation<BrainOutput, Never>?
    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        requested = true
        return await withCheckedContinuation { response = $0 }
    }
    func release() { response?.resume(returning: BrainOutput(decision: .done)); response = nil }
}

@MainActor
private final class ConversationMissionMotion: RoverMotion {
    var state: NavigationController.State = .idle
    private(set) var stops = 0
    func navigate(to goal: Vec2) { XCTFail("No motion expected in phase callback regression") }
    func rotate(by angle: Double) async { XCTFail("No motion expected in phase callback regression") }
    func cancel() { state = .idle }
    func stopAndConfirm() async throws { stops += 1; cancel() }
}

@MainActor
private final class ConversationMissionPerception: RoverPerception {
    var pose: Pose2D? { Pose2D(position: .zero, yaw: 0) }
    func detectObjects() -> [PerceivedObject] { [] }
    func unproject(normalizedPoint: CGPoint) -> Vec2? { nil }
    func capturedFrameJPEG() -> Data? { nil }
}

@MainActor
private final class ConversationMissionVoice: RoverVoice {
    func speak(_ text: String) {}
    func ask(_ question: String, timeout: TimeInterval) async -> String? { nil }
}

@MainActor
private final class HeldRouterMission: OperatorMission {
    let onStop: () async throws -> Void
    init(onStop: @escaping () async throws -> Void) { self.onStop = onStop }
    func handle(_ text: String) async {}
    func cancelCurrentMissionAndWait() async throws { try await onStop() }
}

@MainActor
private final class IdleRouterFollow: OperatorFollow {
    var state: FollowMeState { .idle }
    func start() async -> Bool { true }
    func stop() async -> Bool { true }
    func inhibitMotion() {}
}
