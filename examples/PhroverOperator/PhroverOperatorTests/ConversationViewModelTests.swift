import XCTest
@testable import PhroverOperator
@testable import PhroverKit
import RoverNav

@MainActor
final class ConversationViewModelTests: XCTestCase {
    func testReturnGuidanceIsSpokenOnceAndClearedOnRestart() {
        let guidance = "Come back into view and stand still."
        var state = FollowMeState.waitingForReturn(guidance)
        let model = ConversationViewModel(followState: { state })
        var spoken: [String] = []
        model.announceDisplayedErrors { spoken.append($0) }
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertEqual(spoken, [guidance])
        XCTAssertEqual(model.status, guidance)
        XCTAssertTrue(model.showsStopFollowing)
        state = .pausing
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertTrue(model.displayedErrors.isEmpty)
        state = .waitingForReturn(guidance)
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertEqual(spoken, [guidance, guidance])
    }

    func testEachRejectedOperatorActionAnnouncesEvenWhenTextIsUnchanged() async {
        let message = "Rover stop could not be confirmed."
        let model = ConversationViewModel(submit: { _ in .rejected(message) }, stop: { .rejected(message) })
        var spoken: [String] = []
        for _ in 0..<2 {
            await model.submitFinalSpeech("follow me")
            model.announceDisplayedErrors { spoken.append($0) }
            model.announceDisplayedErrors { spoken.append($0) }
        }
        for _ in 0..<2 {
            await model.stopFollowing()
            model.announceDisplayedErrors { spoken.append($0) }
            model.announceDisplayedErrors { spoken.append($0) }
        }
        XCTAssertEqual(spoken, Array(repeating: message, count: 4))
    }

    func testRecognitionFailuresAnnouncePerCaptureAndIgnoreDuplicateOrOldEvents() {
        let model = ConversationViewModel()
        var spoken: [String] = []
        for sequence: UInt64 in 1...2 {
            let id = SpeechCaptureID(rawValue: sequence)
            model.receiveSpeechCaptureEvent(.started(id: id))
            model.receiveSpeechCaptureEvent(.failed(id: .init(rawValue: 99), message: "Old failure"))
            XCTAssertNil(model.speechErrorMessage)
            let before = model.errorAnnouncement
            model.receiveSpeechCaptureEvent(.failed(id: id, message: "No speech detected. Try again."))
            XCTAssertNotEqual(model.errorAnnouncement, before)
            model.announceDisplayedErrors { spoken.append($0) }
            model.receiveSpeechCaptureEvent(.failed(id: id, message: "No speech detected. Try again."))
            model.announceDisplayedErrors { spoken.append($0) }
        }
        XCTAssertEqual(spoken, Array(repeating: "No speech detected. Try again.", count: 2))
        model.receiveSpeechCaptureEvent(.started(id: .init(rawValue: 3)))
        model.finalizeSpeechCapture(.init(rawValue: 3))
        model.receiveSpeechCaptureEvent(.failed(id: .init(rawValue: 3), message: "Late failure"))
        XCTAssertNil(model.speechErrorMessage)
    }

    func testRecognizerUnavailableStartFailureIsDisplayedAndAnnouncedOnEveryAttempt() {
        let model = ConversationViewModel()
        var spoken: [String] = []
        let message = "Speech recognition unavailable. Try again."
        model.recordSpeechFailure(message)
        let first = model.errorAnnouncement
        model.announceDisplayedErrors { spoken.append($0) }
        model.recordSpeechFailure(message)
        XCTAssertNotEqual(model.errorAnnouncement, first, "SwiftUI must observe a new occurrence despite identical text")
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertEqual(model.speechErrorMessage, message)
        XCTAssertEqual(spoken, [message, message])
    }

    func testAnnouncementFencesQueuedTranscriptBeforeStoppingMicrophone() throws {
        let lifetime = ConversationRuntimeLifetime()
        let capture = try XCTUnwrap(lifetime.beginSpeechCapture(playbackActive: false))
        var microphoneStopped = false
        lifetime.prepareForSpeechPlayback {
            XCTAssertFalse(lifetime.acceptsSpeechCapture(from: capture), "Even a final result queued before playback must be discarded")
            microphoneStopped = true
        }
        XCTAssertTrue(microphoneStopped)
        XCTAssertNil(lifetime.beginSpeechCapture(playbackActive: true), "Do not record queued or playing announcements")
        let freshCapture = try XCTUnwrap(lifetime.beginSpeechCapture(playbackActive: false))
        XCTAssertTrue(lifetime.acceptsSpeechCapture(from: freshCapture))
        XCTAssertFalse(lifetime.acceptsSpeechCapture(from: capture))
    }

    func testClearingRecognitionErrorOnRetryDoesNotReplayCompletedFollowError() throws {
        let model = ConversationViewModel(followState: { .failed("Person lost.") })
        let lifetime = ConversationRuntimeLifetime()
        let first = SpeechCaptureID(rawValue: 1)
        model.receiveSpeechCaptureEvent(.started(id: first))
        model.receiveSpeechCaptureEvent(.failed(id: first, message: "No speech detected. Try again."))
        model.announceDisplayedErrors { _ in }
        // Playback completed; the operator is now permitted to retry capture.
        let retryEpoch = try XCTUnwrap(lifetime.beginSpeechCapture(playbackActive: false))
        model.receiveSpeechCaptureEvent(.started(id: .init(rawValue: 2)))
        model.announceDisplayedErrors { _ in
            lifetime.prepareForSpeechPlayback(stopCapture: {})
            XCTFail("Removing a delivered recognition error must not replay a delivered follow error")
        }
        XCTAssertNil(model.speechErrorMessage)
        XCTAssertTrue(lifetime.acceptsSpeechCapture(from: retryEpoch))
    }

    func testCaptureRetryCannotSuppressANewFollowFailureBeforeViewUpdate() {
        var state = FollowMeState.idle
        let model = ConversationViewModel(followState: { state })
        model.recordSpeechFailure("No speech detected. Try again.")
        model.announceDisplayedErrors { _ in }
        state = .failed("Motor stop could not be confirmed. Motion is blocked.")
        model.receiveSpeechCaptureEvent(.started(id: .init(rawValue: 2)))
        var spoken: [String] = []
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertEqual(spoken, ["Motor stop could not be confirmed. Motion is blocked."])
    }

    func testCommunicationStopEscalationReplacesSpeechAndKeepsRetryControl() {
        var state = FollowMeState.failed("Rover communication lost. Attempting to stop.")
        let model = ConversationViewModel(followState: { state })
        var queue: [String] = []
        model.announceDisplayedErrors(cancel: { queue.removeAll() }) { queue.append($0) }
        XCTAssertEqual(queue, ["Rover communication lost. Attempting to stop."])
        state = .failed("Motor stop could not be confirmed. Motion is blocked.")
        model.announceDisplayedErrors(cancel: { queue.removeAll() }) { queue.append($0) }
        XCTAssertEqual(queue, ["Motor stop could not be confirmed. Motion is blocked."])
        XCTAssertEqual(model.status, queue.first)
        XCTAssertTrue(model.showsStopFollowing)
        model.announceDisplayedErrors(cancel: { queue.removeAll() }) { queue.append($0) }
        XCTAssertEqual(queue.count, 1)
    }

    func testFollowingErrorsAreSpokenOnceAndMoreSevereStopErrorIsAnnounced() {
        var state = FollowMeState.following
        let model = ConversationViewModel(followState: { state })
        var spoken: [String] = []
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertTrue(spoken.isEmpty)
        let depthError = "Following stopped: depth did not become fresh and stable enough to send."
        state = .failed(depthError)
        model.announceDisplayedErrors { spoken.append($0) }
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertEqual(spoken, [depthError])
        XCTAssertEqual(model.status, depthError, "Speech must preserve the on-screen error")
        state = .failed("Motor stop could not be confirmed. Motion is blocked.")
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertEqual(spoken.last, "Motor stop could not be confirmed. Motion is blocked.")
        XCTAssertEqual(spoken.count, 2)
        state = .following
        model.announceDisplayedErrors { spoken.append($0) }
        state = .failed(depthError)
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertEqual(spoken, [depthError, "Motor stop could not be confirmed. Motion is blocked.", depthError])
    }

    func testRejectedCommandsAndStopsSpeakWithoutDuplicatingFollowingError() async {
        let message = "Rover stop could not be confirmed."
        var state = FollowMeState.idle
        let model = ConversationViewModel(submit: { _ in .rejected(message) },
            stop: { .rejected(message) }, followState: { state })
        var spoken: [String] = []
        await model.submitFinalSpeech("follow me")
        model.announceDisplayedErrors { spoken.append($0) }
        state = .failed(message)
        model.announceDisplayedErrors { spoken.append($0) }
        await model.stopFollowing()
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertEqual(spoken, [message, message], "A new explicit failed stop deserves another announcement")
        XCTAssertEqual(model.errorMessage, message)
        XCTAssertTrue(model.showsStopFollowing)
    }

    func testDistinctDisplayedErrorsAreBothAnnouncedAndCanRepeatAfterClearing() async {
        var state = FollowMeState.failed("Person lost.")
        var reject = true
        let model = ConversationViewModel(submit: { _ in reject ? .rejected("Rover is busy.") : .accepted },
            followState: { state })
        var spoken: [String] = []
        await model.submitFinalSpeech("follow me")
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertEqual(spoken, ["Person lost.", "Rover is busy."])
        reject = false
        state = .following
        await model.submitFinalSpeech("follow me")
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertTrue(model.displayedErrors.isEmpty)
        state = .failed("Person lost.")
        model.announceDisplayedErrors { spoken.append($0) }
        XCTAssertEqual(spoken, ["Person lost.", "Rover is busy.", "Person lost."])
    }

    func testErrorQueueReplacesCancelledMessagesAndClearsResolvedErrors() async {
        var state = FollowMeState.failed("Person lost.")
        let model = ConversationViewModel(submit: { _ in .rejected("Rover is busy.") },
            followState: { state })
        var queue: [String] = []
        func refresh() {
            model.announceDisplayedErrors(cancel: { queue.removeAll() }) { queue.append($0) }
        }
        refresh()
        XCTAssertEqual(queue, ["Person lost."])
        await model.submitFinalSpeech("follow me")
        refresh()
        XCTAssertEqual(queue, ["Person lost.", "Rover is busy."], "Interrupted errors must be requeued")
        state = .stopped
        refresh()
        XCTAssertEqual(queue, ["Rover is busy."], "Resolved errors must leave the speech queue")
        queue.removeAll() // View leaves / app backgrounds and cancels playback.
        model.resetErrorAnnouncements()
        refresh()
        XCTAssertEqual(queue, ["Rover is busy."], "Returning can announce an interrupted error again")
    }

    func testFollowSpeechImmediatelyUsesOrdinarySubmission() async {
        var submitted: [String] = []
        var stops = 0
        let model = ConversationViewModel(submit: { submitted.append($0); return .accepted },
                                          stop: { stops += 1; return .accepted })
        await model.submitFinalSpeech("follow me")
        XCTAssertEqual(submitted, ["follow me"])
        XCTAssertEqual(stops, 0, "The ordinary router owns admission and stop confirmation")
    }

    func testRepeatedFollowWhileActiveDoesNotPromptStopOrRenewClearance() async {
        var stops = 0
        var submissions = 0
        let model = ConversationViewModel(submit: { _ in submissions += 1; return .accepted },
            stop: { stops += 1; return .accepted }, followState: { .searching })
        await model.submitFinalSpeech("follow me")
        XCTAssertEqual(submissions, 1)
        XCTAssertEqual(stops, 0)
    }

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
        await model.submitFinalSpeech("go to kitchen") // A later Talk command starts a new exit epoch.
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
        await model.submitFinalSpeech("go to kitchen")
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
            stop: { .accepted }, followState: { .searching }, submitFinalized: { text, time in
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

    func testFinalSpeechRoutesFollowMeWithOriginalFinalizedTimestamp() async {
        var received: [String] = []
        var timestamps: [TimeInterval] = []
        let model = ConversationViewModel(submitFinalized: { text, time in
            received.append(text); timestamps.append(time); return .accepted
        }, monotonic: { 42 })
        await model.submitFinalSpeech("follow me")
        XCTAssertEqual(received, ["follow me"])
        XCTAssertEqual(timestamps, [42])
        XCTAssertNil(model.errorMessage)
    }

    func testStopNewCommandLeaveAndReconfigureFenceOldFollowSubmissionCompletion() async {
        for action in 0..<4 {
            var release: CheckedContinuation<OperatorSubmission, Never>?
            let entered = expectation(description: "follow submission entered \(action)")
            let model = ConversationViewModel(submit: { text in
                if text == "follow me" { return await withCheckedContinuation { release = $0; entered.fulfill() } }
                return .rejected("New command guidance")
            }, stop: { .accepted })
            let starting = Task { await model.submitFinalSpeech("follow me") }
            await fulfillment(of: [entered], timeout: 2)
            guard let release else { starting.cancel(); return }
            defer { starting.cancel() }
            switch action {
            case 0: await model.stopFollowing()
            case 1: await model.submitFinalSpeech("go to the kitchen")
            case 2: model.prepareToLeave()
            default: model.configure(submit: { _ in .accepted }, stop: { .accepted }, followState: { .idle })
            }
            release.resume(returning: .rejected("Stale follow failure"))
            await starting.value
            XCTAssertEqual(model.errorMessage, action == 1 ? "New command guidance" : nil)
        }
    }

    func testUnavailableARGuidanceComesFromOrdinaryFollowAdmission() async {
        let model = ConversationViewModel(submit: { _ in .rejected("Fresh AR tracking is required.") })
        await model.submitFinalSpeech("follow me")
        XCTAssertEqual(model.errorMessage, "Fresh AR tracking is required.")
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
