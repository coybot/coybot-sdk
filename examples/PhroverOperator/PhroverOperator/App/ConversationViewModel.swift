import Foundation
import PhroverKit

@Observable
@MainActor
final class ConversationViewModel {
    private(set) var errorMessage: String? {
        didSet { if errorMessage != nil { errorOccurrence &+= 1 } }
    }
    private(set) var speechErrorMessage: String?
    private var activeSpeechCapture: SpeechCaptureID?
    private var errorOccurrence: UInt64 = 0
    private(set) var missionPhase: MissionAgent.Phase = .idle
    private var acceptsMissionPhase = true
    private var submissionGeneration: UInt64 = 0
    private var exitConfirmed = false
    private var exitPrepared = false
    private var exitTask: (generation: UInt64, task: Task<OperatorSubmission, Never>)?
    private var submit: (String) async -> OperatorSubmission
    private var stop: () async -> OperatorSubmission
    private var followState: () -> FollowMeState
    private var targetFrame: () -> ARFrameID? = { nil }
    private var readySignalClearance: () -> Double
    private var inhibit: () -> Void
    private var submitFinalized: ((String, TimeInterval) async -> OperatorSubmission)?
    private var monotonic: () -> TimeInterval = { ProcessInfo.processInfo.systemUptime }
    private var announcedErrors: Set<String> = []
    private var announcedOccurrence: UInt64 = 0

    struct ErrorAnnouncement: Equatable {
        let messages: [String]
        let occurrence: UInt64
    }

    var errorAnnouncement: ErrorAnnouncement {
        .init(messages: displayedErrors, occurrence: errorOccurrence)
    }

    /// Read the same failure sources the Talk screen displays. Following can
    /// fail asynchronously, after the initial command submission was accepted.
    var displayedErrors: [String] {
        var messages: [String] = []
        if case .failed(let message) = followState() { messages.append(message) }
        if case .waitingForReturn(let message) = followState() { messages.append(message) }
        if let errorMessage, !messages.contains(errorMessage) { messages.append(errorMessage) }
        if let speechErrorMessage, !messages.contains(speechErrorMessage) { messages.append(speechErrorMessage) }
        return messages.filter { !$0.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty }
    }

    func announceDisplayedErrors(cancel: () -> Void = {}, using announce: (String) -> Void) {
        let messages = displayedErrors
        let current = Set(messages)
        guard current != announcedErrors || (!current.isEmpty && errorOccurrence != announcedOccurrence) else { return }
        announcedErrors = current
        announcedOccurrence = errorOccurrence
        // Replace the whole pending error queue when the displayed set changes.
        // Otherwise interrupting an old utterance could silently drop another
        // still-visible error that was queued but never heard.
        cancel()
        for message in messages { announce(message) }
    }

    func resetErrorAnnouncements() { announcedErrors = [] }

    func receiveSpeechCaptureEvent(_ event: SpeechCaptureEvent) {
        switch event {
        case .started(let id):
            activeSpeechCapture = id
            speechErrorMessage = nil
            // Capture is admitted only after playback drains. Removing its old
            // recognition error must not replay completed errors and cancel this
            // new capture. Do not mark any newly arrived error as announced.
            announcedErrors.formIntersection(Set(displayedErrors))
        case .partial: break
        case .failed(let id, let message):
            guard activeSpeechCapture == id else { return }
            activeSpeechCapture = nil
            recordSpeechFailure(message)
        }
    }

    func finalizeSpeechCapture(_ id: SpeechCaptureID) {
        if activeSpeechCapture == id { activeSpeechCapture = nil }
    }

    func recordSpeechFailure(_ message: String) {
        activeSpeechCapture = nil
        speechErrorMessage = message
        errorOccurrence &+= 1
    }

    init(submit: @escaping (String) async -> OperatorSubmission = { _ in .rejected("Rover is starting. Try again.") },
         stop: @escaping () async -> OperatorSubmission = { .accepted },
         followState: @escaping () -> FollowMeState = { .idle },
         readySignalClearance: @escaping () -> Double = { FollowMeConfiguration().readySignalClearance },
         inhibit: @escaping () -> Void = {},
         submitFinalized: ((String, TimeInterval) async -> OperatorSubmission)? = nil,
         monotonic: @escaping () -> TimeInterval = { ProcessInfo.processInfo.systemUptime }) {
        self.submit = submit
        self.stop = stop
        self.followState = followState
        self.readySignalClearance = readySignalClearance
        self.inhibit = inhibit
        self.submitFinalized = submitFinalized
        self.monotonic = monotonic
    }

    func configure(submit: @escaping (String) async -> OperatorSubmission,
                   stop: @escaping () async -> OperatorSubmission,
                   followState: @escaping () -> FollowMeState,
                   targetFrame: @escaping () -> ARFrameID? = { nil },
                   readySignalClearance: @escaping () -> Double = { FollowMeConfiguration().readySignalClearance },
                   inhibit: @escaping () -> Void = {},
                   submitFinalized: ((String, TimeInterval) async -> OperatorSubmission)? = nil,
                   monotonic: @escaping () -> TimeInterval = { ProcessInfo.processInfo.systemUptime }) {
        self.inhibit()
        submissionGeneration &+= 1
        exitConfirmed = false
        exitPrepared = false
        exitTask = nil
        self.submit = submit
        self.stop = stop
        self.followState = followState
        self.targetFrame = targetFrame
        self.readySignalClearance = readySignalClearance
        self.inhibit = inhibit
        self.submitFinalized = submitFinalized
        self.monotonic = monotonic
    }

    var showsStopFollowing: Bool {
        if followState().isActive { return true }
        if case .failed("Rover stop could not be confirmed.") = followState() { return true }
        if case .failed("Motor stop could not be confirmed. Motion is blocked.") = followState() { return true }
        return errorMessage == "Rover stop could not be confirmed."
    }
    var trackedPersonFrameID: ARFrameID? { targetFrame() }
    var status: String {
        switch followState() {
        case .idle: return ""
        case .pausing: return "Pausing — five seconds"
        case .aligning: return "Aligning toward you…"
        case .waitingForClearance:
            let requirement = ceil(readySignalClearance() * 10) / 10
            return "Step back to at least \(String(format: "%.1f", requirement)) m — waiting to signal ready."
        case .signalingReady: return "Signaling ready — moving 10 cm…"
        case .waitingForMovement: return "Ready — walk away to begin following"
        case .searching: return "Searching for you…"
        case .recoveringReadyPath: return "Checking floor — probing, backing up, or finding clear space…"
        case .following: return "Following — 1.5 m"
        case .holdingDistance: return "Holding distance"
        case .reacquiring: return "Person lost — searching…"
        case .waitingForReturn(let message): return message
        case .stopped: return "Stopped"
        case .failed(let message): return message
        }
    }

    func submitFinalSpeech(_ text: String) async {
        // Receipt of finalized recognized text, not acoustic utterance onset.
        let receivedAt = monotonic()
        exitConfirmed = false
        exitPrepared = false
        exitTask = nil
        submissionGeneration &+= 1
        let generation = submissionGeneration
        let kind = OperatorCommandKind.classify(text)
        acceptsMissionPhase = kind == .mission
        missionPhase = kind == .mission ? .thinking : .idle
        if kind == .localStop { inhibit() }
        let result: OperatorSubmission
        if let submitFinalized { result = await submitFinalized(text, receivedAt) }
        else { result = await submit(text) }
        guard generation == submissionGeneration else { return }
        switch result {
        case .accepted:
            errorMessage = nil
            if kind != .mission { missionPhase = .idle }
        case .rejected(let message):
            errorMessage = message
            missionPhase = .idle
            acceptsMissionPhase = false
        }
    }

    func stopFollowing() async {
        submissionGeneration &+= 1
        let generation = submissionGeneration
        acceptsMissionPhase = false
        missionPhase = .idle
        inhibit()
        let result = await stop()
        guard generation == submissionGeneration else { return }
        switch result {
        case .accepted: errorMessage = nil
        case .rejected(let message): errorMessage = message
        }
    }

    func receiveMissionPhase(_ phase: MissionAgent.Phase) {
        if acceptsMissionPhase { missionPhase = phase }
    }

    func prepareToLeave() {
        guard !exitConfirmed, !exitPrepared else { return }
        submissionGeneration &+= 1
        exitPrepared = true
        inhibit()
    }

    @discardableResult
    func leaveTalk() async -> Bool {
        guard !exitConfirmed else { return true }
        prepareToLeave()
        let pending: (generation: UInt64, task: Task<OperatorSubmission, Never>)
        if let exitTask {
            pending = exitTask
        } else {
            submissionGeneration &+= 1
            acceptsMissionPhase = false
            missionPhase = .idle
            let stop = self.stop
            pending = (submissionGeneration, Task { await stop() })
            exitTask = pending
        }
        let result = await pending.task.value
        // A new command/configuration cannot inherit an old exit acknowledgement.
        guard submissionGeneration == pending.generation else { return false }
        exitTask = nil
        switch result {
        case .accepted:
            exitConfirmed = true
            return true
        case .rejected(let message):
            exitPrepared = false
            errorMessage = message
            return false
        }
    }
}

/// Shares the lifetime of the view's detector, agent and speech objects.
@Observable
@MainActor
final class ConversationRuntimeLifetime {
    private(set) var runtimeReady = false
    private var runtimeInitialization: UUID?
    func beginRuntimeInitialization() -> UUID? {
        guard !runtimeReady, runtimeInitialization == nil else { return nil }
        let token = UUID()
        runtimeInitialization = token
        return token
    }
    func completeRuntimeInitialization(_ token: UUID) -> Bool {
        guard runtimeInitialization == token, !runtimeReady else { return false }
        runtimeInitialization = nil
        runtimeReady = true
        return true
    }
    func abandonRuntimeInitialization(_ token: UUID) {
        if runtimeInitialization == token { runtimeInitialization = nil }
    }
    func invalidateRuntimeInitialization() { runtimeInitialization = nil }

    private(set) var speechEpoch: UInt64 = 0
    func invalidateSpeechCapture() { speechEpoch &+= 1 }
    func acceptsSpeechCapture(from epoch: UInt64) -> Bool { speechEpoch == epoch }

    func beginSpeechCapture(playbackActive: Bool) -> UInt64? {
        guard !playbackActive else { return nil }
        invalidateSpeechCapture()
        return speechEpoch
    }

    func prepareForSpeechPlayback(stopCapture: () -> Void) {
        invalidateSpeechCapture()
        stopCapture()
    }
}
