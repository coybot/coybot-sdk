import Foundation
import Speech
import AVFoundation

public struct SpeechCaptureID: RawRepresentable, Equatable, Hashable, Sendable {
    public let rawValue: UInt64

    public init(rawValue: UInt64) {
        self.rawValue = rawValue
    }
}

public enum SpeechCaptureEvent: Equatable, Sendable {
    case started(id: SpeechCaptureID)
    case partial(id: SpeechCaptureID, transcript: String)
    case failed(id: SpeechCaptureID, message: String)
}

@MainActor
final class SpeechCaptureLifecycle {
    static let noSpeechMessage = "No speech detected. Try again."

    private var nextID: UInt64 = 0
    private(set) var activeID: SpeechCaptureID?
    private var eventHandler: ((SpeechCaptureEvent) -> Void)?
    private var finalHandler: ((SpeechCaptureID, String) -> Void)?

    @discardableResult
    func begin(
        onEvent: @escaping (SpeechCaptureEvent) -> Void,
        onFinal: @escaping (SpeechCaptureID, String) -> Void
    ) -> SpeechCaptureID {
        cancelActive()
        nextID += 1
        let id = SpeechCaptureID(rawValue: nextID)
        activeID = id
        eventHandler = onEvent
        finalHandler = onFinal
        onEvent(.started(id: id))
        return id
    }

    func receivePartial(_ transcript: String, for id: SpeechCaptureID) {
        guard activeID == id else { return }
        guard let transcript = Self.normalizedTranscript(transcript) else { return }
        eventHandler?(.partial(id: id, transcript: transcript))
    }

    func complete(_ transcript: String, for id: SpeechCaptureID) {
        guard activeID == id else { return }
        guard let transcript = Self.normalizedTranscript(transcript) else {
            fail(id: id, message: Self.noSpeechMessage)
            return
        }
        let handler = finalHandler
        close(id)
        handler?(id, transcript)
    }

    func fail(id: SpeechCaptureID, message: String) {
        guard activeID == id else { return }
        let handler = eventHandler
        close(id)
        handler?(.failed(id: id, message: message))
    }

    func fail(id: SpeechCaptureID, recognitionError: Error) {
        fail(id: id, message: Self.failureMessage(for: recognitionError))
    }

    func cancelActive() {
        guard let activeID else { return }
        close(activeID)
    }

    private func close(_ id: SpeechCaptureID) {
        guard activeID == id else { return }
        activeID = nil
        eventHandler = nil
        finalHandler = nil
    }

    static func normalizedTranscript(_ transcript: String) -> String? {
        let transcript = transcript.trimmingCharacters(in: .whitespacesAndNewlines)
        return transcript.isEmpty ? nil : transcript
    }

    private static func failureMessage(for error: Error) -> String {
        let error = error as NSError
        let domain = error.domain.lowercased()
        let isAppleNoSpeechCode = error.code == 1110
            && (domain.contains("assistant") || domain.contains("speech"))
        let describesNoSpeech = error.localizedDescription
            .localizedCaseInsensitiveContains("no speech")
        return isAppleNoSpeechCode || describesNoSpeech
            ? noSpeechMessage
            : "Speech recognition failed. Try again."
    }
}

/// On-device speech-to-text. `requiresOnDeviceRecognition = true` so the rover keeps
/// understanding its operator with no WiFi — matches the all-on-device, offline-first
/// voice stack.
@Observable
@MainActor
public final class SpeechIn {
    public enum State: Equatable { case idle, listening, processing, unavailable }

    public private(set) var state: State = .idle
    public private(set) var partialTranscript = ""

    private let recognizer = SFSpeechRecognizer(locale: Locale(identifier: "en-US"))
    private let audioEngine = AVAudioEngine()
    private var request: SFSpeechAudioBufferRecognitionRequest?
    private var task: SFSpeechRecognitionTask?
    private var tapInstalled = false
    private var isStarting = false
    private var captureID: SpeechCaptureID?
    private let captureLifecycle = SpeechCaptureLifecycle()
    private let finalizationWatchdog: SpeechFinalizationWatchdog
    private let audioSessionController: SpeechAudioSessionController
    private let audioInputMetrics = SpeechAudioInputMetrics()

    public init(finalizationTimeout: Duration = .seconds(3)) {
        finalizationWatchdog = SpeechFinalizationWatchdog(timeout: finalizationTimeout)
        audioSessionController = SpeechAudioSessionController()
    }

    public nonisolated func requestAuthorization() async -> Bool {
        let speechStatus = await withCheckedContinuation { c in
            SFSpeechRecognizer.requestAuthorization { status in
                c.resume(returning: status)
            }
        }
        guard speechStatus == .authorized else { return false }

        let microphoneGranted = await withCheckedContinuation { c in
            AVAudioApplication.requestRecordPermission { granted in
                c.resume(returning: granted)
            }
        }
        return SpeechAuthorizationPolicy.isAuthorized(
            speechStatus: speechStatus,
            microphoneGranted: microphoneGranted
        )
    }

    /// Start listening; invokes `onFinal` once a completed utterance is recognized.
    /// Push-to-talk is the intended usage — always-on listening risks false wake triggers
    /// in noisy environments.
    public func start(
        onEvent: @escaping (SpeechCaptureEvent) -> Void,
        onFinal: @escaping (SpeechCaptureID, String) -> Void
    ) throws {
        guard !isStarting, state != .listening else { return }
        guard let recognizer, recognizer.isAvailable else {
            state = .unavailable
            RuntimeFileLog.append("speech_capture_start_failed", fields: [
                "reason": "recognizer_unavailable"
            ])
            throw SpeechError.recognizerUnavailable
        }
        isStarting = true
        defer { isStarting = false }

        stop()
        partialTranscript = ""
        audioInputMetrics.reset()

        do {
            try audioSessionController.activate()
            RuntimeFileLog.append(
                "speech_audio_session_activated",
                fields: Self.audioSessionLogFields()
            )
        } catch {
            RuntimeFileLog.append("speech_capture_start_failed", fields: [
                "reason": "audio_session_activation_failed",
                "error": String(describing: error),
            ])
            finishRecognition(cancelTask: true)
            throw error
        }

        let req = SFSpeechAudioBufferRecognitionRequest()
        req.shouldReportPartialResults = true
        req.requiresOnDeviceRecognition = true
        request = req

        Self.installTap(
            on: audioEngine.inputNode,
            request: req,
            metrics: audioInputMetrics
        )
        tapInstalled = true
        audioEngine.prepare()
        do {
            try audioEngine.start()
        } catch {
            RuntimeFileLog.append("speech_capture_start_failed", fields: [
                "error": String(describing: error)
            ])
            finishRecognition(cancelTask: true)
            throw error
        }
        state = .listening
        let captureID = captureLifecycle.begin(onEvent: onEvent, onFinal: onFinal)
        self.captureID = captureID
        RuntimeFileLog.append("speech_capture_started", fields: [
            "capture_id": String(captureID.rawValue)
        ])

        task = recognizer.recognitionTask(with: req) { [weak self] result, error in
            guard let self else { return }
            Task { @MainActor in
                guard self.captureLifecycle.activeID == captureID else { return }
                if let result {
                    let latestTranscript = result.bestTranscription.formattedString
                    let trimmedTranscript = latestTranscript
                        .trimmingCharacters(in: .whitespacesAndNewlines)
                    if !trimmedTranscript.isEmpty {
                        self.partialTranscript = trimmedTranscript
                        self.captureLifecycle.receivePartial(trimmedTranscript, for: captureID)
                    }
                    if result.isFinal {
                        let finalTranscript = SpeechTranscriptSelection.finalTranscript(
                            latest: latestTranscript,
                            lastNonEmptyPartial: self.partialTranscript
                        ) ?? ""
                        self.completeRecognition(
                            with: finalTranscript,
                            captureID: captureID,
                            source: latestTranscript.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
                                ? "partial_after_blank_final"
                                : "recognizer_final"
                        )
                        return
                    }
                }
                if let error {
                    let recognitionError = error as NSError
                    RuntimeFileLog.append("speech_capture_failed", fields: [
                        "error": String(describing: error),
                        "error_domain": recognitionError.domain,
                        "error_code": String(recognitionError.code)
                    ])
                    self.captureLifecycle.fail(id: captureID, recognitionError: error)
                    self.finishRecognition(cancelTask: false, closeCapture: false)
                }
            }
        }
    }

    public func start(onFinal: @escaping (String) -> Void) throws {
        try start(onEvent: { _ in }) { _, transcript in
            onFinal(transcript)
        }
    }

    /// Listen for a single utterance and return it, or `nil` on timeout / recognizer
    /// failure. Convenience over `start(onFinal:)` for a mission agent's ask-then-listen
    /// turns, where "no answer" must be a normal, handled outcome rather than an error.
    public func listenOnce(timeout: TimeInterval) async -> String? {
        await withCheckedContinuation { (continuation: CheckedContinuation<String?, Never>) in
            var didResume = false
            let resumeOnce: (String?) -> Void = { [weak self] text in
                guard !didResume else { return }
                didResume = true
                self?.finishRecognition(cancelTask: true)
                continuation.resume(returning: text)
            }

            do {
                try start { text in resumeOnce(text) }
            } catch {
                resumeOnce(nil)
                return
            }

            Task {
                try? await Task.sleep(for: .seconds(timeout))
                resumeOnce(nil)
            }
        }
    }

    public func stop() {
        finishRecognition(cancelTask: true)
    }

    /// End microphone capture and let Speech deliver the final transcript. This is the
    /// push-to-talk release path; cancelling here would drop the command before the agent
    /// can interpret it.
    public func finish() {
        guard state == .listening, let captureID else { return }
        finishAudioInput()
        request?.endAudio()
        state = .processing
        RuntimeFileLog.append("speech_capture_finish_requested", fields: [
            "partial": partialTranscript
        ])
        finalizationWatchdog.schedule(
            partialTranscript: { [weak self] in self?.partialTranscript ?? "" },
            completion: { [weak self] transcript in
                guard let self else { return }
                guard let transcript else {
                    RuntimeFileLog.append("speech_capture_timed_out", fields: [
                        "reason": "no_speech"
                    ])
                    self.captureLifecycle.fail(
                        id: captureID,
                        message: SpeechCaptureLifecycle.noSpeechMessage
                    )
                    self.finishRecognition(cancelTask: true, closeCapture: false)
                    return
                }
                RuntimeFileLog.append("speech_capture_timeout_fallback", fields: [
                    "transcript": transcript
                ])
                self.completeRecognition(
                    with: transcript,
                    captureID: captureID,
                    source: "partial_timeout"
                )
            }
        )
    }

    private func finishRecognition(cancelTask: Bool, closeCapture: Bool = true) {
        finalizationWatchdog.cancel()
        finishAudioInput()
        request?.endAudio()
        if cancelTask { task?.cancel() }
        task = nil
        request = nil
        if closeCapture { captureLifecycle.cancelActive() }
        captureID = nil
        isStarting = false
        logAudioInputMetrics()
        do {
            try audioSessionController.deactivate()
        } catch {
            RuntimeFileLog.append("speech_audio_session_deactivation_failed", fields: [
                "error": String(describing: error),
            ])
        }
        if state != .unavailable { state = .idle }
    }

    private func completeRecognition(
        with transcript: String,
        captureID: SpeechCaptureID,
        source: String
    ) {
        guard state == .listening || state == .processing else { return }
        guard captureLifecycle.activeID == captureID else { return }
        guard let normalizedTranscript = SpeechCaptureLifecycle.normalizedTranscript(transcript) else {
            RuntimeFileLog.append("speech_capture_failed", fields: [
                "reason": "blank_final_transcript",
                "source": source
            ])
            captureLifecycle.complete(transcript, for: captureID)
            finishRecognition(cancelTask: false, closeCapture: false)
            return
        }

        RuntimeFileLog.append("speech_capture_completed", fields: [
            "capture_id": String(captureID.rawValue),
            "source": source,
            "transcript": normalizedTranscript
        ])
        finishRecognition(cancelTask: false, closeCapture: false)
        captureLifecycle.complete(normalizedTranscript, for: captureID)
    }

    private func finishAudioInput() {
        if tapInstalled {
            audioEngine.inputNode.removeTap(onBus: 0)
            tapInstalled = false
        }
        if audioEngine.isRunning { audioEngine.stop() }
    }

    private nonisolated static func installTap(
        on node: AVAudioInputNode,
        request: SFSpeechAudioBufferRecognitionRequest,
        metrics: SpeechAudioInputMetrics
    ) {
        let format = node.outputFormat(forBus: 0)
        node.installTap(onBus: 0, bufferSize: 1024, format: format) { buffer, _ in
            metrics.observe(buffer)
            request.append(buffer)
        }
    }

    private func logAudioInputMetrics() {
        let snapshot = audioInputMetrics.snapshot()
        guard snapshot.bufferCount > 0 || audioSessionController.isActive else { return }
        let decibels = 20 * log10(max(snapshot.maxRMS, 0.000_000_1))
        RuntimeFileLog.append("speech_audio_input_completed", fields: [
            "buffer_count": String(snapshot.bufferCount),
            "frame_count": String(snapshot.frameCount),
            "max_rms_db": String(format: "%.1f", decibels),
        ])
        audioInputMetrics.reset()
    }

    private nonisolated static func audioSessionLogFields() -> [String: String] {
        let session = AVAudioSession.sharedInstance()
        let inputs = session.currentRoute.inputs
            .map { "\($0.portType.rawValue):\($0.portName.replacingOccurrences(of: " ", with: "_"))" }
            .joined(separator: ",")
        return [
            "input_available": String(session.isInputAvailable),
            "input_muted": String(AVAudioApplication.shared.isInputMuted),
            "input_route": inputs.isEmpty ? "none" : inputs,
            "sample_rate": String(format: "%.0f", session.sampleRate),
            "input_channels": String(session.inputNumberOfChannels),
        ]
    }
}

public enum SpeechError: Error { case recognizerUnavailable }
