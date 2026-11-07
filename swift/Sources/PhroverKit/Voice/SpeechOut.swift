import Foundation
import AVFoundation

/// On-device text-to-speech. Chosen over cloud TTS so the rover can always speak —
/// including in WiFi dead spots — at the cost of a more robotic voice than a cloud
/// premium voice would give.
@MainActor
public final class SpeechOut: NSObject, AVSpeechSynthesizerDelegate {
    private let synth: AVSpeechSynthesizer
    private var pendingUtterances: Set<ObjectIdentifier> = []
    /// Includes queued speech before didStart, and audio still draining after stop.
    public var isSpeaking: Bool { !pendingUtterances.isEmpty || synth.isSpeaking }
    /// Called synchronously before every utterance so input can be fenced first.
    public var willSpeak: (@MainActor () -> Void)?

    public override init() {
        synth = AVSpeechSynthesizer()
        super.init()
        synth.delegate = self
    }

    init(synthesizer: AVSpeechSynthesizer) {
        synth = synthesizer
        super.init()
        synth.delegate = self
    }

    public func speak(_ text: String, language: String = "en-US", useIndependentAudioSession: Bool = false) {
        willSpeak?()
        // A finalized voice command may leave the app's inactive session in
        // record-only mode. Readiness speech can let the synthesizer manage its
        // own output session without changing microphone capture configuration.
        synth.usesApplicationAudioSession = !useIndependentAudioSession
        let utterance = AVSpeechUtterance(string: text)
        utterance.voice = AVSpeechSynthesisVoice(language: language)
        utterance.rate = AVSpeechUtteranceDefaultSpeechRate
        pendingUtterances.insert(ObjectIdentifier(utterance))
        synth.speak(utterance)
    }

    public func stopSpeaking() {
        synth.stopSpeaking(at: .immediate)
        pendingUtterances.removeAll()
    }

    public nonisolated func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didFinish utterance: AVSpeechUtterance) {
        let id = ObjectIdentifier(utterance)
        Task { @MainActor in pendingUtterances.remove(id) }
    }

    public nonisolated func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didCancel utterance: AVSpeechUtterance) {
        let id = ObjectIdentifier(utterance)
        Task { @MainActor in pendingUtterances.remove(id) }
    }
}
