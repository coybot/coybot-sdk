import Foundation
import AVFoundation

/// On-device text-to-speech. Chosen over cloud TTS so the rover can always speak —
/// including in WiFi dead spots — at the cost of a more robotic voice than a cloud
/// premium voice would give.
@MainActor
public final class SpeechOut: NSObject, AVSpeechSynthesizerDelegate {
    private let synth = AVSpeechSynthesizer()
    public private(set) var isSpeaking = false

    public override init() {
        super.init()
        synth.delegate = self
    }

    public func speak(_ text: String, language: String = "en-US", useIndependentAudioSession: Bool = false) {
        // A finalized voice command may leave the app's inactive session in
        // record-only mode. Readiness speech can let the synthesizer manage its
        // own output session without changing microphone capture configuration.
        synth.usesApplicationAudioSession = !useIndependentAudioSession
        let utterance = AVSpeechUtterance(string: text)
        utterance.voice = AVSpeechSynthesisVoice(language: language)
        utterance.rate = AVSpeechUtteranceDefaultSpeechRate
        synth.speak(utterance)
    }

    public func stopSpeaking() {
        synth.stopSpeaking(at: .immediate)
    }

    public nonisolated func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didStart utterance: AVSpeechUtterance) {
        Task { @MainActor in isSpeaking = true }
    }

    public nonisolated func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didFinish utterance: AVSpeechUtterance) {
        Task { @MainActor in isSpeaking = false }
    }

    public nonisolated func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer, didCancel utterance: AVSpeechUtterance) {
        Task { @MainActor in isSpeaking = false }
    }
}
