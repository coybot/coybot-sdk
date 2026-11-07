import XCTest
import AVFoundation
@testable import PhroverKit

@MainActor
final class SpeechOutTests: XCTestCase {
    func testQueuedSpeechBlocksCaptureBeforeDelegateStartAndAcrossUtterances() async {
        let synth = RecordingSpeechSynthesizer()
        let out = SpeechOut(synthesizer: synth)
        var fenced = false
        out.willSpeak = { fenced = true }
        synth.onSpeak = { XCTAssertTrue(fenced, "Input must be fenced before output is submitted") }
        out.speak("Communication lost. Attempting to stop.")
        XCTAssertTrue(out.isSpeaking, "Do not wait for an asynchronous didStart callback")
        out.speak("Motor stop could not be confirmed. Motion is blocked.")
        out.speechSynthesizer(synth, didFinish: synth.utterances[0])
        for _ in 0..<10 { await Task.yield() }
        XCTAssertTrue(out.isSpeaking, "A second queued utterance still blocks capture")
        out.speechSynthesizer(synth, didFinish: synth.utterances[1])
        for _ in 0..<10 { await Task.yield() }
        XCTAssertFalse(out.isSpeaking)
    }

    func testLateCancellationOfOldUtteranceCannotUnlockNewAnnouncement() async {
        let synth = RecordingSpeechSynthesizer()
        let out = SpeechOut(synthesizer: synth)
        out.speak("Attempting to stop.")
        let old = synth.utterances[0]
        out.stopSpeaking()
        out.speak("Motor stop could not be confirmed. Motion is blocked.")
        out.speechSynthesizer(synth, didCancel: old)
        for _ in 0..<10 { await Task.yield() }
        XCTAssertTrue(out.isSpeaking)
        out.speechSynthesizer(synth, didFinish: synth.utterances[1])
        for _ in 0..<10 { await Task.yield() }
        XCTAssertFalse(out.isSpeaking)
    }
}

private final class RecordingSpeechSynthesizer: AVSpeechSynthesizer {
    var utterances: [AVSpeechUtterance] = []
    var onSpeak: (() -> Void)?
    override func speak(_ utterance: AVSpeechUtterance) {
        onSpeak?()
        utterances.append(utterance)
    }
    override func stopSpeaking(at boundary: AVSpeechBoundary) -> Bool { true }
}
