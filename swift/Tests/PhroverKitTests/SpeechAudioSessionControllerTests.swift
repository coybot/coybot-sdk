import AVFoundation
import Speech
import XCTest
@testable import PhroverKit

@MainActor
final class SpeechAudioSessionControllerTests: XCTestCase {
    func testActivateConfiguresRecordingSessionBeforeMakingItActive() throws {
        let session = RecordingAudioSession()
        let controller = SpeechAudioSessionController(session: session)

        try controller.activate()

        XCTAssertEqual(session.calls, [
            .setCategory(.record, .measurement, [.duckOthers]),
            .setActive(true, [.notifyOthersOnDeactivation]),
        ])
        XCTAssertTrue(controller.isActive)
    }

    func testDeactivateOnlyDeactivatesAnActiveSessionOnce() throws {
        let session = RecordingAudioSession()
        let controller = SpeechAudioSessionController(session: session)
        try controller.activate()

        try controller.deactivate()
        try controller.deactivate()

        XCTAssertEqual(session.calls, [
            .setCategory(.record, .measurement, [.duckOthers]),
            .setActive(true, [.notifyOthersOnDeactivation]),
            .setActive(false, [.notifyOthersOnDeactivation]),
        ])
        XCTAssertFalse(controller.isActive)
    }

    func testSpeechCaptureRequiresSpeechAndMicrophoneAuthorization() {
        XCTAssertTrue(SpeechAuthorizationPolicy.isAuthorized(
            speechStatus: .authorized,
            microphoneGranted: true
        ))
        XCTAssertFalse(SpeechAuthorizationPolicy.isAuthorized(
            speechStatus: .denied,
            microphoneGranted: true
        ))
        XCTAssertFalse(SpeechAuthorizationPolicy.isAuthorized(
            speechStatus: .authorized,
            microphoneGranted: false
        ))
    }

    func testInputMetricsReportReceivedFramesAndSignalLevel() throws {
        let metrics = SpeechAudioInputMetrics()
        let format = try XCTUnwrap(AVAudioFormat(
            standardFormatWithSampleRate: 44_100,
            channels: 1
        ))
        let buffer = try XCTUnwrap(AVAudioPCMBuffer(
            pcmFormat: format,
            frameCapacity: 4
        ))
        buffer.frameLength = 4
        let samples = try XCTUnwrap(buffer.floatChannelData?[0])
        samples[0] = 0.5
        samples[1] = -0.5
        samples[2] = 0.5
        samples[3] = -0.5

        metrics.observe(buffer)
        let snapshot = metrics.snapshot()

        XCTAssertEqual(snapshot.bufferCount, 1)
        XCTAssertEqual(snapshot.frameCount, 4)
        XCTAssertEqual(snapshot.maxRMS, 0.5, accuracy: 0.001)
    }
}

@MainActor
private final class RecordingAudioSession: SpeechAudioSessionConfiguring {
    enum Call: Equatable {
        case setCategory(
            AVAudioSession.Category,
            AVAudioSession.Mode,
            AVAudioSession.CategoryOptions
        )
        case setActive(Bool, AVAudioSession.SetActiveOptions)
    }

    private(set) var calls: [Call] = []

    func setCategory(
        _ category: AVAudioSession.Category,
        mode: AVAudioSession.Mode,
        options: AVAudioSession.CategoryOptions
    ) throws {
        calls.append(.setCategory(category, mode, options))
    }

    func setActive(
        _ active: Bool,
        options: AVAudioSession.SetActiveOptions
    ) throws {
        calls.append(.setActive(active, options))
    }
}
