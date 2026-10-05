import AVFoundation
import Foundation
import Speech

@MainActor
protocol SpeechAudioSessionConfiguring: AnyObject {
    func setCategory(
        _ category: AVAudioSession.Category,
        mode: AVAudioSession.Mode,
        options: AVAudioSession.CategoryOptions
    ) throws

    func setActive(
        _ active: Bool,
        options: AVAudioSession.SetActiveOptions
    ) throws
}

extension AVAudioSession: SpeechAudioSessionConfiguring {}

@MainActor
final class SpeechAudioSessionController {
    private let session: any SpeechAudioSessionConfiguring
    private(set) var isActive = false

    init(session: any SpeechAudioSessionConfiguring = AVAudioSession.sharedInstance()) {
        self.session = session
    }

    func activate() throws {
        guard !isActive else { return }
        try session.setCategory(.record, mode: .measurement, options: [.duckOthers])
        try session.setActive(true, options: [.notifyOthersOnDeactivation])
        isActive = true
    }

    func deactivate() throws {
        guard isActive else { return }
        try session.setActive(false, options: [.notifyOthersOnDeactivation])
        isActive = false
    }
}

enum SpeechAuthorizationPolicy {
    static func isAuthorized(
        speechStatus: SFSpeechRecognizerAuthorizationStatus,
        microphoneGranted: Bool
    ) -> Bool {
        speechStatus == .authorized && microphoneGranted
    }
}

struct SpeechAudioInputSnapshot: Equatable, Sendable {
    let bufferCount: Int
    let frameCount: Int
    let maxRMS: Float
}

final class SpeechAudioInputMetrics: @unchecked Sendable {
    private let lock = NSLock()
    private var bufferCount = 0
    private var frameCount = 0
    private var maxRMS: Float = 0

    func observe(_ buffer: AVAudioPCMBuffer) {
        let frames = Int(buffer.frameLength)
        var observedRMS: Float = 0

        if frames > 0, let channels = buffer.floatChannelData {
            for channelIndex in 0..<Int(buffer.format.channelCount) {
                let samples = channels[channelIndex]
                var sumOfSquares: Float = 0
                for frameIndex in 0..<frames {
                    let sample = samples[frameIndex]
                    sumOfSquares += sample * sample
                }
                observedRMS = max(observedRMS, sqrt(sumOfSquares / Float(frames)))
            }
        }

        lock.lock()
        bufferCount += 1
        frameCount += frames
        maxRMS = max(maxRMS, observedRMS)
        lock.unlock()
    }

    func snapshot() -> SpeechAudioInputSnapshot {
        lock.lock()
        defer { lock.unlock() }
        return SpeechAudioInputSnapshot(
            bufferCount: bufferCount,
            frameCount: frameCount,
            maxRMS: maxRMS
        )
    }

    func reset() {
        lock.lock()
        bufferCount = 0
        frameCount = 0
        maxRMS = 0
        lock.unlock()
    }
}
