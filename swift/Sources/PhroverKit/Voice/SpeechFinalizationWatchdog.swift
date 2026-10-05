import Foundation

enum SpeechTranscriptSelection {
    static func finalTranscript(
        latest: String,
        lastNonEmptyPartial: String
    ) -> String? {
        let latest = latest.trimmingCharacters(in: .whitespacesAndNewlines)
        if !latest.isEmpty { return latest }

        let partial = lastNonEmptyPartial.trimmingCharacters(in: .whitespacesAndNewlines)
        return partial.isEmpty ? nil : partial
    }
}

@MainActor
final class SpeechFinalizationWatchdog {
    private let timeout: Duration
    private var task: Task<Void, Never>?

    init(timeout: Duration) {
        self.timeout = timeout
    }

    func schedule(
        partialTranscript: @escaping @MainActor () -> String,
        completion: @escaping @MainActor (String?) -> Void
    ) {
        cancel()
        task = Task { [weak self] in
            guard let self else { return }
            try? await Task.sleep(for: timeout)
            guard !Task.isCancelled else { return }

            task = nil
            let transcript = partialTranscript()
                .trimmingCharacters(in: .whitespacesAndNewlines)
            completion(transcript.isEmpty ? nil : transcript)
        }
    }

    func cancel() {
        task?.cancel()
        task = nil
    }
}
