public struct LastCommandState: Equatable, Sendable {
    public enum DisplayStatus: String, Equatable, Sendable {
        case listening = "Listening"
        case recognized = "Recognized"
        case working = "Working"
        case succeeded = "Succeeded"
        case failed = "Failed"
        case cancelled = "Cancelled"
    }

    public struct Record: Equatable, Sendable {
        public let id: MissionCommandID?
        public let captureID: SpeechCaptureID?
        public let command: String
        public let status: DisplayStatus
        public let message: String
    }

    public private(set) var record: Record?
    public private(set) var activeCaptureID: SpeechCaptureID?
    public private(set) var latestCommandID: MissionCommandID?

    public init(record: Record? = nil) {
        self.record = record
        latestCommandID = record?.id
    }

    public mutating func reduce(_ event: SpeechCaptureEvent) {
        switch event {
        case .started(let id):
            activeCaptureID = id
            record = Record(
                id: nil,
                captureID: id,
                command: "Listening…",
                status: .listening,
                message: "Listening…"
            )
        case .partial(let id, let transcript):
            guard activeCaptureID == id else { return }
            let transcript = transcript.trimmingCharacters(in: .whitespacesAndNewlines)
            guard !transcript.isEmpty else { return }
            record = Record(
                id: nil,
                captureID: id,
                command: transcript,
                status: .listening,
                message: "Listening…"
            )
        case .failed(let id, let message):
            guard activeCaptureID == id else { return }
            let command = record?.command ?? "Listening…"
            activeCaptureID = nil
            record = Record(
                id: nil,
                captureID: id,
                command: command,
                status: .failed,
                message: message
            )
        }
    }

    public mutating func finalizeCapture(_ id: SpeechCaptureID) {
        guard activeCaptureID == id else { return }
        activeCaptureID = nil
    }

    public mutating func reduce(_ status: MissionCommandStatus) {
        switch status {
        case .recognized(let id, let command):
            guard activeCaptureID == nil else { return }
            guard latestCommandID.map({ id > $0 }) ?? true else { return }
            latestCommandID = id
            record = Record(
                id: id,
                captureID: nil,
                command: command,
                status: .recognized,
                message: "Recognized"
            )
        case .working(let id, let command):
            update(id: id, command: command, status: .working, message: "Working")
        case .succeeded(let id, let command, let message):
            update(id: id, command: command, status: .succeeded, message: message)
        case .failed(let id, let command, let message):
            update(id: id, command: command, status: .failed, message: message)
        case .cancelled(let id, let command):
            update(id: id, command: command, status: .cancelled, message: "Cancelled")
        }
    }

    private mutating func update(
        id: MissionCommandID,
        command: String,
        status: DisplayStatus,
        message: String
    ) {
        guard record?.id == id else { return }
        record = Record(
            id: id,
            captureID: nil,
            command: command,
            status: status,
            message: message
        )
    }
}
