import Foundation

struct FollowDiagnosticError: Sendable {
    let code: String
    let message: String
    var payload: [String: FollowDiagnosticValue] {
        ["code": .string(String(code.prefix(64))),
         "message": .string(String(message.replacingOccurrences(of: "\n", with: " ")
            .replacingOccurrences(of: "\r", with: " ").prefix(256)))]
    }
}

struct FollowDiagnosticEvent: Sendable {
    let event: String
    let context: FollowDiagnosticContext
    let payload: [String: FollowDiagnosticValue]

    init(event: String, context: FollowDiagnosticContext = .init(),
         payload: [String: FollowDiagnosticValue] = [:]) {
        self.event = event
        self.context = context
        self.payload = payload
    }
}

indirect enum FollowDiagnosticValue: Sendable, Equatable {
    case null
    case bool(Bool)
    case number(Double)
    case string(String)
    case array([FollowDiagnosticValue])
    case object([String: FollowDiagnosticValue])

    fileprivate var jsonValue: Any {
        switch self {
        case .null: NSNull()
        case .bool(let value): value
        case .number(let value): value.isFinite ? value as Any : NSNull()
        case .string(let value): value
        case .array(let values): values.map(\.jsonValue)
        case .object(let values): Self.jsonObject(values)
        }
    }

    /// Defense in depth; callers supply only bounded primitive diagnostic facts.
    fileprivate static func jsonObject(_ values: [String: Self]) -> [String: Any] {
        let excluded: Set<String> = ["image", "images", "audio", "transcript", "transcripts",
            "request_body", "response_body", "projection_count", "depth_array", "depth_samples",
            "url", "path", "request_payload", "biometric_identity"]
        var result: [String: Any] = [:]
        for (key, value) in values where !excluded.contains(key) {
            result[key] = value.jsonValue
        }
        // Measurement availability is authoritative regardless of dictionary order.
        for (key, value) in values where !excluded.contains(key) {
            if case .number(let number) = value, !number.isFinite {
                result[key + "_availability"] = "nonfinite"
            }
        }
        return result
    }
}

@MainActor
final class FollowDiagnosticEmitter {
    private let sink: @MainActor (String, [String: String]) -> Void
    private let streamID: String
    private let monotonic: @MainActor () -> Double
    private let utc: @MainActor () -> Date
    private var sequence: UInt64 = 0
    private let deferredRuntime: Bool

    init(streamID: String, monotonic: @escaping @MainActor () -> Double,
         utc: @escaping @MainActor () -> Date,
          sink: @escaping @MainActor (String, [String: String]) -> Void, deferredRuntime: Bool = false) {
        self.deferredRuntime = deferredRuntime
        self.sink = sink
        self.streamID = streamID
        self.monotonic = monotonic
        self.utc = utc
    }

    func emit(_ event: FollowDiagnosticEvent) {
        sequence += 1
        let time = monotonic()
        let date = utc()
        if deferredRuntime {
            let sequence = sequence
            let streamID = streamID
            RuntimeFileLog.appendDeferred(event.event, now: date) {
                Self.serialize(event, sequence: sequence, streamID: streamID, time: time, date: date)
            }
        } else if let fields = Self.serialize(event, sequence: sequence, streamID: streamID, time: time, date: date) {
            sink(event.event, fields)
        }
    }

    nonisolated private static func serialize(_ event: FollowDiagnosticEvent, sequence: UInt64,
                                              streamID: String, time: Double, date: Date) -> [String: String]? {
        let formatter = ISO8601DateFormatter()
        formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
        var payload = FollowDiagnosticValue.jsonObject(event.payload)
        let envelope: [String: Any] = [
            "event": event.event, "schema_version": 1,
            "stream_id": streamID, "event_sequence": sequence,
            "monotonic_s": time, "utc_time": formatter.string(from: date),
            "stale": event.context.stale,
            "session_generation": event.context.sessionGeneration as Any? ?? NSNull(),
            "operation_id": event.context.operationID as Any? ?? NSNull(),
            "purpose": event.context.purpose as Any? ?? NSNull(),
            "phase": event.context.phase as Any? ?? NSNull(),
            "pulse_index": event.context.pulseIndex as Any? ?? NSNull(),
            "outcome": event.context.outcome as Any? ?? NSNull(),
            "reason": event.context.reason as Any? ?? NSNull()
        ]
        payload.merge(envelope) { _, authoritative in authoritative }
        guard let data = try? JSONSerialization.data(withJSONObject: payload, options: [.sortedKeys]),
              let json = String(data: data, encoding: .utf8) else { return nil }
        return ["payload": json]
    }

    func hostTime() -> Double { monotonic() }
    func utcTime() -> Date { utc() }
}
