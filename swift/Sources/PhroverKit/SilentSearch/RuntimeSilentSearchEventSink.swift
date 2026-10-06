import Foundation

public final class RuntimeSilentSearchEventSink: SilentSearchEventSink {
    private let lock = NSLock()
    private static let allowedFields: Set<String> = [
        "backend", "confidence_basis_points", "corner", "direction", "error_code",
        "error_domain", "frame_sequence", "frontier_id", "from", "generation", "grounded",
        "kind", "label", "marker", "mission", "monotonic_timestamp", "orientation", "outcome",
        "path_length_mm", "reason", "release_epoch_ms", "result", "role", "sample_count",
        "sequence", "stage", "to", "x_mm", "y_mm",
    ]

    public init() {}

    public func record(event: String, fields: [String: String]) {
        lock.lock()
        defer { lock.unlock() }
        RuntimeFileLog.append(event, fields: Self.sanitized(fields))
    }

    static func sanitized(_ fields: [String: String]) -> [String: String] {
        fields.filter { key, _ in allowedFields.contains(key.lowercased()) }
    }
}
