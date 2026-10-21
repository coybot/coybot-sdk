import Foundation

public enum PerceptionDebugSummary {
    public static func rawPredictions(_ detections: [Detector.Detection],
                                      personVerification: [PersonBodyVerifier.Decision]? = nil, limit: Int = 3) -> String {
        let ranked = detections.enumerated().sorted {
            ($0.element.confidence.isFinite ? $0.element.confidence : -.infinity) >
                ($1.element.confidence.isFinite ? $1.element.confidence : -.infinity)
        }.prefix(limit)
        guard !ranked.isEmpty else { return "none" }
        return ranked.map { index, detection in
            let accepted = verification(forDetectionAt: index, in: detections, decisions: personVerification)?.accepted ?? false
            return predictionLabel(detection, bodyVerified: accepted)
        }.joined(separator: ", ")
    }

    public static func verification(forDetectionAt index: Int, in detections: [Detector.Detection],
                                    decisions: [PersonBodyVerifier.Decision]?) -> PersonBodyVerifier.Decision? {
        guard detections.indices.contains(index), detections[index].label.lowercased() == "person" else { return nil }
        let personID = detections.prefix(index).filter { $0.label.lowercased() == "person" }.count
        return decisions?.first { $0.rawPersonID == personID }
    }

    public static func predictionLabel(_ detection: Detector.Detection, bodyVerified: Bool = false) -> String {
        let score = detection.confidence.isFinite && detection.confidence >= 0 && detection.confidence <= 1
            ? String(format: "%.4f", detection.confidence) : "unavailable"
        let check = detection.label.lowercased() == "person" ? (bodyVerified ? " (body checked)" : " (unverified)") : ""
        return "\(detection.label) score \(score)\(check)"
    }
    public static func visibleObjects(_ objects: [PerceivedObject], limit: Int = 3) -> String {
        let topObjects = objects
            .sorted { $0.confidence > $1.confidence }
            .prefix(limit)

        guard !topObjects.isEmpty else { return "none" }

        return topObjects
            .map { object in
                "\(object.label) \(Int((object.confidence * 100).rounded()))%"
            }
            .joined(separator: ", ")
    }
}


/// Navigation-focused presentation state for the operator debug panel.
public struct NavigationDebugSummary: Equatable, Sendable {
    private var openingCount: Int?
    private var candidateCount: Int?
    private var target: String?
    private var isReachable: Bool?

    public private(set) var transitionText = "idle"

    public init() {}

    public var openingsText: String {
        openingCount.map(String.init) ?? "—"
    }

    public var doorwayCandidatesText: String {
        guard let candidateCount else { return "—" }
        guard let isReachable else { return String(candidateCount) }
        return "\(candidateCount) (\(isReachable ? "reachable" : "unreachable"))"
    }

    public var targetText: String {
        target ?? "none"
    }

    public mutating func apply(_ state: RoomTransitionDebugState) {
        switch state {
        case .idle:
            openingCount = nil
            candidateCount = nil
            target = nil
            isReachable = nil
            transitionText = "idle"

        case let .scanning(step, total, openingCount, candidateCount):
            self.openingCount = openingCount
            self.candidateCount = candidateCount
            target = nil
            isReachable = nil
            transitionText = "scanning \(step)/\(total)"

        case let .candidateFound(id, reachable):
            target = id.rawValue
            isReachable = reachable
            transitionText = "candidate found"

        case let .unreachable(id):
            target = id.rawValue
            isReachable = false
            transitionText = "unreachable"

        case let .approaching(id):
            target = id.rawValue
            isReachable = true
            transitionText = "approaching"

        case let .confirmingCrossing(id):
            target = id.rawValue
            isReachable = true
            transitionText = "confirming crossing"

        case let .completed(doorwayID, roomID):
            target = doorwayID.rawValue
            isReachable = true
            transitionText = "completed: \(roomID.rawValue)"

        case .exhausted:
            target = nil
            isReachable = nil
            transitionText = "exhausted"

        case let .failed(reason):
            transitionText = "failed: \(reason.replacingOccurrences(of: "_", with: " "))"
        }
    }
}
