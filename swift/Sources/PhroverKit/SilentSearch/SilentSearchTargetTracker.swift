import Foundation
import RoverNav

public struct TargetDetectionObservation: Equatable, Sendable {
    public let label: String
    public let confidence: Double
    public let normalizedCenter: Vec2
    public let localGroundedPoint: Vec2?

    public init(
        label: String,
        confidence: Double,
        normalizedCenter: Vec2,
        localGroundedPoint: Vec2?
    ) {
        self.label = label
        self.confidence = confidence
        self.normalizedCenter = normalizedCenter
        self.localGroundedPoint = localGroundedPoint
    }
}

public struct TargetFrameObservation: Equatable, Sendable {
    public let frameID: UInt64
    public let monotonicTimestamp: TimeInterval
    public let detections: [TargetDetectionObservation]

    public init(
        frameID: UInt64,
        monotonicTimestamp: TimeInterval,
        detections: [TargetDetectionObservation]
    ) {
        self.frameID = frameID
        self.monotonicTimestamp = monotonicTimestamp
        self.detections = detections
    }
}

public struct TargetConfirmation: Equatable, Sendable {
    public let label: String
    public let coordinate: MissionPoint
    public let sampleCount: Int
    public let meanConfidence: Double

    public init(label: String, coordinate: MissionPoint, sampleCount: Int, meanConfidence: Double) {
        self.label = label
        self.coordinate = coordinate
        self.sampleCount = sampleCount
        self.meanConfidence = meanConfidence
    }
}

public enum TargetTrackingResult: Equatable, Sendable {
    case collecting(sampleCount: Int)
    case confirmed(TargetConfirmation)
}

public enum TargetRejectionReason: Equatable, Sendable {
    case duplicateFrame
    case invalidTimestamp
    case noMatchingDetection
    case confidenceBelowThreshold
    case invalidDetection
    case invalidGrounding
    case inconsistentCluster
}

public enum TargetTrackerEvent: Equatable, Sendable {
    case rejected(frameID: UInt64, reason: TargetRejectionReason)
    case evidenceAccepted(frameID: UInt64, sampleCount: Int)
    case confirmed(TargetConfirmation)
}

public protocol TargetTrackerEventSink: AnyObject {
    func record(_ event: TargetTrackerEvent)
}

public final class SilentSearchTargetTracker {
    public private(set) var confirmation: TargetConfirmation?

    private struct Evidence {
        let frameID: UInt64
        let timestamp: TimeInterval
        let coordinate: MissionPoint
        let confidence: Double
    }

    private let canonicalLabel: String
    private let frame: SharedMissionFrame
    private let eventSink: TargetTrackerEventSink?
    private let events: (any SilentSearchEventSink)?
    private var processedFrameIDs = Set<UInt64>()
    private var evidence: [Evidence] = []
    private var latestTimestamp = -Double.infinity

    public init(
        canonicalLabel: String,
        frame: SharedMissionFrame,
        eventSink: TargetTrackerEventSink? = nil,
        events: (any SilentSearchEventSink)? = nil
    ) {
        self.canonicalLabel = canonicalLabel
        self.frame = frame
        self.eventSink = eventSink
        self.events = events
    }

    public func process(_ observation: TargetFrameObservation) -> TargetTrackingResult {
        if let confirmation { return .confirmed(confirmation) }

        guard processedFrameIDs.insert(observation.frameID).inserted else {
            reject(observation.frameID, .duplicateFrame)
            return .collecting(sampleCount: evidence.count)
        }
        guard observation.monotonicTimestamp.isFinite else {
            reject(observation.frameID, .invalidTimestamp)
            return .collecting(sampleCount: evidence.count)
        }

        latestTimestamp = max(latestTimestamp, observation.monotonicTimestamp)
        evidence.removeAll { latestTimestamp - $0.timestamp > 2.0 + 1e-12 }

        var best: TargetDetectionObservation?
        for detection in observation.detections where detection.label == canonicalLabel {
            if best == nil || detection.confidence > best!.confidence { best = detection }
        }
        guard let best else {
            reject(observation.frameID, .noMatchingDetection)
            return .collecting(sampleCount: evidence.count)
        }
        guard best.confidence.isFinite, best.confidence >= 0.90 else {
            reject(observation.frameID, .confidenceBelowThreshold)
            return .collecting(sampleCount: evidence.count)
        }
        guard best.normalizedCenter.x.isFinite,
              best.normalizedCenter.y.isFinite,
              (0...1).contains(best.normalizedCenter.x),
              (0...1).contains(best.normalizedCenter.y) else {
            reject(observation.frameID, .invalidDetection)
            return .collecting(sampleCount: evidence.count)
        }
        guard let localPoint = best.localGroundedPoint,
              localPoint.x.isFinite,
              localPoint.y.isFinite,
              let missionPoint = frame.missionPoint(from: localPoint) else {
            reject(observation.frameID, .invalidGrounding)
            return .collecting(sampleCount: evidence.count)
        }

        evidence.append(Evidence(
            frameID: observation.frameID,
            timestamp: observation.monotonicTimestamp,
            coordinate: missionPoint,
            confidence: best.confidence
        ))
        evidence.removeAll { latestTimestamp - $0.timestamp > 2.0 + 1e-12 }
        eventSink?.record(.evidenceAccepted(frameID: observation.frameID, sampleCount: evidence.count))
        events?.record(event: "silent_search_target_evidence", fields: [
            "confidence_basis_points": Self.basisPoints(best.confidence),
            "grounded": "true",
            "outcome": "accepted",
            "sample_count": "\(evidence.count)",
        ])

        guard evidence.count >= 3 else { return .collecting(sampleCount: evidence.count) }
        let samples = Array(evidence.suffix(3))
        let median = MissionPoint(
            x: Self.median(samples.map { $0.coordinate.x }),
            y: Self.median(samples.map { $0.coordinate.y })
        )!
        guard samples.allSatisfy({ sample in
            hypot(sample.coordinate.x - median.x, sample.coordinate.y - median.y) <= 0.35 + 1e-12
        }) else {
            reject(observation.frameID, .inconsistentCluster)
            return .collecting(sampleCount: evidence.count)
        }

        let result = TargetConfirmation(
            label: canonicalLabel,
            coordinate: median,
            sampleCount: samples.count,
            meanConfidence: samples.reduce(0) { $0 + $1.confidence } / Double(samples.count)
        )
        confirmation = result
        eventSink?.record(.confirmed(result))
        events?.record(event: "silent_search_target_confirmed", fields: [
            "confidence_basis_points": Self.basisPoints(result.meanConfidence),
            "label": result.label,
            "sample_count": "\(result.sampleCount)",
            "x_mm": "\(Int((result.coordinate.x * 1_000).rounded()))",
            "y_mm": "\(Int((result.coordinate.y * 1_000).rounded()))",
        ])
        return .confirmed(result)
    }

    private func reject(_ frameID: UInt64, _ reason: TargetRejectionReason) {
        eventSink?.record(.rejected(frameID: frameID, reason: reason))
        events?.record(event: "silent_search_target_evidence", fields: [
            "outcome": "rejected",
            "reason": String(describing: reason),
        ])
    }

    private static func median(_ values: [Double]) -> Double {
        let sorted = values.sorted()
        return sorted[sorted.count / 2]
    }

    private static func basisPoints(_ confidence: Double) -> String {
        "\(Int((confidence * 10_000).rounded()))"
    }
}
