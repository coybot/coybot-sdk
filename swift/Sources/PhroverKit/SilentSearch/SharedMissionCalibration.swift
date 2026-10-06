import Foundation
import RoverNav

public struct OrientedMarkerCorners: Equatable, Sendable {
    public let topLeft: Vec2
    public let topRight: Vec2
    public let bottomLeft: Vec2
    public let bottomRight: Vec2

    public init(topLeft: Vec2, topRight: Vec2, bottomLeft: Vec2, bottomRight: Vec2) {
        self.topLeft = topLeft
        self.topRight = topRight
        self.bottomLeft = bottomLeft
        self.bottomRight = bottomRight
    }
}

public struct SharedMissionCalibrationObservation: Equatable, Sendable {
    public let markerID: String
    public let sessionGeneration: UInt64
    public let frameID: UInt64
    public let monotonicTimestamp: TimeInterval
    public let corners: OrientedMarkerCorners

    public init(
        markerID: String,
        sessionGeneration: UInt64,
        frameID: UInt64,
        monotonicTimestamp: TimeInterval,
        corners: OrientedMarkerCorners
    ) {
        self.markerID = markerID
        self.sessionGeneration = sessionGeneration
        self.frameID = frameID
        self.monotonicTimestamp = monotonicTimestamp
        self.corners = corners
    }
}

public struct SharedMissionCalibrationConfiguration: Equatable, Sendable {
    public let markerID: String
    public let markerWidth: Double
    public let requiredFrameCount: Int
    public let maximumWindow: TimeInterval
    public let maximumOriginDeviation: Double
    public let maximumHeadingDeviation: Double
    public let maximumWidthFractionDeviation: Double

    public init?(
        markerID: String,
        markerWidth: Double = 0.20,
        requiredFrameCount: Int = 3,
        maximumWindow: TimeInterval = 2,
        maximumOriginDeviation: Double = 0.10,
        maximumHeadingDeviation: Double = 5 * .pi / 180,
        maximumWidthFractionDeviation: Double = 0.15
    ) {
        guard SharedMissionCalibrator.isValidMarkerID(markerID),
              markerWidth.isFinite, markerWidth > 0,
              requiredFrameCount > 0,
              maximumWindow.isFinite, maximumWindow >= 0,
              maximumOriginDeviation.isFinite, maximumOriginDeviation >= 0,
              maximumHeadingDeviation.isFinite, maximumHeadingDeviation >= 0,
              maximumWidthFractionDeviation.isFinite, maximumWidthFractionDeviation >= 0 else {
            return nil
        }
        self.markerID = markerID
        self.markerWidth = markerWidth
        self.requiredFrameCount = requiredFrameCount
        self.maximumWindow = maximumWindow
        self.maximumOriginDeviation = maximumOriginDeviation
        self.maximumHeadingDeviation = maximumHeadingDeviation
        self.maximumWidthFractionDeviation = maximumWidthFractionDeviation
    }
}

public enum SharedMissionCalibrationDiagnostic: Equatable, Sendable {
    case invalidMarkerID
    case unexpectedMarkerID
    case generationMismatch
    case duplicateFrame
    case nonFiniteObservation
    case degenerateCorners
    case timeWindowExceeded
    case originDeviationExceeded
    case headingDeviationExceeded
    case widthDeviationExceeded
}

public struct SharedMissionCalibrator: Sendable {
    public enum Result: Equatable, Sendable {
        case collecting(frameCount: Int)
        case accepted(SharedMissionFrame)
        case rejected(SharedMissionCalibrationDiagnostic)
    }

    private struct Evidence: Sendable {
        let frameID: UInt64
        let timestamp: TimeInterval
        let origin: Vec2
        let heading: Double
        let width: Double
    }

    public let configuration: SharedMissionCalibrationConfiguration
    public let sessionGeneration: UInt64
    private var evidence: [Evidence] = []
    private var acceptedFrame: SharedMissionFrame?
    private let events: (any SilentSearchEventSink)?

    public init(configuration: SharedMissionCalibrationConfiguration, sessionGeneration: UInt64,
                events: (any SilentSearchEventSink)? = nil) {
        self.configuration = configuration
        self.sessionGeneration = sessionGeneration
        self.events = events
    }

    public mutating func observe(_ observation: SharedMissionCalibrationObservation) -> Result {
        if let acceptedFrame { return .accepted(acceptedFrame) }
        guard Self.isValidMarkerID(observation.markerID) else { return .rejected(.invalidMarkerID) }
        guard observation.markerID == configuration.markerID else { return .rejected(.unexpectedMarkerID) }
        guard observation.sessionGeneration == sessionGeneration else { return .rejected(.generationMismatch) }
        guard !evidence.contains(where: { $0.frameID == observation.frameID }) else {
            return .rejected(.duplicateFrame)
        }
        guard let derived = Self.derive(from: observation) else {
            return .rejected(Self.hasFiniteValues(observation) ? .degenerateCorners : .nonFiniteObservation)
        }

        evidence.append(derived)
        evidence.sort { $0.timestamp < $1.timestamp }
        if evidence.count < configuration.requiredFrameCount {
            events?.record(event: "silent_search_calibration_progress", fields: [
                "marker": configuration.markerID,
                "sample_count": "\(evidence.count)",
            ])
            return .collecting(frameCount: evidence.count)
        }

        let samples = Array(evidence.suffix(configuration.requiredFrameCount))
        guard samples.last!.timestamp - samples.first!.timestamp <= configuration.maximumWindow + 1e-12 else {
            return .rejected(.timeWindowExceeded)
        }
        let origin = Vec2(
            Self.median(samples.map { $0.origin.x }),
            Self.median(samples.map { $0.origin.y })
        )
        guard samples.allSatisfy({
            $0.origin.distance(to: origin) <= configuration.maximumOriginDeviation + 1e-12
        }) else {
            return .rejected(.originDeviationExceeded)
        }
        let heading = atan2(
            samples.reduce(0) { $0 + sin($1.heading) },
            samples.reduce(0) { $0 + cos($1.heading) }
        )
        guard samples.allSatisfy({
            abs(normalizeAngle($0.heading - heading)) <= configuration.maximumHeadingDeviation + 1e-12
        }) else {
            return .rejected(.headingDeviationExceeded)
        }
        guard samples.allSatisfy({
            abs($0.width - configuration.markerWidth) / configuration.markerWidth
                <= configuration.maximumWidthFractionDeviation + 1e-12
        }) else {
            return .rejected(.widthDeviationExceeded)
        }
        guard let frame = SharedMissionFrame(
            localOrigin: origin,
            localNorthHeading: heading,
            sessionGeneration: sessionGeneration
        ) else {
            return .rejected(.nonFiniteObservation)
        }
        acceptedFrame = frame
        events?.record(event: "silent_search_calibration_accepted", fields: [
            "generation": "\(sessionGeneration)",
            "marker": configuration.markerID,
            "sample_count": "\(configuration.requiredFrameCount)",
        ])
        return .accepted(frame)
    }

    public static func isValidMarkerID(_ markerID: String) -> Bool {
        guard (1...24).contains(markerID.utf8.count) else { return false }
        return markerID.utf8.allSatisfy {
            (65...90).contains($0) || (48...57).contains($0) || $0 == 95 || $0 == 45
        }
    }

    public static func payload(forMarkerID markerID: String) -> String? {
        guard isValidMarkerID(markerID) else { return nil }
        return "PHROVER-CAL|1|\(markerID)"
    }

    public static func markerID(fromPayload payload: String) -> String? {
        let parts = payload.split(separator: "|", omittingEmptySubsequences: false)
        guard parts.count == 3, parts[0] == "PHROVER-CAL", parts[1] == "1" else { return nil }
        let markerID = String(parts[2])
        return isValidMarkerID(markerID) ? markerID : nil
    }

    private static func derive(from observation: SharedMissionCalibrationObservation) -> Evidence? {
        guard observation.monotonicTimestamp.isFinite, hasFiniteValues(observation) else { return nil }
        let corners = observation.corners
        let topMidpoint = (corners.topLeft + corners.topRight) * 0.5
        let bottomMidpoint = (corners.bottomLeft + corners.bottomRight) * 0.5
        let north = topMidpoint - bottomMidpoint
        let topWidth = corners.topLeft.distance(to: corners.topRight)
        let bottomWidth = corners.bottomLeft.distance(to: corners.bottomRight)
        guard north.length > 1e-9, topWidth > 1e-9, bottomWidth > 1e-9 else { return nil }
        let origin = (corners.topLeft + corners.topRight + corners.bottomLeft + corners.bottomRight) * 0.25
        return Evidence(
            frameID: observation.frameID,
            timestamp: observation.monotonicTimestamp,
            origin: origin,
            heading: atan2(north.y, north.x),
            width: (topWidth + bottomWidth) * 0.5
        )
    }

    private static func hasFiniteValues(_ observation: SharedMissionCalibrationObservation) -> Bool {
        let corners = observation.corners
        return observation.monotonicTimestamp.isFinite && [
            corners.topLeft, corners.topRight, corners.bottomLeft, corners.bottomRight,
        ].allSatisfy { $0.x.isFinite && $0.y.isFinite }
    }

    private static func median(_ values: [Double]) -> Double {
        let sorted = values.sorted()
        let middle = sorted.count / 2
        if sorted.count.isMultiple(of: 2) {
            return (sorted[middle - 1] + sorted[middle]) * 0.5
        }
        return sorted[middle]
    }
}
