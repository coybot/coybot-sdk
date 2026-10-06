import Foundation

public enum SilentSearchCalibrationCorner: String, Equatable, Sendable {
    case topLeft
    case topRight
    case bottomLeft
    case bottomRight
}

public enum SilentSearchCalibrationGroundingFailure: Error, Equatable, Sendable {
    case trackingNotNormal
    case generationMismatch
    case frameMismatch
    case timestampMismatch
    case invalidPayload
    case wrongMarkerID
    case missingDepthMap
    case cornerUnavailable(SilentSearchCalibrationCorner)
}

public struct SilentSearchCalibrationFrameContext: Equatable, Sendable {
    public let frameID: ARFrameID
    public let monotonicTimestamp: TimeInterval

    public init(frameID: ARFrameID, monotonicTimestamp: TimeInterval) {
        self.frameID = frameID
        self.monotonicTimestamp = monotonicTimestamp
    }
}

public struct SilentSearchCalibrationDetection: Equatable, Sendable {
    public let context: SilentSearchCalibrationFrameContext
    public let markerID: String
    public let corners: OrientedMarkerCorners

    public init(
        context: SilentSearchCalibrationFrameContext,
        markerID: String,
        corners: OrientedMarkerCorners
    ) {
        self.context = context
        self.markerID = markerID
        self.corners = corners
    }
}

public enum SilentSearchCalibrationFeedback: Equatable, Sendable {
    case trackingNotNormal(context: SilentSearchCalibrationFrameContext)
    case expectedMarkerDetected(
        context: SilentSearchCalibrationFrameContext,
        markerID: String,
        corners: OrientedMarkerCorners
    )
    case waitingForMarker(context: SilentSearchCalibrationFrameContext)
    case scanCompleted(context: SilentSearchCalibrationFrameContext)
    case qrLost(context: SilentSearchCalibrationFrameContext)
    case scannerFailed(context: SilentSearchCalibrationFrameContext)
    case scannerBackendFailed(
        context: SilentSearchCalibrationFrameContext,
        diagnostic: OpticalScannerBackendDiagnostic
    )
    case groundingFailed(
        context: SilentSearchCalibrationFrameContext,
        reason: SilentSearchCalibrationGroundingFailure
    )
    case allCornersGrounded(context: SilentSearchCalibrationFrameContext)
}

public enum SilentSearchCalibrationIssue: Equatable, Sendable {
    case trackingNotNormal
    case scannerFailure
    case groundingFailure(SilentSearchCalibrationGroundingFailure)
    case calibrationRejection(SharedMissionCalibrationDiagnostic)
}

public struct SilentSearchCalibrationVisualState: Equatable, Sendable {
    private static let detectionRetention: TimeInterval = 0.5
    private static let maximumDetectionCount = 32

    public var qrDecoded = false
    public var cornersGrounded = false
    public var sampleAccepted = false
    public private(set) var recentDetections: [SilentSearchCalibrationDetection] = []
    public var currentIssue: SilentSearchCalibrationIssue?

    public init() {}

    public mutating func recordDetection(_ detection: SilentSearchCalibrationDetection) {
        recentDetections.removeAll { existing in
            existing.context == detection.context
                || detection.context.monotonicTimestamp - existing.context.monotonicTimestamp
                    > Self.detectionRetention
        }
        recentDetections.append(detection)
        if recentDetections.count > Self.maximumDetectionCount {
            recentDetections.removeFirst(recentDetections.count - Self.maximumDetectionCount)
        }
    }

    public mutating func clearDetections() {
        recentDetections.removeAll(keepingCapacity: true)
    }

    public func detection(
        matching context: SilentSearchCalibrationFrameContext?
    ) -> SilentSearchCalibrationDetection? {
        guard let context else { return nil }
        return recentDetections.last { $0.context == context }
    }
}
