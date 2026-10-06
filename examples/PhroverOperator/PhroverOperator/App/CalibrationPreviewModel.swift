import CoreImage
import ImageIO
import Observation
import PhroverKit
import RoverNav
import UIKit

struct CalibrationPreviewTransform: Equatable {
    let imageRect: CGRect

    init(imageSize: CGSize, previewBounds: CGRect) {
        guard imageSize.width > 0, imageSize.height > 0,
              previewBounds.width > 0, previewBounds.height > 0 else {
            imageRect = CGRect(origin: previewBounds.origin, size: .zero)
            return
        }
        let scale = min(previewBounds.width / imageSize.width, previewBounds.height / imageSize.height)
        let size = CGSize(width: imageSize.width * scale, height: imageSize.height * scale)
        imageRect = CGRect(
            x: previewBounds.midX - size.width / 2,
            y: previewBounds.midY - size.height / 2,
            width: size.width,
            height: size.height
        )
    }

    func point(for normalizedPoint: Vec2) -> CGPoint {
        let x = min(max(normalizedPoint.x, 0), 1)
        let y = min(max(normalizedPoint.y, 0), 1)
        return CGPoint(
            x: imageRect.minX + CGFloat(x) * imageRect.width,
            y: imageRect.minY + CGFloat(1 - y) * imageRect.height
        )
    }
}

@MainActor
@Observable
final class CalibrationPreviewModel {
    typealias Renderer = @MainActor (ARFrameSnapshot) -> UIImage?

    private(set) var image: UIImage?
    private(set) var latestRenderedFrameContext: SilentSearchCalibrationFrameContext?
    @ObservationIgnored private let frames: @MainActor () -> AsyncStream<ARFrameSnapshot>
    @ObservationIgnored private let render: Renderer
    @ObservationIgnored private let minimumInterval: TimeInterval
    @ObservationIgnored private var task: Task<Void, Never>?
    @ObservationIgnored private var lastRenderedTimestamp: TimeInterval?

    init(
        frames: @escaping @MainActor () -> AsyncStream<ARFrameSnapshot>,
        minimumInterval: TimeInterval = 0.1,
        render: @escaping Renderer
    ) {
        self.frames = frames
        self.minimumInterval = minimumInterval
        self.render = render
    }

    convenience init(frames: @escaping @MainActor () -> AsyncStream<ARFrameSnapshot>) {
        let renderer = CalibrationFrameRenderer()
        self.init(frames: frames) { renderer.image(for: $0) }
    }

    func start() {
        guard task == nil else { return }
        lastRenderedTimestamp = nil
        task = Task { [weak self, frames] in
            for await snapshot in frames() {
                guard let self, !Task.isCancelled else { return }
                if let lastRenderedTimestamp,
                   snapshot.timestamp - lastRenderedTimestamp < minimumInterval {
                    continue
                }
                lastRenderedTimestamp = snapshot.timestamp
                latestRenderedFrameContext = SilentSearchCalibrationFrameContext(
                    frameID: snapshot.id, monotonicTimestamp: snapshot.timestamp
                )
                image = render(snapshot)
            }
        }
    }

    func stop() {
        task?.cancel()
        task = nil
        lastRenderedTimestamp = nil
        image = nil
        latestRenderedFrameContext = nil
    }
}

@MainActor
private final class CalibrationFrameRenderer {
    private let context = CIContext()

    func image(for snapshot: ARFrameSnapshot) -> UIImage? {
        let oriented = CIImage(cvPixelBuffer: snapshot.image).oriented(.right)
        guard let cgImage = context.createCGImage(oriented, from: oriented.extent) else { return nil }
        return UIImage(cgImage: cgImage)
    }
}

enum CalibrationStageState: Equatable {
    case waiting
    case pending
    case succeeded
}

struct CalibrationStages: Equatable {
    let qrDecoded: CalibrationStageState
    let cornersGrounded: CalibrationStageState
    let sampleAccepted: CalibrationStageState
}

struct CalibrationViewProjection: Equatable {
    let markerText: String?
    let visibleCorners: OrientedMarkerCorners?
    let stages: CalibrationStages
    let guidance: String

    init(
        state: SilentSearchCalibrationVisualState,
        expectedMarkerID: String = "SILENT_SEARCH_01",
        latestRenderedFrameContext: SilentSearchCalibrationFrameContext? = nil
    ) {
        markerText = state.recentDetections.last.map { "Marker \($0.markerID)" }
        visibleCorners = state.detection(matching: latestRenderedFrameContext)?.corners
        stages = CalibrationStages(
            qrDecoded: state.qrDecoded ? .succeeded : .pending,
            cornersGrounded: state.cornersGrounded ? .succeeded : (state.qrDecoded ? .pending : .waiting),
            sampleAccepted: state.sampleAccepted ? .succeeded : (state.cornersGrounded ? .pending : .waiting)
        )
        guidance = Self.guidance(for: state, expectedMarkerID: expectedMarkerID)
    }

    private static func guidance(
        for state: SilentSearchCalibrationVisualState, expectedMarkerID: String
    ) -> String {
        switch state.currentIssue {
        case .trackingNotNormal, .groundingFailure(.trackingNotNormal), .groundingFailure(.generationMismatch),
             .groundingFailure(.frameMismatch), .groundingFailure(.timestampMismatch):
            "Restore normal AR tracking before calibrating."
        case .scannerFailure:
            "QR scanner unavailable. Reframe and try again."
        case .groundingFailure(.wrongMarkerID), .groundingFailure(.invalidPayload):
            "Show marker \(expectedMarkerID)."
        case .groundingFailure:
            "Move the marker toward center or adjust the camera angle."
        case let .calibrationRejection(reason):
            rejectionGuidance(reason, expectedMarkerID: expectedMarkerID)
        case nil where state.cornersGrounded:
            "Hold steady while samples are collected."
        case nil:
            "Center the complete marker with its white border visible."
        }
    }

    private static func rejectionGuidance(
        _ reason: SharedMissionCalibrationDiagnostic, expectedMarkerID: String
    ) -> String {
        switch reason {
        case .invalidMarkerID:
            "Use a marker with a valid identifier."
        case .unexpectedMarkerID:
            "Show marker \(expectedMarkerID)."
        case .generationMismatch:
            "AR session changed. Restart calibration."
        case .duplicateFrame:
            "Keep the marker visible while waiting for a new camera frame."
        case .nonFiniteObservation:
            "Reframe the complete marker and try again."
        case .degenerateCorners:
            "Flatten the marker and keep its complete border visible."
        case .timeWindowExceeded:
            "Calibration took too long. Hold the marker steady and try again."
        case .originDeviationExceeded, .headingDeviationExceeded, .widthDeviationExceeded:
            "Marker samples disagree. Hold the marker still and try again."
        }
    }
}
