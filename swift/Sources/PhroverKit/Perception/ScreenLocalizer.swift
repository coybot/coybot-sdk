import CoreML
import CoreVideo
import Foundation
import ImageIO
import Vision

protocol ScreenLocalizing: AnyObject {
    var isLoaded: Bool { get }
    func detect(_ pixelBuffer: CVPixelBuffer, orientation: CGImagePropertyOrientation) -> [Detector.Detection]
}

final class VisionScreenLocalizer: ScreenLocalizing {
    private var request: VNCoreMLRequest?
    var isLoaded: Bool { request != nil }

    init(modelName: String = "RoverYOLO") async {
        guard let modelURL = Detector.modelResourceURL(modelName: modelName) else {
            RuntimeFileLog.append("screen_detector_unavailable", fields: [
                "model": modelName,
                "reason": "model_resource_missing",
            ])
            return
        }

        do {
            let loadURL = modelURL.pathExtension == "mlmodelc"
                ? modelURL
                : try await MLModel.compileModel(at: modelURL)
            let model = try MLModel(contentsOf: loadURL, configuration: Detector.modelConfiguration())
            let request = VNCoreMLRequest(model: try VNCoreMLModel(for: model))
            request.imageCropAndScaleOption = .scaleFill
            self.request = request
            RuntimeFileLog.append("screen_detector_loaded", fields: [
                "resource": modelURL.lastPathComponent,
            ])
        } catch {
            RuntimeFileLog.append("screen_detector_unavailable", fields: [
                "error": error.localizedDescription,
                "reason": "model_load_failed",
                "resource": modelURL.lastPathComponent,
            ])
        }
    }

    /// Same single-orientation contract as `Detector.detect(_:orientation:)`: boxes must
    /// come back in the upright frame `ARSessionManager.unproject` inverts.
    func detect(_ pixelBuffer: CVPixelBuffer, orientation: CGImagePropertyOrientation) -> [Detector.Detection] {
        guard let request else { return [] }
        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: orientation)
        do {
            try handler.perform([request])
        } catch {
            RuntimeFileLog.append("screen_detector_failed", fields: [
                "error": error.localizedDescription,
                "orientation": "\(orientation.rawValue)",
            ])
            return []
        }
        guard let observations = request.results as? [VNRecognizedObjectObservation] else {
            return []
        }
        return observations.compactMap { observation -> Detector.Detection? in
            guard let label = observation.labels.first else { return nil }
            return Detector.Detection(
                label: label.identifier,
                confidence: label.confidence,
                boundingBox: observation.boundingBox
            )
        }.filter { ScreenDetectionPolicy.isScreenLike($0.label) }
    }
}
