import Foundation
import Vision
import CoreML
import CoreVideo
import ImageIO

/// On-device object detector (person + generic obstacles) via a CoreML model wrapped
/// in Vision. Ships with a bundled YOLO-family `.mlpackage`; swap the model name to use
/// a custom-trained one.
///
/// Detections feed two consumers: `ObstacleGuard` (people directly ahead) and dialog/
/// "who's there" behaviors. This is the on-device half of the hybrid AI split — heavy
/// vision-language reasoning can be offloaded to the cloud via `DialogEscalating`.
public final class Detector {
    public struct Detection: Sendable {
        public let label: String
        public let confidence: Float
        public let boundingBox: CGRect // normalized, Vision coords
    }

    private var request: VNCoreMLRequest?
    public var isLoaded: Bool { request != nil }

    /// Loads Xcode's compiled `.mlmodelc` when available, with source model fallback for
    /// package contexts that still ship `.mlpackage`/`.mlmodel` resources.
    public init(modelName: String = "RoverYOLO") async {
        guard let modelURL = Self.modelResourceURL(modelName: modelName) else {
            request = nil
            RuntimeFileLog.append("detector_unavailable", fields: [
                "reason": "model_resource_missing",
                "model": modelName
            ])
            return
        }

        do {
            let loadURL: URL
            if modelURL.pathExtension == "mlmodelc" {
                loadURL = modelURL
            } else {
                loadURL = try await MLModel.compileModel(at: modelURL)
            }
            let model = try VNCoreMLModel(for: MLModel(contentsOf: loadURL,
                                                       configuration: Self.modelConfiguration()))
            let req = VNCoreMLRequest(model: model)
            req.imageCropAndScaleOption = .scaleFill
            request = req
            RuntimeFileLog.append("detector_loaded", fields: [
                "resource": modelURL.lastPathComponent
            ])
        } catch {
            request = nil
            RuntimeFileLog.append("detector_unavailable", fields: [
                "reason": "model_load_failed",
                "resource": modelURL.lastPathComponent,
                "error": error.localizedDescription
            ])
        }
    }

    static func modelResourceURL(modelName: String, bundle: Bundle = .module) -> URL? {
        bundle.url(forResource: modelName, withExtension: "mlmodelc")
            ?? bundle.url(forResource: modelName, withExtension: "mlpackage")
            ?? bundle.url(forResource: modelName, withExtension: "mlmodel")
    }

    static func modelConfiguration() -> MLModelConfiguration {
        let configuration = MLModelConfiguration()
        // iOS denies GPU/Metal command buffers once the app is backgrounded. Keeping the
        // detector off GPU avoids MPSGraph background-execution crashes from live preview
        // or mission perception work that is winding down during app lifecycle changes.
        configuration.computeUnits = .cpuAndNeuralEngine
        return configuration
    }

    /// Runs the model on the buffer rotated upright by `orientation` (normally
    /// `ARSessionManager.imageOrientation`). Boxes come back in that upright frame, which is
    /// the frame `ARSessionManager.unproject` inverts — so pass the same orientation to both.
    ///
    /// Deliberately one orientation, not a retry over all four: boxes from a different
    /// rotation are in a different frame and unproject to the wrong place, and retrying
    /// quadruples inference on every frame with nothing in view.
    public func detect(_ pixelBuffer: CVPixelBuffer, orientation: CGImagePropertyOrientation = .right) -> [Detection] {
        guard let request else { return [] }
        return detect(pixelBuffer, request: request, orientation: orientation)
    }

    private func detect(_ pixelBuffer: CVPixelBuffer,
                        request: VNCoreMLRequest,
                        orientation: CGImagePropertyOrientation) -> [Detection] {
        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: orientation)
        do {
            try handler.perform([request])
        } catch {
            RuntimeFileLog.append("detector_failed", fields: [
                "orientation": "\(orientation.rawValue)",
                "error": error.localizedDescription
            ])
            return []
        }
        guard let results = request.results as? [VNRecognizedObjectObservation] else { return [] }
        return results.map {
            Detection(label: $0.labels.first?.identifier ?? "object",
                      confidence: $0.labels.first?.confidence ?? 0,
                      boundingBox: $0.boundingBox)
        }
    }
}
