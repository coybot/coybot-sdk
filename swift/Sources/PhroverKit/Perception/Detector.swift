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
    private let primaryDetection: ((CVPixelBuffer) -> [Detection])?
    private let screenLocalizer: ScreenLocalizing?
    private let now: () -> TimeInterval
    private let fallbackInterval: TimeInterval
    private let inferenceStateLock = NSLock()
    private var inferenceEnabled = true
    private var screenInferenceRunning = false
    private var lastScreenInferenceAt: TimeInterval?
    public var isLoaded: Bool { request != nil }
    public var isScreenFallbackLoaded: Bool { screenLocalizer?.isLoaded == true }

    /// Loads Xcode's compiled `.mlmodelc` when available, with source model fallback for
    /// package contexts that still ship `.mlpackage`/`.mlmodel` resources.
    public init(modelName: String = "RoverYOLO") async {
        primaryDetection = nil
        screenLocalizer = await VisionScreenLocalizer()
        now = { ProcessInfo.processInfo.systemUptime }
        fallbackInterval = 0.5
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

    init(primaryDetection: @escaping (CVPixelBuffer) -> [Detection],
         screenLocalizer: ScreenLocalizing?,
         now: @escaping () -> TimeInterval,
         fallbackInterval: TimeInterval = 0.5) {
        request = nil
        self.primaryDetection = primaryDetection
        self.screenLocalizer = screenLocalizer
        self.now = now
        self.fallbackInterval = fallbackInterval
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
    /// quadruples inference on every frame with nothing in view. The screen-localizer
    /// fallback runs under the same orientation for the same reason.
    public func detect(_ pixelBuffer: CVPixelBuffer, orientation: CGImagePropertyOrientation = .right) -> [Detection] {
        let primary: [Detection]
        if let primaryDetection {
            primary = primaryDetection(pixelBuffer)
        } else if let request {
            primary = detect(pixelBuffer, request: request, orientation: orientation)
        } else {
            primary = []
        }

        guard ScreenDetectionPolicy.shouldRunFallback(for: primary),
              let screenLocalizer,
              screenLocalizer.isLoaded,
              beginScreenInferenceIfAllowed() else {
            return primary
        }
        defer { finishScreenInference() }

        RuntimeFileLog.append("screen_detector_inference_started", fields: [
            "primary_count": "\(primary.count)",
        ])
        let fallback = screenLocalizer.detect(pixelBuffer, orientation: orientation)
        guard isInferenceEnabled else {
            RuntimeFileLog.append("screen_detector_result_discarded", fields: [
                "reason": "inference_disabled",
            ])
            return primary
        }

        let merged = ScreenDetectionPolicy.merge(primary: primary, fallback: fallback)
        let accepted = merged.dropFirst(primary.count)
        if accepted.isEmpty {
            RuntimeFileLog.append("screen_detector_candidate_rejected", fields: [
                "candidate_count": "\(fallback.count)",
                "reason": fallback.isEmpty ? "no_candidate" : "confidence_label_or_box",
            ])
        } else {
            for detection in accepted {
                RuntimeFileLog.append("screen_detector_candidate_accepted", fields: [
                    "box": Self.boundingBoxDescription(detection.boundingBox),
                    "confidence": String(format: "%.2f", detection.confidence),
                    "label": detection.label,
                ])
            }
        }
        return merged
    }

    public func setInferenceEnabled(_ enabled: Bool) {
        inferenceStateLock.lock()
        inferenceEnabled = enabled
        inferenceStateLock.unlock()
        RuntimeFileLog.append("detector_inference_state_changed", fields: [
            "enabled": enabled ? "true" : "false",
        ])
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

    private var isInferenceEnabled: Bool {
        inferenceStateLock.lock()
        defer { inferenceStateLock.unlock() }
        return inferenceEnabled
    }

    private func beginScreenInferenceIfAllowed() -> Bool {
        let timestamp = now()
        inferenceStateLock.lock()
        defer { inferenceStateLock.unlock() }
        guard inferenceEnabled else { return false }
        guard !screenInferenceRunning else {
            RuntimeFileLog.append("screen_detector_inference_skipped", fields: [
                "reason": "request_in_progress",
            ])
            return false
        }
        if let lastScreenInferenceAt,
           timestamp - lastScreenInferenceAt < fallbackInterval {
            RuntimeFileLog.append("screen_detector_inference_skipped", fields: [
                "reason": "rate_limited",
            ])
            return false
        }
        lastScreenInferenceAt = timestamp
        screenInferenceRunning = true
        return true
    }

    private func finishScreenInference() {
        inferenceStateLock.lock()
        screenInferenceRunning = false
        inferenceStateLock.unlock()
    }

    private static func boundingBoxDescription(_ box: CGRect) -> String {
        String(
            format: "x=%.3f,y=%.3f,w=%.3f,h=%.3f",
            box.minX,
            box.minY,
            box.width,
            box.height
        )
    }
}
