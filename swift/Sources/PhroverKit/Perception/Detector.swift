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
public final class Detector: @unchecked Sendable {
    // Vision requests are serial; reading the preview cache never waits for
    // inference. Async follow callers retain one active and one newest job.
    private let inferenceLock = NSRecursiveLock()
    private let cacheLock = NSLock()
    private let jobLock = NSLock()
    private let worker: DispatchQueue
    private static func inferenceWorker() -> DispatchQueue {
        DispatchQueue(label: "us.astral.follow-inference", qos: .userInitiated)
    }
    private var workerRunning = false
    private var pendingJob: InferenceJob?
    private var followConsumers: Set<UUID> = []
    private struct InferenceJob: Sendable {
        let snapshot: ARFrameSnapshot
        let cancellation: InferenceCancellation
        let continuation: CheckedContinuation<EvaluationReceipt?, Never>
        let previewOnly: Bool
        let includeScreenFallback: Bool
    }
    private final class InferenceCancellation: @unchecked Sendable {
        private let lock = NSLock()
        private var value = false
        var cancelled: Bool { lock.withLock { value } }
        func cancel() { lock.withLock { value = true } }
    }
    public enum EvaluationStatus: String, Sendable { case executed, failed }
    public enum FailureReason: String, Sendable {
        case unavailable = "detector_unavailable", inferenceFailed = "inference_failed"
    }
    public struct EvaluationReceipt: Sendable {
        public let frame: FrameDetections
        public let status: EvaluationStatus
        public let failureReason: FailureReason?
        /// Orientation actually evaluated for these boxes; nil for unknown legacy handlers/failure.
        public let orientation: CGImagePropertyOrientation?
        public let personVerification: [PersonBodyVerifier.Decision]?
        init(frame: FrameDetections, status: EvaluationStatus, failureReason: FailureReason?,
             orientation: CGImagePropertyOrientation?, personVerification: [PersonBodyVerifier.Decision]? = nil) {
            self.frame = frame; self.status = status; self.failureReason = failureReason
            self.orientation = orientation; self.personVerification = personVerification
        }
    }
    public struct FollowEvaluation: Sendable {
        public let snapshot: ARFrameSnapshot
        public let receipt: EvaluationReceipt
    }
    private var cachedFollowEvaluation: FollowEvaluation?
    public var latestFollowEvaluation: FollowEvaluation? { cacheLock.withLock { cachedFollowEvaluation } }
    public struct Detection: Sendable {
        public let label: String
        public let confidence: Float
        public let boundingBox: CGRect // normalized, Vision coords
    }

    public struct FrameDetections: Sendable {
        public let frameID: ARFrameID
        public let monotonicTimestamp: TimeInterval
        public let detections: [Detection]

        public init(frameID: ARFrameID, monotonicTimestamp: TimeInterval, detections: [Detection]) {
            self.frameID = frameID
            self.monotonicTimestamp = monotonicTimestamp
            self.detections = detections
        }
    }

    private var request: VNCoreMLRequest?
    private let detectionHandler: ((CVPixelBuffer) throws -> [Detection])?
    private var visionHandler: ((CVPixelBuffer, CGImagePropertyOrientation) throws -> [Detection])?
    private let bodyPoseHandler: ((CVPixelBuffer, CGImagePropertyOrientation) throws -> [PersonBodyVerifier.Body])?
    private let screenLocalizer: ScreenLocalizing?
    private let now: () -> TimeInterval
    private let fallbackInterval: TimeInterval
    private let inferenceStateLock = NSLock()
    private var inferenceEnabled = true
    private var screenInferenceRunning = false
    private var lastScreenInferenceAt: TimeInterval?
    public var isLoaded: Bool { request != nil }
    public var isScreenFallbackLoaded: Bool { screenLocalizer?.isLoaded == true }
    public private(set) var supportedCanonicalLabels: Set<String> = []

    /// Loads Xcode's compiled `.mlmodelc` when available, with source model fallback for
    /// package contexts that still ship `.mlpackage`/`.mlmodel` resources.
    public init(modelName: String = "RoverYOLO") async {
        worker = Self.inferenceWorker()
        detectionHandler = nil
        bodyPoseHandler = Self.humanBodies
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
            let coreMLModel = try MLModel(contentsOf: loadURL, configuration: Self.modelConfiguration())
            supportedCanonicalLabels = Self.canonicalLabels(from: coreMLModel.modelDescription.classLabels ?? [])
            let model = try VNCoreMLModel(for: coreMLModel)
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
                "resource": modelURL.lastPathComponent
            ])
        }
    }

    init(supportedLabels: Set<String>, detectionHandler: @escaping (CVPixelBuffer) throws -> [Detection],
         bodyPoseHandler: ((CVPixelBuffer, CGImagePropertyOrientation) throws -> [PersonBodyVerifier.Body])? = nil,
         inferenceQueue: DispatchQueue? = nil) {
        worker = inferenceQueue ?? Self.inferenceWorker()
        screenLocalizer = nil
        now = { ProcessInfo.processInfo.systemUptime }
        fallbackInterval = 0.5
        request = nil
        self.detectionHandler = detectionHandler
        self.bodyPoseHandler = bodyPoseHandler
        supportedCanonicalLabels = supportedLabels
    }

    /// Exercises the same orientation selection as the production Vision request.
    init(supportedLabels: Set<String>, visionHandler: @escaping (CVPixelBuffer, CGImagePropertyOrientation) throws -> [Detection],
         bodyPoseHandler: ((CVPixelBuffer, CGImagePropertyOrientation) throws -> [PersonBodyVerifier.Body])? = nil) {
        worker = Self.inferenceWorker()
        screenLocalizer = nil
        now = { ProcessInfo.processInfo.systemUptime }
        fallbackInterval = 0.5
        request = nil
        detectionHandler = nil
        self.visionHandler = visionHandler
        self.bodyPoseHandler = bodyPoseHandler
        supportedCanonicalLabels = supportedLabels
    }

    init(primaryDetection: @escaping (CVPixelBuffer) throws -> [Detection],
         screenLocalizer: ScreenLocalizing?, now: @escaping () -> TimeInterval,
         fallbackInterval: TimeInterval = 0.5) {
        worker = Self.inferenceWorker()
        request = nil
        detectionHandler = primaryDetection
        bodyPoseHandler = nil
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

    static func canonicalLabels(from classLabels: [Any]) -> Set<String> {
        Set(classLabels.compactMap { $0 as? String })
    }

    /// Evaluate only the snapshot's upright orientation. Retry rotations would mix
    /// boxes with incompatible depth and camera coordinates.
    public func detect(_ pixelBuffer: CVPixelBuffer,
                       orientation: CGImagePropertyOrientation = .right) -> [Detection] {
        evaluate(pixelBuffer, orientation: orientation, includeScreenFallback: true).detections
    }

    private func evaluate(_ pixelBuffer: CVPixelBuffer, orientation: CGImagePropertyOrientation,
                          includeScreenFallback: Bool)
        -> (detections: [Detection], reason: FailureReason?, orientation: CGImagePropertyOrientation?) {
        inferenceLock.lock()
        defer { inferenceLock.unlock() }
        let primary: [Detection]
        var failure: FailureReason?
        do {
            if let detectionHandler { primary = try detectionHandler(pixelBuffer) }
            else if let visionHandler { primary = try visionHandler(pixelBuffer, orientation) }
            else if let request { primary = try detect(pixelBuffer, request: request, orientation: orientation) }
            else { primary = []; failure = .unavailable }
        } catch {
            RuntimeFileLog.append("detector_failed", fields: [
                "orientation": "\(orientation.rawValue)", "reason": FailureReason.inferenceFailed.rawValue])
            primary = []
            failure = .inferenceFailed
        }
        let detections = includeScreenFallback
            ? screenDetections(primary: primary, image: pixelBuffer, orientation: orientation) : primary
        return (detections, failure, failure == nil || !detections.isEmpty ? orientation : nil)
    }

    public func evaluate(_ snapshot: ARFrameSnapshot) -> EvaluationReceipt {
        let result = evaluate(snapshot.image, orientation: snapshot.imageOrientation, includeScreenFallback: true)
        return EvaluationReceipt(frame: .init(frameID: snapshot.id, monotonicTimestamp: snapshot.timestamp,
                                             detections: result.detections),
                                 status: result.reason == nil ? .executed : .failed, failureReason: result.reason,
                                 orientation: result.orientation)
    }

    /// Follow and body verification share the coherent snapshot orientation.
    public func evaluateForFollow(_ snapshot: ARFrameSnapshot) -> EvaluationReceipt {
        inferenceLock.lock()
        defer { inferenceLock.unlock() }
        let result = evaluate(snapshot.image, orientation: snapshot.imageOrientation, includeScreenFallback: false)
        let people = result.detections.filter { $0.label.lowercased() == "person" }
        let verification: [PersonBodyVerifier.Decision]
        if result.reason != nil || people.isEmpty { verification = [] }
        else if let bodyPoseHandler {
            do {
                let bodies = try bodyPoseHandler(snapshot.image, snapshot.imageOrientation)
                verification = people.enumerated().map { id, detection in
                    PersonBodyVerifier.verify(box: detection.boundingBox, rawPersonID: id, bodies: bodies)
                }
            } catch {
                verification = people.indices.map { .init(rawPersonID: $0, accepted: false,
                    reason: "body_verification_failed", matchingBodies: 0) }
            }
        } else {
            verification = people.indices.map { .init(rawPersonID: $0, accepted: false,
                reason: "body_verification_unavailable", matchingBodies: 0) }
        }
        let receipt = EvaluationReceipt(frame: .init(frameID: snapshot.id, monotonicTimestamp: snapshot.timestamp,
                                              detections: result.detections),
                                  status: result.reason == nil ? .executed : .failed,
                                  failureReason: result.reason, orientation: result.orientation,
                                  personVerification: verification)
        cacheLock.withLock { cachedFollowEvaluation = .init(snapshot: snapshot, receipt: receipt) }
        return receipt
    }

    @MainActor public func evaluateForFollowAsync(_ snapshot: ARFrameSnapshot) async -> EvaluationReceipt? {
        await evaluateAsync(snapshot, previewOnly: false)
    }

    /// Mission perception retains generic screen fallback, evaluated against its
    /// own captured orientation and geometry. Active follow consumers take priority.
    @MainActor public func evaluateForMissionAsync(_ snapshot: ARFrameSnapshot) async -> FollowEvaluation? {
        guard let receipt = await evaluateAsync(snapshot, previewOnly: true, includeScreenFallback: true),
              !Task.isCancelled else { return nil }
        return .init(snapshot: snapshot, receipt: receipt)
    }

    func beginFollowConsumer() -> UUID {
        let token = UUID()
        let dropped: InferenceJob? = jobLock.withLock {
            followConsumers.insert(token)
            guard pendingJob?.previewOnly == true else { return nil }
            let old = pendingJob; pendingJob = nil; return old
        }
        dropped?.continuation.resume(returning: nil)
        return token
    }

    func endFollowConsumer(_ token: UUID) { _ = jobLock.withLock { followConsumers.remove(token) } }

    @MainActor private func evaluateAsync(_ snapshot: ARFrameSnapshot, previewOnly: Bool,
                                         includeScreenFallback: Bool = false) async -> EvaluationReceipt? {
        let cancellation = InferenceCancellation()
        return await withTaskCancellationHandler {
            guard !Task.isCancelled else { return nil }
            return await withCheckedContinuation { continuation in
                let job = InferenceJob(snapshot: snapshot, cancellation: cancellation, continuation: continuation,
                    previewOnly: previewOnly, includeScreenFallback: includeScreenFallback)
                var superseded: InferenceJob?
                var skipped = false
                let start = jobLock.withLock {
                    if previewOnly && !followConsumers.isEmpty { skipped = true; return false }
                    if workerRunning { superseded = pendingJob; pendingJob = job; return false }
                    workerRunning = true
                    return true
                }
                if skipped { continuation.resume(returning: nil); return }
                superseded?.continuation.resume(returning: nil)
                if start { worker.async { self.run(job) } }
            }
        } onCancel: { cancellation.cancel() }
    }

    private func run(_ job: InferenceJob) {
        // A preview may already be on the dispatch queue when follow begins.
        // Arbitrate at native execution, after any synchronous inference drains.
        inferenceLock.lock()
        let permitted = jobLock.withLock { !job.previewOnly || followConsumers.isEmpty }
        let receipt: EvaluationReceipt?
        if !permitted || job.cancellation.cancelled { receipt = nil }
        else { receipt = job.includeScreenFallback ? evaluate(job.snapshot) : evaluateForFollow(job.snapshot) }
        inferenceLock.unlock()
        job.continuation.resume(returning: job.cancellation.cancelled ? nil : receipt)
        let next: InferenceJob? = jobLock.withLock {
            guard let next = pendingJob else { workerRunning = false; return nil }
            pendingJob = nil
            return next
        }
        if let next { worker.async { self.run(next) } }
    }

    @MainActor public func followPreviewEvaluationAsync(_ snapshot: ARFrameSnapshot, at uptime: TimeInterval) async -> FollowEvaluation? {
        if let cached = latestFollowEvaluation, uptime.isFinite,
           cached.snapshot.id.generation == snapshot.id.generation, cached.snapshot.id.sequence <= snapshot.id.sequence,
           cached.snapshot.timestamp <= snapshot.timestamp,
           uptime - cached.snapshot.timestamp >= 0, uptime - cached.snapshot.timestamp <= 0.5 {
            return cached
        }
        guard let receipt = await evaluateAsync(snapshot, previewOnly: true), !Task.isCancelled else { return nil }
        return .init(snapshot: snapshot, receipt: receipt)
    }

    /// Preview and receipt always refer to the same upright snapshot.
    public func followPreviewEvaluation(_ snapshot: ARFrameSnapshot, at uptime: TimeInterval) -> FollowEvaluation {
        if let cached = latestFollowEvaluation, uptime.isFinite,
           cached.snapshot.id.generation == snapshot.id.generation,
           cached.snapshot.id.sequence <= snapshot.id.sequence,
           cached.snapshot.timestamp <= snapshot.timestamp,
           uptime - cached.snapshot.timestamp >= 0, uptime - cached.snapshot.timestamp <= 0.5 {
            return cached
        }
        let receipt = evaluateForFollow(snapshot)
        return .init(snapshot: snapshot, receipt: receipt)
    }

    private enum BodyVerificationError: Error { case noSupportedDevice }
    static func humanBodies(_ image: CVPixelBuffer, _ orientation: CGImagePropertyOrientation) throws -> [PersonBodyVerifier.Body] {
        let request = VNDetectHumanBodyPoseRequest()
        // Prefer supported Neural Engine work while preserving the detector's
        // no-GPU background-wind-down policy; CPU remains the fallback.
        for (stage, devices) in try request.supportedComputeStageDevices {
            guard let device = devices.first(where: { if case .neuralEngine = $0 { return true }; return false }) ??
                    devices.first(where: { if case .cpu = $0 { return true }; return false }) else {
                throw BodyVerificationError.noSupportedDevice
            }
            request.setComputeDevice(device, for: stage)
        }
        try VNImageRequestHandler(cvPixelBuffer: image, orientation: orientation).perform([request])
        return try (request.results ?? []).map { observation in
            let points = try observation.recognizedPoints(.all)
            func joint(_ name: VNHumanBodyPoseObservation.JointName) -> PersonBodyVerifier.Joint? {
                points[name].map { .init(location: $0.location, confidence: $0.confidence) }
            }
            return .init(leftShoulder: joint(.leftShoulder), rightShoulder: joint(.rightShoulder),
                leftHip: joint(.leftHip), rightHip: joint(.rightHip))
        }
    }

    public func detect(_ snapshot: ARFrameSnapshot) -> FrameDetections {
        FrameDetections(frameID: snapshot.id, monotonicTimestamp: snapshot.timestamp,
                        detections: detect(snapshot.image, orientation: snapshot.imageOrientation))
    }

    private func detect(_ pixelBuffer: CVPixelBuffer,
                        request: VNCoreMLRequest,
                         orientation: CGImagePropertyOrientation) throws -> [Detection] {
        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: orientation)
        do {
            try handler.perform([request])
        } catch {
            throw error
        }
        guard let results = request.results as? [VNRecognizedObjectObservation] else { return [] }
        return results.map {
            Detection(label: $0.labels.first?.identifier ?? "object",
                      confidence: $0.labels.first?.confidence ?? 0,
                      boundingBox: $0.boundingBox)
        }
    }

    private func screenDetections(primary: [Detection], image: CVPixelBuffer,
                                  orientation: CGImagePropertyOrientation) -> [Detection] {
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
        let fallback = screenLocalizer.detect(image, orientation: orientation)
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
