import Foundation
import Vision

struct FollowVisualTrackingDiagnostics: Sendable {
    enum Mode: String, Sendable { case fullDetection = "full_detection", tracked }
    let mode: Mode
    let reason: String
    let region: CGRect
    let detectionAge: Double
}

/// Boxes and regions always use the original upright image's normalized Vision
/// coordinates. A region of interest does not change AR camera/depth calibration.
protocol FollowObjectTracking: AnyObject {
    func seed(box: CGRect, snapshot: ARFrameSnapshot) throws
    func track(snapshot: ARFrameSnapshot, region: CGRect) throws -> (box: CGRect, confidence: Float)?
    func reset()
}

final class VisionFollowObjectTracker: FollowObjectTracking {
    private var handler = VNSequenceRequestHandler()
    private var request: VNTrackObjectRequest?
    private var observation: VNDetectedObjectObservation?

    func reset() {
        request = nil; observation = nil
        handler = VNSequenceRequestHandler()
    }

    func seed(box: CGRect, snapshot: ARFrameSnapshot) throws {
        reset()
        let request = VNTrackObjectRequest(detectedObjectObservation: .init(boundingBox: box))
        request.trackingLevel = .fast
        // Preserve the detector's no-GPU wind-down policy. The lightweight
        // tracker uses CPU when supported, leaving the Neural Engine for models.
        for (stage, devices) in try request.supportedComputeStageDevices {
            guard let cpu = devices.first(where: { if case .cpu = $0 { return true }; return false }) else {
                throw TrackingError.unsupportedCompute
            }
            request.setComputeDevice(cpu, for: stage)
        }
        try handler.perform([request], on: snapshot.image, orientation: snapshot.imageOrientation)
        guard let observation = request.results?.first as? VNDetectedObjectObservation else { throw TrackingError.noObservation }
        self.request = request
        self.observation = observation
    }

    func track(snapshot: ARFrameSnapshot, region: CGRect) throws -> (box: CGRect, confidence: Float)? {
        guard let request, let observation else { return nil }
        request.inputObservation = observation
        request.regionOfInterest = region
        try handler.perform([request], on: snapshot.image, orientation: snapshot.imageOrientation)
        guard let next = request.results?.first as? VNDetectedObjectObservation else { return nil }
        self.observation = next
        return (next.boundingBox, next.confidence)
    }

    private enum TrackingError: Error { case unsupportedCompute, noObservation }
}

/// Confined to Detector's serial inference worker/lock. Full detection and body
/// revalidation are periodic; every intermediate tracked box also needs CURRENT
/// body evidence before it can become a movement-authorizing observation.
final class FollowVisualTrackingPipeline {
    static let fullFrame = CGRect(x: 0, y: 0, width: 1, height: 1)
    static let revalidationInterval = 0.4
    private let tracker: any FollowObjectTracking
    private struct Source {
        let id: ARFrameID
        let timestamp: Double
        let yaw: Double
        let orientation: UInt32
        let resolution: CGSize
        init(_ snapshot: ARFrameSnapshot) {
            id = snapshot.id; timestamp = snapshot.timestamp; yaw = snapshot.pose.yaw
            orientation = snapshot.imageOrientation.rawValue; resolution = snapshot.imageResolution
        }
    }
    private var lastSnapshot: Source?
    private var lastDetectionTime: Double?
    private var box: CGRect?
    private var displacement = CGPoint.zero

    init(tracker: any FollowObjectTracking = VisionFollowObjectTracker()) { self.tracker = tracker }

    func reset() {
        tracker.reset()
        lastSnapshot = nil; lastDetectionTime = nil; box = nil; displacement = .zero
    }

    static func valid(_ box: CGRect) -> Bool {
        [box.minX, box.minY, box.width, box.height].allSatisfy(\.isFinite) &&
            box.width > 0.02 && box.height > 0.04 && box.minX > 0.01 && box.minY > 0.01 && box.maxX < 0.99 && box.maxY < 0.99
    }

    static func region(around box: CGRect, displacement: CGPoint) -> CGRect {
        let horizontal = max(0.12, box.width * 0.6) + abs(displacement.x) * 2
        let vertical = max(0.10, box.height * 0.3) + abs(displacement.y) * 2
        return box.insetBy(dx: -horizontal, dy: -vertical).intersection(fullFrame)
    }

    func evaluate(_ snapshot: ARFrameSnapshot, allowsTracking: Bool,
                  detect: () -> Detector.EvaluationReceipt,
                  verify: (CGRect, Float) -> Detector.EvaluationReceipt) -> Detector.EvaluationReceipt {
        func belongsToFrame(_ receipt: Detector.EvaluationReceipt) -> Bool {
            receipt.frame.frameID == snapshot.id && receipt.frame.monotonicTimestamp == snapshot.timestamp &&
                (receipt.status == .failed || receipt.orientation == snapshot.imageOrientation)
        }
        func full(_ reason: String) -> Detector.EvaluationReceipt {
            reset()
            var reportedReason = reason
            let receipt = detect()
            guard belongsToFrame(receipt) else {
                return Detector.EvaluationReceipt(frame: .init(frameID: snapshot.id,
                    monotonicTimestamp: snapshot.timestamp, detections: []), status: .failed,
                    failureReason: .inferenceFailed, orientation: nil, personVerification: [])
                    .withTracking(.init(mode: .fullDetection, reason: "receipt_mismatch", region: Self.fullFrame, detectionAge: 0))
            }
            let people = receipt.frame.detections.filter { $0.label.lowercased() == "person" }
            if allowsTracking, receipt.status == .executed, people.count == 1,
               people[0].confidence.isFinite, people[0].confidence >= 0.5,
               receipt.personVerification?.filter(\.accepted).count == 1,
               receipt.personVerification?.first(where: \.accepted)?.rawPersonID == 0,
               Self.valid(people[0].boundingBox), snapshot.timestamp.isFinite,
               snapshot.trackingQuality == .normal, snapshot.depthMap != nil {
                do {
                    try tracker.seed(box: people[0].boundingBox, snapshot: snapshot)
                    box = people[0].boundingBox
                    lastSnapshot = Source(snapshot)
                    lastDetectionTime = snapshot.timestamp
                } catch { reset(); reportedReason = "tracker_seed_failed" }
            }
            return receipt.withTracking(.init(mode: .fullDetection, reason: reportedReason,
                region: Self.fullFrame, detectionAge: 0))
        }

        guard allowsTracking else { return full("acquisition_or_reacquisition") }
        guard let previous = lastSnapshot, let box, let detectedAt = lastDetectionTime else { return full("no_verified_track") }
        guard snapshot.trackingQuality == .normal, snapshot.depthMap != nil,
              snapshot.id.generation == previous.id.generation,
              snapshot.id.sequence > previous.id.sequence, snapshot.timestamp.isFinite,
              snapshot.timestamp > previous.timestamp,
              snapshot.timestamp - previous.timestamp <= 0.25,
              snapshot.imageOrientation.rawValue == previous.orientation,
              snapshot.imageResolution == previous.resolution else { return full("source_changed_or_gap") }
        guard snapshot.timestamp - detectedAt < Self.revalidationInterval else { return full("periodic_revalidation") }
        let yawChange = atan2(sin(snapshot.pose.yaw - previous.yaw), cos(snapshot.pose.yaw - previous.yaw))
        guard yawChange.isFinite, abs(yawChange) <= 0.12 else { return full("camera_direction_changed") }
        let roi = Self.region(around: box, displacement: displacement)
        let tracked: (box: CGRect, confidence: Float)
        do {
            guard let result = try tracker.track(snapshot: snapshot, region: roi) else { return full("tracking_lost") }
            tracked = result
        } catch { return full("tracking_failed") }
        guard tracked.confidence.isFinite, tracked.confidence >= 0.8, Self.valid(tracked.box) else { return full("low_confidence_or_clipped") }
        let delta = CGPoint(x: tracked.box.midX - box.midX, y: tracked.box.midY - box.midY)
        let scale = tracked.box.width * tracked.box.height / (box.width * box.height)
        guard hypot(delta.x, delta.y) <= 0.15, (0.6...1.6).contains(scale) else { return full("track_jump") }
        let step = hypot(delta.x, delta.y), previousStep = hypot(displacement.x, displacement.y)
        let directionChanged = step > 0.02 && previousStep > 0.02 &&
            delta.x * displacement.x + delta.y * displacement.y <= cos(.pi / 4) * step * previousStep
        guard !directionChanged, roi.insetBy(dx: 0.015, dy: 0.015).contains(tracked.box) else { return full("direction_change_or_crop_edge") }
        let receipt = verify(tracked.box, tracked.confidence)
        guard belongsToFrame(receipt), receipt.status == .executed, receipt.personVerification?.count == 1,
              receipt.personVerification?.first?.accepted == true,
              receipt.personVerification?.first?.rawPersonID == 0 else { return full("body_revalidation_failed") }
        self.box = tracked.box
        self.lastSnapshot = Source(snapshot)
        displacement = delta
        return receipt.withTracking(.init(mode: .tracked, reason: "verified_continuation", region: roi,
            detectionAge: snapshot.timestamp - detectedAt))
    }
}
