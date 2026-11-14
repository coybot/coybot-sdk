import XCTest
import CoreVideo
import ImageIO
import simd
@testable import PhroverKit

final class FollowVisualTrackingPipelineTests: XCTestCase {
    private let box = CGRect(x: 0.4, y: 0.25, width: 0.2, height: 0.5)

    func testNativeVisionTrackingRegionPreservesFullImageCoordinates() throws {
        let startBox = CGRect(x: 0.2, y: 0.3, width: 0.2, height: 0.35)
        func image(_ shift: Int, sequence: UInt64) throws -> ARFrameSnapshot {
            var buffer: CVPixelBuffer?
            CVPixelBufferCreate(kCFAllocatorDefault, 256, 256, kCVPixelFormatType_32BGRA, nil, &buffer)
            let image = try XCTUnwrap(buffer)
            CVPixelBufferLockBaseAddress(image, [])
            let bytes = CVPixelBufferGetBaseAddress(image)!.assumingMemoryBound(to: UInt8.self)
            let stride = CVPixelBufferGetBytesPerRow(image)
            for y in 0..<256 { for x in 0..<256 {
                let point = CGPoint(x: Double(x - shift) / 256, y: 1 - Double(y) / 256)
                let value: UInt8 = startBox.contains(point) ? UInt8(40 + ((x - shift) * 17 + y * 31) % 200) : 12
                let offset = y * stride + x * 4
                bytes[offset] = value; bytes[offset + 1] = value; bytes[offset + 2] = value; bytes[offset + 3] = 255
            } }
            CVPixelBufferUnlockBaseAddress(image, [])
            return .init(id: .init(generation: 1, sequence: sequence), timestamp: Double(sequence) * 0.1,
                image: image, cameraTransform: matrix_identity_float4x4, cameraIntrinsics: matrix_identity_float3x3,
                imageResolution: CGSize(width: 256, height: 256), depthMap: nil,
                pose: .init(position: .zero, yaw: 0), trackingQuality: .normal, imageOrientation: .up)
        }
        let tracker = VisionFollowObjectTracker()
        try tracker.seed(box: startBox, snapshot: image(0, sequence: 1))
        let result = try XCTUnwrap(tracker.track(snapshot: image(6, sequence: 2),
            region: FollowVisualTrackingPipeline.region(around: startBox, displacement: .zero)))
        XCTAssertGreaterThan(result.confidence, 0.5)
        XCTAssertEqual(result.box.midX, startBox.midX + 6.0 / 256, accuracy: 0.04)
        XCTAssertEqual(result.box.midY, startBox.midY, accuracy: 0.04)
        tracker.reset()
    }

    private final class Tracker: FollowObjectTracking {
        var seeds = 0
        var regions: [CGRect] = []
        var result: (box: CGRect, confidence: Float)? = (CGRect(x: 0.41, y: 0.25, width: 0.2, height: 0.5), 0.95)
        var fail = false
        func seed(box: CGRect, snapshot: ARFrameSnapshot) throws { seeds += 1 }
        func track(snapshot: ARFrameSnapshot, region: CGRect) throws -> (box: CGRect, confidence: Float)? {
            regions.append(region)
            if fail { throw URLError(.unknown) }
            return result
        }
        func reset() {}
    }

    private func snapshot(_ sequence: UInt64, time: Double, generation: UInt64 = 1,
                          orientation: CGImagePropertyOrientation = .right, yaw: Double = 0,
                          depth: Bool = true) throws -> ARFrameSnapshot {
        var image: CVPixelBuffer?, map: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 32, 32, kCVPixelFormatType_32BGRA, nil, &image)
        CVPixelBufferCreate(kCFAllocatorDefault, 32, 32, kCVPixelFormatType_DepthFloat32, nil, &map)
        return .init(id: .init(generation: generation, sequence: sequence), timestamp: time,
            image: try XCTUnwrap(image), cameraTransform: matrix_identity_float4x4,
            cameraIntrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 32, height: 32),
            depthMap: depth ? map : nil, pose: .init(position: .zero, yaw: yaw), trackingQuality: .normal,
            imageOrientation: orientation)
    }

    private func receipt(_ frame: ARFrameSnapshot, box: CGRect? = nil, confidence: Float = 0.95,
                         verified: Bool = true, count: Int = 1) -> Detector.EvaluationReceipt {
        .init(frame: .init(frameID: frame.id, monotonicTimestamp: frame.timestamp,
            detections: (0..<count).map { _ in .init(label: "person", confidence: confidence, boundingBox: box ?? self.box) }),
            status: .executed, failureReason: nil, orientation: frame.imageOrientation,
            personVerification: (0..<count).map { .init(rawPersonID: $0, accepted: verified,
                reason: verified ? "verified" : "no_matching_body", matchingBodies: verified ? 1 : 0) })
    }

    func testAcquisitionAlwaysUsesFullFrameAndDoesNotStartTracker() throws {
        let tracker = Tracker(), pipeline = FollowVisualTrackingPipeline(tracker: tracker)
        for i in 1...3 {
            let frame = try snapshot(UInt64(i), time: Double(i) * 0.1)
            let result = pipeline.evaluate(frame, allowsTracking: false, detect: { self.receipt(frame) }, verify: { _, _ in
                XCTFail("Acquisition must not use a tracked candidate"); return self.receipt(frame)
            })
            XCTAssertEqual(result.visualTracking?.mode, .fullDetection)
            XCTAssertEqual(result.visualTracking?.region, FollowVisualTrackingPipeline.fullFrame)
        }
        XCTAssertEqual(tracker.seeds, 0)
        XCTAssertTrue(tracker.regions.isEmpty)
    }

    func testTrackingSkipsDetectorButUsesCurrentFrameBodyEvidenceAndPeriodicFullRevalidation() throws {
        let tracker = Tracker(), pipeline = FollowVisualTrackingPipeline(tracker: tracker)
        var detections = 0, bodies = 0
        for (index, time) in [0.0, 0.1, 0.2, 0.4].enumerated() {
            let frame = try snapshot(UInt64(index + 1), time: time)
            let result = pipeline.evaluate(frame, allowsTracking: true, detect: {
                detections += 1; return self.receipt(frame)
            }, verify: { box, confidence in
                bodies += 1; return self.receipt(frame, box: box, confidence: confidence)
            })
            XCTAssertEqual(result.frame.frameID, frame.id)
            XCTAssertEqual(result.frame.monotonicTimestamp, frame.timestamp)
            XCTAssertEqual(result.personVerification?.first?.accepted, true)
            XCTAssertEqual(result.visualTracking?.mode, index == 0 || index == 3 ? .fullDetection : .tracked)
        }
        XCTAssertEqual(detections, 2)
        XCTAssertEqual(bodies, 2, "Every tracking frame must earn its own body evidence")
        XCTAssertEqual(tracker.regions.count, 2)
        XCTAssertLessThan(tracker.regions[0].width, 1)
        XCTAssertTrue(tracker.regions[0].contains(box))
    }

    func testLowConfidenceLossExceptionAndBodyFailureRedetectSameFrame() throws {
        for mode in 0..<4 {
            let tracker = Tracker(), pipeline = FollowVisualTrackingPipeline(tracker: tracker)
            let first = try snapshot(1, time: 0)
            _ = pipeline.evaluate(first, allowsTracking: true, detect: { self.receipt(first) }, verify: { _, _ in self.receipt(first) })
            if mode == 0 { tracker.result = (box, 0.2) }
            if mode == 1 { tracker.result = nil }
            if mode == 2 { tracker.fail = true }
            let next = try snapshot(2, time: 0.1)
            var fullFrames: [ARFrameID] = []
            let result = pipeline.evaluate(next, allowsTracking: true, detect: {
                fullFrames.append(next.id); return self.receipt(next)
            }, verify: { _, _ in self.receipt(next, verified: false) })
            XCTAssertEqual(result.visualTracking?.mode, .fullDetection)
            XCTAssertEqual(result.visualTracking?.region, FollowVisualTrackingPipeline.fullFrame)
            XCTAssertEqual(fullFrames, [next.id])
        }
    }

    func testUnverifiedOrAmbiguousFullDetectionCannotSeedTracking() throws {
        for count in [1, 2] {
            let tracker = Tracker(), pipeline = FollowVisualTrackingPipeline(tracker: tracker)
            let first = try snapshot(1, time: 0)
            _ = pipeline.evaluate(first, allowsTracking: true, detect: { self.receipt(first, verified: count == 2, count: count) },
                verify: { _, _ in self.receipt(first) })
            XCTAssertEqual(tracker.seeds, 0)
            let next = try snapshot(2, time: 0.1)
            let result = pipeline.evaluate(next, allowsTracking: true, detect: { self.receipt(next) },
                verify: { _, _ in XCTFail("No verified seed"); return self.receipt(next) })
            XCTAssertEqual(result.visualTracking?.mode, .fullDetection)
        }
    }

    func testSourceResetReorderedFramesCameraTurnAndModeChangeForceFullFrame() throws {
        for mode in 0..<6 {
            let tracker = Tracker(), pipeline = FollowVisualTrackingPipeline(tracker: tracker)
            let first = try snapshot(1, time: 1)
            _ = pipeline.evaluate(first, allowsTracking: true, detect: { self.receipt(first) }, verify: { _, _ in self.receipt(first) })
            let next = try snapshot(mode == 1 ? 1 : 2, time: mode == 2 ? 0.9 : 1.1,
                generation: mode == 0 ? 2 : 1, orientation: mode == 3 ? .left : .right,
                yaw: mode == 4 ? 0.3 : 0)
            let result = pipeline.evaluate(next, allowsTracking: mode != 5, detect: { self.receipt(next) },
                verify: { _, _ in XCTFail("Must re-detect after source/mode changes"); return self.receipt(next) })
            XCTAssertEqual(result.visualTracking?.mode, .fullDetection)
            XCTAssertTrue(tracker.regions.isEmpty)
        }
    }

    func testDirectionReversalAndClippingExpandToFullFrame() throws {
        for change in 0..<3 {
            let tracker = Tracker(), pipeline = FollowVisualTrackingPipeline(tracker: tracker)
            let first = try snapshot(1, time: 0)
            _ = pipeline.evaluate(first, allowsTracking: true, detect: { self.receipt(first) }, verify: { _, _ in self.receipt(first) })
            tracker.result = (box.offsetBy(dx: 0.04, dy: 0), 0.95)
            let second = try snapshot(2, time: 0.1)
            let tracked = pipeline.evaluate(second, allowsTracking: true, detect: { self.receipt(second) },
                verify: { box, confidence in self.receipt(second, box: box, confidence: confidence) })
            XCTAssertEqual(tracked.visualTracking?.mode, .tracked)
            let changedBox = change == 0 ? box : (change == 1 ? box.offsetBy(dx: 0.04, dy: 0.04) :
                CGRect(x: 0.9, y: 0.2, width: 0.2, height: 0.5))
            tracker.result = (changedBox, 0.95)
            let third = try snapshot(3, time: 0.2)
            let result = pipeline.evaluate(third, allowsTracking: true, detect: { self.receipt(third) },
                verify: { _, _ in XCTFail("Reversal/clipping requires re-detection"); return self.receipt(third) })
            XCTAssertEqual(result.visualTracking?.mode, .fullDetection)
            XCTAssertEqual(result.visualTracking?.region, FollowVisualTrackingPipeline.fullFrame)
        }
    }

    @MainActor func testMissingCurrentDepthCannotReuseTrackedFramesDepth() throws {
        let tracker = Tracker(), pipeline = FollowVisualTrackingPipeline(tracker: tracker)
        let first = try snapshot(1, time: 0)
        _ = pipeline.evaluate(first, allowsTracking: true, detect: { self.receipt(first) }, verify: { _, _ in self.receipt(first) })
        let next = try snapshot(2, time: 0.1, depth: false)
        let result = pipeline.evaluate(next, allowsTracking: true, detect: { self.receipt(next) }, verify: { _, _ in self.receipt(next) })
        XCTAssertEqual(result.visualTracking?.mode, .fullDetection)
        let batch = ARFollowMePerceptionSource.batch(from: next, detections: result.frame.detections,
            personVerification: result.personVerification)
        XCTAssertFalse(batch.depthAvailable)
        XCTAssertTrue(batch.people.isEmpty)
        XCTAssertEqual(batch.frameID, next.id)
    }

    @MainActor func testDetectorEpochAndSessionChangesRequireNewFullDetection() async throws {
        let tracker = Tracker()
        let detector = Detector(supportedLabels: ["person"], detectionHandler: { _ in
            [.init(label: "person", confidence: 0.99, boundingBox: self.box)]
        }, bodyPoseHandler: { _, _ in
            func joint(_ x: Double, _ y: Double) -> PersonBodyVerifier.Joint {
                .init(location: CGPoint(x: x, y: y), confidence: 0.99)
            }
            return [.init(leftShoulder: joint(0.43, 0.65), rightShoulder: joint(0.57, 0.65),
                leftHip: joint(0.45, 0.4), rightHip: joint(0.55, 0.4))]
        }, objectTrackerFactory: { tracker })
        let session = detector.beginFollowConsumer()
        let first = await detector.evaluateAdaptiveForFollowAsync(try snapshot(1, time: 0), session: session, allowsTracking: true)
        XCTAssertEqual(first?.visualTracking?.mode, .fullDetection)
        let next = await detector.evaluateAdaptiveForFollowAsync(try snapshot(2, time: 0.1), session: session, allowsTracking: true)
        XCTAssertEqual(next?.visualTracking?.mode, .tracked)
        // Disable and re-enable between native evaluations: neither evaluation
        // sees allowsTracking=false, but the epoch still revokes the old seed.
        let afterModeChange = await detector.evaluateAdaptiveForFollowAsync(try snapshot(3, time: 0.2),
            session: session, allowsTracking: true, epoch: 2)
        XCTAssertEqual(afterModeChange?.visualTracking?.mode, .fullDetection)
        let freshTrack = await detector.evaluateAdaptiveForFollowAsync(try snapshot(4, time: 0.3),
            session: session, allowsTracking: true, epoch: 2)
        XCTAssertEqual(freshTrack?.visualTracking?.mode, .tracked)
        detector.endFollowConsumer(session)
        let replacement = detector.beginFollowConsumer()
        let newSession = await detector.evaluateAdaptiveForFollowAsync(try snapshot(5, time: 0.35),
            session: replacement, allowsTracking: true, epoch: 2)
        XCTAssertEqual(newSession?.visualTracking?.mode, .fullDetection)
        detector.endFollowConsumer(replacement)
    }

    func testStaleBodyReceiptCannotBeRelabeledAsCurrentTrackingEvidence() throws {
        let tracker = Tracker(), pipeline = FollowVisualTrackingPipeline(tracker: tracker)
        let first = try snapshot(1, time: 0)
        _ = pipeline.evaluate(first, allowsTracking: true, detect: { self.receipt(first) }, verify: { _, _ in self.receipt(first) })
        let next = try snapshot(2, time: 0.1)
        var redetected = false
        let result = pipeline.evaluate(next, allowsTracking: true, detect: {
            redetected = true; return self.receipt(next, verified: false)
        }, verify: { _, _ in self.receipt(first) })
        XCTAssertTrue(redetected)
        XCTAssertEqual(result.visualTracking?.mode, .fullDetection)
        XCTAssertEqual(result.frame.frameID, next.id)
        XCTAssertEqual(result.personVerification?.first?.accepted, false)
    }
}
