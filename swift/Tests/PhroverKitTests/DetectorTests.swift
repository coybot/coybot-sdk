import XCTest
import ImageIO
import CoreML
import CoreVideo
import UIKit
import RoverNav
import simd
@testable import PhroverKit

final class DetectorTests: XCTestCase {
    @MainActor
    func testInferenceFailureRuntimeLogExcludesSensitiveErrorDetails() async throws {
        await RuntimeFileLog.flush()
        let logURL = try XCTUnwrap(RuntimeFileLog.logFileURL)
        let previousBytes = (try? Data(contentsOf: logURL).count) ?? 0
        let detector = Detector(supportedLabels: ["person"], detectionHandler: { _ in
            throw NSError(domain: "https://private/path?transcript=secret", code: 7,
                userInfo: [NSLocalizedDescriptionKey: "private response body"])
        })
        let snapshot = ARFrameSnapshot(id: .init(generation: 1, sequence: 1), timestamp: 100,
            image: makeImage(), cameraTransform: matrix_identity_float4x4,
            cameraIntrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 8, height: 6),
            depthMap: nil, pose: .init(position: .zero, yaw: 0), trackingQuality: .normal)
        let follow = detector.evaluateForFollow(snapshot)
        let generic = detector.evaluate(snapshot)
        XCTAssertEqual(follow.status, .failed)
        XCTAssertEqual(follow.failureReason, .inferenceFailed)
        XCTAssertEqual(generic.status, .failed)
        XCTAssertEqual(generic.failureReason, .inferenceFailed)
        await RuntimeFileLog.flush()
        let delta = String(decoding: try Data(contentsOf: logURL).dropFirst(previousBytes), as: UTF8.self)
        XCTAssertTrue(delta.contains("detector_failed"))
        XCTAssertTrue(delta.contains("reason=inference_failed"))
        XCTAssertFalse(delta.contains("https://private/path"))
        XCTAssertFalse(delta.contains("transcript=secret"))
        XCTAssertFalse(delta.contains("private response body"))
    }

    @MainActor
    func testAsyncMissionEvaluationKeepsScreenFallbackAndYieldsToFollow() async throws {
        let localizer = FakeScreenLocalizer(results: [detection("screen", confidence: 0.96)])
        let detector = Detector(primaryDetection: { _ in [self.detection("chair")] },
            screenLocalizer: localizer, now: { 10 })
        let snapshot = ARFrameSnapshot(id: .init(generation: 1, sequence: 1), timestamp: 100,
            image: makeImage(), cameraTransform: matrix_identity_float4x4,
            cameraIntrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 8, height: 6),
            depthMap: nil, pose: .init(position: .zero, yaw: 0), trackingQuality: .normal,
            imageOrientation: .left)
        let evaluation = await detector.evaluateForMissionAsync(snapshot)
        let mission = try XCTUnwrap(evaluation)
        XCTAssertEqual(mission.receipt.frame.detections.map(\.label), ["chair", "screen"])
        XCTAssertEqual(mission.snapshot.id, snapshot.id)
        XCTAssertEqual(mission.receipt.orientation, .left)
        XCTAssertNil(detector.latestFollowEvaluation, "Generic fallback must not enter the follow verification cache")
        let token = detector.beginFollowConsumer()
        let skipped = await detector.evaluateForMissionAsync(snapshot)
        XCTAssertNil(skipped)
        XCTAssertEqual(localizer.callCount, 1)
        detector.endFollowConsumer(token)
        let follow = await detector.evaluateForFollowAsync(snapshot)
        XCTAssertEqual(follow?.frame.detections.map(\.label), ["chair"])
        XCTAssertEqual(localizer.callCount, 1, "Follow inference never borrows screen fallback")
    }

    @MainActor
    func testPreviewAlreadyDispatchedButNotStartedYieldsToNewFollowConsumer() async {
        let queue = DispatchQueue(label: "test.follow.preview-dispatch")
        queue.suspend()
        let calls = FollowInferenceCallCounter()
        let detector = calls.makeDetector(inferenceQueue: queue)
        let snapshot = ARFrameSnapshot(id: .init(generation: 1, sequence: 1), timestamp: 100,
            image: makeImage(), cameraTransform: matrix_identity_float4x4, cameraIntrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 8, height: 6), depthMap: nil, pose: .init(position: .zero, yaw: 0), trackingQuality: .normal)
        let preview = Task { await detector.followPreviewEvaluationAsync(snapshot, at: 100) }
        for _ in 0..<30 { await Task.yield() }
        let ar = ARSessionManager()
        let stream = ARFollowMePerceptionSource(ar: ar, detector: detector).events()
        let consumer = Task { for await _ in stream {} }
        queue.resume()
        let result = await preview.value
        XCTAssertNil(result)
        XCTAssertEqual(calls.value, 0, "Dispatching a preview closure is not starting native inference")
        consumer.cancel(); await consumer.value
    }

    @MainActor
    func testActiveFollowConsumersKeepPreviewCacheMissesOffTheInferenceQueue() async {
        let calls = FollowInferenceCallCounter()
        let detector = calls.makeDetector()
        let ar = ARSessionManager()
        let source = ARFollowMePerceptionSource(ar: ar, detector: detector)
        let firstStream = source.events()
        let secondStream = source.events()
        let first = Task { for await _ in firstStream {} }
        let second = Task { for await _ in secondStream {} }
        for _ in 0..<20 { await Task.yield() }
        let snapshot = ARFrameSnapshot(id: .init(generation: 1, sequence: 1), timestamp: 100,
            image: makeImage(), cameraTransform: matrix_identity_float4x4, cameraIntrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 8, height: 6), depthMap: nil, pose: .init(position: .zero, yaw: 0), trackingQuality: .normal)
        let unavailable = await detector.followPreviewEvaluationAsync(snapshot, at: 100)
        XCTAssertNil(unavailable, "A preview cache miss cannot compete with an active follow consumer")
        XCTAssertEqual(calls.value, 0)
        let control = await detector.evaluateForFollowAsync(snapshot)
        XCTAssertEqual(control?.frame.frameID, snapshot.id)
        let cached = await detector.followPreviewEvaluationAsync(snapshot, at: 100.1)
        XCTAssertEqual(cached?.snapshot.id, snapshot.id)
        XCTAssertEqual(calls.value, 1, "Active control still infers; preview only reuses its exact fresh result")
        first.cancel(); await first.value
        let stillUnavailable = await detector.followPreviewEvaluationAsync(snapshot, at: 101)
        XCTAssertNil(stillUnavailable)
        XCTAssertEqual(calls.value, 1, "Ending one consumer must not release another's priority")
        second.cancel(); await second.value
        let preview = await detector.followPreviewEvaluationAsync(snapshot, at: 102)
        XCTAssertNotNil(preview)
        XCTAssertEqual(calls.value, 2, "Preview inference resumes after the last follow stream terminates")
    }

    @MainActor
    func testGenericImagePointProjectionDoesNotDependOnLocalInferenceCache() throws {
        let ar = ARSessionManager()
        var depth: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 8, 6, kCVPixelFormatType_DepthFloat32, nil, &depth)
        let map = try XCTUnwrap(depth)
        CVPixelBufferLockBaseAddress(map, [])
        let values = CVPixelBufferGetBaseAddress(map)!.assumingMemoryBound(to: Float.self)
        for row in 0..<6 { for col in 0..<8 { values[row * CVPixelBufferGetBytesPerRow(map) / 4 + col] = 2 } }
        CVPixelBufferUnlockBaseAddress(map, [])
        ar.ingestForTesting(image: makeImage(), timestamp: ProcessInfo.processInfo.systemUptime,
            cameraTransform: matrix_identity_float4x4, intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 8, height: 6), depthMap: map, trackingQuality: .normal)
        let perception = ARPerceptionSource(ar: ar, detector: nil)
        XCTAssertTrue(perception.detectObjects().isEmpty)
        XCTAssertNotNil(perception.unproject(normalizedPoint: CGPoint(x: 0.5, y: 0.5)),
            "Generic normalized image-point projection must not require a local object detector receipt")
    }
    @MainActor
    func testSlowInferenceDoesNotBlockControlAndOnlyNewestQueuedFrameSurvives() async {
        let entered = expectation(description: "inference held off actor")
        let gate = DispatchSemaphore(value: 0)
        let detector = Detector(supportedLabels: ["person"], detectionHandler: { _ in
            entered.fulfill()
            _ = gate.wait(timeout: .now() + 3)
            return []
        })
        entered.expectedFulfillmentCount = 1
        entered.assertForOverFulfill = false
        func snapshot(_ sequence: UInt64) -> ARFrameSnapshot {
            .init(id: .init(generation: 1, sequence: sequence), timestamp: Double(sequence),
                image: makeImage(), cameraTransform: matrix_identity_float4x4, cameraIntrinsics: matrix_identity_float3x3,
                imageResolution: CGSize(width: 8, height: 6), depthMap: nil,
                pose: .init(position: .zero, yaw: 0), trackingQuality: .normal)
        }
        let first = Task { await detector.evaluateForFollowAsync(snapshot(1)) }
        await fulfillment(of: [entered], timeout: 1)
        // These actor operations must remain runnable while native inference is held.
        XCTAssertNil(detector.latestFollowEvaluation)
        let second = Task { await detector.evaluateForFollowAsync(snapshot(2)) }
        for _ in 0..<20 { await Task.yield() }
        let third = Task { await detector.evaluateForFollowAsync(snapshot(3)) }
        for _ in 0..<20 { await Task.yield() }
        first.cancel()
        gate.signal(); gate.signal()
        let a = await first.value
        let b = await second.value
        let c = await third.value
        XCTAssertNil(a, "Cancelled inference cannot be handed to the control loop")
        XCTAssertNil(b, "Superseded queued work is dropped rather than building a frame backlog")
        XCTAssertEqual(c?.frame.frameID.sequence, 3)
    }

    @MainActor
    func testMissionPerceptionDoesNotWaitForBusyPreviewInferenceLock() async {
        let entered = expectation(description: "preview busy")
        let gate = DispatchSemaphore(value: 0)
        let detector = Detector(supportedLabels: ["chair"], detectionHandler: { _ in
            entered.fulfill()
            _ = gate.wait(timeout: .now() + 0.5)
            return []
        })
        entered.assertForOverFulfill = false
        let ar = ARSessionManager()
        ar.ingestForTesting(image: makeImage(), timestamp: ProcessInfo.processInfo.systemUptime,
            cameraTransform: matrix_identity_float4x4, intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 8, height: 6), depthMap: nil, trackingQuality: .normal)
        let preview = Task { await detector.evaluateForFollowAsync(ar.latestSnapshot!) }
        await fulfillment(of: [entered], timeout: 1)
        let perception = ARPerceptionSource(ar: ar, detector: detector)
        let started = ProcessInfo.processInfo.systemUptime
        XCTAssertTrue(perception.detectObjects().isEmpty)
        XCTAssertLessThan(ProcessInfo.processInfo.systemUptime - started, 0.1,
            "Synchronous mission reads must use cached facts, never join inference")
        gate.signal()
        _ = await preview.value
    }
    @MainActor
    func testFollowInferenceRunsAwayFromMainThreadAndKeepsSnapshotIdentity() async throws {
        let detector = Detector(supportedLabels: ["person"], detectionHandler: { _ in
            XCTAssertFalse(Thread.isMainThread, "Inference must not occupy the control/UI actor")
            return []
        })
        let snapshot = ARFrameSnapshot(id: .init(generation: 1, sequence: 7), timestamp: 100,
            image: makeImage(), cameraTransform: matrix_identity_float4x4, cameraIntrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 8, height: 6), depthMap: nil,
            pose: .init(position: .zero, yaw: 0), trackingQuality: .normal)
        let result = await detector.evaluateForFollowAsync(snapshot)
        XCTAssertEqual(result?.frame.frameID, snapshot.id)
        XCTAssertEqual(result?.frame.monotonicTimestamp, 100)
    }
    func testNativeBodyRequestDoesNotVerifyUniformEmptyImage() throws {
        var buffer: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 640, 480, kCVPixelFormatType_32BGRA, nil, &buffer)
        let image = try XCTUnwrap(buffer)
        CVPixelBufferLockBaseAddress(image, [])
        memset(CVPixelBufferGetBaseAddress(image), 0, CVPixelBufferGetDataSize(image))
        CVPixelBufferUnlockBaseAddress(image, [])
        let detector = Detector(supportedLabels: ["person"], detectionHandler: { _ in
            [.init(label: "person", confidence: 1, boundingBox: CGRect(x: 0.1, y: 0.1, width: 0.8, height: 0.8))]
        }, bodyPoseHandler: Detector.humanBodies)
        let snapshot = ARFrameSnapshot(id: .init(generation: 1, sequence: 1), timestamp: 100, image: image,
            cameraTransform: matrix_identity_float4x4, cameraIntrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 640, height: 480), depthMap: nil,
            pose: .init(position: .zero, yaw: 0), trackingQuality: .normal)
        let receipt = detector.evaluateForFollow(snapshot)
        XCTAssertEqual(receipt.frame.detections.first?.confidence, 1, "Raw prediction remains visible for diagnosis")
        let decision = try XCTUnwrap(receipt.personVerification?.first)
        XCTAssertFalse(decision.accepted)
        XCTAssertTrue(["body_verification_failed", "no_matching_body"].contains(decision.reason),
            "Native unavailability or no body must both fail closed, never trust the raw score")
    }
    func testPreviewReusesExactFreshFollowImageAndReceiptWithoutFallbackOrDuplicateInference() {
        var orientations: [CGImagePropertyOrientation] = []
        let detector = Detector(supportedLabels: ["person"], visionHandler: { _, orientation in
            orientations.append(orientation)
            return [.init(label: "person", confidence: 1, boundingBox: CGRect(x: 0.2, y: 0.2, width: 0.6, height: 0.6))]
        })
        func snapshot(_ sequence: UInt64, _ time: Double, generation: UInt64 = 1) -> ARFrameSnapshot {
            .init(id: .init(generation: generation, sequence: sequence), timestamp: time, image: makeImage(),
                cameraTransform: matrix_identity_float4x4, cameraIntrinsics: matrix_identity_float3x3,
                imageResolution: CGSize(width: 8, height: 6), depthMap: nil, pose: .init(position: .zero, yaw: 0), trackingQuality: .normal)
        }
        let original = snapshot(1, 100)
        _ = detector.evaluateForFollow(original)
        let newer = snapshot(2, 100.1)
        let preview = detector.followPreviewEvaluation(newer, at: 100.2)
        XCTAssertEqual(orientations, [.right])
        XCTAssertEqual(preview.snapshot.id, original.id)
        XCTAssertEqual(preview.receipt.frame.frameID, original.id)
        XCTAssertTrue(preview.snapshot.image === original.image)
        XCTAssertFalse(preview.receipt.personVerification?.first?.accepted ?? true)
        let refreshed = detector.followPreviewEvaluation(newer, at: 100.6)
        XCTAssertEqual(orientations, [.right, .right])
        XCTAssertEqual(refreshed.snapshot.id, newer.id)
        let reset = snapshot(1, 100.61, generation: 2)
        let generationChanged = detector.followPreviewEvaluation(reset, at: 100.62)
        XCTAssertEqual(generationChanged.snapshot.id, reset.id)
        XCTAssertEqual(orientations, [.right, .right, .right])
    }
    func testFollowReceiptCapturesActualRightOrientationWithoutFallbackOnEmptyOrError() {
        let snapshot = ARFrameSnapshot(id: .init(generation: 1, sequence: 1), timestamp: 100,
            image: makeImage(), cameraTransform: matrix_identity_float4x4, cameraIntrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 8, height: 6), depthMap: nil,
            pose: .init(position: .zero, yaw: 0), trackingQuality: .normal)
        for scenario in 0..<3 {
            var attempts: [CGImagePropertyOrientation] = []
            let detector = Detector(supportedLabels: ["person"], visionHandler: { _, orientation in
                attempts.append(orientation)
                if scenario == 2 && orientation == .right { throw URLError(.cannotDecodeContentData) }
                if scenario == 0 && orientation == .right { return [] }
                return [.init(label: "person", confidence: 0.99,
                    boundingBox: CGRect(x: 0.2, y: 0.3, width: 0.2, height: 0.3))]
            })
            let receipt = detector.evaluateForFollow(snapshot)
            XCTAssertEqual(attempts, [.right])
            XCTAssertEqual(receipt.status, scenario == 2 ? .failed : .executed)
            XCTAssertEqual(receipt.orientation, scenario == 2 ? nil : .right)
            XCTAssertEqual(receipt.frame.detections.count, scenario == 1 ? 1 : 0)
            XCTAssertEqual(receipt.failureReason, scenario == 2 ? .inferenceFailed : nil)
            attempts.removeAll()
            let generic = detector.evaluate(snapshot)
            XCTAssertEqual(attempts, [.right], "Coybot never mixes boxes from a different rotation")
            XCTAssertEqual(generic.status, scenario == 2 ? .failed : .executed)
            XCTAssertEqual(generic.orientation, scenario == 2 ? nil : .right)
            XCTAssertEqual(generic.frame.detections.count, scenario == 1 ? 1 : 0)
        }
    }

    func testFollowAndBodyVerificationUseTheSnapshotOrientation() {
        for orientation in [CGImagePropertyOrientation.up, .right, .down, .left] {
            var detectorOrientations: [CGImagePropertyOrientation] = []
            var bodyOrientations: [CGImagePropertyOrientation] = []
            let detector = Detector(supportedLabels: ["person"], visionHandler: { _, evaluated in
                detectorOrientations.append(evaluated)
                return [.init(label: "person", confidence: 0.9,
                    boundingBox: CGRect(x: 0.2, y: 0.2, width: 0.5, height: 0.6))]
            }, bodyPoseHandler: { _, evaluated in bodyOrientations.append(evaluated); return [] })
            let snapshot = ARFrameSnapshot(id: .init(generation: 1, sequence: 1), timestamp: 100,
                image: makeImage(), cameraTransform: matrix_identity_float4x4,
                cameraIntrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 8, height: 6),
                depthMap: nil, pose: .init(position: .zero, yaw: 0), trackingQuality: .normal,
                imageOrientation: orientation)
            let receipt = detector.evaluateForFollow(snapshot)
            XCTAssertEqual(detectorOrientations, [orientation])
            XCTAssertEqual(bodyOrientations, [orientation])
            XCTAssertEqual(receipt.orientation, orientation)
        }
    }
    func testModelResourcePrefersCompiledModelBundle() {
        let url = Detector.modelResourceURL(modelName: "RoverYOLO")

        XCTAssertEqual(url?.pathExtension, "mlmodelc")
    }

    func testModelConfigurationAvoidsGPUForBackgroundSafety() {
        XCTAssertEqual(Detector.modelConfiguration().computeUnits, .cpuAndNeuralEngine)
    }

    func testScreenFallbackFindsSuppliedMonitorAtNavigationConfidence() async throws {
        // SwiftPM puts `.copy` resources in the target's resource bundle (`Bundle.module`);
        // the example project's PhroverSDKTests target copies them into the test bundle.
        #if SWIFT_PACKAGE
        let testBundle = Bundle.module
        #else
        let testBundle = Bundle(for: DetectorTests.self)
        #endif
        let fixtureURL = try XCTUnwrap(
            testBundle.url(
                forResource: "supplied-monitor",
                withExtension: "png",
                subdirectory: "ScreenDetector"
            )
                ?? testBundle.url(forResource: "supplied-monitor", withExtension: "png")
        )
        let image = try XCTUnwrap(UIImage(contentsOfFile: fixtureURL.path)?.cgImage)
        let localizer = await VisionScreenLocalizer()

        // The fixture is an upright photo, so it needs no rotation.
        let detections = localizer.detect(try pixelBuffer(image: image), orientation: .up)

        XCTAssertTrue(localizer.isLoaded)
        XCTAssertTrue(detections.contains {
            ScreenDetectionPolicy.isScreenLike($0.label) && $0.confidence >= 0.90
        })
    }

    func testFallbackAppendsScreenWhenPrimaryHasNoScreenLikeObject() throws {
        let localizer = FakeScreenLocalizer(results: [detection("screen", confidence: 0.96)])
        let detector = Detector(
            primaryDetection: { _ in [self.detection("chair", confidence: 0.98)] },
            screenLocalizer: localizer,
            now: { 10 }
        )

        let results = detector.detect(try pixelBuffer())

        XCTAssertEqual(results.map(\.label), ["chair", "screen"])
        XCTAssertEqual(localizer.callCount, 1)
    }

    func testFallbackStillFindsScreensWhenPrimaryInferenceFails() throws {
        let localizer = FakeScreenLocalizer(results: [detection("screen", confidence: 0.96)])
        let detector = Detector(primaryDetection: { _ in throw URLError(.cannotDecodeContentData) },
            screenLocalizer: localizer, now: { 10 })
        XCTAssertEqual(detector.detect(try pixelBuffer()).map(\.label), ["screen"])
        XCTAssertEqual(localizer.callCount, 1)
    }

    func testFallbackIsSkippedWhenPrimaryAlreadyHasScreenLikeObject() throws {
        let localizer = FakeScreenLocalizer(results: [detection("screen")])
        let detector = Detector(
            primaryDetection: { _ in [self.detection("tv", confidence: 0.96)] },
            screenLocalizer: localizer,
            now: { 10 }
        )

        let results = detector.detect(try pixelBuffer())

        XCTAssertEqual(results.map(\.label), ["tv"])
        XCTAssertEqual(localizer.callCount, 0)
    }

    func testFallbackRunsAtMostTwicePerSecond() throws {
        let localizer = FakeScreenLocalizer(results: [detection("screen")])
        var times = [10.0, 10.2, 10.5]
        let detector = Detector(
            primaryDetection: { _ in [] },
            screenLocalizer: localizer,
            now: { times.removeFirst() }
        )
        let buffer = try pixelBuffer()

        _ = detector.detect(buffer)
        _ = detector.detect(buffer)
        _ = detector.detect(buffer)

        XCTAssertEqual(localizer.callCount, 2)
    }

    func testDisablingInferencePreventsFallbackAndPreservesPrimary() throws {
        let localizer = FakeScreenLocalizer(results: [detection("screen")])
        let detector = Detector(
            primaryDetection: { _ in [self.detection("chair")] },
            screenLocalizer: localizer,
            now: { 10 }
        )
        detector.setInferenceEnabled(false)

        let results = detector.detect(try pixelBuffer())

        XCTAssertEqual(results.map(\.label), ["chair"])
        XCTAssertEqual(localizer.callCount, 0)
    }

    func testCanonicalLabelsExposeOnlyStringModelClassLabels() {
        XCTAssertEqual(Detector.canonicalLabels(from: ["chair", 7, "person", "chair"]), ["chair", "person"] as Set)
    }

    func testFrameDetectionSeamPreservesSnapshotIdentity() {
        let detector = Detector(supportedLabels: ["chair"]) { _ in
            [Detector.Detection(label: "chair", confidence: 0.95,
                                boundingBox: CGRect(x: 0.25, y: 0.5, width: 0.2, height: 0.3))]
        }
        let snapshot = ARFrameSnapshot(
            id: ARFrameID(generation: 4, sequence: 9), timestamp: 12.5, image: makeImage(),
            cameraTransform: matrix_identity_float4x4, cameraIntrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 8, height: 6), depthMap: nil,
            pose: RoverNav.Pose2D(position: RoverNav.Vec2(0, 0), yaw: 0), trackingQuality: .normal
        )

        let result = detector.detect(snapshot)

        XCTAssertEqual(detector.supportedCanonicalLabels, ["chair"])
        XCTAssertEqual(result.frameID, snapshot.id)
        XCTAssertEqual(result.monotonicTimestamp, 12.5)
        XCTAssertEqual(result.detections.map(\.label), ["chair"])
    }

    private func makeImage() -> CVPixelBuffer {
        var buffer: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 8, 6, kCVPixelFormatType_32BGRA, nil, &buffer)
        return buffer!
    }

    private func pixelBuffer() throws -> CVPixelBuffer {
        var buffer: CVPixelBuffer?
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            2,
            2,
            kCVPixelFormatType_32BGRA,
            nil,
            &buffer
        )
        XCTAssertEqual(status, kCVReturnSuccess)
        return try XCTUnwrap(buffer)
    }

    private func pixelBuffer(image: CGImage) throws -> CVPixelBuffer {
        var buffer: CVPixelBuffer?
        let attributes: [CFString: Any] = [
            kCVPixelBufferCGImageCompatibilityKey: true,
            kCVPixelBufferCGBitmapContextCompatibilityKey: true,
        ]
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            image.width,
            image.height,
            kCVPixelFormatType_32BGRA,
            attributes as CFDictionary,
            &buffer
        )
        XCTAssertEqual(status, kCVReturnSuccess)
        let pixelBuffer = try XCTUnwrap(buffer)
        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, []) }
        let context = CGContext(
            data: CVPixelBufferGetBaseAddress(pixelBuffer),
            width: image.width,
            height: image.height,
            bitsPerComponent: 8,
            bytesPerRow: CVPixelBufferGetBytesPerRow(pixelBuffer),
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGBitmapInfo.byteOrder32Little.rawValue
                | CGImageAlphaInfo.premultipliedFirst.rawValue
        )
        XCTAssertNotNil(context)
        context?.draw(image, in: CGRect(x: 0, y: 0, width: image.width, height: image.height))
        return pixelBuffer
    }

    private func detection(_ label: String,
                           confidence: Float = 0.95) -> Detector.Detection {
        Detector.Detection(
            label: label,
            confidence: confidence,
            boundingBox: CGRect(x: 0.2, y: 0.2, width: 0.5, height: 0.5)
        )
    }
}

private final class FakeScreenLocalizer: ScreenLocalizing {
    let isLoaded = true
    private let results: [Detector.Detection]
    private(set) var callCount = 0

    init(results: [Detector.Detection]) {
        self.results = results
    }

    func detect(_ pixelBuffer: CVPixelBuffer, orientation: CGImagePropertyOrientation) -> [Detector.Detection] {
        callCount += 1
        return results
    }
}
