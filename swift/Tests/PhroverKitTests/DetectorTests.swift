import XCTest
import ImageIO
import CoreML
import CoreVideo
import UIKit
import RoverNav
import simd
@testable import PhroverKit

final class DetectorTests: XCTestCase {
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
