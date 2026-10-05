import CoreVideo
import XCTest
@testable import PhroverKit

final class LocalObjectColorAnalyzerTests: XCTestCase {
    func testClassifiesBlackRegion() {
        let samples = Array(
            repeating: RGBSample(red: 0.04, green: 0.05, blue: 0.04),
            count: 64
        )

        XCTAssertGreaterThanOrEqual(
            LocalObjectColorAnalyzer.evidence(from: samples)[.black] ?? 0,
            0.70
        )
    }

    func testDoesNotCallDarkBlueBlack() {
        let samples = Array(
            repeating: RGBSample(red: 0.02, green: 0.04, blue: 0.16),
            count: 64
        )
        let evidence = LocalObjectColorAnalyzer.evidence(from: samples)

        XCTAssertGreaterThan(evidence[.blue] ?? 0, evidence[.black] ?? 0)
    }

    func testClassifiesWhiteGrayAndBrownRegions() {
        let fixtures: [(RGBSample, LocalObjectColor)] = [
            (RGBSample(red: 0.95, green: 0.95, blue: 0.95), .white),
            (RGBSample(red: 0.45, green: 0.46, blue: 0.44), .gray),
            (RGBSample(red: 0.35, green: 0.18, blue: 0.07), .brown),
        ]

        for (sample, expectedColor) in fixtures {
            let evidence = LocalObjectColorAnalyzer.evidence(
                from: Array(repeating: sample, count: 64)
            )
            XCTAssertEqual(evidence.first?.color, expectedColor)
        }
    }

    func testAnalyzerUsesInsetRegionInsteadOfBoundingBoxEdge() throws {
        let buffer = try makeBGRABuffer(width: 16, height: 16) { x, y in
            let isBorder = x < 3 || x >= 13 || y < 3 || y >= 13
            return isBorder
                ? RGBSample(red: 1, green: 1, blue: 1)
                : RGBSample(red: 0.03, green: 0.03, blue: 0.03)
        }

        let evidence = LocalObjectColorAnalyzer().analyze(
            pixelBuffer: buffer,
            normalizedBoundingBox: CGRect(x: 0, y: 0, width: 1, height: 1)
        )

        XCTAssertGreaterThanOrEqual(evidence[.black] ?? 0, 0.70)
    }

    func testAnalyzerSupportsFullAndVideoRangeBiPlanarBuffers() throws {
        let fixtures: [(OSType, UInt8)] = [
            (kCVPixelFormatType_420YpCbCr8BiPlanarFullRange, 0),
            (kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange, 16),
        ]

        for (pixelFormat, blackLuma) in fixtures {
            let buffer = try makeBiPlanarBuffer(pixelFormat: pixelFormat, luma: blackLuma)
            let evidence = LocalObjectColorAnalyzer().analyze(
                pixelBuffer: buffer,
                normalizedBoundingBox: CGRect(x: 0, y: 0, width: 1, height: 1)
            )

            XCTAssertGreaterThanOrEqual(evidence[.black] ?? 0, 0.70)
        }
    }

    func testPerceivedObjectMaintainsClampedSortedEvidenceAfterMutation() {
        var object = PerceivedObject(
            label: "chair",
            confidence: 0.5,
            normalizedPoint: .zero
        )

        object.confidence = 1.5
        object.colorEvidence = [
            ObjectColorEvidence(color: .white, confidence: 0.3),
            ObjectColorEvidence(color: .blue, confidence: 0.7),
            ObjectColorEvidence(color: .black, confidence: 0.7),
        ]

        XCTAssertEqual(object.confidence, 1)
        XCTAssertEqual(object.colorEvidence.map(\.color), [.black, .blue, .white])
    }

    private func makeBGRABuffer(
        width: Int,
        height: Int,
        colorAt: (Int, Int) -> RGBSample
    ) throws -> CVPixelBuffer {
        var pixelBuffer: CVPixelBuffer?
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            width,
            height,
            kCVPixelFormatType_32BGRA,
            nil,
            &pixelBuffer
        )
        XCTAssertEqual(status, kCVReturnSuccess)
        let buffer = try XCTUnwrap(pixelBuffer)

        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        let bytesPerRow = CVPixelBufferGetBytesPerRow(buffer)
        let base = try XCTUnwrap(CVPixelBufferGetBaseAddress(buffer))
            .assumingMemoryBound(to: UInt8.self)
        for y in 0..<height {
            for x in 0..<width {
                let sample = colorAt(x, y)
                let offset = y * bytesPerRow + x * 4
                base[offset] = byte(sample.blue)
                base[offset + 1] = byte(sample.green)
                base[offset + 2] = byte(sample.red)
                base[offset + 3] = 255
            }
        }
        return buffer
    }

    private func makeBiPlanarBuffer(pixelFormat: OSType, luma: UInt8) throws -> CVPixelBuffer {
        var pixelBuffer: CVPixelBuffer?
        let attributes = [kCVPixelBufferIOSurfacePropertiesKey: [:]] as CFDictionary
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            8,
            8,
            pixelFormat,
            attributes,
            &pixelBuffer
        )
        XCTAssertEqual(status, kCVReturnSuccess)
        let buffer = try XCTUnwrap(pixelBuffer)

        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        XCTAssertEqual(CVPixelBufferGetPlaneCount(buffer), 2)

        let yBase = try XCTUnwrap(CVPixelBufferGetBaseAddressOfPlane(buffer, 0))
            .assumingMemoryBound(to: UInt8.self)
        let yStride = CVPixelBufferGetBytesPerRowOfPlane(buffer, 0)
        for y in 0..<CVPixelBufferGetHeightOfPlane(buffer, 0) {
            for x in 0..<CVPixelBufferGetWidthOfPlane(buffer, 0) {
                yBase[y * yStride + x] = luma
            }
        }

        let chromaBase = try XCTUnwrap(CVPixelBufferGetBaseAddressOfPlane(buffer, 1))
            .assumingMemoryBound(to: UInt8.self)
        let chromaStride = CVPixelBufferGetBytesPerRowOfPlane(buffer, 1)
        for y in 0..<CVPixelBufferGetHeightOfPlane(buffer, 1) {
            for x in 0..<CVPixelBufferGetWidthOfPlane(buffer, 1) {
                let offset = y * chromaStride + x * 2
                chromaBase[offset] = 128
                chromaBase[offset + 1] = 128
            }
        }
        return buffer
    }

    private func byte(_ component: Float) -> UInt8 {
        UInt8((min(max(component, 0), 1) * 255).rounded())
    }
}
