import CoreVideo
import Foundation
import simd
import XCTest
@testable import PhroverKit

@MainActor
final class DepthVisibilityMemoryTests: XCTestCase {
    func testAttachHandsOverOlderFramesNewestFirstAndRetainsTheNewFrame() {
        var memory = DepthVisibilityMemory()
        var first = snapshot(at: 1.0)
        memory.attach(to: &first)
        XCTAssertTrue(first.history.isEmpty)
        var second = snapshot(at: 1.2)
        memory.attach(to: &second)
        var third = snapshot(at: 1.4)
        memory.attach(to: &third)
        XCTAssertEqual(third.history.map(\.timestamp), [1.2, 1.0])
        XCTAssertEqual(memory.frames.map(\.timestamp), [1.0, 1.2, 1.4])
    }

    func testFramesCloserThanTheMinimumSpacingAreNotRetained() {
        var memory = DepthVisibilityMemory()
        for timestamp in [1.0, 1.05, 1.09, 1.10] {
            var next = snapshot(at: timestamp)
            memory.attach(to: &next)
        }
        XCTAssertEqual(memory.frames.map(\.timestamp), [1.0, 1.10])
    }

    func testFramesBeyondTheMaximumAgeOrCountArePruned() {
        var memory = DepthVisibilityMemory()
        let spacing = 0.100_001 // Just above the minimum, clear of binary rounding.
        for index in 0..<30 {
            var next = snapshot(at: Double(index) * spacing)
            memory.attach(to: &next)
        }
        XCTAssertEqual(memory.frames.count, DepthVisibilityMemory.maximumFrames)
        var late = snapshot(at: 29 * spacing + DepthVisibilityMemory.maximumAge - 0.05)
        memory.attach(to: &late)
        XCTAssertTrue(late.history.allSatisfy { late.timestamp - $0.timestamp <= DepthVisibilityMemory.maximumAge })
        XCTAssertEqual(late.history.count, 1)
    }

    func testFailedOrProjectionlessFramesAreHandedHistoryButNotRetained() {
        var memory = DepthVisibilityMemory()
        var good = snapshot(at: 1.0)
        memory.attach(to: &good)
        var failed = snapshot(at: 1.2, failure: .malformedDepth)
        memory.attach(to: &failed)
        XCTAssertEqual(failed.history.map(\.timestamp), [1.0])
        var blind = snapshot(at: 1.4, projection: false)
        memory.attach(to: &blind)
        XCTAssertEqual(memory.frames.map(\.timestamp), [1.0])
    }

    func testOutOfOrderOrNonfiniteCapturesNeverReceiveNewerFrames() {
        var memory = DepthVisibilityMemory()
        var newer = snapshot(at: 2.0)
        memory.attach(to: &newer)
        var older = snapshot(at: 1.5)
        memory.attach(to: &older)
        XCTAssertTrue(older.history.isEmpty)
        var broken = snapshot(at: .nan)
        memory.attach(to: &broken)
        XCTAssertTrue(broken.history.isEmpty)
        XCTAssertTrue(memory.frames.isEmpty)
    }

    func testProcessorHandsHistoryToLaterCapturesOfTheSameTrackedSession() async throws {
        var delivered: [DepthSafetySnapshot] = []
        let processor = DepthSafetyProcessor(evaluate: Self.result(for:)) { delivered.append($0) }
        for timestamp in [1.0, 1.2, 1.4] {
            try await submit(timestamp, to: processor) { delivered.count }
        }
        XCTAssertEqual(delivered.last?.history.map(\.timestamp), [1.2, 1.0])
    }

    func testProcessorClearsMemoryWhenTrackingIsNotNormal() async throws {
        var delivered: [DepthSafetySnapshot] = []
        let processor = DepthSafetyProcessor(evaluate: Self.result(for:)) { delivered.append($0) }
        try await submit(1.0, to: processor) { delivered.count }
        try await submit(1.2, trackingNormal: false, to: processor) { delivered.count }
        try await submit(1.4, to: processor) { delivered.count }
        XCTAssertEqual(delivered.map(\.history.count), [0, 0, 0],
            "Frames from before limited tracking or relocalization are never compared")
    }

    func testProcessorInvalidationClearsMemory() async throws {
        var delivered: [DepthSafetySnapshot] = []
        let processor = DepthSafetyProcessor(evaluate: Self.result(for:)) { delivered.append($0) }
        try await submit(1.0, to: processor) { delivered.count }
        processor.invalidate()
        try await submit(1.2, to: processor) { delivered.count }
        XCTAssertEqual(delivered.last?.history.count, 0)
    }

    // MARK: - Helpers

    private func submit(_ timestamp: TimeInterval, trackingNormal: Bool = true,
                        to processor: DepthSafetyProcessor, count: () -> Int) async throws {
        let before = count()
        var map: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 2, 2, kCVPixelFormatType_DepthFloat32, nil, &map)
        processor.submit(.init(rawDepthMap: try XCTUnwrap(map), intrinsics: matrix_identity_float3x3,
            imageResolution: nil, cameraTransform: matrix_identity_float4x4, timestamp: timestamp,
            trackingNormal: trackingNormal))
        for _ in 0..<2000 where count() == before { await Task.yield() }
        XCTAssertEqual(count(), before + 1)
    }

    private func snapshot(at timestamp: TimeInterval, failure: DepthSafetyUnavailableReason? = nil,
                          projection: Bool = true) -> DepthSafetySnapshot {
        .init(timestamp: timestamp, cells: [], cellResolution: 0.04, validSampleRatio: 1,
              geometry: RoverConfig.collisionGeometry, projection: projection ? Self.projection : nil,
              failureReason: failure)
    }

    nonisolated private static let projection = DepthSafetyProjection(
        width: 2, height: 2, fx: 1, fy: 1, cx: 1, cy: 1, depthValues: [3, 3, 3, 3],
        cameraFromWorld: matrix_identity_float4x4, roverOrigin: .zero,
        roverRight: SIMD3(1, 0, 0), roverForward: SIMD3(0, 0, -1))

    nonisolated private static func result(for input: DepthSafetyProcessor.Input) -> DepthSafetySnapshot {
        .init(timestamp: input.timestamp, cells: [], cellResolution: 0.04, validSampleRatio: 1,
              geometry: RoverConfig.collisionGeometry, projection: projection, failureReason: nil)
    }
}
