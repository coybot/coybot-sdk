import CoreVideo
import Foundation
import simd
import XCTest
@testable import PhroverKit

@MainActor
final class DepthSafetyProcessorTests: XCTestCase {
    func testDepthDeliveryContinuesWhileDetectorWorkerIsBlocked() async throws {
        let inferenceStarted = expectation(description: "inference is busy")
        let depthDelivered = expectation(description: "depth bypasses inference worker")
        let gate = DispatchSemaphore(value: 0)
        defer { gate.signal() }
        let detector = Detector(supportedLabels: ["person"], detectionHandler: { _ in
            inferenceStarted.fulfill()
            _ = gate.wait(timeout: .now() + 5)
            return []
        })
        let input = try input(at: 37)
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        let frame = ARFrameSnapshot(id: .init(generation: 1, sequence: 1), timestamp: 37,
            image: try XCTUnwrap(image), cameraTransform: matrix_identity_float4x4,
            cameraIntrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 20, height: 20),
            depthMap: input.rawDepthMap, pose: .init(position: .zero, yaw: 0), trackingQuality: .normal)
        let inference = Task { await detector.evaluateForFollowAsync(frame) }
        await fulfillment(of: [inferenceStarted], timeout: 2)
        var capturedTimestamp: Double?
        let processor = DepthSafetyProcessor(evaluate: Self.result) { depth in
            capturedTimestamp = depth.timestamp
            depthDelivered.fulfill()
        }
        processor.submit(input)
        await fulfillment(of: [depthDelivered], timeout: 2)
        XCTAssertEqual(capturedTimestamp, 37, "Publication must preserve capture age, not stamp depth as new")
        gate.signal()
        _ = await inference.value
    }

    func testDefaultProjectorPreservesCapturedDepthAndTimestamp() async throws {
        let completed = expectation(description: "real projection delivered")
        let input = try input(at: 12)
        CVPixelBufferLockBaseAddress(input.rawDepthMap, [])
        let stride = CVPixelBufferGetBytesPerRow(input.rawDepthMap) / MemoryLayout<Float>.size
        let values = CVPixelBufferGetBaseAddress(input.rawDepthMap)!.assumingMemoryBound(to: Float.self)
        for row in 0..<2 { for column in 0..<2 { values[row * stride + column] = 2 } }
        CVPixelBufferUnlockBaseAddress(input.rawDepthMap, [])
        var result: DepthSafetySnapshot?
        let processor = DepthSafetyProcessor { result = $0; completed.fulfill() }
        processor.submit(input)
        await fulfillment(of: [completed], timeout: 2)
        let snapshot = try XCTUnwrap(result)
        XCTAssertEqual(snapshot.timestamp, 12)
        XCTAssertNil(snapshot.failureReason)
        XCTAssertEqual(snapshot.projection?.depthValues, [2, 2, 2, 2])
    }

    func testProjectionDoesNotBlockCameraOrControlActor() async throws {
        let entered = expectation(description: "projection running")
        let completed = expectation(description: "projection delivered")
        let gate = DispatchSemaphore(value: 0)
        defer { gate.signal() }
        var delivered = false
        let processor = DepthSafetyProcessor(evaluate: { input in
            XCTAssertFalse(Thread.isMainThread, "Projection must leave the camera/control thread")
            entered.fulfill()
            _ = gate.wait(timeout: .now() + 2)
            return Self.result(for: input)
        }, didProcess: { _ in delivered = true; completed.fulfill() })

        processor.submit(try input(at: 1))
        await fulfillment(of: [entered], timeout: 1)
        XCTAssertFalse(delivered, "Camera/control work must continue while projection is blocked")
        gate.signal()
        await fulfillment(of: [completed], timeout: 2)
    }

    func testBusyProjectorKeepsOnlyNewestWaitingCapture() async throws {
        let entered = expectation(description: "first capture running")
        let completed = expectation(description: "newest capture delivered")
        let gate = DispatchSemaphore(value: 0)
        defer { gate.signal() }
        var delivered: [TimeInterval] = []
        let processor = DepthSafetyProcessor(evaluate: { input in
            if input.timestamp == 1 {
                entered.fulfill()
                _ = gate.wait(timeout: .now() + 2)
            }
            return Self.result(for: input)
        }, didProcess: { snapshot in
            delivered.append(snapshot.timestamp)
            if snapshot.timestamp == 3 { completed.fulfill() }
        })

        processor.submit(try input(at: 1))
        await fulfillment(of: [entered], timeout: 1)
        processor.submit(try input(at: 2))
        processor.submit(try input(at: 3))
        gate.signal()
        await fulfillment(of: [completed], timeout: 2)
        XCTAssertEqual(delivered, [1, 3])
    }

    func testInvalidationDiscardsRunningAndWaitingCaptures() async throws {
        let entered = expectation(description: "old capture running")
        let completed = expectation(description: "new session capture delivered")
        let gate = DispatchSemaphore(value: 0)
        defer { gate.signal() }
        var delivered: [TimeInterval] = []
        let processor = DepthSafetyProcessor(evaluate: { input in
            if input.timestamp == 1 {
                entered.fulfill()
                _ = gate.wait(timeout: .now() + 2)
            }
            return Self.result(for: input)
        }, didProcess: { snapshot in
            delivered.append(snapshot.timestamp)
            if snapshot.timestamp == 3 { completed.fulfill() }
        })

        processor.submit(try input(at: 1))
        await fulfillment(of: [entered], timeout: 1)
        processor.submit(try input(at: 2))
        processor.invalidate()
        processor.submit(try input(at: 3))
        gate.signal()
        await fulfillment(of: [completed], timeout: 2)
        XCTAssertEqual(delivered, [3])
    }

    private func input(at timestamp: TimeInterval) throws -> DepthSafetyProcessor.Input {
        var map: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 2, 2, kCVPixelFormatType_DepthFloat32, nil, &map)
        return .init(rawDepthMap: try XCTUnwrap(map), intrinsics: matrix_identity_float3x3,
            imageResolution: nil, cameraTransform: matrix_identity_float4x4, timestamp: timestamp)
    }

    nonisolated private static func result(for input: DepthSafetyProcessor.Input) -> DepthSafetySnapshot {
        .init(timestamp: input.timestamp, cells: [], cellResolution: 0.04,
            validSampleRatio: 1, geometry: RoverConfig.collisionGeometry,
            projection: nil, failureReason: nil)
    }
}
