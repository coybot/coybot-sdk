import CoreVideo
import RoverNav
import simd
import XCTest
@testable import PhroverKit

@MainActor
final class AROpticalExchangeServiceTests: XCTestCase {
    func testScanningSnapshotPreservesFullARFrameIdentity() throws {
        let snapshot = makeSnapshot(generation: 8, sequence: 14, timestamp: 3.25)
        var scannedFrameID: UInt64?
        var scannedTimestamp: TimeInterval?
        let service = AROpticalExchangeService(sessionManager: ARSessionManager(),
            clock: RuntimeSilentSearchClock(), scanner: { frame in
                scannedFrameID = frame.frameID
                scannedTimestamp = frame.monotonicTimestamp
                return [OpticalObservation(payload: Data("payload".utf8), frameID: frame.frameID,
                    monotonicTimestamp: frame.monotonicTimestamp, corners: Self.corners)]
            }, presenter: { _ in })

        let result = try XCTUnwrap(service.observations(in: snapshot).first)

        XCTAssertEqual(scannedFrameID, 14)
        XCTAssertEqual(scannedTimestamp, 3.25)
        XCTAssertEqual(result.frameID, ARFrameID(generation: 8, sequence: 14))
        XCTAssertEqual(result.observation.payload, Data("payload".utf8))
    }

    func testDiscardsScannerOutputClaimingDifferentFrame() throws {
        let service = AROpticalExchangeService(sessionManager: ARSessionManager(),
            clock: RuntimeSilentSearchClock(), scanner: { _ in
                [OpticalObservation(payload: Data("payload".utf8), frameID: 99,
                    monotonicTimestamp: 3.25, corners: Self.corners)]
            }, presenter: { _ in })

        XCTAssertEqual(try service.observations(in: makeSnapshot(generation: 8, sequence: 14,
                                                                  timestamp: 3.25)), [])
    }

    func testScannerDoesNotCoalesceSameSequenceAcrossSessionGenerations() throws {
        let image = try OpticalQRCodeRenderer().render(payload: Data("generation-aware".utf8), moduleScale: 8)
        let scanner = OpticalQRCodeScanner()

        let first = try scanner.scan(OpticalFrame(image: image,
            arFrameID: ARFrameID(generation: 1, sequence: 1), monotonicTimestamp: 1))
        let second = try scanner.scan(OpticalFrame(image: image,
            arFrameID: ARFrameID(generation: 2, sequence: 1), monotonicTimestamp: 2))

        XCTAssertFalse(first.isEmpty)
        XCTAssertFalse(second.isEmpty)
        XCTAssertEqual(second.first?.frameID, 1)
    }

    func testCancelCancelsInFlightPresentation() async {
        var started = false
        var cancelled = false
        let service = AROpticalExchangeService(sessionManager: ARSessionManager(),
            clock: RuntimeSilentSearchClock(), scanner: { _ in [] }, presenter: { payload in
                guard payload != nil else { return }
                started = true
                do {
                    try await Task.sleep(for: .seconds(10))
                } catch {
                    cancelled = true
                    throw error
                }
            })
        let presentation = Task { @MainActor in
            try await service.present(payload: Data("payload".utf8))
        }
        while !started { await Task.yield() }

        service.cancel()
        _ = try? await presentation.value

        XCTAssertTrue(cancelled)
    }

    private static let corners = OrientedMarkerCorners(
        topLeft: Vec2(0, 1), topRight: Vec2(1, 1), bottomLeft: Vec2(0, 0), bottomRight: Vec2(1, 0)
    )

    private func makeSnapshot(generation: UInt64, sequence: UInt64,
                              timestamp: TimeInterval) -> ARFrameSnapshot {
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 8, 6, kCVPixelFormatType_32BGRA, nil, &image)
        return ARFrameSnapshot(id: ARFrameID(generation: generation, sequence: sequence), timestamp: timestamp,
            image: image!, cameraTransform: matrix_identity_float4x4,
            cameraIntrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 8, height: 6),
            depthMap: nil, pose: Pose2D(position: Vec2(0, 0), yaw: 0), trackingQuality: .normal)
    }
}
