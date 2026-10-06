import CoreVideo
import PhroverKit
import RoverNav
import simd
import UIKit
import XCTest
@testable import PhroverOperator

@MainActor
final class CalibrationPreviewModelTests: XCTestCase {
    func testPreviewRendersFirstAndNewestEligibleFrame() async throws {
        let frames = AsyncStream.makeStream(of: ARFrameSnapshot.self)
        let firstImage = UIImage()
        let secondImage = UIImage()
        var renderedSequences: [UInt64] = []
        let model = CalibrationPreviewModel(frames: { frames.stream }) { snapshot in
            renderedSequences.append(snapshot.id.sequence)
            return snapshot.id.sequence == 1 ? firstImage : secondImage
        }

        model.start()
        frames.continuation.yield(try snapshot(sequence: 1, timestamp: 10))
        frames.continuation.yield(try snapshot(sequence: 2, timestamp: 10.099))
        frames.continuation.yield(try snapshot(sequence: 3, timestamp: 10.101))
        await waitUntil { model.image === secondImage }

        XCTAssertEqual(renderedSequences, [1, 3])
        XCTAssertTrue(model.image === secondImage)
        XCTAssertEqual(
            model.latestRenderedFrameContext,
            SilentSearchCalibrationFrameContext(
                frameID: ARFrameID(generation: 1, sequence: 3), monotonicTimestamp: 10.101
            )
        )
    }

    func testStopCancelsConsumptionAndReleasesImage() async throws {
        let frames = AsyncStream.makeStream(of: ARFrameSnapshot.self)
        var renderCount = 0
        let model = CalibrationPreviewModel(frames: { frames.stream }) { _ in
            renderCount += 1
            return UIImage()
        }
        model.start()
        frames.continuation.yield(try snapshot(sequence: 1, timestamp: 1))
        await waitUntil { model.image != nil }

        model.stop()
        frames.continuation.yield(try snapshot(sequence: 2, timestamp: 2))
        await Task.yield()

        XCTAssertNil(model.image)
        XCTAssertNil(model.latestRenderedFrameContext)
        XCTAssertEqual(renderCount, 1)
    }

    func testRepeatedStartAndStopDoesNotCreateDuplicateConsumers() async throws {
        var frames = AsyncStream.makeStream(of: ARFrameSnapshot.self)
        var renderCount = 0
        let model = CalibrationPreviewModel(frames: { frames.stream }) { _ in
            renderCount += 1
            return UIImage()
        }

        model.start()
        model.start()
        frames.continuation.yield(try snapshot(sequence: 1, timestamp: 1))
        await waitUntil { renderCount == 1 }
        model.stop()
        frames = AsyncStream.makeStream(of: ARFrameSnapshot.self)
        model.start()
        frames.continuation.yield(try snapshot(sequence: 2, timestamp: 2))
        await waitUntil { renderCount == 2 }

        XCTAssertEqual(renderCount, 2)
    }

    func testTransformAspectFitsPortraitImageAndConvertsScannerCoordinates() {
        let transform = CalibrationPreviewTransform(
            imageSize: CGSize(width: 300, height: 600),
            previewBounds: CGRect(x: 0, y: 0, width: 400, height: 400)
        )

        XCTAssertEqual(transform.imageRect, CGRect(x: 100, y: 0, width: 200, height: 400))
        assertPoint(transform.point(for: Vec2(0, 1)), x: 100, y: 0)
        assertPoint(transform.point(for: Vec2(1, 1)), x: 300, y: 0)
        assertPoint(transform.point(for: Vec2(0, 0)), x: 100, y: 400)
        assertPoint(transform.point(for: Vec2(1, 0)), x: 300, y: 400)
    }

    func testTransformClampsOutOfRangeCoordinates() {
        let transform = CalibrationPreviewTransform(
            imageSize: CGSize(width: 200, height: 100),
            previewBounds: CGRect(x: 10, y: 20, width: 100, height: 100)
        )

        XCTAssertEqual(transform.imageRect, CGRect(x: 10, y: 45, width: 100, height: 50))
        assertPoint(transform.point(for: Vec2(-2, 4)), x: 10, y: 45)
        assertPoint(transform.point(for: Vec2(3, -1)), x: 110, y: 95)
    }

    func testProjectionMapsLatchedStagesAndGuidance() {
        var state = SilentSearchCalibrationVisualState()
        XCTAssertEqual(CalibrationViewProjection(state: state).stages.qrDecoded, .pending)
        XCTAssertEqual(CalibrationViewProjection(state: state).stages.cornersGrounded, .waiting)
        XCTAssertEqual(CalibrationViewProjection(state: state).stages.sampleAccepted, .waiting)
        XCTAssertEqual(
            CalibrationViewProjection(state: state).guidance,
            "Center the complete marker with its white border visible."
        )

        state.qrDecoded = true
        state.recordDetection(.init(
            context: .init(
                frameID: ARFrameID(generation: 1, sequence: 1), monotonicTimestamp: 1
            ),
            markerID: "SILENT_SEARCH_01", corners: markerCorners
        ))
        XCTAssertEqual(CalibrationViewProjection(state: state).stages.qrDecoded, .succeeded)
        XCTAssertEqual(CalibrationViewProjection(state: state).stages.cornersGrounded, .pending)
        XCTAssertEqual(CalibrationViewProjection(state: state).stages.sampleAccepted, .waiting)
        XCTAssertEqual(CalibrationViewProjection(state: state).markerText, "Marker SILENT_SEARCH_01")

        state.clearDetections()
        XCTAssertEqual(CalibrationViewProjection(state: state).stages.qrDecoded, .succeeded)
        XCTAssertNil(CalibrationViewProjection(state: state).markerText)

        state.cornersGrounded = true
        XCTAssertEqual(CalibrationViewProjection(state: state).stages.cornersGrounded, .succeeded)
        XCTAssertEqual(CalibrationViewProjection(state: state).stages.sampleAccepted, .pending)
        XCTAssertEqual(CalibrationViewProjection(state: state).guidance, "Hold steady while samples are collected.")

        state.sampleAccepted = true
        XCTAssertEqual(CalibrationViewProjection(state: state).stages.sampleAccepted, .succeeded)
    }

    func testProjectionShowsCornersOnlyForTheRenderedFrameContext() {
        let renderedContext = SilentSearchCalibrationFrameContext(
            frameID: ARFrameID(generation: 1, sequence: 2), monotonicTimestamp: 2
        )
        var state = SilentSearchCalibrationVisualState()
        XCTAssertNil(
            CalibrationViewProjection(state: state).visibleCorners,
            "Two missing contexts must not count as a context match"
        )
        state.recordDetection(.init(
            context: .init(
                frameID: ARFrameID(generation: 1, sequence: 1), monotonicTimestamp: 1
            ),
            markerID: "SILENT_SEARCH_01", corners: markerCorners
        ))
        XCTAssertNil(
            CalibrationViewProjection(
                state: state, latestRenderedFrameContext: renderedContext
            ).visibleCorners
        )

        state.recordDetection(.init(
            context: renderedContext,
            markerID: "SILENT_SEARCH_01", corners: markerCorners
        ))
        state.recordDetection(.init(
            context: .init(
                frameID: ARFrameID(generation: 1, sequence: 3), monotonicTimestamp: 2.01
            ),
            markerID: "SILENT_SEARCH_01",
            corners: OrientedMarkerCorners(
                topLeft: Vec2(0.3, 0.7), topRight: Vec2(0.7, 0.7),
                bottomLeft: Vec2(0.3, 0.3), bottomRight: Vec2(0.7, 0.3)
            )
        ))
        XCTAssertEqual(
            CalibrationViewProjection(
                state: state, latestRenderedFrameContext: renderedContext
            ).visibleCorners,
            markerCorners,
            "A newer detection must not hide the exact detection for the rendered preview frame"
        )
    }

    func testProjectionUsesExactIssueGuidancePrecedence() {
        var state = SilentSearchCalibrationVisualState()
        state.qrDecoded = true

        state.currentIssue = .groundingFailure(.cornerUnavailable(.topLeft))
        XCTAssertEqual(
            CalibrationViewProjection(state: state).guidance,
            "Move the marker toward center or adjust the camera angle."
        )
        state.currentIssue = .groundingFailure(.wrongMarkerID)
        XCTAssertEqual(CalibrationViewProjection(state: state).guidance, "Show marker SILENT_SEARCH_01.")
        state.currentIssue = .scannerFailure
        XCTAssertEqual(CalibrationViewProjection(state: state).guidance, "QR scanner unavailable. Reframe and try again.")
        state.currentIssue = .groundingFailure(.trackingNotNormal)
        XCTAssertEqual(CalibrationViewProjection(state: state).guidance, "Restore normal AR tracking before calibrating.")
    }

    func testProjectionMapsTypedCalibrationRejectionsToActionableGuidance() {
        let expectedGuidance: [(SharedMissionCalibrationDiagnostic, String)] = [
            (.invalidMarkerID, "Use a marker with a valid identifier."),
            (.unexpectedMarkerID, "Show marker EXPECTED."),
            (.generationMismatch, "AR session changed. Restart calibration."),
            (.duplicateFrame, "Keep the marker visible while waiting for a new camera frame."),
            (.nonFiniteObservation, "Reframe the complete marker and try again."),
            (.degenerateCorners, "Flatten the marker and keep its complete border visible."),
            (.timeWindowExceeded, "Calibration took too long. Hold the marker steady and try again."),
            (.originDeviationExceeded, "Marker samples disagree. Hold the marker still and try again."),
            (.headingDeviationExceeded, "Marker samples disagree. Hold the marker still and try again."),
            (.widthDeviationExceeded, "Marker samples disagree. Hold the marker still and try again."),
        ]

        for (rejection, guidance) in expectedGuidance {
            var state = SilentSearchCalibrationVisualState()
            state.currentIssue = .calibrationRejection(rejection)
            XCTAssertEqual(
                CalibrationViewProjection(state: state, expectedMarkerID: "EXPECTED").guidance,
                guidance,
                "Unexpected guidance for \(rejection)"
            )
        }
    }

    private var markerCorners: OrientedMarkerCorners {
        OrientedMarkerCorners(
            topLeft: Vec2(0.2, 0.8), topRight: Vec2(0.8, 0.8),
            bottomLeft: Vec2(0.2, 0.2), bottomRight: Vec2(0.8, 0.2)
        )
    }

    private func snapshot(sequence: UInt64, timestamp: TimeInterval) throws -> ARFrameSnapshot {
        var buffer: CVPixelBuffer?
        XCTAssertEqual(
            CVPixelBufferCreate(nil, 1, 1, kCVPixelFormatType_32BGRA, nil, &buffer),
            kCVReturnSuccess
        )
        return ARFrameSnapshot(
            id: ARFrameID(generation: 1, sequence: sequence), timestamp: timestamp,
            image: try XCTUnwrap(buffer), cameraTransform: matrix_identity_float4x4,
            cameraIntrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 1, height: 1),
            depthMap: nil, pose: Pose2D(position: .zero, yaw: 0), trackingQuality: .normal
        )
    }

    private func waitUntil(_ condition: @escaping @MainActor () -> Bool) async {
        for _ in 0..<100 where !condition() { await Task.yield() }
    }

    private func assertPoint(
        _ point: CGPoint, x: CGFloat, y: CGFloat,
        file: StaticString = #filePath, line: UInt = #line
    ) {
        XCTAssertEqual(point.x, x, accuracy: 0.000_001, file: file, line: line)
        XCTAssertEqual(point.y, y, accuracy: 0.000_001, file: file, line: line)
    }
}
