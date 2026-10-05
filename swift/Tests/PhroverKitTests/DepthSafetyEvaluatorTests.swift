import CoreVideo
import simd
import XCTest
import RoverNav
@testable import PhroverKit

final class DepthSafetyEvaluatorTests: XCTestCase {
    private let geometry = RoverCollisionGeometry(
        length: 0.30,
        width: 0.25,
        minimumCollisionHeight: 0.04,
        maximumCollisionHeight: 0.50
    )

    func testVisibleNarrowLowCrossbarProducesCautionInsteadOfLookingPastIt() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        // The wide synthetic frustum sees the full near swept volume. These rows are a low crossbar.
        fill(depth: 0.70, x: 35...44, y: 33...37, in: map)

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: wideIntrinsics,
                cameraTransform: cameraTransform(height: 0.55),
                timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.55),
                geometry: geometry
            ),
            command: WheelCommand(left: 0.35, right: 0.35),
            now: 10.05
        )

        XCTAssertEqual(observation.state, .caution)
        XCTAssertEqual(observation.clearance, 0.55, accuracy: 0.08)
        XCTAssertGreaterThanOrEqual(observation.supportCount, 3)
    }

    func testBlindNearSweptVolumeFailsClosedInsteadOfClearing() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: ordinaryIntrinsics,
                cameraTransform: cameraTransform(height: 0.55),
                timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.55),
                geometry: geometry
            ),
            command: WheelCommand(left: 0.20, right: 0.20),
            now: 10.05
        )

        XCTAssertEqual(observation.state, .unavailable(.blindSweptVolume))
    }

    func testSingleNearDepthOutlierDoesNotCreateHazardWhenSweepIsVisible() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: 0.70, x: 40...40, y: 35...35, in: map)

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: wideIntrinsics,
                cameraTransform: cameraTransform(height: 0.55),
                timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.55),
                geometry: geometry
            ),
            command: WheelCommand(left: 0.20, right: 0.20),
            now: 10.05
        )

        XCTAssertEqual(observation.state, .clear)
    }

    func testFloorPlaneDoesNotBecomeObstacleWhenConfiguredMountHeightIsTooHigh() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fillFloorPlane(cameraHeight: 0.40, in: map)

        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: map,
            intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.40),
            timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55),
            geometry: geometry
        )
        let commands = [
            WheelCommand(left: 0.20, right: 0.20),
            WheelCommand(left: 0.15, right: 0.20),
        ]

        for command in commands {
            let observation = DepthSafetyEvaluator.evaluate(snapshot, command: command, now: 10.05)
            XCTAssertEqual(observation.state, .clear, "command=\(command)")
        }
        XCTAssertEqual(
            DepthSafetyEvaluator.evaluate(
                snapshot,
                command: WheelCommand(left: 0.10, right: 0.30),
                now: 10.05
            ).state,
            .unavailable(.blindSweptVolume)
        )
    }

    func testThirtyCentimeterProductionMountKeepsVisibleFloorClear() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fillFloorPlane(cameraHeight: 0.30, in: map)

        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: map,
            intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.30),
            timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration,
            geometry: geometry
        )
        let observation = DepthSafetyEvaluator.evaluate(
            snapshot,
            command: WheelCommand(left: 0.20, right: 0.20),
            now: 10.05
        )

        XCTAssertEqual(RoverConfig.cameraMountCalibration.cameraHeight, 0.30)
        XCTAssertEqual(observation.state, .clear)
    }

    func testThirtyCentimeterProductionMountStillDetectsLowCrossbar() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: 0.70, x: 35...44, y: 31...32, in: map)

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: wideIntrinsics,
                cameraTransform: cameraTransform(height: 0.30),
                timestamp: 10,
                calibration: RoverConfig.cameraMountCalibration,
                geometry: geometry
            ),
            command: WheelCommand(left: 0.35, right: 0.35),
            now: 10.05
        )

        XCTAssertTrue(observation.state == .caution || observation.state == .stop)
        XCTAssertLessThan(observation.clearance, 0.80)
        XCTAssertGreaterThanOrEqual(observation.supportCount, 3)
    }

    func testIntrinsicsAreScaledFromCameraImageToDepthResolution() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: 0.70, x: 35...44, y: 33...37, in: map)
        let cameraIntrinsics = simd_float3x3(columns: (
            SIMD3<Float>(40, 0, 0),
            SIMD3<Float>(0, 16, 0),
            SIMD3<Float>(80, 60, 1)
        ))

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: cameraIntrinsics,
                intrinsicsImageSize: CGSize(width: 160, height: 120),
                cameraTransform: cameraTransform(height: 0.55),
                timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.55),
                geometry: geometry
            ),
            command: WheelCommand(left: 0.35, right: 0.35),
            now: 10.05
        )

        XCTAssertEqual(observation.state, .caution)
    }

    func testCurvedCommandUsesArcContactDistance() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: 0.35, x: 27...31, y: 36...40, in: map)

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: wideIntrinsics,
                cameraTransform: cameraTransform(height: 0.55),
                timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.55),
                geometry: geometry
            ),
            command: WheelCommand(left: 0.10, right: 0.30),
            now: 10.05
        )

        XCTAssertEqual(observation.motionClass, .curved)
        XCTAssertEqual(observation.state, .stop)
        XCTAssertTrue(observation.clearance.isFinite)
    }

    func testForwardOnlyDepthCannotAuthorizeInPlaceRotation() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)

        let observation = DepthSafetyEvaluator.evaluate(
            DepthSafetyEvaluator.ingest(
                rawDepthMap: map,
                intrinsics: wideIntrinsics,
                cameraTransform: cameraTransform(height: 0.55),
                timestamp: 10,
                calibration: CameraMountCalibration(cameraHeight: 0.55),
                geometry: geometry
            ),
            command: WheelCommand(left: -0.15, right: 0.15),
            now: 10.05
        )

        XCTAssertEqual(observation.motionClass, .rotating)
        XCTAssertEqual(observation.state, .unavailable(.blindSweptVolume))
    }

    func testStaleRawDepthFailsClosed() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: map,
            intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.55),
            timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55),
            geometry: geometry
        )

        XCTAssertEqual(
            DepthSafetyEvaluator.evaluate(
                snapshot,
                command: WheelCommand(left: 0.20, right: 0.20),
                now: 10.31
            ).state,
            .unavailable(.staleRawDepth)
        )
    }

    func testReverseFailsClosedWithForwardFacingDepth() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: map,
            intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.55),
            timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55),
            geometry: geometry
        )

        XCTAssertEqual(
            DepthSafetyEvaluator.evaluate(
                snapshot,
                command: WheelCommand(left: -0.20, right: -0.20),
                now: 10.05
            ).state,
            .unavailable(.blindSweptVolume)
        )
    }

    private var ordinaryIntrinsics: simd_float3x3 {
        simd_float3x3(columns: (
            SIMD3<Float>(50, 0, 0),
            SIMD3<Float>(0, 40, 0),
            SIMD3<Float>(40, 30, 1)
        ))
    }

    private var wideIntrinsics: simd_float3x3 {
        simd_float3x3(columns: (
            SIMD3<Float>(20, 0, 0),
            SIMD3<Float>(0, 8, 0),
            SIMD3<Float>(40, 30, 1)
        ))
    }

    private func cameraTransform(height: Float) -> simd_float4x4 {
        var transform = matrix_identity_float4x4
        transform.columns.3 = SIMD4<Float>(0, height, 0, 1)
        return transform
    }

    private func makeDepthBuffer(width: Int, height: Int, constantDepth: Float) -> CVPixelBuffer {
        var pixelBuffer: CVPixelBuffer?
        CVPixelBufferCreate(
            kCFAllocatorDefault,
            width,
            height,
            kCVPixelFormatType_DepthFloat32,
            nil,
            &pixelBuffer
        )
        let buffer = pixelBuffer!
        fill(depth: constantDepth, x: 0...(width - 1), y: 0...(height - 1), in: buffer)
        return buffer
    }

    private func fill(depth: Float,
                      x: ClosedRange<Int>,
                      y: ClosedRange<Int>,
                      in buffer: CVPixelBuffer) {
        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        let stride = CVPixelBufferGetBytesPerRow(buffer) / MemoryLayout<Float32>.size
        let base = CVPixelBufferGetBaseAddress(buffer)!.assumingMemoryBound(to: Float32.self)
        for row in y {
            for column in x {
                base[row * stride + column] = depth
            }
        }
    }

    private func fillFloorPlane(cameraHeight: Float, in buffer: CVPixelBuffer) {
        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        let width = CVPixelBufferGetWidth(buffer)
        let height = CVPixelBufferGetHeight(buffer)
        let stride = CVPixelBufferGetBytesPerRow(buffer) / MemoryLayout<Float32>.size
        let base = CVPixelBufferGetBaseAddress(buffer)!.assumingMemoryBound(to: Float32.self)
        let fy = wideIntrinsics[1][1]
        let cy = wideIntrinsics[2][1]
        for row in Int(cy + 1)..<height {
            let depth = cameraHeight * fy / (Float(row) - cy)
            for column in 0..<width {
                base[row * stride + column] = depth
            }
        }
    }
}
