import CoreVideo
import simd
import XCTest
import RoverNav
@testable import PhroverKit

final class DepthSafetyEvaluatorTests: XCTestCase {
    func testBoundedProbeUsesObservedStoppingCorridorWithoutCruiseLookahead() {
        // Real projection/evaluator, deliberately incomplete depth outside the
        // stopping corridor. This is a synthetic regression, not device imagery.
        let map = stoppingCorridorDepth()
        let snapshot = DepthSafetyEvaluator.ingest(rawDepthMap: map, intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration, geometry: geometry)
        let command = WheelCommand(left: 0.25, right: 0.25)
        let normal = DepthSafetyEvaluator.evaluate(snapshot, command: command, now: 10.05)
        XCTAssertEqual(normal.state, .unavailable(.blindSweptVolume))
        XCTAssertEqual(normal.coverage?.horizon ?? 0, 0.65, accuracy: 1e-12)
        XCTAssertGreaterThan(normal.coverage?.missingDepthPoints ?? 0, 0)
        let probe = DepthSafetyEvaluator.evaluate(snapshot, command: command, now: 10.05, boundedReadyMotion: true)
        XCTAssertEqual(probe.state, .clear)
        XCTAssertEqual(probe.requiredStoppingDistance, 0.30)
        XCTAssertEqual(probe.coverage?.horizon, 0.30)
        XCTAssertGreaterThanOrEqual(probe.coverage?.fraction ?? 0, 0.90)
        XCTAssertEqual(probe.coverageProvenance, "bounded_ready_motion_stopping_corridor")
    }

    @MainActor
    func testARDepthProviderUsesProbeScopeOnlyDuringExplicitAssessment() {
        let ar = ARSessionManager()
        ar.ingestDepthSafety(rawDepthMap: stoppingCorridorDepth(), intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10)
        let command = WheelCommand(left: 0.25, right: 0.25)
        let probe = ReadyMotionDepthScope.$active.withValue(true) {
            ar.depthSafetyObservation(for: command, at: 10.05)
        }
        XCTAssertEqual(probe.state, .clear)
        XCTAssertEqual(ar.depthSafetyObservation(for: command, at: 10.05).state, .unavailable(.blindSweptVolume))
        let stale = ReadyMotionDepthScope.$active.withValue(true) {
            ar.depthSafetyObservation(for: command, at: 10.251)
        }
        XCTAssertEqual(stale.state, .unavailable(.staleRawDepth))
    }

    func testProbeCannotClearNearBlindSpaceOrKnownStoppingCorridorObstacle() {
        let nearBlind = DepthSafetyEvaluator.ingest(
            rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: 3), intrinsics: ordinaryIntrinsics,
            cameraTransform: cameraTransform(height: 0.30), timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration, geometry: geometry)
        let command = WheelCommand(left: 0.25, right: 0.25)
        let blind = DepthSafetyEvaluator.evaluate(nearBlind, command: command, now: 10.05, boundedReadyMotion: true)
        XCTAssertEqual(blind.state, .unavailable(.blindSweptVolume))
        XCTAssertGreaterThan(blind.coverage?.outsideViewPoints ?? 0, 0)
        XCTAssertEqual(blind.coverage?.missingDepthPoints, 0)
        let cells = Set((1...3).map { DepthSafetyCell(lateralIndex: 0, heightIndex: $0, forwardIndex: 10) })
        let obstacle = DepthSafetySnapshot(timestamp: 10, cells: cells, cellResolution: 0.04,
            validSampleRatio: 1, geometry: geometry, projection: nearBlind.projection, failureReason: nil)
        let blocked = DepthSafetyEvaluator.evaluate(obstacle, command: command, now: 10.05, boundedReadyMotion: true)
        XCTAssertEqual(blocked.state, .stop)
        XCTAssertEqual(blocked.requiredStoppingDistance, 0.30)
        for other in [WheelCommand(left: 0.5, right: 0.5), .init(left: 0.2, right: 0.25),
                      .init(left: -0.25, right: -0.25), .init(left: -0.25, right: 0.25)] {
            let result = DepthSafetyEvaluator.evaluate(nearBlind, command: other, now: 10.05, boundedReadyMotion: true)
            XCTAssertNotEqual(result.coverageProvenance, "bounded_ready_motion_stopping_corridor")
        }
    }

    private func stoppingCorridorDepth() -> CVPixelBuffer {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: .nan)
        for index in 0...8 {
            let z = 0.15 + 0.30 * Double(index) / 8
            for x in [-0.205, 0, 0.205] {
                for height in [0.06, 0.27, 0.48] {
                    let px = Int((40 + 20 * x / z).rounded())
                    let py = Int((30 - 8 * (height - 0.30) / z).rounded())
                    fill(depth: 3, x: px...px, y: py...py, in: map)
                }
            }
        }
        return map
    }

    private let geometry = RoverCollisionGeometry(
        length: 0.30,
        width: 0.25,
        minimumCollisionHeight: 0.04,
        maximumCollisionHeight: 0.50
    )

    func testConfirmedRotationStopsAtThirtyCentimeterRadialGap() {
        // Centers at (+/-2 cm, 54 cm), with a 25.4 cm swept chassis radius
        // and 2.83 cm voxel uncertainty: conservative gap is about 25.8 cm.
        let cells = Set((2...4).map { DepthSafetyCell(lateralIndex: 0, heightIndex: $0, forwardIndex: 13) })
        let snapshot = DepthSafetySnapshot(timestamp: 10, cells: cells, cellResolution: 0.04,
            validSampleRatio: 1, geometry: geometry, projection: nil, failureReason: nil)
        let result = DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: -0.25, right: 0.25),
            now: 10.05, operatorClearedRotation: true)
        XCTAssertEqual(result.state, .stop)
        XCTAssertLessThanOrEqual(result.clearance, 0.30)
    }

    func testConfirmedRotationRadialBoundaryCautionAndDynamicStopping() {
        func evaluate(_ index: Int, speed: Double = 0.25, authorized: Bool = true,
                      command: WheelCommand? = nil) -> DepthSafetyObservation {
            let cells = Set((2...4).map { DepthSafetyCell(lateralIndex: 0, heightIndex: $0, forwardIndex: index) })
            let snapshot = DepthSafetySnapshot(timestamp: 10, cells: cells, cellResolution: 0.04,
                validSampleRatio: 1, geometry: geometry, projection: nil, failureReason: nil)
            return DepthSafetyEvaluator.evaluate(snapshot, command: command ?? .init(left: -speed, right: speed),
                now: 10.05, operatorClearedRotation: authorized)
        }
        // 58 cm center minus 25.402 cm chassis sweep and 2.828 cm cell
        // uncertainty gives 29.804 cm: including uncertainty is decisive.
        let inside = evaluate(14)
        XCTAssertEqual(inside.state, .stop)
        XCTAssertEqual(inside.clearance, 0.29804, accuracy: 0.0001)
        let outside = evaluate(15)
        XCTAssertEqual(outside.state, .caution)
        XCTAssertEqual(outside.clearance, 0.33801, accuracy: 0.0001)
        XCTAssertEqual(outside.coverageProvenance, "operator_confirmed_rotation")
        XCTAssertEqual(outside.speedLimit, 0.12)
        XCTAssertEqual(evaluate(15, authorized: false).state, .unavailable(.blindSweptVolume))
        XCTAssertEqual(evaluate(15, speed: 0.8).state, .stop, "Dynamic distance must exceed the 30 cm minimum")
        XCTAssertGreaterThan(evaluate(15, speed: 0.8).requiredStoppingDistance, 0.30)
        XCTAssertEqual(evaluate(24).state, .clear)
        XCTAssertEqual(evaluate(24, command: .init(left: 0.25, right: 0.25)).state, .unavailable(.blindSweptVolume))
        XCTAssertEqual(evaluate(24, command: .init(left: -0.25, right: -0.25)).state, .unavailable(.blindSweptVolume))
        XCTAssertEqual(evaluate(24, command: .init(left: -0.24, right: 0.25)).state, .unavailable(.blindSweptVolume))
    }

    func testOperatorClearedRotationDoesNotAuthorizeForwardObstacleCorridor() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: 1.0, x: 35...44, y: 33...37, in: map)
        let snapshot = DepthSafetyEvaluator.ingest(rawDepthMap: map, intrinsics: ordinaryIntrinsics,
            cameraTransform: cameraTransform(height: 0.55), timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55), geometry: geometry)
        let rotating = DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: -0.25, right: 0.25),
            now: 10.05, operatorClearedRotation: true)
        XCTAssertEqual(rotating.state, .clear)
        XCTAssertEqual(rotating.coverageProvenance, "operator_confirmed_rotation")
        XCTAssertEqual(rotating.sampleAge, 0.05, accuracy: 0.00001)
        let forward = DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: 1.2, right: 1.2),
            now: 10.05, operatorClearedRotation: true)
        XCTAssertEqual(forward.state, .stop)
        XCTAssertGreaterThan(forward.requiredStoppingDistance, 0.30)
    }

    func testOperatorClearedRotationRetainsVisibleHazardAndRawDepthVetoes() {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: 0.20, x: 30...49, y: 30...59, in: map)
        let snapshot = DepthSafetyEvaluator.ingest(rawDepthMap: map, intrinsics: ordinaryIntrinsics,
            cameraTransform: cameraTransform(height: 0.55), timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55), geometry: geometry)
        XCTAssertEqual(DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: -0.25, right: 0.25),
            now: 10.05, operatorClearedRotation: true).state, .stop)
        XCTAssertEqual(DepthSafetyEvaluator.evaluate(snapshot, command: .init(left: -0.25, right: 0.25),
            now: 10.251, operatorClearedRotation: true).state, .unavailable(.staleRawDepth))
        let malformed = DepthSafetyEvaluator.ingest(rawDepthMap: makeDepthBuffer(width: 80, height: 60, constantDepth: .nan),
            intrinsics: ordinaryIntrinsics, cameraTransform: cameraTransform(height: 0.55), timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55), geometry: geometry)
        XCTAssertEqual(DepthSafetyEvaluator.evaluate(malformed, command: .init(left: -0.25, right: 0.25),
            now: 10.05, operatorClearedRotation: true).state, .unavailable(.malformedDepth))
    }

    func testFollowSpeedUsesThirtyCentimeterTotalStoppingDistance() {
        let observation = stoppingMarginObservation(speed: 0.25, age: 0.15, obstacleDepth: 0.50)

        XCTAssertEqual(observation.requiredStoppingDistance, 0.30, accuracy: 0.0001)
        XCTAssertGreaterThan(observation.clearance, 0.30)
        XCTAssertLessThan(observation.clearance, 0.38)
        XCTAssertEqual(observation.state, .caution)
        XCTAssertGreaterThanOrEqual(observation.supportCount, 3)
    }

    func testObstacleInsideThirtyCentimetersStillStops() {
        let observation = stoppingMarginObservation(speed: 0.25, age: 0.15, obstacleDepth: 0.43)

        XCTAssertEqual(observation.requiredStoppingDistance, 0.30, accuracy: 0.0001)
        XCTAssertLessThanOrEqual(observation.clearance, 0.30)
        XCTAssertEqual(observation.state, .stop)
        XCTAssertGreaterThanOrEqual(observation.supportCount, 3)
    }

    func testHigherSpeedRetainsLatencyAndBrakingDistanceBeyondThirtyCentimeters() {
        let observation = stoppingMarginObservation(speed: 0.50, age: 0.20, obstacleDepth: 0.50)

        // 0.50 m/s * (0.20 s sample age + 0.25 s latency), plus 0.15625 m braking.
        XCTAssertEqual(observation.requiredStoppingDistance, 0.38125, accuracy: 0.0001)
        XCTAssertGreaterThan(observation.clearance, 0.30)
        XCTAssertLessThan(observation.clearance, observation.requiredStoppingDistance)
        XCTAssertEqual(observation.state, .stop)
    }

    private func stoppingMarginObservation(speed: Double, age: TimeInterval, obstacleDepth: Float) -> DepthSafetyObservation {
        let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
        fill(depth: obstacleDepth, x: 35...44, y: 33...37, in: map)
        let snapshot = DepthSafetyEvaluator.ingest(
            rawDepthMap: map,
            intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.55),
            timestamp: 10,
            calibration: CameraMountCalibration(cameraHeight: 0.55),
            geometry: geometry
        )
        return DepthSafetyEvaluator.evaluate(
            snapshot,
            command: WheelCommand(left: speed, right: speed),
            now: 10 + age
        )
    }

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

@MainActor
final class FollowSearchClearanceTests: XCTestCase {
    private func sample(x: Double = 0, generation: UInt64 = 7, now: Double = 10,
                        tracking: ARTrackingQuality = .normal) -> NavigationPoseSample {
        .init(pose: .init(position: .init(x, 0), yaw: 0), frameID: .init(generation: generation, sequence: 1),
              sourceTimestamp: now, trackingQuality: tracking)
    }

    func testHealthyRotationRetainsGrantThroughTwentyCentimeterObservedShift() throws {
        let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: 10).consume(session: 3))
        XCTAssertTrue(grant.validate(sample: sample(x: 0.12), session: 3, now: 10))
        XCTAssertTrue(grant.validate(sample: sample(x: 0.20), session: 3, now: 10))
    }

    func testConfirmationIsSingleSessionAndNeverRenewsAbsoluteExpiry() throws {
        let confirmation = FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: 10)
        let grant = try XCTUnwrap(confirmation.consume(session: 3))
        XCTAssertNil(confirmation.consume(session: 4))
        XCTAssertTrue(grant.validate(sample: sample(now: 69.99), session: 3, now: 69.99))
        XCTAssertFalse(grant.validate(sample: sample(now: 70), session: 3, now: 70))
        XCTAssertTrue(grant.revoked)
    }

    func testSourceResetDriftAndTrackingLossPermanentlyRevoke() throws {
        for invalid in [sample(x: 0.201), sample(generation: 8), sample(tracking: .limited), sample(now: 9), NavigationPoseSample.unavailable] {
            let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: 10).consume(session: 3))
            XCTAssertFalse(grant.validate(sample: invalid, session: 3, now: 10))
            XCTAssertFalse(grant.validate(sample: sample(), session: 3, now: 10), "Returning to origin cannot restore assertion")
        }
    }

    func testExcursionReasonAndFrameLatchAcrossReturnAndCleanup() throws {
        let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: 10).consume(session: 3))
        XCTAssertFalse(grant.validate(sample: sample(x: 0.20001, now: 11), session: 3, now: 11))
        XCTAssertFalse(grant.validate(sample: sample(now: 12), session: 3, now: 12))
        grant.revoke(now: 13)
        XCTAssertEqual(grant.revocationCause, .searchClearanceSpaceExceeded)
        XCTAssertEqual(grant.diagnosticFields["search_clearance_observed_displacement_m"], .number(0.20001))
        XCTAssertEqual(grant.diagnosticFields["search_clearance_observed_frame"], .string("7:1"))
        XCTAssertEqual(grant.diagnosticFields["search_clearance_source_timestamp_s"], .number(11))
        XCTAssertEqual(grant.diagnosticFields["search_clearance_revoked_at_s"], .number(11))
        XCTAssertEqual(grant.diagnosticFields["search_clearance_displacement_limit_m"], .number(0.20))
    }

    func testRadialBudgetDoesNotAccumulatePathOrRebaseOrigin() throws {
        let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: 10).consume(session: 3))
        for x in [0.19, -0.19, 0.19, -0.19] {
            XCTAssertTrue(grant.validate(sample: sample(x: x), session: 3, now: 10))
        }
        XCTAssertFalse(grant.validate(sample: sample(x: -0.20001), session: 3, now: 10))
    }

    func testFutureNonfiniteAndDifferentSessionAssertionsFailClosed() throws {
        for timestamp in [11.0, .infinity, .nan] {
            let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: timestamp).consume(session: 3))
            XCTAssertFalse(grant.validate(sample: sample(), session: 3, now: 10))
        }
        let grant = try XCTUnwrap(FollowSearchClearanceConfirmation(sourceGeneration: 7, origin: .zero, confirmedAt: 10).consume(session: 3))
        XCTAssertFalse(grant.validate(sample: sample(), session: 4, now: 10))
    }
}
