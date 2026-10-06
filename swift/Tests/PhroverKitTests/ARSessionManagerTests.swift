import ARKit
import CoreVideo
import XCTest
import simd
import RoverNav
@testable import PhroverKit

@MainActor
final class ARSessionManagerTests: XCTestCase {
    func testPlainObservationsAreMonotonicWithinExplicitGeneration() {
        let ar = ARSessionManager()
        ar.resetTracking(generation: 4, runSession: false)

        XCTAssertTrue(ar.ingest(PoseObservation(
            pose: Pose2D(position: Vec2(1, 2), yaw: 0.3),
            frameSequence: 1,
            timestamp: 10,
            trackingQuality: .normal,
            sessionGeneration: 4
        )))
        XCTAssertFalse(ar.ingest(PoseObservation(
            pose: Pose2D(position: Vec2(9, 9), yaw: 0),
            frameSequence: 1,
            timestamp: 11,
            trackingQuality: .normal,
            sessionGeneration: 4
        )))

        XCTAssertEqual(ar.pose, Pose2D(position: Vec2(1, 2), yaw: 0.3))
        XCTAssertEqual(ar.frameSequence, 1)
        XCTAssertEqual(ar.latestObservation?.timestamp, 10)
    }

    func testResetSynchronouslyClearsStaleStateAndRejectsOldGeneration() {
        let ar = ARSessionManager()
        ar.resetTracking(generation: 7, runSession: false)
        XCTAssertTrue(ar.ingest(PoseObservation(
            pose: Pose2D(position: Vec2(1, 2), yaw: 0),
            frameSequence: 3,
            timestamp: 20,
            trackingQuality: .normal,
            sessionGeneration: 7
        )))

        ar.resetTracking(generation: 8, runSession: false)

        XCTAssertNil(ar.pose)
        XCTAssertNil(ar.latestObservation)
        XCTAssertNil(ar.latestPixelBuffer)
        XCTAssertNil(ar.latestDepthMap)
        XCTAssertEqual(ar.frameSequence, 0)
        XCTAssertNil(ar.latestDepthSafetySnapshot)
        XCTAssertFalse(ar.ingest(PoseObservation(
            pose: Pose2D(position: Vec2(8, 8), yaw: 0),
            frameSequence: 4,
            timestamp: 21,
            trackingQuality: .normal,
            sessionGeneration: 7
        )))
    }


    func testRawDepthIngestionPublishesAndResetClearsSafetySnapshot() {
        let ar = ARSessionManager()
        ar.resetTracking(generation: 1, runSession: false)
        var depthMap: CVPixelBuffer?
        CVPixelBufferCreate(
            kCFAllocatorDefault,
            8,
            6,
            kCVPixelFormatType_DepthFloat32,
            nil,
            &depthMap
        )
        let map = depthMap!
        CVPixelBufferLockBaseAddress(map, [])
        let stride = CVPixelBufferGetBytesPerRow(map) / MemoryLayout<Float32>.size
        let base = CVPixelBufferGetBaseAddress(map)!.assumingMemoryBound(to: Float32.self)
        for row in 0..<6 {
            for column in 0..<8 { base[row * stride + column] = 2.0 }
        }
        CVPixelBufferUnlockBaseAddress(map, [])
        let intrinsics = simd_float3x3(columns: (
            SIMD3<Float>(5, 0, 0),
            SIMD3<Float>(0, 5, 0),
            SIMD3<Float>(4, 3, 1)
        ))
        var transform = matrix_identity_float4x4
        transform.columns.3.y = 0.55

        ar.ingestDepthSafety(
            rawDepthMap: map,
            intrinsics: intrinsics,
            cameraTransform: transform,
            timestamp: 12
        )

        XCTAssertEqual(ar.latestDepthSafetySnapshot?.timestamp, 12)
        ar.resetTracking(generation: 2, runSession: false)
        XCTAssertNil(ar.latestDepthSafetySnapshot)
    }

    func testPoseObservationAdaptsLosslesslyToTransitionObservation() {
        let observation = PoseObservation(
            pose: Pose2D(position: Vec2(2, 3), yaw: 0.4),
            frameSequence: 9,
            timestamp: 42,
            trackingQuality: .normal,
            sessionGeneration: 6
        )

        XCTAssertEqual(observation.transitionObservation, TransitionObservation(
            pose: observation.pose,
            frameSequence: 9,
            timestamp: 42,
            isTrackingNormal: true,
            sessionGeneration: 6
        ))
    }

    func testResetNotificationOccursBeforeNewSamplesCanBeAccepted() {
        let ar = ARSessionManager()
        var acceptedDuringReset: Bool?
        ar.onReset = { generation in
            acceptedDuringReset = ar.ingest(PoseObservation(
                pose: Pose2D(position: .zero, yaw: 0),
                frameSequence: 1,
                timestamp: 1,
                trackingQuality: .normal,
                sessionGeneration: generation
            ))
        }

        ar.resetTracking(generation: 12, runSession: false)

        XCTAssertEqual(acceptedDuringReset, false)
        XCTAssertTrue(ar.ingest(PoseObservation(
            pose: Pose2D(position: .zero, yaw: 0),
            frameSequence: 1,
            timestamp: 1,
            trackingQuality: .normal,
            sessionGeneration: 12
        )))
    }

    func testInterruptionResumesWithFreshMatchingGenerationObservation() {
        let ar = ARSessionManager()
        ar.resetTracking(generation: 2, runSession: false)
        var resetGenerations: [UInt64] = []
        var acceptedPositions: [Vec2] = []
        ar.onReset = { resetGenerations.append($0) }
        ar.observationHandler = { acceptedPositions.append($0.pose.position) }
        XCTAssertTrue(ar.ingest(PoseObservation(
            pose: Pose2D(position: .zero, yaw: 0),
            frameSequence: 1,
            timestamp: 1,
            trackingQuality: .normal,
            sessionGeneration: 2
        )))

        ar.sessionWasInterrupted(ar.session)

        XCTAssertNil(ar.pose)
        XCTAssertNil(ar.latestObservation)
        XCTAssertFalse(ar.isTrackingNormal)
        XCTAssertFalse(ar.ingest(PoseObservation(
            pose: Pose2D(position: Vec2(1, 0), yaw: 0),
            frameSequence: 2,
            timestamp: 2,
            trackingQuality: .normal,
            sessionGeneration: 2
        )))

        ar.sessionInterruptionEnded(ar.session)

        XCTAssertFalse(ar.ingest(PoseObservation(
            pose: Pose2D(position: Vec2(9, 9), yaw: 0),
            frameSequence: 1,
            timestamp: 3,
            trackingQuality: .normal,
            sessionGeneration: 2
        )))
        XCTAssertFalse(ar.ingest(PoseObservation(
            pose: Pose2D(position: Vec2(7, 7), yaw: 0),
            frameSequence: 2,
            timestamp: 1,
            trackingQuality: .normal,
            sessionGeneration: 2
        )))
        XCTAssertFalse(ar.ingest(PoseObservation(
            pose: Pose2D(position: Vec2(8, 8), yaw: 0),
            frameSequence: 2,
            timestamp: 3,
            trackingQuality: .normal,
            sessionGeneration: 1
        )))
        XCTAssertTrue(ar.ingest(PoseObservation(
            pose: Pose2D(position: Vec2(2, 0), yaw: 0.2),
            frameSequence: 2,
            timestamp: 3,
            trackingQuality: .normal,
            sessionGeneration: 2
        )))
        XCTAssertEqual(ar.pose, Pose2D(position: Vec2(2, 0), yaw: 0.2))
        XCTAssertEqual(ar.frameSequence, 2)
        XCTAssertEqual(acceptedPositions, [.zero, Vec2(2, 0)])
        XCTAssertTrue(resetGenerations.isEmpty)
    }

    func testRelativeHeadingMeasurementIsZeroBasedAndMountIndependent() {
        let ar = ARSessionManager()
        ar.beginRelativeHeadingMeasurement()

        XCTAssertTrue(ar.ingestRelativeHeadingSample(RelativeHeadingSample(
            timestamp: 3,
            rotationRate: SIMD3(0, -2, 0),
            gravity: SIMD3(0, -1, 0)
        )))
        XCTAssertTrue(ar.ingestRelativeHeadingSample(RelativeHeadingSample(
            timestamp: 3.05,
            rotationRate: SIMD3(0, -2, 0),
            gravity: SIMD3(0, -1, 0)
        )))

        let measurement = ar.relativeHeadingMeasurement(at: 3.05)
        XCTAssertEqual(measurement.accumulatedAngle, 0.1, accuracy: 0.000_001)
        XCTAssertEqual(measurement.reliability, .reliable)
    }

    func testRelativeHeadingMotionHandlerCanRunOffMainActor() async {
        let ar = ARSessionManager()
        let handler = ar.makeRelativeHeadingMotionHandler()
        let callbackCompleted = expectation(description: "motion callback completed off main actor")

        DispatchQueue.global(qos: .userInteractive).async {
            handler(nil, nil)
            callbackCompleted.fulfill()
        }

        await fulfillment(of: [callbackCompleted], timeout: 1)
    }

    func testTrackingResetInvalidatesRelativeHeadingMeasurement() {
        let ar = ARSessionManager()
        ar.beginRelativeHeadingMeasurement()
        XCTAssertTrue(ar.ingestRelativeHeadingSample(RelativeHeadingSample(
            timestamp: 4,
            rotationRate: SIMD3(0, -1, 0),
            gravity: SIMD3(0, -1, 0)
        )))

        ar.resetTracking(generation: 9, runSession: false)

        XCTAssertEqual(
            ar.relativeHeadingMeasurement(at: 4).reliability,
            .unreliable(.sessionGenerationChanged)
        )
    }

    func testRelativeHeadingTelemetryReportsReliabilityTransitionsWithoutPerSampleSpam() {
        var events: [(String, [String: String])] = []
        let ar = ARSessionManager { events.append(($0, $1)) }
        ar.beginRelativeHeadingMeasurement()

        XCTAssertTrue(ar.ingestRelativeHeadingSample(RelativeHeadingSample(
            timestamp: 1,
            rotationRate: SIMD3(0, -1, 0),
            gravity: SIMD3(0, -1, 0)
        )))
        XCTAssertTrue(ar.ingestRelativeHeadingSample(RelativeHeadingSample(
            timestamp: 1.05,
            rotationRate: SIMD3(0, -1, 0),
            gravity: SIMD3(0, -1, 0)
        )))
        ar.resetTracking(generation: 10, runSession: false)

        XCTAssertEqual(events.map(\.0), [
            "relative_heading_measurement_started",
            "relative_heading_sample_accepted",
            "relative_heading_reliability_changed",
            "relative_heading_measurement_invalidated",
            "relative_heading_reliability_changed",
        ])
        XCTAssertEqual(events[2].1["from"], "not_started")
        XCTAssertEqual(events[2].1["to"], "reliable")
        XCTAssertEqual(events[3].1["reliability"], "session_generation_changed")
        XCTAssertEqual(events[4].1["to"], "session_generation_changed")
    }

    func testTrackingInterruptionPreservesRelativeHeadingInvalidationReason() {
        var events: [(String, [String: String])] = []
        let ar = ARSessionManager { events.append(($0, $1)) }
        ar.beginRelativeHeadingMeasurement()
        XCTAssertTrue(ar.ingestRelativeHeadingSample(RelativeHeadingSample(
            timestamp: 5,
            rotationRate: SIMD3(0, -1, 0),
            gravity: SIMD3(0, -1, 0)
        )))

        ar.sessionWasInterrupted(ar.session)

        XCTAssertEqual(
            ar.relativeHeadingMeasurement(at: 5).reliability,
            .unreliable(.trackingInterrupted)
        )
        XCTAssertTrue(events.contains {
            $0.0 == "relative_heading_measurement_invalidated"
                && $0.1["reliability"] == "tracking_interrupted"
        })
    }

    func testSessionFailureInvalidatesPoseAndRejectsFurtherObservations() {
        let ar = ARSessionManager()
        ar.resetTracking(generation: 20, runSession: false)
        XCTAssertTrue(ar.ingest(PoseObservation(
            pose: Pose2D(position: Vec2(1, 2), yaw: 0.5),
            frameSequence: 1,
            timestamp: 1,
            trackingQuality: .normal,
            sessionGeneration: 20
        )))

        ar.session(ar.session, didFailWithError: NSError(domain: "test", code: 1))

        XCTAssertNil(ar.pose)
        XCTAssertNil(ar.latestObservation)
        XCTAssertFalse(ar.isTrackingNormal)
        XCTAssertFalse(ar.ingest(PoseObservation(
            pose: Pose2D(position: Vec2(9, 9), yaw: 0),
            frameSequence: 2,
            timestamp: 2,
            trackingQuality: .normal,
            sessionGeneration: 20
        )))
    }

    // MARK: - Coherent frame snapshots and lifecycle (Silent Search)

    func testResetAdvancesGenerationAndFramesAdvanceSequenceOnce() throws {
        let manager = ARSessionManager()

        manager.resetForTesting()
        XCTAssertEqual(manager.sessionGeneration, 1)
        XCTAssertNil(manager.latestSnapshot)

        manager.ingestForTesting(image: makeImage(), timestamp: 10, cameraTransform: transform(x: 1, z: 2),
            intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 8, height: 6),
            depthMap: makeDepth(2), trackingQuality: .normal)
        manager.ingestForTesting(image: makeImage(), timestamp: 11, cameraTransform: transform(x: 3, z: 4),
            intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 8, height: 6),
            depthMap: nil, trackingQuality: .limited)

        XCTAssertEqual(manager.latestSnapshot?.id, ARFrameID(generation: 1, sequence: 2))
        XCTAssertEqual(manager.latestSnapshot?.timestamp, 11)
        XCTAssertEqual(manager.pose, Pose2D(position: Vec2(3, 4), yaw: -.pi / 2))
        XCTAssertEqual(manager.trackingQuality, .limited)
        XCTAssertNil(manager.latestDepthMap, "a color frame without depth must clear stale depth")

        manager.resetForTesting()
        XCTAssertEqual(manager.sessionGeneration, 2)
        manager.ingestForTesting(image: makeImage(), timestamp: 12, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 8, height: 6),
            depthMap: nil, trackingQuality: .normal)
        XCTAssertEqual(manager.latestSnapshot?.id, ARFrameID(generation: 2, sequence: 1))
    }

    func testSnapshotStreamsAreIndependentAndBufferOnlyNewestFrame() async throws {
        let manager = ARSessionManager()
        manager.resetForTesting()
        let firstStream = manager.snapshots()
        let secondStream = manager.snapshots()
        manager.ingestForTesting(image: makeImage(), timestamp: 1, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 8, height: 6),
            depthMap: nil, trackingQuality: .normal)
        manager.ingestForTesting(image: makeImage(), timestamp: 2, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 8, height: 6),
            depthMap: nil, trackingQuality: .normal)

        var first = firstStream.makeAsyncIterator()
        var second = secondStream.makeAsyncIterator()
        let firstValue = await first.next()
        let secondValue = await second.next()
        XCTAssertEqual(firstValue?.id.sequence, 2)
        XCTAssertEqual(secondValue?.id.sequence, 2)
    }

    func testLifecycleStreamsRetainResetInterruptionAndFailureEventsPerSubscriber() async {
        let manager = ARSessionManager()
        let firstStream = manager.lifecycleEvents()
        let secondStream = manager.lifecycleEvents()

        manager.resetForTesting()
        manager.interruptionBeganForTesting()
        manager.interruptionEndedForTesting()
        manager.failureForTesting(description: "camera unavailable")

        var first = firstStream.makeAsyncIterator()
        var second = secondStream.makeAsyncIterator()
        var firstEvents: [ARSessionLifecycleEvent] = []
        var secondEvents: [ARSessionLifecycleEvent] = []
        for _ in 0..<4 {
            if let event = await first.next() { firstEvents.append(event) }
            if let event = await second.next() { secondEvents.append(event) }
        }
        let expected: [ARSessionLifecycleEvent] = [
            .reset(generation: 1), .interrupted(generation: 1),
            .interruptionEnded(generation: 1), .failed(generation: 1, description: "camera unavailable"),
        ]
        XCTAssertEqual(firstEvents, expected)
        XCTAssertEqual(secondEvents, expected)
        XCTAssertNil(manager.latestSnapshot)
        XCTAssertNil(manager.latestDepthMap)
        XCTAssertNil(manager.pose)
        XCTAssertEqual(manager.trackingQuality, .unavailable)
    }

    private func makeImage() -> CVPixelBuffer {
        var buffer: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 8, 6, kCVPixelFormatType_32BGRA, nil, &buffer)
        return buffer!
    }

    private func makeDepth(_ value: Float) -> CVPixelBuffer {
        var buffer: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 2, 2, kCVPixelFormatType_DepthFloat32, nil, &buffer)
        let result = buffer!
        CVPixelBufferLockBaseAddress(result, [])
        CVPixelBufferGetBaseAddress(result)!.assumingMemoryBound(to: Float.self)
            .initialize(repeating: value, count: 4)
        CVPixelBufferUnlockBaseAddress(result, [])
        return result
    }

    private func transform(x: Float, z: Float) -> simd_float4x4 {
        var value = matrix_identity_float4x4
        value.columns.3 = SIMD4<Float>(x, 0, z, 1)
        return value
    }
}
