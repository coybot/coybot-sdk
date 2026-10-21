import XCTest
import CoreVideo
import simd
import RoverNav
import ImageIO
@testable import PhroverKit

/// The bearing-only side of tolerant continuation: a direction that needs no depth.
@MainActor
final class FollowBearingObservationTests: XCTestCase {
    /// An upright phone: camera x (the landscape sensor's long axis) points up, camera y points
    /// to the world -x side, so a horizontal offset in the portrait image is a change of azimuth.
    private let upright = simd_float4x4(columns: (
        SIMD4<Float>(0, 1, 0, 0), SIMD4<Float>(-1, 0, 0, 0), SIMD4<Float>(0, 0, 1, 0), SIMD4<Float>(0, 0, 0, 1)))

    private func depth(_ value: Float = 3) throws -> CVPixelBuffer {
        var result: CVPixelBuffer?
        let attributes = [kCVPixelBufferBytesPerRowAlignmentKey: 128] as CFDictionary
        XCTAssertEqual(CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_DepthFloat32,
                                           attributes, &result), kCVReturnSuccess)
        let map = try XCTUnwrap(result)
        CVPixelBufferLockBaseAddress(map, [])
        defer { CVPixelBufferUnlockBaseAddress(map, []) }
        let base = try XCTUnwrap(CVPixelBufferGetBaseAddress(map))
        for y in 0..<20 { for x in 0..<20 {
            base.advanced(by: y * CVPixelBufferGetBytesPerRow(map)).assumingMemoryBound(to: Float.self)[x] = value
        } }
        return map
    }

    private func snapshot(transform: simd_float4x4 = simd_float4x4(1), intrinsics: simd_float3x3? = nil) throws -> ARFrameSnapshot {
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        return ARFrameSnapshot(id: .init(generation: 2, sequence: 7), timestamp: 12, image: try XCTUnwrap(image),
            cameraTransform: transform,
            cameraIntrinsics: intrinsics ?? simd_float3x3(columns: (
                SIMD3<Float>(10, 0, 0), SIMD3<Float>(0, 10, 0), SIMD3<Float>(10, 10, 1))),
            imageResolution: CGSize(width: 20, height: 20), depthMap: try depth(),
            pose: .init(position: .zero, yaw: 0), trackingQuality: .normal, depthConfidenceMap: nil)
    }

    func testCenterBearingPointsTheSameWayAsTheDepthProjectedPosition() throws {
        let frame = try snapshot(transform: upright)
        for box in [CGRect(x: 0.6, y: 0.3, width: 0.2, height: 0.5), CGRect(x: 0.2, y: 0.3, width: 0.2, height: 0.5)] {
            let projected = try XCTUnwrap(FollowPersonProjection.project(box: box, in: frame))
            let bearing = try XCTUnwrap(FollowPersonProjection.centerBearing(box: box, in: frame))
            XCTAssertEqual(bearing, atan2(projected.y, projected.x), accuracy: 1e-5,
                           "same ray, with or without a usable depth reading")
        }
    }

    func testAllMountOrientationsUseTheSameSensorRayForDepthAndBearing() throws {
        let original = try snapshot()
        let box = CGRect(x: 0.2, y: 0.2, width: 0.4, height: 0.5)
        for orientation in [CGImagePropertyOrientation.up, .right, .down, .left] {
            let frame = ARFrameSnapshot(id: original.id, timestamp: original.timestamp, image: original.image,
                cameraTransform: original.cameraTransform, cameraIntrinsics: original.cameraIntrinsics,
                imageResolution: original.imageResolution, depthMap: original.depthMap, pose: original.pose,
                trackingQuality: .normal, imageOrientation: orientation)
            let evidence = FollowPersonProjection.evaluate(box: box, detectorConfidence: 0.9,
                rawPersonID: 0, in: frame)
            let expectedSensor = ARSessionManager.sensorPixel(forVisionNormalizedPoint: CGPoint(x: box.midX, y: box.minY),
                imageSize: frame.imageResolution, orientation: orientation)
            XCTAssertEqual(evidence.sensorPixel, expectedSensor)
            let projected = try XCTUnwrap(evidence.position)
            let expectedPosition = ARSessionManager.unprojectPoint(CGPoint(x: box.midX, y: box.minY),
                imageSize: frame.imageResolution, orientation: orientation, intrinsics: frame.cameraIntrinsics,
                cameraTransform: frame.cameraTransform, depth: 3)
            XCTAssertEqual(projected.x, expectedPosition.x, accuracy: 1e-5)
            XCTAssertEqual(projected.y, expectedPosition.y, accuracy: 1e-5)
            let center = ARSessionManager.unprojectPoint(CGPoint(x: box.midX, y: box.midY),
                imageSize: frame.imageResolution, orientation: orientation, intrinsics: frame.cameraIntrinsics,
                cameraTransform: frame.cameraTransform, depth: 3)
            XCTAssertEqual(try XCTUnwrap(FollowPersonProjection.centerBearing(box: box, in: frame)),
                atan2(center.y, center.x), accuracy: 1e-5)
        }
    }

    func testBoxesOnOppositeSidesGiveOppositeBearingOffsets() throws {
        let frame = try snapshot(transform: upright)
        let right = try XCTUnwrap(FollowPersonProjection.centerBearing(box: CGRect(x: 0.6, y: 0.3, width: 0.2, height: 0.5), in: frame))
        let left = try XCTUnwrap(FollowPersonProjection.centerBearing(box: CGRect(x: 0.2, y: 0.3, width: 0.2, height: 0.5), in: frame))
        let centre = try XCTUnwrap(FollowPersonProjection.centerBearing(box: CGRect(x: 0.4, y: 0.3, width: 0.2, height: 0.5), in: frame))
        XCTAssertEqual(abs(right - centre), abs(left - centre), accuracy: 1e-5)
        XCTAssertNotEqual(right, left)
    }

    func testCenterBearingNeedsDepthNeitherNorAnUnclippedBox() throws {
        var frame = try snapshot(transform: upright)
        frame = ARFrameSnapshot(id: frame.id, timestamp: frame.timestamp, image: frame.image,
            cameraTransform: frame.cameraTransform, cameraIntrinsics: frame.cameraIntrinsics,
            imageResolution: frame.imageResolution, depthMap: nil, pose: frame.pose,
            trackingQuality: .normal, depthConfidenceMap: nil)
        XCTAssertNotNil(FollowPersonProjection.centerBearing(box: CGRect(x: 0.4, y: 0.3, width: 0.2, height: 0.5), in: frame))
        // Clipped by the frame edge: the visible part still has a centre.
        XCTAssertNotNil(FollowPersonProjection.centerBearing(box: CGRect(x: 0.8, y: -0.2, width: 0.4, height: 0.6), in: frame))
    }

    func testCenterBearingRejectsUnusableInput() throws {
        let frame = try snapshot(transform: upright)
        XCTAssertNil(FollowPersonProjection.centerBearing(box: CGRect(x: 2, y: 2, width: 0.2, height: 0.2), in: frame))
        XCTAssertNil(FollowPersonProjection.centerBearing(box: CGRect(x: 0.4, y: 0.4, width: 0, height: 0.2), in: frame))
        let bad = try snapshot(transform: upright, intrinsics: simd_float3x3(columns: (
            SIMD3<Float>(0, 0, 0), SIMD3<Float>(0, 10, 0), SIMD3<Float>(10, 10, 1))))
        XCTAssertNil(FollowPersonProjection.centerBearing(box: CGRect(x: 0.4, y: 0.3, width: 0.2, height: 0.5), in: bad))
    }

    func testRejectedPeopleBecomePartialObservationsAndAcceptedOnesDoNot() throws {
        let frame = try snapshot(transform: upright)
        func joint(_ x: Double, _ y: Double) -> PersonBodyVerifier.Joint { .init(location: .init(x: x, y: y), confidence: 0.9) }
        let goodBox = CGRect(x: 0.6, y: 0.2, width: 0.2, height: 0.6)
        let hiddenBox = CGRect(x: 0.2, y: 0.2, width: 0.2, height: 0.6)
        let body = PersonBodyVerifier.Body(leftShoulder: joint(0.65, 0.7), rightShoulder: joint(0.75, 0.7),
            leftHip: joint(0.66, 0.4), rightHip: joint(0.74, 0.4))
        let detections: [Detector.Detection] = [
            .init(label: "person", confidence: 0.95, boundingBox: goodBox),
            .init(label: "person", confidence: 0.55, boundingBox: hiddenBox),
            .init(label: "person", confidence: 0.10, boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.1, height: 0.3))]
        let verification = detections.enumerated().map { index, detection in
            PersonBodyVerifier.verify(box: detection.boundingBox, rawPersonID: index, bodies: index == 0 ? [body] : [])
        }
        let batch = ARFollowMePerceptionSource.batch(from: frame, detections: detections, personVerification: verification)
        XCTAssertEqual(batch.people.compactMap(\.rawPersonID), [0], "only the verified person is a full observation")
        XCTAssertEqual(batch.partialPeople.compactMap(\.rawPersonID), [1],
                       "the unverified person is a bearing; the 0.10-confidence box is noise")
        let partial = try XCTUnwrap(batch.partialPeople.first)
        XCTAssertEqual(partial.bearing, try XCTUnwrap(FollowPersonProjection.centerBearing(box: hiddenBox, in: frame)), accuracy: 1e-9)
        XCTAssertEqual(partial.frameID, batch.frameID)
    }

    func testNoPartialObservationsWhenInferenceDidNotRun() throws {
        let frame = try snapshot(transform: upright)
        let batch = ARFollowMePerceptionSource.batch(from: frame, detections: [], inferenceStatus: .failed)
        XCTAssertTrue(batch.partialPeople.isEmpty)
    }

    func testLowConfidenceVerifiedPersonRemainsAvailableAsBearing() throws {
        let frame = try snapshot(transform: upright)
        func joint(_ x: Double, _ y: Double) -> PersonBodyVerifier.Joint {
            .init(location: .init(x: x, y: y), confidence: 0.9)
        }
        let box = CGRect(x: 0.6, y: 0.2, width: 0.2, height: 0.6)
        let body = PersonBodyVerifier.Body(leftShoulder: joint(0.65, 0.7), rightShoulder: joint(0.75, 0.7),
            leftHip: joint(0.66, 0.4), rightHip: joint(0.74, 0.4))
        let decision = PersonBodyVerifier.verify(box: box, rawPersonID: 0, bodies: [body])
        XCTAssertTrue(decision.accepted)
        let batch = ARFollowMePerceptionSource.batch(from: frame,
            detections: [.init(label: "person", confidence: 0.35, boundingBox: box)],
            personVerification: [decision])
        XCTAssertNil(FollowTargetTracker().selectInitial(batch.people, now: frame.timestamp))
        XCTAssertEqual(batch.partialPeople.compactMap(\.rawPersonID), [0],
            "A person below the 0.50 full-observation threshold but above 0.25 should retain bearing evidence")
    }

}
