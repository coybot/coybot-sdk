import XCTest
import CoreVideo
import simd
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowPersonProjectionTests: XCTestCase {
    func testVerifiedTorsoDepthRemainsGroundedWhenWalkingFeetPatchMixesSurfaces() throws {
        let map = try buffer(value: 3)
        patch(map, values: (0..<25).map { $0.isMultiple(of: 2) ? 1 : 5 })
        let snapshot = try snapshot(map)
        let detection = Detector.Detection(label: "person", confidence: 0.99,
            boundingBox: CGRect(x: 0.2, y: 0.2, width: 0.6, height: 0.6))
        func joint(_ x: Double, _ y: Double) -> PersonBodyVerifier.Joint {
            .init(location: .init(x: x, y: y), confidence: 0.9)
        }
        let body = PersonBodyVerifier.Body(leftShoulder: joint(0.4, 0.7), rightShoulder: joint(0.6, 0.7),
            leftHip: joint(0.43, 0.4), rightHip: joint(0.57, 0.4))
        let verification = PersonBodyVerifier.verify(box: detection.boundingBox, rawPersonID: 0, bodies: [body])
        XCTAssertTrue(verification.accepted)
        XCTAssertTrue(ARFollowMePerceptionSource.batch(from: snapshot, detections: [detection]).people.isEmpty,
            "The mixed feet patch must remain rejected for unverified legacy input")
        let batch = ARFollowMePerceptionSource.batch(from: snapshot, detections: [detection], personVerification: [verification])
        let person = try XCTUnwrap(batch.people.first, "A coherent independently verified torso patch avoids the moving leg/floor boundary")
        XCTAssertEqual(person.position.x, -0.3, accuracy: 1e-5)
        XCTAssertEqual(person.position.y, -3, accuracy: 1e-5)
        XCTAssertNotNil(FollowTargetTracker(configuration: .init()).selectInitial(batch.people, now: 12))
        XCTAssertEqual(batch.perceptionDiagnostics?.candidates.first?.depthAnchorKind, "verified_torso")
        let invalidTorso = try buffer(value: 3)
        CVPixelBufferLockBaseAddress(invalidTorso, [])
        let base = CVPixelBufferGetBaseAddress(invalidTorso)!.assumingMemoryBound(to: Float.self)
        for row in 8...12 { for col in 7...11 {
            base[row * CVPixelBufferGetBytesPerRow(invalidTorso) / 4 + col] = (row + col).isMultiple(of: 2) ? 1 : 5
        } }
        CVPixelBufferUnlockBaseAddress(invalidTorso, [])
        let rejected = ARFollowMePerceptionSource.batch(from: try self.snapshot(invalidTorso), detections: [detection],
            personVerification: [verification])
        XCTAssertTrue(rejected.people.isEmpty, "Torso sampling must still reject inconsistent depth rather than falling back to background")
        XCTAssertEqual(rejected.perceptionDiagnostics?.candidates.first?.rejection, .inconsistentDepth)
    }

    func testVerifiedTorsoKeepsTrackAcrossTinyImageEdgeOverrun() throws {
        func joint(_ x: Double, _ y: Double) -> PersonBodyVerifier.Joint {
            .init(location: .init(x: x, y: y), confidence: 0.9)
        }
        let torso = PersonBodyVerifier.Body(leftShoulder: joint(0.80, 0.85), rightShoulder: joint(0.94, 0.85),
            leftHip: joint(0.81, 0.50), rightHip: joint(0.92, 0.50))
        let tracker = FollowTargetTracker(configuration: .init())
        var previous: FollowPersonObservation?
        for (sequence, right) in [(UInt64(7), 0.9995), (8, 1.0005), (9, 1.0)] {
            let box = CGRect(x: 0.75, y: 0.24, width: right - 0.75, height: 0.75)
            let verification = PersonBodyVerifier.verify(box: box, rawPersonID: 0, bodies: [torso])
            let batch = ARFollowMePerceptionSource.batch(from: try snapshot(buffer(), sequence: sequence), detections: [
                .init(label: "person", confidence: 0.99, boundingBox: box)], personVerification: [verification])
            let person = try XCTUnwrap(batch.people.first, "Independent torso evidence and valid feet depth must survive tiny edge jitter")
            if let previous {
                guard case .matched = tracker.continueTrackEvaluated(batch.people, previous: previous,
                    predictedPosition: previous.position, now: 12, frameID: batch.frameID).decision else {
                    return XCTFail("Normalization must survive the tracker gate too")
                }
            } else { XCTAssertNotNil(tracker.selectInitial(batch.people, now: 12)) }
            previous = person
        }
        for bad in [CGRect(x: 0.75, y: 0.24, width: 0.28, height: 0.75),
                    CGRect(x: 0.75, y: -0.0005, width: 0.2505, height: 0.99)] {
            let decision = PersonBodyVerifier.verify(box: bad, rawPersonID: 0, bodies: [torso])
            let batch = ARFollowMePerceptionSource.batch(from: try snapshot(buffer()), detections: [
                .init(label: "person", confidence: 1, boundingBox: bad)], personVerification: [decision])
            XCTAssertTrue(batch.people.isEmpty, "Material clipping or missing feet must not become a grounded target")
        }
        XCTAssertFalse(PersonBodyVerifier.verify(box: CGRect(x: 0.75, y: 0.24, width: 0.2505, height: 0.75),
            rawPersonID: 0, bodies: []).accepted)
    }

    let box = CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6)

    func buffer(width: Int = 20, height: Int = 20, format: OSType = kCVPixelFormatType_DepthFloat32,
                value: Float = 3) throws -> CVPixelBuffer {
        var result: CVPixelBuffer?
        let attributes = [kCVPixelBufferBytesPerRowAlignmentKey: 128] as CFDictionary
        XCTAssertEqual(CVPixelBufferCreate(kCFAllocatorDefault, width, height, format, attributes, &result), kCVReturnSuccess)
        let map = try XCTUnwrap(result)
        CVPixelBufferLockBaseAddress(map, [])
        defer { CVPixelBufferUnlockBaseAddress(map, []) }
        let base = try XCTUnwrap(CVPixelBufferGetBaseAddress(map))
        for y in 0..<height {
            for x in 0..<width {
                if format == kCVPixelFormatType_DepthFloat32 {
                    base.advanced(by: y * CVPixelBufferGetBytesPerRow(map)).assumingMemoryBound(to: Float.self)[x] = value
                } else if format == kCVPixelFormatType_OneComponent8 {
                    base.advanced(by: y * CVPixelBufferGetBytesPerRow(map)).assumingMemoryBound(to: UInt8.self)[x] = UInt8(value)
                }
            }
        }
        return map
    }

    func patch(_ map: CVPixelBuffer, values: [Float]) {
        CVPixelBufferLockBaseAddress(map, [])
        defer { CVPixelBufferUnlockBaseAddress(map, []) }
        let base = CVPixelBufferGetBaseAddress(map)!
        for (index, value) in values.enumerated() {
            let x = 14 + index % 5, y = 8 + index / 5
            base.advanced(by: y * CVPixelBufferGetBytesPerRow(map)).assumingMemoryBound(to: Float.self)[x] = value
        }
    }

    func patchConfidence(_ map: CVPixelBuffer, values: [UInt8]) {
        CVPixelBufferLockBaseAddress(map, [])
        defer { CVPixelBufferUnlockBaseAddress(map, []) }
        let base = CVPixelBufferGetBaseAddress(map)!
        for (index, value) in values.enumerated() {
            let x = 14 + index % 5, y = 8 + index / 5
            base.advanced(by: y * CVPixelBufferGetBytesPerRow(map)).assumingMemoryBound(to: UInt8.self)[x] = value
        }
    }

    func snapshot(_ depth: CVPixelBuffer?, imageSize: CGSize = CGSize(width: 20, height: 20),
                  intrinsics: simd_float3x3? = nil, transform: simd_float4x4 = simd_float4x4(1),
                  confidence: CVPixelBuffer? = nil, pose: Pose2D = .init(position: .zero, yaw: 0),
                  sequence: UInt64 = 7, image: CVPixelBuffer? = nil) throws -> ARFrameSnapshot {
        ARFrameSnapshot(id: .init(generation: 2, sequence: sequence), timestamp: 12,
            image: try image ?? buffer(format: kCVPixelFormatType_32BGRA), cameraTransform: transform,
            cameraIntrinsics: intrinsics ?? simd_float3x3(columns: (
                SIMD3<Float>(10, 0, 0), SIMD3<Float>(0, 10, 0), SIMD3<Float>(10, 10, 1))),
            imageResolution: imageSize, depthMap: depth, pose: pose, trackingQuality: .normal,
            depthConfidenceMap: confidence)
    }

    func batch(_ snapshot: ARFrameSnapshot, box: CGRect? = nil) -> FollowFrameBatch {
        ARFollowMePerceptionSource.batch(from: snapshot, detections: [
            .init(label: "person", confidence: 0.9, boundingBox: box ?? self.box)])
    }

    func testBoxesMustHavePositiveFiniteAreaStrictlyInsideEveryEdge() throws {
        let frame = try snapshot(buffer())
        let boxes = [CGRect(x: 0, y: 0.2, width: 0.5, height: 0.6),
                     CGRect(x: 0.4, y: 0, width: 0.2, height: 0.6),
                     CGRect(x: 0.5, y: 0.2, width: 0.5, height: 0.6),
                     CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.8),
                     CGRect(x: -0.1, y: 0.2, width: 0.5, height: 0.6),
                     CGRect(x: 0.4, y: 0.2, width: 0, height: 0.6),
                     CGRect(x: 0.6, y: 0.2, width: -0.2, height: 0.6),
                     CGRect(x: CGFloat.nan, y: 0.2, width: 0.2, height: 0.6)]
        for box in boxes { XCTAssertTrue(batch(frame, box: box).people.isEmpty, "Rejected box: \(box)") }
    }

    func testUnsupportedDepthFormatRejectsRatherThanReadingAsFloat() throws {
        XCTAssertTrue(batch(try snapshot(buffer(format: kCVPixelFormatType_32BGRA))).people.isEmpty)
    }

    func testExactlyFiveFiniteDepthsAboveLowerBoundUseOddMedian() throws {
        let map = try buffer()
        let invalid: [Float] = [.nan, .infinity, -.infinity, 0.05, -1]
        patch(map, values: [2.95, 3, 3.02, 3.03, 3.04] + Array(repeating: invalid, count: 4).flatMap { $0 })
        let person = try XCTUnwrap(batch(try snapshot(map)).people.first)
        XCTAssertEqual(person.position.y, -3.02, accuracy: 0.00001)
    }

    func testFourValidDepthsCannotProject() throws {
        let map = try buffer()
        patch(map, values: Array(repeating: 3, count: 4) + Array(repeating: .nan, count: 21))
        XCTAssertTrue(batch(try snapshot(map)).people.isEmpty)
    }

    func testEvenMedianIsMeanOfMiddleTwoNotUpperMiddle() throws {
        let map = try buffer()
        patch(map, values: [2.94, 2.96, 2.98, 3.02, 3.04, 3.06] + Array(repeating: .nan, count: 19))
        XCTAssertEqual(try XCTUnwrap(batch(try snapshot(map)).people.first).position.y, -3, accuracy: 0.00001)
    }

    func testMedianAbsoluteDeviationAbovePointOneRejectsMixedPatch() throws {
        let map = try buffer()
        patch(map, values: Array(repeating: 2, count: 12) + [3] + Array(repeating: 4, count: 12))
        XCTAssertTrue(batch(try snapshot(map)).people.isEmpty)
    }

    func testInliersRequireCeilingSixtyPercentEvenWhenMADIsZero() throws {
        let map = try buffer()
        patch(map, values: Array(repeating: 3, count: 14) + Array(repeating: 6, count: 11))
        XCTAssertTrue(batch(try snapshot(map)).people.isEmpty)
    }

    func testLowAndInvalidConfidenceCannotSupplyValidSamples() throws {
        for level: Float in [0, 3, 255] {
            let confidence = try buffer(format: kCVPixelFormatType_OneComponent8, value: level)
            XCTAssertTrue(batch(try snapshot(buffer(), confidence: confidence)).people.isEmpty)
        }
    }

    func testMalformedConfidenceIsRejectedInsteadOfIgnoredOrReinterpreted() throws {
        let maps = [try buffer(width: 24, format: kCVPixelFormatType_OneComponent8, value: 1),
                    try buffer(value: Float(bitPattern: 0x01010101))]
        for confidence in maps { XCTAssertTrue(batch(try snapshot(buffer(), confidence: confidence)).people.isEmpty) }
    }

    func testCalibrationRequiresAllFiniteElementsAndPositiveFocalLengths() throws {
        var negative = simd_float3x3(1); negative[0][0] = -10
        var zero = simd_float3x3(1); zero[1][1] = 0
        var nonfinite = simd_float3x3(1); nonfinite[0][2] = .nan
        for calibration in [negative, zero, nonfinite] {
            XCTAssertTrue(batch(try snapshot(buffer(), intrinsics: calibration)).people.isEmpty)
        }
        var transform = simd_float4x4(1); transform[3][1] = .infinity
        XCTAssertTrue(batch(try snapshot(buffer(), transform: transform)).people.isEmpty)
    }

    func testBatchRetainsRejectedRawPeopleAndMeasuredProjectionFacts() throws {
        let frame = try snapshot(buffer())
        let output = ARFollowMePerceptionSource.batch(from: frame, detections: [
            .init(label: "chair", confidence: 0.7, boundingBox: box),
            .init(label: "person", confidence: 0.9, boundingBox: box),
            .init(label: "person", confidence: 0.8, boundingBox: CGRect(x: 0, y: 0.2, width: 0.5, height: 0.6))])
        let evidence = try XCTUnwrap(output.perceptionDiagnostics)
        XCTAssertEqual(evidence.rawDetectorCount, 3)
        XCTAssertEqual(evidence.rawPersonCount, 2)
        XCTAssertEqual(evidence.projectionAttemptedCount, 2)
        XCTAssertEqual(evidence.projectionAcceptedCount, 1)
        XCTAssertEqual(evidence.projectionRejectedCount, 1)
        XCTAssertEqual(evidence.projectedPersonCount, 1)
        XCTAssertEqual(evidence.candidates.map(\.rawPersonID), [0, 1])
        let accepted = evidence.candidates[0]
        XCTAssertEqual(accepted.feet, CGPoint(x: 0.5, y: 0.2))
        XCTAssertEqual(accepted.sensorPixel, CGPoint(x: 16, y: 10))
        XCTAssertEqual(accepted.depthCenter, CGPoint(x: 16, y: 10))
        XCTAssertEqual(accepted.validSampleCount, 25)
        XCTAssertEqual(accepted.invalidDepthCount, 0)
        XCTAssertEqual(accepted.lowConfidenceCount, 0)
        XCTAssertEqual(accepted.medianDepth, 3)
        XCTAssertEqual(accepted.medianAbsoluteDeviation, 0)
        XCTAssertEqual(accepted.inlierCount, 25)
        XCTAssertEqual(accepted.pairedPose, frame.pose)
        XCTAssertEqual(accepted.confidenceAvailability, .unavailable)
        XCTAssertEqual(evidence.candidates[1].rejection, .clippedBox)
        XCTAssertEqual(evidence.candidates[1].clippedLeft, true)
        XCTAssertNil(evidence.candidates[1].validSampleCount)
    }

    func testNonfiniteWorldProjectionHasTerminalReasonAndNoGeometry() throws {
        var transform = simd_float4x4(1)
        transform[0][0] = .greatestFiniteMagnitude
        let evidence = try XCTUnwrap(batch(try snapshot(buffer(), transform: transform)).perceptionDiagnostics).candidates[0]
        XCTAssertEqual(evidence.rejection, .nonfiniteProjection)
        XCTAssertNil(evidence.position)
        XCTAssertNil(evidence.groundRange)
        XCTAssertEqual(evidence.validSampleCount, 25)
        XCTAssertEqual(evidence.medianDepth, 3)
    }

    func testFullWindowRejectsAllBordersAndUndersizedMapsWithoutFallback() throws {
        let frame = try snapshot(buffer())
        let boxes = [CGRect(x: 0.4, y: 0.05, width: 0.2, height: 0.6),
                     CGRect(x: 0.4, y: 0.9, width: 0.2, height: 0.05),
                     CGRect(x: 0.05, y: 0.2, width: 0.1, height: 0.6),
                     CGRect(x: 0.86, y: 0.2, width: 0.1, height: 0.6)]
        for box in boxes {
            let output = batch(frame, box: box)
            XCTAssertTrue(output.people.isEmpty)
            let facts = try XCTUnwrap(output.perceptionDiagnostics).candidates[0]
            XCTAssertEqual(facts.rejection, .clippedDepthWindow)
            XCTAssertNil(facts.validSampleCount)
        }
        XCTAssertEqual(try XCTUnwrap(batch(try snapshot(buffer(width: 4, height: 4))).perceptionDiagnostics)
            .candidates[0].rejection, .clippedDepthWindow)
        // The last complete window at the map's right edge remains valid.
        XCTAssertEqual(batch(frame, box: CGRect(x: 0.4, y: 0.125, width: 0.2, height: 0.6)).people.count, 1)
    }

    func testMediumHighConfidenceUsesAlignedPaddedRowsAndExclusiveSampleCounts() throws {
        let depth = try buffer()
        patch(depth, values: Array(repeating: 3, count: 5) + Array(repeating: .nan, count: 10) + Array(repeating: 3, count: 10))
        let confidence = try buffer(format: kCVPixelFormatType_OneComponent8, value: 0)
        XCTAssertGreaterThan(CVPixelBufferGetBytesPerRow(confidence), 20)
        patchConfidence(confidence, values: [1, 2, 1, 2, 1] + Array(repeating: 0, count: 10) + Array(repeating: 255, count: 10))
        let output = batch(try snapshot(depth, confidence: confidence))
        XCTAssertEqual(output.people.count, 1)
        let facts = try XCTUnwrap(output.perceptionDiagnostics).candidates[0]
        XCTAssertEqual(facts.confidenceAvailability, .available)
        XCTAssertEqual(facts.validSampleCount, 5)
        XCTAssertEqual(facts.invalidDepthCount, 10, "Invalid depth takes priority for doubly-invalid pixels")
        XCTAssertEqual(facts.lowConfidenceCount, 10)
        XCTAssertEqual(facts.medianDepth, 3)
    }

    func testInverseRightMappingUsesActualDimensionsAndOriginalUnroundedRay() throws {
        let intrinsics = simd_float3x3(columns: (
            SIMD3<Float>(50, 0, 0), SIMD3<Float>(0, 40, 0), SIMD3<Float>(50, 40, 1)))
        let frame = try snapshot(buffer(), imageSize: CGSize(width: 100, height: 80), intrinsics: intrinsics,
                                 image: buffer(width: 100, height: 80, format: kCVPixelFormatType_32BGRA))
        let output = batch(frame, box: CGRect(x: 0.4125, y: 0.2125, width: 0.2, height: 0.6))
        let person = try XCTUnwrap(output.people.first)
        XCTAssertEqual(person.position.x, 1.725, accuracy: 0.00001)
        XCTAssertEqual(person.position.y, -3, accuracy: 0.00001)
        let facts = try XCTUnwrap(output.perceptionDiagnostics).candidates[0]
        XCTAssertEqual(try XCTUnwrap(facts.sensorPixel).x, 78.75, accuracy: 0.00001)
        XCTAssertEqual(try XCTUnwrap(facts.sensorPixel).y, 39, accuracy: 0.00001)
        XCTAssertEqual(try XCTUnwrap(facts.depthPixel).x, 15.75, accuracy: 0.00001)
        XCTAssertEqual(try XCTUnwrap(facts.depthPixel).y, 9.75, accuracy: 0.00001)
        XCTAssertEqual(facts.depthCenter, CGPoint(x: 15, y: 9))
        XCTAssertEqual(facts.depthSize, CGSize(width: 20, height: 20))
    }

    func testCardinalAndObliqueTransformsKeepWorldXZAndAxialDepthDistinctFromRange() throws {
        let cases: [(Float, Double, Double, Double)] = [
            (0, -.pi / 2, 11.8, 17),
            (.pi / 2, -.pi, 7, 18.2),
            (.pi, .pi / 2, 8.2, 23),
            (-.pi / 2, 0, 13, 21.8),
            (.pi / 4, -3 * .pi / 4, 9.151471863, 16.60588745)]
        for (angle, yaw, x, z) in cases {
            var transform = simd_float4x4(simd_quatf(angle: angle, axis: SIMD3<Float>(0, 1, 0)))
            transform[3] = SIMD4<Float>(10, 4, 20, 1)
            let pose = Pose2D(position: Vec2(10, 20), yaw: yaw)
            let facts = FollowPersonProjection.evaluate(box: box, detectorConfidence: 0.9, rawPersonID: 0,
                in: try snapshot(buffer(), transform: transform, pose: pose))
            let position = try XCTUnwrap(facts.position)
            XCTAssertEqual(position.x, x, accuracy: 0.00001)
            XCTAssertEqual(position.y, z, accuracy: 0.00001)
            XCTAssertEqual(facts.medianDepth, 3)
            XCTAssertEqual(try XCTUnwrap(facts.groundRange), 3.498571137, accuracy: 0.00001)
            XCTAssertEqual(try XCTUnwrap(facts.headingError), 0.5404195, accuracy: 0.00001)
        }
    }

    func testHeadingWrapsAcrossPositiveAndNegativePi() throws {
        let cases: [(Float, Double, Double)] = [
            (0.03, -.pi + 0.01, -0.0199996667), (-0.03, .pi - 0.01, 0.0199996667)]
        for (worldZ, yaw, expected) in cases {
            var transform = simd_float4x4(1)
            transform[3] = SIMD4<Float>(-4.8, 0, 3 + worldZ, 1)
            let facts = FollowPersonProjection.evaluate(box: box, detectorConfidence: 0.9, rawPersonID: 0,
                in: try snapshot(buffer(), transform: transform, pose: .init(position: .zero, yaw: yaw)))
            XCTAssertEqual(try XCTUnwrap(facts.headingError), expected, accuracy: 0.00001)
            XCTAssertEqual(try XCTUnwrap(facts.groundRange), 3.000149996, accuracy: 0.00001)
        }
    }

    func testBatchToRealTrackerRetainsCorruptPixelPersonButRejectsGenuineWorldJump() throws {
        let tracker = FollowTargetTracker()
        let previous = try XCTUnwrap(tracker.selectInitial(batch(try snapshot(buffer())).people, now: 12))
        let corrupted = try buffer()
        var values = Array(repeating: Float(3), count: 25); values[12] = 30
        patch(corrupted, values: values)
        let stable = batch(try snapshot(corrupted, sequence: 8))
        guard case .matched(let person) = tracker.continueTrack(stable.people, previous: previous,
            predictedPosition: previous.position, now: 12) else { return XCTFail("Spatially stable median must retain association") }
        XCTAssertEqual(person.frameID.sequence, 8)
        XCTAssertEqual(person.rawPersonID, 0)
        let jump = batch(try snapshot(buffer(value: 4), sequence: 9))
        XCTAssertEqual(jump.people.count, 1, "Projection succeeds; tracker independently rejects the jump")
        guard case .lost = tracker.continueTrack(jump.people, previous: person,
            predictedPosition: person.position, now: 12) else { return XCTFail("The unchanged 0.75 m gate must reject the jump") }
    }

    func testRetainedSnapshotUsesItsDepthConfidenceTransformAndSourceAfterNewFrame() throws {
        let manager = ARSessionManager()
        let original = try snapshot(buffer(), confidence: buffer(format: kCVPixelFormatType_OneComponent8, value: 1))
        manager.ingestForTesting(image: original.image, timestamp: 12, cameraTransform: original.cameraTransform,
            intrinsics: original.cameraIntrinsics, imageResolution: original.imageResolution,
            depthMap: original.depthMap, trackingQuality: .normal, depthConfidenceMap: original.depthConfidenceMap,
            depthSource: .smoothedSceneDepth)
        let retained = try XCTUnwrap(manager.latestSnapshot)
        var moved = simd_float4x4(1); moved[3][0] = 50
        manager.ingestForTesting(image: original.image, timestamp: 13, cameraTransform: moved,
            intrinsics: original.cameraIntrinsics, imageResolution: original.imageResolution,
            depthMap: try buffer(value: 8), trackingQuality: .normal,
            depthConfidenceMap: try buffer(format: kCVPixelFormatType_OneComponent8, value: 0), depthSource: .sceneDepth)
        let output = batch(retained)
        let person = try XCTUnwrap(output.people.first)
        XCTAssertEqual(person.position.x, 1.8, accuracy: 0.00001)
        XCTAssertEqual(person.position.y, -3, accuracy: 0.00001)
        XCTAssertEqual(person.frameID, retained.id)
        XCTAssertEqual(person.timestamp, 12)
        XCTAssertEqual(person.pose, retained.pose)
        XCTAssertEqual(output.perceptionDiagnostics?.candidates.first?.depthSource, .smoothedSceneDepth)
        XCTAssertTrue(batch(try XCTUnwrap(manager.latestSnapshot)).people.isEmpty)
    }

    func testSevenValidSamplesRequireFiveInliersRoundedUp() throws {
        let map = try buffer()
        patch(map, values: Array(repeating: 3, count: 4) + Array(repeating: 6, count: 3) + Array(repeating: .nan, count: 18))
        let rejected = try XCTUnwrap(batch(try snapshot(map)).perceptionDiagnostics).candidates[0]
        XCTAssertEqual(rejected.requiredInlierCount, 5)
        XCTAssertEqual(rejected.inlierCount, 4)
        XCTAssertEqual(rejected.medianAbsoluteDeviation, 0)
        XCTAssertEqual(rejected.rejection, .inconsistentDepth)
        patch(map, values: Array(repeating: 3, count: 5) + Array(repeating: 6, count: 2) + Array(repeating: .nan, count: 18))
        XCTAssertEqual(batch(try snapshot(map)).people.count, 1)
    }

    func testMADBoundaryUsesInclusivePointOneWithoutRoundingOrRelaxation() throws {
        let map = try buffer()
        // Nearest Float32 patches on either side of 0.10 m; no decimal rounding tolerance.
        patch(map, values: Array(repeating: 0.15, count: 12) + [0.25] + Array(repeating: 0.35, count: 12))
        let accepted = try XCTUnwrap(batch(try snapshot(map)).perceptionDiagnostics).candidates[0]
        XCTAssertNil(accepted.rejection)
        XCTAssertLessThanOrEqual(try XCTUnwrap(accepted.medianAbsoluteDeviation), 0.10)
        patch(map, values: Array(repeating: Float(0.15).nextDown, count: 12) + [0.25]
            + Array(repeating: Float(0.35).nextUp, count: 12))
        let rejected = try XCTUnwrap(batch(try snapshot(map)).perceptionDiagnostics).candidates[0]
        XCTAssertGreaterThan(try XCTUnwrap(rejected.medianAbsoluteDeviation), 0.10)
        XCTAssertEqual(rejected.rejection, .inconsistentDepth)
    }

    func testInlierPointTwoBoundaryRetainsFifteenOfTwentyFiveSamples() throws {
        let map = try buffer()
        patch(map, values: Array(repeating: 0.25, count: 13) + Array(repeating: Float(0.45), count: 2)
            + Array(repeating: 0.75, count: 10))
        let accepted = try XCTUnwrap(batch(try snapshot(map)).perceptionDiagnostics).candidates[0]
        XCTAssertNil(accepted.rejection)
        XCTAssertEqual(accepted.inlierCount, 15)
        XCTAssertEqual(accepted.requiredInlierCount, 15)
        patch(map, values: Array(repeating: 0.25, count: 13) + Array(repeating: Float(0.45).nextUp, count: 2)
            + Array(repeating: 0.75, count: 10))
        let rejected = try XCTUnwrap(batch(try snapshot(map)).perceptionDiagnostics).candidates[0]
        XCTAssertEqual(rejected.inlierCount, 13)
        XCTAssertEqual(rejected.rejection, .inconsistentDepth)
    }

    func testEvidenceRetainsActualBufferFormatsOriginalCameraRayAndFullWorldPoint() throws {
        let facts = FollowPersonProjection.evaluate(box: box, detectorConfidence: 0.9, rawPersonID: 0,
            in: try snapshot(buffer(), confidence: buffer(format: kCVPixelFormatType_OneComponent8, value: 2)))
        XCTAssertEqual(facts.depthPixelFormat, kCVPixelFormatType_DepthFloat32)
        XCTAssertEqual(facts.confidencePixelFormat, kCVPixelFormatType_OneComponent8)
        XCTAssertEqual(try XCTUnwrap(facts.cameraRay).x, 0.6, accuracy: 0.00001)
        XCTAssertEqual(try XCTUnwrap(facts.cameraRay).y, 0, accuracy: 0.00001)
        XCTAssertEqual(try XCTUnwrap(facts.cameraRay).z, -1, accuracy: 0.00001)
        XCTAssertEqual(try XCTUnwrap(facts.worldPoint).x, 1.8, accuracy: 0.00001)
        XCTAssertEqual(try XCTUnwrap(facts.worldPoint).y, 0, accuracy: 0.00001)
        XCTAssertEqual(try XCTUnwrap(facts.worldPoint).z, -3, accuracy: 0.00001)
    }

    func testUnavailableAndEntirelyInvalidDepthKeepMeasuredCountsAndFirstReason() throws {
        let unavailable = try XCTUnwrap(batch(try snapshot(nil)).perceptionDiagnostics).candidates[0]
        XCTAssertEqual(unavailable.rejection, .depthUnavailable)
        XCTAssertNil(unavailable.validSampleCount)
        let empty = try XCTUnwrap(batch(try snapshot(buffer(value: .nan))).perceptionDiagnostics).candidates[0]
        XCTAssertEqual(empty.rejection, .insufficientValidDepth)
        XCTAssertEqual(empty.validSampleCount, 0)
        XCTAssertEqual(empty.invalidDepthCount, 25)
        XCTAssertEqual(empty.lowConfidenceCount, 0)
        XCTAssertNil(empty.medianDepth)
        let clipped = try XCTUnwrap(batch(try snapshot(nil), box: CGRect(x: 0, y: 0.2, width: 0.2, height: 0.6))
            .perceptionDiagnostics).candidates[0]
        XCTAssertEqual(clipped.rejection, .clippedBox, "Do not replace the first terminal reason with a later one")
    }

    func testInvalidImageDimensionsRejectBeforeIntegerMapping() throws {
        for size in [CGSize(width: 0, height: 20), CGSize(width: 20, height: -1),
                     CGSize(width: CGFloat.nan, height: 20), CGSize(width: 20, height: CGFloat.infinity)] {
            let output = batch(try snapshot(buffer(), imageSize: size))
            XCTAssertTrue(output.people.isEmpty)
            let facts = try XCTUnwrap(output.perceptionDiagnostics).candidates[0]
            XCTAssertEqual(facts.rejection, .invalidCalibration)
            XCTAssertNil(facts.depthCenter)
            XCTAssertNil(facts.validSampleCount)
        }
    }

    func testBatchUsesNeighborhoodMedianDespiteCorruptCenterWithPaddedStride() throws {
        let map = try buffer()
        XCTAssertGreaterThan(CVPixelBufferGetBytesPerRow(map), 20 * 4)
        var values = Array(repeating: Float(3), count: 25)
        values[12] = 30
        patch(map, values: values)
        let frame = try snapshot(map)
        let person = try XCTUnwrap(batch(frame).people.first)
        XCTAssertEqual(person.position.x, 1.8, accuracy: 0.00001)
        XCTAssertEqual(person.position.y, -3, accuracy: 0.00001)
        // Coybot generic consumers retain their original near-quartile neighborhood
        // sampling, so an isolated distant center reading is rejected there too.
        XCTAssertEqual(try XCTUnwrap(ARSessionManager.unproject(normalizedPoint: CGPoint(x: 0.5, y: 0.2), in: frame)).y, -3)
    }
}
