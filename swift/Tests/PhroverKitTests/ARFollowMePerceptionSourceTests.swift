import XCTest
import CoreVideo
import simd
import RoverNav
import ImageIO
@testable import PhroverKit

@MainActor
final class ARFollowMePerceptionSourceTests: XCTestCase {
    func testNewestInferenceSelectionCannotDiscardQueuedDepthOrTrackingLoss() throws {
        var image: CVPixelBuffer?
        var depth: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_DepthFloat32, nil, &depth)
        let buffer = try XCTUnwrap(image)
        let depthMap = try XCTUnwrap(depth)
        func snapshot(_ sequence: UInt64, depth: Bool = true, tracking: ARTrackingQuality = .normal) -> ARFrameSnapshot {
            .init(id: .init(generation: 1, sequence: sequence), timestamp: Double(sequence), image: buffer,
                cameraTransform: matrix_identity_float4x4, cameraIntrinsics: matrix_identity_float3x3,
                imageResolution: CGSize(width: 20, height: 20), depthMap: depth ? depthMap : nil,
                pose: .init(position: .zero, yaw: 0), trackingQuality: tracking)
        }
        let latest = snapshot(3)
        let depthLost = snapshot(2, depth: false)
        XCTAssertEqual(ARFollowMePerceptionSource.inferenceSnapshot(queued: depthLost, latest: latest).id, depthLost.id)
        let trackingLost = snapshot(2, tracking: .limited)
        XCTAssertEqual(ARFollowMePerceptionSource.inferenceSnapshot(queued: trackingLost, latest: latest).id, trackingLost.id)
        XCTAssertEqual(ARFollowMePerceptionSource.inferenceSnapshot(queued: snapshot(2), latest: latest).id, latest.id)
    }

    func testDroppingUnconsumedStreamReleasesFollowInferencePriority() async throws {
        let ar = ARSessionManager()
        let detector = Detector(supportedLabels: ["person"], detectionHandler: { _ in [] })
        let source = ARFollowMePerceptionSource(ar: ar, detector: detector)
        var stream: AsyncStream<FollowPerceptionEvent>? = source.events()
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        ar.ingestForTesting(image: try XCTUnwrap(image), timestamp: 1, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 20, height: 20),
            depthMap: nil, trackingQuality: .normal)
        let snapshot = try XCTUnwrap(ar.latestSnapshot)
        let suppressed = await detector.evaluateForMissionAsync(snapshot)
        XCTAssertNil(suppressed)
        XCTAssertNotNil(stream)
        stream = nil
        for _ in 0..<30 { await Task.yield() }
        let resumed = await detector.evaluateForMissionAsync(snapshot)
        XCTAssertNotNil(resumed, "Abandoning the event stream must release its detector consumer token")
    }

    func testSlowConsumerReceivesNewestCompletedFrameInsteadOfInferenceBacklog() async throws {
        let ar = ARSessionManager()
        let detector = Detector(supportedLabels: ["person"], detectionHandler: { _ in [] })
        let source = ARFollowMePerceptionSource(ar: ar, detector: detector)
        var iterator = source.events().makeAsyncIterator()
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        let buffer = try XCTUnwrap(image)
        func publish(_ sequence: Int) {
            ar.ingestForTesting(image: buffer, timestamp: Double(sequence), cameraTransform: matrix_identity_float4x4,
                intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 20, height: 20),
                depthMap: nil, trackingQuality: .normal)
        }
        publish(1)
        guard case .frame? = await iterator.next() else { return XCTFail("Expected initial frame") }
        for sequence in 2...4 {
            publish(sequence)
            for _ in 0..<1000 where detector.latestFollowEvaluation?.snapshot.timestamp != Double(sequence) {
                try await Task.sleep(for: .milliseconds(1))
            }
            XCTAssertEqual(detector.latestFollowEvaluation?.snapshot.timestamp, Double(sequence))
            for _ in 0..<30 { await Task.yield() }
        }
        guard case .frame(let latest)? = await iterator.next() else { return XCTFail("Expected newest completed frame") }
        XCTAssertEqual(latest.timestamp, 4)
    }

    func testMissionReceiptSurvivesSkippedInferenceAndLimitedTrackingWithoutSpatialGrounding() async throws {
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        let buffer = try XCTUnwrap(image)
        let ar = ARSessionManager()
        let detector = Detector(supportedLabels: ["chair"], detectionHandler: { _ in
            [.init(label: "chair", confidence: 0.9, boundingBox: CGRect(x: 0.2, y: 0.2, width: 0.4, height: 0.4))]
        })
        var uptime = 10.0
        let source = ARPerceptionSource(ar: ar, detector: detector, detectionNow: { uptime })
        ar.ingestForTesting(image: buffer, timestamp: uptime,
            cameraTransform: matrix_identity_float4x4, intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 20, height: 20), depthMap: nil, trackingQuality: .normal)
        await source.refreshDetections()
        XCTAssertEqual(source.detectObjects().map(\.label), ["chair"])
        let consumer = detector.beginFollowConsumer()
        uptime = 10.4
        await source.refreshDetections() // Preview-only request deliberately skipped.
        XCTAssertEqual(source.detectObjects().map(\.label), ["chair"])
        ar.ingestForTesting(image: buffer, timestamp: uptime,
            cameraTransform: matrix_identity_float4x4, intrinsics: matrix_identity_float3x3,
            imageResolution: CGSize(width: 20, height: 20), depthMap: nil, trackingQuality: .limited)
        XCTAssertEqual(source.detectObjects().map(\.label), ["chair"])
        XCTAssertNil(source.unprojectDetectedObject(normalizedPoint: CGPoint(x: 0.5, y: 0.5)))
        XCTAssertNil(source.unproject(normalizedPoint: CGPoint(x: 0.5, y: 0.5)))
        detector.endFollowConsumer(consumer)
        await source.refreshDetections()
        XCTAssertEqual(source.detectObjects().map(\.label), ["chair"])
        if let object = source.detectObjects().first {
            XCTAssertNil(source.groundPoint(of: object))
            XCTAssertNil(source.unproject(object: object))
        }
        let secondConsumer = detector.beginFollowConsumer()
        uptime = 10.8
        await source.refreshDetections()
        XCTAssertEqual(source.detectObjects().map(\.label), ["chair"])
        uptime = 11.0
        XCTAssertTrue(source.detectObjects().isEmpty, "Skipped refresh must not renew the receipt timestamp")
        detector.endFollowConsumer(secondConsumer)
    }

    func testLiveFollowRequiresMatchingBodyDespitePerfectRawPersonScore() async throws {
        for scenario in ["none", "matched", "failed", "unavailable"] {
            try await checkLiveBodyVerification(scenario)
        }
    }

    private func checkLiveBodyVerification(_ scenario: String) async throws {
        var bodyOrientations: [CGImagePropertyOrientation] = []
        let handler: ((CVPixelBuffer, CGImagePropertyOrientation) throws -> [PersonBodyVerifier.Body])?
        if scenario == "unavailable" { handler = nil }
        else {
            handler = { _, orientation in
                bodyOrientations.append(orientation)
                if scenario == "failed" { throw URLError(.cannotDecodeContentData) }
                if scenario == "none" { return [] }
                func joint(_ x: Double, _ y: Double) -> PersonBodyVerifier.Joint {
                    .init(location: CGPoint(x: x, y: y), confidence: 0.9)
                }
                return [.init(leftShoulder: joint(0.4, 0.7), rightShoulder: joint(0.6, 0.7),
                    leftHip: joint(0.43, 0.4), rightHip: joint(0.57, 0.4))]
            }
        }
        let detector = Detector(supportedLabels: ["person"], detectionHandler: { _ in
            [.init(label: "person", confidence: 1, boundingBox: CGRect(x: 0.2, y: 0.2, width: 0.6, height: 0.6))]
        }, bodyPoseHandler: handler)
        var image: CVPixelBuffer?
        var depth: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_DepthFloat32, nil, &depth)
        let map = try XCTUnwrap(depth)
        CVPixelBufferLockBaseAddress(map, [])
        let base = CVPixelBufferGetBaseAddress(map)!.assumingMemoryBound(to: Float.self)
        for row in 0..<20 { for col in 0..<20 { base[row * CVPixelBufferGetBytesPerRow(map) / 4 + col] = 2 } }
        CVPixelBufferUnlockBaseAddress(map, [])
        let ar = ARSessionManager()
        var stream = ARFollowMePerceptionSource(ar: ar, detector: detector).events().makeAsyncIterator()
        ar.ingestForTesting(image: try XCTUnwrap(image), timestamp: 1, cameraTransform: matrix_identity_float4x4,
            intrinsics: matrix_identity_float3x3, imageResolution: CGSize(width: 20, height: 20),
            depthMap: map, trackingQuality: .normal)
        guard case .frame(let batch)? = await stream.next() else { return XCTFail("Expected frame") }
        XCTAssertEqual(bodyOrientations, scenario == "unavailable" ? [] : [.right])
        XCTAssertEqual(batch.perceptionDiagnostics?.rawPersonCount, 1)
        XCTAssertEqual(batch.perceptionDiagnostics?.projectionAttemptedCount, scenario == "matched" ? 1 : 0)
        XCTAssertEqual(batch.people.count, scenario == "matched" ? 1 : 0, "Raw score alone cannot create a motion-eligible person")
        XCTAssertEqual(batch.perceptionDiagnostics?.personVerification?.first?.accepted, scenario == "matched")
        XCTAssertEqual(detector.latestFollowEvaluation?.receipt.frame.frameID, batch.frameID)
    }
    func testLiveFollowDoesNotRetryWithAnIncompatibleOrientation() async throws {
        var orientations: [CGImagePropertyOrientation] = []
        let detector = Detector(supportedLabels: ["person"], visionHandler: { _, orientation in
            orientations.append(orientation)
            return orientation == .right ? [] : [
                .init(label: "person", confidence: 0.99,
                    boundingBox: CGRect(x: 0.2, y: 0.3, width: 0.2, height: 0.3))]
        })
        var image: CVPixelBuffer?
        var depth: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 100, 80, kCVPixelFormatType_32BGRA, nil, &image)
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 16, kCVPixelFormatType_DepthFloat32, nil, &depth)
        let map = try XCTUnwrap(depth)
        CVPixelBufferLockBaseAddress(map, [])
        let base = CVPixelBufferGetBaseAddress(map)!.assumingMemoryBound(to: Float.self)
        for row in 0..<16 { for col in 0..<20 {
            // Upright-up feet and right-inverse feet see different coherent surfaces.
            base[row * CVPixelBufferGetBytesPerRow(map) / 4 + col] = col < 10 ? 1 : 3
        } }
        CVPixelBufferUnlockBaseAddress(map, [])
        let ar = ARSessionManager()
        let source = ARFollowMePerceptionSource(ar: ar, detector: detector)
        var iterator = source.events().makeAsyncIterator()
        ar.ingestForTesting(image: try XCTUnwrap(image), timestamp: 3,
            cameraTransform: simd_float4x4(1), intrinsics: simd_float3x3(columns: (
                SIMD3<Float>(50, 0, 0), SIMD3<Float>(0, 40, 0), SIMD3<Float>(50, 40, 1))),
            imageResolution: CGSize(width: 100, height: 80), depthMap: map, trackingQuality: .normal)
        guard case .frame(let batch)? = await iterator.next() else { return XCTFail("Expected live batch") }
        XCTAssertEqual(orientations, [.right])
        XCTAssertTrue(batch.people.isEmpty, "An up-oriented box must not become a right-oriented world observation")
        XCTAssertEqual(batch.perceptionDiagnostics?.rawPersonCount, 0)
        XCTAssertEqual(batch.perceptionDiagnostics?.projectionAttemptedCount, 0)
        // Coybot generic detection also keeps a single orientation per request.
        orientations.removeAll()
        let generic = detector.detect(try XCTUnwrap(image))
        XCTAssertEqual(orientations, [.right])
        XCTAssertTrue(generic.isEmpty)
        // An explicitly up-oriented request can find this asymmetric box, but it
        // must be paired with up-oriented snapshot geometry to project correctly.
        let rotated = detector.detect(try XCTUnwrap(image), orientation: .up)
        XCTAssertEqual(orientations, [.right, .up])
        XCTAssertEqual(rotated.count, 1)
        let snapshot = try XCTUnwrap(ar.latestSnapshot)
        let upSnapshot = ARFrameSnapshot(id: snapshot.id, timestamp: snapshot.timestamp,
            image: snapshot.image, cameraTransform: snapshot.cameraTransform,
            cameraIntrinsics: snapshot.cameraIntrinsics, imageResolution: snapshot.imageResolution,
            depthMap: snapshot.depthMap, pose: snapshot.pose, trackingQuality: snapshot.trackingQuality,
            imageOrientation: .up)
        let correct = ARFollowMePerceptionSource.batch(from: upSnapshot, detections: rotated)
        XCTAssertEqual(try XCTUnwrap(correct.people.first).position.x, -0.4, accuracy: 0.001)
        XCTAssertEqual(try XCTUnwrap(correct.people.first).position.y, -1, accuracy: 0.001)
        let wrong = ARFollowMePerceptionSource.batch(from: snapshot, detections: rotated)
        XCTAssertEqual(wrong.people.count, 1)
        XCTAssertEqual(try XCTUnwrap(wrong.people.first).position.x, 1.2, accuracy: 0.001)
        XCTAssertEqual(try XCTUnwrap(wrong.people.first).position.y, -3, accuracy: 0.001)
    }
    func testFailedInferenceHasUnknownCountsAndDoesNotProjectSuppliedCandidates() throws {
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        let snapshot = ARFrameSnapshot(id: .init(generation: 1, sequence: 1), timestamp: 1,
            image: try XCTUnwrap(image), cameraTransform: simd_float4x4(1), cameraIntrinsics: simd_float3x3(1),
            imageResolution: CGSize(width: 20, height: 20), depthMap: nil,
            pose: .init(position: .zero, yaw: 0), trackingQuality: .normal)
        let output = ARFollowMePerceptionSource.batch(from: snapshot, detections: [
            .init(label: "person", confidence: 0.9, boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6))],
            inferenceStatus: .failed)
        let facts = try XCTUnwrap(output.perceptionDiagnostics)
        XCTAssertEqual(facts.inferenceStatus, .failed)
        XCTAssertNil(facts.rawDetectorCount)
        XCTAssertNil(facts.projectionAttemptedCount)
        XCTAssertNil(facts.projectionAcceptedCount)
        XCTAssertNil(facts.projectionRejectedCount)
        XCTAssertTrue(facts.candidates.isEmpty)
    }

    func testPersonFeetProjectFromTheSameSnapshot() {
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        var depth: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_DepthFloat32, nil, &depth)
        let map = try! XCTUnwrap(depth)
        CVPixelBufferLockBaseAddress(map, [])
        let base = CVPixelBufferGetBaseAddress(map)!.assumingMemoryBound(to: Float32.self)
        for row in 0..<20 {
            for col in 0..<20 { base[row * CVPixelBufferGetBytesPerRow(map) / 4 + col] = 3 }
        }
        CVPixelBufferUnlockBaseAddress(map, [])
        let id = ARFrameID(generation: 2, sequence: 7)
        let snapshot = ARFrameSnapshot(
            id: id, timestamp: 12, image: try! XCTUnwrap(image),
            cameraTransform: simd_float4x4(1),
            cameraIntrinsics: simd_float3x3(columns: (
                SIMD3<Float>(10, 0, 0), SIMD3<Float>(0, 10, 0), SIMD3<Float>(10, 10, 1))),
            imageResolution: CGSize(width: 20, height: 20), depthMap: map,
            pose: Pose2D(position: .zero, yaw: 0), trackingQuality: .normal)
        let detections = [
            Detector.Detection(label: "person", confidence: 0.9,
                               boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6)),
            Detector.Detection(label: "chair", confidence: 0.9,
                               boundingBox: CGRect(x: 0.1, y: 0.1, width: 0.2, height: 0.2))
        ]

        let batch = ARFollowMePerceptionSource.batch(from: snapshot, detections: detections,
                                                    trackingReason: .excessiveMotion)

        XCTAssertEqual(batch.frameID, id)
        XCTAssertEqual(batch.timestamp, 12)
        XCTAssertEqual(batch.people.count, 1)
        XCTAssertEqual(batch.trackingQuality, .normal)
        XCTAssertEqual(batch.pose, snapshot.pose, "Live diagnostic limitations cannot invalidate a normal snapshot")
        // Vision's portrait y=.2 maps to raw sensor x=16; (16-10)/10 * 3 = 1.8.
        XCTAssertEqual(batch.people[0].position.x, 1.8, accuracy: 0.01)
        XCTAssertEqual(batch.people[0].position.y, -3, accuracy: 0.01)
    }

    func testARInterruptionCannotBeOverwrittenByLaterFrame() async {
        let ar = ARSessionManager()
        let detector = Detector(supportedLabels: ["person"], detectionHandler: { _ in [] })
        let source = ARFollowMePerceptionSource(ar: ar, detector: detector)
        let events = source.events()
        var iterator = events.makeAsyncIterator()
        ar.interruptionBeganForTesting()
        for _ in 0..<20 { await Task.yield() }

        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        ar.ingestForTesting(image: try! XCTUnwrap(image), timestamp: 3,
                            cameraTransform: simd_float4x4(1), intrinsics: simd_float3x3(1),
                            imageResolution: CGSize(width: 20, height: 20), depthMap: nil,
                            trackingQuality: .normal)
        for _ in 0..<20 { await Task.yield() }

        guard case .interrupted? = await iterator.next() else {
            return XCTFail("An AR interruption must never be discarded in favor of a frame")
        }
    }

    func testLimitedSnapshotKeepsItsQualityAndCannotUseDiagnosticNormalTrackingForPerception() throws {
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        let snapshot = ARFrameSnapshot(
            id: ARFrameID(generation: 1, sequence: 4), timestamp: 8,
            image: try XCTUnwrap(image), cameraTransform: simd_float4x4(1),
            cameraIntrinsics: simd_float3x3(1), imageResolution: CGSize(width: 20, height: 20),
            depthMap: nil, pose: Pose2D(position: Vec2(2, 3), yaw: 0), trackingQuality: .limited)
        let batch = ARFollowMePerceptionSource.batch(from: snapshot, detections: [],
                                                    trackingReason: nil, inferenceDuration: 0.125)
        XCTAssertEqual(batch.trackingQuality, .limited)
        XCTAssertNil(batch.pose)
        XCTAssertTrue(batch.people.isEmpty)
        XCTAssertEqual(batch.inferenceDuration, 0.125)
        XCTAssertEqual(ARFollowMePerceptionSource.trackingReason(from: .limited(.excessiveMotion)), .excessiveMotion)
        XCTAssertEqual(ARFollowMePerceptionSource.trackingReason(from: .limited(.insufficientFeatures)), .insufficientFeatures)
        XCTAssertEqual(ARFollowMePerceptionSource.trackingReason(from: .limited(.initializing)), .initializing)
        XCTAssertEqual(ARFollowMePerceptionSource.trackingReason(from: .limited(.relocalizing)), .relocalizing)
        XCTAssertNil(ARFollowMePerceptionSource.trackingReason(from: .normal))
    }

    func testStreamMeasuresInferenceAndSkipsItForLimitedTracking() async throws {
        let ar = ARSessionManager()
        var inferenceCount = 0
        let detector = Detector(supportedLabels: ["person"], detectionHandler: { _ in
            inferenceCount += 1
            Thread.sleep(forTimeInterval: 0.02)
            return []
        })
        let source = ARFollowMePerceptionSource(ar: ar, detector: detector)
        var iterator = source.events().makeAsyncIterator()
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        let buffer = try XCTUnwrap(image)
        ar.ingestForTesting(image: buffer, timestamp: 3,
                            cameraTransform: simd_float4x4(1), intrinsics: simd_float3x3(1),
                            imageResolution: CGSize(width: 20, height: 20), depthMap: nil,
                            trackingQuality: .normal)
        guard case .frame(let normal)? = await iterator.next() else { return XCTFail("Expected normal batch") }
        XCTAssertEqual(normal.trackingQuality, .normal)
        XCTAssertEqual(normal.perceptionDiagnostics?.inferenceStatus, .executed)
        XCTAssertEqual(normal.perceptionDiagnostics?.rawDetectorCount, 0)
        XCTAssertGreaterThanOrEqual(try XCTUnwrap(normal.inferenceDuration), 0.02)
        XCTAssertEqual(inferenceCount, 1)
        ar.ingestForTesting(image: buffer, timestamp: 4,
                            cameraTransform: simd_float4x4(1), intrinsics: simd_float3x3(1),
                            imageResolution: CGSize(width: 20, height: 20), depthMap: nil,
                            trackingQuality: .limited)
        guard case .frame(let limited)? = await iterator.next() else { return XCTFail("Expected limited batch") }
        XCTAssertEqual(limited.trackingQuality, .limited)
        XCTAssertNil(limited.pose)
        XCTAssertNil(limited.inferenceDuration)
        XCTAssertEqual(limited.perceptionDiagnostics?.inferenceStatus, .skippedTracking)
        XCTAssertNil(limited.perceptionDiagnostics?.rawPersonCount)
        XCTAssertNil(limited.perceptionDiagnostics?.projectionAttemptedCount)
        XCTAssertEqual(inferenceCount, 1)
    }
}
