import XCTest
import CoreVideo
import simd
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowPipelineDiagnosticsTests: XCTestCase {
    func testRecoverySnapshotsKeepLastReliableSeparateFromProvisionalAndShareSummaryBudget() async throws {
        let perception = FollowPerceptionFake()
        let motion = FollowMotionFake()
        let clock = ManualFollowClock()
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        let sink = FollowDiagnosticRecordingSink()
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock,
            configuration: config, eventSink: sink.append)
        _ = await coordinator.start()
        func send(_ sequence: UInt64, point: Double?, healthy: Bool = true) async {
            let id = ARFrameID(generation: 1, sequence: sequence)
            let pose = Pose2D(position: .zero, yaw: 0)
            let people: [FollowPersonObservation] = point.map { [ .init(frameID: id, timestamp: 0, confidence: 0.9,
                boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6), position: Vec2($0, 0), pose: pose) ] } ?? []
            perception.send(.frame(.init(frameID: id, timestamp: 0, pose: pose, depthAvailable: true,
                people: people, trackingQuality: healthy ? .normal : .limited)))
            for _ in 0..<100 { await Task.yield() }
        }
        func records(_ name: String) throws -> [[String: Any]] {
            try sink.records.filter { $0.event == name }.map {
                try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap($0.fields["payload"]).utf8)) as? [String: Any])
            }
        }
        await send(1, point: 4)
        await send(2, point: nil)
        let unavailable = try XCTUnwrap(records("follow_recovery.unavailable").first)
        XCTAssertEqual(unavailable["reason"] as? String, "absolute_heading_companion_unavailable")
        XCTAssertTrue(unavailable["center_source_timestamp_s"] is NSNull)
        let entry = try XCTUnwrap(records("follow_recovery.started").first)
        XCTAssertTrue(entry["provisional_frame_id"] is NSNull, "The last reliable lock is not a provisional detection")
        await send(3, point: 4.2)
        await send(4, point: 4.2, healthy: false)
        let snapshot = try XCTUnwrap(records("follow_frame").last)
        XCTAssertEqual(snapshot["recovery_active"] as? Bool, true)
        XCTAssertEqual(snapshot["reliable_frame_id"] as? String, "1:1")
        XCTAssertEqual(snapshot["provisional_frame_id"] as? String, "1:3")
        XCTAssertEqual(snapshot["provisional_person_x"] as? Double, 4.2)
        XCTAssertEqual(snapshot["reliable_person_x"] as? Double, 4)
        XCTAssertEqual(snapshot["anchor_frame_id"] as? String, "1:1")
        XCTAssertEqual(snapshot["current_target_availability"] as? String, "unavailable_unhealthy_perception")
        XCTAssertEqual(snapshot["deadline_s"] as? Double, 10)
        let count = sink.records.count
        for sequence: UInt64 in 5...20 { await send(sequence, point: 4.2, healthy: false) }
        XCTAssertEqual(sink.records.count, count, "Repeated unhealthy frames do not create a new diagnostic stream")
        _ = await coordinator.stop()
        let stopped = try XCTUnwrap(records("follow_phase").last)
        XCTAssertEqual(stopped["current_target_availability"] as? String, "unavailable_inactive")
        XCTAssertTrue(stopped["episode_id"] is NSNull)
        _ = await coordinator.start()
        await send(21, point: nil)
        let fresh = try XCTUnwrap(records("follow_person.association").last)
        XCTAssertEqual(fresh["recovery_active"] as? Bool, false)
        XCTAssertTrue(fresh["reliable_frame_id"] is NSNull)
        XCTAssertTrue(fresh["provisional_frame_id"] is NSNull)
        _ = await coordinator.stop()
    }

    func testEmittedConfidenceAndDispersionFactsComeFromTheEvaluatedWindow() throws {
        let detection = Detector.Detection(label: "person", confidence: 0.9,
            boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6))
        let low = ARFollowMePerceptionSource.batch(from: try snapshot(depth: 3, confidence: 0), detections: [detection])
        let lowEvent = try envelope(low)
        let lowFacts = try XCTUnwrap((lowEvent["projection_evaluations"] as? [[String: Any]])?.first)
        XCTAssertEqual(lowFacts["low_confidence_count"] as? Int, 25)
        XCTAssertEqual(lowFacts["invalid_depth_count"] as? Int, 0)
        XCTAssertEqual(lowFacts["valid_sample_count"] as? Int, 0)
        XCTAssertEqual(lowFacts["confidence_availability"] as? String, "available")
        XCTAssertEqual(lowFacts["rejection_reason"] as? String, "insufficient_valid_depth")
        XCTAssertTrue(lowFacts["median_depth_m"] is NSNull)
        let mixed = Array(repeating: Float(2.8), count: 8) + Array(repeating: Float(3), count: 9) + Array(repeating: Float(3.2), count: 8)
        let dispersion = ARFollowMePerceptionSource.batch(from: try snapshot(depth: 3, patch: mixed, confidence: 2), detections: [detection])
        let dispersionEvent = try envelope(dispersion)
        let facts = try XCTUnwrap((dispersionEvent["projection_evaluations"] as? [[String: Any]])?.first)
        XCTAssertEqual(facts["valid_sample_count"] as? Int, 25)
        XCTAssertEqual(facts["median_depth_m"] as? Double, 3)
        XCTAssertGreaterThan(try XCTUnwrap(facts["mad_m"] as? Double), 0.1)
        XCTAssertEqual(facts["rejection_reason"] as? String, "inconsistent_depth")
        XCTAssertTrue(facts["projected_position"] is NSNull)
        XCTAssertEqual(facts["raw_person_id"] as? Int, 0)
        XCTAssertEqual(facts["depth_size"] as? [String: Int], ["width": 20, "height": 20])
        XCTAssertEqual(facts["sensor_pixel"] as? [String: Double], ["x": 16, "y": 10])
    }

    func testLegacyReadyAdapterNeverClaimsControllerFirstWheelOrSourceTelemetry() async throws {
        let perception = FollowPerceptionFake()
        let clock = ManualFollowClock()
        let motion = FollowMotionFake(clock: clock)
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        var authorized: [String: Any]?
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config) { name, fields in
            if name == "follow_ready.admission_authorized" {
                authorized = try! JSONSerialization.jsonObject(with: fields["payload"]!.data(using: .utf8)!) as? [String: Any]
            }
        }
        _ = await coordinator.start()
        for sequence: UInt64 in 1...3 {
            clock.advance(to: Double(sequence - 1) * 0.31)
            let id = ARFrameID(generation: 1, sequence: sequence)
            let pose = Pose2D(position: .zero, yaw: 0)
            perception.send(.frame(.init(frameID: id, timestamp: clock.now, pose: pose, depthAvailable: true,
                people: [.init(frameID: id, timestamp: clock.now, confidence: 0.9,
                    boundingBox: CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6), position: Vec2(1.6, 0), pose: pose)],
                trackingQuality: .normal)))
            for _ in 0..<100 { await Task.yield() }
        }
        let event = try XCTUnwrap(authorized)
        XCTAssertEqual(event["admission_boundary"] as? String, "legacy_signal_call")
        XCTAssertEqual(event["first_wheel_telemetry_availability"] as? String, "unknown")
        XCTAssertEqual(event["controller_source_age_s_availability"] as? String, "unknown")
        XCTAssertEqual(event["controller_perception_pairing"] as? String, "unknown")
        XCTAssertTrue(event["controller_pose_frame_id"] is NSNull)
        _ = await coordinator.stop()
    }
    func testHealthSummaryKeepsUnevaluatedTrackerAndLegacyPipelineCountsUnknown() throws {
        let batch = ARFollowMePerceptionSource.batch(from: try snapshot(depth: 3), detections: [])
        let health = FollowAssociationEvaluation.healthPayload(batch: batch, now: 10)
        XCTAssertEqual(health["projected_person_count"], .number(0))
        XCTAssertEqual(health["eligible_candidate_count"], .null)
        XCTAssertEqual(health["matched_candidate_count"], .null)
        XCTAssertEqual(health["selected_candidate_count"], .null)
        XCTAssertEqual(health["tracker_counts_availability"], .string("not_evaluated"))
        let legacy = FollowFrameBatch(frameID: batch.frameID, timestamp: 10, pose: batch.pose,
            depthAvailable: true, people: [])
        let unknown = FollowAssociationEvaluation.healthPayload(batch: legacy, now: 10)
        XCTAssertEqual(unknown["raw_detector_count"], .null)
        XCTAssertEqual(unknown["pipeline_availability"], .string("unknown_legacy_provider"))
        XCTAssertEqual(unknown["projected_person_count"], .number(0), "Legacy batch still measures its supplied projected list")
        XCTAssertEqual(unknown["same_frame"], .null)
    }
    func testUnchangedLostOutcomeEmitsChangedProjectionReasonButDeduplicatesRepeats() async throws {
        let perception = FollowPerceptionFake()
        let motion = FollowMotionFake()
        motion.suspendRotation = true
        let clock = ManualFollowClock()
        clock.advance(to: 10, wakeSleepers: false)
        var config = FollowMeConfiguration()
        config.stationaryPauseSeconds = 0
        var records: [[String: Any]] = []
        let coordinator = FollowMeCoordinator(perception: perception, motion: motion, clock: clock, configuration: config) { name, fields in
            if name == "follow_person.association" {
                records.append(try! JSONSerialization.jsonObject(with: fields["payload"]!.data(using: .utf8)!) as! [String: Any])
            }
        }
        _ = await coordinator.start()
        for _ in 0..<30 { await Task.yield() }
        for sequence: UInt64 in 1...4 {
            let box = sequence <= 2 ? CGRect(x: 0, y: 0.2, width: 0.2, height: 0.6) : CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6)
            perception.send(.frame(ARFollowMePerceptionSource.batch(from: try snapshot(depth: 0.04, sequence: sequence),
                detections: [.init(label: "person", confidence: 0.9, boundingBox: box)])))
            for _ in 0..<100 { await Task.yield() }
        }
        XCTAssertEqual(records.count, 2, "Each changed upstream reason emits immediately; identical losses do not repeat")
        XCTAssertEqual(records.compactMap { ($0["projection_evaluations"] as? [[String: Any]])?.first?["rejection_reason"] as? String },
                       ["clipped_box", "insufficient_valid_depth"])
        motion.releaseRotation()
        _ = await coordinator.stop()
    }
    func testRawIDsSurviveFilteringAndWorldJumpKeepsProjectionAndTrackerReasonsSeparate() throws {
        let box = CGRect(x: 0.4, y: 0.2, width: 0.2, height: 0.6)
        let detections: [Detector.Detection] = [
            .init(label: "person", confidence: 0.9, boundingBox: CGRect(x: 0, y: 0.2, width: 0.2, height: 0.6)),
            .init(label: "chair", confidence: 0.9, boundingBox: box),
            .init(label: "PERSON", confidence: 0.9, boundingBox: box)]
        let first = ARFollowMePerceptionSource.batch(from: try snapshot(depth: 3), detections: detections)
        let initial = FollowTargetTracker().selectInitialEvaluated(first.people, now: 10)
        let event = try envelope(first)
        XCTAssertEqual(event["raw_detector_count"] as? Int, 3)
        XCTAssertEqual(event["raw_person_count"] as? Int, 2)
        XCTAssertEqual(event["projection_accepted_count"] as? Int, 1)
        XCTAssertEqual(event["eligible_candidate_count"] as? Int, 1)
        XCTAssertEqual(event["selected_candidate_count"] as? Int, 1)
        let selected = try XCTUnwrap(event["selected_candidate"] as? [String: Any])
        XCTAssertEqual(selected["raw_person_id"] as? Int, 1)
        XCTAssertEqual(selected["candidate_id"] as? Int, 1)
        let raw = try XCTUnwrap(event["projection_evaluations"] as? [[String: Any]])
        XCTAssertEqual(raw[1]["valid_sample_count"] as? Int, 25)
        XCTAssertEqual(raw[1]["invalid_depth_count"] as? Int, 0)
        XCTAssertEqual(raw[1]["low_confidence_count"] as? Int, 0)
        XCTAssertEqual(raw[1]["median_depth_m"] as? Double, 3)
        XCTAssertEqual(raw[1]["mad_m"] as? Double, 0)
        XCTAssertEqual(raw[1]["inlier_count"] as? Int, 25)
        XCTAssertEqual(raw[1]["confidence_availability"] as? String, "unavailable")
        let jumped = ARFollowMePerceptionSource.batch(from: try snapshot(depth: 4), detections: detections)
        let evaluated = FollowTargetTracker().continueTrackEvaluated(jumped.people,
            previous: try XCTUnwrap(initial.decision), predictedPosition: first.people[0].position, now: 10).evaluation
        XCTAssertEqual(evaluated.outcome, "lost")
        XCTAssertEqual(evaluated.candidates[0]["raw_person_id"], .number(1))
        XCTAssertEqual(evaluated.candidates[0]["rejection_reason"], .string("world_distance_exceeded"))
        XCTAssertNil(jumped.perceptionDiagnostics?.candidates[1].rejection)
        XCTAssertEqual(jumped.perceptionDiagnostics?.projectionAcceptedCount, 1)
    }
    func testMeasuredEmptyInferenceDiffersFromRejectedProjectionInEnvelope() throws {
        let snapshot = try snapshot()
        let empty = ARFollowMePerceptionSource.batch(from: snapshot, detections: [])
        let rejected = ARFollowMePerceptionSource.batch(from: snapshot, detections: [
            .init(label: "person", confidence: 0.9, boundingBox: CGRect(x: 0, y: 0.2, width: 0.2, height: 0.6))])
        let emptyEvent = try envelope(empty)
        let rejectedEvent = try envelope(rejected)
        XCTAssertEqual(emptyEvent["raw_detector_count"] as? Int, 0)
        XCTAssertEqual(emptyEvent["raw_person_count"] as? Int, 0)
        XCTAssertEqual(rejectedEvent["raw_person_count"] as? Int, 1)
        XCTAssertEqual(rejectedEvent["projection_attempted_count"] as? Int, 1)
        XCTAssertEqual(rejectedEvent["projection_rejected_count"] as? Int, 1)
        XCTAssertEqual(rejectedEvent["projection_accepted_count"] as? Int, 0)
        let candidates = try XCTUnwrap(rejectedEvent["projection_evaluations"] as? [[String: Any]])
        XCTAssertEqual(candidates.first?["rejection_reason"] as? String, "clipped_box")
        XCTAssertEqual(candidates.first?["requested_sample_count"] as? Int, 25)
        XCTAssertTrue(candidates.first?["valid_sample_count"] is NSNull)
    }

    func testActualThrowingDetectorFailureAndTrackingSkipKeepCountsUnknown() async throws {
        let ar = ARSessionManager()
        let detector = Detector(supportedLabels: ["person"], detectionHandler: { _ in
            throw NSError(domain: "https://private/path?transcript=secret", code: 7,
                          userInfo: [NSLocalizedDescriptionKey: "private response body"])
        })
        var iterator = ARFollowMePerceptionSource(ar: ar, detector: detector).events().makeAsyncIterator()
        let input = try snapshot()
        for (index, quality) in [ARTrackingQuality.normal, .limited].enumerated() {
            ar.ingestForTesting(image: input.image, timestamp: 10 + Double(index), cameraTransform: input.cameraTransform,
                intrinsics: input.cameraIntrinsics, imageResolution: input.imageResolution, depthMap: nil,
                trackingQuality: quality)
            guard case .frame(let batch)? = await iterator.next() else { return XCTFail("Missing frame") }
            let event = try envelope(batch)
            XCTAssertEqual(event["inference_status"] as? String, quality == .normal ? "failed" : "skipped_tracking")
            XCTAssertTrue(event["raw_detector_count"] is NSNull)
            XCTAssertTrue(event["projection_attempted_count"] is NSNull)
            XCTAssertTrue(event["projected_person_count"] is NSNull)
            XCTAssertEqual(event["projected_person_count_availability"] as? String, "not_evaluated")
            XCTAssertTrue(event["same_frame"] is NSNull)
            if quality == .normal { XCTAssertEqual(event["inference_failure_reason"] as? String, "inference_failed") }
            let json = String(data: try JSONSerialization.data(withJSONObject: event), encoding: .utf8)!
            XCTAssertFalse(json.contains("private"))
            XCTAssertFalse(json.contains("https"))
        }
        XCTAssertTrue(detector.detect(input).detections.isEmpty, "Legacy detect still folds errors into empty output")
    }

    private func envelope(_ batch: FollowFrameBatch) throws -> [String: Any] {
        var result: [String: Any] = [:]
        let emitter = FollowDiagnosticEmitter(streamID: "pipeline", monotonic: { 10 }, utc: { Date(timeIntervalSince1970: 0) }) {
            _, fields in result = try! JSONSerialization.jsonObject(with: fields["payload"]!.data(using: .utf8)!) as! [String: Any]
        }
        let evaluation = FollowTargetTracker().selectInitialEvaluated(batch.people, now: 10).evaluation
        emitter.emit(.init(event: "follow_person.association", payload: evaluation.payload(batch: batch, now: 10, previousOutcome: nil)))
        return result
    }

    private func snapshot(depth value: Float? = nil, sequence: UInt64 = 7,
                          patch: [Float]? = nil, confidence valueOfConfidence: UInt8? = nil) throws -> ARFrameSnapshot {
        var image: CVPixelBuffer?
        CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_32BGRA, nil, &image)
        var depth: CVPixelBuffer?
        if let value {
            CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_DepthFloat32, nil, &depth)
            let buffer = try XCTUnwrap(depth)
            CVPixelBufferLockBaseAddress(buffer, [])
            let base = CVPixelBufferGetBaseAddress(buffer)!.assumingMemoryBound(to: Float.self)
            for row in 0..<20 { for col in 0..<20 { base[row * CVPixelBufferGetBytesPerRow(buffer) / 4 + col] = value } }
            if let patch {
                for row in 0..<5 { for col in 0..<5 { base[(row + 8) * CVPixelBufferGetBytesPerRow(buffer) / 4 + col + 14] = patch[row * 5 + col] } }
            }
            CVPixelBufferUnlockBaseAddress(buffer, [])
        }
        var confidence: CVPixelBuffer?
        if let valueOfConfidence {
            CVPixelBufferCreate(kCFAllocatorDefault, 20, 20, kCVPixelFormatType_OneComponent8, nil, &confidence)
            let buffer = try XCTUnwrap(confidence)
            CVPixelBufferLockBaseAddress(buffer, [])
            let base = CVPixelBufferGetBaseAddress(buffer)!.assumingMemoryBound(to: UInt8.self)
            for row in 0..<20 { for col in 0..<20 { base[row * CVPixelBufferGetBytesPerRow(buffer) + col] = valueOfConfidence } }
            CVPixelBufferUnlockBaseAddress(buffer, [])
        }
        return ARFrameSnapshot(id: .init(generation: 1, sequence: sequence), timestamp: 10,
            image: try XCTUnwrap(image), cameraTransform: simd_float4x4(1), cameraIntrinsics: simd_float3x3(columns: (
                SIMD3<Float>(10, 0, 0), SIMD3<Float>(0, 10, 0), SIMD3<Float>(10, 10, 1))),
            imageResolution: CGSize(width: 20, height: 20), depthMap: depth,
            pose: .init(position: .zero, yaw: 0), trackingQuality: .normal, depthConfidenceMap: confidence)
    }
}
