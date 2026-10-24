import XCTest
@testable import PhroverKit

@MainActor
final class FollowDiagnosticEventTests: XCTestCase {
    func testPlanTelemetrySeparatesWindowOverheadFromOverlappingLatencyDiagnostics() throws {
        let calibration = FollowTurnBurstPlanner.Calibration(operationID: 1, generation: 1, targetYaw: 0.5,
            clockDomain: "ar_system_uptime")
        func sample(_ id: UInt64, _ time: Double, _ yaw: Double) -> FollowTurnBurstPlanner.Sample {
            .init(yaw: yaw, sequence: id, generation: 1, sourceTimestamp: time,
                collectedUptime: time, clockDomain: "ar_system_uptime", healthy: true)
        }
        let response = FollowTurnBurstPlanner.Response(operationID: 1, generation: 1, targetYaw: 0.5,
            clockDomain: "ar_system_uptime", requestedBudget: 0.080, sendEntryUptime: 10,
            sendResponseUptime: 10.12, stopObligationUptime: 10.08, stopAcknowledgementUptime: 10.23,
            samples: [sample(1, 9.99, 0), sample(2, 10.24, 0.1), sample(3, 10.55, 0.2)])
        let profile = FollowTurnBurstPlanner.Profile(purpose: .scan)
        let reduction = FollowTurnBurstPlanner.recording(response, in: calibration, profile: profile)
        let trace = FollowTurnBurstDiagnosticTrace()
        trace.reduction(response, reduction)
        let actual = NavigationPoseSample(pose: .init(position: .zero, yaw: 0.3),
            frameID: .init(generation: 1, sequence: 4), sourceTimestamp: 10.9, trackingQuality: .normal)
        let decision = FollowTurnBurstPlanner.plan(.init(actualYaw: 0.3, profile: profile,
            calibration: reduction.calibration, sendEntryUptime: 11))
        trace.planning(reduction.calibration, sample: actual, profile: profile, decision: decision, uptime: 11)
        XCTAssertEqual(trace.fields["allowance_policy"], .string("command_stop_window_overhead_once;coast_in_response"))
        XCTAssertEqual(trace.fields["budget_formula"], .string("min(maximum,max(E/R-H,repeatableObservedBudget),overshootCeiling)"))
        guard case .number(let budget) = trace.fields["candidate_budget_s"] else { return XCTFail() }
        XCTAssertEqual(budget, -0.11284036747819653, accuracy: 1e-12)
        XCTAssertEqual(decision, .burst(direction: 1, budget: 0.080))
        XCTAssertEqual(trace.fields["budget_selection_reason"], .string("repeat_small_valid_response"))
        guard case .number(let overhead) = trace.fields["retained_response_overhead_s"],
              case .number(let window) = trace.fields["command_stop_window_s"] else { return XCTFail() }
        XCTAssertEqual(overhead, 0.15, accuracy: 1e-12)
        XCTAssertEqual(window, 0.23, accuracy: 1e-12)
        guard case .number(let latency) = trace.fields["latency_allowance_s"] else { return XCTFail() }
        XCTAssertEqual(latency, 0.27, accuracy: 1e-12, "Overlapping send/stop durations are not the model overhead")
    }
    func testMeasuredHostLatencyWithoutYawCannotBeReportedAsObservedRateOrZeroCoast() throws {
        let profile = FollowTurnBurstPlanner.Profile(purpose: .alignment)
        let calibration = FollowTurnBurstPlanner.Calibration(operationID: 41, generation: 3, targetYaw: 0.3,
            clockDomain: "ar_system_uptime")
        let response = FollowTurnBurstPlanner.Response(operationID: 41, generation: 3, targetYaw: 0.3,
            clockDomain: "ar_system_uptime", requestedBudget: 0.080, sendEntryUptime: 10,
            sendResponseUptime: 10.020, stopObligationUptime: 10.020, stopAcknowledgementUptime: 10.030, samples: [])
        let trace = FollowTurnBurstDiagnosticTrace()
        trace.reduction(response, FollowTurnBurstPlanner.recording(response, in: calibration, profile: profile))
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "missing-yaw", monotonic: { 300 }, utc: { Date() }, sink: sink.append)
        emitter.emit(.init(event: "follow_scan.burst_response", payload: trace.fields))
        let record = try decode(try XCTUnwrap(sink.records.first?.fields))
        XCTAssertEqual(record["latency_confidence"] as? String, "measured_host_maxima")
        XCTAssertEqual(record["response_rate_confidence"] as? String, "provisional_reference")
        XCTAssertTrue(record["net_source_rate_rad_s"] is NSNull)
        XCTAssertTrue(record["effective_budget_response_rate_rad_s"] is NSNull)
        XCTAssertTrue(record["observed_post_ack_travel_rad"] is NSNull)
        XCTAssertEqual(record["post_ack_travel_confidence"] as? String, "unknown")
        XCTAssertEqual(record["bracket_response_confidence"] as? String, "unknown_missing_endpoints")
    }

    func testRecoveryFactsUseExistingPrecisePrimitiveEnvelopeAndPrivacyFilter() throws {
        let memory = FollowReliableMemory(position: nil, pairedPose: nil, bearing: 0.123456789123,
            frameID: .init(generation: 3, sequence: 4), timestamp: 7, association: .continued, pairedYaw: 0.4)
        let episode = FollowReacquisitionEpisode(firstLoss: 8, anchor: memory)
        var payload = FollowReacquisitionDiagnostics.payload(episode, now: 9, stop: "pending")
        payload["images"] = .string("excluded")
        payload["measurement"] = .object(["yaw": .number(.nan), "depth_array": .array([.number(1)])])
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "recovery", monotonic: { 9 },
            utc: { Date(timeIntervalSince1970: 0) }, sink: sink.append)
        emitter.emit(.init(event: "follow_recovery.started", context: .init(sessionGeneration: 2, reason: "first_loss"), payload: payload))
        let record = try decode(try XCTUnwrap(sink.records.first?.fields))
        XCTAssertEqual(record["anchor_bearing_rad"] as? Double, 0.123456789123)
        XCTAssertEqual(record["event_sequence"] as? Int, 1)
        XCTAssertEqual(record["session_generation"] as? Int, 2)
        XCTAssertEqual(record["deadline_s"] as? Double, 18)
        XCTAssertTrue(record["anchor_raw_person_id"] is NSNull)
        XCTAssertTrue(record["measured_movement_rad"] is NSNull)
        XCTAssertNil(record["images"])
        let measurement = try XCTUnwrap(record["measurement"] as? [String: Any])
        XCTAssertTrue(measurement["yaw"] is NSNull)
        XCTAssertEqual(measurement["yaw_availability"] as? String, "nonfinite")
        XCTAssertNil(measurement["depth_array"])
    }

    func testEnvelopeIncludesExplicitNullsAndOrdersEqualTimeEvents() throws {
        var records: [(String, [String: String])] = []
        let emitter = FollowDiagnosticEmitter(streamID: "controller", monotonic: { 12.5 },
            utc: { Date(timeIntervalSince1970: 0) }, sink: { records.append(($0, $1)) })
        emitter.emit(FollowDiagnosticEvent(event: "follow_scan.operation_begin"))
        emitter.emit(FollowDiagnosticEvent(event: "follow_scan.operation_complete"))
        XCTAssertEqual(records.count, 2)
        let first = try decode(records[0].1)
        let second = try decode(records[1].1)
        XCTAssertEqual(first["event"] as? String, records[0].0)
        XCTAssertEqual(first["schema_version"] as? Int, 1)
        XCTAssertEqual(first["stream_id"] as? String, "controller")
        XCTAssertEqual(first["event_sequence"] as? Int, 1)
        XCTAssertEqual(second["event_sequence"] as? Int, 2)
        XCTAssertEqual(first["monotonic_s"] as? Double, 12.5)
        XCTAssertEqual(first["utc_time"] as? String, "1970-01-01T00:00:00.000Z")
        XCTAssertEqual(first["stale"] as? Bool, false)
        for key in ["session_generation", "operation_id", "purpose", "phase", "pulse_index", "outcome", "reason"] {
            XCTAssertTrue(first[key] is NSNull, "Missing explicit null for \(key)")
        }
    }

    func testCapturedContextNestedNumbersAndIndependentStreams() throws {
        var records: [[String: String]] = []
        let sink: @MainActor (String, [String: String]) -> Void = { _, fields in records.append(fields) }
        let controller = FollowDiagnosticEmitter(streamID: "controller", monotonic: { 1 }, utc: { Date() }, sink: sink)
        let coordinator = FollowDiagnosticEmitter(streamID: "coordinator", monotonic: { 1 }, utc: { Date() }, sink: sink)
        let context = FollowDiagnosticContext(sessionGeneration: 7, operationID: 23,
            purpose: "followScan", phase: "reacquiring", pulseIndex: 1, stale: true,
            outcome: "failed", reason: "no_yaw_progress")
        controller.emit(FollowDiagnosticEvent(event: "follow_person.association", context: context,
            payload: ["candidate_evaluations": .array([.object([
                "confidence": .number(0.987654321), "matched": .bool(false),
                "position": .null, "position_availability": .string("unavailable")
            ])]), "event": .string("must_not_override_envelope")]))
        coordinator.emit(FollowDiagnosticEvent(event: "follow_frame"))
        let record = try decode(records[0])
        XCTAssertEqual(record["session_generation"] as? Int, 7)
        XCTAssertEqual(record["operation_id"] as? Int, 23)
        XCTAssertEqual(record["purpose"] as? String, "followScan")
        XCTAssertEqual(record["phase"] as? String, "reacquiring")
        XCTAssertEqual(record["pulse_index"] as? Int, 1)
        XCTAssertEqual(record["stale"] as? Bool, true)
        XCTAssertEqual(record["outcome"] as? String, "failed")
        XCTAssertEqual(record["reason"] as? String, "no_yaw_progress")
        XCTAssertEqual(record["event"] as? String, "follow_person.association")
        let candidates = try XCTUnwrap(record["candidate_evaluations"] as? [[String: Any]])
        XCTAssertEqual(candidates[0]["confidence"] as? Double, 0.987654321)
        XCTAssertEqual(candidates[0]["matched"] as? Bool, false)
        XCTAssertTrue(candidates[0]["position"] is NSNull)
        XCTAssertEqual(try decode(records[1])["event_sequence"] as? Int, 1)
        XCTAssertEqual(try decode(records[1])["stream_id"] as? String, "coordinator")
    }

    func testPreciseAnglesWraparoundAndUnknownPoseProvenance() throws {
        let pre = RotationPoseDiagnosticSample(yaw: 3.13, readMonotonic: 4)
        let post = RotationPoseDiagnosticSample(yaw: -3.13, readMonotonic: 5)
        let measurement = RotationDiagnosticMeasurement(pre: pre, post: post, targetYaw: -3)
        var fields: [String: String] = [:]
        let emitter = FollowDiagnosticEmitter(streamID: "controller", monotonic: { 5 },
            utc: { Date() }, sink: { _, value in fields = value })
        emitter.emit(FollowDiagnosticEvent(event: "follow_scan.pulse_complete", payload: measurement.payload))
        let record = try decode(fields)
        XCTAssertEqual(try XCTUnwrap(record["signed_yaw_delta_rad"] as? Double), 0.023185307179586, accuracy: 1e-14)
        XCTAssertEqual(try XCTUnwrap(record["error_improvement_rad"] as? Double), 0.023185307179586, accuracy: 1e-14)
        XCTAssertEqual(record["pre_yaw_rad_display"] as? String, "+3.130000")
        XCTAssertEqual(record["post_yaw_rad_display"] as? String, "-3.130000")
        XCTAssertEqual(record["post_pose_read_monotonic_s"] as? Double, 5)
        XCTAssertTrue(record["pose_source_timestamp"] is NSNull)
        XCTAssertEqual(record["pose_source_age_status"] as? String, "unknown")
        XCTAssertEqual(record["pose_pairing"] as? String, "unknown")
        XCTAssertEqual(record["yaw_measurement_source"] as? String, "ar_visual_inertial")
        XCTAssertEqual(record["duration_clock"] as? String, "host_monotonic")
        let backwards = RotationDiagnosticMeasurement(
            pre: .init(yaw: 0.2, readMonotonic: 1), post: .init(yaw: 0.1, readMonotonic: 2), targetYaw: 1)
        XCTAssertEqual(backwards.errorImprovement!, -0.1, accuracy: 1e-14)
        XCTAssertEqual(RotationDiagnosticMeasurement.normalize(.pi), -.pi)
        XCTAssertEqual(RotationDiagnosticMeasurement.angleDisplay(0.0000004), "+0.000000")
        XCTAssertEqual(RotationDiagnosticMeasurement.angleDisplay(-0.0000004), "-0.000000")
    }

    func testMissingAndNonfiniteSamplesNeverBecomeZeroMeasurements() throws {
        var records: [[String: String]] = []
        let emitter = FollowDiagnosticEmitter(streamID: "controller", monotonic: { 2 },
            utc: { Date() }, sink: { _, value in records.append(value) })
        for yaw: Double? in [nil, .nan, .infinity, -.infinity] {
            let sample = RotationPoseDiagnosticSample(yaw: yaw, readMonotonic: 1)
            emitter.emit(FollowDiagnosticEvent(event: "follow_scan.failure", payload:
                RotationDiagnosticMeasurement(pre: sample, post: nil, targetYaw: 1).payload))
        }
        XCTAssertEqual(records.count, 4, "Unavailable measurements must still emit a record")
        for (index, fields) in records.enumerated() {
            let record = try decode(fields)
            for key in ["pre_yaw_rad", "post_yaw_rad", "pre_error_rad", "post_error_rad", "signed_yaw_delta_rad", "error_improvement_rad"] {
                XCTAssertTrue(record[key] is NSNull, key)
            }
            XCTAssertEqual(record["pre_pose_availability"] as? String, index == 0 ? "unavailable" : "nonfinite")
            XCTAssertEqual(record["post_pose_availability"] as? String, "unavailable")
        }
    }

    func testBoundedErrorsSingleRecordAndPrivacy() throws {
        var fields: [String: String] = [:]
        let emitter = FollowDiagnosticEmitter(streamID: "controller", monotonic: { 1 },
            utc: { Date() }, sink: { _, value in fields = value })
        let error = FollowDiagnosticError(code: String(repeating: "e", count: 200),
            message: "transport\nfailed\r" + String(repeating: "x", count: 1000))
        emitter.emit(FollowDiagnosticEvent(event: "follow_scan.failure", payload: [
            "error": .object(error.payload), "images": .string("secret"),
            "audio": .string("secret"), "transcript": .string("secret"),
            "response_body": .string("secret"), "raw_detector_count": .number(2),
            "candidate": .object(["projection_count": .number(3), "depth_samples": .array([.number(3)]),
                                  "url": .string("https://private/path"), "path": .string("/private"),
                                  "request_payload": .string("secret"), "biometric_identity": .string("secret")]),
            "missing_metric": .number(.nan)
        ]))
        let record = try decode(fields)
        let encodedError = try XCTUnwrap(record["error"] as? [String: Any])
        XCTAssertEqual((encodedError["code"] as? String)?.count, 64)
        XCTAssertEqual((encodedError["message"] as? String)?.count, 256)
        XCTAssertFalse(try XCTUnwrap(fields["payload"]).contains("\n"))
        for key in ["images", "audio", "transcript", "response_body"] {
            XCTAssertNil(record[key])
        }
        XCTAssertNil((record["candidate"] as? [String: Any])?["projection_count"])
        XCTAssertEqual(record["raw_detector_count"] as? Int, 2, "Task 5 permits the measured raw count")
        for key in ["depth_samples", "url", "path", "request_payload", "biometric_identity"] {
            XCTAssertNil((record["candidate"] as? [String: Any])?[key])
        }
        XCTAssertTrue(record["missing_metric"] is NSNull)
        XCTAssertEqual(record["missing_metric_availability"] as? String, "nonfinite")
    }

    func testRawAnglePrecisionAndIndependentMonotonicAndUTCClocks() throws {
        let clock = FollowDiagnosticTestClock()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "controller", monotonic: { clock.monotonic },
            utc: { clock.utc }, sink: sink.append)
        let sample = RotationPoseDiagnosticSample(yaw: 1.123456789123, readMonotonic: 0)
        emitter.emit(FollowDiagnosticEvent(event: "follow_scan.pulse_begin", payload:
            RotationDiagnosticMeasurement(pre: sample, post: nil, targetYaw: 2).payload))
        clock.monotonic = 1
        clock.utc = Date(timeIntervalSince1970: -1)
        emitter.emit(FollowDiagnosticEvent(event: "follow_scan.pulse_begin"))
        let first = try decode(sink.records[0].fields)
        let second = try decode(sink.records[1].fields)
        XCTAssertEqual(first["pre_yaw_rad"] as? Double, 1.123456789123)
        XCTAssertEqual(first["pre_yaw_rad_display"] as? String, "+1.123457")
        XCTAssertEqual(first["monotonic_s"] as? Double, 0)
        XCTAssertEqual(second["monotonic_s"] as? Double, 1)
        XCTAssertEqual(second["utc_time"] as? String, "1969-12-31T23:59:59.000Z")
        XCTAssertEqual(second["event_sequence"] as? Int, 2)
    }

    func testKnownObservationMetadataRetainsPairingWithoutInventingSourceTime() throws {
        var records: [[String: String]] = []
        let emitter = FollowDiagnosticEmitter(streamID: "controller", monotonic: { 5 },
            utc: { Date() }, sink: { _, fields in records.append(fields) })
        for pairing in RotationPosePairing.allCases {
            let sample = RotationPoseDiagnosticSample(yaw: 0.4, readMonotonic: 5,
                trackingState: "normal", frameID: "frame-42", observationAge: 0.125, pairing: pairing)
            emitter.emit(FollowDiagnosticEvent(event: "follow_scan.pulse_complete", payload:
                RotationDiagnosticMeasurement(pre: nil, post: sample, targetYaw: 1).payload))
        }
        XCTAssertEqual(records.count, 3)
        for (fields, pairing) in zip(records, RotationPosePairing.allCases) {
            let record = try decode(fields)
            XCTAssertEqual(record["tracking_state"] as? String, "normal")
            XCTAssertEqual(record["perception_frame_id"] as? String, "frame-42")
            XCTAssertEqual(record["observation_age_s"] as? Double, 0.125)
            XCTAssertEqual(record["pose_pairing"] as? String, pairing.rawValue)
            XCTAssertTrue(record["pose_source_timestamp"] is NSNull)
            XCTAssertEqual(record["pose_source_age_status"] as? String, "unknown")
        }
    }

    func testNonfiniteAvailabilityCannotBeOverriddenBySuppliedMetadata() throws {
        var fields: [String: String] = [:]
        let emitter = FollowDiagnosticEmitter(streamID: "controller", monotonic: { 1 },
            utc: { Date() }, sink: { _, value in fields = value })
        var metrics: [String: FollowDiagnosticValue] = [:]
        for index in 0..<64 {
            metrics["metric_\(index)"] = .number(.nan)
            metrics["metric_\(index)_availability"] = .string("available")
        }
        emitter.emit(FollowDiagnosticEvent(event: "follow_scan.failure", payload: metrics))
        let record = try decode(fields)
        for index in 0..<64 {
            XCTAssertTrue(record["metric_\(index)"] is NSNull)
            XCTAssertEqual(record["metric_\(index)_availability"] as? String, "nonfinite")
        }
    }

    private func decode(_ fields: [String: String]) throws -> [String: Any] {
        let payload = try XCTUnwrap(fields["payload"])
        return try XCTUnwrap(JSONSerialization.jsonObject(with: Data(payload.utf8)) as? [String: Any])
    }
}
