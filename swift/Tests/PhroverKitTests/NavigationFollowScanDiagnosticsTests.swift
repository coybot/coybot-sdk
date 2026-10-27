import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class NavigationFollowScanDiagnosticsTests: XCTestCase {
    func testNextBurstKeepsOperationMaximaButDoesNotRelabelPreviousResponseAsCurrent() async throws {
        let source = FollowRecoveryDiagnosticSourceFixture()
        let sink = FollowDiagnosticRecordingSink()
        var sends = 0
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 }, plan: { _, _ in nil },
            lastAckAt: { Date() }, sendCommand: { _ in sends += 1; source.uptime += 0.001 },
            stopRover: { source.uptime += 0.002 }, sleep: { duration in
                if sends > 0, Self.seconds(duration) >= 0.299 { source.pose = .init(position: .zero, yaw: sends == 1 ? 0.2 : 0.8) }
                await source.advance(duration)
            }, diagnosticEmitter: .init(streamID: "two-bursts", monotonic: { 100 }, utc: { Date() }, sink: sink.append),
            poseSample: { source.snapshot }, sourceNow: { source.uptime }, sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let result = await NavigationFollowMeMotion(navigation: controller).perform(.alignment(0.8), context:
            .init(sessionGeneration: 9, requestToken: 905, purpose: .followAlignment, phase: "aligning"))
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(sends, 2)
        let all = try records(sink)
        let second = try XCTUnwrap(all.first { $0["event"] as? String == "follow_scan.send_begin" && $0["burst_index"] as? Int == 2 })
        XCTAssertEqual(try XCTUnwrap(second["retained_response_rate_rad_s"] as? Double), 2.43902439024390, accuracy: 1e-12)
        XCTAssertEqual(second["requested_host_budget_s"] as? Double, 0.080)
        XCTAssertTrue(second["bracket_sample_count"] is NSNull)
        XCTAssertTrue(second["post_ack_sample_endpoints"] is NSNull)
        XCTAssertTrue(second["send_response_uptime_s"] is NSNull)
        XCTAssertTrue(second["sender_outcome"] is NSNull)
        let final = try XCTUnwrap(all.last { $0["event"] as? String == "follow_scan.burst_response" })
        XCTAssertEqual(try XCTUnwrap(final["retained_response_rate_rad_s"] as? Double), 7.31707317073171, accuracy: 1e-12)
    }

    func testThreeDegreeProbeTraceUsesReferenceOnlyAndSubMillisecondExcessWithoutInventedFloor() async throws {
        let source = FollowRecoveryDiagnosticSourceFixture()
        let sink = FollowDiagnosticRecordingSink()
        var reads = 0
        var sends = 0
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 }, plan: { _, _ in nil },
            lastAckAt: { Date() }, sendCommand: { _ in sends += 1; source.uptime += 0.100 }, stopRover: {},
            sleep: { duration in
                if sends > 0 { source.pose = .init(position: .zero, yaw: .pi / 60) }
                await source.advance(duration)
            }, diagnosticEmitter: .init(streamID: "small-probe", monotonic: { 300 }, utc: { Date() }, sink: sink.append),
            poseSample: { reads += 1; return source.snapshot }, sourceNow: { source.uptime }, sourceEvents: { source.events.stream },
            sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let result = await NavigationFollowMeMotion(navigation: controller).perform(.alignment(.pi / 60), context:
            .init(sessionGeneration: 9, requestToken: 904, purpose: .followAlignment, phase: "aligning"))
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(sends, 1)
        XCTAssertEqual(reads, 1)
        let plan = try XCTUnwrap(try records(sink).first { $0["event"] as? String == "follow_scan.burst_plan" })
        XCTAssertEqual(try XCTUnwrap(plan["selected_budget_s"] as? Double), 0.0011267585362157, accuracy: 1e-14)
        XCTAssertEqual(try XCTUnwrap(plan["excess_rad"] as? Double), 0.00235987755982988, accuracy: 1e-14)
        XCTAssertEqual(plan["tolerance_rad"] as? Double, 0.05)
        XCTAssertEqual(plan["response_rate_confidence"] as? String, "provisional_reference")
        XCTAssertTrue(plan["latency_allowance_s"] is NSNull)
        XCTAssertTrue(plan["minimum_host_budget_s"] is NSNull)
        XCTAssertEqual(plan["budget_floor_policy"] as? String, "no_hard_floor")
        XCTAssertEqual(plan["reference_rate_provenance"] as? String, "provisional_120deg_per_s_not_certified")
    }

    func testDroppedResponseFrameCannotBecomeObservedRateOrCoastInTelemetry() async throws {
        let source = FollowRecoveryDiagnosticSourceFixture()
        let sink = FollowDiagnosticRecordingSink()
        var sends = 0
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 }, plan: { _, _ in nil },
            lastAckAt: { Date() }, sendCommand: { _ in
                sends += 1
                source.uptime += 0.100
                source.snapshot = .init(pose: .init(position: .zero, yaw: 0.2),
                    frameID: .init(generation: 4, sequence: 14), sourceTimestamp: source.uptime,
                    trackingQuality: .normal, source: "dropped_frame_fixture")
                source.controller?.ingestFollowTurnSource(source.snapshot)
            }, stopRover: {}, sleep: { duration in
                if sends > 0 { source.pose = .init(position: .zero, yaw: 0.3) }
                await source.advance(duration)
            }, diagnosticEmitter: .init(streamID: "invalid-bracket", monotonic: { 100 }, utc: { Date() }, sink: sink.append),
            poseSample: { source.snapshot }, sourceNow: { source.uptime }, sourceEvents: { source.events.stream },
            sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let result = await NavigationFollowMeMotion(navigation: controller).perform(.scan(0.3), context:
            .init(sessionGeneration: 9, requestToken: 903, purpose: .followScan, phase: "scanning"))
        XCTAssertEqual(result.result, .arrived, "Fresh stopped arrival is independent of a dropped calibration frame")
        XCTAssertEqual(sends, 1)
        let rejected = try XCTUnwrap(try records(sink).first { $0["event"] as? String == "follow_scan.burst_response" })
        XCTAssertEqual(rejected["bracket_rejection_reason"] as? String, "incomplete_or_ambiguous")
        XCTAssertEqual(rejected["bracket_valid"] as? Bool, false)
        XCTAssertEqual(rejected["bracket_complete_for_rate"] as? Bool, false)
        XCTAssertTrue(rejected["effective_budget_response_rate_rad_s"] is NSNull)
        XCTAssertTrue(rejected["observed_post_ack_travel_rad"] is NSNull)
        XCTAssertEqual(rejected["response_rate_confidence"] as? String, "provisional_reference")
        let intervals = try XCTUnwrap(rejected["source_intervals"] as? [[String: Any]])
        XCTAssertTrue(intervals.allSatisfy { $0["valid"] as? Bool == false && $0["observed_rate_rad_s"] is NSNull })
        XCTAssertNil(result.failure)
    }

    func testOvershootTraceReportsMeasuredResolutionBeforeIneffectiveCorrection() async throws {
        let source = FollowRecoveryDiagnosticSourceFixture()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "model", monotonic: { 500 }, utc: { Date() }, sink: sink.append)
        var sends = 0
        let controller = FollowNavigationController(currentPose: { XCTFail("No legacy reads"); return nil },
            forwardClearance: { 2 }, plan: { _, _ in nil }, lastAckAt: { Date() },
            sendCommand: { _ in sends += 1; source.uptime += 0.100 },
            stopRover: { if sends > 0 { source.uptime += 0.020 } }, sleep: { duration in
                if sends > 0 { source.pose = .init(position: .zero, yaw: sends == 1 ? 0.5 : 0.3) }
                await source.advance(duration)
            }, diagnosticEmitter: emitter, poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let result = await NavigationFollowMeMotion(navigation: controller).perform(.scan(0.3), context:
            .init(sessionGeneration: 9, requestToken: 902, purpose: .followScan, phase: "scanning"))
        XCTAssertEqual(result.result, .failed(.rotationResolutionInsufficient))
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(sends, 1)
        let events = try records(sink)
        XCTAssertNotNil(result.failure)
        let planned = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.burst_plan" })
        XCTAssertEqual(planned["response_rate_confidence"] as? String, "provisional_reference")
        XCTAssertTrue(planned["maximum_send_duration_s"] is NSNull)
        let measured = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.burst_response" })
        XCTAssertEqual(measured["bracket_valid"] as? Bool, true)
        XCTAssertEqual(try XCTUnwrap(measured["retained_response_rate_rad_s"] as? Double), 4.16666666666667, accuracy: 1e-12)
        XCTAssertEqual(try XCTUnwrap(measured["maximum_send_duration_s"] as? Double), 0.1, accuracy: 1e-12)
        XCTAssertEqual(try XCTUnwrap(measured["maximum_stop_duration_s"] as? Double), 0.04, accuracy: 1e-12)
        XCTAssertEqual(measured["post_ack_travel_confidence"] as? String, "unknown")
        XCTAssertTrue(measured["observed_post_ack_travel_rad"] is NSNull)
        XCTAssertEqual(try XCTUnwrap(measured["overshoot_ceiling_s"] as? Double), 0.02845231237766351, accuracy: 1e-12)
        XCTAssertEqual(measured["post_source_frame_id"] as? String, "4:12")
        let plans = events.filter { $0["event"] as? String == "follow_scan.burst_plan" }
        let correction = try XCTUnwrap(plans.dropFirst().first)
        XCTAssertEqual(correction["allowance_policy"] as? String, "command_stop_window_overhead_once;coast_in_response")
        XCTAssertTrue(correction["selected_budget_s"] is NSNull)
        XCTAssertEqual(try XCTUnwrap(correction["candidate_budget_s"] as? Double), -0.021321531433504737, accuracy: 1e-12)
        XCTAssertEqual(correction["planning_decision"] as? String, "rotation_resolution_insufficient")
        XCTAssertEqual(correction["stop_outcome"] as? String, "confirmed")
        XCTAssertTrue(events.contains { $0["event"] as? String == "follow_scan.failure" })
    }

    func testRecordedAlignmentSequenceCorrectsAfterStopLatencySpike() async throws {
        let source = FollowRecoveryDiagnosticSourceFixture()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "alignment-latency", monotonic: { source.uptime },
            utc: { Date() }, sink: sink.append)
        let initialYaw = 1.2934638440161939
        let target = 1.6160948671430133 - initialYaw
        let measuredYaws = [1.3571727525186204, 1.3919495786239806, 1.438210875207493,
            1.4594608838320653, 1.517722599984274, 2.0431603854449154]
        var commands: [WheelCommand] = []
        var budgets: [Double] = []
        let controller = FollowNavigationController(currentPose: { XCTFail("No legacy reads"); return nil },
            forwardClearance: { 2 }, plan: { _, _ in nil }, lastAckAt: { Date() },
            sendCommand: { command in
                commands.append(command)
                let authorization = try XCTUnwrap(FollowTurnBurstTransportScope.authorization)
                budgets.append(authorization.deadline - authorization.sendEntryUptime)
                guard commands.count <= 7 else { throw URLError(.timedOut) }
                source.uptime += 0.031
            }, stopRover: {
                if !commands.isEmpty { source.uptime += commands.count == 6 ? 0.314495 : 0.020 }
            }, sleep: { duration in
                if !commands.isEmpty {
                    source.pose = .init(position: .zero, yaw: commands.count <= measuredYaws.count
                        ? measuredYaws[commands.count - 1] - initialYaw : target)
                }
                await source.advance(duration)
            }, diagnosticEmitter: emitter, poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let result = await NavigationFollowMeMotion(navigation: controller).perform(.alignment(target), context:
            .init(sessionGeneration: 1, requestToken: 1, purpose: .followAlignment, phase: "aligning"))
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(commands.count, 7)
        XCTAssertTrue(commands.prefix(6).allSatisfy { $0.left < 0 && $0.right > 0 })
        XCTAssertGreaterThan(commands.last?.left ?? 0, 0)
        XCTAssertLessThan(budgets.last ?? 1, 0.008)
        let correction = try XCTUnwrap(try records(sink).first {
            $0["budget_selection_reason"] as? String == "bounded_overshoot_correction"
        })
        XCTAssertEqual(correction["stop_outcome"] as? String, "confirmed")
        XCTAssertEqual(correction["responses_after_overshoot"] as? Int, 0)
        XCTAssertEqual(correction["tolerance_rad"] as? Double, 0.05)
        XCTAssertGreaterThan(try XCTUnwrap(correction["retained_response_overhead_s"] as? Double), 0.3)
    }

    func testRuntimeBurstTraceCapturesLateAckAndOnlyActuallyEnteredWaits() async throws {
        for purpose in [FollowMotionPurpose.followScan, .followAlignment] {
            let source = FollowRecoveryDiagnosticSourceFixture()
            let sink = FollowDiagnosticRecordingSink()
            let emitter = FollowDiagnosticEmitter(streamID: "runtime-burst", monotonic: { source.uptime },
                utc: { Date(timeIntervalSince1970: 100) }, sink: sink.append)
            var reads = 0
            var sends = 0
            let controller = FollowNavigationController(currentPose: { XCTFail("No legacy reads"); return nil },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
                sendCommand: { _ in sends += 1; source.uptime += 0.100 }, stopRover: {},
                sleep: { duration in
                    if sends > 0 { source.pose = .init(position: .zero, yaw: 0.3) }
                    await source.advance(duration)
                }, diagnosticEmitter: emitter, poseSample: { reads += 1; return source.snapshot },
                sourceNow: { source.uptime }, sourceEvents: { source.events.stream },
                sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let context = FollowMotionRequestContext(sessionGeneration: 9, requestToken: 901,
                purpose: purpose, phase: "reacquiring")
            let request: FollowMotionRequest = purpose == .followScan ? .scan(0.3) : .alignment(0.3)
            let result = await NavigationFollowMeMotion(navigation: controller).perform(request, context: context)
            XCTAssertEqual(result.result, .arrived)
            XCTAssertEqual(sends, 1)
            XCTAssertEqual(reads, 1, "Tracing must reuse ingress captures")
            let events = try records(sink)
            let pulse = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.pulse_complete" })
            XCTAssertEqual(pulse["purpose"] as? String, purpose.rawValue)
            XCTAssertEqual(pulse["pre_source_frame_id"] as? String, "4:11")
            XCTAssertEqual(pulse["post_source_frame_id"] as? String, "4:12")
            XCTAssertEqual(pulse["post_yaw_rad"] as? Double, 0.3)
            XCTAssertEqual(pulse["profile_pulse_wait_s"] as? Double, 0.080,
                "Compatibility field describes the configured maximum, not a fixed motor wait")
            XCTAssertEqual(pulse["profile_angular_tolerance_rad"] as? Double,
                purpose == .followScan ? 7 * .pi / 180 : 0.05)
            XCTAssertEqual(RoverConfig.followScanRotationProfile.pulseWait, 0.200,
                "Historical configuration remains inactive metadata")
            let ack = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.send_ack" })
            XCTAssertEqual(try XCTUnwrap(ack["send_host_duration_s"] as? Double), 0.100, accuracy: 1e-12)
            XCTAssertEqual(try XCTUnwrap(ack["requested_host_budget_s"] as? Double), 0.080, accuracy: 1e-12)
            XCTAssertEqual(try XCTUnwrap(ack["ack_overrun_s"] as? Double), 0.020, accuracy: 1e-12)
            XCTAssertEqual(ack["requested_additional_wait_s"] as? Double, 0)
            XCTAssertEqual(ack["remaining_budget_at_response_s"] as? Double, 0)
            XCTAssertEqual(ack["physical_motor_duration_confidence"] as? String, "unknown")
            XCTAssertEqual(ack["burst_clock"] as? String, "ar_system_uptime")
            XCTAssertEqual(ack["profile_historical_pulse_wait_active"] as? Bool, false)
            let obligation = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.burst_stop_obligation" })
            XCTAssertEqual(try XCTUnwrap(obligation["stop_obligation_observed_uptime_s"] as? Double), 8.4, accuracy: 1e-12)
            XCTAssertEqual(try XCTUnwrap(obligation["stop_obligation_uptime_s"] as? Double), 8.38, accuracy: 1e-12)
            XCTAssertFalse(events.contains { ($0["event"] as? String)?.hasPrefix("follow_scan.pulse_wait") == true },
                "Expired ACK cannot fabricate a wait stage")
            XCTAssertEqual(events.filter { $0["event"] as? String == "follow_scan.settle_begin" }.count, 2)
            XCTAssertEqual(events.filter { $0["event"] as? String == "follow_scan.settle_end" }.count, 2)
            XCTAssertEqual(events.last?["outcome"] as? String, "completed")
            XCTAssertEqual(events.last?["stop_outcome"] as? String, "confirmed")
        }
    }

    func testRecoveryDiagnosticsRejectSourceAndDeadlineWithoutInventingResolvedMovement() async throws {
        for fault in ["generation", "missing", "expired_feedback"] {
            let sink = FollowDiagnosticRecordingSink()
            let emitter = FollowDiagnosticEmitter(streamID: "rejected", monotonic: { 50 }, utc: { Date() }, sink: sink.append)
            var time = 8.0
            var reads = 0
            let events = AsyncStream<NavigationPoseSample>.makeStream()
            let snapshot: NavigationPoseSample = fault == "missing" ? .unavailable : .init(
                pose: .init(position: Vec2(2, 3), yaw: 0.4),
                frameID: .init(generation: fault == "generation" ? 5 : 4, sequence: 10),
                sourceTimestamp: 8, trackingQuality: .normal, source: "synthetic_ar")
            let controller = FollowNavigationController(currentPose: { .init(position: .zero, yaw: 0.4) },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: {
                    if fault == "expired_feedback" { time = 12 }; return Date()
                }, sendCommand: { _ in XCTFail("Rejected recovery cannot send") }, stopRover: {}, sleep: { _ in },
                diagnosticEmitter: emitter, poseSample: {
                    reads += 1
                    return snapshot
                }, sourceNow: { time }, sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
            let request = FollowRecoveryHeadingRequest(stageHeading: 1.2,
                authorization: .init(episodeID: UUID(), expectedGeneration: 4, deadline: 12, now: { time }, canContinue: { true }))
            let result = await NavigationFollowMeMotion(navigation: controller).performRecoveryHeading(request,
                context: .init(sessionGeneration: 7, requestToken: 14, purpose: .followScan, phase: "reacquiring"))
            XCTAssertNotEqual(result.result, .arrived)
            XCTAssertEqual(reads, 1)
            let record = try XCTUnwrap(sink.records.last { $0.event == "follow_scan.operation_complete" })
            let json = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap(record.fields["payload"]).utf8)) as? [String: Any])
            for key in ["requested_movement_rad", "measured_movement_rad", "segment_target_rad", "stage_arrived"] {
                XCTAssertTrue(json[key] is NSNull, key)
            }
            XCTAssertEqual(json["stage_target_rad"] as? Double, 1.2)
            XCTAssertEqual(json["stage_target_rad_availability"] as? String, "available")
            if fault == "generation" {
                XCTAssertEqual(json["controller_post_stop_frame_id"] as? String, "5:10")
                XCTAssertEqual(json["controller_post_stop_availability"] as? String, "source_generation_changed")
            } else if fault == "expired_feedback" {
                XCTAssertEqual(json["authorization_checked_uptime_s"] as? Double, 12)
                XCTAssertEqual(json["authorization_outcome"] as? String, "deadline_expired")
                XCTAssertEqual(json["authorization_remaining_s"] as? Double, 0)
            } else {
                XCTAssertTrue(json["controller_post_stop_timestamp_s"] is NSNull)
                XCTAssertEqual(json["controller_post_stop_availability"] as? String, "missing_pose")
            }
        }
    }

    func testRecoveryDiagnosticStreamUsesCapturedControllerReadsAndActualFinalYaw() async throws {
        let source = FollowRecoveryDiagnosticSourceFixture()
        var reads = 0
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "recovery-source", monotonic: { 50 },
            utc: { Date(timeIntervalSince1970: 50) }, sink: sink.append)
        let controller = FollowNavigationController(currentPose: { source.pose },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in source.pose = .init(position: Vec2(2, 3), yaw: 0.35) },
            stopRover: {}, sleep: source.advance,
            diagnosticEmitter: emitter, poseSample: {
                reads += 1
                return source.snapshot
            }, sourceNow: { source.uptime }, sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let episodeID = UUID()
        let request = FollowRecoveryHeadingRequest(stageHeading: 0.3,
            authorization: .init(episodeID: episodeID, expectedGeneration: 4, deadline: 12, now: { source.uptime }, canContinue: { true }),
            stageIndex: 2, segmentIndex: 1)
        let context = FollowMotionRequestContext(sessionGeneration: 7, requestToken: 13, purpose: .followScan, phase: "reacquiring")
        let result = await NavigationFollowMeMotion(navigation: controller).performRecoveryHeading(request, context: context)
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(reads, 1, "Diagnostics must not sample the pose provider")
        let record = try XCTUnwrap(sink.records.last { $0.event == "follow_scan.operation_complete" })
        let json = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(try XCTUnwrap(record.fields["payload"]).utf8)) as? [String: Any])
        XCTAssertEqual(json["episode_id"] as? String, episodeID.uuidString)
        XCTAssertEqual(json["stage_index"] as? Int, 2)
        XCTAssertEqual(json["segment_index"] as? Int, 1)
        XCTAssertEqual(json["deadline_s"] as? Double, 12)
        XCTAssertEqual(json["deadline_clock"] as? String, "system_uptime")
        XCTAssertEqual(json["authorization_checked_uptime_s"] as? Double, source.uptime)
        XCTAssertEqual(json["authorization_outcome"] as? String, "authorized")
        XCTAssertEqual(try XCTUnwrap(json["authorization_remaining_s"] as? Double), 12 - source.uptime, accuracy: 1e-12)
        XCTAssertEqual(json["request_token"] as? Int, 13)
        XCTAssertEqual(json["session_generation"] as? Int, 7)
        XCTAssertEqual(json["controller_post_stop_frame_id"] as? String, "4:11")
        XCTAssertEqual(json["controller_resolution_frame_id"] as? String, "4:11")
        XCTAssertEqual(json["controller_arrival_frame_id"] as? String, "4:13")
        XCTAssertEqual(json["controller_arrival_read_uptime_s"] as? Double, source.uptime)
        XCTAssertEqual(json["controller_arrival_timestamp_s"] as? Double, source.snapshot.sourceTimestamp)
        XCTAssertGreaterThan(source.snapshot.sourceTimestamp ?? 0, try XCTUnwrap(result.turnStopFence?.acknowledgementUptime))
        XCTAssertEqual(json["controller_arrival_source_identity"] as? String, source.snapshot.source)
        XCTAssertEqual(json["requested_movement_rad"] as? Double, 0.3)
        XCTAssertEqual(json["measured_movement_rad"] as? Double, 0.35)
        XCTAssertEqual(try XCTUnwrap(json["stage_error_rad"] as? Double), -0.05, accuracy: 1e-12)
        XCTAssertEqual(json["segment_arrived"] as? Bool, true)
        XCTAssertEqual(json["stage_arrived"] as? Bool, true)
        XCTAssertEqual(json["operation_start_monotonic_s"] as? Double, 50)
        XCTAssertEqual(json["duration_clock"] as? String, "host_monotonic")
    }

    func testIncompleteRecoveryFinalStopCannotReturnLateArrivalAfterOwnerReplacement() async throws {
        var results: [NavigationResult] = []
        for request in [FollowMotionRequest.alignment(0.3), .ready] {
            let gate = FollowDiagnosticSuspension()
            let source = FollowRecoveryDiagnosticSourceFixture()
            var stops = 0
            let controller = FollowNavigationController(currentPose: { source.pose }, forwardClearance: { 2 }, plan: { _, goal in [goal] },
                lastAckAt: { Date() }, sendCommand: { _ in
                    source.pose = .init(position: Vec2(0.1, 0), yaw: request.purpose == .followAlignment ? 0.3 : 0)
                    source.capture(after: 0.001)
                }, stopRover: {
                    stops += 1
                    if stops == 2 { await gate.suspend() }
                }, sleep: source.advance, poseSample: { source.snapshot }, sourceNow: { source.uptime },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 4, deadline: 12,
                now: { source.uptime }, canContinue: { true })
            let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 14, purpose: request.purpose, phase: "recovery")
            let caller = Task { await NavigationFollowMeMotion(navigation: controller).performContextual(request, context: context, recovery: authorization) }
            await gate.waitUntilEntered()
            let stop = Task { try await controller.stopAndConfirm() }
            for _ in 0..<20 { await Task.yield() }
            gate.release()
            try await stop.value
            results.append(await caller.value.result)
        }
        XCTAssertEqual(results, [.cancelled, .cancelled])
    }

    func testOrdinaryRelativeAlignmentRejectsYawOnlyLegacyProvenance() async {
        var yaw = 0.0
        let controller = FollowNavigationController(currentPose: { .init(position: Vec2(.nan, .nan), yaw: yaw) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in yaw = 0.3 }, stopRover: {}, sleep: { _ in })
        let result = await controller.rotateForFollowAlignment(by: 0.3)
        XCTAssertEqual(result, .failed(.trackingLost))
        XCTAssertEqual(yaw, 0, "Missing follow source cannot authorize a command")
    }

    func testRecoveryArrivalUsesOneFinalPostStopSampleForResultAndEvidence() async {
        let source = FollowRecoveryDiagnosticSourceFixture()
        var reads = 0
        var sent = false
        let controller = FollowNavigationController(currentPose: { source.pose },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in sent = true }, stopRover: {}, sleep: { duration in
                if sent, Self.seconds(duration) >= 0.299 { source.pose = .init(position: .zero, yaw: 0.3) }
                await source.advance(duration)
            }, poseSample: {
                reads += 1
                if reads > 1 { return .init(pose: source.pose, frameID: .init(generation: 5, sequence: 99),
                    sourceTimestamp: source.uptime, trackingQuality: .normal) }
                return source.snapshot
            }, sourceNow: { source.uptime }, sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let request = FollowRecoveryHeadingRequest(stageHeading: 0.3,
            authorization: .init(episodeID: UUID(), expectedGeneration: 4, deadline: 12, now: { source.uptime }, canContinue: { true }))
        let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 13, purpose: .followScan, phase: "reacquiring")
        let result = await NavigationFollowMeMotion(navigation: controller).performRecoveryHeading(request, context: context)
        XCTAssertTrue(result.result == .arrived && result.recovery?.stageArrived == true
            && result.recovery?.arrivalSource?.frameID == source.snapshot.frameID && reads == 1)
        XCTAssertGreaterThan(source.snapshot.sourceTimestamp ?? 0, result.turnStopFence?.acknowledgementUptime ?? .infinity)
    }

    func testInterruptedRecoveryReportsUnknownUnresolvedSegmentWithoutInventedRelativeRequest() async {
        let gate = FollowDiagnosticSuspension()
        let clock = FollowDiagnosticTestClock()
        let source = FollowRecoveryDiagnosticSourceFixture()
        source.pose = .init(position: Vec2(2, 3), yaw: 0.4)
        source.capture(after: 0)
        let controller = FollowNavigationController(currentPose: { .init(position: Vec2(2, 3), yaw: 0.4) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { await gate.suspend(); return Date() },
            sendCommand: { _ in XCTFail("Interrupted feedback cannot send") }, stopRover: {}, sleep: source.advance,
            poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let request = FollowRecoveryHeadingRequest(stageHeading: 1.2,
            authorization: .init(episodeID: UUID(), expectedGeneration: 4, deadline: 12, now: { source.uptime }, canContinue: { clock.monotonic == 0 }))
        let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 12, purpose: .followScan, phase: "reacquiring")
        let caller = Task { await NavigationFollowMeMotion(navigation: controller).performRecoveryHeading(request, context: context) }
        await gate.waitUntilEntered()
        clock.monotonic = 1
        gate.release()
        let result = await caller.value
        XCTAssertTrue(result.result == .cancelled && result.context.requestedRotation == nil && result.context.targetYaw == nil
            && result.recovery?.postStopSource.pose?.yaw == 0.4 && result.recovery?.resolutionSource == nil
            && result.recovery?.segmentHeading == nil && result.recovery?.stageArrived == nil)
    }

    func testAbsoluteRecoveryKeepsSegmentTargetThroughOppositeSignCorrectionAndInclusiveTolerance() async {
        for withinTolerance in [false, true] {
            let source = FollowRecoveryDiagnosticSourceFixture()
            var commands: [Double] = []
            var waits: [Double] = []
            let controller = FollowNavigationController(currentPose: { source.pose },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
                sendCommand: { commands.append($0.right) }, stopRover: {}, sleep: { duration in
                    waits.append(Self.seconds(duration))
                    if Self.seconds(duration) >= 0.299, !commands.isEmpty {
                        source.pose = .init(position: .zero, yaw: commands.count == 1 ? 0.7 : .pi / 6)
                    }
                    await source.advance(duration)
                }, poseSample: { source.snapshot }, sourceNow: { source.uptime },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let request = FollowRecoveryHeadingRequest(stageHeading: withinTolerance ? 7 * .pi / 180 : 1.2,
                authorization: .init(episodeID: UUID(), expectedGeneration: 4, deadline: 12, now: { source.uptime }, canContinue: { true }))
            let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 10, purpose: .followScan, phase: "reacquiring")
            let result = await NavigationFollowMeMotion(navigation: controller).performRecoveryHeading(request, context: context)
            XCTAssertEqual(result.result, .arrived)
            XCTAssertEqual(result.recovery?.segmentHeading, withinTolerance ? 0 : .pi / 6)
            XCTAssertEqual(result.recovery?.stageArrived, withinTolerance)
            XCTAssertEqual(commands, withinTolerance ? [] : [0.25, -0.25])
            if !withinTolerance {
                XCTAssertEqual(waits.count, 5)
                XCTAssertEqual(waits[1], 0.080, accuracy: 1e-12)
                XCTAssertGreaterThan(waits[3], 0)
                XCTAssertLessThan(waits[3], waits[1], "Measured overshoot must shrink the opposite correction")
            }
        }
    }

    func testAbsoluteRecoveryCallerCancellationDrainsStopAndRetainsFailedLatch() async {
        var outcomes: [Bool] = []
        for boundary in ["pre_stop", "feedback", "pulse_wait"] {
            for failStop in [false, true] {
                let gate = FollowDiagnosticSuspension()
                let source = FollowRecoveryDiagnosticSourceFixture()
                var sends = 0
                var stops = 0
                let controller = FollowNavigationController(currentPose: { .init(position: .zero, yaw: 0) },
                    forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: {
                        if boundary == "feedback", !gate.entered { await gate.suspend() }; return Date()
                    }, sendCommand: { _ in sends += 1 }, stopRover: {
                        stops += 1
                        if boundary == "pre_stop", stops == 1 { await gate.suspend() }
                        if Task.isCancelled { throw CancellationError() }
                        if failStop, FollowMotionTaskScope.evidence?.fenced == true { throw URLError(.cannotConnectToHost) }
                    }, sleep: { duration in
                        if boundary == "pulse_wait", sends == 1, stops == 1, !gate.entered { await gate.suspend() }
                        await source.advance(duration)
                    }, poseSample: { source.snapshot }, sourceNow: { source.uptime },
                    sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
                source.controller = controller
                let request = FollowRecoveryHeadingRequest(stageHeading: 0.3,
                    authorization: .init(episodeID: UUID(), expectedGeneration: 4, deadline: 12,
                        now: { source.uptime }, canContinue: { true }))
                let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 11, purpose: .followScan, phase: "reacquiring")
                let motion = NavigationFollowMeMotion(navigation: controller)
                let caller = Task { await motion.performRecoveryHeading(request, context: context) }
                await gate.waitUntilEntered()
                caller.cancel()
                for _ in 0..<20 { await Task.yield() }
                gate.release()
                let result = await caller.value
                var blocked = true
                if failStop {
                    blocked = await motion.performRecoveryHeading(request, context: context).result == .failed(.commandFailed)
                }
                outcomes.append(result.result == (failStop ? .failed(.commandFailed) : .cancelled)
                    && result.stopOutcome == (failStop ? .failed : .confirmed) && blocked
                    && sends == (boundary == "pulse_wait" ? 1 : 0))
            }
        }
        XCTAssertEqual(outcomes, [true, true, true, true, true, true])
    }

    func testRecoveryRevalidatesSourceAgeImmediatelyBeforeEachMotorSend() async {
        var counts: [Int] = []
        for purpose in [FollowMotionPurpose.followScan, .followAlignment, .followReady] {
            let source = FollowRecoveryDiagnosticSourceFixture()
            var sends = 0
            var guardReads = 0
            let expireOnRead = purpose == .followReady ? 1 : 3
            let controller = FollowNavigationController(currentPose: { source.pose }, forwardClearance: {
                guardReads += 1
                if guardReads == expireOnRead { source.uptime += 0.501 }
                return 2
            },
                plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in
                    sends += 1
                    source.pose = .init(position: Vec2(0.1, 0), yaw: purpose == .followReady ? 0 : 0.3)
                }, stopRover: {}, sleep: source.advance, poseSample: { source.snapshot }, sourceNow: { source.uptime },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 4, deadline: 12,
                now: { source.uptime }, canContinue: { true })
            let motion = NavigationFollowMeMotion(navigation: controller)
            let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 9, purpose: purpose, phase: "recovery")
            if purpose == .followScan {
                _ = await motion.performRecoveryHeading(.init(stageHeading: 0.3, authorization: authorization), context: context)
            } else {
                _ = await motion.performContextual(purpose == .followReady ? .ready : .alignment(0.3), context: context, recovery: authorization)
            }
            counts.append(sends)
            XCTAssertGreaterThanOrEqual(guardReads, expireOnRead, "Must reach the intended pre-send guard")
        }
        XCTAssertEqual(counts, [0, 0, 0])
    }

    func testIncompleteRecoveryCannotRestoreFromGenerationChangedDuringFinalStop() async {
        var results: [NavigationResult] = []
        for request in [FollowMotionRequest.alignment(0.3), .ready] {
            let source = FollowRecoveryDiagnosticSourceFixture()
            var stops = 0
            var sends = 0
            let terminalStop = request.purpose == .followReady ? 3 : 2
            let controller = FollowNavigationController(currentPose: { source.pose }, forwardClearance: { 2 }, plan: { _, goal in [goal] },
                lastAckAt: { Date() }, sendCommand: { _ in
                    sends += 1
                    source.pose = .init(position: Vec2(0.1, 0), yaw: request.purpose == .followAlignment ? 0.3 : 0)
                    source.capture(after: 0.001)
                }, stopRover: {
                    stops += 1; await Task.yield()
                    if stops >= terminalStop { source.generation = 5; source.capture(after: 0.001) }
                }, sleep: source.advance, poseSample: { source.snapshot }, sourceNow: { source.uptime },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 4, deadline: 12,
                now: { source.uptime }, canContinue: { true })
            let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 8, purpose: request.purpose, phase: "recovery")
            results.append(await NavigationFollowMeMotion(navigation: controller).performContextual(request, context: context, recovery: authorization).result)
            XCTAssertEqual(sends, 1)
            XCTAssertEqual(stops, terminalStop, "Generation fault must reach the actual terminal stop, after readiness pulse stopping")
        }
        XCTAssertEqual(results, [.failed(.trackingLost), .failed(.trackingLost)])
    }

    func testRecoveryCannotClaimArrivalFromSourceInvalidatedDuringFinalStop() async {
        var outcomes: [Bool] = []
        for fault in ["age", "generation", "future"] {
            let source = FollowRecoveryDiagnosticSourceFixture()
            var stops = 0
            var sends = 0
            let controller = FollowNavigationController(currentPose: { source.pose },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
                sendCommand: { _ in
                    sends += 1
                    source.pose = .init(position: .zero, yaw: 0.3)
                    source.capture(after: 0.001)
                }, stopRover: {
                    stops += 1; await Task.yield()
                    if stops == 2 {
                        if fault == "age" { source.uptime += 0.501 }
                        if fault == "generation" { source.generation = 5; source.capture(after: 0.001) }
                        if fault == "future" {
                            source.snapshot = .init(pose: source.pose, frameID: .init(generation: 4, sequence: 99),
                                sourceTimestamp: source.uptime + 0.001, trackingQuality: .normal)
                            source.controller?.ingestFollowTurnSource(source.snapshot)
                        }
                    }
                }, sleep: source.advance, poseSample: { source.snapshot }, sourceNow: { source.uptime },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let request = FollowRecoveryHeadingRequest(stageHeading: 0.3, authorization: .init(episodeID: UUID(),
                expectedGeneration: 4, deadline: 12, now: { source.uptime }, canContinue: { true }))
            let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 7, purpose: .followScan, phase: "reacquiring")
            let result = await NavigationFollowMeMotion(navigation: controller).performRecoveryHeading(request, context: context)
            outcomes.append(result.result == .failed(.trackingLost) && result.recovery?.stageArrived == nil
                && result.recovery?.segmentArrived == false && result.stopOutcome == .confirmed)
            XCTAssertEqual(sends, 1)
            XCTAssertEqual(stops, 2)
        }
        XCTAssertEqual(outcomes, [true, true, true])
    }

    func testLegacyRecoveryStopsWithoutTurningOrInventingSourceFacts() async {
        let motion = FollowMotionFake()
        let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 4, deadline: 12,
            now: { 8 }, canContinue: { true })
        let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 6, purpose: .followScan, phase: "reacquiring")
        let heading = await motion.performRecoveryHeading(.init(stageHeading: 0.3, authorization: authorization), context: context)
        let alignment = await motion.performContextual(.alignment(0.3), context: context, recovery: authorization)
        let ready = await motion.performContextual(.ready, context: context, recovery: authorization)
        XCTAssertEqual([motion.rotations.count, motion.alignments.count, motion.readySignals, motion.stops,
            heading.recovery == nil && heading.context.controllerOperationID == nil && heading.context.targetYaw == nil ? 1 : 0,
            alignment.result == .cancelled && ready.result == .cancelled ? 1 : 0], [0, 0, 0, 3, 1, 1])
    }

    func testRecoveryAuthorizationAlsoFencesIncompleteAlignmentAndReadyAfterFeedback() async {
        var outcomes: [Int] = []
        for request in [FollowMotionRequest.alignment(0.3), .ready] {
            for fault in ["deadline", "ownership", "source_generation"] {
                let source = FollowRecoveryDiagnosticSourceFixture()
                var allowed = true
                var sends = 0
                var feedbackReads = 0
                let controller = FollowNavigationController(currentPose: { source.pose }, forwardClearance: { 2 }, plan: { _, goal in [goal] },
                    lastAckAt: {
                        await Task.yield()
                        feedbackReads += 1
                        if fault == "deadline" { source.uptime = 12 }
                        if fault == "ownership" { allowed = false }
                        if fault == "source_generation" { source.generation = 5; source.capture(after: 0.001) }
                        return Date()
                    }, sendCommand: { _ in
                        sends += 1
                        source.pose = .init(position: Vec2(0.1, 0), yaw: request.purpose == .followAlignment ? 0.3 : 0)
                    }, stopRover: {}, sleep: source.advance,
                    poseSample: { source.snapshot }, sourceNow: { source.uptime },
                    sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
                source.controller = controller
                let authorization = FollowRecoveryAuthorization(episodeID: UUID(), expectedGeneration: 4, deadline: 12,
                    now: { source.uptime }, canContinue: { allowed })
                let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 5, purpose: request.purpose, phase: "recovery")
                _ = await NavigationFollowMeMotion(navigation: controller).performContextual(request, context: context, recovery: authorization)
                outcomes.append(sends)
                XCTAssertGreaterThan(feedbackReads, 0, "Fault must occur at the actual feedback await")
            }
        }
        XCTAssertEqual(outcomes, [0, 0, 0, 0, 0, 0])
    }

    func testRecoveryDeadlineAndOwnershipAreRecheckedAfterEverySuspension() async {
        var outcomes: [Int] = []
        for boundary in ["pre_stop", "feedback", "detection", "send", "pulse_wait", "settle", "final_stop", "nonfinite_heading"] {
            if boundary == "pulse_wait" {
                outcomes.append(contentsOf: await recoveryCancellationAtActualPulseWait())
                continue
            }
            var time = 8.0
            let source = FollowRecoveryDiagnosticSourceFixture()
            var allowed = true
            var yaw = 0.0
            var stops = 0
            var sends = 0
            let controller = FollowNavigationController(currentPose: { .init(position: .zero, yaw: yaw) },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: {
                    await Task.yield()
                    if boundary == "feedback" { time = 12 }
                    if boundary == "detection" { allowed = false }
                    return Date()
                }, sendCommand: { _ in
                    sends += 1; await Task.yield(); if boundary == "send" { time = 12 }
                    yaw = 0.3
                },
                stopRover: {
                    stops += 1; await Task.yield()
                    if boundary == "pre_stop", stops == 1 { time = 12 }
                    if boundary == "final_stop", stops == 2 { time = 12 }
                }, sleep: { duration in
                    await Task.yield()
                    guard !Task.isCancelled else { return }
                    if sends > 0, boundary == "settle", Self.seconds(duration) >= 0.299 { time = 12 }
                    let durationSeconds = Self.seconds(duration)
                    time += durationSeconds
                    source.uptime = time - durationSeconds
                    source.pose = .init(position: .zero, yaw: yaw)
                    source.capture(after: durationSeconds)
                }, poseSample: { source.snapshot }, sourceNow: { time },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let request = FollowRecoveryHeadingRequest(stageHeading: boundary == "nonfinite_heading" ? .nan : 0.3,
                authorization: .init(episodeID: UUID(), expectedGeneration: 4, deadline: 12,
                    now: { time }, canContinue: { allowed }))
            let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 4, purpose: .followScan, phase: "reacquiring")
            let result = await NavigationFollowMeMotion(navigation: controller).performRecoveryHeading(request, context: context)
            XCTAssertEqual(result.result, .cancelled, "Expired or denied recovery at \(boundary)")
            outcomes.append(sends)
            outcomes.append(result.result == .cancelled ? 1 : 0)
        }
        XCTAssertEqual(outcomes, [0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1])
    }

    /// Independent safety timers use the same sleeper as pulse waiting. They
    /// must never advance a shared fake clock or identify this stage by duration.
    /// Drive source time explicitly after the real controller reports its wait.
    private func recoveryCancellationAtActualPulseWait() async -> [Int] {
        var outcome = [0, 0]
        for expireDeadline in [true, false] {
            let initialSettle = expectation(description: "recovery initial post-stop source wait")
            let pulseWait = expectation(description: "recovery actual pulse wait")
            let completed = expectation(description: "recovery cancellation drained")
            let source = FollowRecoveryDiagnosticSourceFixture()
            let clock = FollowRecoveryDriverClock()
            var allowed = true
            var sends = 0
            var stops = 0
            var initialSettleReached = false
            var pulseWaitReached = false
            var result: FollowMotionResult?
            let emitter = FollowDiagnosticEmitter(streamID: "recovery-controlled-pulse-wait",
                monotonic: { clock.now }, utc: { Date() }) { event, _ in
                    if event == "follow_scan.settle_begin", sends == 0, !initialSettleReached {
                        initialSettleReached = true
                        initialSettle.fulfill()
                    }
                    if event == "follow_scan.pulse_wait_begin", !pulseWaitReached {
                        pulseWaitReached = true
                        pulseWait.fulfill()
                    }
                }
            let controller = FollowNavigationController(currentPose: { source.pose }, forwardClearance: { 2 },
                plan: { _, goal in [goal] }, lastAckAt: { Date(timeIntervalSince1970: clock.now) },
                sendCommand: { _ in sends += 1 }, stopRover: { stops += 1 },
                sleep: { duration in try? await Task.sleep(for: duration) },
                now: { Date(timeIntervalSince1970: clock.now) }, diagnosticEmitter: emitter,
                poseSample: { source.snapshot }, sourceNow: { clock.now },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot },
                transportUptime: { clock.now })
            source.controller = controller
            let request = FollowRecoveryHeadingRequest(stageHeading: 0.3,
                authorization: .init(episodeID: UUID(), expectedGeneration: 4, deadline: 12,
                    now: { clock.now }, canContinue: { allowed }))
            let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 4,
                purpose: .followScan, phase: "reacquiring")
            let caller = Task {
                result = await NavigationFollowMeMotion(navigation: controller).performRecoveryHeading(request, context: context)
                completed.fulfill()
            }
            defer {
                caller.cancel()
                source.events.continuation.finish()
            }
            await fulfillment(of: [initialSettle], timeout: 2)
            guard initialSettleReached else { return [sends, 0] }
            // One real, settled source frame admits the first burst. Neither its
            // timer nor any monitor advances this clock while awaiting the event.
            clock.now = 8.301
            source.capture(after: 0.301)
            await fulfillment(of: [pulseWait], timeout: 2)
            guard pulseWaitReached else { return [sends, 0] }
            XCTAssertEqual(sends, 1)
            XCTAssertEqual(stops, 1, "Inject before the burst stop, while pulse_wait owns the suspension")
            if expireDeadline { clock.now = 12; source.uptime = 12 }
            else { allowed = false }
            source.capture(after: 0) // Wake the actual source wait after invalidation.
            await fulfillment(of: [completed], timeout: 2)
            XCTAssertEqual(result?.result, .cancelled,
                expireDeadline ? "Deadline expires during actual pulse wait" : "Ownership revoked during actual pulse wait")
            XCTAssertEqual(sends, 1, "No second burst after recovery loses authority")
            outcome = [sends, result?.result == .cancelled ? 1 : 0]
        }
        return outcome
    }

    func testRecoveryRequiresRealFreshExpectedGenerationAtStopAndAfterFeedback() async {
        var counts: [Int] = []
        for fault in ["wrong_generation", "generation_after_feedback", "aged_after_feedback", "future", "nonfinite", "legacy", "limited"] {
            let source = FollowRecoveryDiagnosticSourceFixture()
            source.snapshot = fault == "legacy" ? .legacy(source.pose) : .init(pose: source.pose,
                frameID: .init(generation: fault == "wrong_generation" ? 5 : 4, sequence: 10),
                sourceTimestamp: fault == "future" ? 8.001 : (fault == "nonfinite" ? .nan : 8),
                trackingQuality: fault == "limited" ? .limited : .normal, source: "synthetic_ar")
            var sends = 0
            var feedbackReads = 0
            let controller = FollowNavigationController(currentPose: { source.pose },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: {
                    feedbackReads += 1
                    if fault == "generation_after_feedback" { source.generation = 5; source.capture(after: 0.001) }
                    if fault == "aged_after_feedback" { source.uptime = 8.501 }
                    return Date()
                }, sendCommand: { _ in sends += 1; source.pose = .init(position: .zero, yaw: 0.3) }, stopRover: {}, sleep: source.advance,
                poseSample: { source.snapshot }, sourceNow: { source.uptime },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let request = FollowRecoveryHeadingRequest(stageHeading: 0.3, authorization: .init(episodeID: UUID(),
                expectedGeneration: 4, deadline: 12, now: { source.uptime }, canContinue: { true }))
            let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 3, purpose: .followScan, phase: "reacquiring")
            _ = await NavigationFollowMeMotion(navigation: controller).performRecoveryHeading(request, context: context)
            counts.append(sends)
            if fault.hasSuffix("after_feedback") { XCTAssertGreaterThan(feedbackReads, 0) }
        }
        XCTAssertEqual(counts, [0, 0, 0, 0, 0, 0, 0])
    }

    func testAbsoluteRecoveryResolvesAfterAcknowledgedStopAndFeedback() async {
        let stopGate = FollowDiagnosticSuspension()
        let feedbackGate = FollowDiagnosticSuspension()
        var pose = Pose2D(position: .zero, yaw: 0)
        var stops = 0
        var sentYaws: [Double] = []
        let source = FollowRecoveryDiagnosticSourceFixture()
        let controller = FollowNavigationController(currentPose: { pose }, forwardClearance: { 2 }, plan: { _, goal in [goal] },
            lastAckAt: { if !feedbackGate.entered { await feedbackGate.suspend() }; return Date() },
            sendCommand: { _ in sentYaws.append(pose.yaw) }, stopRover: {
                stops += 1; if stops == 1 { await stopGate.suspend() }
            }, sleep: { duration in
                if !sentYaws.isEmpty, Self.seconds(duration) >= 0.299 { pose = .init(position: Vec2(3, 4), yaw: 1.2) }
                source.pose = pose
                await source.advance(duration)
            }, poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let request = FollowRecoveryHeadingRequest(stageHeading: 1.2, authorization: .init(episodeID: UUID(),
            expectedGeneration: 4, deadline: 12, now: { source.uptime }, canContinue: { true }))
        let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 1, purpose: .followScan, phase: "reacquiring")
        let task = Task { await NavigationFollowMeMotion(navigation: controller).performRecoveryHeading(request, context: context) }
        await stopGate.waitUntilEntered()
        pose = .init(position: Vec2(1, 2), yaw: 0.2)
        source.pose = pose
        source.capture(after: 0.001)
        stopGate.release()
        await feedbackGate.waitUntilEntered()
        pose = .init(position: Vec2(2, 3), yaw: 0.8)
        source.pose = pose
        source.capture(after: 0.001)
        feedbackGate.release()
        let result = await task.value
        XCTAssertEqual(sentYaws + [result.recovery?.postStopSource.pose?.yaw ?? -1,
            result.recovery?.postStopSource.pose?.position.x ?? -1, result.recovery?.postStopSource.pose?.position.y ?? -1,
            result.recovery?.resolutionSource?.pose?.position.x ?? -1, result.recovery?.resolutionSource?.pose?.position.y ?? -1,
            result.recovery?.segmentHeading ?? -1], [0.8, 0.8, 2, 3, 2, 3, 1.2],
            "Fixed stage must use actual post-stop and post-feedback source geometry")
    }

    func testRecoveryResultReportsActualSegmentArrivalWithoutClaimingStageArrival() async {
        var pose = Pose2D(position: Vec2(2, 3), yaw: 0)
        let source = FollowRecoveryDiagnosticSourceFixture()
        source.pose = pose
        source.capture(after: 0)
        var sent = false
        let controller = FollowNavigationController(currentPose: { pose }, forwardClearance: { 2 }, plan: { _, goal in [goal] },
            lastAckAt: { Date() }, sendCommand: { _ in sent = true }, stopRover: {},
            sleep: { duration in
                if sent, Self.seconds(duration) >= 0.299 { pose = .init(position: Vec2(4, 5), yaw: .pi / 6) }
                source.pose = pose
                await source.advance(duration)
            }, poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let request = FollowRecoveryHeadingRequest(stageHeading: 1.2, authorization: .init(episodeID: UUID(),
            expectedGeneration: 4, deadline: 12, now: { source.uptime }, canContinue: { true }))
        let context = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 2, purpose: .followScan, phase: "reacquiring")
        let result = await NavigationFollowMeMotion(navigation: controller).performRecoveryHeading(request, context: context)
        XCTAssertEqual([result.recovery?.postStopSource.pose?.position.x, result.recovery?.postStopSource.pose?.position.y,
            result.recovery?.requestedDelta, result.recovery?.segmentHeading,
            result.recovery?.arrivalSource?.pose?.position.x, result.recovery?.arrivalSource?.pose?.yaw,
            result.recovery?.segmentArrived == true ? 1 : 0, result.recovery?.stageArrived == false ? 1 : 0,
            result.result == .arrived && result.stopOutcome == .confirmed ? 1 : 0],
            [2, 3, .pi / 6, .pi / 6, 4, .pi / 6, 1, 1, 1])
    }

    func testCancelledHighPriorityContextualCallerCannotLaunchAfterPreStop() async {
        for request in [FollowMotionRequest.alignment(0.3), .following(Vec2(1, 0), 1.25)] {
            let gate = FollowDiagnosticSuspension()
            var stops = 0
            var sends = 0
            var pose = Pose2D(position: .zero, yaw: 0)
            var movingStates = 0
            let controller = FollowNavigationController(currentPose: { pose }, forwardClearance: { 2 },
                plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in
                    sends += 1
                    pose = Pose2D(position: Vec2(1, 0), yaw: 0.3)
                }, stopRover: {
                    stops += 1
                    if stops == 1 { await gate.suspend() }
                }, sleep: { _ in await Task.yield() })
            let states = controller.safetyStates()
            let observer = Task { for await state in states { if state == .moving { movingStates += 1 } } }
            let context = FollowMotionRequestContext(sessionGeneration: 43, requestToken: 803,
                purpose: request.purpose, phase: "pre-stop")
            let caller = Task(priority: .high) {
                await NavigationFollowMeMotion(navigation: controller).perform(request, context: context)
            }
            await gate.waitUntilEntered()
            // The cancellation actor hop inherits this lower priority. The high-priority
            // pre-stop waiter may resume first and must check its own cancellation flag.
            let cancellation = Task(priority: .background) { caller.cancel(); gate.release() }
            await cancellation.value
            let result = await caller.value
            for _ in 0..<20 { await Task.yield() }
            observer.cancel()
            await observer.value
            XCTAssertEqual(result.result, .cancelled)
            XCTAssertEqual(sends, 0)
            XCTAssertEqual(movingStates, 0, "Cancelled pre-stop cannot authorize a new loop: \(request.purpose)")
            XCTAssertEqual(result.stopOutcome, .confirmed)
        }
    }

    func testGenericReplacementWaitsForRunningCallerCancellationStopAndFailsClosed() async throws {
        for rotate in [false, true] {
            for failStop in [false, true] {
                let clock = FollowDiagnosticTestClock()
                let sink = FollowDiagnosticRecordingSink()
                let scanGate = FollowDiagnosticSuspension()
                let stopGate = FollowDiagnosticSuspension()
                let replacementSendGate = FollowDiagnosticSuspension()
                let source = FollowRecoveryDiagnosticSourceFixture()
                let emitter = FollowDiagnosticEmitter(streamID: "generic-replacement", monotonic: { clock.monotonic },
                    utc: { clock.utc }, sink: sink.append)
                var pose = Pose2D(position: .zero, yaw: 0)
                var sends = 0
                let controller = FollowNavigationController(currentPose: { pose }, forwardClearance: { 2 },
                    plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in
                        sends += 1
                        if sends == 1 { await scanGate.suspend() }
                        if sends == 2 {
                            await replacementSendGate.suspend()
                            pose = Pose2D(position: Vec2(1, 0), yaw: 0.3)
                        }
                    }, stopRover: {
                        if Task.isCancelled { throw CancellationError() }
                        if FollowMotionTaskScope.evidence?.fenced == true {
                            await stopGate.suspend()
                            if failStop { throw CancellationError() }
                        }
                    }, sleep: source.advance, diagnosticEmitter: emitter,
                    poseSample: { source.snapshot }, sourceNow: { source.uptime },
                    sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
                source.controller = controller
                let caller = Task { await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3) }
                await scanGate.waitUntilEntered()
                caller.cancel()
                for _ in 0..<40 { await Task.yield() }
                scanGate.release()
                await stopGate.waitUntilEntered()
                let replacement = Task {
                    if rotate { return await controller.rotateAndWait(by: 0.3) }
                    return await controller.navigateAndWait(to: Vec2(1, 0))
                }
                for _ in 0..<80 { await Task.yield() }
                XCTAssertEqual(sends, 1, "Generic replacement must wait for old independent acknowledgement")
                XCTAssertFalse(replacementSendGate.entered)
                stopGate.release()
                for _ in 0..<80 { await Task.yield() }
                replacementSendGate.release()
                let oldResult = await caller.value
                let replacementResult = await replacement.value
                XCTAssertEqual(oldResult, failStop ? .failed(.commandFailed) : .cancelled)
                XCTAssertEqual(replacementResult, failStop ? .failed(.commandFailed) : .arrived)
                XCTAssertEqual(sends, failStop ? 1 : 2)
                if failStop {
                    let blocked = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3)
                    XCTAssertEqual(blocked, .failed(.commandFailed))
                    XCTAssertEqual(sends, 1)
                }
            }
        }
    }

    func testQueuedCallerCancellationCleanupCannotStopAReplacementReservation() async throws {
        let clock = FollowDiagnosticTestClock()
        let sink = FollowDiagnosticRecordingSink()
        let sendGate = FollowDiagnosticSuspension()
        let replacementStopGate = FollowDiagnosticSuspension()
        var beginReplacement: (() -> Void)?
        var replacement: Task<FollowMotionResult, Never>?
        var yaw = 0.0
        var oldIndependentStops = 0
        let source = FollowRecoveryDiagnosticSourceFixture()
        var newSends = 0
        let emitter = FollowDiagnosticEmitter(streamID: "queued-cancellation", monotonic: { clock.monotonic },
            utc: { clock.utc }, sink: { event, fields in
                sink.append(event, fields: fields)
                if event == "follow_scan.cancel" { beginReplacement?(); beginReplacement = nil }
            })
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: yaw) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in
                if FollowMotionTaskScope.evidence?.context.request?.requestToken == 801 { await sendGate.suspend() }
                else { newSends += 1 }
            }, stopRover: {
                if Task.isCancelled { throw CancellationError() }
                if FollowMotionTaskScope.evidence?.context.request?.requestToken == 801 { oldIndependentStops += 1 }
                if FollowMotionTaskScope.evidence?.context.request?.requestToken == 802,
                   !replacementStopGate.entered { await replacementStopGate.suspend() }
            }, sleep: { duration in
                if newSends > 0, Self.seconds(duration) >= 0.299 { yaw = 0.3 }
                source.pose = .init(position: .zero, yaw: yaw)
                await source.advance(duration)
            }, diagnosticEmitter: emitter, poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let motion = NavigationFollowMeMotion(navigation: controller)
        let oldContext = FollowMotionRequestContext(sessionGeneration: 41, requestToken: 801,
            purpose: .followScan, phase: "searching")
        let newContext = FollowMotionRequestContext(sessionGeneration: 42, requestToken: 802,
            purpose: .followScan, phase: "reacquiring")
        beginReplacement = { replacement = Task { await motion.perform(.scan(0.3), context: newContext) } }
        let caller = Task { await motion.perform(.scan(0.3), context: oldContext) }
        await sendGate.waitUntilEntered()
        caller.cancel()
        for _ in 0..<40 { await Task.yield() }
        sendGate.release()
        await replacementStopGate.waitUntilEntered()
        replacementStopGate.release()
        let second = await replacement?.value
        let first = await caller.value
        XCTAssertNotNil(second)
        XCTAssertEqual(first.result, .cancelled)
        XCTAssertEqual(second?.result, .arrived)
        XCTAssertEqual(second?.context.request, newContext)
        XCTAssertEqual(oldIndependentStops, 1, "Old cancellation cleanup cannot issue a stop after replacement reserves the controller")
        XCTAssertEqual(controller.safetyState, .idle)
        let oldBegins = try records(sink).filter {
            $0["event"] as? String == "follow_scan.stop_begin" && $0["operation_id"] as? UInt64 == first.context.controllerOperationID
                && $0["stop_origin"] as? String == "independent"
        }
        XCTAssertEqual(oldBegins.count, 1, "Only the original pre-stop belongs to the old operation")
    }

    func testContextualCallerCancellationAloneDrainsScanAndRequiresIndependentStop() async throws {
        try await checkCallerCancellationAlone(failIndependentStop: false)
    }

    func testContextualCallerCancellationAloneFailedStopLatchesAndBlocksMotion() async throws {
        try await checkCallerCancellationAlone(failIndependentStop: true)
    }

    func testContextualCallerCancellationAlsoFencesAlignmentReadyAndFollowing() async {
        for request in [FollowMotionRequest.alignment(0.3), .ready, .following(Vec2(1, 0), 1.25)] {
            for boundary in ["ack", "send"] {
                let gate = FollowDiagnosticSuspension()
                var pose = Pose2D(position: .zero, yaw: 0)
                var sends = 0
                var independentStops = 0
                let source = FollowRecoveryDiagnosticSourceFixture()
                let controller = FollowNavigationController(currentPose: { pose }, forwardClearance: { 2 },
                    plan: { _, goal in [goal] }, lastAckAt: {
                        if boundary == "ack", !gate.entered { await gate.suspend() }
                        return Date()
                    }, sendCommand: { _ in
                        sends += 1
                        if sends == 1, boundary == "send" { await gate.suspend() }
                        if sends == 2 {
                            switch request {
                            case .alignment: pose = Pose2D(position: .zero, yaw: 0.3)
                            case .ready: pose = Pose2D(position: Vec2(0.1, 0), yaw: 0)
                            default: pose = Pose2D(position: Vec2(1, 0), yaw: 0)
                            }
                        }
                    }, stopRover: {
                        if Task.isCancelled { throw CancellationError() }
                        if FollowMotionTaskScope.evidence?.fenced == true { independentStops += 1 }
                    }, sleep: { duration in source.pose = pose; await source.advance(duration) },
                    poseSample: { source.snapshot }, sourceNow: { source.uptime },
                    sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
                source.controller = controller
                let context = FollowMotionRequestContext(sessionGeneration: 32, requestToken: 702,
                    purpose: request.purpose, phase: "captured-phase")
                let caller = Task { await NavigationFollowMeMotion(navigation: controller).perform(request, context: context) }
                await gate.waitUntilEntered()
                caller.cancel()
                for _ in 0..<40 { await Task.yield() }
                gate.release()
                let result = await caller.value
                XCTAssertEqual(result.result, .cancelled, "\(request.purpose) \(boundary)")
                XCTAssertEqual(result.context.request, context)
                XCTAssertEqual(sends, boundary == "ack" ? 0 : 1, "\(request.purpose) \(boundary)")
                XCTAssertEqual(independentStops, 1, "One noncancelled serialized confirmation")
                XCTAssertEqual(result.stopOutcome, .confirmed)
                XCTAssertEqual(controller.safetyState, .idle)
            }
        }
    }

    private func checkCallerCancellationAlone(failIndependentStop: Bool) async throws {
        for boundary in ["ack_read", "send", "pulse_wait", "pulse_stop", "settle", "final_confirmation"] {
            let clock = FollowDiagnosticTestClock()
            let sink = FollowDiagnosticRecordingSink()
            let gate = FollowDiagnosticSuspension()
            let stopGate = FollowDiagnosticSuspension()
            let source = FollowRecoveryDiagnosticSourceFixture()
            let emitter = FollowDiagnosticEmitter(streamID: boundary, monotonic: { clock.monotonic },
                utc: { clock.utc }, sink: sink.append)
            var yaw = 0.0
            var sends = 0
            var stops = 0
            var settles = 0
            var waits: [Double] = []
            var completed = false
            let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: yaw) },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: {
                    if boundary == "ack_read", !gate.entered { await gate.suspend() }
                    return Date()
                },
                sendCommand: { _ in
                    sends += 1
                    if sends == 1, boundary == "send" { await gate.suspend() }
                }, stopRover: {
                    stops += 1
                    let fencedAtEntry = FollowMotionTaskScope.evidence?.fenced == true
                    if (boundary == "pulse_stop" && stops == 2)
                         || (boundary == "final_confirmation" && stops == 3) { await gate.suspend() }
                    // A cancelled loop's cleanup is deliberately not an acknowledgement.
                    if Task.isCancelled { throw CancellationError() }
                    if fencedAtEntry {
                        if !stopGate.entered { await stopGate.suspend() }
                        if failIndependentStop { throw CancellationError() }
                    }
                }, sleep: { duration in
                    if boundary == "ack_read", gate.entered {
                        // Read-only safety deadline, not a motor/settle stage.
                        try? await Task.sleep(for: duration)
                        return
                    }
                    let seconds = Self.seconds(duration)
                    waits.append(seconds)
                    if (boundary == "pulse_wait" && sends == 1 && stops == 1)
                        || (boundary == "settle" && seconds >= 0.299 && stops == 2) {
                        await gate.suspend()
                    }
                    // Bound the unfixed loop: it naturally arrives on its second pulse,
                    // so missing propagation fails assertions instead of hanging red.
                    if seconds >= 0.299 { settles += 1; if settles >= 2 { yaw = settles == 2 ? 0.25 : 0.6 } }
                    source.pose = .init(position: .zero, yaw: yaw)
                    await source.advance(duration)
                }, diagnosticEmitter: emitter, poseSample: { source.snapshot }, sourceNow: { source.uptime },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let request = FollowMotionRequestContext(sessionGeneration: 31, requestToken: 701,
                purpose: .followScan, phase: "reacquiring")
            let caller = Task {
                let result = await NavigationFollowMeMotion(navigation: controller).perform(.scan(0.6), context: request)
                completed = true
                return result
            }
            await gate.waitUntilEntered()
            caller.cancel() // No external stopAndConfirm, public cancel, or coordinator inhibition.
            for _ in 0..<40 { await Task.yield() }
            let cancellation = try records(sink).first { $0["event"] as? String == "follow_scan.cancel" }
            XCTAssertEqual(cancellation?["cancel_origin"] as? String, "task_cancellation", boundary)
            XCTAssertEqual(cancellation?["fenced"] as? Bool, true, boundary)
            XCTAssertFalse(completed, "Suspended transport/wait must drain before returning")
            gate.release()
            for _ in 0..<100 { await Task.yield() }
            XCTAssertTrue(stopGate.entered, "Caller cancellation must request independent serialized confirmation: \(boundary)")
            XCTAssertFalse(completed, "Cancellation cannot imply stop confirmation: \(boundary)")
            stopGate.release()
            let result = await caller.value
            XCTAssertEqual(result.context.request, request)
            XCTAssertEqual(result.context.profile?.pulseWait, 0.080)
            let expectedSends = boundary == "ack_read" ? 0 : (boundary == "final_confirmation" ? 2 : 1)
            XCTAssertEqual(sends, expectedSends, "Caller cancellation alone inhibits subsequent nonzero commands: \(boundary)")
            XCTAssertEqual(result.result, failIndependentStop ? .failed(.commandFailed) : .cancelled)
            XCTAssertEqual(result.stopOutcome, failIndependentStop ? .failed : .confirmed)
            if boundary == "ack_read" { XCTAssertTrue(waits.isEmpty) }
            if boundary == "send" {
                XCTAssertFalse(try records(sink).contains { $0["event"] as? String == "follow_scan.pulse_wait_begin" },
                    "Pending budget monitors are not an added motor wait")
            }
            if boundary == "pulse_wait" || boundary == "pulse_stop" { XCTAssertEqual(waits.count, 2) }
            if boundary == "settle" { XCTAssertEqual(waits.count, 3) }
            if boundary == "final_confirmation" { XCTAssertEqual(waits.count, 4) }
            for wait in waits where wait < 0.299 { XCTAssertGreaterThan(wait, 0); XCTAssertLessThanOrEqual(wait, 0.080 + 1e-12) }
            let events = try records(sink)
            let response = try XCTUnwrap(events.last { $0["event"] as? String == "follow_scan.stop_response" })
            XCTAssertEqual(response["stop_origin"] as? String, "independent")
            XCTAssertEqual(response["stop_outcome"] as? String, failIndependentStop ? "failed" : "confirmed")
            XCTAssertEqual(response["operation_id"] as? UInt64, result.context.controllerOperationID)
            XCTAssertEqual(events.last?["stop_unconfirmed"] as? Bool, failIndependentStop)
            if failIndependentStop {
                XCTAssertEqual(controller.safetyState, .failed(.commandFailed))
                let blocked = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3)
                XCTAssertEqual(blocked, .failed(.commandFailed))
                XCTAssertEqual(sends, expectedSends)
            } else {
                XCTAssertEqual(controller.safetyState, .idle)
            }
        }
    }

    func testSerializedStopResponsesKeepTheirOwnReceiptAcrossConcurrentConfirmation() async throws {
        let clock = FollowDiagnosticTestClock()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "stop-receipt-race", monotonic: { clock.monotonic },
            utc: { clock.utc }, sink: sink.append)
        let gate = FollowDiagnosticSuspension()
        var stops = 0
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { nil },
            sendCommand: { _ in XCTFail("Cancelled pre-stop cannot send") }, stopRover: {}, sleep: { _ in },
            stopRoverReceipt: {
                stops += 1
                let status = 200 + stops
                if stops == 1 { await gate.suspend() }
                return .init(receipt: .init(httpStatus: status, acknowledged: true,
                    acknowledgementUTC: Date(timeIntervalSince1970: Double(status)), attempts: 1,
                    outcome: "acknowledged"), failure: nil)
            }, diagnosticEmitter: emitter)
        let scan = Task { await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3) }
        await gate.waitUntilEntered()
        let stop = Task { try await controller.stopAndConfirm() }
        for _ in 0..<20 { await Task.yield() }
        gate.release()
        try await stop.value
        let result = await scan.value
        XCTAssertEqual(result, .cancelled)
        let responses = try records(sink).filter { $0["event"] as? String == "follow_scan.stop_response" }
        for (identity, status) in [(1, 201), (2, 202)] {
            let response = try XCTUnwrap(responses.first { $0["stop_id"] as? Int == identity })
            let receipt = try XCTUnwrap(response["receipt"] as? [String: Any])
            XCTAssertEqual(receipt["http_status"] as? Int, status, "Receipt must belong to stop \(identity)")
            XCTAssertEqual(receipt["command_ack_utc_s"] as? Double, Double(status))
        }
    }

    func testWatchdogCheckpointTimeIsObservationBoundaryNotEarlierPoseRead() async throws {
        let clock = FollowDiagnosticTestClock()
        let source = FollowRecoveryDiagnosticSourceFixture()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "checkpoint-latency", monotonic: { clock.monotonic },
            utc: { clock.utc }, sink: sink.append)
        var yaw = 0.0
        var reads = 0
        var sent = false
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: yaw) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: {
                reads += 1
                if reads == 3 { clock.monotonic += 0.4 }
                return clock.utc.addingTimeInterval(clock.monotonic)
            }, sendCommand: { _ in sent = true }, stopRover: {}, sleep: { duration in
                clock.monotonic += Self.seconds(duration)
                if sent, Self.seconds(duration) >= 0.299 { yaw = 0.3 }
                source.pose = .init(position: .zero, yaw: yaw)
                await source.advance(duration)
            }, now: { clock.utc.addingTimeInterval(clock.monotonic) }, diagnosticEmitter: emitter,
            poseSample: { source.snapshot }, sourceNow: { source.uptime }, sourceEvents: { source.events.stream },
            sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let result = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3)
        XCTAssertEqual(result, .arrived)
        let pulse = try XCTUnwrap(records(sink).first { $0["event"] as? String == "follow_scan.pulse_begin" })
        XCTAssertEqual(try XCTUnwrap(pulse["watchdog_checkpoint_monotonic_s"] as? Double), 0.3, accuracy: 1e-12)
        XCTAssertEqual(pulse["watchdog_checkpoint_yaw_rad"] as? Double, 0)
        XCTAssertEqual(try XCTUnwrap(pulse["watchdog_elapsed_s"] as? Double), 0.4, accuracy: 1e-6,
            "Post-checkpoint feedback await consumes the unchanged epoch")
    }

    func testCompletedPulseTraceUsesExistingSamplesExactHostTimingAndCapturedContext() async throws {
        let clock = FollowDiagnosticTestClock()
        let source = FollowRecoveryDiagnosticSourceFixture()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "controller-test", monotonic: { source.uptime },
            utc: { clock.utc }, sink: sink.append)
        var reads = 0
        var sends = 0
        let controller = FollowNavigationController(currentPose: { XCTFail("Enriched trace cannot read legacy pose"); return nil },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in sends += 1; source.uptime += 0.04 }, stopRover: { source.uptime += 0.01 },
            sleep: { duration in
                if sends > 0, Self.seconds(duration) >= 0.299 { source.pose = .init(position: .zero, yaw: 0.3) }
                await source.advance(duration)
            }, diagnosticEmitter: emitter, poseSample: { reads += 1; return source.snapshot },
            sourceNow: { source.uptime }, sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let request = FollowMotionRequestContext(sessionGeneration: 9, requestToken: 901,
            purpose: .followScan, phase: "reacquiring", scanUsed: 1.2, scanRemaining: nil)
        let result = await NavigationFollowMeMotion(navigation: controller).perform(.scan(0.3), context: request)
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(reads, 1, "Telemetry must reuse the captured ingress samples")
        let events = try records(sink)
        XCTAssertEqual(events.compactMap { $0["event"] as? String }, [
            "follow_scan.operation_begin", "follow_scan.stop_begin", "follow_scan.stop_response",
            "follow_scan.settle_begin", "follow_scan.stopped_source_rejected", "follow_scan.stopped_source_accepted", "follow_scan.settle_end",
            "follow_scan.burst_plan", "follow_depth.candidate",
            "follow_scan.pulse_begin", "follow_scan.send_begin", "follow_depth.candidate", "follow_scan.send_ack",
            "follow_scan.pulse_wait_begin", "follow_scan.burst_wait_end", "follow_scan.pulse_wait_end",
            "follow_scan.burst_stop_obligation",
            "follow_scan.stop_begin", "follow_scan.stop_response",
            "follow_scan.burst_stop_confirmed",
            "follow_scan.settle_begin", "follow_scan.stopped_source_rejected", "follow_scan.stopped_source_accepted", "follow_scan.settle_end", "follow_scan.pulse_complete",
            "follow_scan.burst_response", "follow_scan.burst_plan",
            "follow_scan.operation_complete"
        ])
        for event in events {
            XCTAssertEqual(event["session_generation"] as? Int, 9)
            XCTAssertEqual(event["operation_id"] as? UInt64, result.context.controllerOperationID)
            XCTAssertEqual(event["phase"] as? String, "reacquiring")
            XCTAssertEqual(event["schema_version"] as? Int, 1)
            XCTAssertEqual(event["stream_id"] as? String, "controller-test")
        }
        let pulse = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.pulse_complete" })
        XCTAssertEqual(pulse["pulse_index"] as? Int, 1)
        XCTAssertEqual(pulse["pre_yaw_rad"] as? Double, 0)
        XCTAssertEqual(pulse["post_yaw_rad"] as? Double, 0.3)
        XCTAssertEqual(pulse["signed_yaw_delta_rad"] as? Double, 0.3)
        XCTAssertEqual(pulse["error_improvement_rad"] as? Double, 0.3)
        XCTAssertEqual(pulse["post_yaw_rad_display"] as? String, "+0.300000")
        XCTAssertEqual(pulse["profile_pulse_wait_s"] as? Double, 0.080)
        XCTAssertEqual(pulse["profile_settle_s"] as? Double, 0.3)
        XCTAssertEqual(pulse["profile_wheel_cap_mps"] as? Double, 0.25)
        XCTAssertEqual(pulse["profile_wheel_floor_mps"] as? Double, 0.25)
        XCTAssertEqual(pulse["profile_command_law"] as? String, "fixed_signed_magnitude")
        XCTAssertEqual(pulse["profile_yaw_gain_active"] as? Bool, false)
        XCTAssertEqual(pulse["profile_yaw_gain"] as? Double, 0.3)
        XCTAssertEqual(pulse["pose_source_age_status"] as? String, "available")
        XCTAssertEqual(pulse["pose_source_clock"] as? String, "ar_system_uptime")
        XCTAssertEqual(pulse["pose_read_clock"] as? String, "ar_system_uptime")
        XCTAssertEqual(pulse["pose_source_timestamp"] as? Double, source.snapshot.sourceTimestamp)
        XCTAssertEqual(pulse["watchdog_progress_rad"] as? Double, 0,
            "The actual healthy 0.3-rad advance moved the original checkpoint")
        XCTAssertEqual(pulse["watchdog_required_progress_rad"] as? Double, 0.05)
        XCTAssertEqual(pulse["watchdog_interval_s"] as? Double, 2.5)
        let ack = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.send_ack" })
        XCTAssertEqual(try XCTUnwrap(ack["host_duration_s"] as? Double), 0.04, accuracy: 1e-12)
        XCTAssertTrue(ack["http_status"] is NSNull)
        let wait = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.pulse_wait_end" })
        XCTAssertEqual(try XCTUnwrap(wait["host_duration_s"] as? Double), 0.04, accuracy: 1e-12)
        XCTAssertEqual(events.last?["outcome"] as? String, "completed")
        XCTAssertEqual(events.last?["stop_outcome"] as? String, "confirmed")
        XCTAssertEqual(events.last?["stop_unconfirmed"] as? Bool, false)
    }

    private func records(_ sink: FollowDiagnosticRecordingSink) throws -> [[String: Any]] {
        try sink.records.map { record in
            let data = try XCTUnwrap(record.fields["payload"]?.data(using: .utf8))
            return try XCTUnwrap(JSONSerialization.jsonObject(with: data) as? [String: Any])
        }
    }

    func testCancellationTraceEndsOnlyEnteredStagesAndNeverContinuesMotion() async throws {
        for boundary in ["pre_stop", "ack_read", "send", "pulse_wait", "pulse_stop", "settle", "arrival_stop", "final_confirmation"] {
            let clock = FollowDiagnosticTestClock()
            let sink = FollowDiagnosticRecordingSink()
            let gate = FollowDiagnosticSuspension()
            let source = FollowRecoveryDiagnosticSourceFixture()
            let emitter = FollowDiagnosticEmitter(streamID: boundary, monotonic: { clock.monotonic }, utc: { clock.utc }, sink: sink.append)
            var yaw = 0.0
            var sends = 0
            var stops = 0
            var waits: [Double] = []
            let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: yaw) },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: {
                    if boundary == "ack_read", !gate.entered { await gate.suspend() }; return Date()
                }, sendCommand: { _ in
                    sends += 1; if boundary == "send" { await gate.suspend() }
                }, stopRover: {
                    stops += 1
                    if (boundary == "pre_stop" && stops == 1) || (boundary == "pulse_stop" && stops == 2)
                         || (["arrival_stop", "final_confirmation"].contains(boundary) && stops == 2) {
                        await gate.suspend()
                    }
                }, sleep: {
                    let duration = Self.seconds($0)
                    waits.append(duration)
                    if (boundary == "pulse_wait" && sends > 0 && stops == 1)
                        || (boundary == "settle" && duration >= 0.299 && stops == 2) {
                        await gate.suspend()
                    }
                    clock.monotonic += duration
                    if duration >= 0.299, sends > 0 { yaw = 0.3 }
                    source.pose = .init(position: .zero, yaw: yaw)
                    await source.advance($0)
                }, diagnosticEmitter: emitter, poseSample: { source.snapshot }, sourceNow: { source.uptime },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let request = FollowMotionRequestContext(sessionGeneration: 10, requestToken: 950, purpose: .followScan, phase: "searching")
            let operation = Task { await NavigationFollowMeMotion(navigation: controller).perform(.scan(0.3), context: request) }
            await gate.waitUntilEntered()
            let stop = Task { try await controller.stopAndConfirm() }
            // Stop fences synchronously before entering its first suspension.
            while controller.safetyState == .moving && !sink.records.contains(where: { $0.event == "follow_scan.cancel" }) {
                await Task.yield()
                // Bounded red: missing telemetry must fail an assertion, not hang.
                break
            }
            for _ in 0..<10 { await Task.yield() }
            gate.release()
            try await stop.value
            let result = await operation.value
            let events = try records(sink)
            XCTAssertEqual(result.result, .cancelled, boundary)
            let cancel = events.first { $0["event"] as? String == "follow_scan.cancel" }
            XCTAssertNotNil(cancel, boundary)
            XCTAssertEqual(cancel?["fenced"] as? Bool, true, boundary)
            XCTAssertEqual(cancel?["cancel_origin"] as? String, "independent", boundary)
            XCTAssertEqual(events.last(where: { $0["event"] as? String == "follow_scan.operation_complete" })?["outcome"] as? String, "cancelled", boundary)
            if ["pre_stop", "ack_read"].contains(boundary) { XCTAssertEqual(sends, 0, boundary) }
            if boundary == "send" {
                XCTAssertFalse(events.contains { $0["event"] as? String == "follow_scan.pulse_wait_begin" })
            }
            if ["pulse_wait", "pulse_stop"].contains(boundary) {
                XCTAssertEqual(waits.count, 2, "Initial settle and one burst budget only")
                XCTAssertEqual(waits[1], 0.080, accuracy: 1e-12)
            }
            if ["pulse_wait", "settle"].contains(boundary) {
                let name = boundary == "settle" ? "settle" : "pulse_wait"
                XCTAssertEqual(events.filter { $0["event"] as? String == "follow_scan.\(name)_end" }.count, name == "settle" ? 2 : 1)
                XCTAssertEqual(events.last { $0["event"] as? String == "follow_scan.\(name)_end" }?["outcome"] as? String, "interrupted")
            }
            XCTAssertFalse(events.contains { $0["event"] as? String == "follow_scan.pulse_complete" && $0["stale"] as? Bool == true }, boundary)
            XCTAssertEqual(controller.safetyState, .idle, boundary)
        }
    }

    func testFailureTracesRetainFailedStageReceiptPrimaryReasonAndLatch() async throws {
        for boundary in ["send", "pulse_stop", "independent_stop", "watchdog"] {
            let clock = FollowDiagnosticTestClock()
            let source = FollowRecoveryDiagnosticSourceFixture()
            let sink = FollowDiagnosticRecordingSink()
            let emitter = FollowDiagnosticEmitter(streamID: boundary, monotonic: { clock.monotonic }, utc: { clock.utc }, sink: sink.append)
            var stops = 0
            var sends = 0
            let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { clock.utc.addingTimeInterval(clock.monotonic) },
                sendCommand: { _ in }, stopRover: {}, sleep: {
                    clock.monotonic += Self.seconds($0)
                    await source.advance($0)
                },
                now: { clock.utc.addingTimeInterval(clock.monotonic) }, sendCommandReceipt: { _ in
                    sends += 1
                    if boundary == "watchdog" { clock.monotonic += 2.5 }
                    let failed = boundary == "send"
                    return .init(receipt: .init(httpStatus: failed ? 503 : 204, acknowledged: !failed,
                        acknowledgementUTC: failed ? nil : clock.utc, attempts: 1, outcome: failed ? "failed" : "acknowledged"),
                        failure: failed ? RoverControlError.serverError(503) : nil)
                }, stopRoverReceipt: {
                    stops += 1
                    let failed = (boundary == "pulse_stop" && stops == 2) || (boundary == "independent_stop" && stops == 1)
                    return .init(receipt: .init(httpStatus: failed ? 503 : 200, acknowledged: !failed,
                        acknowledgementUTC: failed ? nil : clock.utc, attempts: 1, outcome: failed ? "failed" : "acknowledged"),
                        failure: failed ? RoverControlError.serverError(503) : nil)
                }, diagnosticEmitter: emitter, poseSample: { source.snapshot }, sourceNow: { source.uptime },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let result = await controller.rotateForFollowScan(by: 0.3)
            XCTAssertEqual(result, .failed(boundary == "watchdog" ? .stalled : .commandFailed))
            let events = try records(sink)
            let failure = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.failure" }, boundary)
            XCTAssertEqual(failure["failed_stage"] as? String, boundary == "watchdog" ? "watchdog" : boundary)
            XCTAssertEqual(failure["reason"] as? String, boundary == "watchdog" ? "no_yaw_progress" : "commandFailed")
            XCTAssertEqual(failure["typed_reason"] as? String, boundary == "watchdog" ? "stalled" : "commandFailed")
            for key in ["pre_yaw_rad", "post_yaw_rad", "target_yaw_rad", "pre_error_rad", "post_error_rad",
                        "signed_yaw_delta_rad", "error_improvement_rad", "watchdog_checkpoint_yaw_rad",
                        "watchdog_checkpoint_monotonic_s", "watchdog_progress_rad", "watchdog_elapsed_s",
                        "operation_elapsed_s", "stage_start_monotonic_s"] {
                XCTAssertNotNil(failure[key], "\(boundary): \(key)")
            }
            if boundary == "send" {
                let ack = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.send_ack" })
                XCTAssertEqual(ack["outcome"] as? String, "failed")
                XCTAssertEqual(ack["http_status"] as? Int, 503)
                XCTAssertFalse(events.contains { $0["event"] as? String == "follow_scan.pulse_wait_begin" })
            }
            if boundary == "pulse_stop" || boundary == "independent_stop" {
                let response = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.stop_response" && $0["outcome"] as? String == "failed" })
                XCTAssertEqual(response["stop_unconfirmed"] as? Bool, true)
                XCTAssertEqual(events.last?["stop_unconfirmed"] as? Bool, true)
                let blocked = await controller.rotateForFollowScan(by: 0.3)
                XCTAssertEqual(blocked, .failed(.commandFailed))
                XCTAssertEqual(sends, boundary == "pulse_stop" ? 1 : 0)
            }
            if boundary == "watchdog" {
                XCTAssertEqual(failure["watchdog_progress_rad"] as? Double, 0)
                XCTAssertEqual(failure["watchdog_elapsed_s"] as? Double, 2.5)
                XCTAssertEqual(failure["watchdog_checkpoint_yaw_rad"] as? Double, 0)
                XCTAssertEqual(events.last?["primary_failure"] as? String, "stalled")
            }
        }
    }

    func testPulseSummaryRetainsStageTimingsReceiptClockAndNullableStopCorrelation() async throws {
        let clock = FollowDiagnosticTestClock()
        let source = FollowRecoveryDiagnosticSourceFixture()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "summary", monotonic: { source.uptime }, utc: { clock.utc }, sink: sink.append)
        var yaw = 3.10
        source.pose = .init(position: .zero, yaw: yaw)
        source.capture(after: 0)
        var sent = false
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: yaw) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in }, stopRover: {},
            sleep: { duration in
                if sent, Self.seconds(duration) >= 0.299 { yaw = -2.883185307179586 }
                source.pose = .init(position: .zero, yaw: yaw)
                await source.advance(duration)
            },
            sendCommandReceipt: { _ in
                sent = true
                source.uptime += 0.04
                return .init(receipt: .init(httpStatus: 202, acknowledged: true,
                    acknowledgementUTC: clock.utc.addingTimeInterval(-0.125), attempts: 2, outcome: "acknowledged"), failure: nil)
            }, diagnosticEmitter: emitter, poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let result = await controller.rotateForFollowScan(by: 0.3)
        XCTAssertEqual(result, .arrived)
        let events = try records(sink)
        let complete = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.pulse_complete" })
        XCTAssertEqual(try XCTUnwrap(complete["signed_yaw_delta_rad"] as? Double), 0.3, accuracy: 1e-12)
        XCTAssertEqual(try XCTUnwrap(complete["error_improvement_rad"] as? Double), 0.3, accuracy: 1e-12)
        XCTAssertEqual(complete["watchdog_checkpoint_yaw_rad_display"] as? String, "-2.883185")
        for (key, expected) in [("send_start_monotonic_s", 8.3), ("send_end_monotonic_s", 8.34),
                                ("send_host_duration_s", 0.04), ("pulse_wait_start_monotonic_s", 8.34),
                                ("pulse_wait_end_monotonic_s", 8.38), ("pulse_wait_host_duration_s", 0.04),
                                ("settle_start_monotonic_s", 8.38), ("settle_end_monotonic_s", 8.68),
                                ("settle_host_duration_s", 0.3)] {
            XCTAssertEqual(try XCTUnwrap(complete[key] as? Double, key), expected, accuracy: 1e-12)
        }
        let ack = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.send_ack" })
        XCTAssertEqual(ack["http_status"] as? Int, 202)
        XCTAssertEqual(ack["command_ack_age_s"] as? Double, 0.125)
        XCTAssertEqual(ack["command_ack_clock"] as? String, "transport_utc")
        for event in events where ["follow_scan.operation_complete", "follow_scan.stop_begin", "follow_scan.stop_response"].contains(event["event"] as? String ?? "") {
            if event["stop_origin"] as? String != "pulse" {
                XCTAssertTrue(event["pulse_index"] is NSNull, "Non-pulse stop/completion has operation correlation only")
            }
        }
    }

    func testTrackingLossAfterSettleEndsEnteredWaitWithUnavailablePostSample() async throws {
        let clock = FollowDiagnosticTestClock()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "lost", monotonic: { clock.monotonic }, utc: { clock.utc }, sink: sink.append)
        let source = FollowRecoveryDiagnosticSourceFixture()
        var sent = false
        let controller = FollowNavigationController(currentPose: { source.pose },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in sent = true }, stopRover: {},
            sleep: { duration in
                clock.monotonic += Self.seconds(duration)
                if sent, Self.seconds(duration) >= 0.299 {
                    source.uptime += Self.seconds(duration)
                    source.snapshot = .init(pose: nil, frameID: .init(generation: 4, sequence: 13),
                        sourceTimestamp: source.uptime, trackingQuality: .normal)
                    source.controller?.ingestFollowTurnSource(source.snapshot)
                } else { await source.advance(duration) }
            }, diagnosticEmitter: emitter, poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let result = await controller.rotateForFollowScan(by: 0.3)
        XCTAssertEqual(result, .failed(.trackingLost))
        let events = try records(sink)
        let end = try XCTUnwrap(events.last { $0["event"] as? String == "follow_scan.settle_end" })
        XCTAssertEqual(end["outcome"] as? String, "completed")
        XCTAssertTrue(end["post_yaw_rad"] is NSNull)
        let failure = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.failure" })
        XCTAssertEqual(failure["reason"] as? String, "trackingLost")
        XCTAssertTrue(failure["post_yaw_rad"] is NSNull)
        XCTAssertTrue(failure["watchdog_progress_rad"] is NSNull, "Missing current pose cannot reuse earlier current progress")
        XCTAssertEqual(failure["watchdog_progress_rad_availability"] as? String, "unavailable")
        XCTAssertEqual(failure["watchdog_checkpoint_yaw_rad"] as? Double, 0, "The authoritative checkpoint remains known")
    }

    func testControllerTraceUsesActualWatchdogBoundaryResetAndNegativeErrorProgress() async throws {
        for adequate in [false, true] {
            let clock = FollowDiagnosticTestClock()
            clock.utc = Date(timeIntervalSinceReferenceDate: 0)
            let sink = FollowDiagnosticRecordingSink()
            let emitter = FollowDiagnosticEmitter(streamID: "watchdog", monotonic: { clock.monotonic }, utc: { clock.utc }, sink: sink.append)
            var yaw = 0.0
            var pulses = 0
            var sends = 0
            let source = FollowRecoveryDiagnosticSourceFixture()
            let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: yaw) },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { clock.utc.addingTimeInterval(clock.monotonic) },
                sendCommand: { _ in sends += 1 }, stopRover: {}, sleep: { duration in
                    if sends > 0, Self.seconds(duration) >= 0.299 {
                        pulses += 1
                        if !adequate {
                            clock.monotonic = pulses == 1 ? 2.499 : 2.5
                            yaw = 0.049
                        } else {
                            clock.monotonic = pulses == 1 ? 2.5 : (pulses == 2 ? 4.999 : 5)
                            yaw = pulses == 1 ? 0.05 : 0
                        }
                    } else if sends > 0 {
                        clock.monotonic += Self.seconds(duration)
                        let terminalTick = adequate ? 5.0 : 2.5
                        if abs(clock.monotonic - terminalTick) < 1e-12 {
                            // Publish the exact simulated Date deadline tick;
                            // sub-ULP uptime arithmetic cannot drive this clock.
                            clock.monotonic = terminalTick
                        }
                    }
                    source.pose = .init(position: .zero, yaw: yaw)
                    await source.advance(duration)
                }, now: { clock.utc.addingTimeInterval(clock.monotonic) }, diagnosticEmitter: emitter,
                poseSample: { source.snapshot }, sourceNow: { source.uptime }, sourceEvents: { source.events.stream },
                sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let result = await controller.rotateForFollowScan(by: 1)
            XCTAssertEqual(result, .failed(.stalled))
            let events = try records(sink)
            let starts = events.filter { $0["event"] as? String == "follow_scan.pulse_begin" }
            XCTAssertEqual(starts.count, adequate ? 3 : 2)
            let failure = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.failure" })
            XCTAssertEqual(try XCTUnwrap(failure["watchdog_elapsed_s"] as? Double), 2.5, accuracy: 1e-12)
            XCTAssertEqual(failure["watchdog_checkpoint_yaw_rad"] as? Double, adequate ? 0.05 : 0)
            XCTAssertEqual(failure["watchdog_checkpoint_monotonic_s"] as? Double, adequate ? 2.5 : 0)
            XCTAssertEqual(try XCTUnwrap(failure["watchdog_progress_rad"] as? Double), adequate ? -0.05 : 0.049, accuracy: 1e-12)
            if adequate {
                XCTAssertEqual(starts[1]["watchdog_checkpoint_yaw_rad"] as? Double, 0.05)
                XCTAssertEqual(starts[1]["watchdog_elapsed_s"] as? Double, 0)
                let completions = events.filter { $0["event"] as? String == "follow_scan.pulse_complete" }
                XCTAssertGreaterThanOrEqual(completions.count, 2)
                if completions.count >= 2 { XCTAssertEqual(try XCTUnwrap(completions[1]["error_improvement_rad"] as? Double), -0.05, accuracy: 1e-12) }
            } else {
                XCTAssertEqual(try XCTUnwrap(starts[1]["watchdog_elapsed_s"] as? Double), 2.499, accuracy: 1e-12)
            }
        }
    }

    func testCancelledPulseStopTraceDefersSafetyToIndependentConfirmation() async throws {
        for independentFails in [false, true] {
            let clock = FollowDiagnosticTestClock()
            let sink = FollowDiagnosticRecordingSink()
            let emitter = FollowDiagnosticEmitter(streamID: "stop", monotonic: { clock.monotonic }, utc: { clock.utc }, sink: sink.append)
            let gate = FollowDiagnosticSuspension()
            var stops = 0
            var sends = 0
            let source = FollowRecoveryDiagnosticSourceFixture()
            let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in sends += 1 },
                stopRover: {
                    stops += 1
                    if stops == 2 { await gate.suspend(); throw URLError(.cancelled) }
                    if stops == 3 && independentFails { throw URLError(.cannotConnectToHost) }
                }, sleep: source.advance, diagnosticEmitter: emitter, poseSample: { source.snapshot },
                sourceNow: { source.uptime }, sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let scan = Task { await controller.rotateForFollowScan(by: 0.3) }
            await gate.waitUntilEntered()
            var stopEntered = false
            let stop = Task { () -> Bool in
                stopEntered = true
                do { try await controller.stopAndConfirm(); return true } catch { return false }
            }
            while !stopEntered { await Task.yield() }
            gate.release()
            let confirmed = await stop.value
            let result = await scan.value
            XCTAssertEqual(confirmed, !independentFails)
            XCTAssertEqual(result, .cancelled)
            let events = try records(sink)
            let pulseResponse = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.stop_response" && $0["stop_origin"] as? String == "pulse" })
            XCTAssertEqual(pulseResponse["outcome"] as? String, "cancelled")
            XCTAssertEqual(pulseResponse["stop_outcome"] as? String, "pending", "Cancelled transport is not an independent stop acknowledgement")
            let independentResponse = try XCTUnwrap(events.last { $0["event"] as? String == "follow_scan.stop_response" && $0["stop_origin"] as? String == "independent" })
            XCTAssertEqual(independentResponse["stop_unconfirmed"] as? Bool, independentFails)
            XCTAssertEqual(independentResponse["stop_outcome"] as? String, independentFails ? "failed" : "confirmed")
            XCTAssertEqual(independentResponse["operation_id"] as? Int, pulseResponse["operation_id"] as? Int)
            if independentFails {
                let blocked = await controller.rotateForFollowScan(by: 0.3)
                XCTAssertEqual(blocked, .failed(.commandFailed))
                XCTAssertEqual(controller.safetyState, .failed(.commandFailed))
            }
            XCTAssertEqual(sends, 1)
        }
    }

    func testLateSuccessfulPreStopCannotClearNewerFailedIndependentStopOrSend() async throws {
        let clock = FollowDiagnosticTestClock()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "stale", monotonic: { clock.monotonic }, utc: { clock.utc }, sink: sink.append)
        let gate = FollowDiagnosticSuspension()
        var stops = 0
        var sends = 0
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { nil }, sendCommand: { _ in sends += 1 },
            stopRover: { stops += 1; if stops == 1 { await gate.suspend() } else { throw URLError(.cannotConnectToHost) } },
            sleep: { _ in }, diagnosticEmitter: emitter)
        let scan = Task { await controller.rotateForFollowScan(by: 0.3) }
        await gate.waitUntilEntered()
        var stopEntered = false
        let stop = Task { () -> Bool in
            stopEntered = true
            do { try await controller.stopAndConfirm(); return true } catch { return false }
        }
        while !stopEntered { await Task.yield() }
        gate.release()
        let confirmed = await stop.value
        let result = await scan.value
        XCTAssertFalse(confirmed)
        XCTAssertEqual(result, .cancelled)
        XCTAssertEqual(controller.safetyState, .failed(.commandFailed))
        let blocked = await controller.rotateForFollowScan(by: 0.3)
        XCTAssertEqual(blocked, .failed(.commandFailed))
        XCTAssertEqual(sends, 0)
        let events = try records(sink)
        let staleSuccess = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.stop_response" && $0["outcome"] as? String == "acknowledged" })
        XCTAssertEqual(staleSuccess["stale"] as? Bool, true)
        XCTAssertNotEqual(staleSuccess["stop_outcome"] as? String, "confirmed")
        XCTAssertTrue(events.contains { $0["event"] as? String == "follow_scan.stop_response" && $0["stop_unconfirmed"] as? Bool == true })
    }

    func testPublicCancelAndCallerCancellationKeepBalancedCapturedStopTrace() async throws {
        for callerCancellation in [false, true] {
            let clock = FollowDiagnosticTestClock()
            let sink = FollowDiagnosticRecordingSink()
            let emitter = FollowDiagnosticEmitter(streamID: "cancel", monotonic: { clock.monotonic }, utc: { clock.utc }, sink: sink.append)
            let gate = FollowDiagnosticSuspension()
            var stops = 0
            var sends = 0
            let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { nil }, sendCommand: { _ in sends += 1 },
                stopRover: { stops += 1; if stops == 1 { await gate.suspend() } }, sleep: { _ in }, diagnosticEmitter: emitter)
            let request = FollowMotionRequestContext(sessionGeneration: 7, requestToken: 1005, purpose: .followScan, phase: "searching")
            let operation = Task { await NavigationFollowMeMotion(navigation: controller).perform(.scan(0.3), context: request) }
            await gate.waitUntilEntered()
            if callerCancellation { operation.cancel() } else { controller.cancel() }
            gate.release()
            let result = await operation.value
            for _ in 0..<20 { await Task.yield() }
            XCTAssertEqual(result.result, .cancelled)
            XCTAssertEqual(sends, 0)
            let events = try records(sink)
            let cancel = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.cancel" })
            XCTAssertEqual(cancel["cancel_origin"] as? String, callerCancellation ? "task_cancellation" : "cancel")
            XCTAssertEqual(cancel["operation_id"] as? UInt64, result.context.controllerOperationID)
            if !callerCancellation {
                XCTAssertEqual(stops, 2)
                XCTAssertEqual(events.filter { $0["event"] as? String == "follow_scan.stop_begin" }.count, 2)
                XCTAssertEqual(events.filter { $0["event"] as? String == "follow_scan.stop_response" }.count, 2)
                XCTAssertTrue(events.last { $0["event"] as? String == "follow_scan.stop_response" }?["pulse_index"] is NSNull)
            }
        }
    }

    func testDetectionStopUsesCapturedOriginWithoutRelabellingPulseOrFinalStops() async throws {
        let clock = FollowDiagnosticTestClock()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "detection", monotonic: { clock.monotonic }, utc: { clock.utc }, sink: sink.append)
        let gate = FollowDiagnosticSuspension()
        let controller = makeFollowController(pose: { 0 }, send: { _ in },
            sleep: { if Self.seconds($0) <= 0.080 + 1e-12 { await gate.suspend() } }, diagnosticEmitter: emitter)
        let motion = NavigationFollowMeMotion(navigation: controller)
        let scan = Task { await motion.rotateForScan(by: 0.3) }
        await gate.waitUntilEntered()
        var entered = false
        let stop = Task {
            entered = true
            try await FollowMotionTaskScope.$stopOrigin.withValue(.detection) { try await motion.stopAndConfirm() }
        }
        while !entered { await Task.yield() }
        gate.release()
        try await stop.value
        let result = await scan.value
        XCTAssertEqual(result, .cancelled)
        let events = try records(sink)
        XCTAssertEqual(events.first { $0["event"] as? String == "follow_scan.cancel" }?["cancel_origin"] as? String, "detection")
        let response = try XCTUnwrap(events.last { $0["event"] as? String == "follow_scan.stop_response" })
        XCTAssertEqual(response["stop_origin"] as? String, "detection")
        XCTAssertEqual(response["stop_outcome"] as? String, "confirmed")
        XCTAssertTrue(response["pulse_index"] is NSNull)
        XCTAssertEqual(events.first { $0["event"] as? String == "follow_scan.stop_begin" }?["stop_origin"] as? String, "independent")
    }

    func testUnavailableMeasurementsHaveReasonsAndCompletionNamesLastReachedStage() async throws {
        let clock = FollowDiagnosticTestClock()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "availability", monotonic: { clock.monotonic }, utc: { clock.utc }, sink: sink.append)
        var yaw = 0.0
        let controller = makeFollowController(pose: { yaw }, send: { _ in }, sleep: {
            clock.monotonic += Self.seconds($0)
            if Self.seconds($0) >= 0.299 { yaw = 0.3 }
        }, diagnosticEmitter: emitter)
        let result = await controller.rotateForFollowScan(by: 0.3)
        XCTAssertEqual(result, .arrived)
        let events = try records(sink)
        let begin = try XCTUnwrap(events.first)
        XCTAssertTrue(begin["target_yaw_rad"] is NSNull, "Target is unknown until the existing post-prestop sample")
        for key in ["watchdog_checkpoint_yaw_rad", "watchdog_checkpoint_monotonic_s", "watchdog_progress_rad", "watchdog_elapsed_s"] {
            XCTAssertTrue(begin[key] is NSNull)
            XCTAssertEqual(begin[key + "_availability"] as? String, "unavailable")
        }
        let ack = try XCTUnwrap(events.first { $0["event"] as? String == "follow_scan.send_ack" })
        XCTAssertEqual(ack["http_status_availability"] as? String, "unknown")
        XCTAssertEqual(ack["acknowledged_availability"] as? String, "unknown")
        XCTAssertEqual(ack["command_ack_age_s_availability"] as? String, "unknown")
        let complete = try XCTUnwrap(events.last)
        XCTAssertEqual(complete["last_reached_stage"] as? String, "settle", "No unentered wrapper stop may be reported")
        XCTAssertEqual(complete["final_yaw_rad_display"] as? String, "+0.300000")
        XCTAssertEqual(complete["final_error_rad_display"] as? String, "+0.000000")
        XCTAssertEqual(complete["profile_angular_tolerance_rad_display"] as? String, "+0.122173")
    }

    func testWatchdogSnapshotExposesActualCheckpointWithoutChangingBoundaryPolicy() {
        let epoch = Date(timeIntervalSinceReferenceDate: 0)
        var watchdog = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.05)
        XCTAssertFalse(watchdog.observe(distanceToGoal: 1, now: epoch, commanded: true))
        XCTAssertFalse(watchdog.observe(distanceToGoal: 0.951, now: epoch.addingTimeInterval(2.499), commanded: true))
        let before = watchdog.diagnosticSnapshot(distanceToGoal: 0.951, now: epoch.addingTimeInterval(2.499))
        XCTAssertEqual(before.bestDistance, 1)
        XCTAssertEqual(before.lastProgressAt, epoch)
        XCTAssertEqual(before.progress ?? 0, 0.049, accuracy: 1e-12)
        XCTAssertEqual(before.elapsed ?? 0, 2.499, accuracy: 1e-12)
        XCTAssertEqual(before.interval, 2.5)
        XCTAssertEqual(before.requiredProgress, 0.05)
        XCTAssertTrue(watchdog.observe(distanceToGoal: 0.951, now: epoch.addingTimeInterval(2.5), commanded: true))
        watchdog.reset()
        XCTAssertNil(watchdog.diagnosticSnapshot(distanceToGoal: 0.951, now: epoch).bestDistance)
        XCTAssertFalse(watchdog.observe(distanceToGoal: 1, now: epoch, commanded: true))
        let resetTime = epoch.addingTimeInterval(2.5)
        XCTAssertFalse(watchdog.observe(distanceToGoal: 0.95, now: resetTime, commanded: true))
        let adequate = watchdog.diagnosticSnapshot(distanceToGoal: 0.95, now: resetTime)
        XCTAssertEqual(adequate.bestDistance, 0.95)
        XCTAssertEqual(adequate.lastProgressAt, resetTime)
        XCTAssertEqual(adequate.progress, 0)
        XCTAssertEqual(adequate.elapsed, 0)
        let backwards = watchdog.diagnosticSnapshot(distanceToGoal: 1.05, now: resetTime.addingTimeInterval(1))
        XCTAssertEqual(backwards.progress ?? 0, -0.1, accuracy: 1e-12)
        XCTAssertEqual(backwards.elapsed, 1)
    }

    func testPulseStopFailureDeliveryCapturesAuthoritativeUnconfirmedLatch() async {
        var stops = 0
        let source = FollowRecoveryDiagnosticSourceFixture()
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in }, stopRover: {}, sleep: source.advance, stopRoverReceipt: {
                stops += 1
                let failed = stops == 2
                return .init(receipt: .init(httpStatus: failed ? 503 : 200, acknowledged: !failed,
                    acknowledgementUTC: failed ? nil : Date(), attempts: 1, outcome: failed ? "failed" : "acknowledged"),
                    failure: failed ? RoverControlError.serverError(503) : nil)
            }, poseSample: { source.snapshot }, sourceNow: { source.uptime }, sourceEvents: { source.events.stream },
            sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let motion = NavigationFollowMeMotion(navigation: controller)
        let stream = motion.motionFailures()
        let first = Task { () -> FollowMotionFailureDelivery? in
            for await failure in stream { return failure }; return nil
        }
        let context = FollowMotionRequestContext(sessionGeneration: 8, requestToken: 200, purpose: .followScan, phase: "searching")
        let result = await motion.perform(.scan(0.3), context: context)
        let failure = await first.value
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertEqual(failure?.context, result.context)
        XCTAssertEqual(failure?.stopOutcome, .failed)
        XCTAssertEqual(result.stopOutcome, .failed)
        XCTAssertEqual(result.stopReceipt?.httpStatus, 503)
        XCTAssertEqual(stops, 2, "The unconfirmed latch blocks any subsequent motion or implicit retry")
    }

    func testContextualSubscriptionPreservesExistingFailedSafetySnapshotWithoutInventedOperation() async {
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 },
            plan: { _, goal in [goal] }, lastAckAt: { nil }, sendCommand: { _ in },
            stopRover: { throw URLError(.cannotConnectToHost) }, sleep: { _ in })
        let original = await controller.rotateForFollowScan(by: 0.3)
        XCTAssertEqual(original, .failed(.commandFailed))
        let states = controller.safetyStates()
        let failures = controller.followMotionFailures()
        let legacy = Task { () -> NavigationSafetyState? in
            for await state in states { return state }; return nil
        }
        let contextual = Task { () -> FollowMotionFailureDelivery? in
            for await failure in failures { return failure }; return nil
        }
        for _ in 0..<30 { await Task.yield() }
        contextual.cancel()
        let state = await legacy.value
        let failure = await contextual.value
        XCTAssertEqual(state, .failed(.commandFailed))
        XCTAssertNotNil(failure, "Contextual subscription must not hide an already-failed safety state")
        XCTAssertEqual(failure?.reason, .commandFailed)
        XCTAssertEqual(failure?.stopOutcome, .failed)
        XCTAssertEqual(failure?.context, .unknown)
        XCTAssertNil(failure?.stopReceipt)
    }

    func testBlockedRequestCapturesUnconfirmedLatchWithoutInventingAnotherReceipt() async {
        var attempts = 0
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 },
            plan: { _, goal in [goal] }, lastAckAt: { nil }, sendCommand: { _ in }, stopRover: {}, sleep: { _ in },
            stopRoverReceipt: {
                attempts += 1
                return .init(receipt: .init(httpStatus: 503, acknowledged: false, acknowledgementUTC: nil,
                    attempts: 1, outcome: "failed"), failure: RoverControlError.serverError(503))
            })
        let motion = NavigationFollowMeMotion(navigation: controller)
        let firstContext = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 50, purpose: .followScan, phase: "searching")
        let first = await motion.perform(.scan(0.3), context: firstContext)
        XCTAssertEqual(first.stopOutcome, .failed)
        let blockedContext = FollowMotionRequestContext(sessionGeneration: 2, requestToken: 51, purpose: .followAlignment, phase: "aligning")
        let blocked = await motion.perform(.alignment(0.4), context: blockedContext)
        XCTAssertEqual(blocked.result, .failed(.commandFailed))
        XCTAssertEqual(blocked.context.request, blockedContext)
        XCTAssertEqual(blocked.stopOutcome, .failed)
        XCTAssertEqual(blocked.failure?.stopOutcome, .failed)
        XCTAssertNil(blocked.stopReceipt, "No transport request was entered for the blocked operation")
        XCTAssertEqual(attempts, 1)
        XCTAssertNotEqual(first.context.controllerOperationID, blocked.context.controllerOperationID)
    }

    func testLatePreStopFailureKeepsOldContextWhileReplacementFailsClosed() async {
        let gate = FollowDiagnosticSuspension()
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 },
            plan: { _, goal in [goal] }, lastAckAt: { nil }, sendCommand: { _ in },
            stopRover: { await gate.suspend(); throw URLError(.cannotConnectToHost) }, sleep: { _ in })
        let motion = NavigationFollowMeMotion(navigation: controller)
        let stream = motion.motionFailures()
        let delivered = Task { () -> [FollowMotionFailureDelivery] in
            var deliveries: [FollowMotionFailureDelivery] = []
            for await failure in stream {
                deliveries.append(failure)
                if deliveries.count == 2 { return deliveries }
            }
            return deliveries
        }
        let older = FollowMotionRequestContext(sessionGeneration: 3, requestToken: 52, purpose: .followScan,
            phase: "reacquiring", scanUsed: 1, scanRemaining: nil)
        let newer = FollowMotionRequestContext(sessionGeneration: 4, requestToken: 53, purpose: .followAlignment, phase: "aligning")
        let old = Task { await motion.perform(.scan(-0.3), context: older) }
        await gate.waitUntilEntered()
        var replacementStarted = false
        let replacement = Task { replacementStarted = true; return await motion.perform(.alignment(0.4), context: newer) }
        while !replacementStarted { await Task.yield() }
        gate.release()
        let first = await old.value
        let second = await replacement.value
        let failures = await delivered.value
        XCTAssertEqual(first.result, .failed(.commandFailed))
        XCTAssertEqual(second.result, .failed(.commandFailed))
        XCTAssertEqual(first.context.request, older)
        XCTAssertEqual(second.context.request, newer)
        XCTAssertEqual(first.context.profile?.pulseWait, 0.080)
        XCTAssertEqual(second.context.profile?.angularTolerance, 0.05)
        XCTAssertNil(first.context.targetYaw)
        XCTAssertNil(second.context.targetYaw)
        XCTAssertTrue(first.failure?.stale == true)
        XCTAssertEqual(failures.first(where: { $0.context.request == older })?.context, first.context)
        XCTAssertEqual(failures.first(where: { $0.context.request == newer })?.context, second.context)
        XCTAssertNil(second.stopReceipt, "The serialized prior failure prevented a new transport request")
        XCTAssertEqual(controller.safetyState, .failed(.commandFailed))
    }

    func testNonzeroAckDoesNotReusePreStopConfirmationAndStopFailureRetainsStall() async {
        var elapsed = 0.0
        let source = FollowRecoveryDiagnosticSourceFixture()
        let epoch = Date(timeIntervalSince1970: 1000)
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { epoch.addingTimeInterval(elapsed) },
            sendCommand: { _ in }, stopRover: {}, sleep: { elapsed += Self.seconds($0); await source.advance($0) },
            now: { epoch.addingTimeInterval(elapsed) },
            sendCommandReceipt: { _ in elapsed += 2.5; return .init(receipt: .init(httpStatus: 200, acknowledged: true,
                acknowledgementUTC: epoch.addingTimeInterval(elapsed), attempts: 1, outcome: "acknowledged"), failure: nil) },
            stopRoverReceipt: {
                let failed = elapsed >= 2.5
                return .init(receipt: .init(httpStatus: failed ? 503 : 200, acknowledged: !failed,
                    acknowledgementUTC: failed ? nil : epoch.addingTimeInterval(elapsed), attempts: 1,
                    outcome: failed ? "failed" : "acknowledged"), failure: failed ? RoverControlError.serverError(503) : nil)
            }, poseSample: { source.snapshot }, sourceNow: { source.uptime }, sourceEvents: { source.events.stream },
            sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let motion = NavigationFollowMeMotion(navigation: controller)
        let stream = motion.motionFailures()
        let first = Task { () -> FollowMotionFailureDelivery? in
            for await failure in stream { return failure }; return nil
        }
        let request = FollowMotionRequestContext(sessionGeneration: 5, requestToken: 98, purpose: .followScan, phase: "searching")
        let result = await motion.perform(.scan(0.3), context: request)
        let streamed = await first.value
        XCTAssertEqual(streamed?.reason, .stalled)
        XCTAssertEqual(streamed?.stopOutcome, .pending, "The pre-turn stop cannot confirm stopping after nonzero commands")
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertEqual(result.failure?.reason, .stalled, "Final-stop failure must not replace the captured typed stall")
        XCTAssertEqual(result.failure?.stopOutcome, .failed)
        XCTAssertEqual(result.failure?.context, streamed?.context)
        XCTAssertEqual(result.commandReceipt?.httpStatus, 200)
        XCTAssertEqual(result.stopReceipt?.httpStatus, 503)
    }

    func testCancelledPreStopRetainsRequestAndUnknownTargetWithoutSending() async throws {
        let gate = FollowDiagnosticSuspension()
        var stops = 0
        var sends = 0
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { nil }, sendCommand: { _ in sends += 1 },
            stopRover: { stops += 1; if stops == 1 { await gate.suspend() } }, sleep: { _ in })
        let motion = NavigationFollowMeMotion(navigation: controller)
        let request = FollowMotionRequestContext(sessionGeneration: 5, requestToken: 90, purpose: .followScan,
            phase: "reacquiring", scanUsed: 1.2, scanRemaining: 2.3)
        let scan = Task { await motion.perform(.scan(-0.3), context: request) }
        await gate.waitUntilEntered()
        let stop = Task { try await controller.stopAndConfirm() }
        await Task.yield()
        gate.release()
        try await stop.value
        let result = await scan.value
        XCTAssertEqual(result.result, .cancelled)
        XCTAssertEqual(result.context.request, request)
        XCTAssertEqual(result.context.purpose, .followScan)
        XCTAssertEqual(result.context.profile?.pulseWait, 0.080)
        XCTAssertEqual(result.context.requestedRotation, -0.3)
        XCTAssertNil(result.context.targetYaw)
        XCTAssertNotNil(result.context.controllerOperationID)
        XCTAssertEqual(sends, 0)
        XCTAssertEqual(controller.safetyState, .idle)
    }

    func testReplacingSuspendedRequestCannotRelabelOldOperation() async {
        let gate = FollowDiagnosticSuspension()
        var stops = 0
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 },
            plan: { _, goal in [goal] }, lastAckAt: { nil }, sendCommand: { _ in },
            stopRover: { stops += 1; if stops == 1 { await gate.suspend() } }, sleep: { _ in })
        let motion = NavigationFollowMeMotion(navigation: controller)
        let older = FollowMotionRequestContext(sessionGeneration: 5, requestToken: 91, purpose: .followScan, phase: "searching")
        let newer = FollowMotionRequestContext(sessionGeneration: 6, requestToken: 92, purpose: .followAlignment, phase: "aligning")
        let old = Task { await motion.perform(.scan(0.4), context: older) }
        await gate.waitUntilEntered()
        let replacement = Task { await motion.perform(.alignment(-0.4), context: newer) }
        await Task.yield()
        gate.release()
        let first = await old.value
        let second = await replacement.value
        XCTAssertEqual(first.result, .cancelled)
        XCTAssertEqual(first.context.request, older)
        XCTAssertEqual(first.context.purpose, .followScan)
        XCTAssertEqual(first.context.profile?.pulseWait, 0.080)
        XCTAssertEqual(second.result, .failed(.trackingLost))
        XCTAssertEqual(second.context.request, newer)
        XCTAssertEqual(second.context.purpose, .followAlignment)
        XCTAssertEqual(second.context.profile?.angularTolerance, 0.05)
        XCTAssertNotEqual(first.context.controllerOperationID, second.context.controllerOperationID)
    }

    func testAllFourAdapterRequestsDeliverActualPurposeOnEarlyFailure() async {
        let cases: [FollowMotionRequest] = [.scan(0.3), .alignment(-0.3), .ready, .following(Vec2(2, 0), 1.25)]
        for request in cases {
            let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 },
                plan: { _, goal in [goal] }, lastAckAt: { nil }, sendCommand: { _ in }, stopRover: {}, sleep: { _ in })
            let motion = NavigationFollowMeMotion(navigation: controller)
            let stream = motion.motionFailures()
            let streamed = Task { () -> FollowMotionFailureDelivery? in
                for await failure in stream { return failure }; return nil
            }
            let context = FollowMotionRequestContext(sessionGeneration: 7, requestToken: 123,
                purpose: request.purpose, phase: "captured_phase")
            let result = await motion.perform(request, context: context)
            let delivery = await streamed.value
            let reason: NavigationFailure = request.purpose == .followScan || request.purpose == .followAlignment
                ? .trackingLost : .noPose
            XCTAssertEqual(result.result, .failed(reason))
            XCTAssertEqual(result.context.request, context)
            XCTAssertEqual(result.context.purpose, request.purpose)
            XCTAssertEqual(result.context.profile != nil, request.purpose == .followScan || request.purpose == .followAlignment)
            XCTAssertEqual(result.context.requestedRotation, request.requestedRotation)
            XCTAssertNil(result.context.targetYaw)
            XCTAssertEqual(result.context, delivery?.context)
            XCTAssertEqual(delivery?.reason, reason)
        }
    }

    func testControllerUsesReturnedReceiptSnapshotsWithoutCallingVoidTransport() async {
        var yaw = 0.0
        let source = FollowRecoveryDiagnosticSourceFixture()
        var sent = false
        var legacySends = 0
        var legacyStops = 0
        let commandAck = Date(timeIntervalSince1970: 123)
        let stopAck = Date(timeIntervalSince1970: 456)
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: yaw) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in legacySends += 1 }, stopRover: { legacyStops += 1 },
            sleep: { duration in
                if sent, Self.seconds(duration) >= 0.299 { yaw = 0.3 }
                source.pose = .init(position: .zero, yaw: yaw)
                await source.advance(duration)
            },
            sendCommandReceipt: { _ in sent = true; return .init(receipt: .init(httpStatus: 204, acknowledged: true,
                acknowledgementUTC: commandAck, attempts: 2, outcome: "acknowledged"), failure: nil) },
            stopRoverReceipt: { .init(receipt: .init(httpStatus: 200, acknowledged: true,
                acknowledgementUTC: stopAck, attempts: 1, outcome: "acknowledged"), failure: nil) },
            poseSample: { source.snapshot }, sourceNow: { source.uptime }, sourceEvents: { source.events.stream },
            sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let request = FollowMotionRequestContext(sessionGeneration: 4, requestToken: 80, purpose: .followScan, phase: "searching")
        let result = await NavigationFollowMeMotion(navigation: controller).perform(.scan(0.3), context: request)
        XCTAssertEqual(result.result, .arrived)
        XCTAssertEqual(legacySends, 0)
        XCTAssertEqual(legacyStops, 0)
        XCTAssertEqual(result.context.requestedRotation, 0.3)
        XCTAssertEqual(result.context.targetYaw ?? 0, 0.3, accuracy: 1e-12)
        XCTAssertEqual(result.commandReceipt?.httpStatus, 204)
        XCTAssertEqual(result.commandReceipt?.attempts, 2)
        XCTAssertEqual(result.commandReceipt?.acknowledgementUTC, commandAck)
        XCTAssertEqual(result.stopReceipt?.httpStatus, 200)
        XCTAssertEqual(result.stopReceipt?.acknowledgementUTC, stopAck)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testVoidControllerTransportReportsUnknownMetadataDespiteConfirmedStop() async {
        var yaw = 0.0
        let controller = makeFollowController(pose: { yaw }, send: { _ in }, sleep: {
            if Self.seconds($0) >= 0.299 { yaw = 0.3 }
        })
        let request = FollowMotionRequestContext(sessionGeneration: 4, requestToken: 81, purpose: .followScan, phase: "searching")
        let result = await NavigationFollowMeMotion(navigation: controller).perform(.scan(0.3), context: request)
        XCTAssertEqual(result.result, .arrived)
        XCTAssertNotNil(result.commandReceipt)
        XCTAssertNil(result.commandReceipt?.httpStatus)
        XCTAssertNil(result.commandReceipt?.acknowledged)
        XCTAssertNotNil(result.stopReceipt)
        XCTAssertNil(result.stopReceipt?.httpStatus)
        XCTAssertNil(result.stopReceipt?.acknowledgementUTC)
        XCTAssertEqual(result.stopOutcome, .confirmed)
    }

    func testContextIsReservedBeforePreStopAndBothDeliveriesKeepIt() async {
        let gate = FollowDiagnosticSuspension()
        var stops = 0
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 },
            plan: { _, goal in [goal] }, lastAckAt: { nil }, sendCommand: { _ in },
            stopRover: { stops += 1; if stops == 1 { await gate.suspend() }; throw URLError(.cannotConnectToHost) }, sleep: { _ in })
        let adapter = NavigationFollowMeMotion(navigation: controller)
        let failures = adapter.motionFailures()
        let delivery = Task { () -> FollowMotionFailureDelivery? in
            for await failure in failures { return failure }; return nil
        }
        let request = FollowMotionRequestContext(sessionGeneration: 7, requestToken: 93, purpose: .followScan, phase: "reacquiring", scanUsed: 1, scanRemaining: 2)
        let operation = Task { await adapter.perform(.scan(0.3), context: request) }
        await gate.waitUntilEntered()
        gate.release()
        let result = await operation.value
        let streamed = await delivery.value
        XCTAssertEqual(result.result, .failed(.commandFailed))
        XCTAssertEqual(result.context.request, request)
        XCTAssertNotNil(result.context.controllerOperationID)
        XCTAssertNotEqual(result.context.controllerOperationID, request.requestToken)
        XCTAssertEqual(result.context.purpose, .followScan)
        XCTAssertEqual(result.context.profile?.pulseWait, 0.080)
        XCTAssertEqual(streamed?.context, result.context)
        XCTAssertEqual(result.failure?.stopOutcome, .failed)
        XCTAssertEqual(result.failure?.source, .result)
        XCTAssertEqual(streamed?.source, .stream)
    }

    func testLegacyConformerRetainsResultsWithoutInventedControllerFacts() async {
        let legacy = FollowMotionFake()
        legacy.readySignalResult = .failed(.stalled)
        let request = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 99, purpose: .followReady, phase: "signalingReady")
        let result = await legacy.performContextual(.ready, context: request)
        XCTAssertEqual(result.result, .failed(.stalled))
        XCTAssertEqual(result.context.request, request)
        XCTAssertNil(result.context.controllerOperationID)
        XCTAssertNil(result.context.purpose)
        XCTAssertNil(result.context.profile)
        XCTAssertEqual(result.failure?.stopOutcome, .unknown)
    }

    func testFollowAdapterSearchAndReacquisitionRequestAtMost80msThen300ms() async {
        for angle in [0.3, -0.3] {
            var yaw = 0.0
            var waits: [Double] = []
            var commands: [WheelCommand] = []
            let controller = makeFollowController(pose: { yaw }, send: { commands.append($0) }, sleep: {
                waits.append(Self.seconds($0))
                if waits.count == 2 { yaw = angle }
            })
            let result = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: angle)
            XCTAssertEqual(result, .arrived)
            XCTAssertEqual(waits.count, 2)
            XCTAssertEqual(waits.first ?? 0, 0.080, accuracy: 1e-12)
            XCTAssertEqual(waits.last ?? 0, 0.300, accuracy: 1e-12)
            XCTAssertEqual(commands.count, 1)
            XCTAssertEqual(commands.first?.left ?? 0, angle > 0 ? -0.25 : 0.25, accuracy: 1e-12)
            XCTAssertEqual(commands.first?.right ?? 0, angle > 0 ? 0.25 : -0.25, accuracy: 1e-12)
        }
    }

    func testGenericScanKeepsCoybotPulseAndSettleDurations() async {
        var yaw = 0.0
        var waits: [Double] = []
        var commands: [WheelCommand] = []
        let controller = makeController(pose: { yaw }, send: { commands.append($0) }, sleep: {
            waits.append(Self.seconds($0))
            if waits.count == 2 { yaw = 0.3 }
        })
        await controller.rotateForScan(by: 0.3)
        XCTAssertEqual(waits, [RoverConfig.scanTurnPulseDuration, RoverConfig.scanTurnSettleDuration])
        XCTAssertEqual(commands.first?.left ?? 0, -0.25, accuracy: 1e-12)
        XCTAssertEqual(commands.first?.right ?? 0, 0.25, accuracy: 1e-12)
    }

    func testGenericScanAndAlignmentDoNotConsultFollowSourceClockOrProvider() async {
        for scan in [true, false] {
            var yaw = 0.0
            var sourceReads = 0
            let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: yaw) },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
                sendCommand: { _ in }, stopRover: {}, sleep: { _ in yaw = 0.3 },
                poseSample: { XCTFail("Generic motion cannot depend on follow provenance"); return nil },
                sourceNow: { sourceReads += 1; return .nan })
            if scan { await controller.rotateForScan(by: 0.3) }
            else { let result = await controller.rotateAndWait(by: 0.3); XCTAssertEqual(result, .arrived) }
            XCTAssertEqual(sourceReads, 0)
        }
    }

    func testFollowFixedSignedMagnitudeOutsideTolerance() async {
        for angle in [0.52, -0.52, 0.13, -0.13] {
            var yaw = 0.0
            var commands: [WheelCommand] = []
            let controller = makeFollowController(pose: { yaw }, send: { commands.append($0) }, sleep: {
                if Self.seconds($0) >= 0.299 { yaw = angle }
            })
            let result = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: angle)
            XCTAssertEqual(result, .arrived)
            XCTAssertEqual(commands.count, 1)
            XCTAssertEqual(commands.first?.left ?? 0, angle > 0 ? -0.25 : 0.25, accuracy: 1e-12)
            XCTAssertEqual(commands.first?.right ?? 0, angle > 0 ? 0.25 : -0.25, accuracy: 1e-12)
        }
        var commands = 0
        let withinTolerance = makeFollowController(pose: { 0 }, send: { _ in commands += 1 }, sleep: { _ in })
        let result = await withinTolerance.rotateForFollowScan(by: 0.12)
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(commands, 0, "The existing seven-degree scan tolerance remains unchanged")
    }

    func testGenericAlignmentRemainsContinuousAndFollowAlignmentUsesBursts() async {
        for follow in [false, true] {
            var yaw = 0.0
            var waits: [Double] = []
            var commands: [WheelCommand] = []
            let controller: FollowNavigationController
            if follow {
                controller = makeFollowController(pose: { yaw }, send: { commands.append($0) }, sleep: {
                    waits.append(Self.seconds($0))
                    if Self.seconds($0) >= 0.299 { yaw = 0.3 }
                })
            } else {
                controller = makeController(pose: { yaw }, send: { commands.append($0) }, sleep: {
                    waits.append(Self.seconds($0)); yaw = 0.3
                })
            }
            let result: NavigationResult
            if follow { result = await NavigationFollowMeMotion(navigation: controller).alignTowardPerson(by: 0.3) }
            else { result = await controller.rotateAndWait(by: 0.3) }
            XCTAssertEqual(result, .arrived)
            XCTAssertEqual(waits.count, follow ? 2 : 1)
            XCTAssertEqual(waits[0], follow ? 0.080 : 0.1, accuracy: 1e-12)
            if follow { XCTAssertEqual(waits[1], 0.300, accuracy: 1e-12) }
            XCTAssertEqual(abs(commands.first?.left ?? 0), 0.25, accuracy: 1e-12)
        }
    }

    func testFollowInclusiveSevenDegreesConfirmsStopWithoutPulse() async {
        for angle in [7 * Double.pi / 180, -7 * Double.pi / 180] {
            var sends = 0
            var stops = 0
            let source = FollowRecoveryDiagnosticSourceFixture()
            let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
                forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
                sendCommand: { _ in sends += 1 }, stopRover: { stops += 1 }, sleep: source.advance,
                poseSample: { source.snapshot }, sourceNow: { source.uptime }, sourceEvents: { source.events.stream },
                sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let result = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: angle)
            XCTAssertEqual(result, .arrived)
            XCTAssertEqual(sends, 0)
            XCTAssertEqual(stops, 1, "A fresh initial stopped evaluation arrives without another motor command")
        }
    }

    func testFrozenOldSourceFrameCannotAuthorizeFollowPulse() async {
        var sends = 0
        var stops = 0
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in sends += 1 }, stopRover: { stops += 1 }, sleep: { _ in },
            poseSample: { NavigationPoseSample(pose: Pose2D(position: .zero, yaw: 0),
                frameID: ARFrameID(generation: 8, sequence: 1), sourceTimestamp: 99.499,
                trackingQuality: .normal) }, sourceNow: { 100 })
        let result = await controller.performFollowMotion(.scan(0.3), context: nil)
        XCTAssertEqual(result.result, .failed(.trackingLost))
        XCTAssertEqual(result.stopOutcome, .confirmed)
        XCTAssertEqual(sends, 0)
        XCTAssertEqual(stops, 1, "The preflight stop is already authoritatively confirmed")
    }

    func testFutureSourceTimeCannotAuthorizeFollowPulse() async {
        let (result, commands) = await scanWithSource(timestamp: 100.001)
        XCTAssertEqual(result, .failed(.trackingLost))
        XCTAssertTrue(commands.isEmpty)
    }

    func testSourceFreshnessBoundaryAndMissingEnrichedPoseStayFailClosed() async {
        let (boundary, boundaryCommands) = await scanWithSource(timestamp: 99.5)
        XCTAssertEqual(boundary, .arrived)
        XCTAssertEqual(boundaryCommands, [WheelCommand(left: -0.25, right: 0.25)])
        let (expired, expiredCommands) = await scanWithSource(timestamp: 99.499999)
        XCTAssertEqual(expired, .failed(.trackingLost))
        XCTAssertTrue(expiredCommands.isEmpty)
        let (missing, missingCommands) = await scanWithSource(timestamp: 100, pose: nil)
        XCTAssertEqual(missing, .failed(.trackingLost))
        XCTAssertTrue(missingCommands.isEmpty)
        var sends = 0
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in sends += 1 }, stopRover: {}, sleep: { _ in },
            poseSample: { nil }, sourceNow: { 100 })
        let unavailable = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3)
        XCTAssertEqual(unavailable, .failed(.trackingLost))
        XCTAssertEqual(sends, 0, "Configured enriched nil must never fall back to a legacy pose")
    }

    func testNonfiniteSourceTimeCannotAuthorizeFollowPulse() async {
        for timestamp in [Double.nan, .infinity, -.infinity] {
            let (result, commands) = await scanWithSource(timestamp: timestamp)
            XCTAssertEqual(result, .failed(.trackingLost))
            XCTAssertTrue(commands.isEmpty)
        }
    }

    func testUnhealthyTrackingCannotAuthorizeFollowPulse() async {
        for tracking in [ARTrackingQuality.limited, .unavailable] {
            let (result, commands) = await scanWithSource(timestamp: 100, tracking: tracking)
            XCTAssertEqual(result, .failed(.trackingLost))
            XCTAssertTrue(commands.isEmpty)
        }
    }

    func testNonfinitePoseCannotAuthorizeFollowPulse() async {
        for pose in [Pose2D(position: Vec2(.nan, 0), yaw: 0),
                     Pose2D(position: Vec2(0, .infinity), yaw: 0),
                     Pose2D(position: .zero, yaw: .nan)] {
            let (result, commands) = await scanWithSource(timestamp: 100, pose: pose)
            XCTAssertEqual(result, .failed(.trackingLost))
            XCTAssertTrue(commands.isEmpty)
        }
    }

    func testARGenerationChangeInhibitsNextPulse() async {
        let source = FollowRecoveryDiagnosticSourceFixture()
        source.generation = 8
        source.capture(after: 0)
        var sends = 0
        let yaw = 0.0
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: yaw) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in sends += 1 }, stopRover: {}, sleep: { duration in
                if sends > 0, Self.seconds(duration) >= 0.299 { source.generation = 9 }
                await source.advance(duration)
            }, poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let result = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3)
        XCTAssertEqual(result, .failed(.trackingLost))
        XCTAssertEqual(sends, 1)
    }

    func testFrozenFrameExpiresWhileChangingFreshFramesWithFlatYawStillStall() async {
        for frozen in [true, false] {
            let source = FollowRecoveryDiagnosticSourceFixture()
            source.uptime = 100
            source.capture(after: 0)
            var sends = 0
            let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 },
                plan: { _, goal in [goal] }, lastAckAt: { Date(timeIntervalSince1970: source.uptime) },
                sendCommand: { _ in sends += 1 }, stopRover: {}, sleep: { duration in
                    if frozen, sends > 0 { source.uptime += Self.seconds(duration) }
                    else { await source.advance(duration) }
                }, now: { Date(timeIntervalSince1970: source.uptime) }, poseSample: { source.snapshot },
                sourceNow: { source.uptime }, sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let result = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3)
            XCTAssertEqual(result, .failed(frozen ? .trackingLost : .stalled))
            XCTAssertEqual(sends, frozen ? 1 : 7)
            XCTAssertEqual(source.uptime, frozen ? 100.8 : 102.8, accuracy: 1e-6)
        }
    }

    func testReplacementDuringEnrichedFeedbackFencesOldScanAndReadySend() async {
        for request in [FollowMotionRequest.scan(0.3), .ready] {
            let gate = FollowDiagnosticSuspension()
            var yaw = 0.0
            var commands: [WheelCommand] = []
            let source = FollowRecoveryDiagnosticSourceFixture()
            let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 },
                plan: { _, goal in [goal] }, lastAckAt: {
                    if !gate.entered { await gate.suspend() }; return Date()
                }, sendCommand: { commands.append($0) }, stopRover: {}, sleep: { duration in
                    if !commands.isEmpty, Self.seconds(duration) >= 0.299 { yaw = -0.3 }
                    source.pose = .init(position: .zero, yaw: yaw)
                    await source.advance(duration)
                }, poseSample: { source.snapshot }, sourceNow: { source.uptime },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            let motion = NavigationFollowMeMotion(navigation: controller)
            let older = FollowMotionRequestContext(sessionGeneration: 1, requestToken: 1, purpose: request.purpose, phase: "old")
            let newer = FollowMotionRequestContext(sessionGeneration: 2, requestToken: 2, purpose: .followScan, phase: "new")
            let old = Task { await motion.perform(request, context: older) }
            await gate.waitUntilEntered()
            let replacement = Task { await motion.perform(.scan(-0.3), context: newer) }
            for _ in 0..<30 { await Task.yield() }
            gate.release()
            let first = await old.value
            let second = await replacement.value
            XCTAssertEqual(first.result, .cancelled)
            XCTAssertEqual(second.result, .arrived)
            XCTAssertEqual(commands, [WheelCommand(left: 0.25, right: -0.25)], "Only the replacement may send")
            XCTAssertEqual(second.context.request, newer)
        }
    }

    func testSuspendedFeedbackCannotAuthorizeExpiredSourcePose() async {
        let gate = FollowDiagnosticSuspension()
        let source = FollowRecoveryDiagnosticSourceFixture()
        source.uptime = 100
        source.capture(after: 0)
        var sends = 0
        let controller = FollowNavigationController(currentPose: { source.pose },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: {
                if !gate.entered { await gate.suspend() }; return Date()
            }, sendCommand: { _ in sends += 1 }, stopRover: {}, sleep: { duration in
                if gate.entered { try? await Task.sleep(for: duration) }
                else { await source.advance(duration) }
            },
            poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let scan = Task { await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3) }
        await gate.waitUntilEntered()
        source.snapshot = .init(pose: source.pose, frameID: .init(generation: 4, sequence: 12),
            sourceTimestamp: 99.499, trackingQuality: .normal)
        controller.ingestFollowTurnSource(source.snapshot)
        gate.release()
        let result = await scan.value
        XCTAssertEqual(result, .failed(.trackingLost))
        XCTAssertEqual(sends, 0)
    }

    func testSuspendedFeedbackUsesCurrentYawForToleranceAndPulseSign() async {
        for updatedYaw in [0.3, 0.43, -0.13] {
            let gate = FollowDiagnosticSuspension()
            let source = FollowRecoveryDiagnosticSourceFixture()
            var commands: [WheelCommand] = []
            let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 },
                plan: { _, goal in [goal] }, lastAckAt: {
                    if !gate.entered { await gate.suspend() }; return Date()
                }, sendCommand: { commands.append($0) }, stopRover: {}, sleep: { duration in
                    if !commands.isEmpty, Self.seconds(duration) >= 0.299 { source.pose = .init(position: .zero, yaw: 0.3) }
                    await source.advance(duration)
                }, poseSample: { source.snapshot }, sourceNow: { source.uptime },
                sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
            source.controller = controller
            // A fixed absolute stage allows feedback to change the actual error;
            // relative turns now resolve only after this stopped-source boundary.
            let request = FollowRecoveryHeadingRequest(stageHeading: 0.3, authorization: .init(episodeID: UUID(),
                expectedGeneration: 4, deadline: 12, now: { source.uptime }, canContinue: { true }))
            let scan = Task { await NavigationFollowMeMotion(navigation: controller).performRecoveryHeading(request,
                context: .init(sessionGeneration: 1, requestToken: 1, purpose: .followScan, phase: "recovery")).result }
            await gate.waitUntilEntered()
            source.pose = .init(position: .zero, yaw: updatedYaw)
            source.capture(after: 0.001)
            gate.release()
            let result = await scan.value
            XCTAssertEqual(result, .arrived)
            let expected: [WheelCommand] = updatedYaw == 0.3 ? [] : [WheelCommand(
                left: updatedYaw > 0.3 ? 0.25 : -0.25, right: updatedYaw > 0.3 ? -0.25 : 0.25)]
            XCTAssertEqual(commands, expected)
        }
    }

    func testTraceRetainsExactPrePostSourceProvenanceWithoutExtraReads() async throws {
        let clock = FollowDiagnosticTestClock()
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "source", monotonic: { clock.monotonic }, utc: { clock.utc }, sink: sink.append)
        let source = FollowRecoveryDiagnosticSourceFixture()
        source.generation = 8
        source.uptime = 100
        source.snapshot = .init(pose: source.pose, frameID: .init(generation: 8, sequence: 1),
            sourceTimestamp: 99.75, trackingQuality: .normal)
        var sent = false
        var stops = 0
        var stopUptime = source.uptime
        var reads = 0
        let controller = FollowNavigationController(currentPose: { XCTFail("Enriched path must not read legacy pose"); return nil },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { _ in sent = true }, stopRover: { stops += 1; stopUptime = source.uptime }, sleep: { duration in
                clock.monotonic += Self.seconds(duration)
                if sent, stops >= 2, source.uptime + Self.seconds(duration) >= stopUptime + 0.300 {
                    source.pose = .init(position: .zero, yaw: 0.3)
                }
                source.capture(after: Self.seconds(duration), age: 0.25)
            }, diagnosticEmitter: emitter, poseSample: {
                reads += 1
                return source.snapshot
            }, sourceNow: { source.uptime }, sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let result = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3)
        XCTAssertEqual(result, .arrived)
        XCTAssertEqual(reads, 1)
        let pulse = try XCTUnwrap(records(sink).first { $0["event"] as? String == "follow_scan.pulse_complete" })
        XCTAssertEqual(pulse["pre_source_frame_id"] as? String, "8:3")
        XCTAssertEqual(pulse["post_source_frame_id"] as? String, "8:6")
        XCTAssertEqual(pulse["pre_source_generation"] as? Double, 8)
        XCTAssertEqual(pulse["post_source_generation"] as? Double, 8)
        XCTAssertEqual(try XCTUnwrap(pulse["pre_source_timestamp_s"] as? Double), 100.05, accuracy: 1e-12)
        XCTAssertEqual(try XCTUnwrap(pulse["post_source_timestamp_s"] as? Double), 100.43, accuracy: 1e-12)
        XCTAssertEqual(pulse["pre_source_age_s"] as? Double, 0.25)
        XCTAssertEqual(pulse["post_source_age_s"] as? Double, 0.25)
        XCTAssertEqual(try XCTUnwrap(pulse["pre_pose_read_monotonic_s"] as? Double), 100.3, accuracy: 1e-12)
        XCTAssertEqual(try XCTUnwrap(pulse["post_pose_read_monotonic_s"] as? Double), 100.68, accuracy: 1e-12)
        XCTAssertEqual(pulse["pose_pairing"] as? String, "independently_sampled")
        XCTAssertEqual(pulse["pose_source_age_status"] as? String, "available")
        XCTAssertEqual(pulse["post_tracking_state"] as? String, "normal")
        XCTAssertEqual(pulse["post_source_availability"] as? String, "available")
    }

    func testExpiredPostSampleTraceRetainsActualSourceAndRejectionReason() async throws {
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "expired", monotonic: { 0 }, utc: { Date() }, sink: sink.append)
        let source = FollowRecoveryDiagnosticSourceFixture()
        source.uptime = 100
        source.capture(after: 0)
        var sent = false
        var pre: NavigationPoseSample?
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 },
            plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: { _ in sent = true; pre = source.snapshot }, stopRover: {},
            sleep: { duration in
                if sent, Self.seconds(duration) >= 0.299 {
                    source.uptime += Self.seconds(duration)
                    source.snapshot = .init(pose: source.pose, frameID: pre?.frameID,
                        sourceTimestamp: 99.499, trackingQuality: .normal)
                    source.controller?.ingestFollowTurnSource(source.snapshot)
                } else { await source.advance(duration) }
            }, diagnosticEmitter: emitter, poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        let result = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3)
        XCTAssertEqual(result, .failed(.trackingLost))
        let failure = try XCTUnwrap(records(sink).first { $0["event"] as? String == "follow_scan.failure" })
        XCTAssertEqual(failure["post_source_frame_id"] as? String, "4:12")
        XCTAssertEqual(failure["post_source_timestamp_s"] as? Double, 99.499)
        XCTAssertEqual(failure["post_source_availability"] as? String, "stale_source")
        XCTAssertEqual(failure["pose_pairing"] as? String, "same_frame")
        XCTAssertTrue(failure["watchdog_progress_rad"] is NSNull)
    }

    func testPreflightRejectionLogsAvailableSourceFacts() async throws {
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "preflight", monotonic: { 0 }, utc: { Date() }, sink: sink.append)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        let snapshot = NavigationPoseSample(pose: Pose2D(position: .zero, yaw: 0),
            frameID: ARFrameID(generation: 8, sequence: 1), sourceTimestamp: 99, trackingQuality: .normal)
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 }, plan: { _, goal in [goal] },
            lastAckAt: { nil }, sendCommand: { _ in XCTFail("Stale preflight cannot send") }, stopRover: {}, sleep: { _ in },
            diagnosticEmitter: emitter, poseSample: { snapshot }, sourceNow: { 100 },
            sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
        let result = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3)
        XCTAssertEqual(result, .failed(.trackingLost))
        let failure = try XCTUnwrap(records(sink).first { $0["event"] as? String == "follow_scan.failure" })
        XCTAssertEqual(failure["post_source_frame_id"] as? String, "8:1")
        XCTAssertEqual(failure["post_source_availability"] as? String, "stale_source")
        XCTAssertEqual(failure["post_source_age_s"] as? Double, 1)
    }

    func testNonfiniteSourceTraceCannotClaimAvailableSourceAge() async throws {
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "nonfinite", monotonic: { 0 }, utc: { Date() }, sink: sink.append)
        let events = AsyncStream<NavigationPoseSample>.makeStream()
        let snapshot = NavigationPoseSample(pose: Pose2D(position: .zero, yaw: 0),
            frameID: ARFrameID(generation: 8, sequence: 1), sourceTimestamp: .nan, trackingQuality: .normal)
        let controller = FollowNavigationController(currentPose: { nil }, forwardClearance: { 2 }, plan: { _, goal in [goal] },
            lastAckAt: { nil }, sendCommand: { _ in XCTFail("Invalid source cannot send") }, stopRover: {}, sleep: { _ in },
            diagnosticEmitter: emitter, poseSample: { snapshot }, sourceNow: { 100 },
            sourceEvents: { events.stream }, sourceStopSnapshot: { snapshot })
        let result = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3)
        XCTAssertEqual(result, .failed(.trackingLost))
        let failure = try XCTUnwrap(records(sink).first { $0["event"] as? String == "follow_scan.failure" })
        XCTAssertEqual(failure["pose_source_age_status"] as? String, "nonfinite")
        XCTAssertEqual(failure["post_source_availability"] as? String, "nonfinite_source_time")
        XCTAssertTrue(failure["post_source_age_s"] is NSNull)
    }

    private func scanWithSource(timestamp: Double, tracking: ARTrackingQuality = .normal,
                                 pose: Pose2D? = Pose2D(position: .zero, yaw: 0)) async -> (NavigationResult, [WheelCommand]) {
        var commands: [WheelCommand] = []
        let source = FollowRecoveryDiagnosticSourceFixture()
        source.generation = 8
        source.uptime = 100
        source.snapshot = .init(pose: pose, frameID: .init(generation: 8, sequence: 1),
            sourceTimestamp: timestamp, trackingQuality: tracking)
        let controller = FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: 0) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: { commands.append($0) }, stopRover: {}, sleep: { duration in
                if !commands.isEmpty, Self.seconds(duration) >= 0.299 { source.pose = .init(position: .zero, yaw: 0.3) }
                await source.advance(duration)
            }, poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        return (await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: 0.3), commands)
    }

    func testFollowWraparoundAndOvershootKeepFixedOppositeSignedPulse() async {
        for sign in [1.0, -1.0] {
            var yaw = sign * 3.10
            var commands: [WheelCommand] = []
            var settles = 0
            let target = sign * -2.883185307179586
            let controller = makeFollowController(pose: { yaw }, send: { commands.append($0) }, sleep: {
                if Self.seconds($0) >= 0.299 {
                    settles += 1
                    yaw = settles == 1 ? target + sign * 0.13 : target
                }
            })
            let result = await NavigationFollowMeMotion(navigation: controller).rotateForScan(by: sign * 0.3)
            XCTAssertEqual(result, .arrived)
            XCTAssertEqual(commands, [WheelCommand(left: -sign * 0.25, right: sign * 0.25),
                                      WheelCommand(left: sign * 0.25, right: -sign * 0.25)])
        }
    }

    private func makeController(pose: @escaping () -> Double,
                                send: @escaping (WheelCommand) async throws -> Void,
                                sleep: @escaping (Duration) async -> Void) -> FollowNavigationController {
        FollowNavigationController(currentPose: { Pose2D(position: .zero, yaw: pose()) },
            forwardClearance: { 2 }, plan: { _, goal in [goal] }, lastAckAt: { Date() },
            sendCommand: send, stopRover: {}, sleep: sleep)
    }

    /// Follow-only simulated captures. Reads never advance source provenance;
    /// movement begins only after a real nonzero sender has been invoked.
    private func makeFollowController(pose: @escaping () -> Double,
                                      send: @escaping (WheelCommand) async throws -> Void,
                                      sleep: @escaping (Duration) async -> Void,
                                      diagnosticEmitter: FollowDiagnosticEmitter? = nil) -> FollowNavigationController {
        let source = FollowRecoveryDiagnosticSourceFixture()
        source.pose = .init(position: .zero, yaw: pose())
        source.capture(after: 0)
        var sent = false
        let controller = FollowNavigationController(currentPose: { source.pose }, forwardClearance: { 2 },
            plan: { _, goal in [goal] }, lastAckAt: { Date() }, sendCommand: {
                sent = true
                try await send($0)
            }, stopRover: {}, sleep: { duration in
                if sent {
                    await sleep(duration)
                    source.pose = .init(position: .zero, yaw: pose())
                }
                await source.advance(duration)
            }, diagnosticEmitter: diagnosticEmitter, poseSample: { source.snapshot }, sourceNow: { source.uptime },
            sourceEvents: { source.events.stream }, sourceStopSnapshot: { source.snapshot })
        source.controller = controller
        return controller
    }

    private static func seconds(_ duration: Duration) -> Double {
        Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18
    }
}

@MainActor
private final class FollowRecoveryDiagnosticSourceFixture {
    weak var controller: FollowNavigationController?
    var uptime = 8.0
    var generation: UInt64 = 4
    let events = AsyncStream<NavigationPoseSample>.makeStream()
    var pose = Pose2D(position: .zero, yaw: 0)
    var snapshot = NavigationPoseSample(pose: .init(position: .zero, yaw: 0),
        frameID: .init(generation: 4, sequence: 10), sourceTimestamp: 7.99, trackingQuality: .normal)

    func advance(_ duration: Duration) async {
        await Task.yield()
        guard !Task.isCancelled else { return }
        capture(after: Double(duration.components.seconds) + Double(duration.components.attoseconds) / 1e18)
    }

    func capture(after elapsed: Double, age: Double = 0) {
        uptime += elapsed
        snapshot = .init(pose: pose,
            frameID: .init(generation: generation, sequence: snapshot.frameID!.sequence + 1),
            sourceTimestamp: uptime - age, trackingQuality: .normal)
        controller?.ingestFollowTurnSource(snapshot)
    }
}

/// Read by both MainActor controller code and synchronous transport eligibility.
/// Only the test driver writes time; sleeps and safety monitors cannot advance it.
private final class FollowRecoveryDriverClock: @unchecked Sendable {
    private let lock = NSLock()
    private var value = 8.0
    var now: Double {
        get { lock.withLock { value } }
        set { lock.withLock { value = newValue } }
    }
}
