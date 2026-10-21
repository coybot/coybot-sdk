import Foundation
import RoverNav

/// Bounded per-operation diagnostic facts. No pose provider, timer or motor authority.
@MainActor
final class FollowScanDiagnosticTrace {
    private let emitter: FollowDiagnosticEmitter
    private let started: Double
    private var pulseIndex: Int?
    private var pre: RotationPoseDiagnosticSample?
    private var post: RotationPoseDiagnosticSample?
    private var checkpoint: RotationPoseDiagnosticSample?
    private var checkpointMonotonic: Double?
    private var watchdog: DriveProgressWatchdog.DiagnosticSnapshot?
    private var stageStart: Double?
    private var waitDuration: Double?
    private var stopSequence = 0
    private(set) var stage = "operation_begin"
    private var pendingSettle = false
    private var finalSample: RotationPoseDiagnosticSample?
    private var cancelled = false
    private var timings: [String: FollowDiagnosticValue] = [:]
    private var settleEnd: Double?
    private var insidePulse = false
    private var failedStage: String?

    func enter(_ name: String) { stage = name; stageStart = emitter.hostTime() }

    func cancel(origin: String, evidence: FollowMotionOperationEvidence, latch: Bool) {
        guard !cancelled else { return }
        cancelled = true
        emit("cancel", evidence: evidence, latch: latch, outcome: "cancelled", fields: [
            "cancel_origin": .string(origin), "interrupted_stage": .string(stage),
            "cancel_monotonic_s": .number(emitter.hostTime())])
    }

    func failure(_ reason: NavigationFailure, evidence: FollowMotionOperationEvidence, latch: Bool) {
        let resolution = evidence.failure(source: .stream).map { FollowMotionFailureResolution($0) }
        emit("failure", evidence: evidence, latch: latch, outcome: "failed",
            reason: reason == .stalled ? "no_yaw_progress" : (reason == .rotationResolutionInsufficient ?
                (resolution?.diagnosticReason ?? "rotation_resolution_insufficient") : String(describing: reason)), fields: [
                "failed_stage": .string(reason == .stalled ? "watchdog" : (failedStage ?? stage)),
                "typed_reason": .string(String(describing: reason)),
                "formatter_message": resolution.map { .string($0.message) } ?? .null,
                "priority": resolution.map { .number(Double($0.priority)) } ?? .null])
    }

    init(emitter: FollowDiagnosticEmitter) {
        self.emitter = emitter
        started = emitter.hostTime()
    }

    func emit(_ name: String, evidence: FollowMotionOperationEvidence, latch: Bool,
              outcome: String? = nil, reason: String? = nil,
              fields: [String: FollowDiagnosticValue] = [:]) {
        let context = evidence.context
        var payload = RotationDiagnosticMeasurement(pre: pre, post: post, targetYaw: context.targetYaw).payload
        let profile = context.profile
        let operationFields: [String: FollowDiagnosticValue] = [
            "request_token": context.request.map { .number(Double($0.requestToken)) } ?? .null,
            "episode_id": evidence.recoveryEpisodeID.map { .string($0.uuidString) } ?? .null,
            "stage_index": evidence.recoveryStageIndex.map { .number(Double($0)) } ?? .null,
            "segment_index": evidence.recoverySegmentIndex.map { .number(Double($0)) } ?? .null,
            "stage_segment_availability": .string(evidence.recoveryStageIndex == nil ? "not_supplied" : "captured_coordinator_cursor"),
            "requested_increment_rad": context.requestedRotation.map { .number($0) } ?? .null,
            "requested_scan_used_rad": context.request?.scanUsed.map { .number($0) } ?? .null,
            "requested_scan_remaining_rad": context.request?.scanRemaining.map { .number($0) } ?? .null,
            "operation_start_monotonic_s": .number(started),
            "operation_elapsed_s": .number(emitter.hostTime() - started),
            "profile_pulse_wait_s": profile.map { .number($0.pulseWait) } ?? .null,
            "profile_settle_s": profile.map { .number($0.settleWait) } ?? .null,
            "profile_wheel_cap_mps": profile.map { .number($0.wheelCap) } ?? .null,
            "profile_wheel_floor_mps": profile.map { .number($0.wheelFloor) } ?? .null,
            "profile_command_law": profile.map { .string($0.commandLaw) } ?? .null,
            "profile_yaw_gain_active": profile.map { .bool($0.yawGainActive) } ?? .null,
            "profile_yaw_gain": profile.map { .number($0.yawGain) } ?? .null,
            "profile_angular_tolerance_rad": profile.map { .number($0.angularTolerance) } ?? .null,
        ]
        let watchdogFields: [String: FollowDiagnosticValue] = [
            "watchdog_checkpoint_yaw_rad": checkpoint?.finiteYaw.map { .number($0) } ?? .null,
            "watchdog_checkpoint_monotonic_s": checkpointMonotonic.map { .number($0) } ?? .null,
            "watchdog_best_distance_rad": watchdog?.bestDistance.map { .number($0) } ?? .null,
            "watchdog_progress_rad": watchdog?.progress.map { .number($0) } ?? .null,
            "watchdog_elapsed_s": watchdog?.elapsed.map { .number($0) } ?? .null,
            "watchdog_elapsed_clock": .string("controller_watchdog_date"),
            "watchdog_checkpoint_wall_ms": watchdog?.lastProgressAt.map { .number($0.timeIntervalSince1970 * 1000) } ?? .null,
            "watchdog_checkpoint_clock": .string("controller_watchdog_date"),
            "watchdog_required_progress_rad": .number(watchdog?.requiredProgress ?? 0.05),
            "watchdog_interval_s": .number(watchdog?.interval ?? 2.5),
        ]
        let stageFields: [String: FollowDiagnosticValue] = [
            "stage": .string(stage),
            "last_reached_stage": .string(stage),
            "stage_start_monotonic_s": stageStart.map { .number($0) } ?? .null,
            "stop_outcome": .string(evidence.stopOutcome.rawValue),
            "stop_unconfirmed": .bool(latch),
            "fenced": .bool(evidence.fenced),
            "final_yaw_rad": finalSample?.finiteYaw.map { .number($0) } ?? .null,
            "final_error_rad": finalSample?.finiteYaw.flatMap { yaw in context.targetYaw.map { .number(RotationDiagnosticMeasurement.normalize($0 - yaw)) } } ?? .null,
            "primary_failure": evidence.primaryFailure.map { .string(String(describing: $0)) } ?? .null
        ]
        payload.merge(operationFields) { _, value in value }
        payload.merge(watchdogFields) { _, value in value }
        payload.merge(stageFields) { _, value in value }
        payload.merge(evidence.burstTrace?.fields ?? [:]) { _, value in value }
        if evidence.recoveryEpisodeID != nil {
            payload.merge(FollowReacquisitionDiagnostics.controllerPayload(evidence)) { _, value in value }
        }
        for timing in ["pulse_begin_monotonic_s", "send_start_monotonic_s", "send_end_monotonic_s", "send_host_duration_s",
                       "pulse_wait_start_monotonic_s", "pulse_wait_end_monotonic_s", "pulse_wait_host_duration_s",
                       "settle_start_monotonic_s", "settle_end_monotonic_s", "settle_host_duration_s"] {
            payload[timing] = timings[timing] ?? .null
        }
        let checkpointYaw = checkpoint?.finiteYaw
        payload["watchdog_checkpoint_yaw_rad_display"] = checkpointYaw.map { .string(RotationDiagnosticMeasurement.angleDisplay($0)) } ?? .null
        payload["watchdog_checkpoint_yaw_rad_availability"] = .string(checkpointYaw == nil ? "unavailable" : "available")
        for key in ["watchdog_checkpoint_monotonic_s", "watchdog_best_distance_rad", "watchdog_progress_rad", "watchdog_elapsed_s"] {
            payload[key + "_availability"] = .string(payload[key] == .null ? "unavailable" : "available")
        }
        for (key, value) in [
            ("final_yaw_rad", finalSample?.finiteYaw),
            ("final_error_rad", finalSample?.finiteYaw.flatMap { yaw in context.targetYaw.map { RotationDiagnosticMeasurement.normalize($0 - yaw) } }),
            ("profile_angular_tolerance_rad", profile?.angularTolerance)
        ] {
            payload[key + "_display"] = value.map { .string(RotationDiagnosticMeasurement.angleDisplay($0)) } ?? .null
            payload[key + "_availability"] = .string(value == nil ? "unavailable" : "available")
        }
        payload.merge(fields) { _, value in value }
        let correlatedPulse = !insidePulse || name == "operation_complete" || (name.hasPrefix("stop_") && fields["stop_origin"] != .string("pulse")) ? nil : pulseIndex
        emitter.emit(.init(event: "follow_scan." + name,
            context: .init(sessionGeneration: context.request?.sessionGeneration,
                operationID: context.controllerOperationID, purpose: context.purpose?.rawValue,
                phase: context.request?.phase, pulseIndex: correlatedPulse, stale: evidence.fenced,
                outcome: outcome, reason: reason), payload: payload))
    }

    func sample(yaw: Double) -> RotationPoseDiagnosticSample {
        .init(yaw: yaw, readMonotonic: emitter.hostTime())
    }

    func evaluation(_ sample: RotationPoseDiagnosticSample, snapshot: DriveProgressWatchdog.DiagnosticSnapshot,
                    evidence: FollowMotionOperationEvidence, latch: Bool) {
        finalSample = sample
        watchdog = snapshot
        if pendingSettle {
            post = sample
            endWait("settle", evidence: evidence, latch: latch, interrupted: false)
            emit("pulse_complete", evidence: evidence, latch: latch, outcome: "completed")
            insidePulse = false
            pendingSettle = false
        }
    }

    func unavailablePost(sample: RotationPoseDiagnosticSample? = nil,
                         evidence: FollowMotionOperationEvidence, latch: Bool) {
        finalSample = sample
        post = sample
        watchdog = nil // No current error-distance sample; retain only the known checkpoint pairing.
        if pendingSettle {
            endWait("settle", evidence: evidence, latch: latch, interrupted: false)
            emit("pulse_complete", evidence: evidence, latch: latch, outcome: "completed")
            insidePulse = false
            pendingSettle = false
        }
    }

    func observed(_ snapshot: DriveProgressWatchdog.DiagnosticSnapshot,
                  previous: DriveProgressWatchdog.DiagnosticSnapshot, sample: RotationPoseDiagnosticSample) {
        watchdog = snapshot
        if checkpoint == nil || snapshot.lastProgressAt != previous.lastProgressAt || snapshot.bestDistance != previous.bestDistance {
            checkpoint = sample
            checkpointMonotonic = emitter.hostTime()
        }
    }

    func pulse(_ sample: RotationPoseDiagnosticSample, command: WheelCommand,
               evidence: FollowMotionOperationEvidence, latch: Bool) {
        pulseIndex = (pulseIndex ?? 0) + 1
        insidePulse = true
        timings = [:]
        settleEnd = nil
        pre = sample
        post = nil
        stage = "pulse_begin"
        stageStart = emitter.hostTime()
        timings["pulse_begin_monotonic_s"] = .number(stageStart!)
        emit("pulse_begin", evidence: evidence, latch: latch, fields: [
            "pulse_begin_monotonic_s": .number(stageStart!),
            "wheel_left_mps": .number(command.left), "wheel_right_mps": .number(command.right)])
    }

    func beginSend(_ command: WheelCommand, evidence: FollowMotionOperationEvidence, latch: Bool) {
        stage = "send"
        stageStart = emitter.hostTime()
        timings["send_start_monotonic_s"] = .number(stageStart!)
        emit("send_begin", evidence: evidence, latch: latch, fields: [
            "send_start_monotonic_s": .number(stageStart!), "command_kind": .string("nonzero_wheels"),
            "wheel_left_mps": .number(command.left), "wheel_right_mps": .number(command.right)])
    }

    func endSend(evidence: FollowMotionOperationEvidence, latch: Bool, outcome: String) {
        if outcome == "failed", failedStage == nil { failedStage = "send" }
        retainTiming("send", end: emitter.hostTime())
        emit("send_ack", evidence: evidence, latch: latch, outcome: outcome,
            fields: timingFields().merging(receiptFields(evidence.commandReceipt)) { _, value in value })
    }

    func beginWait(_ name: String, duration: Double, evidence: FollowMotionOperationEvidence, latch: Bool) {
        stage = name
        stageStart = emitter.hostTime()
        waitDuration = duration
        timings[name + "_start_monotonic_s"] = .number(stageStart!)
        emit(name + "_begin", evidence: evidence, latch: latch,
            fields: ["requested_wait_s": .number(duration), "start_monotonic_s": .number(stageStart!)])
    }

    func endWait(_ name: String, evidence: FollowMotionOperationEvidence, latch: Bool, interrupted: Bool) {
        let end = name == "settle" ? (settleEnd ?? emitter.hostTime()) : emitter.hostTime()
        retainTiming(name, end: end)
        var fields = timingFields(end: end)
        // Post-settle feedback may suspend and enter ack_read before evaluation.
        // Preserve the wait's own boundaries rather than that newer stage start.
        if case .number(let start) = timings[name + "_start_monotonic_s"] {
            timings[name + "_host_duration_s"] = .number(end - start)
            fields["start_monotonic_s"] = .number(start)
            fields["host_duration_s"] = .number(end - start)
        }
        fields["requested_wait_s"] = waitDuration.map { .number($0) } ?? .null
        emit(name + "_end", evidence: evidence, latch: latch,
            outcome: interrupted ? "interrupted" : "completed", fields: fields)
    }

    func settled() { settleEnd = emitter.hostTime(); pendingSettle = true }

    func beginStop(origin: String, evidence: FollowMotionOperationEvidence, latch: Bool) -> (Int, Double) {
        if !evidence.fenced && origin != "cleanup" { enter(origin == "pulse" ? "pulse_stop" : origin + "_stop") }
        stopSequence += 1
        let time = emitter.hostTime()
        emit("stop_begin", evidence: evidence, latch: latch, fields: [
            "stop_id": .number(Double(stopSequence)), "stop_origin": .string(origin),
            "start_monotonic_s": .number(time)])
        return (stopSequence, time)
    }

    func endStop(_ token: (Int, Double), origin: String, evidence: FollowMotionOperationEvidence,
                 receipt: RoverCommandDiagnosticReceipt?, latch: Bool, outcome: String) {
        emit("stop_response", evidence: evidence, latch: latch, outcome: outcome, fields: [
            "stop_id": .number(Double(token.0)), "stop_origin": .string(origin),
            "start_monotonic_s": .number(token.1), "end_monotonic_s": .number(emitter.hostTime()),
            "host_duration_s": .number(emitter.hostTime() - token.1),
            "receipt": .object(receiptFields(receipt))])
    }

    private func retainTiming(_ name: String, end: Double) {
        timings[name + "_end_monotonic_s"] = .number(end)
        timings[name + "_host_duration_s"] = stageStart.map { .number(end - $0) } ?? .null
    }

    private func timingFields(end: Double? = nil) -> [String: FollowDiagnosticValue] {
        let end = end ?? emitter.hostTime()
        return ["start_monotonic_s": stageStart.map { .number($0) } ?? .null,
         "end_monotonic_s": .number(end),
         "host_duration_s": stageStart.map { .number(end - $0) } ?? .null]
    }

    private func receiptFields(_ receipt: RoverCommandDiagnosticReceipt?) -> [String: FollowDiagnosticValue] {
        var fields: [String: FollowDiagnosticValue] = ["http_status": receipt?.httpStatus.map { .number(Double($0)) } ?? .null,
         "acknowledged": receipt?.acknowledged.map { .bool($0) } ?? .null,
         "command_ack_utc_s": receipt?.acknowledgementUTC.map { .number($0.timeIntervalSince1970) } ?? .null,
         "command_ack_age_s": receipt?.acknowledgementUTC.map { .number(emitter.utcTime().timeIntervalSince($0)) } ?? .null,
         "command_ack_clock": .string(receipt?.acknowledgementUTC == nil ? "unknown" : "transport_utc"),
         "attempts": receipt?.attempts.map { .number(Double($0)) } ?? .null]
        for key in ["http_status", "acknowledged", "command_ack_utc_s", "command_ack_age_s", "attempts"] {
            fields[key + "_availability"] = .string(fields[key] == .null ? "unknown" : "available")
        }
        return fields
    }
}
