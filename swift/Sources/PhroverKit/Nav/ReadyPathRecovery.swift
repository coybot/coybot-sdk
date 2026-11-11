import Foundation
import RoverNav

/// A short pre-ready experiment. It never declares the ready signal complete.
/// A map disagreement may be checked with live depth; unknown depth never moves.
@MainActor
enum ReadyPathRecovery {
    enum Outcome: Equatable {
        case clear, turned, failed(NavigationFailure), cancelled
    }

    enum PulseResult: Equatable {
        case arrived, reobservationRequired, recheck, cancelled, failed(NavigationFailure)
    }

    struct Environment {
        var uptime: () -> Double
        var pose: () -> NavigationPoseSample
        var depth: (WheelCommand) -> DepthSafetyObservation
        var authorized: () -> Bool
        /// The adapter must drain and confirm stop for every pulse before returning.
        var pulse: (PulseAuthority) async -> PulseResult
        var wait: (Duration) async -> Void
        var emit: (String, [String: FollowDiagnosticValue]) -> Void
        var prepareProbe: (NavigationPoseSample) -> Bool = { _ in true }
        var rejected: (PulseAuthority.Rejection) -> Void = { _ in }
    }

    /// Controller-owned, per-pulse permission. Reverse is confined to the fresh
    /// straight segment just probed; it does not assert rear camera coverage.
    struct PulseAuthority {
        enum Decision: Equatable {
            case permitted, goalReached, rejected(Rejection)
        }
        enum Rejection: Equatable {
            case source(String), commandMismatch, invalidClock, expired, invalidGeometry, unstablePose, unsettledMotion, pulseResponseLimit
            case backwardLimit, travelLimit, lateralLimit, headingLimit, scanTranslationLimit, scanTurnLimit, unearnedProgress

            var code: String {
                switch self {
                case .source(let reason): return reason
                case .commandMismatch: return "command_mismatch"
                case .invalidClock: return "invalid_clock"
                case .expired: return "authorization_expired"
                case .invalidGeometry: return "invalid_geometry"
                case .unstablePose: return "unstable_pose"
                case .unsettledMotion: return "motion_not_settled"
                case .pulseResponseLimit: return "pulse_response_exceeds_remaining_travel"
                case .backwardLimit: return "backward_limit"
                case .travelLimit: return "travel_limit"
                case .lateralLimit: return "lateral_limit"
                case .headingLimit: return "heading_limit"
                case .scanTranslationLimit: return "scan_translation_limit"
                case .scanTurnLimit: return "scan_turn_limit"
                case .unearnedProgress: return "progress_without_probe"
                }
            }
            var failure: NavigationFailure {
                if case .source = self { return .trackingLost }
                return .stalled
            }
            var cause: FollowMotionFailureCause {
                switch self {
                case .source("stale_source"): return .poseSourceStale
                case .source: return .readyProbeSourceInvalid
                case .expired: return .readyProbeExpired
                case .backwardLimit: return .readyProbeBackwardLimit
                case .travelLimit: return .readyProbeTravelLimit
                case .lateralLimit: return .readyProbeLateralDeviation
                case .headingLimit, .scanTurnLimit: return .readyProbeHeadingDeviation
                case .scanTranslationLimit: return .readyProbeScanTranslation
                case .unearnedProgress: return .readyPreSendPoseShift
                case .unstablePose: return .readyProbePoseUnstable
                case .unsettledMotion: return .readyProbeSettleTimeout
                case .pulseResponseLimit: return .readyProbeResponseLimit
                case .commandMismatch, .invalidClock, .invalidGeometry: return .readyProbeInvalidGeometry
                }
            }
        }
        enum Stage: String { case probe, backtrack, scan }
        let id = UUID()
        let stage: Stage
        let origin: Pose2D
        let generation: UInt64
        let maximumAlong: Double
        let expiresAt: Double
        let command: WheelCommand
        let budget: Double = 0.020

        func permits(_ candidate: WheelCommand, sample: NavigationPoseSample, now: Double) -> Bool {
            assessment(candidate, sample: sample, now: now) == .permitted
        }

        func assessment(_ candidate: WheelCommand, sample: NavigationPoseSample, now: Double) -> Decision {
            guard candidate == command else { return .rejected(.commandMismatch) }
            guard now.isFinite, expiresAt.isFinite else { return .rejected(.invalidClock) }
            guard now < expiresAt else { return .rejected(.expired) }
            if let reason = sample.rejection(at: now, expectedGeneration: generation, requireEnriched: true) {
                return .rejected(.source(reason))
            }
            guard let pose = sample.pose else { return .rejected(.source("missing_pose")) }
            let delta = pose.position - origin.position
            let along = delta.x * cos(origin.yaw) + delta.y * sin(origin.yaw)
            let lateral = abs(delta.x * sin(origin.yaw) - delta.y * cos(origin.yaw))
            let heading = abs(atan2(sin(pose.yaw - origin.yaw), cos(pose.yaw - origin.yaw)))
            guard along.isFinite, lateral.isFinite, heading.isFinite, maximumAlong.isFinite else { return .rejected(.invalidGeometry) }
            if stage != .scan {
                guard along >= -0.005 else { return .rejected(.backwardLimit) }
                guard along <= (stage == .probe ? 0.05 : maximumAlong + 0.005) else { return .rejected(.travelLimit) }
                guard lateral <= 0.01 else { return .rejected(.lateralLimit) }
                guard heading <= 0.05 else { return .rejected(.headingLimit) }
            }
            switch stage {
            case .probe:
                return along >= 0.03 ? .goalReached : .permitted
            case .backtrack:
                return along <= 0.005 ? .goalReached : .permitted
            case .scan:
                guard delta.distance(to: .zero) <= 0.015 else { return .rejected(.scanTranslationLimit) }
                guard heading < .pi / 2 else { return .rejected(.scanTurnLimit) }
                return .permitted
            }
        }

        func diagnosticFields(sample: NavigationPoseSample, now: Double, decision: Decision) -> [String: FollowDiagnosticValue] {
            func number(_ value: Double?) -> FollowDiagnosticValue { value.flatMap { $0.isFinite ? .number($0) : nil } ?? .null }
            let delta = sample.pose.map { $0.position - origin.position }
            let along = delta.map { $0.x * cos(origin.yaw) + $0.y * sin(origin.yaw) }
            let lateral = delta.map { -$0.x * sin(origin.yaw) + $0.y * cos(origin.yaw) }
            let heading = sample.pose.map { atan2(sin($0.yaw - origin.yaw), cos($0.yaw - origin.yaw)) }
            let reason: String
            switch decision {
            case .permitted: reason = "permitted"
            case .goalReached: reason = "goal_reached"
            case .rejected(let rejection): reason = rejection.code
            }
            return ["ready_recovery_decision": .string(reason), "ready_recovery_stage": .string(stage.rawValue),
                "ready_recovery_authority_id": .string(id.uuidString),
                "ready_recovery_expected_generation": .number(Double(generation)),
                "ready_recovery_maximum_along_m": number(maximumAlong),
                "ready_recovery_frame_id": sample.frameID.map { .string("\($0.generation):\($0.sequence)") } ?? .null,
                "ready_recovery_source_timestamp_s": number(sample.sourceTimestamp),
                "ready_recovery_read_uptime_s": number(now), "ready_recovery_source_age_s": number(sample.sourceTimestamp.map { now - $0 }),
                "ready_recovery_tracking": sample.trackingQuality.map { .string(String(describing: $0)) } ?? .null,
                "ready_recovery_origin_x": number(origin.position.x), "ready_recovery_origin_z": number(origin.position.y),
                "ready_recovery_origin_yaw_rad": number(origin.yaw), "ready_recovery_pose_x": number(sample.pose?.position.x),
                "ready_recovery_pose_z": number(sample.pose?.position.y), "ready_recovery_pose_yaw_rad": number(sample.pose?.yaw),
                "ready_recovery_along_m": number(along), "ready_recovery_lateral_m": number(lateral),
                "ready_recovery_backward_limit_exceeded": along.map { .bool($0 < -0.005) } ?? .null,
                "ready_recovery_lateral_limit_exceeded": lateral.map { .bool(abs($0) > 0.01) } ?? .null,
                "ready_recovery_heading_change_rad": number(heading), "ready_recovery_deadline_s": number(expiresAt),
                "ready_probe_goal_m": .number(0.03), "ready_probe_maximum_m": .number(0.05),
                "ready_probe_lateral_limit_m": .number(0.01), "ready_probe_heading_limit_rad": .number(0.05)]
        }
    }

    static func run(start: NavigationPoseSample, environment: Environment) async -> Outcome {
        enum Abort: Error { case failed(NavigationFailure), cancelled }
        let began = environment.uptime()
        guard began.isFinite, let initialOrigin = start.pose, let generation = start.frameID?.generation,
               start.rejection(at: began, requireEnriched: true) == nil else { return .failed(.trackingLost) }
        var origin = initialOrigin
        let deadline = began + 12
        let forward = WheelCommand(left: 0.12, right: 0.12)
        var maximumForwardPulseTravel = 0.0
        var pulses = 0
        var forwardPulses = 0
        var latestStopSource: ARFrameID?
        var latestStopTime: Double?

        var envelope = PulseAuthority(stage: .probe, origin: origin, generation: generation,
            maximumAlong: 0.05, expiresAt: deadline, command: forward)
        func reject(_ reason: PulseAuthority.Rejection, sample: NavigationPoseSample, boundary: String,
                    authority: PulseAuthority? = nil) -> Abort {
            var fields = (authority ?? envelope).diagnosticFields(sample: sample, now: environment.uptime(), decision: .rejected(reason))
            fields["ready_recovery_boundary"] = .string(boundary)
            environment.emit("pulse_rejected", fields)
            environment.rejected(reason)
            return .failed(reason.failure)
        }

        func active() throws {
            guard !Task.isCancelled, environment.authorized() else { throw Abort.cancelled }
            guard environment.uptime() < deadline, pulses < 32 else { throw Abort.failed(.noPath) }
        }
        func fresh() async throws -> NavigationPoseSample {
            for _ in 0..<100 {
                try active()
                let sample = environment.pose()
                if let reason = sample.rejection(at: environment.uptime(), expectedGeneration: generation, requireEnriched: true) {
                    throw reject(.source(reason), sample: sample, boundary: "source_wait")
                }
                if let previous = latestStopSource, let stoppedAt = latestStopTime,
                   (sample.frameID!.sequence <= previous.sequence || sample.sourceTimestamp! <= stoppedAt) {
                    await environment.wait(.milliseconds(20))
                    continue
                }
                return sample
            }
            throw reject(.source("post_stop_source_not_advanced"), sample: environment.pose(), boundary: "source_wait")
        }
        func depth() throws -> DepthSafetyObservation {
            try active()
            let value = environment.depth(forward)
            try active()
            guard !value.clearance.isNaN else { throw Abort.failed(.depthUnavailable(.malformedDepth)) }
            if case .unavailable(let reason) = value.state { throw Abort.failed(.depthUnavailable(reason)) }
            guard value.sampleAge.isFinite, value.sampleAge >= 0,
                  value.sampleAge <= DepthSafetyEvaluator.maximumRawDepthAge else {
                throw Abort.failed(.depthUnavailable(.staleRawDepth))
            }
            return value
        }
        // Only called while entry or the previous pulse is confirmed stopped.
        // Distinct identities AND capture times are required; polling a cached
        // frame cannot provide independent corroboration.
        func stoppedSamples(after first: NavigationPoseSample, afterTime: Double,
                            until limit: Double) async throws -> [NavigationPoseSample] {
            var samples: [NavigationPoseSample] = []
            var previous = first
            for _ in 0..<30 {
                try active()
                guard environment.uptime() < limit else { break }
                await environment.wait(.milliseconds(20))
                try active()
                guard environment.uptime() < limit else { break }
                let sample = environment.pose()
                if let reason = sample.rejection(at: environment.uptime(), expectedGeneration: generation, requireEnriched: true) {
                    throw reject(.source(reason), sample: sample, boundary: "stopped_pose_samples")
                }
                guard sample.frameID!.sequence > previous.frameID!.sequence,
                      sample.sourceTimestamp! > max(afterTime, previous.sourceTimestamp!) else { continue }
                if let last = samples.last, sample.sourceTimestamp! - last.sourceTimestamp! < 0.05 { continue }
                samples.append(sample)
                previous = sample
                if samples.count == 3 { return samples }
            }
            environment.emit("stopped_pose_timeout", ["sample_count": .number(Double(samples.count)),
                "stopped_pose_deadline_s": .number(limit), "evaluated_at_s": .number(environment.uptime())])
            throw reject(environment.uptime() >= began + 3 ? .expired : .unstablePose,
                sample: environment.pose(), boundary: "stopped_pose_timeout")
        }

        func establishBaseline() async throws {
            let samples = try await stoppedSamples(after: start, afterTime: began,
                until: min(began + 0.3, began + FollowReadyAdmission.recoveryRefreshLimit))
            let poses = samples.map { $0.pose! }
            func median(_ values: [Double]) -> Double { values.sorted()[values.count / 2] }
            let position = Vec2(median(poses.map { $0.position.x }), median(poses.map { $0.position.y }))
            let yaw = initialOrigin.yaw + median(poses.map {
                atan2(sin($0.yaw - initialOrigin.yaw), cos($0.yaw - initialOrigin.yaw))
            })
            let spread = poses.map { $0.position.distance(to: position) }.max()!
            let yawSpread = poses.map { abs(atan2(sin($0.yaw - yaw), cos($0.yaw - yaw))) }.max()!
            let stable = spread <= 0.004 && yawSpread <= 0.01 &&
                position.distance(to: initialOrigin.position) <= 0.005 &&
                abs(atan2(sin(yaw - initialOrigin.yaw), cos(yaw - initialOrigin.yaw))) <= 0.01
            environment.emit(stable ? "baseline_accepted" : "baseline_unstable", [
                "baseline_sample_count": .number(Double(samples.count)),
                "baseline_capture_span_s": .number(samples.last!.sourceTimestamp! - samples.first!.sourceTimestamp!),
                "baseline_deadline_s": .number(began + 0.3),
                "baseline_max_deviation_m": .number(spread), "baseline_max_heading_deviation_rad": .number(yawSpread),
                "baseline_position_limit_m": .number(0.004), "baseline_heading_limit_rad": .number(0.01),
                "baseline_x": .number(position.x), "baseline_z": .number(position.y), "baseline_yaw_rad": .number(yaw)])
            guard stable else { throw reject(.unstablePose, sample: samples.last!, boundary: "baseline") }
            origin = Pose2D(position: position, yaw: yaw)
            envelope = PulseAuthority(stage: .probe, origin: origin, generation: generation,
                maximumAlong: 0.05, expiresAt: deadline, command: forward)
        }
        func geometry(_ sample: NavigationPoseSample, stageDeadline: Double? = nil) async throws -> Double {
            var evaluated = sample
            let decision = envelope.assessment(forward, sample: evaluated, now: environment.uptime())
            if case .rejected(let reason) = decision,
               reason == .backwardLimit || reason == .lateralLimit {
                environment.emit("ready_recovery_pose_recheck", envelope.diagnosticFields(
                    sample: evaluated, now: environment.uptime(), decision: decision).merging([
                        "ready_recovery_boundary": .string("loop_geometry"),
                        "ready_recovery_recheck_reason": .string(reason.code)
                    ]) { _, new in new })
                let samples = try await stoppedSamples(after: sample,
                    afterTime: max(latestStopTime ?? began, sample.sourceTimestamp!),
                    until: min(environment.uptime() + 0.3, stageDeadline ?? began + 3))
                for confirmedSample in samples {
                    let assessment = envelope.assessment(forward, sample: confirmedSample, now: environment.uptime())
                    if case .rejected(let confirmed) = assessment {
                        throw reject(confirmed, sample: confirmedSample, boundary: "loop_geometry_recheck")
                    }
                }
                evaluated = samples.last!
                environment.emit("pose_recheck_recovered", ["sample_count": .number(Double(samples.count))])
            } else if case .rejected(let reason) = decision {
                throw reject(reason, sample: evaluated, boundary: "loop_geometry")
            }
            guard let pose = evaluated.pose else { throw reject(.source("missing_pose"), sample: evaluated, boundary: "loop_geometry") }
            let delta = pose.position - origin.position
            let along = delta.x * cos(origin.yaw) + delta.y * sin(origin.yaw)
            return along
        }
        /// Entry is confirmed stopped. Transient startup depth/perception age
        /// shares one fixed refresh window before any pulse. Original motion
        /// and recovery deadlines are retained.
        func awaitInitialProbeReadiness() async throws -> DepthSafetyObservation {
            var waitStarted: Double?
            let refreshDeadline = min(deadline, began + 3, began + FollowReadyAdmission.recoveryRefreshLimit)
            var polls = 0
            var lastReason = DepthSafetyUnavailableReason.staleRawDepth
            var waitingForPerception = false
            func emitWait(_ stage: String) {
                let time = environment.uptime()
                environment.emit(stage, [
                    "depth_wait_elapsed_s": .number(waitStarted.map { time - $0 } ?? 0),
                    "depth_wait_deadline_s": .number(refreshDeadline),
                    "depth_wait_limit_s": .number(FollowReadyAdmission.recoveryRefreshLimit), "depth_wait_poll_count": .number(Double(polls)),
                    "waiting_for_perception": .bool(waitingForPerception),
                    "depth_wait_reason": .string(lastReason.rawValue),
                    "ready_probe_deadline_s": .number(min(deadline, began + 3)),
                    "ready_recovery_deadline_s": .number(deadline)])
            }
            func checkDeadline() throws {
                if environment.uptime() >= refreshDeadline {
                    emitWait(waitingForPerception ? "perception_wait_expired" : "depth_wait_expired")
                    if waitingForPerception { throw Abort.cancelled }
                    throw Abort.failed(.depthUnavailable(lastReason))
                }
            }
            while true {
                try active()
                try checkDeadline()
                _ = try await geometry(try await fresh())
                try checkDeadline()
                let observation: DepthSafetyObservation
                do { observation = try depth() }
                catch Abort.failed(.depthUnavailable(let reason)) where reason == .staleRawDepth || reason == .missingRawDepth {
                    lastReason = reason
                    if waitStarted == nil {
                        let time = environment.uptime()
                        waitStarted = time
                        emitWait("depth_wait_started")
                    }
                    try active()
                    try checkDeadline()
                    guard polls < 12 else {
                        emitWait("depth_wait_expired")
                        throw Abort.failed(.depthUnavailable(reason))
                    }
                    polls += 1
                    await environment.wait(.seconds(min(0.05, max(0, refreshDeadline - environment.uptime()))))
                    continue
                }
                try active()
                try checkDeadline()
                waitingForPerception = !environment.prepareProbe(try await fresh())
                try active()
                if waitingForPerception {
                    if waitStarted == nil {
                        waitStarted = environment.uptime()
                        emitWait("perception_wait_started")
                    }
                    try checkDeadline()
                    guard polls < 12 else { throw Abort.cancelled }
                    polls += 1
                    await environment.wait(.seconds(min(0.05, max(0, refreshDeadline - environment.uptime()))))
                    continue
                }
                try checkDeadline()
                if waitStarted != nil { emitWait("depth_wait_recovered") }
                return observation
            }
        }
        func settle(_ authority: PulseAuthority, before: NavigationPoseSample) async throws {
            let stoppedAt = latestStopTime!
            let limit = min(authority.expiresAt, stoppedAt + 0.6)
            var samples: [NavigationPoseSample] = []
            var lastFrame = latestStopSource!
            var lastTimestamp = stoppedAt
            var peakTravel = 0.0
            for _ in 0..<30 {
                try active()
                guard environment.uptime() < limit else { break }
                await environment.wait(.milliseconds(20))
                try active()
                guard environment.uptime() < limit else { break }
                let sample = environment.pose(), time = environment.uptime()
                if let reason = sample.rejection(at: time, expectedGeneration: generation, requireEnriched: true) {
                    throw reject(.source(reason), sample: sample, boundary: "pulse_settle", authority: authority)
                }
                // Travel and heading limits remain hard even while stopped.
                peakTravel = max(peakTravel, sample.pose!.position.distance(to: before.pose!.position))
                if case .rejected(let reason) = authority.assessment(authority.command, sample: sample, now: time),
                   reason != .backwardLimit && reason != .lateralLimit {
                    throw reject(reason, sample: sample, boundary: "pulse_settle", authority: authority)
                }
                guard sample.frameID!.sequence > lastFrame.sequence, sample.sourceTimestamp! > lastTimestamp else { continue }
                if let last = samples.last, sample.sourceTimestamp! - last.sourceTimestamp! < 0.05 { continue }
                lastFrame = sample.frameID!; lastTimestamp = sample.sourceTimestamp!
                samples.append(sample)
                if samples.count > 3 { samples.removeFirst() }
                let current = sample.pose!
                let spread = samples.map { $0.pose!.position.distance(to: current.position) }.max()!
                let yawSpread = samples.map { abs(atan2(sin($0.pose!.yaw - current.yaw), cos($0.pose!.yaw - current.yaw))) }.max()!
                var fields = authority.diagnosticFields(sample: sample, now: time,
                    decision: authority.assessment(authority.command, sample: sample, now: time))
                fields["settle_elapsed_s"] = .number(time - stoppedAt)
                fields["settle_deadline_s"] = .number(limit)
                fields["settle_position_spread_m"] = .number(spread)
                fields["settle_heading_spread_rad"] = .number(yawSpread)
                fields["settle_sample_count"] = .number(Double(samples.count))
                environment.emit("pulse_settle_sample", fields)
                guard samples.count == 3, time - stoppedAt >= 0.15,
                      spread <= 0.002, yawSpread <= 0.005 else { continue }
                let travel = current.position.distance(to: before.pose!.position)
                if authority.stage == .probe { maximumForwardPulseTravel = max(maximumForwardPulseTravel, peakTravel) }
                fields["observed_pulse_travel_m"] = .number(travel)
                fields["observed_pulse_peak_travel_m"] = .number(peakTravel)
                fields["maximum_forward_pulse_travel_m"] = .number(maximumForwardPulseTravel)
                environment.emit("pulse_settled", fields)
                return
            }
            throw reject(environment.uptime() >= authority.expiresAt ? .expired : .unsettledMotion,
                sample: environment.pose(), boundary: "pulse_settle_timeout", authority: authority)
        }

        func pulse(_ authority: PulseAuthority) async throws -> PulseResult {
            try active()
            let sample = environment.pose(), time = environment.uptime()
            let decision = authority.assessment(authority.command, sample: sample, now: time)
            let fields = authority.diagnosticFields(sample: sample, now: time, decision: decision)
            switch decision {
            case .permitted: break
            case .goalReached:
                environment.emit("pulse_recheck", fields)
                return .recheck
            case .rejected(let reason):
                environment.emit("pulse_rejected", fields)
                environment.rejected(reason)
                throw Abort.failed(reason.failure)
            }
            environment.emit(authority.stage.rawValue, ["pulse": .number(Double(pulses + 1)),
                "budget_s": .number(authority.budget), "maximum_probe_m": .number(0.05)].merging(fields) { _, new in new })
            let result = await environment.pulse(authority)
            try active()
            switch result {
            case .arrived, .reobservationRequired: break
            case .recheck:
                environment.emit("pulse_recheck", ["completed_pulses": .number(Double(pulses))])
                await environment.wait(.milliseconds(20))
                return .recheck
            case .cancelled: throw Abort.cancelled
            case .failed(let failure): throw Abort.failed(failure)
            }
            pulses += 1
            latestStopSource = environment.pose().frameID
            latestStopTime = environment.uptime()
            if authority.stage == .scan { await environment.wait(.milliseconds(150)) }
            else { try await settle(authority, before: sample) }
            return result
        }
        do {
            try await establishBaseline()
            var travelled = 0.0
            let probeDeadline = min(deadline, began + 3)
            let probeEnvelope = PulseAuthority(stage: .probe, origin: origin, generation: generation,
                maximumAlong: 0.05, expiresAt: probeDeadline, command: forward)
            while true {
                guard environment.uptime() < probeDeadline else {
                    throw reject(.expired, sample: environment.pose(), boundary: "probe_deadline", authority: probeEnvelope)
                }
                let sample = try await fresh()
                travelled = try await geometry(sample)
                let observation: DepthSafetyObservation
                if pulses == 0 {
                    observation = try await awaitInitialProbeReadiness()
                    // Waiting may span several captures. Validate the new pose,
                    // never use the pre-wait geometry as movement authority.
                    travelled = try await geometry(try await fresh())
                } else { observation = try depth() }
                if observation.state != .clear { break }
                guard environment.uptime() < probeDeadline else {
                    throw reject(.expired, sample: environment.pose(), boundary: "probe_deadline", authority: probeEnvelope)
                }
                if travelled >= 0.03 {
                    guard forwardPulses > 0 else {
                        throw reject(.unearnedProgress, sample: environment.pose(), boundary: "probe_completion", authority: probeEnvelope)
                    }
                    environment.emit("clear", ["probe_travel_m": .number(travelled)])
                    return .clear
                }
                guard environment.uptime() < probeDeadline else {
                    throw reject(.expired, sample: environment.pose(), boundary: "probe_deadline", authority: probeEnvelope)
                }
                if forwardPulses > 0 {
                    let allowance = 0.05 - travelled
                    let predicted = maximumForwardPulseTravel * 1.5 + 0.005
                    environment.emit(predicted > allowance ? "pulse_response_unfit" : "pulse_response_fits", [
                        "observed_maximum_pulse_travel_m": .number(maximumForwardPulseTravel),
                        "reserved_pulse_travel_m": .number(predicted), "remaining_travel_m": .number(allowance)])
                    guard predicted <= allowance else {
                        throw reject(.pulseResponseLimit, sample: environment.pose(), boundary: "next_probe_budget", authority: probeEnvelope)
                    }
                }
                let entered = try await pulse(.init(stage: .probe, origin: origin, generation: generation,
                    maximumAlong: 0.05, expiresAt: probeDeadline, command: forward))
                if entered == .recheck { continue }
                if entered == .reobservationRequired {
                    travelled = try await geometry(try await fresh())
                    break
                }
                forwardPulses += 1
            }
            // The return grant exists only after a real, depth-checked forward
            // segment. It expires rather than renewing for every reverse pulse.
            let returnDeadline = min(deadline, environment.uptime() + 3)
            guard travelled <= 0.005 || forwardPulses > 0 else { throw Abort.failed(.stalled) }
            while travelled > 0.005 {
                _ = try depth() // Unknown live sensing stops the whole recovery.
                _ = try await pulse(.init(stage: .backtrack, origin: origin, generation: generation,
                    maximumAlong: travelled, expiresAt: returnDeadline,
                    command: .init(left: -0.12, right: -0.12)))
                let previous = travelled
                travelled = try await geometry(try await fresh(), stageDeadline: returnDeadline)
                guard travelled < previous else { throw Abort.failed(.stalled) }
            }
            let scanOrigin = try await fresh().pose!
            var previousDepthCapture: Double?
            while true {
                let sample = try await fresh()
                guard sample.pose!.position.distance(to: scanOrigin.position) <= 0.015 else {
                    throw Abort.failed(.stalled)
                }
                let angle = atan2(sin(sample.pose!.yaw - scanOrigin.yaw), cos(sample.pose!.yaw - scanOrigin.yaw))
                guard angle >= -0.05, angle <= .pi / 2 else { throw Abort.failed(.noPath) }
                let observation = try depth()
                let capture = environment.uptime() - observation.sampleAge
                if observation.state == .clear {
                    if let previousDepthCapture, capture > previousDepthCapture {
                        environment.emit("clear_heading", [:])
                        return .turned
                    }
                    previousDepthCapture = capture
                    await environment.wait(.milliseconds(150))
                    continue
                }
                previousDepthCapture = nil
                guard angle < .pi / 2 else { throw Abort.failed(.noPath) }
                _ = try await pulse(.init(stage: .scan, origin: scanOrigin, generation: generation,
                    maximumAlong: 0, expiresAt: deadline, command: .init(left: -0.25, right: 0.25)))
            }
        } catch Abort.cancelled { return .cancelled }
        catch Abort.failed(let failure) { return .failed(failure) }
        catch { return .failed(.commandFailed) }
    }
}
