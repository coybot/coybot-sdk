import Foundation
import RoverNav

/// A short pre-ready experiment. It never declares the ready signal complete.
/// A map disagreement may be checked with live depth; unknown depth never moves.
@MainActor
enum ReadyPathRecovery {
    enum Outcome: Equatable {
        case clear, turned, failed(NavigationFailure), cancelled
    }

    struct Environment {
        var uptime: () -> Double
        var pose: () -> NavigationPoseSample
        var depth: (WheelCommand) -> DepthSafetyObservation
        var authorized: () -> Bool
        /// The adapter must drain and confirm stop for every pulse before returning.
        var pulse: (PulseAuthority) async -> NavigationResult
        var wait: (Duration) async -> Void
        var emit: (String, [String: FollowDiagnosticValue]) -> Void
        var prepareProbe: (NavigationPoseSample) -> Bool = { _ in true }
    }

    /// Controller-owned, per-pulse permission. Reverse is confined to the fresh
    /// straight segment just probed; it does not assert rear camera coverage.
    struct PulseAuthority {
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
            guard candidate == command, now.isFinite, now < expiresAt,
                  sample.rejection(at: now, expectedGeneration: generation, requireEnriched: true) == nil,
                  let pose = sample.pose else { return false }
            let delta = pose.position - origin.position
            let along = delta.x * cos(origin.yaw) + delta.y * sin(origin.yaw)
            let lateral = abs(delta.x * sin(origin.yaw) - delta.y * cos(origin.yaw))
            let heading = abs(atan2(sin(pose.yaw - origin.yaw), cos(pose.yaw - origin.yaw)))
            guard along.isFinite, lateral.isFinite, heading.isFinite else { return false }
            switch stage {
            case .probe:
                return along >= -0.005 && along < 0.03 && lateral <= 0.01 && heading <= 0.05
            case .backtrack:
                return along > 0.005 && along <= maximumAlong + 0.005 && lateral <= 0.01 && heading <= 0.05
            case .scan:
                return delta.distance(to: .zero) <= 0.015 && heading < .pi / 2
            }
        }
    }

    static func run(start: NavigationPoseSample, environment: Environment) async -> Outcome {
        enum Abort: Error { case failed(NavigationFailure), cancelled }
        let began = environment.uptime()
        guard began.isFinite, let origin = start.pose, let generation = start.frameID?.generation,
              start.rejection(at: began, requireEnriched: true) == nil else { return .failed(.trackingLost) }
        let deadline = began + 12
        let forward = WheelCommand(left: 0.25, right: 0.25)
        var pulses = 0
        var forwardPulses = 0
        var latestStopSource: ARFrameID?
        var latestStopTime: Double?

        func active() throws {
            guard !Task.isCancelled, environment.authorized() else { throw Abort.cancelled }
            guard environment.uptime() < deadline, pulses < 32 else { throw Abort.failed(.noPath) }
        }
        func fresh() async throws -> NavigationPoseSample {
            for _ in 0..<100 {
                try active()
                let sample = environment.pose()
                guard sample.rejection(at: environment.uptime(), expectedGeneration: generation, requireEnriched: true) == nil
                else { throw Abort.failed(.trackingLost) }
                if let previous = latestStopSource, let stoppedAt = latestStopTime,
                   (sample.frameID!.sequence <= previous.sequence || sample.sourceTimestamp! <= stoppedAt) {
                    await environment.wait(.milliseconds(20))
                    continue
                }
                return sample
            }
            throw Abort.failed(.trackingLost)
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
        func geometry(_ pose: Pose2D) throws -> Double {
            let delta = pose.position - origin.position
            let along = delta.x * cos(origin.yaw) + delta.y * sin(origin.yaw)
            let lateral = abs(delta.x * sin(origin.yaw) - delta.y * cos(origin.yaw))
            let yaw = abs(atan2(sin(pose.yaw - origin.yaw), cos(pose.yaw - origin.yaw)))
            guard along.isFinite, lateral.isFinite, yaw.isFinite,
                  along >= -0.005, along <= 0.05, lateral <= 0.01, yaw <= 0.05 else {
                throw Abort.failed(.stalled)
            }
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
                environment.emit(stage, [
                    "depth_wait_elapsed_s": .number(waitStarted.map { environment.uptime() - $0 } ?? 0),
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
                _ = try geometry(try await fresh().pose!)
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
        func pulse(_ authority: PulseAuthority) async throws -> Bool {
            try active()
            guard authority.permits(authority.command, sample: environment.pose(), now: environment.uptime()) else {
                throw Abort.failed(.trackingLost)
            }
            environment.emit(authority.stage.rawValue, ["pulse": .number(Double(pulses + 1)),
                "budget_s": .number(authority.budget), "maximum_probe_m": .number(0.05)])
            let result = await environment.pulse(authority)
            pulses += 1
            try active()
            switch result {
            case .arrived, .reobservationRequired: break
            case .cancelled: throw Abort.cancelled
            case .failed(let failure): throw Abort.failed(failure)
            }
            latestStopSource = environment.pose().frameID
            latestStopTime = environment.uptime()
            await environment.wait(.milliseconds(150))
            return result == .arrived
        }
        do {
            var travelled = 0.0
            let probeDeadline = min(deadline, began + 3)
            while true {
                let sample = try await fresh()
                travelled = try geometry(sample.pose!)
                let observation: DepthSafetyObservation
                if pulses == 0 {
                    observation = try await awaitInitialProbeReadiness()
                    // Waiting may span several captures. Validate the new pose,
                    // never use the pre-wait geometry as movement authority.
                    travelled = try geometry(try await fresh().pose!)
                } else { observation = try depth() }
                if observation.state != .clear { break }
                if travelled >= 0.03 {
                    guard forwardPulses > 0 else { throw Abort.failed(.stalled) }
                    environment.emit("clear", ["probe_travel_m": .number(travelled)])
                    return .clear
                }
                guard environment.uptime() < probeDeadline else { throw Abort.failed(.stalled) }
                let entered = try await pulse(.init(stage: .probe, origin: origin, generation: generation,
                    maximumAlong: 0.05, expiresAt: probeDeadline, command: forward))
                if !entered {
                    travelled = try geometry(try await fresh().pose!)
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
                    command: .init(left: -0.25, right: -0.25)))
                let previous = travelled
                travelled = try geometry(try await fresh().pose!)
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
