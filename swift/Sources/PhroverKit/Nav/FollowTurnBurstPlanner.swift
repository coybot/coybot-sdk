import Foundation

/// Pure, operation-local host-request planning. Angles are radians; host/source
/// durations are seconds. Neither observed rates nor sampled travel certify
/// physical speed, total coast, or motor-on duration. No motor or stop authority.
public enum FollowTurnBurstPlanner {
    public enum Purpose: Sendable { case alignment, scan }
    public struct Profile: Sendable, Equatable {
        public let maximumHostBurstBudget: Double = 0.080
        public let initialReferenceRate: Double = 2 * .pi / 3
        public let settleWait: Double
        public let fixedWheelMagnitude: Double = 0.25
        public let tolerance: Double
        public init(purpose: Purpose) { self.init(purpose: purpose, fastSearch: false) }
        init(purpose: Purpose, fastSearch: Bool) {
            tolerance = purpose == .alignment ? 0.05 : 7 * .pi / 180
            settleWait = fastSearch ? 0.100 : 0.300
        }
    }
    public struct Calibration: Sendable, Equatable {
        public struct ObservedBurst: Sendable, Equatable {
            public let budget: Double
            public let signedTravel: Double
            public let absoluteTravel: Double
        }
        public let operationID: UInt64
        public let generation: UInt64
        public let targetYaw: Double
        public let clockDomain: String
        /// Conservative end-to-end AR travel per second of command/stop window,
        /// not instantaneous angular velocity or a physical speed guarantee.
        public let responseRate: Double
        /// Observed window beyond the requested budget. Kept separately from gain
        /// so shrinking a request cannot amplify fixed transport/stop overhead.
        public let responseOverhead: Double
        public let lastObservedBurst: ObservedBurst?
        /// Diagnostic durations; these overlap and must not be summed for planning.
        public let maximumSendDuration: Double?
        public let maximumStopDuration: Double?
        /// Partial sampled travel after acknowledgement, not total physical coast.
        public let observedPostAckTravel: Double?
        public let overshootCeiling: Double?
        public let completedResponses: Int
        public let measuredResponses: Int
        public let terminalResolutionFailure: Bool
        public let lastStopAcknowledgementUptime: Double?
        public let lastSourceSequence: UInt64?
        public let lastSourceTimestamp: Double?
        public let lastSourceSample: Sample?
        public init(operationID: UInt64, generation: UInt64, targetYaw: Double, clockDomain: String,
                    responseRateFloor: Double = 2 * .pi / 3) {
            self.operationID = operationID
            self.generation = generation
            self.targetYaw = targetYaw
            self.clockDomain = clockDomain
            responseRate = responseRateFloor.isFinite ? max(2 * .pi / 3, responseRateFloor) : .infinity
            responseOverhead = 0
            lastObservedBurst = nil
            maximumSendDuration = nil
            maximumStopDuration = nil
            observedPostAckTravel = nil
            overshootCeiling = nil
            completedResponses = 0
            measuredResponses = 0
            terminalResolutionFailure = false
            lastStopAcknowledgementUptime = nil
            lastSourceSequence = nil
            lastSourceTimestamp = nil
            lastSourceSample = nil
        }
        fileprivate init(previous: Self, rate: Double, overhead: Double, send: Double, stop: Double,
                          travel: Double?, ceiling: Double? = nil, terminal: Bool = false,
                          measured: Bool, response: Response, observedBurst: ObservedBurst?) {
            operationID = previous.operationID
            generation = previous.generation
            targetYaw = previous.targetYaw
            clockDomain = previous.clockDomain
            responseRate = rate
            responseOverhead = overhead
            lastObservedBurst = observedBurst
            maximumSendDuration = send
            maximumStopDuration = stop
            observedPostAckTravel = travel
            overshootCeiling = ceiling
            completedResponses = previous.completedResponses + 1
            measuredResponses = previous.measuredResponses + (measured ? 1 : 0)
            terminalResolutionFailure = previous.terminalResolutionFailure || terminal
            lastStopAcknowledgementUptime = response.stopAcknowledgementUptime
            lastSourceSequence = response.samples.last?.sequence ?? previous.lastSourceSequence
            lastSourceTimestamp = response.samples.last?.sourceTimestamp ?? previous.lastSourceTimestamp
            lastSourceSample = response.samples.last ?? previous.lastSourceSample
        }
    }
    public struct Input: Sendable {
        public let actualYaw: Double
        public let profile: Profile
        public let calibration: Calibration
        public let sendEntryUptime: Double
        /// Integration attests current stopped/fresh source and operation authority.
        public let authorized: Bool
        /// Integration records sender entry even if no valid response can be reduced.
        /// Unknown response permits one initial probe, never a calibration retry.
        public let provisionalProbeIssued: Bool
        public init(actualYaw: Double, profile: Profile, calibration: Calibration,
                    sendEntryUptime: Double, authorized: Bool = true, provisionalProbeIssued: Bool = false) {
            self.actualYaw = actualYaw
            self.profile = profile
            self.calibration = calibration
            self.sendEntryUptime = sendEntryUptime
            self.authorized = authorized
            self.provisionalProbeIssued = provisionalProbeIssued
        }
    }
    public enum Decision: Sendable, Equatable {
        case arrived
        case burst(direction: Int, budget: Double)
        case resolutionFailure(NavigationFailure)
        case unavailable
    }
    /// Collection uptime and source time must share the explicitly named clock domain.
    /// Missing provenance represents unknown/legacy evidence, never manufactured freshness.
    public struct Sample: Sendable, Equatable {
        public let yaw: Double
        public let sequence: UInt64?
        public let generation: UInt64?
        public let sourceTimestamp: Double?
        public let collectedUptime: Double
        public let clockDomain: String?
        /// Health at `collectedUptime`: ingress health for `Response.samples`,
        /// or read-time health for the separate control-evaluation properties.
        /// A later read never replaces an ingress witness's health or timestamp.
        public let healthy: Bool
        public let sourceIdentity: String?
        public let trackingState: String?
        public init(yaw: Double, sequence: UInt64?, generation: UInt64?, sourceTimestamp: Double?,
                    collectedUptime: Double, clockDomain: String?, healthy: Bool) {
            self.init(yaw: yaw, sequence: sequence, generation: generation, sourceTimestamp: sourceTimestamp,
                collectedUptime: collectedUptime, clockDomain: clockDomain, healthy: healthy,
                sourceIdentity: nil, trackingState: nil)
        }
        public init(yaw: Double, sequence: UInt64?, generation: UInt64?, sourceTimestamp: Double?,
                    collectedUptime: Double, clockDomain: String?, healthy: Bool,
                    sourceIdentity: String?, trackingState: String?) {
            self.yaw = yaw
            self.sequence = sequence
            self.generation = generation
            self.sourceTimestamp = sourceTimestamp
            self.collectedUptime = collectedUptime
            self.clockDomain = clockDomain
            self.healthy = healthy
            self.sourceIdentity = sourceIdentity
            self.trackingState = trackingState
        }
        /// Diagnostic labels do not change the planner's exact-boundary equality.
        public static func == (lhs: Self, rhs: Self) -> Bool {
            lhs.yaw == rhs.yaw && lhs.sequence == rhs.sequence && lhs.generation == rhs.generation &&
                lhs.sourceTimestamp == rhs.sourceTimestamp && lhs.collectedUptime == rhs.collectedUptime &&
                lhs.clockDomain == rhs.clockDomain && lhs.healthy == rhs.healthy
        }
    }
    public enum Completion: Sendable { case completed, cancelled, failed, replaced, ambiguous }
    public struct Response: Sendable {
        public let operationID: UInt64
        public let generation: UInt64
        public let targetYaw: Double
        public let clockDomain: String
        public let completion: Completion
        public let requestedBudget: Double
        public let sendEntryUptime: Double
        public let sendResponseUptime: Double
        public let stopObligationUptime: Double
        public let stopAcknowledgementUptime: Double
        /// Ordered source evidence; endpoints may be missing. Missing is unknown,
        /// while supplied invalid/unhealthy evidence rejects the entire reduction.
        public let samples: [Sample]
        /// Caller attests intervals are unambiguous; shortest deltas cannot prove missed full turns.
        public let traversalUnambiguous: Bool
        /// Separate planning read. Its `collectedUptime` is evaluation time and
        /// `healthy` is read-time health; it does not replace ingress facts.
        public let planningEvaluation: Sample?
        /// Separate stopped read with evaluation-time timestamp and health.
        /// Ingress collection times remain unchanged in `samples`.
        public let stoppedEvaluation: Sample?
        public init(operationID: UInt64, generation: UInt64, targetYaw: Double, clockDomain: String,
                    completion: Completion = .completed, requestedBudget: Double,
                    sendEntryUptime: Double, sendResponseUptime: Double, stopObligationUptime: Double,
                    stopAcknowledgementUptime: Double, samples: [Sample], traversalUnambiguous: Bool = true,
                    planningEvaluation: Sample? = nil, stoppedEvaluation: Sample? = nil) {
            self.operationID = operationID
            self.generation = generation
            self.targetYaw = targetYaw
            self.clockDomain = clockDomain
            self.completion = completion
            self.requestedBudget = requestedBudget
            self.sendEntryUptime = sendEntryUptime
            self.sendResponseUptime = sendResponseUptime
            self.stopObligationUptime = stopObligationUptime
            self.stopAcknowledgementUptime = stopAcknowledgementUptime
            self.samples = samples
            self.traversalUnambiguous = traversalUnambiguous
            self.planningEvaluation = planningEvaluation
            self.stoppedEvaluation = stoppedEvaluation
        }
    }
    public struct Reduction: Sendable {
        public let calibration: Calibration
        /// Rejection preserves all prior calibration. It is not motion authority:
        /// integration must remain stopped and resolve invalid/cancelled evidence.
        public let rejection: String?
        public let signedResponse: Double?
        public let sampledAbsoluteTravel: Double?
        fileprivate init(calibration: Calibration, rejection: String?, signedResponse: Double? = nil,
                         sampledAbsoluteTravel: Double? = nil) {
            self.calibration = calibration
            self.rejection = rejection
            self.signedResponse = signedResponse
            self.sampledAbsoluteTravel = sampledAbsoluteTravel
        }
    }
    public static func recording(_ response: Response, in calibration: Calibration,
                                 profile: Profile) -> Reduction {
        func reject(_ reason: String) -> Reduction { .init(calibration: calibration, rejection: reason) }
        guard response.operationID == calibration.operationID, response.generation == calibration.generation,
              response.targetYaw.isFinite, response.targetYaw == calibration.targetYaw,
              !calibration.clockDomain.isEmpty, response.clockDomain == calibration.clockDomain else {
            return reject("attribution_changed")
        }
        guard response.completion == .completed, response.traversalUnambiguous else { return reject("incomplete_or_ambiguous") }
        let times = [response.sendEntryUptime, response.sendResponseUptime,
                     response.stopObligationUptime, response.stopAcknowledgementUptime]
        guard times.allSatisfy({ $0.isFinite && $0 >= 0 }),
              response.sendResponseUptime >= response.sendEntryUptime,
              response.stopObligationUptime >= response.sendEntryUptime,
              response.stopAcknowledgementUptime >= max(response.sendResponseUptime, response.stopObligationUptime),
              response.requestedBudget.isFinite, response.requestedBudget > 0,
              response.sendEntryUptime + response.requestedBudget > response.sendEntryUptime,
              response.requestedBudget <= profile.maximumHostBurstBudget else { return reject("invalid_host_timing") }
        if let previousAck = calibration.lastStopAcknowledgementUptime,
           response.sendEntryUptime <= previousAck { return reject("replayed_response") }
        var previous: Sample?
        for sample in response.samples {
            guard sample.healthy, sample.yaw.isFinite, (calibration.targetYaw - sample.yaw).isFinite,
                  let sequence = sample.sequence, sample.generation == calibration.generation,
                  sample.clockDomain == calibration.clockDomain, let timestamp = sample.sourceTimestamp,
                  timestamp.isFinite, timestamp >= 0, sample.collectedUptime.isFinite,
                  sample.collectedUptime - timestamp >= 0, sample.collectedUptime - timestamp <= 0.500 else {
                return reject("invalid_source_at_collection")
            }
            if let previous {
                guard sequence > previous.sequence!, timestamp > previous.sourceTimestamp!,
                      sample.collectedUptime >= previous.collectedUptime else { return reject("nonadvancing_source") }
                if abs(wrap(sample.yaw - previous.yaw)) == .pi { return reject("ambiguous_half_turn") }
            }
            previous = sample
        }
        var stoppedEvaluationTime: Double?
        if response.planningEvaluation != nil || response.stoppedEvaluation != nil {
            guard let planning = response.planningEvaluation, let stopped = response.stoppedEvaluation,
                  let first = response.samples.first, let last = response.samples.last else {
                return reject("missing_control_evaluation")
            }
            func validRead(_ read: Sample, of ingress: Sample) -> Bool {
                read.healthy && read.yaw == ingress.yaw && read.sequence == ingress.sequence &&
                    read.generation == ingress.generation && read.sourceTimestamp == ingress.sourceTimestamp &&
                    read.clockDomain == ingress.clockDomain && read.collectedUptime.isFinite &&
                    read.collectedUptime >= ingress.collectedUptime &&
                    read.collectedUptime - ingress.sourceTimestamp! >= 0 &&
                    read.collectedUptime - ingress.sourceTimestamp! <= 0.500
            }
            guard validRead(planning, of: first), validRead(stopped, of: last),
                  planning.collectedUptime <= response.sendEntryUptime,
                  stopped.collectedUptime >= response.stopAcknowledgementUptime + profile.settleWait,
                  last.sourceTimestamp! > response.stopAcknowledgementUptime else {
                return reject("invalid_control_evaluation")
            }
            stoppedEvaluationTime = stopped.collectedUptime
        }
        let hasSettledEndpoint = response.samples.last.map {
            (stoppedEvaluationTime ?? $0.collectedUptime) >= response.stopAcknowledgementUptime + profile.settleWait
                && $0.sourceTimestamp! > response.stopAcknowledgementUptime
        } ?? false
        let hasResponseBracket = response.samples.count >= 2 && hasSettledEndpoint
            && response.samples.first!.collectedUptime <= response.sendEntryUptime
        if let first = response.samples.first {
            // Adjacent brackets can share their exact immutable settled endpoint.
            // This is reuse as a boundary, never a claim of a new source frame.
            if first != calibration.lastSourceSample {
                if let sequence = calibration.lastSourceSequence, first.sequence! <= sequence { return reject("replayed_source") }
                if let timestamp = calibration.lastSourceTimestamp, first.sourceTimestamp! <= timestamp { return reject("replayed_source") }
            }
        }
        var rate = calibration.responseRate
        var overhead = calibration.responseOverhead
        var net = 0.0
        var sampledTravel = 0.0
        var postAckTravel = 0.0
        var postAckIntervals = 0
        for (before, after) in zip(response.samples, response.samples.dropFirst()) {
            let delta = wrap(after.yaw - before.yaw)
            let interval = after.sourceTimestamp! - before.sourceTimestamp!
            let observedRate = abs(delta) / interval
            guard delta.isFinite, observedRate.isFinite else { return reject("nonfinite_response_rate") }
            net += delta
            sampledTravel += abs(delta)
            if hasSettledEndpoint, before.sourceTimestamp! > response.stopAcknowledgementUptime {
                postAckTravel += abs(delta)
                postAckIntervals += 1
            }
        }
        if hasResponseBracket {
            let duration = response.samples.last!.sourceTimestamp! - response.samples.first!.sourceTimestamp!
            let sourceRate = abs(net) / duration
            // Include pending request drain and stopping exactly once. The full
            // sampled response includes coast; its time is not charged again.
            let window = max(response.requestedBudget,
                response.stopAcknowledgementUptime - response.sendEntryUptime)
            let windowResponseRate = sampledTravel / window
            guard net.isFinite, sampledTravel.isFinite, sourceRate.isFinite, windowResponseRate.isFinite, postAckTravel.isFinite else {
                return reject("nonfinite_response_rate")
            }
            rate = max(rate, windowResponseRate)
            overhead = max(overhead, window - response.requestedBudget)
        }
        let travel = postAckIntervals > 0
            ? max(calibration.observedPostAckTravel ?? 0, postAckTravel) : calibration.observedPostAckTravel
        var ceiling = calibration.overshootCeiling
        // Even a previously calibrated operation cannot retry after losing the
        // current response. Latency-only telemetry remains retainable evidence.
        var terminal = !hasResponseBracket
        if hasResponseBracket {
            let preError = wrap(calibration.targetYaw - response.samples.first!.yaw)
            let postError = wrap(calibration.targetYaw - response.samples.last!.yaw)
            let directedResponse = net * (preError < 0 ? -1 : 1)
            if abs(preError) > profile.tolerance, directedResponse >= abs(preError), abs(postError) > profile.tolerance {
                let previousExcess = abs(preError) - profile.tolerance
                let distance = abs(net)
                let shrink = response.requestedBudget * min(1, previousExcess / distance)
                if distance > previousExcess, shrink.isFinite, shrink > 0, shrink < response.requestedBudget {
                    ceiling = min(ceiling ?? .infinity, shrink)
                } else { terminal = true }
            }
        }
        return .init(calibration: .init(previous: calibration, rate: rate, overhead: overhead,
            send: max(calibration.maximumSendDuration ?? 0, response.sendResponseUptime - response.sendEntryUptime),
            stop: max(calibration.maximumStopDuration ?? 0, response.stopAcknowledgementUptime - response.stopObligationUptime),
            travel: travel, ceiling: ceiling, terminal: terminal, measured: hasResponseBracket,
            response: response, observedBurst: hasResponseBracket
                ? .init(budget: response.requestedBudget, signedTravel: net, absoluteTravel: sampledTravel) : nil), rejection: nil,
            signedResponse: hasResponseBracket ? net : nil,
            sampledAbsoluteTravel: hasResponseBracket ? sampledTravel : nil)
    }
    public static func plan(_ input: Input) -> Decision {
        guard validInput(input) else { return .unavailable }
        let error = wrap(input.calibration.targetYaw - input.actualYaw)
        if abs(error) <= input.profile.tolerance { return .arrived }
        let calibration = input.calibration
        if calibration.terminalResolutionFailure || (calibration.measuredResponses == 0 &&
            (input.provisionalProbeIssued || calibration.completedResponses > 0)) {
            return .resolutionFailure(.rotationResolutionInsufficient)
        }
        let excess = abs(error) - input.profile.tolerance
        // Invert travel = gain * (requested budget + observed overhead).
        // This inverse model is an estimate, not a physical minimum angle.
        // A complete small response may justify repeating its proven host budget.
        let candidate = excess / calibration.responseRate - calibration.responseOverhead
        let observed = repeatableBudget(calibration, error: error, profile: input.profile)
        let budget = min(input.profile.maximumHostBurstBudget, max(candidate, observed ?? -.infinity),
            calibration.overshootCeiling ?? .infinity)
        let deadline = input.sendEntryUptime + budget
        guard budget.isFinite, budget > 0, deadline.isFinite, deadline > input.sendEntryUptime else {
            return .resolutionFailure(.rotationResolutionInsufficient)
        }
        return .burst(direction: error < 0 ? -1 : 1, budget: budget)
    }

    /// Host overhead is not a physical minimum angle. A previously successful
    /// small pulse can be repeated if its entire sampled response fits the
    /// remaining heading corridor. Never enlarge it or override an overshoot cap.
    static func repeatableBudget(_ calibration: Calibration, error: Double, profile: Profile) -> Double? {
        guard !calibration.terminalResolutionFailure, calibration.overshootCeiling == nil,
              let observed = calibration.lastObservedBurst,
              observed.signedTravel * error > 0,
              observed.absoluteTravel > 0,
              observed.absoluteTravel <= abs(error) + profile.tolerance else { return nil }
        return observed.budget
    }

    /// Initial search covers an arc rather than converging on an exact heading.
    /// HTTP overhead and precision-correction ceilings do not select its pulse.
    /// The controller still requires a valid stopped response between pulses.
    static func planSearchSweep(_ input: Input, requestedAngle: Double, signedProgress: Double) -> Decision {
        guard validInput(input), requestedAngle.isFinite, requestedAngle != 0,
              abs(requestedAngle) <= .pi, signedProgress.isFinite else { return .unavailable }
        let direction = requestedAngle < 0 ? -1 : 1
        if signedProgress * Double(direction) >= abs(requestedAngle) { return .arrived }
        let calibration = input.calibration
        let error = wrap(calibration.targetYaw - input.actualYaw)
        guard !calibration.terminalResolutionFailure,
              !input.provisionalProbeIssued || calibration.measuredResponses > 0,
              Double(direction) * error > 0 || error == -.pi else {
            return .resolutionFailure(.rotationResolutionInsufficient)
        }
        let budget = input.profile.maximumHostBurstBudget
        let deadline = input.sendEntryUptime + budget
        guard deadline.isFinite, deadline > input.sendEntryUptime else {
            return .resolutionFailure(.rotationResolutionInsufficient)
        }
        return .burst(direction: direction, budget: budget)
    }

    private static func validInput(_ input: Input) -> Bool {
        input.authorized && input.actualYaw.isFinite && input.calibration.targetYaw.isFinite &&
            input.sendEntryUptime.isFinite && input.sendEntryUptime >= 0 &&
            (input.calibration.targetYaw - input.actualYaw).isFinite
    }
    private static func wrap(_ angle: Double) -> Double {
        var value = angle.truncatingRemainder(dividingBy: 2 * .pi)
        if value >= .pi { value -= 2 * .pi }
        if value < -.pi { value += 2 * .pi }
        return value
    }
}
