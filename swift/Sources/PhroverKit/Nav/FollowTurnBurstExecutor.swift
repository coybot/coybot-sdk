import Foundation
import RoverNav

struct FollowTurnBurstExecutionReceipt {
    let send: FollowTurnBurstSendReceipt
    let confirmedStopFence: FollowTurnStopFence?
    var stopObligationUptime: Double? = nil
}

/// One serialized burst. The controller supplies operation-fenced boundaries;
/// this executor never creates another motor owner or interprets a stop as arrival.
@MainActor
enum FollowTurnBurstExecutor {
    static func execute(command: WheelCommand, budget: Double,
                        send: (WheelCommand, Double) async -> FollowTurnBurstSendReceipt,
                        waitRemaining: (FollowTurnBurstSendReceipt) async -> Void,
                        stop: () async throws -> FollowTurnStopFence?) async throws -> FollowTurnBurstExecutionReceipt {
        let receipt = await send(command, budget)
        if receipt.result.failure == nil, !receipt.stopObligation {
            await waitRemaining(receipt)
        }
        let fence = try await stop()
        return .init(send: receipt, confirmedStopFence: fence)
    }
}

/// Shared follow operation loop. Its target and progress epoch are never rebased
/// at burst stops. Controller closures retain source, safety and motor ownership.
@MainActor
enum FollowTurnOperationExecutor {
    static func execute(targetYaw: Double, generation: UInt64, operationID: UInt64,
                         profile: FollowTurnBurstPlanner.Profile,
                          runtime: FollowTurnRuntimeState,
                          responseRateFloor: Double = 2 * .pi / 3,
                         requestedSearchSweep: Double? = nil,
                         searchSweepProgress: (Double, Double) -> Void = { _, _ in },
                        uptime: () -> Double, now: () -> Date, authorized: () -> Bool,
                        admit: (FollowTurnWaitingProgress) async -> FollowTurnSourceResult,
                          burst: (WheelCommand, Double, FollowTurnWaitingProgress, FollowTurnBurstPlanner.Sample?, NavigationPoseSample) async throws -> FollowTurnBurstExecutionReceipt,
                          stoppedSource: (FollowTurnStopFence, FollowTurnWaitingProgress) async -> FollowTurnSourceResult,
                          diagnostic: (FollowTurnBurstPlanner.Calibration, NavigationPoseSample, FollowTurnBurstPlanner.Decision, Double) -> Void,
                          reduced: (FollowTurnBurstPlanner.Response, FollowTurnBurstPlanner.Reduction) -> Void,
                         response: (FollowTurnBurstExecutionReceipt, Double, NavigationPoseSample) -> FollowTurnBurstPlanner.Response?) async -> NavigationResult {
        var calibration = FollowTurnBurstPlanner.Calibration(operationID: operationID, generation: generation,
            targetYaw: targetYaw, clockDomain: "ar_system_uptime", responseRateFloor: responseRateFloor)
        var progress: FollowTurnWaitingProgress {
            get { runtime.progress }
            set { runtime.progress = newValue }
        }
        var probeIssued = false
        var sweptAngle = 0.0
        var sampledTravel = 0.0
        while authorized(), !Task.isCancelled {
            let selection = await admit(progress)
            guard authorized(), !Task.isCancelled else { return .cancelled }
            if let failure = runtime.failure { return .failed(failure) }
            let sample: NavigationPoseSample
            switch selection {
            case .sample(let value): sample = value
            case .failed(let reason): return .failed(reason)
            case .cancelled: return .cancelled
            }
            guard let pose = sample.pose else { return .failed(.trackingLost) }
            progress.distanceToGoal = abs(FollowReacquisitionPlanner.wrap(targetYaw - pose.yaw))
            let planningUptime = uptime()
            let input = FollowTurnBurstPlanner.Input(actualYaw: pose.yaw, profile: profile,
                calibration: calibration, sendEntryUptime: planningUptime, provisionalProbeIssued: probeIssued)
            let decision = requestedSearchSweep.map {
                FollowTurnBurstPlanner.planSearchSweep(input, requestedAngle: $0, signedProgress: sweptAngle)
            } ?? FollowTurnBurstPlanner.plan(input)
            diagnostic(calibration, sample, decision, planningUptime)
            switch decision {
            case .arrived:
                // A later pose alone cannot manufacture directed sweep evidence.
                if requestedSearchSweep != nil && probeIssued { return .failed(.rotationResolutionInsufficient) }
                return .arrived
            case .resolutionFailure(let reason): return .failed(reason)
            case .unavailable: return .failed(.trackingLost)
            case .burst(let direction, let budget):
                if let sweep = requestedSearchSweep, direction != (sweep < 0 ? -1 : 1) {
                    return .failed(.rotationResolutionInsufficient)
                }
                if progress.watchdog.observe(distanceToGoal: progress.distanceToGoal, now: now(), commanded: true) {
                    return .failed(.stalled)
                }
                let signed = Double(direction) * profile.fixedWheelMagnitude
                let receipt: FollowTurnBurstExecutionReceipt
                do { receipt = try await burst(.init(left: -signed, right: signed), budget, progress, calibration.lastSourceSample, sample) }
                catch {
                    guard authorized(), !Task.isCancelled else { return .cancelled }
                    if let failure = runtime.failure { return .failed(failure) }
                    return .failed(.commandFailed)
                }
                probeIssued = true
                guard authorized(), !Task.isCancelled else { return .cancelled }
                guard let fence = receipt.confirmedStopFence else { return .failed(.commandFailed) }
                runtime.expire(at: now())
                if let failure = runtime.failure { return .failed(failure) }
                if receipt.send.result.failure != nil {
                    let send = receipt.send
                    if send.definiteUnsentTargetStop {
                        // A valid source inhibited an unnecessary request before
                        // HTTP. It is not a transport failure or a motion response.
                        let settled: NavigationPoseSample
                        switch await stoppedSource(fence, progress) {
                        case .sample(let value): settled = value
                        case .failed(let reason): return .failed(reason)
                        case .cancelled: return .cancelled
                        }
                        guard authorized(), !Task.isCancelled else { return .cancelled }
                        runtime.expire(at: now())
                        if let failure = runtime.failure { return .failed(failure) }
                        guard let pose = settled.pose else { return .failed(.trackingLost) }
                        let error = abs(FollowReacquisitionPlanner.wrap(targetYaw - pose.yaw))
                        guard error.isFinite, error <= profile.tolerance else {
                            return .failed(.rotationResolutionInsufficient)
                        }
                        // This branch has no entered command/response bracket.
                        // A target pose can complete an exact heading, but cannot
                        // supply the unmeasured remainder of a directed sweep.
                        guard requestedSearchSweep == nil else {
                            return .failed(.rotationResolutionInsufficient)
                        }
                        diagnostic(calibration, settled, .arrived, uptime())
                        return .arrived
                    }
                    // An entered request may have moved the rover even if its response
                    // failed. Only an unambiguous, typed, zero-attempt expiry is not-started.
                    if send.definitePreSendExpiry {
                        FollowMotionTaskScope.evidence?.recordFailure(.rotationResolutionInsufficient,
                            cause: .burstPreSendExpired)
                        return .failed(.rotationResolutionInsufficient)
                    }
                    return .failed(.commandFailed)
                }
                let settled: NavigationPoseSample
                switch await stoppedSource(fence, progress) {
                case .sample(let sample): settled = sample
                case .failed(let reason): return .failed(reason)
                case .cancelled: return .cancelled
                }
                guard authorized(), !Task.isCancelled else { return .cancelled }
                runtime.expire(at: now())
                if let failure = runtime.failure { return .failed(failure) }
                // Calibration validity and actual stopped arrival are independent.
                // Preserve rejected-bracket diagnostics without learning or retrying.
                let reduction = response(receipt, budget, settled).map { captured in
                    let reduction = FollowTurnBurstPlanner.recording(captured, in: calibration, profile: profile)
                    reduced(captured, reduction)
                    return reduction
                }
                guard let pose = settled.pose else { return .failed(.trackingLost) }
                let error = abs(FollowReacquisitionPlanner.wrap(targetYaw - pose.yaw))
                guard error.isFinite else { return .failed(.trackingLost) }
                if let sweep = requestedSearchSweep {
                    guard let reduction, reduction.rejection == nil,
                          let net = reduction.signedResponse, let travel = reduction.sampledAbsoluteTravel else {
                        FollowMotionTaskScope.evidence?.recordFailure(.rotationResolutionInsufficient,
                            cause: .calibrationEvidenceIncomplete)
                        return .failed(.rotationResolutionInsufficient)
                    }
                    guard net * (sweep < 0 ? -1 : 1) >= 0 else {
                        return .failed(.rotationResolutionInsufficient)
                    }
                    sweptAngle += net
                    sampledTravel += travel
                    guard sweptAngle.isFinite, sampledTravel.isFinite else { return .failed(.rotationResolutionInsufficient) }
                    searchSweepProgress(sweptAngle, sampledTravel)
                    // Search covers an arc and then looks from the confirmed stopped
                    // pose. It does not reverse to recover an already swept heading.
                    if sweptAngle * (sweep < 0 ? -1 : 1) >= abs(sweep) {
                        diagnostic(reduction.calibration, settled, .arrived, uptime())
                        return .arrived
                    }
                } else if error <= profile.tolerance {
                    diagnostic(reduction?.calibration ?? calibration, settled, .arrived, uptime())
                    return .arrived
                }
                guard let reduction, reduction.rejection == nil else {
                    FollowMotionTaskScope.evidence?.recordFailure(.rotationResolutionInsufficient,
                        cause: .calibrationEvidenceIncomplete)
                    return .failed(.rotationResolutionInsufficient)
                }
                calibration = reduction.calibration
            }
        }
        return .cancelled
    }
}
