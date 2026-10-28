import Foundation
import RoverNav

public enum FollowMeState: Equatable {
    case idle, pausing, searching, aligning, waitingForClearance, recoveringReadyPath, signalingReady, waitingForMovement, following, holdingDistance, reacquiring, stopped
    case failed(String)

    public var isActive: Bool {
        switch self {
        case .pausing, .searching, .aligning, .waitingForClearance, .recoveringReadyPath, .signalingReady, .waitingForMovement, .following, .holdingDistance, .reacquiring: true
        default: false
        }
    }
}

@Observable
@MainActor
public final class FollowMeCoordinator {
    public private(set) var state: FollowMeState = .idle {
        didSet {
            if state != oldValue {
                if state != .searching { clearScanAdmissionWait() }
                if state == .signalingReady || state == .following || state == .reacquiring {
                    searchClearance?.revoke()
                    searchClearance = nil
                }
                let changedAt = clock.now
                let previousPhaseElapsed = phaseStartedAt.map { changedAt - $0 }
                phaseStartedAt = changedAt
                log("follow_state")
                var phasePayload = sessionTimingPayload
                phasePayload.merge(recoverySnapshot(now: changedAt)) { _, new in new }
                phasePayload["previous_phase"] = .string(String(describing: oldValue))
                phasePayload["previous_phase_elapsed_s"] = previousPhaseElapsed.map { .number($0) } ?? .null
                failureEmitter.emit(.init(event: "follow_phase", context: .init(sessionGeneration: generation,
                    phase: String(describing: state)), payload: phasePayload))
                if state == .waitingForClearance {
                    emitReadiness("follow_ready.clearance_entered", reason: "too_close_before_send")
                } else if oldValue == .waitingForClearance {
                    emitReadiness("follow_ready.clearance_exited", reason: state == .signalingReady ? "first_send_authorized" : "state_transition")
                }
            }
        }
    }
    public private(set) var perceptionIssue: FollowPerceptionIssue?
    public var isActive: Bool { state.isActive }
    /// Display-only current lock provenance, not independent motion authority.
    public var trackedPersonFrameID: ARFrameID? {
        guard isActive, !stopBlocked, perceptionIssue == nil, recoveryEpisode == nil,
               state != .pausing, state != .searching, state != .reacquiring, state != .recoveringReadyPath,
              let locked, let batch = latestBatch, locked.frameID == batch.frameID,
              batch.trackingQuality == .normal else { return nil }
        let age = clock.now - batch.timestamp
        return age.isFinite && age >= 0 && age <= config.maximumObservationAge ? batch.frameID : nil
    }
    public var readySignalClearance: Double { config.readySignalClearance }

    private let perception: any FollowMePerception
    private let motion: any FollowMeMotion
    private let clock: any FollowMeClock
    private let tracker: FollowTargetTracker
    private let config: FollowMeConfiguration
    private let eventSink: @MainActor (String, [String: String]) -> Void
    private let failureEmitter: FollowDiagnosticEmitter
    // At most one terminal delivery window; prune after stream/result/cleanup drain.
    private var failureResolution: FollowMotionFailureResolution?
    private var searchClearance: FollowSearchClearanceGrant?
    private var activeRequest: FollowMotionRequestContext?
    private var summaryBudget = FollowSummaryBudget()
    private var diagnosticStopState = "unknown"
    private var readinessStopPending = false
    private var loggedIssue: FollowPerceptionIssue?
    private var loggedTrackingReason: FollowTrackingReason?
    private var generation: UInt64 = 0
    private var operation: UInt64 = 0
    private var framesTask: Task<Void, Never>?
    private var frameProcessorTask: Task<Void, Never>?
    private var pendingFrame: FollowFrameBatch?
    private var pendingHealthFrame: FollowFrameBatch?
    private var evaluatedPendingSnapshot: FollowAdmissionSnapshot?
    /// Original continuity decision, before adopting this frame's selected lock.
    private var currentAssociation: (generation: UInt64, frameID: ARFrameID, decision: FollowTrackMatch, evaluation: FollowAssociationEvaluation)?
    private var safetyTask: Task<Void, Never>?
    private var movementTask: Task<Void, Never>?
    private var deadlineTask: Task<Void, Never>?
    private var poseTask: Task<Void, Never>?
    private var frameWatchdogTask: Task<Void, Never>?
    private var startupTask: Task<Void, Never>?
    private var pauseTask: Task<Void, Never>?
    private var alignmentTask: Task<Void, Never>?
    private var acquisitionHold: (id: UUID, deadline: TimeInterval)?
    private var acquisitionHoldTask: Task<Void, Never>?
    private var alignmentSerial: UInt64 = 0
    private var alignmentConfirmedAfter: ARFrameID?
    private var alignmentCompletionTime: TimeInterval?
    private var awaitingDepthReobservation = false
    private var newestStopFence: FollowTurnStopFence?
    private var ingestedSource = FollowTurnSourceGate()
    private var alignmentSourceWaiter: CheckedContinuation<Void, Never>?
    private var departureBaseline: Double?
    private var readySignalAttempted = false
    private var readyPathRecoveryAttempts = 0
    private var activeReadyAdmission: FollowReadyAdmission?
    private var lastFrameTimingUptime = -Double.infinity
    private var pendingAdmission: FollowReadyAdmissionToken?
    private var readySignalSucceeded = false
    private var readySignalConfirmedAfter: ARFrameID?
    private var readySignalCompletionTime: TimeInterval?
    private var departurePending = true
    private var perceptionReady = false
    private var readinessDeadline: TimeInterval?
    private var stopTask: Task<Bool, Never>?
    private var stopCleanupOwned = false
    private var confirmationTask: Task<Bool, Never>?
    private var confirmationID: UInt64 = 0
    private var stopBlocked = false
    private var locked: FollowPersonObservation?
    private var reliableMemory: FollowReliableMemory?
    private var recoveryEpisode: FollowReacquisitionEpisode?
    private var recoveryCenterSource: NavigationPoseSample?
    private var recoveryCenterReadUptime: Double?
    private var recoveryDiagnosticUnavailable: String?
    private var pendingRecoveryTermination: FollowDiagnosticEvent?
    private var recoveryHasProvisionalLock = false
    private struct RecoveryStopBoundary {
        let frameID: ARFrameID
        let time: TimeInterval
    }
    private var recoveryStopBoundary: RecoveryStopBoundary?
    private var poseDeadline: TimeInterval?
    private var scanRotation: Double = 0
    private var scanAttempts = 0
    private var scanInterruptions = 0
    private var scanAdmissionDeferrals = 0
    private var scanAdmissionDeadline: TimeInterval?
    private var scanAdmissionTask: Task<Void, Never>?
    private var scanDeferredFrame: ARFrameID?
    private final class ScanDispatch { var entered = false }
    private var scanning = false
    private struct ScanObservationGate {
        let until: TimeInterval
        let afterFrame: ARFrameID
    }
    private var scanObservation: ScanObservationGate?
    private var scanResponseSeed: FollowScanResponseSeed?
    private var lastGoal: Vec2?
    private var lastGoalTime: TimeInterval?
    /// Constant-velocity track of the locked person (`config.coastSeconds > 0`). Seeded on the
    /// first steady-following frame and dropped whenever the lock is dropped.
    private var track: TargetKalmanFilter?
    /// Capture time of the last full position measurement / of the last measurement of any kind.
    private var lastPositionAt: TimeInterval?
    private var lastMeasurementAt: TimeInterval?
    private var coasting = false
    private var latestFrame: ARFrameID?
    private var latestBatch: FollowFrameBatch?
    private var commandReceivedAt: TimeInterval?
    private var sessionStartedAt: TimeInterval?
    private var phaseStartedAt: TimeInterval?

    private var perceptionFailureMessage: String {
        let issue = perceptionIssue ?? .noFrames
        if issue == .trackingLimited, let reason = latestBatch?.trackingReason {
            return "AR tracking limited (\(reason.rawValue)). \(reason.action)"
        }
        return issue.message
    }

    public init(perception: any FollowMePerception, motion: any FollowMeMotion,
                clock: any FollowMeClock, configuration: FollowMeConfiguration = .init(),
                eventSink: @escaping @MainActor (String, [String: String]) -> Void = {
                    RuntimeFileLog.append($0, fields: $1)
                }) {
        self.perception = perception
        self.motion = motion
        self.clock = clock
        self.config = configuration
        self.tracker = FollowTargetTracker(configuration: configuration)
        self.eventSink = eventSink
        self.failureEmitter = FollowDiagnosticEmitter(streamID: "follow-coordinator-\(UUID().uuidString)",
            monotonic: { clock.now }, utc: { Date() }, sink: eventSink)
    }

    @discardableResult
    public func start() async -> Bool {
        await startSession(commandReceivedAt: nil)
    }

    func start(commandReceivedAt: TimeInterval) async -> Bool {
        await startSession(commandReceivedAt: commandReceivedAt)
    }

    func start(commandReceivedAt: TimeInterval, searchClearance: FollowSearchClearanceConfirmation) async -> Bool {
        await startSession(commandReceivedAt: commandReceivedAt, confirmation: searchClearance)
    }

    private func startSession(commandReceivedAt: TimeInterval?, confirmation: FollowSearchClearanceConfirmation? = nil) async -> Bool {
        guard !isActive, stopTask == nil, !stopBlocked else { return isActive }
        guard perception.detectorReady, perception.personLabelAvailable else {
            state = .failed("Person detector unavailable.")
            return false
        }
        generation &+= 1
        searchClearance?.revoke()
        searchClearance = confirmation?.consume(session: generation)
        self.commandReceivedAt = commandReceivedAt
        sessionStartedAt = clock.now
        phaseStartedAt = nil
        safetyTask?.cancel()
        failureResolution = nil
        activeRequest = nil
        let token = generation
        locked = nil
        resetTrack()
        alignmentConfirmedAfter = nil
        departureBaseline = nil
        readySignalAttempted = false
        readyPathRecoveryAttempts = 0
        pendingAdmission = nil
        activeReadyAdmission = nil
        readySignalSucceeded = false
        readySignalConfirmedAfter = nil
        readySignalCompletionTime = nil
        departurePending = config.departureRangeIncrease > 0
        reliableMemory = nil
        recoveryEpisode = nil
        recoveryCenterSource = nil
        recoveryCenterReadUptime = nil
        recoveryDiagnosticUnavailable = nil
        pendingRecoveryTermination = nil
        recoveryHasProvisionalLock = false
        recoveryStopBoundary = nil
        poseDeadline = nil
        perceptionReady = false
        scanRotation = 0
        scanAttempts = 0
        scanInterruptions = 0
        scanAdmissionDeferrals = 0
        clearScanAdmissionWait()
        scanning = false
        scanObservation = nil
        scanResponseSeed = nil
        lastGoal = nil
        lastGoalTime = nil
        latestFrame = nil
        latestBatch = nil
        newestStopFence = nil
        awaitingDepthReobservation = false
        ingestedSource = FollowTurnSourceGate()
        evaluatedPendingSnapshot = nil
        currentAssociation = nil
        summaryBudget = FollowSummaryBudget()
        diagnosticStopState = "unknown"
        readinessStopPending = false
        loggedIssue = nil
        loggedTrackingReason = nil
        perceptionIssue = .noFrames
        state = config.stationaryPauseSeconds > 0 ? .pausing : .searching
        failureEmitter.emit(.init(event: "follow_session.started", context: .init(sessionGeneration: token,
            phase: String(describing: state)), payload: sessionTimingPayload))
        updatePerceptionIssue(.noFrames)
        let events = perception.events()
        framesTask = Task { [weak self] in
            for await event in events {
                guard !Task.isCancelled, let self, self.generation == token, self.isActive else { break }
                await self.receive(event, generation: token)
            }
            if !Task.isCancelled, let self, self.generation == token, self.isActive {
                _ = await self.finish(.failed("Perception ended."))
            }
        }
        if let contextual = motion as? any FollowMeContextualMotion {
            let failures = contextual.motionFailures()
            safetyTask = Task { [weak self] in
                for await failure in failures {
                    guard !Task.isCancelled, let self else { break }
                    await self.resolveFailure(failure, subscriptionGeneration: token)
                }
            }
        } else {
            let safety = motion.safetyStates()
            safetyTask = Task { [weak self] in
                for await value in safety {
                    guard let self, self.generation == token else { break }
                    if case .failed(let reason) = value {
                        await self.resolveFailure(.init(context: .unknown, reason: reason,
                            stopOutcome: .unknown, source: .stream), subscriptionGeneration: token)
                    }
                }
            }
        }
        let pauseStartedAt = clock.now
        let pauseDeadline = pauseStartedAt + config.stationaryPauseSeconds
        let startupDeadline = pauseDeadline + config.startupReadinessSeconds
        readinessDeadline = startupDeadline
        startupTask = Task { [weak self, clock] in
            await clock.sleep(seconds: max(0, startupDeadline - clock.now))
            guard !Task.isCancelled, let self, self.generation == token, !self.perceptionReady else { return }
            _ = await self.finish(.failed(self.perceptionFailureMessage))
        }
        if state == .pausing {
            var payload = sessionTimingPayload
            payload["pause_started_at_s"] = .number(pauseStartedAt)
            payload["pause_deadline_s"] = .number(pauseDeadline)
            failureEmitter.emit(.init(event: "follow_pause.started", context: .init(sessionGeneration: token,
                phase: "pausing"), payload: payload))
            pauseTask = Task { [weak self, clock] in
                await clock.sleep(seconds: max(0, pauseDeadline - clock.now))
                guard !Task.isCancelled, let self, self.generation == token, self.state == .pausing else { return }
                var payload = self.sessionTimingPayload
                payload["pause_started_at_s"] = .number(pauseStartedAt)
                payload["pause_deadline_s"] = .number(pauseDeadline)
                payload["pause_elapsed_s"] = .number(clock.now - pauseStartedAt)
                self.failureEmitter.emit(.init(event: "follow_pause.completed", context: .init(sessionGeneration: token,
                    phase: "pausing"), payload: payload))
                self.state = .searching
                self.perceptionReady = false
                // Revalidate the latest pause frame against the clock at pause completion.
                if let batch = self.latestBatch {
                    self.latestFrame = nil
                    await self.receive(batch, generation: token)
                }
            }
        }
        return true
    }

    @discardableResult
    public func stop() async -> Bool { await finish(.stopped) }

    /// Synchronously fence observations and goals when the app is leaving the foreground
    /// or Talk. The caller must subsequently await stop() before handing off motion.
    public func inhibitMotion() {
        searchClearance?.revoke()
        guard isActive, stopTask == nil else { return }
        emitRecovery("fenced", reason: "lifecycle_inhibition", stop: "pending")
        pendingRecoveryTermination = recoveryTermination(reason: "lifecycle_inhibition", phase: .stopped)
        emitReadyCancellation(.stopped)
        pendingAdmission = nil
        activeReadyAdmission?.endRecoveryRefresh()
        activeReadyAdmission = nil
        generation &+= 1
        operation &+= 1
        state = .stopped
        framesTask?.cancel()
        frameProcessorTask?.cancel()
        frameProcessorTask = nil
        pendingFrame = nil
        pendingHealthFrame = nil
        evaluatedPendingSnapshot = nil
        currentAssociation = nil
        safetyTask?.cancel()
        movementTask?.cancel()
        deadlineTask?.cancel()
        poseTask?.cancel()
        frameWatchdogTask?.cancel()
        startupTask?.cancel()
        pauseTask?.cancel()
        cancelAlignment()
        let pending = confirmationTask
        stopTask = Task { [motion] in
            if let pending, !(await pending.value) { return false }
            do { try await motion.stopAndConfirm(); return true }
            catch { return false }
        }
    }

    private func finish(_ result: FollowMeState, resolvingFailure: Bool = false) async -> Bool {
        searchClearance?.revoke()
        if let stopTask {
            // Only one waiter publishes terminal state and clears the task.
            // Followers must not overwrite resolved failure text or clear a
            // replacement stop after the original owner's acknowledgement.
            if stopCleanupOwned { return await stopTask.value }
            stopCleanupOwned = true
            let recoveryTermination = pendingRecoveryTermination
            pendingRecoveryTermination = nil
            let confirmed = await stopTask.value
            if !confirmed {
                stopBlocked = true
                state = .failed("Rover stop could not be confirmed.")
            }
            self.stopTask = nil
            stopCleanupOwned = false
            recordReadinessStop(confirmed)
            if let recoveryTermination { emitRecoveryTermination(recoveryTermination, confirmed: confirmed) }
            return confirmed
        }
        if !isActive {
            guard stopBlocked else { return true }
            let retry = Task { [motion] in
                do { try await motion.stopAndConfirm(); return true }
                catch { return false }
            }
            stopCleanupOwned = true
            stopTask = retry
            let confirmed = await retry.value
            if confirmed {
                stopBlocked = false
                failureResolution = nil
                activeRequest = nil
                state = .stopped
            }
            stopTask = nil
            stopCleanupOwned = false
            recordReadinessStop(confirmed)
            return confirmed
        }
        let terminalRecovery = recoveryTermination(reason: result == .stopped ? "local_stop" : "terminal_failure", phase: result)
        emitReadyCancellation(result)
        // A safety-stream failure can terminate before the movement result returns.
        // Revoke its unconsumed reservation before publishing terminal state.
        pendingAdmission = nil
        activeReadyAdmission?.endRecoveryRefresh()
        activeReadyAdmission = nil
        generation &+= 1 // Inhibit callbacks and new goals before the first suspension.
        operation &+= 1
        state = result
        framesTask?.cancel()
        frameProcessorTask?.cancel()
        frameProcessorTask = nil
        pendingFrame = nil
        pendingHealthFrame = nil
        evaluatedPendingSnapshot = nil
        currentAssociation = nil
        if !resolvingFailure { safetyTask?.cancel() }
        movementTask?.cancel()
        deadlineTask?.cancel()
        poseTask?.cancel()
        frameWatchdogTask?.cancel()
        startupTask?.cancel()
        pauseTask?.cancel()
        cancelAlignment()
        framesTask = nil
        if !resolvingFailure { safetyTask = nil }
        movementTask = nil
        deadlineTask = nil
        poseTask = nil
        let pendingConfirmation = confirmationTask
        let task = Task { [motion] in
            if let pendingConfirmation {
                guard await pendingConfirmation.value else { return false }
            }
            do { try await motion.stopAndConfirm(); return true }
            catch { return false }
        }
        stopCleanupOwned = true
        stopTask = task
        let confirmed = await task.value
        if !confirmed {
            stopBlocked = true
            state = .failed("Rover stop could not be confirmed.")
        }
        recordReadinessStop(confirmed)
        if resolvingFailure, let record = failureResolution {
            let acknowledgement = FollowMotionFailureDelivery(context: record.context, reason: record.primaryReason,
                stopOutcome: confirmed ? .confirmed : .failed, source: .confirmation)
            var updated = record
            updated.consume(acknowledgement)
            failureResolution = updated
            emitFailureResolution(updated, delivery: acknowledgement, stale: false)
            state = .failed(updated.message)
            if updated.stopOutcome == .failed { stopBlocked = true }
        }
        stopTask = nil
        stopCleanupOwned = false
        pruneFailureResolution()
        if let terminalRecovery { emitRecoveryTermination(terminalRecovery, confirmed: confirmed) }
        return confirmed
    }

    private func matches(_ context: FollowMotionOperationContext, _ other: FollowMotionOperationContext) -> Bool {
        guard context.request?.sessionGeneration == other.request?.sessionGeneration else { return false }
        if let id = context.controllerOperationID, let otherID = other.controllerOperationID { return id == otherID }
        return context.request?.requestToken == other.request?.requestToken
    }

    private func resolveFailure(_ supplied: FollowMotionFailureDelivery, subscriptionGeneration: UInt64? = nil) async {
        // Legacy streams expose no operation facts. Attach only the request we actually own,
        // leaving controller purpose/profile/ID unknown rather than inferring scan/stall context.
        let context: FollowMotionOperationContext
        if supplied.context.request == nil, supplied.context.controllerOperationID == nil,
           subscriptionGeneration == generation {
            context = .init(request: activeRequest, controllerOperationID: nil, purpose: nil, profile: nil)
        } else { context = supplied.context }
        let delivery = FollowMotionFailureDelivery(context: context, reason: supplied.reason,
            stopOutcome: supplied.stopOutcome, source: supplied.source, stale: supplied.stale,
            commandReceipt: supplied.commandReceipt, stopReceipt: supplied.stopReceipt,
            turnDiagnosticFields: supplied.turnDiagnosticFields)
        let retained = failureResolution.map { matches(context, $0.context) } ?? false
        let current = !delivery.stale && (context.request.map {
            $0.sessionGeneration == generation && $0.requestToken == activeRequest?.requestToken
        } ?? (subscriptionGeneration == generation && isActive))
        var record = retained ? failureResolution! : FollowMotionFailureResolution(delivery)
        if retained { record.consume(delivery) }
        let stale = delivery.stale || !current
        emitFailureResolution(record, delivery: delivery, stale: stale)
        guard retained || current else { return }
        failureResolution = record
        if retained {
            // An in-flight terminal result can enrich evidence after fencing. Explicit stale
            // callbacks never publish UI, and no second delivery creates another stop owner.
            if !delivery.stale {
                state = .failed(record.message)
                if record.stopOutcome == .failed { stopBlocked = true }
            }
            pruneFailureResolution()
            return
        }
        guard isActive else { return }
        _ = await finish(.failed(record.message), resolvingFailure: true)
    }

    private func pruneFailureResolution() {
        guard stopTask == nil, failureResolution?.deliveriesDrained == true else { return }
        failureResolution = nil
        activeRequest = nil
    }

    private func consumeResult(_ result: FollowMotionResult) async -> Bool {
        if case .notStarted = result.outcome { return false }
        if case .scanDeferred = result.outcome { return false }
        if let failure = result.failure {
            await resolveFailure(failure)
            return true
        }
        if case .failed(let reason) = result.result {
            await resolveFailure(.init(context: result.context, reason: reason,
                stopOutcome: result.stopOutcome, source: .result))
            return true
        }
        // An initial turn must stop and obtain a newly captured,
        // settled perception frame even when the configured look delay is zero.
        let initialSearchStopped = result.result == .arrived &&
            ((result.context.purpose == .followScan && result.context.controllerOperationID != nil) ||
             result.context.request?.searchClearance != nil) &&
            result.context.request?.isSearchSweep == true && result.context.request?.phase == "searching"
        if result.result == .reobservationRequired || initialSearchStopped,
           result.context.request == activeRequest, result.context.request?.sessionGeneration == generation {
            guard result.stopOutcome == .confirmed, let fence = result.turnStopFence,
                  fence.context == result.context else {
                await resolveFailure(.init(context: result.context, reason: .commandFailed,
                    stopOutcome: result.stopOutcome, source: .result))
                return true
            }
            captureStopFence(fence)
            awaitingDepthReobservation = true
            if result.result == .reobservationRequired { scanResponseSeed = nil }
            alignmentConfirmedAfter = nil
            alignmentCompletionTime = nil
        }
        // Cancellation/success can carry terminal stop evidence, but cannot invent a failure.
        if let record = failureResolution, matches(result.context, record.context) {
            await resolveFailure(.init(context: result.context, reason: record.primaryReason,
                stopOutcome: result.stopOutcome, source: .result,
                stale: result.context.request?.sessionGeneration != generation))
        }
        return false
    }

    private func emitFailureResolution(_ record: FollowMotionFailureResolution,
                                       delivery: FollowMotionFailureDelivery, stale: Bool) {
        failureEmitter.emit(.init(event: "follow_motion.failure_resolution", context: .init(
            sessionGeneration: record.context.request?.sessionGeneration,
            operationID: record.context.controllerOperationID, purpose: record.context.purpose?.rawValue,
            phase: record.context.request?.phase, stale: stale, outcome: "failed", reason: record.diagnosticReason),
            payload: record.turnDiagnosticFields.merging(["source": .string(delivery.source.rawValue),
                "request_token": record.context.request.map { .number(Double($0.requestToken)) } ?? .null,
                "primary_typed_reason": .string(String(describing: record.primaryReason)),
                "delivered_typed_reason": .string(String(describing: delivery.reason)),
                "stop_outcome": .string(record.stopOutcome.rawValue),
                "formatter_message": .string(record.message), "priority": .number(Double(record.priority)),
                "deduplicated": .bool(record.deduplicated)]) { _, value in value }))
    }

    private func receive(_ event: FollowPerceptionEvent, generation token: UInt64) async {
        switch event {
        case .interrupted: _ = await finish(.failed("AR session interrupted."))
        case .failed(let message): _ = await finish(.failed(message))
        case .frame(var batch):
            guard generation == token, isActive, !stopBlocked else { return }
            ingestedSource.include(.init(frameID: batch.frameID, sourceTimestamp: batch.timestamp))
            let receivedAt = clock.now
            batch.ingestedUptime = receivedAt
            searchClearance?.validate(sample: .init(pose: batch.pose, frameID: batch.frameID,
                sourceTimestamp: batch.timestamp, trackingQuality: batch.trackingQuality), session: token, now: receivedAt)
            if let expected = recoveryEpisode?.anchor?.frameID.generation, batch.frameID.generation != expected {
                _ = await finish(.failed("AR session reset during recovery."))
                return
            }
            if let episode = recoveryEpisode, receivedAt >= episode.deadline {
                _ = await expireRecovery(generation: token)
                return
            }
            if let previous = pendingFrame?.frameID ?? latestFrame,
                batch.frameID.generation == previous.generation,
                batch.frameID.sequence <= previous.sequence { return }
            // Once an outage has already fenced motion and requested its stop,
            // intermediate bad frames are superseded by the newest recovery
            // observation. A new fault after recovery begins is latched again.
            if poseDeadline == nil, FollowFrameHealth.safetyTransitionPriority(batch) > 0, frameIssue(batch, now: receivedAt) != nil,
               pendingHealthFrame == nil || FollowFrameHealth.safetyTransitionPriority(batch) > FollowFrameHealth.safetyTransitionPriority(pendingHealthFrame!) {
                pendingHealthFrame = batch
            }
            pendingFrame = batch
            wakeAlignmentSourceWaiter()
            guard frameProcessorTask == nil else { return }
            frameProcessorTask = Task { [weak self] in
                guard let self else { return }
                defer {
                    // An old processor must never clear a newer generation's task.
                    if self.generation == token { self.frameProcessorTask = nil }
                }
                while !Task.isCancelled, self.generation == token, self.isActive,
                      let frame = self.pendingHealthFrame ?? self.pendingFrame {
                    let preservingHealthFault = self.pendingHealthFrame != nil
                    self.pendingHealthFrame = nil
                    if self.pendingFrame?.frameID == frame.frameID { self.pendingFrame = nil }
                    await self.receive(frame, generation: token, preservingHealthFault: preservingHealthFault)
                }
            }
        }
    }

    private func receive(_ batch: FollowFrameBatch, generation token: UInt64, preservingHealthFault: Bool = false) async {
        guard generation == token, isActive, !stopBlocked else { return }
        if recoveryExpired { _ = await expireRecovery(generation: token); return }
        if state != .pausing, !perceptionReady, let deadline = readinessDeadline, clock.now >= deadline {
            _ = await finish(.failed(perceptionFailureMessage))
            return
        }
        // A timer and frame processor can resume in either order at the boundary.
        // Never let a late frame clear an outage whose recovery window has ended.
        if let deadline = poseDeadline, clock.now >= deadline {
            _ = await finish(.failed(perceptionFailureMessage))
            return
        }
        let evaluatedPending = evaluatedPendingSnapshot?.batch.frameID == batch.frameID ? evaluatedPendingSnapshot : nil
        if let latestFrame, batch.frameID.generation == latestFrame.generation,
            batch.frameID.sequence <= latestFrame.sequence, evaluatedPending == nil {
            // A health transition is an obligation to stop, not an older target
            // observation that can be discarded after another consumer commits
            // a newer match. Keep current geometry while processing the fault.
            if preservingHealthFault, let issue = frameIssue(batch, now: clock.now) {
                await perceptionUnavailable(issue, generation: token)
            }
            return
        }
        evaluatedPendingSnapshot = nil
        latestFrame = batch.frameID
        currentAssociation = nil
        latestBatch = batch
        let now = clock.now
        if let published = batch.publishedUptime, let ingested = batch.ingestedUptime,
           now - lastFrameTimingUptime >= 1 {
            lastFrameTimingUptime = now
            RuntimeFileLog.append("follow_perception_delivery", fields: [
                "frame_id": "\(batch.frameID.generation):\(batch.frameID.sequence)",
                "capture_to_receive_s": String(now - batch.timestamp),
                "publish_to_ingest_s": String(ingested - published),
                "ingest_to_process_s": String(now - ingested)])
        }
        frameWatchdogTask?.cancel()
        let issue = frameIssue(batch, now: now)
        updatePerceptionIssue(issue)
        var associationAvailable = false
        let frameContext = FollowDiagnosticContext(sessionGeneration: token, phase: String(describing: state),
            outcome: issue?.rawValue ?? "healthy")
        let framePayload = FollowAssociationEvaluation.healthPayload(batch: batch, now: now)
        defer {
            // An evaluated association owns the combined summary, including when
            // its periodic allowance is exhausted. No deferred frame history.
            if !associationAvailable, generation == token, isActive,
               summaryBudget.takePipeline(signature: (issue?.rawValue ?? "healthy") + "|"
                    + (batch.trackingReason?.rawValue ?? "none") + "|"
                    + (batch.perceptionDiagnostics?.transitionSignature ?? "unknown"), now: now, healthy: issue == nil) {
                failureEmitter.emit(.init(event: "follow_frame", context: frameContext,
                    payload: framePayload.merging(recoverySnapshot(now: now)) { _, new in new }))
            }
        }
        // Observe diagnostics throughout the mandatory stationary interval. Readiness
        // and the post-readiness outage watchdog begin only after that interval.
        if state == .pausing { return }
        if !perceptionReady {
            guard issue == nil else { return }
            perceptionReady = true
            startupTask?.cancel()
            pauseTask?.cancel()
            startupTask = nil
        }
        let frameID = batch.frameID
        frameWatchdogTask = Task { [weak self, clock, config] in
            guard batch.timestamp.isFinite else { return }
            let expiry = batch.timestamp + config.maximumObservationAge + 0.001
            await clock.sleep(seconds: max(0, expiry - clock.now))
            guard !Task.isCancelled, let self, self.generation == token, self.latestFrame == frameID,
                   self.clock.now - batch.timestamp > config.maximumObservationAge else { return }
            await self.perceptionUnavailable(self.perceptionIssue ?? .staleFrame, generation: token)
        }
        if let issue {
            await perceptionUnavailable(issue, generation: token)
            return
        }
        poseDeadline = nil
        poseTask?.cancel()
        poseTask = nil
        // Alignment owns its pre/post-stop waits. Keep healthy frame association
        // flowing during those acknowledgements without launching other motion.
        if confirmationTask != nil, !(state == .aligning && alignmentTask != nil) {
            guard await confirmStop(generation: token) else { return }
            scanning = false
            lastGoal = nil
            // A newer completed observation may arrive while stop confirmation
            // suspends this processor. Let the next loop iteration evaluate it
            // instead of starting another outage from the superseded frame.
            if let newer = pendingFrame, newer.frameID.generation == batch.frameID.generation,
               newer.frameID.sequence > batch.frameID.sequence { return }
            guard clock.now - batch.timestamp <= config.maximumObservationAge else {
                await perceptionUnavailable(.staleFrame, generation: token)
                return
            }
        }
        // A translating recovery invalidates the old target geometry, including
        // queued inference completed before motor stop. Reuse the full source
        // generation/sequence/timestamp and 300ms settled-stop admission gate.
        if awaitingDepthReobservation {
            guard matchedFrameClearsNewestStop(batch) else { return }
            awaitingDepthReobservation = false
        }
        switch state {
        case .recoveringReadyPath:
            // The bounded controller owns probe/return/search. Perception still
            // updates health and the latest target, but cannot launch competing
            // alignment or interpret a search turn as permanent target loss.
            if let locked {
                let evaluated = tracker.continueTrackEvaluated(batch.people, previous: locked,
                    predictedPosition: locked.position, now: now, frameID: batch.frameID)
                currentAssociation = (token, batch.frameID, evaluated.decision, evaluated.evaluation)
                if case .matched(let selected) = evaluated.decision { self.locked = selected }
            }
        case .searching:
            let evaluated = tracker.selectInitialEvaluated(batch.people, now: now, frameID: batch.frameID)
            associationAvailable = true
            emitAssociation(evaluated.evaluation, batch: batch, now: now)
            if let selected = evaluated.decision {
                if scanning {
                    (motion as? any FollowMeContextualMotion)?.inhibitScanContinuation(origin: .detection)
                    movementTask?.cancel()
                }
                locked = selected
                adoptMemory(selected, batch: batch, association: .initial)
                // A sighting is provisional until readiness completes. A direct
                // following goal still requires depth before translating.
                let issue = config.departureRangeIncrease > 0 ? frameIssue(batch, now: now)
                    : FollowFrameHealth.issue(batch, now: now, configuration: config)
                if let issue {
                    state = .aligning
                    await perceptionUnavailable(issue, generation: token)
                    return
                }
                if config.departureRangeIncrease > 0 {
                    state = .aligning
                    align(generation: token)
                } else {
                    if scanning { guard await confirmStop(generation: token, origin: .detection) else { return }; scanning = false }
                    await follow(selected, rover: batch.pose!.position, generation: token)
                }
            } else if !scanning { scan(generation: token) }
        case .aligning, .waitingForClearance, .signalingReady, .waitingForMovement:
            guard let locked else { return }
            let evaluated = evaluatedPending?.association ?? tracker.continueTrackEvaluated(
                batch.people, previous: locked, predictedPosition: locked.position, now: now, frameID: batch.frameID)
            currentAssociation = (token, batch.frameID, evaluated.decision, evaluated.evaluation)
            associationAvailable = true
            emitAssociation(evaluated.evaluation, batch: batch, now: now)
            switch evaluated.decision {
            case .matched(let selected):
                if let hold = acquisitionHold {
                    if clock.now >= hold.deadline { await loseTarget(generation: token); return }
                    guard selected.bodyVerified, matchedFrameClearsNewestStop(batch) else { return }
                    if clock.now >= hold.deadline { await loseTarget(generation: token); return }
                    clearAcquisitionHold()
                    log("follow_acquisition.hold_resumed")
                }
                self.locked = selected
                adoptMemory(selected, batch: batch, association: .continued)
                if state == .waitingForMovement {
                    if let baseline = departureBaseline,
                       batch.pose!.position.distance(to: selected.position) >= baseline + config.departureRangeIncrease {
                        guard matchedFrameClearsNewestStop(batch) else { return }
                        await follow(selected, rover: batch.pose!.position, generation: token,
                            confirmedStop: newestStopFence?.identity)
                    }
                } else if state == .signalingReady {
                    if !readySignalSucceeded,
                       batch.pose!.position.distance(to: selected.position) < config.minimumHoldDistance + 0.12 {
                        _ = await finish(.failed("Person too close during ready signal. Step back and start following again."))
                        return
                    }
                    if readySignalSucceeded, let confirmed = readySignalConfirmedAfter,
                       matchedFrameClearsNewestStop(batch),
                       confirmed.generation == batch.frameID.generation, batch.frameID.sequence > confirmed.sequence,
                       let completed = readySignalCompletionTime,
                       batch.timestamp >= completed {
                        guard await restoreRecovery(selected, phase: .waitingForMovement) else { return }
                        departureBaseline = batch.pose!.position.distance(to: selected.position)
                        state = .waitingForMovement
                    }
                } else if alignmentTask == nil {
                    guard self.matchedFrameClearsNewestStop(batch) else { return }
                    if let completed = alignmentCompletionTime, batch.timestamp < completed { return }
                    let heading = atan2(selected.position.y - batch.pose!.position.y,
                                        selected.position.x - batch.pose!.position.x)
                    if let confirmed = alignmentConfirmedAfter,
                       confirmed.generation == batch.frameID.generation, batch.frameID.sequence > confirmed.sequence,
                       abs(normalizeAngle(heading - batch.pose!.yaw)) <= config.alignmentAngularTolerance {
                        if recoveryEpisode != nil, !recoveryFrameEligible(selected, batch: batch) { return }
                        if !readySignalAttempted {
                            await signalReady(generation: token)
                        } else if readySignalSucceeded {
                            if departureBaseline == nil {
                                guard let confirmed = readySignalConfirmedAfter,
                                      confirmed.generation == batch.frameID.generation,
                                       batch.frameID.sequence > confirmed.sequence,
                                       matchedFrameClearsNewestStop(batch),
                                      let completed = readySignalCompletionTime, batch.timestamp >= completed else { return }
                            }
                            guard await restoreRecovery(selected, phase: .waitingForMovement) else { return }
                            if departureBaseline == nil {
                                // The one move finished, but loss preceded its post-stop baseline.
                                departureBaseline = batch.pose!.position.distance(to: selected.position)
                            }
                            state = .waitingForMovement
                        } else {
                            _ = await finish(.failed("Ready signal interrupted. Stop and start following again."))
                        }
                    } else {
                        state = .aligning
                        align(generation: token)
                    }
                }
            case .lost:
                if await holdAcquisitionForVerificationMiss(batch, generation: token) { return }
                await loseTarget(generation: token)
            case .ambiguous: await loseTarget(generation: token)
            }
        case .following, .holdingDistance:
            guard let locked else { return }
            seedTrackIfNeeded(from: locked)
            let gate = trackGate(at: batch.timestamp, now: now)
            let evaluated = tracker.continueTrackEvaluated(batch.people, previous: locked,
                predictedPosition: predictedPosition(at: batch.timestamp) ?? locked.position,
                now: now, frameID: batch.frameID, worldGate: gate.world,
                requireScreenAssociation: gate.screen)
            currentAssociation = (token, batch.frameID, evaluated.decision, evaluated.evaluation)
            associationAvailable = true
            emitAssociation(evaluated.evaluation, batch: batch, now: now)
            switch evaluated.decision {
            case .matched(let selected):
                self.locked = selected
                noteMeasurement(selected)
                adoptMemory(selected, batch: batch, association: .continued)
                await follow(selected, rover: batch.pose!.position, generation: token,
                             lead: trackVelocity(at: batch.timestamp))
            case .lost, .ambiguous:
                if await coast(batch, generation: token, now: now) { return }
                await loseTarget(generation: token)
            }
        case .reacquiring:
            guard let episode = recoveryEpisode, now < episode.deadline, let point = episode.anchor?.position else { return }
            if recoveryStopBoundary != nil, let locked {
                let evaluated = tracker.continueTrackEvaluated(batch.people, previous: locked,
                    predictedPosition: locked.position, now: now, frameID: batch.frameID)
                currentAssociation = (token, batch.frameID, evaluated.decision, evaluated.evaluation)
                associationAvailable = true
                emitAssociation(evaluated.evaluation, batch: batch, now: now)
                if case .matched(let selected) = evaluated.decision {
                    self.locked = selected
                    if departurePending, hasRetainedReadyBaseline,
                       recoveryFrameEligible(selected, batch: batch), matchedFrameClearsNewestStop(batch) {
                        // Readiness already completed. A fresh post-stop match
                        // resumes the original departure gate, not initial alignment.
                        if batch.pose!.position.distance(to: selected.position) >= departureBaseline! + config.departureRangeIncrease {
                            await follow(selected, rover: batch.pose!.position, generation: token,
                                confirmedStop: newestStopFence?.identity)
                        } else {
                            _ = await restoreRecovery(selected, phase: .waitingForMovement)
                        }
                    } else if !departurePending, recoveryFrameEligible(selected, batch: batch) {
                        await follow(selected, rover: batch.pose!.position, generation: token)
                    }
                } else {
                    recoveryStopBoundary = nil
                    scan(generation: token)
                }
                return
            }
            let deadline = episode.deadline
            let evaluated = tracker.reacquireEvaluated(batch.people, lastPosition: point, now: now,
                expectedGeneration: episode.anchor?.frameID.generation, frameID: batch.frameID)
            currentAssociation = (token, batch.frameID, evaluated.decision, evaluated.evaluation)
            associationAvailable = true
            emitAssociation(evaluated.evaluation, batch: batch, now: now)
            if case .matched(let selected) = evaluated.decision {
                (motion as? any FollowMeContextualMotion)?.inhibitScanContinuation(origin: .detection)
                movementTask?.cancel()
                guard await confirmStop(generation: token, origin: .detection) else { return }
                scanning = false
                guard generation == token, recoveryEpisode?.id == episode.id, state == .reacquiring,
                      let stoppedFrame = pendingFrame?.frameID ?? latestFrame else { return }
                let stopBoundary = RecoveryStopBoundary(frameID: stoppedFrame, time: clock.now)
                if stopBoundary.time >= deadline {
                    _ = await expireRecovery(generation: token, at: stopBoundary.time)
                    return
                }
                recoveryStopBoundary = stopBoundary
                locked = selected
                recoveryHasProvisionalLock = true
                emitRecovery("retained", reason: "provisional_detection_post_stop", stop: "confirmed")
                if departurePending && !hasRetainedReadyBaseline {
                    state = .aligning
                    align(generation: token)
                }
            } else if !scanning { scan(generation: token) }
        default: break
        }
    }

    private func follow(_ selected: FollowPersonObservation, rover: Vec2, generation token: UInt64,
                        confirmedStop: UUID? = nil, lead: Vec2? = nil) async {
        guard generation == token else { return }
        if let confirmedStop {
            let stoppedRecovery = state == .reacquiring && recoveryStopBoundary != nil
            let stoppedDeparture = state == .waitingForMovement && hasRetainedReadyBaseline && recoveryEpisode == nil
            guard stoppedRecovery || stoppedDeparture, !scanning, !stopBlocked, stopTask == nil, confirmationTask == nil,
                  newestStopFence?.identity == confirmedStop, let batch = latestBatch, poseDeadline == nil,
                  batch.frameID == selected.frameID, pendingFrame == nil || pendingFrame?.frameID == batch.frameID,
                  matchedFrameClearsNewestStop(batch) else { return }
            if stoppedRecovery, !recoveryFrameEligible(selected, batch: batch) { return }
        }
        let distance = rover.distance(to: selected.position)
        guard distance.isFinite else { await loseTarget(generation: token); return }
        guard distance > config.maximumHoldDistance else {
            if lastGoal != nil { _ = await confirmStop(generation: token) }
            guard generation == token else { return }
            if recoveryExpired { _ = await expireRecovery(generation: token); return }
            guard await restoreRecovery(selected, phase: .holdingDistance) else { return }
            departurePending = false
            state = .holdingDistance
            lastGoal = nil
            return
        }
        let aim = leadPoint(selected.position, velocity: lead)
        guard let goal = tracker.standOffGoal(rover: rover, person: aim) else { return }
        if recoveryEpisode == nil { state = .following }
        let minimumChange = config.continuousGoals ? config.continuousMinimumGoalChange : config.minimumGoalChange
        let maximumRate = config.continuousGoals ? config.continuousMaximumGoalUpdatesPerSecond
            : config.maximumGoalUpdatesPerSecond
        if let lastGoal, let lastGoalTime {
            guard goal.distance(to: lastGoal) >= minimumChange,
                  clock.now - lastGoalTime >= 1 / maximumRate else { return }
        }
        if config.continuousGoals, confirmedStop == nil, recoveryEpisode == nil, !scanning,
           activeRequest?.purpose == .followGoal,
           (motion as? any FollowMeContextualMotion)?.updateFollowGoal(goal) == true {
            // The follow drive is still running: steer it, don't stop it.
            lastGoal = goal
            lastGoalTime = clock.now
            return
        }
        if confirmedStop == nil {
            guard await confirmStop(generation: token), generation == token else { return }
        }
        if recoveryExpired { _ = await expireRecovery(generation: token); return }
        guard latestFrame == selected.frameID,
              clock.now - selected.timestamp >= 0,
              clock.now - selected.timestamp <= config.maximumObservationAge,
              poseDeadline == nil else {
            await perceptionUnavailable(perceptionIssue ?? .staleFrame, generation: token)
            return
        }
        guard await restoreRecovery(selected, phase: .following) else { return }
        departurePending = false
        state = .following
        lastGoal = goal
        lastGoalTime = clock.now
        launchMovement(generation: token, purpose: .followGoal) { [motion] context in
            await motion.performContextual(.following(goal, self.config.minimumHoldDistance), context: context)
        }
    }

    // MARK: Track, coasting and tolerant continuation

    private func resetTrack() {
        track = nil
        lastPositionAt = nil
        lastMeasurementAt = nil
        coasting = false
    }

    private func seedTrackIfNeeded(from person: FollowPersonObservation) {
        guard config.coastSeconds > 0, track == nil else { return }
        track = TargetKalmanFilter(position: person.position, at: person.timestamp)
        lastPositionAt = person.timestamp
        lastMeasurementAt = person.timestamp
        coasting = false
    }

    private func predictedPosition(at time: TimeInterval) -> Vec2? {
        track?.estimate(at: time).position
    }

    private func trackVelocity(at time: TimeInterval) -> Vec2? {
        guard config.continuousGoals, let track else { return nil }
        return track.estimate(at: time).velocity
    }

    /// Association gate for this frame. A track without a recent full position widens its world
    /// gate with its uncertainty and drops the box-overlap test against a stale box.
    private func trackGate(at time: TimeInterval, now: TimeInterval) -> (world: Double?, screen: Bool) {
        guard let track, let last = lastPositionAt else { return (nil, true) }
        let silent = now - last
        guard silent > 0.35 else { return (nil, true) }
        let sigma = track.estimate(at: time).positionSigma
        return (min(2.0, max(config.maximumWorldDistance, 3 * sigma)), false)
    }

    private func leadPoint(_ position: Vec2, velocity: Vec2?) -> Vec2 {
        guard let velocity else { return position }
        var dx = velocity.x * config.goalLeadSeconds, dy = velocity.y * config.goalLeadSeconds
        let length = hypot(dx, dy)
        if length > config.maximumGoalLead, length > 0 {
            dx *= config.maximumGoalLead / length
            dy *= config.maximumGoalLead / length
        }
        return Vec2(position.x + dx, position.y + dy)
    }

    private func noteMeasurement(_ person: FollowPersonObservation) {
        guard config.coastSeconds > 0, var filter = track else { return }
        filter.predict(to: person.timestamp)
        let range = person.pose.position.distance(to: person.position)
        filter.update(position: person.position, sigma: 0.06 + 0.03 * range)
        track = filter
        let gap = person.timestamp - (lastPositionAt ?? person.timestamp)
        lastPositionAt = person.timestamp
        lastMeasurementAt = person.timestamp
        if coasting {
            coasting = false
            log("follow_coast.recovered", extra: ["gap_s": String(format: "%.2f", gap)])
        }
    }

    /// The target was not matched this frame. Within the coast window, keep the estimate alive
    /// (with a bearing-only update when a partly visible person lines up) and keep steering at
    /// the predicted position. Returns false when coasting is off or has run out, so the caller
    /// stops and searches exactly as before.
    private func coast(_ batch: FollowFrameBatch, generation token: UInt64, now: TimeInterval) async -> Bool {
        guard config.coastSeconds > 0, var filter = track, let locked, let lastMeasurementAt,
              let lastPositionAt, let pose = batch.pose else { return false }
        if config.tolerantContinuation, now - lastPositionAt <= config.maximumBearingOnlySeconds,
           let partial = bearingMatch(batch, filter: filter, pose: pose) {
            filter.predict(to: partial.timestamp)
            filter.update(bearing: partial.bearing, from: pose.position, sigma: config.bearingSigma)
            track = filter
            self.lastMeasurementAt = partial.timestamp
        }
        let silent = now - (self.lastMeasurementAt ?? lastMeasurementAt)
        let bearingOnlyExpired = config.tolerantContinuation && (self.lastMeasurementAt ?? lastMeasurementAt) > lastPositionAt
            && now - lastPositionAt > config.maximumBearingOnlySeconds
        guard silent <= config.coastSeconds, !bearingOnlyExpired else {
            log("follow_coast.expired", extra: ["silent_s": String(format: "%.2f", silent)])
            return false
        }
        if !coasting {
            coasting = true
            log("follow_coast.started", extra: ["last_position_age_s": String(format: "%.2f", now - lastPositionAt)])
        }
        let estimate = (track ?? filter).estimate(at: batch.timestamp)
        let synthetic = FollowPersonObservation(frameID: batch.frameID, timestamp: batch.timestamp,
            confidence: locked.confidence, boundingBox: locked.boundingBox, position: estimate.position,
            pose: pose, rawPersonID: nil, bodyVerified: locked.bodyVerified)
        await follow(synthetic, rover: pose.position, generation: token,
                     lead: config.continuousGoals ? estimate.velocity : nil)
        return true
    }

    private func bearingMatch(_ batch: FollowFrameBatch, filter: TargetKalmanFilter,
                              pose: Pose2D) -> FollowBearingObservation? {
        var ahead = filter
        ahead.predict(to: batch.timestamp)
        let gated = batch.partialPeople.filter { partial in
            partial.confidence >= config.minimumPartialConfidence && partial.frameID == batch.frameID
                && abs(ahead.bearingInnovation(partial.bearing, from: pose.position) ?? .infinity) <= config.maximumBearingInnovation
                && ahead.gate(bearing: partial.bearing, from: pose.position, sigma: config.bearingSigma) <= 6.63
        }
        return gated.count == 1 ? gated[0] : nil   // two candidates in the gate: do not guess
    }

    private func align(generation token: UInt64) {
        guard alignmentTask == nil else { return }
        alignmentSerial &+= 1
        let serial = alignmentSerial
        let capturedPhase = String(describing: state)
        let capturedStopOrigin: FollowMotionStopOrigin = scanning ? .detection : .independent
        alignmentConfirmedAfter = nil
        alignmentTask = Task { [weak self] in
            guard let self else { return }
            defer {
                if self.generation == token, self.alignmentSerial == serial { self.alignmentTask = nil }
            }
            guard await self.confirmStop(generation: token, origin: capturedStopOrigin), self.alignmentSerial == serial,
                  self.state == .aligning else { return }
            self.scanning = false
            guard let snapshot = await self.awaitAlignmentMatch(generation: token, serial: serial) else { return }
            guard self.commitAlignmentSnapshot(snapshot, generation: token) else { return }
            guard self.generation == token, self.alignmentSerial == serial,
                   self.state == .aligning, self.canScan,
                   let pose = snapshot.batch.pose, let selected = snapshot.person
            else { return }
            let heading = atan2(selected.position.y - pose.position.y, selected.position.x - pose.position.x)
            let angle = normalizeAngle(heading - pose.yaw)
            guard angle.isFinite else { return }
            let id = self.operation
            let context = FollowMotionRequestContext(sessionGeneration: token, requestToken: id,
                purpose: .followAlignment, phase: capturedPhase, searchClearance: self.recoveryEpisode == nil ? self.searchClearance : nil,
                allowsAcquisitionRotationWithoutDepth: self.acquisitionRotationIgnoresDepth)
            self.activeRequest = context
            let contextualResult = await self.motion.performContextual(.alignment(angle), context: context,
                recovery: self.recoveryAuthorization(generation: token, operation: id))
            if await self.consumeResult(contextualResult) { return }
            if self.recoveryExpired { _ = await self.expireRecovery(generation: token); return }
            let result = contextualResult.result
            guard self.generation == token, self.operation == id, self.alignmentSerial == serial,
                  self.state == .aligning else { return }
            guard await self.confirmStop(generation: token), self.alignmentSerial == serial,
                  self.state == .aligning, self.canScan else { return }
            guard result == .arrived else { return }
            self.alignmentConfirmedAfter = self.pendingFrame?.frameID ?? self.latestFrame
            self.alignmentCompletionTime = self.newestStopFence?.acknowledgementUptime
            guard let handoff = await self.awaitAlignmentMatch(generation: token, serial: serial) else { return }
            guard self.commitAlignmentSnapshot(handoff, generation: token) else { return }
            self.alignmentTask = nil
            self.evaluatedPendingSnapshot = handoff
            await self.receive(handoff.batch, generation: token)
        }
    }

    private func cancelAlignment() {
        clearAcquisitionHold()
        alignmentSerial &+= 1
        alignmentTask?.cancel()
        alignmentTask = nil
        alignmentConfirmedAfter = nil
        alignmentCompletionTime = nil
        wakeAlignmentSourceWaiter()
    }

    private func clearAcquisitionHold() {
        acquisitionHold = nil
        acquisitionHoldTask?.cancel()
        acquisitionHoldTask = nil
    }

    private func holdAcquisitionForVerificationMiss(_ batch: FollowFrameBatch, generation token: UInt64) async -> Bool {
        guard generation == token, state == .aligning || (state == .waitingForMovement && hasRetainedReadyBaseline),
              locked?.bodyVerified == true,
              config.acquisitionVerificationGraceSeconds.isFinite, config.acquisitionVerificationGraceSeconds > 0,
              batch.people.isEmpty, let decisions = batch.perceptionDiagnostics?.personVerification,
              !decisions.isEmpty, decisions.allSatisfy({ !$0.accepted &&
                  ($0.reason == "invalid_person_box" || $0.reason == "no_matching_body") }) else { return false }
        if let hold = acquisitionHold {
            if clock.now >= hold.deadline { await loseTarget(generation: token) }
            return true
        }
        (motion as? any FollowMeContextualMotion)?.inhibitScanContinuation(origin: .independent)
        cancelAlignment()
        let id = UUID()
        let deadline = min(clock.now + min(0.5, config.acquisitionVerificationGraceSeconds),
            recoveryEpisode?.deadline ?? .infinity)
        acquisitionHold = (id, deadline)
        log("follow_acquisition.hold_started", extra: ["deadline_uptime_s": String(deadline)])
        acquisitionHoldTask = Task { [weak self, clock] in
            await clock.sleep(seconds: max(0, deadline - clock.now))
            guard !Task.isCancelled, let self, self.generation == token,
                  self.acquisitionHold?.id == id,
                  self.state == .aligning || self.state == .waitingForMovement else { return }
            self.log("follow_acquisition.hold_expired")
            await self.loseTarget(generation: token)
        }
        scanning = false
        _ = await confirmStop(generation: token)
        return true
    }

    private var hasRetainedReadyBaseline: Bool {
        readySignalSucceeded && departureBaseline.map { $0.isFinite && $0 >= 0 } == true
    }

    private func wakeAlignmentSourceWaiter() {
        let waiter = alignmentSourceWaiter
        alignmentSourceWaiter = nil
        waiter?.resume()
    }

    private func admissionSnapshot(_ batch: FollowFrameBatch, requiresDepth: Bool = true) -> FollowAdmissionSnapshot {
        if let saved = evaluatedPendingSnapshot, saved.batch.frameID == batch.frameID {
            return .init(batch: batch, previous: locked, pending: saved.pending, tracker: tracker,
                now: clock.now, configuration: config, acceptedAssociation: saved.association, requiresDepth: requiresDepth)
        }
        let accepted = currentAssociation.flatMap { value in
            value.generation == generation && value.frameID == batch.frameID
                ? (decision: value.decision, evaluation: value.evaluation) : nil
        }
        return .init(batch: batch, previous: locked, pending: pendingFrame?.frameID == batch.frameID,
            tracker: tracker, now: clock.now, configuration: config, acceptedAssociation: accepted, requiresDepth: requiresDepth)
    }

    private func commitAlignmentSnapshot(_ snapshot: FollowAdmissionSnapshot, generation token: UInt64) -> Bool {
        guard generation == token, state == .aligning, pendingHealthFrame == nil,
              let person = snapshot.person else { return false }
        if snapshot.pending { evaluatedPendingSnapshot = snapshot }
        latestBatch = snapshot.batch
        latestFrame = snapshot.batch.frameID
        locked = person
        if let association = snapshot.association {
            currentAssociation = (token, snapshot.batch.frameID, association.decision, association.evaluation)
        }
        adoptMemory(person, batch: snapshot.batch, association: .acceptedPendingContinuity)
        return true
    }

    private func awaitAlignmentMatch(generation token: UInt64, serial: UInt64) async -> FollowAdmissionSnapshot? {
        while generation == token, alignmentSerial == serial, state == .aligning, !Task.isCancelled {
            if recoveryExpired { _ = await expireRecovery(generation: token); return nil }
            if let batch = pendingFrame ?? latestBatch, matchedFrameClearsNewestStop(batch) {
                let snapshot = admissionSnapshot(batch, requiresDepth: !acquisitionRotationIgnoresDepth)
                if snapshot.rejection == nil, case .matched = snapshot.association?.decision { return snapshot }
            }
            let timer: Task<Void, Never>?
            if let fence = newestStopFence, let uptime = (motion as? any FollowMeSourceClockMotion)?.sourceUptime,
               uptime < fence.acknowledgementUptime + 0.300 {
                timer = Task { [weak self, clock] in
                    await clock.sleep(seconds: fence.acknowledgementUptime + 0.300 - uptime)
                    guard !Task.isCancelled, let self, self.generation == token, self.alignmentSerial == serial else { return }
                    self.wakeAlignmentSourceWaiter()
                }
            } else { timer = nil }
            await withCheckedContinuation { alignmentSourceWaiter = $0 }
            timer?.cancel()
        }
        return nil
    }

    private func signalReady(generation token: UInt64) async {
        guard generation == token, canScan, !readySignalAttempted, pendingAdmission == nil else { return }
        guard let pose = latestBatch?.pose, let locked,
              pose.position.distance(to: locked.position) >= config.minimumHoldDistance + 0.12 else {
            if let locked, !(await restoreRecovery(locked, phase: .waitingForClearance)) { return }
            state = .waitingForClearance
            return
        }
        operation &+= 1
        let id = operation
        let admissionToken = FollowReadyAdmissionToken(generation: token, operation: id)
        pendingAdmission = admissionToken
        let context = FollowMotionRequestContext(sessionGeneration: token, requestToken: id,
            purpose: .followReady, phase: String(describing: state))
        activeRequest = context
        emitReadiness("follow_ready.admission_pending", reason: "eligible_preflight")
        let admission = FollowReadyAdmission(allowsPathRecovery: motion is NavigationFollowMeMotion && readyPathRecoveryAttempts < 2,
            validating: { [weak self] sample, readUptime in
            guard let self else { return .deferred(.ownership) }
            // Capture the actual newest transaction even when an earlier frame
            // has already stopped/replaced this request. Facts never grant authority.
            let newestSnapshot = (self.pendingFrame ?? self.latestBatch).map { self.admissionSnapshot($0) }
            FollowReadyAdmissionScope.current?.observationSnapshot = newestSnapshot
            FollowReadyAdmissionScope.current?.observationValidUntil = newestSnapshot.map {
                $0.batch.timestamp + self.config.maximumObservationAge
            }
            @MainActor func reject(_ reason: FollowReadyDeferral, _ condition: String,
                                   staleObservation: Bool = false) -> FollowReadyAdmissionDecision {
                FollowReadyAdmissionScope.current?.rejectionCondition = condition
                // Record the decision now, before asynchronous stop/result handling
                // can replace the observation or fence this operation's delivery.
                self.emitReadiness("follow_ready.admission_rejected", reason: reason.rawValue)
                return staleObservation ? .staleObservation : .deferred(reason)
            }
            guard self.generation == token else { return reject(.ownership, "session_generation_changed") }
            guard self.operation == id else { return reject(.ownership, "operation_replaced") }
            guard self.pendingAdmission == admissionToken else { return reject(.ownership, "admission_token_changed") }
            guard !self.recoveryExpired else { return reject(.ownership, "recovery_deadline_expired") }
            guard !Task.isCancelled else { return reject(.ownership, "task_cancelled") }
            guard !self.readySignalAttempted else { return reject(.ownership, "attempt_already_consumed") }
            guard self.isActive else { return reject(.ownership, "session_inactive") }
            guard !self.stopBlocked else { return reject(.ownership, "stop_blocked") }
            guard self.stopTask == nil else { return reject(.ownership, "stop_pending") }
            guard self.confirmationTask == nil else { return reject(.ownership, "stop_confirmation_pending") }
            guard self.perceptionReady else { return reject(.observation, "perception_not_ready") }
            guard self.poseDeadline == nil else { return reject(.observation, "outage_recovery_pending") }
            if let issue = self.perceptionIssue {
                return reject(.observation, "perception_" + issue.rawValue,
                    staleObservation: issue == .staleFrame && self.isStoppedReadyRefresh)
            }
            guard let snapshot = newestSnapshot else { return reject(.observation, "frame_missing") }
            let batch = snapshot.batch
            FollowReadyAdmissionScope.current?.rejectionCondition = snapshot.rejection
            if let rejection = snapshot.rejection {
                return reject(.observation, rejection, staleObservation: self.isStoppedReadyRefresh &&
                    FollowFrameHealth.issue(batch, now: self.clock.now, configuration: self.config) == .staleFrame)
            }
            guard self.pendingHealthFrame == nil else { return reject(.observation, "pending_health_transition") }
            guard self.matchedFrameClearsNewestStop(batch) else { return reject(.observation, "post_stop_match_required") }
            guard let person = snapshot.person, let batchPose = batch.pose else { return reject(.observation, "matched_geometry_missing") }
            if self.recoveryEpisode != nil {
                guard self.recoveryFrameEligible(person, batch: batch),
                      let confirmed = self.alignmentConfirmedAfter,
                      batch.frameID.generation == confirmed.generation, batch.frameID.sequence > confirmed.sequence,
                      let completed = self.alignmentCompletionTime, batch.timestamp >= completed else {
                    return reject(.observation, "recovery_post_stop_match_required")
                }
            }
            let pose: Pose2D
            if let sample, sample.source != "legacy_unknown" {
                guard let readUptime else { return reject(.observation, "controller_read_time_missing") }
                if let rejection = sample.rejection(at: readUptime, expectedGeneration: batch.frameID.generation) {
                    return reject(.observation, "controller_" + rejection)
                }
                guard let controllerPose = sample.pose else { return reject(.observation, "controller_pose_missing") }
                pose = controllerPose
            } else {
                // Legacy call/pose-only providers retain explicitly unknown provenance.
                pose = batchPose
            }
            let heading = atan2(person.position.y - pose.position.y, person.position.x - pose.position.x)
            guard abs(normalizeAngle(heading - pose.yaw)) <= self.config.alignmentAngularTolerance else { return reject(.heading, "heading_outside_tolerance") }
            guard pose.position.distance(to: person.position) >= self.config.minimumHoldDistance + 0.12 else { return reject(.clearance, "range_below_clearance") }
            guard !self.recoveryExpired else { return reject(.ownership, "recovery_deadline_expired") }
            if snapshot.pending {
                // Publish the accepted observation atomically. The processor still
                // owns watchdogs and state handling, reusing this exact association.
                self.evaluatedPendingSnapshot = snapshot
                if let association = snapshot.association {
                    self.currentAssociation = (token, batch.frameID, association.decision, association.evaluation)
                }
                self.latestBatch = batch
                self.latestFrame = batch.frameID
                self.locked = person
                self.adoptMemory(person, batch: batch, association: .acceptedPendingContinuity)
            }
            if FollowReadyAdmissionScope.current?.boundary == .recoveryProbeSend {
                // The stopped refresh has finished. This check neither consumes
                // another recovery attempt nor starts the ready signal.
                guard self.state == .recoveringReadyPath else { return reject(.ownership, "recovery_not_active") }
                return .accepted
            }
            if FollowReadyAdmissionScope.current?.boundary == .recoveryProbe {
                guard self.readyPathRecoveryAttempts < 2 else { return reject(.ownership, "path_recovery_exhausted") }
                self.readyPathRecoveryAttempts += 1
                self.state = .recoveringReadyPath
                self.emitReadiness("follow_ready.recovery_started", reason: "mapped_path_live_depth_probe")
                return .accepted
            }
            self.pendingAdmission = nil
            self.readySignalAttempted = true
            self.searchClearance?.revoke()
            self.state = .signalingReady
            self.emitReadiness("follow_ready.admission_authorized", reason: "first_send_authorized")
            self.diagnosticStopState = "not_stopped"
            return .accepted
        })
        activeReadyAdmission = admission
        movementTask = Task { [weak self, motion] in
            let contextualResult = await motion.performContextual(.ready, context: context, admission: admission,
                recovery: self?.recoveryAuthorization(generation: token, operation: id))
            guard let self else { return }
            if self.activeReadyAdmission === admission { self.activeReadyAdmission = nil }
            if self.pendingAdmission == admissionToken { self.pendingAdmission = nil }
            if await self.consumeResult(contextualResult) { return }
            if self.recoveryExpired { _ = await self.expireRecovery(generation: token); return }
            guard self.generation == token, self.operation == id else { return }
            if contextualResult.result == .reobservationRequired, self.state == .recoveringReadyPath {
                self.emitReadiness("follow_ready.recovery_search_completed", reason: "clear_heading_reacquire_person")
                self.movementTask = nil
                await self.loseTarget(generation: token)
                return
            }
            if contextualResult.result == .cancelled, self.state == .recoveringReadyPath,
               admission.deferred == nil || admission.deferred == .observation || admission.deferred == .ownership {
                // Admission may observe a queued hard fault before its frame
                // processor. Never turn that recovery cancellation into a
                // normal pre-motion observation deferral that can resume later.
                _ = await self.finish(.failed("Floor check interrupted. Wait for stable perception and restart following."))
                return
            }
            if case .notStarted(let deferred) = contextualResult.outcome {
                self.diagnosticStopState = "confirmed"
                if deferred == .clearance, contextualResult.stopOutcome == .confirmed,
                   let snapshot = admission.observationSnapshot, let person = snapshot.person,
                   let batch = self.pendingFrame ?? self.latestBatch,
                   batch.frameID == snapshot.batch.frameID,
                   self.recoveryFrameEligible(person, batch: batch) {
                    if snapshot.pending {
                        self.evaluatedPendingSnapshot = snapshot
                        self.latestBatch = batch
                        self.latestFrame = batch.frameID
                        self.locked = person
                    }
                    guard await self.restoreRecovery(person, phase: .waitingForClearance) else { return }
                }
                self.state = .waitingForClearance
                self.emitReadiness("follow_ready.admission_deferred", reason: deferred.rawValue, admission: admission)
                if deferred == .heading { self.state = .aligning; self.align(generation: token) }
                if deferred == .observation, self.pendingFrame == nil, !self.canScan {
                    await self.perceptionUnavailable(self.perceptionIssue ?? .staleFrame, generation: token)
                }
                return
            }
            let result = contextualResult.result
            guard self.generation == token, self.operation == id,
                  self.state == .signalingReady else { return }
            guard result == .arrived else {
                _ = await self.finish(.failed("Ready signal could not complete safely. Stop and start following again."))
                return
            }
            guard await self.confirmStop(generation: token), self.state == .signalingReady,
                  self.canScan else { return }
            self.readySignalSucceeded = true
            self.readySignalConfirmedAfter = self.pendingFrame?.frameID ?? self.latestFrame
            self.readySignalCompletionTime = self.clock.now
            self.emitReadiness("follow_ready.completed", reason: "final_stop_confirmed_new_frame_required")
        }
    }

    private func confirmStop(generation token: UInt64, origin: FollowMotionStopOrigin = .independent) async -> Bool {
        guard generation == token, !stopBlocked, stopTask == nil else { return false }
        operation &+= 1
        movementTask?.cancel()
        movementTask = nil
        if let confirmationTask {
            let id = confirmationID
            let confirmed = await confirmationTask.value
            if confirmed, confirmationID == id { self.confirmationTask = nil }
            if confirmed, generation == token, recoveryExpired {
                _ = await expireRecovery(generation: token)
                return false
            }
            return confirmed && generation == token && !stopBlocked && stopTask == nil
        }
        confirmationID &+= 1
        let id = confirmationID
        let task = Task { [weak self, motion] in
            do {
                let capture = FollowMotionStopReceiptCapture()
                capture.onSourceFence = { [weak self] fence in
                    guard let self, self.generation == token, self.confirmationID == id else { return }
                    self.captureStopFence(fence)
                }
                try await FollowMotionTaskScope.$stopReceiptCapture.withValue(capture) {
                    try await FollowMotionTaskScope.$stopOrigin.withValue(origin) { try await motion.stopAndConfirm() }
                }
                if let self, self.generation == token, self.confirmationID == id {
                    if capture.sourceFence == nil { self.newestStopFence = nil }
                }
                return true
            }
            catch { return false }
        }
        confirmationTask = task
        diagnosticStopState = "pending"
        let recoveryStopPayload = recoveryEpisode.map {
            FollowReacquisitionDiagnostics.payload($0, now: clock.now, stop: "pending",
                centerSource: recoveryCenterSource, centerReadUptime: recoveryCenterReadUptime)
        }
        let recoveryStopOperation = operation
        let recoveryStopPhase = String(describing: state)
        let recoveryStopStarted = recoveryStopPayload == nil ? nil : clock.now
        let confirmed = await task.value
        if var payload = recoveryStopPayload {
            payload["stop_outcome"] = .string(confirmed ? "confirmed" : "failed")
            payload["stop_origin"] = .string(origin.rawValue)
            payload["confirmation_id"] = .number(Double(id))
            payload["stop_host_duration_s"] = FollowReacquisitionDiagnostics.number(recoveryStopStarted.map { clock.now - $0 })
            payload["stop_duration_clock"] = .string("system_uptime")
            payload["budget_snapshot_boundary"] = .string("stop_request")
            failureEmitter.emit(.init(event: "follow_recovery.stop_response",
                context: .init(sessionGeneration: token, operationID: recoveryStopOperation,
                    phase: recoveryStopPhase, stale: generation != token || confirmationID != id,
                    reason: confirmed ? "stop_acknowledged" : "stop_not_confirmed"), payload: payload))
        }
        diagnosticStopState = confirmed ? "confirmed" : "failed"
        if confirmationID == id { confirmationTask = nil }
        if !confirmed {
            generation &+= 1
            operation &+= 1
            stopBlocked = true
            state = .failed("Rover stop could not be confirmed.")
            framesTask?.cancel()
            frameProcessorTask?.cancel()
            frameProcessorTask = nil
            pendingFrame = nil
            pendingHealthFrame = nil
            evaluatedPendingSnapshot = nil
            currentAssociation = nil
            safetyTask?.cancel()
            movementTask?.cancel()
            deadlineTask?.cancel()
            poseTask?.cancel()
            frameWatchdogTask?.cancel()
            startupTask?.cancel()
            pauseTask?.cancel()
            cancelAlignment()
            return false
        }
        if generation == token, recoveryExpired {
            _ = await expireRecovery(generation: token)
            return false
        }
        return generation == token && !stopBlocked && stopTask == nil
    }

    private func matchedFrameClearsNewestStop(_ batch: FollowFrameBatch) -> Bool {
        guard pendingHealthFrame == nil, let fence = newestStopFence,
              let uptime = (motion as? any FollowMeSourceClockMotion)?.sourceUptime,
              uptime.isFinite, fence.acknowledgementUptime.isFinite,
              uptime >= fence.acknowledgementUptime + 0.300,
              batch.frameID.generation == fence.sourceGeneration,
              batch.frameID.sequence > (fence.highestSequence ?? 0),
              batch.timestamp > fence.acknowledgementUptime,
              batch.timestamp > (fence.highestSourceTimestamp ?? -.infinity),
              frameIssue(batch, now: uptime) == nil else { return false }
        return true
    }

    private func captureStopFence(_ fence: FollowTurnStopFence) {
        let ingress = ingestedSource.fence(at: fence.acknowledgementUptime,
            operationGeneration: fence.operationGeneration, context: fence.context)
        let sameGeneration = ingress.sourceGeneration == fence.sourceGeneration
        newestStopFence = .init(identity: fence.identity, operationGeneration: fence.operationGeneration,
            context: fence.context, acknowledgementUptime: fence.acknowledgementUptime,
            sourceGeneration: fence.sourceGeneration,
            highestSequence: max(fence.highestSequence ?? 0, sameGeneration ? ingress.highestSequence ?? 0 : 0),
            highestSourceTimestamp: max(fence.highestSourceTimestamp ?? -.infinity,
                sameGeneration ? ingress.highestSourceTimestamp ?? -.infinity : -.infinity))
        wakeAlignmentSourceWaiter()
    }

    private func launchMovement(generation token: UInt64, purpose: FollowMotionPurpose,
                                scanUsed: Double? = nil, scanRemaining: Double? = nil, scanStep: Double? = nil,
                                action: @escaping @MainActor (FollowMotionRequestContext) async -> FollowMotionResult) {
        if purpose == .followGoal || purpose == .followReady { searchClearance?.revoke() }
        operation &+= 1
        let id = operation
        let context = FollowMotionRequestContext(sessionGeneration: token, requestToken: id, purpose: purpose,
            phase: String(describing: state), scanUsed: scanUsed, scanRemaining: scanRemaining,
            scanResponseSeed: purpose == .followScan && scanResponseSeed?.sourceGeneration == latestFrame?.generation ? scanResponseSeed : nil,
            isSearchSweep: purpose == .followScan && state == .searching,
            searchClearance: purpose == .followScan && state == .searching ? searchClearance : nil,
            allowsAcquisitionRotationWithoutDepth: (purpose == .followScan || purpose == .followAlignment) && acquisitionRotationIgnoresDepth)
        activeRequest = context
        movementTask = Task { [weak self] in
            let result = await action(context)
            guard let self else { return }
            // Completion accounting belongs to the captured attempt, even when
            // a perception stop has since replaced its motor owner. It never
            // authorizes motion or applies across a new follow session.
            if case .scanDeferred(let reason) = result.outcome {
                if self.generation == token, self.state == .searching {
                    self.log("follow_search.dispatch_deferred", extra: ["reason": reason.rawValue,
                        "scan_attempts": String(self.scanAttempts), "scan_interruptions": String(self.scanInterruptions)])
                }
            } else if self.generation == token, self.state == .searching, context.isSearchSweep,
               result.context.request == context, let scanStep {
                let stopped = result.stopOutcome == .confirmed
                let legacyArrival = result.context.controllerOperationID == nil && result.stopOutcome == .unknown
                let completed = result.result == .arrived && result.failure == nil && (stopped || legacyArrival)
                let measured: Double? = result.measuredScanResponse.flatMap { learned in
                    guard stopped, result.failure == nil, let frame = self.latestFrame,
                          learned.sourceGeneration == frame.generation, learned.responseRate.isFinite,
                          let signed = result.searchSweepProgress, signed.isFinite,
                          let travel = result.searchSweepTravel, travel.isFinite, travel >= 0,
                          abs(signed) <= travel + 1e-9 else { return nil }
                    let directed = signed * (scanStep < 0 ? -1 : 1)
                    return directed >= 0 ? directed : nil
                }
                self.scanRotation += completed ? max(abs(scanStep), measured ?? 0) : (measured ?? 0)
                self.scanRotation = min(min(2 * .pi, self.config.maximumScanRotation), self.scanRotation)
                if !completed { self.scanInterruptions += 1 }
                self.log(completed ? "follow_search.sweep_completed" : "follow_search.sweep_interrupted", extra: [
                    "scan_used_rad": String(self.scanRotation), "scan_attempts": String(self.scanAttempts),
                    "scan_interruptions": String(self.scanInterruptions),
                    "measured_sweep_progress_rad": measured.map { String($0) } ?? "unknown",
                    "sampled_travel_rad": result.searchSweepTravel.map { String($0) } ?? "unknown",
                    "coverage_source": measured != nil ? "measured_response" : (completed ? "completed_request" : "none")])
            }
            if self.generation == token, self.operation == id, result.result == .arrived,
               result.context.request == context, result.failure == nil, result.stopOutcome == .confirmed,
               let learned = result.measuredScanResponse,
               learned.sourceGeneration == self.latestFrame?.generation, learned.responseRate.isFinite {
                self.retainScanResponse(learned)
            }
            _ = await self.consumeResult(result)
        }
    }

    /// A provisional sighting does not finish acquisition. Search, alignment and
    /// reacquisition turns retain the policy until the ready signal succeeds.
    /// Translation/readiness still require real depth and clearance.
    private var acquisitionRotationIgnoresDepth: Bool {
        !readySignalSucceeded && (state == .searching ||
            (config.departureRangeIncrease > 0 &&
                (state == .aligning || state == .reacquiring || state == .waitingForClearance))) &&
            motion is any FollowMeContextualMotion
    }

    private func frameIssue(_ batch: FollowFrameBatch, now: TimeInterval) -> FollowPerceptionIssue? {
        let issue = FollowFrameHealth.issue(batch, now: now, configuration: config)
        return issue == .depthUnavailable && acquisitionRotationIgnoresDepth ? nil : issue
    }

    private var canScan: Bool {
        guard isActive, !recoveryExpired, !awaitingDepthReobservation, perceptionReady, !stopBlocked, stopTask == nil, confirmationTask == nil,
              pendingHealthFrame == nil,
              perceptionIssue == nil, poseDeadline == nil, let batch = latestBatch,
              batch.pose != nil, batch.depthAvailable || acquisitionRotationIgnoresDepth,
              batch.trackingQuality != .limited, batch.trackingQuality != .unavailable,
              batch.timestamp.isFinite else { return false }
        let age = clock.now - batch.timestamp
        return age >= 0 && age <= config.maximumObservationAge
    }

    private func retainScanResponse(_ learned: FollowScanResponseSeed) {
        let previous = scanResponseSeed?.sourceGeneration == learned.sourceGeneration ? scanResponseSeed?.responseRate ?? 0 : 0
        scanResponseSeed = .init(sourceGeneration: learned.sourceGeneration, responseRate: max(previous, learned.responseRate))
    }

    private func beginScanObservation() {
        guard config.scanObservationSeconds > 0, let latestFrame else { return }
        let gate = ScanObservationGate(until: clock.now + config.scanObservationSeconds, afterFrame: latestFrame)
        scanObservation = gate
        log("follow_search.look_started", extra: ["until_uptime_s": String(gate.until)])
    }

    private func scan(generation token: UInt64) {
        guard generation == token else { return }
        if state == .searching, finishSearchIfAdmissionExpired(generation: token) { return }
        guard generation == token, !scanning, canScan else { return }
        if let gate = scanObservation {
            guard clock.now >= gate.until, let batch = latestBatch, batch.timestamp >= gate.until,
                  batch.frameID.generation == gate.afterFrame.generation,
                  batch.frameID.sequence > gate.afterFrame.sequence else { return }
            scanObservation = nil
            log("follow_search.look_completed")
        }
        if state == .reacquiring { scanRecovery(generation: token); return }
        let limit = min(2 * .pi, config.maximumScanRotation)
        guard config.maximumScanRotation.isFinite, limit.isFinite, limit > 0,
              config.scanIncrement.isFinite, config.scanIncrement > 0,
               config.maximumScanAttempts > 0, config.maximumScanInterruptions > 0,
               config.scanAdmissionFreshnessReserve.isFinite, config.scanAdmissionFreshnessReserve >= 0,
               config.scanAdmissionFreshnessReserve < config.maximumObservationAge,
               config.scanAdmissionTimeout.isFinite, config.scanAdmissionTimeout > 0 else {
            finishSearch("Invalid search configuration.", generation: token)
            return
        }
        if state == .searching && scanRotation >= limit - 0.0001 {
            finishSearch("No person found.", generation: token)
            return
        }
        if scanAttempts >= config.maximumScanAttempts || scanInterruptions >= config.maximumScanInterruptions {
            finishSearch("Search interrupted before the scan completed. Wait for stable perception and restart following.", generation: token)
            return
        }
        if scanDeferredFrame == latestFrame { return }
        if let reason = initialScanDeferral {
            deferInitialScan(reason, generation: token)
            return
        }
        scanning = true
        let angle = state == .searching ? min(config.scanIncrement, limit - scanRotation) : config.scanIncrement
        let dispatch = ScanDispatch()
        log("follow_search.dispatch_scheduled", extra: [
            "scan_attempts": String(scanAttempts), "scan_interruptions": String(scanInterruptions)])
        launchMovement(generation: token, purpose: .followScan, scanUsed: scanRotation + angle,
                       scanRemaining: state == .searching ? max(0, limit - scanRotation - angle) : nil, scanStep: angle) { [motion] context in
            guard self.generation == token, self.operation == context.requestToken,
                  self.state == .searching, !Task.isCancelled else {
                return .deferredScan(.ownershipChanged, request: context)
            }
            if self.finishSearchIfAdmissionExpired(generation: token) {
                return .deferredScan(.insufficientFreshness, request: context)
            }
            if let reason = self.initialScanDeferral {
                self.deferInitialScan(reason, generation: token)
                return .deferredScan(reason, request: context)
            }
            self.clearScanAdmissionWait()
            self.scanAttempts += 1
            dispatch.entered = true
            return await motion.performContextual(.scan(angle), context: context)
        }
        // The scan completion schedules the next increment only after this one has finished.
        let id = operation
        let rotation = movementTask
        Task { [weak self] in
            await rotation?.value
            guard let self, self.generation == token, self.operation == id else { return }
            self.scanning = false
            if dispatch.entered, self.state == .searching, self.config.scanObservationSeconds > 0 {
                self.beginScanObservation()
            }
            if self.state == .searching || self.state == .reacquiring { self.scan(generation: token) }
        }
    }

    private var initialScanDeferral: FollowScanDeferral? {
        if pendingFrame != nil || pendingHealthFrame != nil { return .newerFramePending }
        guard canScan, let batch = latestBatch else { return .perceptionUnavailable }
        let remaining = config.maximumObservationAge - (clock.now - batch.timestamp)
        return remaining >= config.scanAdmissionFreshnessReserve ? nil : .insufficientFreshness
    }

    private func deferInitialScan(_ reason: FollowScanDeferral, generation token: UInt64) {
        guard generation == token, state == .searching else { return }
        if scanAdmissionDeadline == nil {
            let deadline = clock.now + config.scanAdmissionTimeout
            scanAdmissionDeadline = deadline
            scanAdmissionTask = Task { [weak self, clock] in
                await clock.sleep(seconds: max(0, deadline - clock.now))
                guard !Task.isCancelled, let self, self.generation == token,
                      self.scanAdmissionDeadline == deadline, self.state == .searching else { return }
                _ = self.finishSearchIfAdmissionExpired(generation: token)
            }
        }
        guard scanDeferredFrame != latestFrame else { return }
        scanDeferredFrame = latestFrame
        scanAdmissionDeferrals += 1
        log("follow_search.admission_wait", extra: ["reason": reason.rawValue,
            "scan_admission_deferrals": String(scanAdmissionDeferrals),
            "scan_attempts": String(scanAttempts), "scan_interruptions": String(scanInterruptions),
            "admission_deadline_s": scanAdmissionDeadline.map { String($0) } ?? "unknown"])
    }

    private func finishSearchIfAdmissionExpired(generation token: UInt64) -> Bool {
        guard let deadline = scanAdmissionDeadline, clock.now >= deadline else { return false }
        finishSearch("Search could not get a fresh frame in time. Wait for stable perception and restart following.", generation: token)
        return true
    }

    private func clearScanAdmissionWait() {
        scanAdmissionTask?.cancel()
        scanAdmissionTask = nil
        scanAdmissionDeadline = nil
        scanDeferredFrame = nil
    }

    private func finishSearch(_ message: String, generation token: UInt64) {
        Task { [weak self] in
            guard let self, self.generation == token, self.state == .searching else { return }
            _ = await self.finish(.failed(message))
        }
    }

    private func loseTarget(generation token: UInt64) async {
        guard generation == token, state != .reacquiring else { return }
        resetTrack()
        if recoveryEpisode == nil {
            recoveryHasProvisionalLock = false
            recoveryCenterSource = nil
            recoveryCenterReadUptime = nil
            recoveryDiagnosticUnavailable = nil
            recoveryEpisode = FollowReacquisitionEpisode(firstLoss: clock.now, anchor: reliableMemory)
            let episode = recoveryEpisode!
            emitRecovery("started", reason: "first_loss", episode: episode, stop: "pending")
            deadlineTask = Task { [weak self, clock] in
                await clock.sleep(seconds: max(0, episode.deadline - clock.now))
                guard !Task.isCancelled, let self, self.generation == token,
                      self.recoveryEpisode?.id == episode.id else { return }
                _ = await self.expireRecovery(generation: token)
            }
        } else {
            emitRecovery("retained", reason: "loss_before_normal_restoration")
        }
        (motion as? any FollowMeContextualMotion)?.inhibitScanContinuation(origin: .independent)
        movementTask?.cancel()
        scanning = false
        cancelAlignment()
        recoveryStopBoundary = nil
        state = .reacquiring
        guard await confirmStop(generation: token) else { return }
        guard generation == token else { return }
        scanning = false
        lastGoal = nil
        if recoveryExpired { _ = await expireRecovery(generation: token); return }
        scan(generation: token)
    }

    private func expireRecovery(generation token: UInt64, at validationTime: TimeInterval? = nil) async -> Bool {
        guard generation == token, isActive, let episode = recoveryEpisode,
              (validationTime ?? clock.now) >= episode.deadline else { return false }
        emitRecovery("expired", reason: "original_deadline_reached", episode: episode)
        _ = await finish(.failed("Person lost."))
        return true
    }

    private var recoveryExpired: Bool {
        recoveryEpisode.map { clock.now >= $0.deadline } ?? false
    }

    private func recoveryFrameEligible(_ person: FollowPersonObservation, batch: FollowFrameBatch) -> Bool {
        guard let episode = recoveryEpisode, clock.now < episode.deadline,
              let stopBoundary = recoveryStopBoundary,
              batch.frameID.generation == episode.anchor?.frameID.generation,
              batch.frameID.generation == stopBoundary.frameID.generation, batch.frameID.sequence > stopBoundary.frameID.sequence,
              person.frameID == batch.frameID, batch.timestamp >= stopBoundary.time,
              frameIssue(batch, now: clock.now) == nil,
              pendingFrame == nil || pendingFrame?.frameID == batch.frameID,
              !stopBlocked, stopTask == nil, confirmationTask == nil, poseDeadline == nil else { return false }
        return true
    }

    /// No suspension between the final matched-phase gate and paired memory commit.
    private func restoreRecovery(_ person: FollowPersonObservation, phase: FollowMeState) async -> Bool {
        guard let episode = recoveryEpisode else { return true }
        guard let batch = latestBatch else { return false }
        let eligible = recoveryFrameEligible(person, batch: batch)
        let validationTime = clock.now
        if validationTime >= episode.deadline {
            _ = await expireRecovery(generation: generation, at: validationTime)
            return false
        }
        // This one authoritative time validates both health and accepted memory. No
        // suspension or additional clock read precedes the phase/memory transaction.
        guard eligible, FollowFrameHealth.issue(batch, now: validationTime, configuration: config) == nil,
              let quality = batch.trackingQuality,
              let memory = FollowReliableMemory(accepted: person, association: .continued,
                  now: validationTime, trackingQuality: quality) else { return false }
        reliableMemory = memory
        state = phase
        emitRecovery("cleared", reason: "normal_phase_restored", stop: "confirmed",
            extra: ["restored_frame_id": FollowReacquisitionDiagnostics.frame(person.frameID),
                "recovery_active": .bool(false), "deadline_active": .bool(false),
                "provisional_frame_id": .null, "provisional_person_x": .null,
                "provisional_person_z": .null, "provisional_raw_person_id": .null])
        recoveryEpisode = nil
        recoveryStopBoundary = nil
        deadlineTask?.cancel()
        deadlineTask = nil
        recoveryCenterSource = nil
        recoveryCenterReadUptime = nil
        recoveryDiagnosticUnavailable = nil
        return true
    }

    private func recoveryAuthorization(generation token: UInt64, operation id: UInt64) -> FollowRecoveryAuthorization? {
        guard let episode = recoveryEpisode, let anchor = episode.anchor else { return nil }
        return .init(episodeID: episode.id, expectedGeneration: anchor.frameID.generation, deadline: episode.deadline,
            now: { [clock] in clock.now }, canContinue: { [weak self] in
                guard let self, self.generation == token, self.operation == id,
                       self.recoveryEpisode?.id == episode.id, self.canScan || self.isStoppedReadyRefresh,
                      let batch = self.pendingFrame ?? self.latestBatch,
                      batch.frameID.generation == anchor.frameID.generation else { return false }
                guard self.frameIssue(batch, now: self.clock.now) == nil || self.isStoppedReadyRefresh else { return false }
                if self.state == .reacquiring, self.recoveryStopBoundary == nil,
                   self.pendingFrame?.frameID == batch.frameID, let point = anchor.position {
                    let association = self.tracker.reacquireEvaluated(batch.people,
                        lastPosition: point, now: self.clock.now,
                        expectedGeneration: anchor.frameID.generation, frameID: batch.frameID)
                    if case .matched = association.decision {
                        (self.motion as? any FollowMeContextualMotion)?.inhibitScanContinuation(origin: .detection)
                        return false
                    }
                }
                if self.state != .reacquiring, self.state != .recoveringReadyPath, let locked = self.locked {
                    let association: FollowTrackMatch?
                    if self.currentAssociation?.generation == token,
                       self.currentAssociation?.frameID == batch.frameID {
                        association = self.currentAssociation?.decision
                    } else if self.pendingFrame?.frameID == batch.frameID {
                        if self.evaluatedPendingSnapshot?.batch.frameID != batch.frameID {
                            self.evaluatedPendingSnapshot = FollowAdmissionSnapshot(batch: batch,
                                previous: locked, pending: true, tracker: self.tracker,
                                now: self.clock.now, configuration: self.config,
                                requiresDepth: !self.acquisitionRotationIgnoresDepth)
                        }
                        association = self.evaluatedPendingSnapshot?.association?.decision
                    } else {
                        association = nil
                    }
                    guard case .matched = association else { return false }
                }
                return true
            })
    }

    private func adoptMemory(_ person: FollowPersonObservation, batch: FollowFrameBatch,
                             association: FollowReliableMemory.Association) {
        guard recoveryEpisode == nil, person.frameID == batch.frameID,
              reliableMemory?.frameID != person.frameID,
              frameIssue(batch, now: clock.now) == nil,
              let quality = batch.trackingQuality,
              let memory = FollowReliableMemory(accepted: person, association: association,
                  now: clock.now, trackingQuality: quality) else { return }
        reliableMemory = memory
    }

    private func recoveryPose(_ sample: NavigationPoseSample, expectedGeneration: UInt64) -> FollowRecoveryPose? {
        guard sample.source != "legacy_unknown", sample.rejection(at: clock.now, expectedGeneration: expectedGeneration) == nil,
              let pose = sample.pose, let frame = sample.frameID, let timestamp = sample.sourceTimestamp,
              let quality = sample.trackingQuality else { return nil }
        return .init(pose: pose, frameID: frame, timestamp: timestamp, trackingQuality: quality)
    }

    private func scanRecovery(generation token: UInt64) {
        guard var episode = recoveryEpisode, clock.now < episode.deadline else { return }
        guard let anchor = episode.anchor else {
            emitRecoveryUnavailable("missing_reliable_memory"); return
        }
        guard let companion = motion as? any FollowMeAbsoluteHeadingMotion else {
            emitRecoveryUnavailable("absolute_heading_companion_unavailable"); return
        }
        if episode.center == nil {
            let sample = companion.recoveryPoseSample()
            guard let pose = recoveryPose(sample, expectedGeneration: anchor.frameID.generation) else {
                emitRecoveryUnavailable(sample.rejection(at: clock.now, expectedGeneration: anchor.frameID.generation)
                    ?? "missing_recovery_provenance"); return
            }
            episode = episode.selectingCenter(current: pose, now: clock.now)
            recoveryEpisode = episode
            if episode.center != nil {
                recoveryCenterSource = sample
                recoveryCenterReadUptime = clock.now
                recoveryDiagnosticUnavailable = nil
            }
            emitRecovery(episode.center == nil ? "unavailable" : "center_selected",
                reason: episode.center == nil ? "no_valid_center_or_source" : "center_frozen",
                episode: episode, centerSource: sample)
        }
        guard let center = episode.center, episode.stageIndex < FollowReacquisitionPlanner.offsets.count else { return }
        let episodeID = episode.id
        operation &+= 1
        let id = operation
        guard let authorization = recoveryAuthorization(generation: token, operation: id) else { return }
        let request = FollowRecoveryHeadingRequest(stageHeading: FollowReacquisitionPlanner.wrap(
            center.heading + FollowReacquisitionPlanner.offsets[episode.stageIndex]), authorization: authorization,
            stageIndex: episode.stageIndex, segmentIndex: episode.segmentIndex,
            maximumSegment: min(config.scanIncrement, .pi / 6))
        let context = FollowMotionRequestContext(sessionGeneration: token, requestToken: id,
            purpose: .followScan, phase: String(describing: state),
            scanResponseSeed: scanResponseSeed?.sourceGeneration == latestFrame?.generation ? scanResponseSeed : nil,
            allowsAcquisitionRotationWithoutDepth: acquisitionRotationIgnoresDepth)
        activeRequest = context
        scanning = true
        emitRecovery("request_started", reason: "absolute_stage_requested", episode: episode,
            extra: ["request_token": .number(Double(id)), "requested_stage_heading_rad": .number(request.stageHeading)])
        movementTask = Task { [weak self, motion] in
            let result = await motion.performRecoveryHeading(request, context: context)
            guard let self else { return }
            if await self.consumeResult(result) { return }
            guard self.generation == token, self.operation == id,
                  self.recoveryEpisode?.id == episodeID, self.state == .reacquiring else { return }
            if self.recoveryExpired { _ = await self.expireRecovery(generation: token); return }
            self.scanning = false
            self.movementTask = nil
            if result.result == .arrived, result.stopOutcome == .confirmed { self.beginScanObservation() }
            self.emitRecovery("request_completed", reason: String(describing: result.result),
                stop: result.stopOutcome.rawValue,
                extra: FollowReacquisitionDiagnostics.segmentPayload(result.recovery).merging([
                    "controller_operation_id": FollowReacquisitionDiagnostics.number(result.context.controllerOperationID.map(Double.init)),
                    "request_token": .number(Double(id))]) { _, new in new })
            guard self.canScan, result.result == .arrived, result.stopOutcome == .confirmed,
                  let evidence = result.recovery, let source = evidence.arrivalSource,
                  let actual = self.recoveryPose(source, expectedGeneration: anchor.frameID.generation),
                   let current = self.recoveryEpisode else { return }
            if result.context.request == context, result.failure == nil,
               let learned = result.measuredScanResponse, learned.sourceGeneration == anchor.frameID.generation,
               learned.responseRate.isFinite { self.retainScanResponse(learned) }
            // The measured stage gate owns progression; command counts and result labels do not.
            let stageArrival = current.recordingArrival(actual: actual, now: self.clock.now)
            if stageArrival.stageIndex != current.stageIndex {
                self.recoveryEpisode = stageArrival
            } else if evidence.segmentArrived, let target = evidence.segmentHeading {
                self.recoveryEpisode = current.recordingSegmentArrival(target: target, actual: actual, now: self.clock.now)
            } else { return }
            self.emitRecovery("segment_completed", reason: evidence.requestedDelta == 0
                ? "stage_skipped_within_tolerance" : (evidence.stageArrived == true ? "measured_stage_arrival" : "measured_segment_arrival"),
                episode: current, stop: result.stopOutcome.rawValue,
                extra: FollowReacquisitionDiagnostics.segmentPayload(evidence).merging([
                    "next_stage_index": .number(Double(self.recoveryEpisode?.stageIndex ?? current.stageIndex)),
                    "next_segment_index": .number(Double(self.recoveryEpisode?.segmentIndex ?? current.segmentIndex)),
                    "controller_operation_id": FollowReacquisitionDiagnostics.number(result.context.controllerOperationID.map(Double.init)),
                    "request_token": .number(Double(id))]) { _, new in new })
            if self.recoveryEpisode?.stageIndex == FollowReacquisitionPlanner.offsets.count {
                self.emitRecovery("exhausted", reason: "finite_pass_complete_observe_stationary", stop: "confirmed")
            }
            self.scan(generation: token)
        }
    }

    /// Only the controller's confirmed-stopped pre-probe window can tolerate
    /// capture age. Structural faults and stop/ownership transitions never do.
    private var isStoppedReadyRefresh: Bool {
        guard state == .recoveringReadyPath, poseDeadline == nil,
              !stopBlocked, stopTask == nil, confirmationTask == nil, pendingHealthFrame == nil,
              let latestBatch, FollowFrameHealth.safetyTransitionPriority(latestBatch) == 0,
              clock.now >= latestBatch.timestamp else { return false }
        let batch = pendingFrame ?? latestBatch
        guard FollowFrameHealth.safetyTransitionPriority(batch) == 0,
              clock.now.isFinite, clock.now >= batch.timestamp else { return false }
        return activeReadyAdmission?.isRefreshingRecovery(at: clock.now) == true
    }

    private func perceptionUnavailable(_ issue: FollowPerceptionIssue, generation token: UInt64) async {
        guard generation == token else { return }
        if state == .recoveringReadyPath {
            updatePerceptionIssue(issue)
            if issue == .staleFrame, isStoppedReadyRefresh {
                // No probe has started. The controller owns the fixed refresh
                // deadline and must revalidate the newest target before sending.
                return
            }
            _ = await finish(.failed("Floor check stopped. " + issue.message))
            return
        }
        searchClearance?.revoke()
        updatePerceptionIssue(issue)
        // The deadline also marks an outage whose stop has already been requested.
        // Subsequent unhealthy frames update diagnostics without stopping again.
        guard poseDeadline == nil else { return }
        cancelAlignment()
        if state == .signalingReady { state = .aligning }
        let deadline = clock.now + config.perceptionRecoverySeconds
        poseDeadline = deadline
        poseTask?.cancel()
        poseTask = Task { [weak self, clock] in
            await clock.sleep(seconds: max(0, deadline - clock.now))
            guard !Task.isCancelled, let self, self.generation == token, self.poseDeadline == deadline,
                  self.clock.now >= deadline else { return }
            _ = await self.finish(.failed(self.perceptionFailureMessage))
        }
        // Enforce recovery independently of the duration of motor acknowledgement.
        guard await confirmStop(generation: token) else { return }
        guard generation == token, poseDeadline == deadline,
              perceptionIssue != nil else { return }
        lastGoal = nil
        scanning = false
    }

    private func emitAssociation(_ evaluation: FollowAssociationEvaluation, batch: FollowFrameBatch, now: Double) {
        let previous = summaryBudget.previousOutcome
        let trackerReasons = Set(evaluation.candidates.compactMap { candidate -> String? in
            if case .string(let reason) = candidate["rejection_reason"] { return reason }; return nil
        }).sorted().joined(separator: ",")
        let reasons = (batch.perceptionDiagnostics?.transitionSignature ?? "unknown") + "|" + trackerReasons
        guard summaryBudget.takeAssociation(outcome: evaluation.outcome, now: now, reasons: reasons) else { return }
        var payload = evaluation.payload(batch: batch, now: now, previousOutcome: previous)
        payload.merge(recoverySnapshot(now: now)) { _, new in new }
        if state == .waitingForClearance {
            let selected = evaluation.selectedIndex.map { batch.people[$0] }
            payload.merge(FollowReadinessDiagnostics(batch: batch, person: selected, now: now, gate: readySignalClearance,
                pending: pendingAdmission != nil, attempted: readySignalAttempted, succeeded: readySignalSucceeded,
                stopState: stopBlocked ? "failed" : diagnosticStopState).payload) { existing, _ in existing }
        }
        failureEmitter.emit(.init(event: "follow_person.association",
            context: .init(sessionGeneration: generation, phase: String(describing: state), outcome: evaluation.outcome),
            payload: payload))
    }

    private func emitRecovery(_ event: String, reason: String, episode: FollowReacquisitionEpisode? = nil,
                              stop: String? = nil, centerSource: NavigationPoseSample? = nil,
                              extra: [String: FollowDiagnosticValue] = [:]) {
        guard let episode = episode ?? recoveryEpisode else { return }
        var payload = FollowReacquisitionDiagnostics.payload(episode, now: clock.now,
            stop: stop ?? (stopBlocked ? "failed" : diagnosticStopState), centerSource: centerSource ?? recoveryCenterSource,
            centerReadUptime: recoveryCenterReadUptime)
        payload.merge(FollowReacquisitionDiagnostics.snapshot(memory: reliableMemory,
            provisional: recoveryHasProvisionalLock ? locked : nil, episode: episode,
            healthy: perceptionIssue == nil, currentFrame: latestFrame)) { _, new in new }
        payload.merge(extra) { _, new in new }
        failureEmitter.emit(.init(event: "follow_recovery." + event,
            context: .init(sessionGeneration: generation, operationID: activeRequest?.requestToken,
                purpose: activeRequest?.purpose.rawValue, phase: String(describing: state), reason: reason),
            payload: payload))
    }

    private func emitRecoveryUnavailable(_ reason: String) {
        guard recoveryDiagnosticUnavailable != reason else { return }
        recoveryDiagnosticUnavailable = reason
        emitRecovery("unavailable", reason: reason, extra: ["unavailable_reason": .string(reason)])
    }

    private func recoveryTermination(reason: String, phase: FollowMeState) -> FollowDiagnosticEvent? {
        guard let episode = recoveryEpisode else { return nil }
        return .init(event: "follow_recovery.terminated", context: .init(sessionGeneration: generation,
            operationID: activeRequest?.requestToken, phase: String(describing: phase), reason: reason),
            payload: FollowReacquisitionDiagnostics.payload(episode, now: clock.now, stop: "pending",
                centerSource: recoveryCenterSource, centerReadUptime: recoveryCenterReadUptime))
    }

    private func emitRecoveryTermination(_ event: FollowDiagnosticEvent, confirmed: Bool) {
        var payload = event.payload
        payload["stop_outcome"] = .string(confirmed ? "confirmed" : "failed")
        payload["terminal_confirmation_uptime_s"] = .number(clock.now)
        payload["budget_snapshot_boundary"] = .string("terminal_fence_before_stop_wait")
        failureEmitter.emit(.init(event: event.event, context: .init(sessionGeneration: event.context.sessionGeneration,
            operationID: event.context.operationID, phase: event.context.phase,
            stale: event.context.sessionGeneration.map { generation != $0 &+ 1 } ?? true,
            reason: confirmed ? event.context.reason : "stop_not_confirmed"), payload: payload))
    }

    private func recoverySnapshot(now: Double) -> [String: FollowDiagnosticValue] {
        let episode = isActive ? recoveryEpisode : nil
        var payload = FollowReacquisitionDiagnostics.snapshot(memory: reliableMemory, provisional: locked,
            episode: episode, healthy: latestBatch.map {
                FollowFrameHealth.issue($0, now: now, configuration: config) == nil
            } ?? false, currentFrame: latestBatch?.frameID)
        if !isActive { payload["current_target_availability"] = .string("unavailable_inactive") }
        if episode != nil, !recoveryHasProvisionalLock {
            for key in ["provisional_frame_id", "provisional_person_x", "provisional_person_z", "provisional_raw_person_id"] {
                payload[key] = .null
            }
        }
        payload["episode_id"] = .null
        payload["deadline_s"] = .null
        payload["center_heading_rad"] = .null
        if let episode {
            payload.merge(FollowReacquisitionDiagnostics.payload(episode, now: now,
                stop: stopBlocked ? "failed" : diagnosticStopState, centerSource: recoveryCenterSource,
                centerReadUptime: recoveryCenterReadUptime)) { _, new in new }
        }
        return payload
    }

    private var sessionTimingPayload: [String: FollowDiagnosticValue] {
        let now = clock.now
        let commandElapsed = commandReceivedAt.flatMap { $0.isFinite && now >= $0 ? now - $0 : nil }
        return ["command_received_at_s": commandReceivedAt.map { .number($0) } ?? .null,
            "command_elapsed_s": commandElapsed.map { .number($0) } ?? .null,
            "session_started_at_s": sessionStartedAt.map { .number($0) } ?? .null,
            "session_elapsed_s": sessionStartedAt.map { .number(now - $0) } ?? .null,
            "phase_started_at_s": phaseStartedAt.map { .number($0) } ?? .null,
            "phase_elapsed_s": phaseStartedAt.map { .number(now - $0) } ?? .null,
            "stationary_pause_seconds": .number(config.stationaryPauseSeconds),
            "maximum_observation_age_s": .number(config.maximumObservationAge),
            "alignment_angular_tolerance_rad": .number(config.alignmentAngularTolerance),
            "ready_signal_clearance_m": .number(config.readySignalClearance),
            "timing_clock": .string("system_uptime")]
    }

    private var readinessPayload: [String: FollowDiagnosticValue] {
        FollowReadinessDiagnostics(batch: latestBatch, person: locked, now: clock.now, gate: readySignalClearance,
            pending: pendingAdmission != nil, attempted: readySignalAttempted, succeeded: readySignalSucceeded,
            stopState: stopBlocked ? "failed" : diagnosticStopState).payload
    }

    private func emitReadiness(_ event: String, reason: String, admission: FollowReadyAdmission? = nil) {
        let admission = admission ?? FollowReadyAdmissionScope.current
        let snapshot = admission?.observationSnapshot
        var payload = snapshot.map { FollowReadinessDiagnostics(batch: $0.batch, person: $0.person,
            now: $0.evaluatedAt, gate: config.readySignalClearance, pending: pendingAdmission != nil,
            attempted: readySignalAttempted, succeeded: readySignalSucceeded, stopState: diagnosticStopState).payload } ?? readinessPayload
        if let snapshot, let association = snapshot.association {
            payload.merge(association.evaluation.payload(batch: snapshot.batch, now: snapshot.evaluatedAt, previousOutcome: nil)) { _, new in new }
        }
        payload["rejection_condition"] = admission?.rejectionCondition.map { .string($0) } ?? .null
        payload["observation_rejection_condition"] = snapshot?.rejection.map { .string($0) } ?? .null
        payload["pending_frame_evaluation"] = .string(snapshot.map { $0.pending ? "evaluated" : "not_pending" } ?? "not_evaluated")
        payload["admission_evaluated_at_s"] = snapshot.map { .number($0.evaluatedAt) } ?? .null
        payload.merge(sessionTimingPayload) { _, new in new }
        payload.merge(recoverySnapshot(now: clock.now)) { _, new in new }
        payload.merge(FollowReadinessDiagnostics.controllerPayload(admission,
            perceptionFrame: snapshot?.batch.frameID ?? latestBatch?.frameID, sendAuthorized: readySignalAttempted,
            person: snapshot == nil ? locked : snapshot?.person)) { _, new in new }
        failureEmitter.emit(.init(event: event,
            context: .init(sessionGeneration: generation, operationID: activeRequest?.requestToken,
                purpose: "followReady", phase: String(describing: state), reason: reason), payload: payload))
    }

    private func emitReadyCancellation(_ result: FollowMeState) {
        guard state == .waitingForClearance || state == .signalingReady || pendingAdmission != nil else { return }
        let reason: String
        if result == .stopped { reason = "local_stop" }
        else if result == .failed("Person too close during ready signal. Step back and start following again.") {
            reason = "person_approached_during_signal"
        } else { reason = "readiness_interrupted" }
        diagnosticStopState = "pending"
        readinessStopPending = true
        emitReadiness("follow_ready.cancelled", reason: reason)
    }

    private func recordReadinessStop(_ confirmed: Bool) {
        diagnosticStopState = confirmed ? "confirmed" : "failed"
        guard readinessStopPending else { return }
        readinessStopPending = false
        emitReadiness(confirmed ? "follow_ready.stop_confirmed" : "follow_ready.stop_failed",
            reason: confirmed ? "stop_acknowledged" : "stop_not_confirmed")
    }

    private func updatePerceptionIssue(_ issue: FollowPerceptionIssue?) {
        perceptionIssue = issue
        let reason = issue == .trackingLimited ? latestBatch?.trackingReason : nil
        guard issue != loggedIssue || reason != loggedTrackingReason else { return }
        let previous = loggedIssue
        loggedIssue = issue
        loggedTrackingReason = reason
        if issue != nil {
            log("follow_perception_unavailable")
        } else if previous != nil {
            log("follow_perception_recovered", extra: ["previous_issue": previous!.rawValue])
        }
    }

    private func log(_ event: String, extra: [String: String] = [:]) {
        let batch = latestBatch
        var fields: [String: String] = [
            "state": String(describing: state),
            "ready_signal_attempted": String(readySignalAttempted),
            "ready_admission_pending": String(pendingAdmission != nil),
            "ready_signal_succeeded": String(readySignalSucceeded),
            "ready_signal_clearance_metres": String(readySignalClearance),
            "issue": perceptionIssue?.rawValue ?? "none",
            "frame_id": batch.map { "\($0.frameID.generation):\($0.frameID.sequence)" } ?? "none",
            "frame_timestamp": batch.map { String($0.timestamp) } ?? "unknown",
            "frame_age_seconds": batch.map { String(clock.now - $0.timestamp) } ?? "unknown",
            "tracking_quality": batch?.trackingQuality.map { String(describing: $0) } ?? "unknown",
            "tracking_reason": batch?.trackingReason?.rawValue ?? "unknown",
            "tracking_reason_source": "live_ar_diagnostic_only",
            "pose_available": batch.map { String($0.pose != nil) } ?? "unknown",
            "depth_available": batch.map { String($0.depthAvailable) } ?? "unknown",
            "inference_duration_seconds": batch?.inferenceDuration.map { String($0) } ?? "unknown"
        ]
        fields.merge(extra) { _, new in new }
        eventSink(event, fields)
    }
}
