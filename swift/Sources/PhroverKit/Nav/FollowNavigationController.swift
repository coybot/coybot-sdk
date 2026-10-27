import Foundation
import RoverNav

/// Follow-me motion runtime, ported from Astral's proven source-evidence and burst
/// controller. NavigationController owns admission and drains this runtime before
/// another navigation operation can send commands. Its ordinary mission navigation
/// and depth-recovery behavior remain in NavigationController.
@Observable
@MainActor
final class FollowNavigationController {
    public enum State: Equatable, Sendable { case idle, planning, driving, arrived, failed(String) }
    enum VisualTargetApproachDecision: Equatable { case inactive, approach, arrived }
    private enum RotationMode { case continuous, scan, followScan }

    var onStateChange: ((State, [Vec2]) -> Void)?
    var onSafetyStateChange: ((NavigationSafetyState) -> Void)?
    var onFailure: ((FollowMotionFailureDelivery) -> Void)?
    var hasUnconfirmedStop: Bool { stopUnconfirmed }
    public private(set) var state: State = .idle { didSet { onStateChange?(state, path) } }
    public private(set) var path: [Vec2] = [] { didSet { onStateChange?(state, path) } }
    public private(set) var safetyState: NavigationSafetyState = .idle

    private let currentPose: () -> Pose2D?
    private let currentPoseSample: (() -> NavigationPoseSample?)?
    private let sourceNow: () -> TimeInterval
    private let sourceEvents: (() -> AsyncStream<NavigationPoseSample>)?
    private let sourceStopSnapshot: (() -> NavigationPoseSample?)?
    private let sourceHighWater: (() -> FollowTurnSourceHighWater?)?
    private let sourceHealth: (() -> FollowTurnSourceHealth)?
    private let turnPoseEvidence: ((ARFrameID, ARFrameID) -> [FollowTurnBurstPlanner.Sample]?)?
    private var followTurnSourceGate = FollowTurnSourceGate()
    private var followTurnSourceTask: Task<Void, Never>?
    private var followTurnSourceSubscriptionID: UUID?
    private var followTurnSourceWaiters: [UUID: CheckedContinuation<Void, Never>] = [:]
    private final class FollowTurnAckRead {
        let owner: UInt
        var completed = false
        var value: Date?
        init(owner: UInt) { self.owner = owner }
    }
    private var followTurnAckRead: FollowTurnAckRead?
    private var followTurnSourceObservers: [UUID: (NavigationPoseSample) -> Void] = [:]
    private(set) var followTurnStopFence: FollowTurnStopFence?
    private var pendingFollowTurnBurst: FollowTurnBurstFence?
    private var readyRecoveryPulse: ReadyPathRecovery.PulseAuthority?
    private var boundedReadyGeneration: UInt?
    private var pendingFollowTurnBurstDrain: FollowTurnBurstDrain?
    var followTurnBurstPendingStatus: FollowTurnBurstPendingStatus? { pendingFollowTurnBurst?.status }
    private let depthSafetyObservation: (WheelCommand) -> DepthSafetyObservation
    private let observeDepthSafety: ((@escaping (DepthSafetySnapshot?) -> Void) -> (() -> Void))?
    private struct DepthSafetyStop: Error { let failure: NavigationFailure }
    private let currentForwardClearance: () -> Double
    private let makePlan: (Vec2, Vec2) -> [Vec2]?
    private let readySignalCostmap: (@MainActor (Vec2) async -> Costmap?)?
    private let currentLastAck: () async -> Date?
    private let sendCommand: (WheelCommand) async throws -> Void
    private let sendFollowBurstCommand: (WheelCommand) async -> RoverCommandDiagnosticResult
    private let transportUptime: @Sendable () -> TimeInterval
    private let stopRover: () async throws -> Void
    private let sleep: (Duration) async -> Void
    private let now: () -> Date
    private let pursuit = PursuitController(params: .init(
        wheelBase: RoverConfig.wheelBase,
        goalTolerance: 0.2,
        minimumRotateWheelSpeed: RoverConfig.minimumRotateWheelSpeed))
    private let guardLayer = ObstacleGuard()
    private static let obstacleArrivalDistance = 0.65

    private var loop: Task<NavigationResult, Never>?
    /// Latest goal for the running follow drive; `drive` adopts it on its next tick so a
    /// moving person does not need a stop-and-relaunch per goal. Nil outside a follow drive.
    private var liveFollowGoal: Vec2?
    private var stopConfirmation: Task<Void, Error>?
    private var stopUnconfirmed = false
    private var operationGeneration: UInt = 0 {
        didSet {
            pendingFollowTurnBurst?.inhibit()
            wakeFollowTurnSourceWaiters()
        }
    }
    private var replanCounter = 0
    private var activePolicy: (any PathAdmissibilityPolicy)?
    private var safetyStateContinuations: [UUID: AsyncStream<NavigationSafetyState>.Continuation] = [:]
    private var depthRecoverySession: UInt64 = 0
    private var depthRecoveryAttempts = 0
    private var nextFollowOperationID: UInt64 = 0
    private var activeFollowEvidence: FollowMotionOperationEvidence?
    private var terminalFollowEvidence: FollowMotionOperationEvidence?
    private var followFailureContinuations: [UUID: AsyncStream<FollowMotionFailureDelivery>.Continuation] = [:]
    private var diagnosticEmitter: FollowDiagnosticEmitter?

    func followMotionFailures() -> AsyncStream<FollowMotionFailureDelivery> {
        let id = UUID()
        return AsyncStream { continuation in
            followFailureContinuations[id] = continuation
            if case .failed(let reason) = safetyState {
                // Current safety snapshot, not an inferred historical follow operation.
                continuation.yield(.init(context: .unknown, reason: reason,
                    stopOutcome: stopUnconfirmed ? .failed : .unknown, source: .stream))
            }
            continuation.onTermination = { @Sendable [weak self] _ in
                Task { @MainActor in self?.followFailureContinuations[id] = nil }
            }
        }
    }

    func performFollowMotion(_ request: FollowMotionRequest, context: FollowMotionRequestContext?) async -> FollowMotionResult {
        // Session identity is monotonic. Operation/reacquisition churn cannot
        // replenish the finite recovery allowance, nor can an old session.
        if let session = context?.sessionGeneration, session > depthRecoverySession {
            depthRecoverySession = session
            depthRecoveryAttempts = 0
        }
        nextFollowOperationID &+= 1 // Controller-owned identity reserved before the first suspension.
        let captured = FollowMotionOperationContext(request: context, controllerOperationID: nextFollowOperationID,
            purpose: request.purpose, profile: (request.purpose == .followScan || request.purpose == .followAlignment)
                ? .turnBurst(purpose: request.purpose) : nil,
            requestedRotation: FollowRecoveryScope.heading == nil ? request.requestedRotation : nil)
        let evidence = FollowMotionOperationEvidence(context: captured)
        if (request.purpose == .followScan || request.purpose == .followAlignment), let diagnosticEmitter {
            evidence.scanTrace = FollowScanDiagnosticTrace(emitter: diagnosticEmitter)
            evidence.burstTrace = FollowTurnBurstDiagnosticTrace()
        }
        if stopUnconfirmed { evidence.recordStop(.failed) }
        if let previous = activeFollowEvidence {
            previous.fenced = true
            previous.scanTrace?.cancel(origin: "replacement", evidence: previous, latch: stopUnconfirmed)
        }
        terminalFollowEvidence = nil
        activeFollowEvidence = evidence
        evidence.scanTrace?.emit("operation_begin", evidence: evidence, latch: stopUnconfirmed)
        return await withTaskCancellationHandler {
            await FollowMotionTaskScope.$evidence.withValue(evidence) {
                var result: NavigationResult
                switch request {
                case .scan(let angle): result = await rotateForFollowScan(by: angle)
                case .alignment(let angle): result = await rotateForFollowAlignment(by: angle)
                case .ready: result = await navigateForFollowReadySignal()
                case .following(let goal, let clearance): result = await navigateForFollow(to: goal, stoppingAtForwardClearance: clearance)
                }
                if result == .arrived, FollowRecoveryScope.authorization != nil,
                   !recoveryAuthorized || evidence.fenced || evidence.ownedGeneration != operationGeneration {
                    result = .cancelled
                }
                if result == .arrived, request.purpose != .followScan, FollowRecoveryScope.authorization != nil,
                   followPoseRejection(readFollowPose(), at: sourceNow()) != nil {
                    result = .failed(.trackingLost)
                    finish(result)
                }
                if Task.isCancelled, let confirmation = beginCallerCancellation(for: evidence) {
                    // Cancellation is inhibition, not acknowledgement. Freeze terminal
                    // evidence only after the independently executing confirmation drains.
                    let confirmed = await confirmation.value
                    result = confirmed == false ? .failed(.commandFailed) : .cancelled
                }
                if result == .cancelled {
                    evidence.fenced = true
                    evidence.scanTrace?.cancel(origin: "task_cancellation", evidence: evidence, latch: stopUnconfirmed)
                }
                if case .failed(let reason) = result {
                    evidence.recordFailure(reason)
                    if !evidence.emittedFailure, let failure = evidence.failure(source: .stream) {
                        evidence.scanTrace?.failure(reason, evidence: evidence, latch: stopUnconfirmed)
                        deliverFollowFailure(failure)
                        evidence.emittedFailure = true
                    }
                }
                let terminal = evidence.result(result) // Freeze before clearing ownership or returning.
                evidence.scanTrace?.emit("operation_complete", evidence: evidence, latch: stopUnconfirmed,
                    outcome: result == .arrived ? "completed" : (result == .reobservationRequired ? "reobservation_required" : (result == .cancelled ? "cancelled" : "failed")))
                if activeFollowEvidence === evidence {
                    activeFollowEvidence = nil
                    terminalFollowEvidence = evidence
                }
                return terminal
            }
        } onCancel: {
            // onCancel is Sendable and can run off actor. One bounded actor hop
            // requests controller-owned cleanup; it never sends motor commands.
            Task { @MainActor [weak self] in
                _ = self?.beginCallerCancellation(for: evidence)
            }
        }
    }

    private func beginCallerCancellation(for evidence: FollowMotionOperationEvidence) -> Task<Bool?, Never>? {
        if let confirmation = evidence.callerCancellationStop { return confirmation }
        guard activeFollowEvidence === evidence, evidence.ownedGeneration == operationGeneration,
              !evidence.fenced else { return nil }
        evidence.fenced = true
        evidence.scanTrace?.cancel(origin: "task_cancellation", evidence: evidence, latch: stopUnconfirmed)
        operationGeneration &+= 1
        let reservation = operationGeneration
        loop?.cancel()
        let confirmation = Task { @MainActor () -> Bool? in
            // A replacement may reserve the controller before this actor task runs.
            // Stale cancellation must not cancel its loop or append another motor stop.
            guard activeFollowEvidence === evidence, operationGeneration == reservation else { return nil }
            do {
                try await confirmStop(retryingFailedStop: true, evidence: evidence, origin: "independent")
                return true
            } catch { return false }
        }
        evidence.callerCancellationStop = confirmation
        return confirmation
    }

    private func deliverFollowFailure(_ failure: FollowMotionFailureDelivery) {
        onFailure?(failure)
        for continuation in followFailureContinuations.values { continuation.yield(failure) }
    }

    public func safetyStates() -> AsyncStream<NavigationSafetyState> {
        let id = UUID()
        return AsyncStream { continuation in
            safetyStateContinuations[id] = continuation
            continuation.yield(safetyState)
            continuation.onTermination = { @Sendable [weak self] _ in
                Task { @MainActor in self?.safetyStateContinuations[id] = nil }
            }
        }
    }

    init(ar: ARSessionManager, control: RoverControl,
         depthSafetyObservation: ((WheelCommand) -> DepthSafetyObservation)? = nil) {
        let planner = AStarPlanner()
        currentPose = { ar.pose }
        currentPoseSample = { ar.latestSnapshot.map(NavigationPoseSample.init(snapshot:)) }
        sourceNow = { ProcessInfo.processInfo.systemUptime }
        sourceStopSnapshot = { ar.latestSnapshot.map(NavigationPoseSample.init(snapshot:)) }
        sourceHighWater = { ar.sourceHighWater }
        sourceHealth = { .init(generation: ar.sessionGeneration, trackingQuality: ar.trackingQuality) }
        turnPoseEvidence = { ar.turnPoseEvidence(from: $0, through: $1) }
        sourceEvents = {
            let snapshots = ar.snapshots() // Subscribe synchronously before a stop can be admitted.
            let lifecycle = ar.lifecycleEvents()
            return AsyncStream(bufferingPolicy: .bufferingNewest(1)) { continuation in
                let task = Task { @MainActor in
                    for await snapshot in snapshots { continuation.yield(.init(snapshot: snapshot)) }
                    continuation.finish()
                }
                let healthTask = Task { @MainActor in
                    for await event in lifecycle {
                        switch event {
                        case .reset, .interrupted, .failed: continuation.yield(.unavailable)
                        case .interruptionEnded: break // Only an actual new healthy source restores provenance.
                        }
                    }
                }
                continuation.onTermination = { @Sendable _ in task.cancel(); healthTask.cancel() }
            }
        }
        currentForwardClearance = { ar.forwardClearance }
        self.depthSafetyObservation = depthSafetyObservation ?? { ar.depthSafetyObservation(for: $0) }
        self.observeDepthSafety = { ar.observeDepthSafety($0) }
        makePlan = { start, goal in
            let costmap = CostmapBuilder.build(from: ar.meshAnchors, center: start)
            return planner.plan(from: start, to: goal, in: costmap)
        }
        readySignalCostmap = { await CostmapBuilder.buildAsync(from: ar.meshAnchors, center: $0) }
        currentLastAck = { await control.lastAckAt }
        sendCommand = Self.diagnosticSender(legacy: { try await control.sendNavigation($0) },
            receipt: { await control.sendNavigationWithReceipt($0) })
        sendFollowBurstCommand = { await control.sendNavigationWithReceipt($0) }
        transportUptime = { ProcessInfo.processInfo.systemUptime }
        stopRover = Self.diagnosticStopper(legacy: { try await control.stop() },
            receipt: { await control.stopWithReceipt() })
        sleep = { try? await Task.sleep(for: $0) }
        now = Date.init
        diagnosticEmitter = Self.runtimeDiagnosticEmitter()
    }

    init(currentPose: @escaping () -> Pose2D?,
         forwardClearance: @escaping () -> Double,
         depthSafetyObservation: ((WheelCommand) -> DepthSafetyObservation)? = nil,
         observeDepthSafety: ((@escaping (DepthSafetySnapshot?) -> Void) -> (() -> Void))? = nil,
         plan: @escaping (Vec2, Vec2) -> [Vec2]?,
         readySignalCostmap: (@MainActor (Vec2) async -> Costmap?)? = nil,
         lastAckAt: @escaping () async -> Date?,
         sendCommand: @escaping (WheelCommand) async throws -> Void,
         stopRover: @escaping () async throws -> Void,
         sleep: @escaping (Duration) async -> Void,
         now: @escaping () -> Date = Date.init,
         sendCommandReceipt: ((WheelCommand) async -> RoverCommandDiagnosticResult)? = nil,
         stopRoverReceipt: (() async -> RoverCommandDiagnosticResult)? = nil,
         diagnosticEmitter: FollowDiagnosticEmitter? = nil,
         poseSample: (() -> NavigationPoseSample?)? = nil,
         sourceNow: @escaping () -> TimeInterval = { ProcessInfo.processInfo.systemUptime },
         sourceEvents: (() -> AsyncStream<NavigationPoseSample>)? = nil,
         sourceStopSnapshot: (() -> NavigationPoseSample?)? = nil,
         sourceHighWater: (() -> FollowTurnSourceHighWater?)? = nil,
          sourceHealth: (() -> FollowTurnSourceHealth)? = nil,
          turnPoseEvidence: ((ARFrameID, ARFrameID) -> [FollowTurnBurstPlanner.Sample]?)? = nil,
         transportUptime: @escaping @Sendable () -> TimeInterval = { ProcessInfo.processInfo.systemUptime }) {
        self.currentPose = currentPose
        self.currentPoseSample = poseSample
        self.sourceNow = sourceNow
        self.sourceEvents = sourceEvents
        self.sourceStopSnapshot = sourceStopSnapshot
        self.sourceHighWater = sourceHighWater
        self.sourceHealth = sourceHealth
        self.turnPoseEvidence = turnPoseEvidence
        self.currentForwardClearance = forwardClearance
        self.observeDepthSafety = observeDepthSafety
        // Legacy fixtures supply scalar clearance; production always uses AR depth provenance.
        self.depthSafetyObservation = depthSafetyObservation ?? { command in
            .init(state: .clear, clearance: forwardClearance(), supportCount: 0, sampleAge: 0,
                  requiredStoppingDistance: 0, motionClass: .classify(command), speedLimit: nil)
        }
        self.makePlan = plan
        self.readySignalCostmap = readySignalCostmap
        self.currentLastAck = lastAckAt
        self.sendCommand = Self.diagnosticSender(legacy: sendCommand, receipt: sendCommandReceipt)
        self.sendFollowBurstCommand = sendCommandReceipt ?? { command in
            do { try await sendCommand(command); return .init(receipt: .unknown, failure: nil) }
            catch { return .init(receipt: .unknown, failure: error) }
        }
        self.transportUptime = transportUptime
        self.stopRover = Self.diagnosticStopper(legacy: stopRover, receipt: stopRoverReceipt)
        self.sleep = sleep
        self.now = now
        self.diagnosticEmitter = diagnosticEmitter ?? Self.runtimeDiagnosticEmitter()
    }

    private static func runtimeDiagnosticEmitter() -> FollowDiagnosticEmitter {
        .init(streamID: UUID().uuidString, monotonic: { ProcessInfo.processInfo.systemUptime },
            utc: Date.init, sink: { RuntimeFileLog.append($0, fields: $1) }, deferredRuntime: true)
    }

    private static func diagnosticSender(
        legacy: @escaping (WheelCommand) async throws -> Void,
        receipt: ((WheelCommand) async -> RoverCommandDiagnosticResult)?
    ) -> (WheelCommand) async throws -> Void {
        { command in
            let captured = FollowMotionTaskScope.evidence
            if command.left != 0 || command.right != 0 { captured?.recordStop(.pending) }
            if let receipt {
                let response = await receipt(command)
                captured?.recordCommandReceipt(response.receipt)
                _ = try response.get()
            } else {
                do { try await legacy(command) }
                catch { captured?.recordCommandReceipt(.unknown); throw error }
                captured?.recordCommandReceipt(.unknown)
            }
        }
    }

    private static func diagnosticStopper(
        legacy: @escaping () async throws -> Void,
        receipt: (() async -> RoverCommandDiagnosticResult)?
    ) -> () async throws -> Void {
        {
            let captured = FollowMotionTaskScope.evidence
            if let receipt {
                let response = await receipt()
                FollowMotionTaskScope.stopReceiptCapture?.receipt = response.receipt
                captured?.recordStopReceipt(response.receipt)
                _ = try response.get()
            } else {
                do { try await legacy() }
                catch {
                    FollowMotionTaskScope.stopReceiptCapture?.receipt = .unknown
                    captured?.recordStopReceipt(.unknown)
                    throw error
                }
                FollowMotionTaskScope.stopReceiptCapture?.receipt = .unknown
                captured?.recordStopReceipt(.unknown)
            }
        }
    }

    /// Begin autonomously driving to a nav-plane goal.
    public func navigate(to goal: Vec2) {
        startNavigation(to: goal, stoppingAtForwardClearance: nil, policy: nil)
    }

    /// Drive toward a locked visual target and stop at the requested LiDAR stand-off.
    public func navigate(to goal: Vec2, stoppingAtForwardClearance clearance: Double) {
        startNavigation(to: goal, stoppingAtForwardClearance: clearance, policy: nil)
    }

    public func navigateAndWait(
        to goal: Vec2,
        stoppingAtForwardClearance clearance: Double? = nil,
        policy: (any PathAdmissibilityPolicy)? = nil
    ) async -> NavigationResult {
        operationGeneration &+= 1
        let reservation = operationGeneration
        if stopConfirmation != nil {
            // A follow caller may already be draining an independent stop.
            // Do not bypass it through the legacy direct-stop cancellation path.
            do { try await confirmStop() } catch { return .failed(.commandFailed) }
        } else {
            await cancelAndWait()
        }
        guard operationGeneration == reservation else { return .cancelled }
        guard !stopUnconfirmed else { return .failed(.commandFailed) }
        return await startNavigation(
            to: goal, stoppingAtForwardClearance: clearance, policy: policy, cancellingCurrent: false
        ).value
    }

    public func navigateForFollow(to goal: Vec2, stoppingAtForwardClearance clearance: Double) async -> NavigationResult {
        guard !stopUnconfirmed else { return .failed(.commandFailed) }
        operationGeneration &+= 1
        let reservation = operationGeneration
        FollowMotionTaskScope.evidence?.ownedGeneration = reservation
        do { try await confirmStop() } catch { return .failed(.commandFailed) }
        guard operationGeneration == reservation, !Task.isCancelled,
              FollowMotionTaskScope.evidence?.fenced != true else { return .cancelled }
        let task = startNavigation(
            // This goal already stands off from the person. LiDAR clearance alone
            // cannot tell the person from a cart crossing in front of the rover;
            // treating low clearance as target arrival would bypass ObstacleGuard.
            to: goal, stoppingAtForwardClearance: nil, policy: nil,
            cancellingCurrent: false, isFollowGoal: true
        )
        let generation = operationGeneration
        FollowMotionTaskScope.evidence?.ownedGeneration = generation
        let result = await task.value
        guard operationGeneration == generation else { return .cancelled }
        do { try await confirmStop() } catch { return .failed(.commandFailed) }
        return result
    }

    /// Move the running follow drive's goal without stopping. Returns false, changing nothing,
    /// when no follow drive owns the controller, it has been fenced or cancelled, or the new
    /// goal cannot be planned; the caller then falls back to a stop-and-relaunch.
    func updateFollowGoal(_ newGoal: Vec2) -> Bool {
        guard state == .driving, liveFollowGoal != nil, !stopUnconfirmed,
              let evidence = activeFollowEvidence, !evidence.fenced,
              evidence.context.purpose == .followGoal,
              evidence.ownedGeneration == operationGeneration,
              !(loop?.isCancelled ?? true), let start = currentPose()?.position else { return false }
        guard planAndStore(from: start, to: newGoal, policy: activePolicy) == nil else { return false }
        liveFollowGoal = newGoal
        return true
    }

    /// A bounded, pose-controlled 10 cm ready signal, independent of pursuit's 20 cm tolerance.
    public func navigateForFollowReadySignal() async -> NavigationResult {
        guard !stopUnconfirmed else { return .failed(.commandFailed) }
        operationGeneration &+= 1
        let reservation = operationGeneration
        defer { if boundedReadyGeneration == reservation { boundedReadyGeneration = nil } }
        FollowMotionTaskScope.evidence?.ownedGeneration = reservation
        do { try await confirmStop() } catch { return .failed(.commandFailed) }
        guard operationGeneration == reservation, !Task.isCancelled, recoveryAuthorized else { return .cancelled }
        var startSample = readFollowPose()
        guard followPoseRejection(startSample, at: sourceNow()) == nil, let initialStart = startSample.pose else {
            return .failed(currentPoseSample == nil ? .noPose : .trackingLost)
        }
        let start = initialStart
        let direction = Vec2(cos(start.yaw), sin(start.yaw))
        if let readySignalCostmap {
            let planningStarted = sourceNow()
            let sourceGeneration = startSample.frameID?.generation
            let map = await readySignalCostmap(start.position)
            guard operationGeneration == reservation, !Task.isCancelled, recoveryAuthorized,
                  FollowMotionTaskScope.evidence?.fenced != true, !stopUnconfirmed else { return .cancelled }
            guard let map else {
                let evidence = FollowMotionTaskScope.evidence
                evidence?.readyDiagnosticFields["ready_path_rejection"] = .string("map_unavailable")
                evidence?.recordFailure(.noPath, cause: .readyMapUnavailable)
                return .failed(.noPath)
            }
            let planningElapsed = sourceNow() - planningStarted
            guard planningElapsed.isFinite, planningElapsed >= 0,
                  planningElapsed <= RoverConfig.navigationTrackingFreshness else {
                let evidence = FollowMotionTaskScope.evidence
                evidence?.readyDiagnosticFields.merge([
                    "ready_path_rejection": .string("map_stale"),
                    "ready_path_preflight_s": .number(planningElapsed),
                    "ready_path_maximum_age_s": .number(RoverConfig.navigationTrackingFreshness)
                ]) { _, new in new }
                evidence?.recordFailure(.stalled, cause: .readyMapStale)
                return .failed(.stalled)
            }
            // Revalidate liveness without rebasing onto a different AR world frame.
            // The map, corridor and pulse retain the same pre-build origin.
            startSample = readFollowPose()
            guard followPoseRejection(startSample, at: sourceNow(), expectedGeneration: sourceGeneration) == nil,
                  let current = startSample.pose else { return .failed(.trackingLost) }
            let yawShift = abs(normalizeAngle(current.yaw - start.yaw))
            let sweptRadius = 0.12 + CostmapBuilder.Params().inflationRadius
            let poseUncertainty = current.position.distance(to: start.position)
                + 2 * sweptRadius * abs(sin(yawShift / 2))
            let evidence = FollowMotionTaskScope.evidence
            guard poseUncertainty <= 0.02, yawShift <= 0.10 else {
                evidence?.readyDiagnosticFields.merge([
                    "ready_preflight_pose_shift_m": .number(current.position.distance(to: start.position)),
                    "ready_preflight_yaw_shift_rad": .number(yawShift),
                    "ready_preflight_pose_uncertainty_m": .number(poseUncertainty)
                ]) { _, new in new }
                evidence?.recordFailure(.stalled, cause: .readyPreSendPoseShift)
                return .failed(.stalled)
            }
            let goal = start.position + direction * 0.10
            let assessment = AStarPlanner().assess(from: start.position, to: goal, in: map)
            // Check the actual swept segment in the inflated grid; quantized cell
            // centers are not measured rover displacement.
            let corridorClear = map.isSegmentClear(from: start.position,
                to: start.position + direction * 0.12, margin: 0.02 + poseUncertainty)
            let rejection: String?
            let mappedObstruction: Bool
            switch assessment {
            case .path(let points):
                rejection = points.isEmpty ? "empty_path" : (corridorClear ? nil : "corridor_blocked")
                mappedObstruction = !points.isEmpty && !corridorClear
            case .rejected(let failure):
                rejection = failure.rawValue
                mappedObstruction = failure == .startBlocked || failure == .goalBlocked
            }
            let startCell = map.worldToCell(start.position)
            let goalCell = map.worldToCell(goal)
            let fields: [String: FollowDiagnosticValue] = [
                "ready_path_rejection": rejection.map(FollowDiagnosticValue.string) ?? .null,
                "ready_path_preflight_s": .number(sourceNow() - planningStarted),
                "ready_start_cell_cost": .number(Double(map.cost(startCell.cx, startCell.cy))),
                "ready_goal_cell_cost": .number(Double(map.cost(goalCell.cx, goalCell.cy))),
                "ready_corridor_clear": .bool(corridorClear),
                "ready_preflight_pose_uncertainty_m": .number(poseUncertainty),
                "ready_source_age_s": startSample.sourceTimestamp.map { .number(sourceNow() - $0) } ?? .null,
                "ready_map_resolution_m": .number(map.resolution)
            ]
            evidence?.readyDiagnosticFields.merge(fields) { _, new in new }
            diagnosticEmitter?.emit(.init(event: "follow_ready.path_preflight", context: .init(
                sessionGeneration: evidence?.context.request?.sessionGeneration,
                operationID: evidence?.context.controllerOperationID, purpose: "followReady",
                outcome: rejection == nil ? "accepted" : "rejected", reason: rejection), payload: fields))
            if rejection != nil {
                if let admission = FollowReadyAdmissionScope.current, admission.allowsPathRecovery, mappedObstruction {
                    guard admission.admit(boundary: .recoveryProbe, sample: startSample, readUptime: sourceNow()) else {
                        return .cancelled
                    }
                    let recovered = await recoverReadyPath(start: startSample, ownedGeneration: reservation)
                    guard operationGeneration == reservation, !Task.isCancelled, recoveryAuthorized else { return .cancelled }
                    switch recovered {
                    case .clear:
                        // Complete the original 10 cm ready motion, including
                        // probe displacement; never rebase an extra 10 cm here.
                        await sleep(.milliseconds(500))
                        guard operationGeneration == reservation, !Task.isCancelled, recoveryAuthorized else { return .cancelled }
                    case .turned: return .reobservationRequired
                    case .cancelled: return .cancelled
                    case .failed(let failure):
                        evidence?.recordFailure(failure)
                        return .failed(failure)
                    }
                } else {
                    evidence?.recordFailure(.noPath, cause: .readyPathBlocked)
                    return .failed(.noPath)
                }
            }
        } else {
            let goal = start.position + direction * 0.10
            guard let planned = makePlan(start.position, goal), !planned.isEmpty else { return .failed(.noPath) }
            // Exact-path test/custom planning seam without a costmap remains fail-closed.
            guard planned.allSatisfy({ point in
                  let delta = point - start.position
                  let along = delta.x * direction.x + delta.y * direction.y
                  let lateral = abs(delta.x * direction.y - delta.y * direction.x)
                  return along.isFinite && lateral.isFinite && along >= -0.02 && along <= 0.12 && lateral <= 0.02
               }) else { return .failed(.noPath) }
        }
        // Every ready signal is the same bounded 10 cm motion. Its visibility
        // policy must not depend on whether map recovery happened beforehand.
        boundedReadyGeneration = reservation
        state = .driving
        publishSafetyState(.moving)
        let task = Task { await driveReadySignal(from: start, direction: direction,
            sourceGeneration: startSample.frameID?.generation, ownedGeneration: reservation) }
        loop = task
        let result = await task.value
        guard operationGeneration == reservation else { return .cancelled }
        do { try await confirmStop() } catch { return .failed(.commandFailed) }
        guard operationGeneration == reservation, !Task.isCancelled else { return .cancelled }
        if case .failed = result { finish(result) }
        return result
    }

    private func driveReadySignal(from start: Pose2D, direction: Vec2,
                                  sourceGeneration: UInt64?, ownedGeneration: UInt) async -> NavigationResult {
        let started = now()
        var progress = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.01)
        var sent = false
        var previousStop: (uptime: Double, frame: ARFrameID?, returnedElapsed: TimeInterval)?
        var stopReserve = RoverConfig.readySignalStopReserve
        var confirmingStopped = false
        var pulseIndex = 0
        let limits = (travel: 0.12, backward: 0.02, lateral: 0.02, heading: 0.10, duration: 5.0)
        func timingFields(at time: Date) -> [String: FollowDiagnosticValue] {
            ["ready_elapsed_s": .number(time.timeIntervalSince(started)),
             "ready_confirmation_active": .bool(confirmingStopped),
             "ready_final_confirmation_s": .number(RoverConfig.readySignalFinalConfirmation),
             "ready_stop_reserve_s": .number(stopReserve),
             "ready_last_stop_elapsed_s": previousStop.map { .number($0.returnedElapsed) } ?? .null]
        }
        while !Task.isCancelled {
            let ack = await currentLastAck()
            guard !Task.isCancelled, operationGeneration == ownedGeneration,
                  FollowMotionTaskScope.evidence?.fenced != true, !stopUnconfirmed, recoveryAuthorized else { return .cancelled }
            // Actor acknowledgement reads can suspend. Sample clock and safety only
            // after that read so neither fresh acknowledgements nor changed hazards
            // are evaluated against an older snapshot.
            let sample = readFollowPose()
            let time = now()
            let sourceTime = sourceNow()
            FollowMotionTaskScope.evidence?.readyDiagnosticFields = timingFields(at: time)
            guard followPoseRejection(sample, at: sourceTime, expectedGeneration: sourceGeneration) == nil,
                  let pose = sample.pose else { return .failed(.trackingLost) }
            let delta = pose.position - start.position
            let along = delta.x * direction.x + delta.y * direction.y
            let lateral = abs(delta.x * direction.y - delta.y * direction.x)
            let travel = delta.distance(to: .zero)
            let headingChange = normalizeAngle(pose.yaw - start.yaw)
            func poseFields(_ value: Pose2D) -> FollowDiagnosticValue {
                .object(["world_x": .number(value.position.x), "world_z": .number(value.position.y), "yaw_rad": .number(value.yaw)])
            }
            func fail(_ cause: FollowMotionFailureCause) -> NavigationResult {
                let checkpoint = progress.diagnosticSnapshot(distanceToGoal: 0.10 - along, now: time)
                let fields: [String: FollowDiagnosticValue] = [
                    "ready_failure_cause": .string(cause.rawValue), "ready_pulse_index": .number(Double(pulseIndex)),
                    "ready_start_pose": poseFields(start), "ready_current_pose": poseFields(pose),
                    "ready_along_m": .number(along), "ready_lateral_m": .number(lateral),
                    "ready_signed_lateral_m": .number(delta.x * direction.y - delta.y * direction.x),
                    "ready_total_distance_m": .number(travel), "ready_heading_change_rad": .number(abs(headingChange)),
                    "ready_signed_heading_change_rad": .number(headingChange),
                    "ready_watchdog_elapsed_s": checkpoint.elapsed.map { .number($0) } ?? .null,
                    "ready_progress_timeout_s": .number(progress.timeout), "ready_minimum_progress_m": .number(progress.minimumProgress),
                    "ready_maximum_duration_s": .number(limits.duration), "ready_maximum_travel_m": .number(limits.travel),
                    "ready_maximum_backward_m": .number(limits.backward), "ready_maximum_lateral_m": .number(limits.lateral),
                    "ready_maximum_heading_change_rad": .number(limits.heading),
                    "ready_command_invoked": .bool(sent),
                    "ready_source_frame_id": sample.frameID.map { .string("\($0.generation):\($0.sequence)") } ?? .null,
                    "ready_source_timestamp_s": sample.sourceTimestamp.map { .number($0) } ?? .null,
                    "ready_source_read_uptime_s": .number(sourceTime),
                    "ready_source_age_s": sample.sourceTimestamp.map { .number(sourceTime - $0) } ?? .null]
                    .merging(timingFields(at: time)) { _, latest in latest }
                let evidence = FollowMotionTaskScope.evidence
                evidence?.readyDiagnosticFields = fields
                evidence?.recordFailure(.stalled, cause: cause)
                diagnosticEmitter?.emit(.init(event: "follow_ready.failure", context: .init(
                    sessionGeneration: evidence?.context.request?.sessionGeneration,
                    operationID: evidence?.context.controllerOperationID, purpose: "followReady",
                    phase: "signalingReady", outcome: "failed", reason: cause.rawValue), payload: fields))
                return .failed(.stalled)
            }
            guard travel.isFinite, along.isFinite, lateral.isFinite, headingChange.isFinite else { return fail(.readyInvalidGeometry) }
            guard travel <= limits.travel else { return fail(.readyTravelLimit) }
            guard along >= -limits.backward else { return fail(.readyBackwardMotion) }
            guard lateral <= limits.lateral else { return fail(.readyLateralDeviation) }
            guard abs(headingChange) <= limits.heading else { return fail(.readyHeadingDeviation) }
            let clearance = currentForwardClearance()
            guard clearance.isFinite else { return .failed(.obstacle) }
            guard let ack, time.timeIntervalSince(ack) >= 0,
                  time.timeIntervalSince(ack) <= RoverConfig.commsWatchdogTimeout else { return .failed(.commsLost) }
            let decision = guardLayer.evaluate(forwardClearance: clearance,
                lastAckAt: ack, now: time, feedback: nil,
                requireFreshAck: true, checkForwardObstacle: true)
            switch decision {
            case .go: break
            case .stopObstacle: return .failed(.obstacle)
            case .stopCommsLost: return .failed(.commsLost)
            case .stopTipping: return .failed(.tipping)
            }
            guard !Task.isCancelled else { return .cancelled }
            // Reserve the last pulse's stop/drain cost. Once that reserve is
            // reached, only read fresh post-stop evidence; never resume pulsing.
            // A stop that returned after the movement cutoff cannot be rescued.
            let existingCheckpoint = progress.diagnosticSnapshot(distanceToGoal: 0.10 - along, now: time)
            let elapsed = time.timeIntervalSince(started)
            if let previousStop, previousStop.returnedElapsed >= limits.duration { return fail(.readyOverallTimeout) }
            if !confirmingStopped, elapsed + RoverConfig.readySignalPulseBudget + stopReserve >= limits.duration {
                guard sent, previousStop != nil else { return fail(.readyOverallTimeout) }
                confirmingStopped = true
                diagnosticEmitter?.emit(.init(event: "follow_ready.confirmation_started", context: .init(
                    operationID: FollowMotionTaskScope.evidence?.context.controllerOperationID,
                    purpose: "followReady", phase: "signalingReady", outcome: "stopped_observation"), payload: [
                        "ready_elapsed_s": .number(elapsed), "ready_along_m": .number(along),
                        "ready_stop_reserve_s": .number(stopReserve),
                        "ready_movement_cutoff_s": .number(limits.duration),
                        "ready_confirmation_cutoff_s": .number(limits.duration + RoverConfig.readySignalFinalConfirmation)]))
            }
            let completionCutoff = limits.duration + (confirmingStopped ? RoverConfig.readySignalFinalConfirmation : 0)
            if elapsed >= completionCutoff { return fail(.readyOverallTimeout) }
            if (existingCheckpoint.elapsed ?? 0) >= progress.timeout { return fail(.readyProgressTimeout) }
            if let previousStop, sourceGeneration != nil {
                guard let id = sample.frameID, let timestamp = sample.sourceTimestamp else { return .failed(.trackingLost) }
                if timestamp <= previousStop.uptime || id.sequence <= (previousStop.frame?.sequence ?? 0) {
                    await sleep(.milliseconds(20))
                    continue
                }
            }
            // A pre-send pose correction is not measured progress from this signal.
            if along >= 0.08 { return sent ? .arrived : fail(.readyPreSendPoseShift) }
            if confirmingStopped {
                await sleep(.milliseconds(20))
                continue
            }
            if progress.observe(
                distanceToGoal: 0.10 - along, now: time, commanded: true) { return fail(.readyProgressTimeout) }
            let speed = RoverConfig.readySignalWheelMagnitude
            if !sent, let admission = FollowReadyAdmissionScope.current,
               !admission.admit(boundary: .controllerFirstSend, sample: sample, readUptime: sourceTime) { return .cancelled }
            if FollowRecoveryScope.authorization != nil {
                guard recoveryAuthorized, operationGeneration == ownedGeneration,
                      FollowMotionTaskScope.evidence?.fenced != true, !stopUnconfirmed else { return .cancelled }
                if followPoseRejection(sample, at: sourceNow()) != nil { return .failed(.trackingLost) }
            }
            sent = true
            pulseIndex += 1
            // Bound the stronger open-loop output by host budget, not a guessed
            // physical velocity. The shared sender drains before serialized stop.
            let checkpoint = progress.diagnosticSnapshot(distanceToGoal: 0.10 - along, now: time)
            let validUntil = min(sample.sourceTimestamp.map { ($0 + 0.5).nextUp } ?? .infinity,
                sourceTime + RoverConfig.commsWatchdogTimeout - time.timeIntervalSince(ack),
                sourceTime + limits.duration - elapsed - RoverConfig.readySignalPulseBudget - stopReserve,
                sourceTime + max(0, progress.timeout - (checkpoint.elapsed ?? 0)))
            do {
                let pulse = try await executeFollowTurnBurst(WheelCommand(left: speed, right: speed),
                    requestedBudget: RoverConfig.readySignalPulseBudget, purpose: .followReady,
                    sendValidUntil: validUntil)
                guard operationGeneration == ownedGeneration, !Task.isCancelled,
                      FollowMotionTaskScope.evidence?.fenced != true, !stopUnconfirmed, recoveryAuthorized else { return .cancelled }
                if let denial = pulse.send.result.failure as? DepthSafetyStop { return .failed(denial.failure) }
                if pulse.send.result.failure != nil { return .failed(.commandFailed) }
                let pulseReturnUptime = sourceNow()
                let pulseReturnedElapsed = now().timeIntervalSince(started)
                let stoppedSample = readFollowPose()
                let stoppedPoseReadUptime = sourceNow()
                previousStop = (pulse.confirmedStopFence?.acknowledgementUptime ?? pulseReturnUptime,
                    stoppedSample.frameID, pulseReturnedElapsed)
                stopReserve = max(stopReserve,
                    pulseReturnUptime - pulse.send.sendEntryUptime - RoverConfig.readySignalPulseBudget)
                diagnosticEmitter?.emit(.init(event: "follow_ready.pulse_stopped", context: .init(
                    operationID: FollowMotionTaskScope.evidence?.context.controllerOperationID,
                    purpose: "followReady", phase: "signalingReady", outcome: "stop_confirmed"), payload: [
                        "along_before_m": .number(along), "target_distance_m": .number(0.10),
                        "ready_pulse_index": .number(Double(pulseIndex)), "ready_start_pose": poseFields(start),
                        "ready_pose_at_stop_return": stoppedSample.pose.map(poseFields) ?? .null,
                        "ready_stop_return_source_timestamp_s": stoppedSample.sourceTimestamp.map { .number($0) } ?? .null,
                        "ready_stop_ack_uptime_s": pulse.confirmedStopFence.map { .number($0.acknowledgementUptime) } ?? .null,
                        "ready_pulse_return_uptime_s": .number(pulseReturnUptime),
                        "ready_stop_return_pose_read_uptime_s": .number(stoppedPoseReadUptime),
                        "ready_stop_return_frame_id": stoppedSample.frameID.map { .string("\($0.generation):\($0.sequence)") } ?? .null,
                        "wheel_magnitude": .number(speed), "requested_host_budget_s": .number(RoverConfig.readySignalPulseBudget),
                        "send_host_duration_s": .number(pulse.send.responseUptime - pulse.send.sendEntryUptime),
                        "sender_outcome": .string(pulse.send.result.receipt.outcome),
                        "physical_motor_duration_confidence": .string("unknown")]))
            } catch { return Task.isCancelled || operationGeneration != ownedGeneration ? .cancelled : .failed(.commandFailed) }
            await sleep(.seconds(RoverConfig.readySignalStoppedObservation))
        }
        return .cancelled
    }

    private func recoverReadyPath(start: NavigationPoseSample, ownedGeneration: UInt) async -> ReadyPathRecovery.Outcome {
        let evidence = FollowMotionTaskScope.evidence
        func owned() -> Bool {
            !Task.isCancelled && operationGeneration == ownedGeneration && activeFollowEvidence === evidence &&
                evidence?.fenced != true && !stopUnconfirmed && recoveryAuthorized
        }
        let outcome = await ReadyPathRecovery.run(start: start, environment: .init(
            uptime: sourceNow, pose: readFollowPose, depth: { command in
                let observation = self.boundedReadyDepthObservation(command)
                self.recordDepthObservation(observation, disposition: "ready_recovery_assessment")
                return observation
            }, authorized: owned,
            pulse: { authority in
                let ack = await self.currentLastAck()
                guard owned() else { return .cancelled }
                let date = self.now()
                guard let ack, date.timeIntervalSince(ack) >= 0,
                      date.timeIntervalSince(ack) <= RoverConfig.commsWatchdogTimeout else { return .failed(.commsLost) }
                guard authority.permits(authority.command, sample: self.readFollowPose(), now: self.sourceNow()) else {
                    return .failed(.trackingLost)
                }
                self.readyRecoveryPulse = authority
                defer { if self.readyRecoveryPulse?.id == authority.id { self.readyRecoveryPulse = nil } }
                do {
                    let result = try await self.executeFollowTurnBurst(authority.command, requestedBudget: authority.budget,
                        purpose: .followReady, sendValidUntil: min(authority.expiresAt,
                            self.sourceNow() + RoverConfig.commsWatchdogTimeout - date.timeIntervalSince(ack)))
                    guard owned() else { return .cancelled }
                    guard result.confirmedStopFence != nil else { return .failed(.commandFailed) }
                    if let denial = result.send.result.failure as? DepthSafetyStop {
                        if authority.stage == .probe, denial.failure == .obstacle { return .reobservationRequired }
                        return .failed(denial.failure)
                    }
                    return result.send.result.failure == nil ? .arrived : .failed(.commandFailed)
                } catch { return owned() ? .failed(.commandFailed) : .cancelled }
            }, wait: sleep, emit: { stage, fields in
                self.diagnosticEmitter?.emit(.init(event: "follow_ready.path_recovery", context: .init(
                    sessionGeneration: evidence?.context.request?.sessionGeneration,
                    operationID: evidence?.context.controllerOperationID, purpose: "followReady",
                    phase: "recoveringReadyPath", reason: stage), payload: fields))
            }))
        if operationGeneration == ownedGeneration, activeFollowEvidence === evidence, stopUnconfirmed {
            return .failed(.commandFailed)
        }
        guard owned() else { return .cancelled }
        do { try await confirmStop() } catch { return .failed(.commandFailed) }
        return owned() ? outcome : .cancelled
    }

    @discardableResult
    private func startNavigation(
        to goal: Vec2,
        stoppingAtForwardClearance: Double?,
        policy: (any PathAdmissibilityPolicy)?,
        cancellingCurrent: Bool = true,
        isFollowGoal: Bool = false
    ) -> Task<NavigationResult, Never> {
        if stopUnconfirmed { return Task { .failed(.commandFailed) } }
        if cancellingCurrent { cancel() }
        operationGeneration &+= 1
        replanCounter = 0
        activePolicy = policy
        guard let start = currentPose()?.position else {
            let result = NavigationResult.failed(.noPose)
            finish(result)
            return Task { result }
        }
        if let failure = planAndStore(from: start, to: goal, policy: policy) {
            let result = NavigationResult.failed(failure)
            finish(result)
            return Task { result }
        }
        RuntimeFileLog.append("nav_goal_start", fields: [
            "goal_x": Self.formatMeters(goal.x),
            "goal_y": Self.formatMeters(goal.y),
            "pose_x": Self.formatMeters(start.x),
            "pose_y": Self.formatMeters(start.y),
            "distance_to_goal": Self.formatMeters(start.distance(to: goal)),
            "target_stop_clearance": stoppingAtForwardClearance.map(Self.formatMeters) ?? "none"
        ])
        state = .driving
        liveFollowGoal = isFollowGoal ? goal : nil
        publishSafetyState(.moving)
        let task = Task {
            await drive(to: goal, stoppingAtForwardClearance: stoppingAtForwardClearance,
                        isFollowGoal: isFollowGoal)
        }
        loop = task
        return task
    }

    /// Rotate in place by `angle` radians (CCW positive, matching `Pose2D.yaw`) and wait
    /// for it to finish. A pure turn, no path planning — used by the mission agent to scan
    /// for something not currently in view (e.g. up to a full `2 * .pi` look-around).
    /// Uses the same command cadence/watchdog as `drive()`, but does not treat forward
    /// clearance as a hard stop because this is an in-place search turn, not forward motion.
    public func rotate(by angle: Double) async {
        _ = await rotateAndWait(by: angle)
    }

    public func rotateAndWait(by angle: Double) async -> NavigationResult {
        operationGeneration &+= 1
        let reservation = operationGeneration
        if stopConfirmation != nil {
            do { try await confirmStop() } catch { return .failed(.commandFailed) }
        } else {
            await cancelAndWait()
        }
        guard operationGeneration == reservation else { return .cancelled }
        guard !stopUnconfirmed else { return .failed(.commandFailed) }
        operationGeneration &+= 1
        guard let startYaw = currentPose()?.yaw else {
            let result = NavigationResult.failed(.noPose)
            finish(result)
            return result
        }
        let targetYaw = normalizeAngle(startYaw + angle)
        state = .driving
        publishSafetyState(.moving)
        let task = Task { await performRotate(to: targetYaw, mode: .continuous) }
        loop = task
        return await task.value
    }

    /// Rotate in short pulses for camera-based target search. Stopping between pulses
    /// prevents a 30-degree scan step from sweeping past the object before detection can
    /// process a stable frame.
    public func rotateForScan(by angle: Double) async {
        cancel()
        operationGeneration &+= 1
        guard let startYaw = currentPose()?.yaw else {
            state = .failed("No ARKit pose yet — move the device to establish tracking.")
            publishSafetyState(.failed(.noPose))
            return
        }
        let targetYaw = normalizeAngle(startYaw + angle)
        state = .driving
        publishSafetyState(.moving)
        let task = Task { await performRotate(to: targetYaw, mode: .scan) }
        loop = task
        _ = await task.value
    }

    public func rotateForFollowScan(by angle: Double) async -> NavigationResult {
        if FollowMotionTaskScope.evidence == nil {
            return await performFollowMotion(.scan(angle), context: nil).result
        }
        return await startFollowTurn(by: angle, purpose: .followScan)
    }

    /// Follow-only adaptive alignment. Generic rotation remains continuous.
    public func rotateForFollowAlignment(by angle: Double) async -> NavigationResult {
        if FollowMotionTaskScope.evidence == nil {
            return await performFollowMotion(.alignment(angle), context: nil).result
        }
        return await startFollowTurn(by: angle, purpose: .followAlignment)
    }

    private func startFollowTurn(by angle: Double, purpose: FollowMotionPurpose) async -> NavigationResult {
        guard !stopUnconfirmed else { return .failed(.commandFailed) }
        operationGeneration &+= 1
        let owner = operationGeneration
        let evidence = FollowMotionTaskScope.evidence
        evidence?.ownedGeneration = owner
        startFollowTurnSourceEvents()
        let subscription = followTurnSourceSubscriptionID
        defer {
            if followTurnSourceSubscriptionID == subscription {
                followTurnSourceTask?.cancel()
                followTurnSourceTask = nil
                followTurnSourceSubscriptionID = nil
            }
        }
        do { try await confirmStop() } catch { return .failed(.commandFailed) }
        // Preserve the actual stopped capture even when source admission rejects
        // it. This is evidence only; resolution remains absent until admission.
        if let recovery = FollowRecoveryScope.heading {
            evidence?.recovery = .init(postStopSource: followTurnSourceGate.latest ?? .unavailable,
                resolutionSource: nil, stageHeading: recovery.stageHeading,
                segmentHeading: nil, requestedDelta: nil, arrivalSource: nil,
                segmentArrived: false, stageArrived: nil, postStopReadUptime: sourceNow())
        }
        guard operationGeneration == owner, !Task.isCancelled, evidence?.fenced != true,
              recoveryAuthorized, let initialFence = followTurnStopFence else { return .cancelled }
        let selection = await awaitFollowTurnSource(after: initialFence, progress: .init())
        guard operationGeneration == owner, !Task.isCancelled, evidence?.fenced != true,
              recoveryAuthorized else { return .cancelled }
        let initial: NavigationPoseSample
        switch selection {
        case .sample(let sample): initial = sample
        case .failed(let reason): let result = NavigationResult.failed(reason); finish(result); return result
        case .cancelled: return .cancelled
        }
        guard angle.isFinite, let yaw = initial.pose?.yaw, (yaw + angle).isFinite else {
            return .failed(.trackingLost)
        }
        let target: Double
        if purpose == .followScan, let recovery = FollowRecoveryScope.heading {
            let delta: Double
            switch FollowReacquisitionPlanner.resolveAbsoluteStage(stageHeading: recovery.stageHeading, actualYaw: yaw,
                maximumSegment: recovery.maximumSegment) {
            case .turn(let resolvedDelta, let resolvedTarget): delta = resolvedDelta; target = resolvedTarget
            case .stageArrived: delta = 0; target = FollowReacquisitionPlanner.wrap(yaw)
            case .unavailable, .exhausted: return .cancelled
            }
            evidence?.recovery = .init(postStopSource: initial, resolutionSource: initial,
                stageHeading: recovery.stageHeading, segmentHeading: target, requestedDelta: delta,
                arrivalSource: nil, segmentArrived: false, stageArrived: nil,
                postStopReadUptime: sourceNow(), resolutionReadUptime: sourceNow())
        } else {
            target = FollowReacquisitionPlanner.wrap(yaw + angle)
        }
        evidence?.targetYaw = target
        let searchSweep = purpose == .followScan && evidence?.context.request?.isSearchSweep == true &&
            FollowRecoveryScope.heading == nil && FollowRecoveryScope.authorization == nil &&
            angle.isFinite && angle != 0 && abs(angle) <= .pi ? angle : nil
        let profile = FollowTurnBurstPlanner.Profile(purpose: purpose == .followAlignment ? .alignment : .scan,
            fastSearch: searchSweep != nil)
        state = .driving
        publishSafetyState(.moving)
        let task = Task { @MainActor in
            let runtime = FollowTurnRuntimeState(targetYaw: target, initial: initial,
                tolerance: searchSweep == nil ? profile.tolerance : 0, date: self.now(), settleWait: profile.settleWait)
            defer {
                if runtime.failure == .trackingLost, runtime.sourceRejection == "stale_source" {
                    evidence?.recordFailure(.trackingLost, cause: .poseSourceStale)
                }
                if let evidence, runtime.failure != .trackingLost {
                    let snapshot = runtime.progress.watchdog.diagnosticSnapshot(
                        distanceToGoal: runtime.progress.distanceToGoal, now: self.now())
                    evidence.scanTrace?.observed(snapshot, previous: snapshot,
                        sample: .init(sample: initial, uptime: self.sourceNow(), expectedGeneration: runtime.generation))
                }
            }
            if let evidence {
                let snapshot = runtime.progress.watchdog.diagnosticSnapshot(
                    distanceToGoal: runtime.progress.distanceToGoal, now: self.now())
                evidence.scanTrace?.observed(snapshot, previous: snapshot,
                    sample: .init(sample: initial, uptime: self.sourceNow(), expectedGeneration: runtime.generation))
            }
            let progressObserver = UUID()
            self.followTurnSourceObservers[progressObserver] = { sample in
                guard self.operationGeneration == owner, evidence?.fenced != true else { return }
                let previous = runtime.progress.watchdog.diagnosticSnapshot(
                    distanceToGoal: runtime.progress.distanceToGoal, now: self.now())
                runtime.observe(sample, uptime: self.sourceNow(), date: self.now())
                if let evidence {
                    if runtime.failure == .trackingLost {
                        if sample.rejection(at: self.sourceNow(), expectedGeneration: runtime.generation,
                            requireEnriched: true) != nil {
                            evidence.scanTrace?.unavailablePost(sample: .init(sample: sample, uptime: self.sourceNow(),
                                expectedGeneration: runtime.generation), evidence: evidence, latch: self.stopUnconfirmed)
                        }
                        return
                    }
                    evidence.scanTrace?.observed(runtime.progress.watchdog.diagnosticSnapshot(
                        distanceToGoal: runtime.progress.distanceToGoal, now: self.now()), previous: previous,
                        sample: .init(sample: sample, uptime: self.sourceNow(), expectedGeneration: runtime.generation))
                }
            }
            defer { self.followTurnSourceObservers.removeValue(forKey: progressObserver) }
            var depthWaitSpent = false
            var bracket: FollowTurnResponseBracket?
            let bracketObserver = UUID()
            defer { self.followTurnSourceObservers.removeValue(forKey: bracketObserver) }
            return await FollowTurnOperationExecutor.execute(targetYaw: target, generation: initial.frameID!.generation,
                operationID: evidence?.context.controllerOperationID ?? UInt64(owner), profile: profile, runtime: runtime,
                responseRateFloor: purpose == .followScan && evidence?.context.request?.scanResponseSeed?.sourceGeneration == initial.frameID!.generation
                    ? evidence!.context.request!.scanResponseSeed!.responseRate : 2 * .pi / 3,
                requestedSearchSweep: searchSweep,
                searchSweepProgress: { signed, travel in
                    evidence?.searchSweepTravel = travel
                    evidence?.searchSweepProgress = signed
                    evidence?.burstTrace?.searchSweepProgress(signed: signed, travel: travel)
                },
                uptime: self.sourceNow, now: self.now,
                authorized: { self.operationGeneration == owner && evidence?.fenced != true && self.recoveryAuthorized && !self.stopUnconfirmed },
                admit: { progress in
                    let refreshed = await self.refreshFollowTurnRuntime(runtime, owner: owner, evidence: evidence,
                        requireFreshAck: progress.hasSentCommand)
                    guard self.operationGeneration == owner, !Task.isCancelled, evidence?.fenced != true,
                           self.recoveryAuthorized else { return .cancelled }
                    if self.stopUnconfirmed { return .failed(.commandFailed) }
                    if let failure = runtime.failure { return .failed(failure) }
                    guard refreshed else { return .cancelled }
                    guard let sample = self.followTurnSourceGate.latest else { return .failed(.trackingLost) }
                    return .sample(sample)
                },
                burst: { command, budget, _, boundary, plannedSource in
                    guard self.operationGeneration == owner, evidence?.fenced != true, !Task.isCancelled,
                          self.recoveryAuthorized, !self.stopUnconfirmed else { throw CancellationError() }
                    let previouslyWaitedForDepth = depthWaitSpent
                    let prepared = try await self.prepareFollowTurnDepth(command, runtime: runtime, owner: owner,
                        evidence: evidence, waitSpent: &depthWaitSpent)
                    guard self.operationGeneration == owner, evidence?.fenced != true, !Task.isCancelled,
                          self.recoveryAuthorized, !self.stopUnconfirmed else { throw CancellationError() }
                    guard let start = self.followTurnSourceGate.latest else { throw CancellationError() }
                    if start.pose?.yaw != plannedSource.pose?.yaw {
                        if depthWaitSpent && !previouslyWaitedForDepth { throw FollowTurnPreparationError.replan }
                        evidence?.recordCommandReceipt(.init(httpStatus: nil, acknowledged: false,
                            acknowledgementUTC: nil, attempts: 0, outcome: "fenced"))
                        throw FollowTurnBurstTransportDenial.fenced
                    }
                    if prepared.arc {
                        guard let baseline = start.pose else { throw CancellationError() }
                        // Charge before any suspension; denied/uncertain requests
                        // never refund a reservation and permit unlimited creeping.
                        self.depthRecoveryAttempts += 1
                        let excursionObserver = UUID()
                        @MainActor func checkExcursion(_ sample: NavigationPoseSample) {
                            guard self.operationGeneration == owner, evidence?.fenced != true,
                                  let pose = sample.pose else { return }
                            if pose.position.distance(to: baseline.position) > 0.12 ||
                               abs(FollowReacquisitionPlanner.wrap(pose.yaw - baseline.yaw)) > 0.35 {
                                evidence?.recordFailure(.depthUnavailable(.blindSweptVolume), cause: .depthRecoveryExcursion)
                                runtime.fail(.depthUnavailable(.blindSweptVolume))
                                self.pendingFollowTurnBurst?.inhibit(at: self.sourceNow())
                            }
                        }
                        self.followTurnSourceObservers[excursionObserver] = checkExcursion
                        defer { self.followTurnSourceObservers.removeValue(forKey: excursionObserver) }
                        var receipt = try await self.executeFollowTurnBurst(prepared.command, requestedBudget: 0.080,
                            purpose: purpose, runtime: runtime, plannedYaw: start.pose?.yaw)
                        if let latest = self.followTurnSourceGate.latest { checkExcursion(latest) }
                        receipt.requiresReobservation = true
                        // Earlier pure-turn bursts in this operation are not a
                        // calibration/sweep completion for the translating outcome.
                        evidence?.measuredScanResponse = nil
                        evidence?.searchSweepProgress = nil
                        evidence?.searchSweepTravel = nil
                        return receipt
                    }
                     if let evidence {
                         let snapshot = runtime.progress.watchdog.diagnosticSnapshot(
                             distanceToGoal: runtime.progress.distanceToGoal, now: self.now())
                         evidence.scanTrace?.observed(snapshot, previous: snapshot,
                             sample: .init(sample: start, uptime: self.sourceNow(), expectedGeneration: runtime.generation))
                         evidence.scanTrace?.pulse(.init(sample: start, uptime: self.sourceNow(),
                             expectedGeneration: runtime.generation), command: command, evidence: evidence,
                             latch: self.stopUnconfirmed)
                     }
                    bracket = .init(start: start, at: self.sourceNow(), generation: initial.frameID!.generation,
                        boundary: boundary)
                    self.followTurnSourceObservers[bracketObserver] = { sample in
                        let health = self.sourceHealth?()
                        bracket?.collect(sample, at: self.sourceNow(), healthy: health == nil ||
                            (health?.trackingQuality == .normal && health?.generation == initial.frameID!.generation))
                    }
                     let receipt = try await self.executeFollowTurnBurst(command, requestedBudget: budget, purpose: purpose,
                         runtime: runtime, plannedYaw: plannedSource.pose?.yaw)
                    if let latest = self.followTurnSourceGate.latest {
                        bracket?.collect(latest, at: self.sourceNow(), healthy: true)
                    }
                    return receipt
                },
                stoppedSource: { fence, progress in
                    await self.awaitFollowTurnSource(after: fence, progress: progress, runtime: runtime)
                },
                diagnostic: { calibration, sample, decision, uptime in
                    evidence?.burstTrace?.completionPolicy(searchSweep: searchSweep)
                    evidence?.burstTrace?.planning(calibration, sample: sample, profile: profile, decision: decision, uptime: uptime)
                    if let evidence { evidence.scanTrace?.emit("burst_plan", evidence: evidence, latch: self.stopUnconfirmed) }
                },
                reduced: { response, reduction in
                    if purpose == .followScan, reduction.rejection == nil, reduction.signedResponse != nil,
                       reduction.calibration.responseRate.isFinite {
                        evidence?.measuredScanResponse = .init(sourceGeneration: initial.frameID!.generation,
                            responseRate: reduction.calibration.responseRate)
                    }
                    evidence?.burstTrace?.reduction(response, reduction)
                    if let evidence { evidence.scanTrace?.emit("burst_response", evidence: evidence, latch: self.stopUnconfirmed,
                        outcome: reduction.rejection == nil ? "accepted" : "rejected", reason: reduction.rejection) }
                },
                response: { receipt, budget, settled in
                    self.followTurnSourceObservers.removeValue(forKey: bracketObserver)
                    guard var captured = bracket, let fence = receipt.confirmedStopFence,
                          self.followTurnStopFence?.identity == fence.identity,
                          fence.operationGeneration == owner, fence.sourceGeneration == initial.frameID!.generation,
                          let obligation = receipt.stopObligationUptime else { return nil }
                     captured.collect(settled, at: self.sourceNow(), healthy: true, settled: true)
                     if let ingress = self.turnPoseEvidence, let first = captured.samples.first?.sequence,
                        let last = settled.frameID {
                         captured = captured.usingIngressEvidence(ingress(
                             .init(generation: initial.frameID!.generation, sequence: first), last))
                     }
                     evidence?.burstTrace?.evidenceDelivery(ingress: self.turnPoseEvidence != nil,
                         rejection: captured.ingressRejection)
                    return .init(operationID: evidence?.context.controllerOperationID ?? UInt64(owner),
                        generation: initial.frameID!.generation, targetYaw: target, clockDomain: "ar_system_uptime",
                        requestedBudget: budget, sendEntryUptime: receipt.send.sendEntryUptime,
                        sendResponseUptime: receipt.send.responseUptime, stopObligationUptime: obligation,
                        stopAcknowledgementUptime: fence.acknowledgementUptime, samples: captured.samples,
                         traversalUnambiguous: captured.unambiguous,
                         planningEvaluation: self.turnPoseEvidence == nil ? nil : captured.planningEvaluation,
                         stoppedEvaluation: self.turnPoseEvidence == nil ? nil : captured.stoppedEvaluation)
                })
        }
        loop = task
        let result = await task.value
        guard operationGeneration == owner else { return .cancelled }
        loop = nil
        if stopUnconfirmed { return .failed(.commandFailed) }
        guard !Task.isCancelled, evidence?.fenced != true, recoveryAuthorized else { return .cancelled }
        if let previous = evidence?.recovery, let arrival = followTurnSourceGate.latest {
            let readUptime = sourceNow()
            let valid = arrival.rejection(at: readUptime, expectedGeneration: initial.frameID!.generation,
                requireEnriched: true) == nil
            let segmentArrived = valid && arrival.pose.map {
                abs(FollowReacquisitionPlanner.wrap(target - $0.yaw)) <= profile.tolerance
            } == true
            let stageArrived = valid ? arrival.pose.map {
                abs(FollowReacquisitionPlanner.wrap(previous.stageHeading - $0.yaw)) <= profile.tolerance
            } : nil
            evidence?.recovery = .init(postStopSource: previous.postStopSource,
                resolutionSource: previous.resolutionSource, stageHeading: previous.stageHeading,
                segmentHeading: target, requestedDelta: previous.requestedDelta, arrivalSource: arrival,
                segmentArrived: result == .arrived && segmentArrived,
                stageArrived: result == .arrived ? stageArrived : nil,
                postStopReadUptime: previous.postStopReadUptime, resolutionReadUptime: previous.resolutionReadUptime,
                arrivalReadUptime: readUptime)
            if result == .arrived, !valid { return .failed(.trackingLost) }
        }
        finish(result)
        return result
    }

    /// Invalidate motion before waiting for the loop and an acknowledged motor stop.
    /// Detection uses this synchronous fence before scheduling serialized confirmation.
    /// Retain the loop so confirmation still drains all suspended transport work.
    func inhibitFollowScanContinuation(origin: FollowMotionStopOrigin) {
        guard let evidence = activeFollowEvidence,
              evidence.context.purpose == .followScan || evidence.context.purpose == .followAlignment else { return }
        pendingFollowTurnBurst?.inhibit()
        evidence.fenced = true
        evidence.scanTrace?.cancel(origin: origin.rawValue, evidence: evidence, latch: stopUnconfirmed)
        operationGeneration &+= 1
        loop?.cancel()
    }

    public func stopAndConfirm() async throws {
        pendingFollowTurnBurst?.inhibit()
        let evidence = activeFollowEvidence ?? terminalFollowEvidence
        let origin = FollowMotionTaskScope.stopOrigin.rawValue
        evidence?.fenced = true
        if let evidence { evidence.scanTrace?.cancel(origin: origin, evidence: evidence, latch: stopUnconfirmed) }
        operationGeneration &+= 1
        try await confirmStop(retryingFailedStop: true, evidence: evidence, origin: origin)
    }

    var followSourceUptime: TimeInterval { sourceNow() }

    private func confirmStop(retryingFailedStop: Bool = false, evidence: FollowMotionOperationEvidence? = nil,
                             origin: String = "independent") async throws {
        let captured = evidence ?? FollowMotionTaskScope.evidence
        let captureSource = captured?.context.purpose == .followScan || captured?.context.purpose == .followAlignment ||
            FollowTurnSourceScope.required || followTurnSourceTask != nil
        let trace = captured?.scanTrace
        let token = captured.flatMap { trace?.beginStop(origin: origin, evidence: $0, latch: stopUnconfirmed) }
        captured?.recordStop(.pending)
        let generation = operationGeneration
        let previousStop = stopConfirmation
        let previousLoop = loop
        let pendingBurst = pendingFollowTurnBurstDrain
        previousLoop?.cancel()
        let receiptCapture = FollowMotionStopReceiptCapture()
        let callerCapture = FollowMotionTaskScope.stopReceiptCapture
        let confirmation = Task { @MainActor in
            // Serialize stops so an older stop cannot race a newer motion command.
            if retryingFailedStop {
                _ = try? await previousStop?.value
            } else {
                try await previousStop?.value
            }
            _ = await previousLoop?.value
            await pendingBurst?.wait()
            try await FollowMotionTaskScope.$evidence.withValue(captured) {
                try await FollowMotionTaskScope.$stopReceiptCapture.withValue(receiptCapture) { try await stopRover() }
            }
            // Capture in the returning stop task, before its waiter or another actor hop.
            if captureSource {
                let acknowledgementUptime = sourceNow()
                if let snapshot = sourceStopSnapshot?() { followTurnSourceGate.ingest(snapshot) }
                if let highWater = sourceHighWater?() { followTurnSourceGate.include(highWater) }
                let fence = followTurnSourceGate.fence(at: acknowledgementUptime,
                    operationGeneration: generation, context: captured?.context ?? .unknown)
                receiptCapture.sourceFence = fence
                callerCapture?.recordSourceFence(fence)
                callerCapture?.receipt = receiptCapture.receipt
                captured?.turnStopFence = fence
                if operationGeneration == generation { followTurnStopFence = fence }
            }
        }
        stopConfirmation = confirmation
        do {
            try await confirmation.value
        } catch {
            captured?.recordStop(.failed)
            stopUnconfirmed = true
            if operationGeneration == generation {
                state = .failed("Rover stop could not be confirmed.")
                publishSafetyState(.failed(.commandFailed), evidence: captured)
            }
            if let captured, let token {
                trace?.endStop(token, origin: origin, evidence: captured, receipt: receiptCapture.receipt,
                    latch: stopUnconfirmed, outcome: "failed")
            }
            throw error
        }
        guard operationGeneration == generation else {
            if let captured, let token {
                trace?.endStop(token, origin: origin, evidence: captured, receipt: receiptCapture.receipt,
                    latch: stopUnconfirmed, outcome: "acknowledged")
            }
            return
        }
        captured?.recordStop(.confirmed)
        stopUnconfirmed = false
        loop = nil
        activePolicy = nil
        path = []
        state = .idle
        publishSafetyState(.idle)
        if let captured, let token {
            trace?.endStop(token, origin: origin, evidence: captured, receipt: receiptCapture.receipt,
                latch: stopUnconfirmed, outcome: "acknowledged")
        }
    }

    private func rotationStop(origin: String) async throws {
        let evidence = FollowMotionTaskScope.evidence
        let owner = operationGeneration
        let captureSource = evidence?.context.purpose == .followScan || evidence?.context.purpose == .followAlignment ||
            FollowTurnSourceScope.required || followTurnSourceTask != nil
        let token = evidence.flatMap { $0.scanTrace?.beginStop(origin: origin, evidence: $0, latch: stopUnconfirmed) }
        let receiptCapture = FollowMotionStopReceiptCapture()
        do {
            try await FollowMotionTaskScope.$stopReceiptCapture.withValue(receiptCapture) { try await stopRover() }
            if captureSource {
                let acknowledgementUptime = sourceNow()
                if let snapshot = sourceStopSnapshot?() { followTurnSourceGate.ingest(snapshot) }
                if let highWater = sourceHighWater?() { followTurnSourceGate.include(highWater) }
                let fence = followTurnSourceGate.fence(at: acknowledgementUptime,
                    operationGeneration: owner, context: evidence?.context ?? .unknown)
                receiptCapture.sourceFence = fence
                evidence?.turnStopFence = fence
                if operationGeneration == owner { followTurnStopFence = fence }
            }
            if operationGeneration == owner, !Task.isCancelled, evidence?.fenced != true {
                evidence?.recordStop(.confirmed)
            }
            if let evidence, let token {
                evidence.scanTrace?.endStop(token, origin: origin, evidence: evidence, receipt: receiptCapture.receipt,
                    latch: stopUnconfirmed, outcome: "acknowledged")
            }
        } catch {
            // Burst confirmation drains independently of the cancelled loop.
            // Losing its captured owner still interrupts the old response; only
            // the replacing serialized confirmation can decide the new latch.
            let interrupted = Task.isCancelled || (operationGeneration != owner && evidence?.fenced == true)
            if origin == "pulse" || origin == "cleanup", operationGeneration == owner, !interrupted {
                // An owned cleanup failure has the same stop obligation as a pulse.
                // A stale operation cannot latch or publish over its replacement.
                stopUnconfirmed = true
                evidence?.recordStop(.failed)
            }
            if let evidence, let token {
                evidence.scanTrace?.endStop(token, origin: origin, evidence: evidence, receipt: receiptCapture.receipt, latch: stopUnconfirmed,
                    outcome: interrupted ? "cancelled" : "failed")
            }
            throw error
        }
    }

    /// Stop and clear the current goal.
    /// Synchronous cross-controller fence. The owner must drain performFollowMotion
    /// and confirm the stop before admitting another motion operation.
    func inhibitForHandoff() {
        pendingFollowTurnBurst?.inhibit()
        activeFollowEvidence?.fenced = true
        operationGeneration &+= 1
        liveFollowGoal = nil
        loop?.cancel()
    }

    public func cancel() {
        pendingFollowTurnBurst?.inhibit()
        let captured = activeFollowEvidence
        if let evidence = captured {
            evidence.fenced = true
            evidence.scanTrace?.cancel(origin: "cancel", evidence: evidence, latch: stopUnconfirmed)
        }
        operationGeneration &+= 1
        loop?.cancel()
        loop = nil
        activePolicy = nil
        path = []
        let previousStop = stopConfirmation
        let pendingBurst = pendingFollowTurnBurstDrain
        stopConfirmation = Task { @MainActor in
            _ = try? await previousStop?.value
            await pendingBurst?.wait()
            try await FollowMotionTaskScope.$evidence.withValue(captured) {
                try await rotationStop(origin: "independent")
            }
        }
        // Without this, an external cancel (e.g. a hard-stop bypassing the brain) leaves
        // `state` at `.driving` forever, so anything polling `state == .driving` to know
        // when motion has settled never returns.
        state = .idle
        publishSafetyState(.idle)
    }

    public func cancelAndWait() async {
        let generation = operationGeneration
        let task = loop
        let pendingBurst = pendingFollowTurnBurstDrain
        pendingFollowTurnBurst?.inhibit()
        task?.cancel()
        _ = await task?.value
        await pendingBurst?.wait()
        guard operationGeneration == generation else { return }
        try? await stopRover()
        guard operationGeneration == generation else { return }
        loop = nil
        activePolicy = nil
        path = []
        state = .idle
        publishSafetyState(.idle)
    }

    // MARK: - Loop

    /// Revalidate a completed actual ACK read against current Date/source facts.
    /// Rotation retains its existing nil-IMU / no-forward-obstacle policy.
    private func validateFollowTurnRuntime(_ runtime: FollowTurnRuntimeState, owner: UInt,
                                          evidence: FollowMotionOperationEvidence?, requireFreshAck: Bool? = nil) -> Bool {
        guard !Task.isCancelled, operationGeneration == owner, evidence?.fenced != true,
              !stopUnconfirmed, recoveryAuthorized else { return false }
        runtime.expire(at: now())
        if let failure = runtime.failure {
            evidence?.recordFailure(failure)
            return false
        }
        guard validateSearchClearance(evidence?.context.request, evidence: evidence) else {
            runtime.fail(.depthUnavailable(.blindSweptVolume)); return false
        }
        guard let sample = followTurnSourceGate.latest else { runtime.failTracking("missing_source"); return false }
        if let rejection = sample.rejection(at: sourceNow(), expectedGeneration: runtime.generation, requireEnriched: true) {
            runtime.failTracking(rejection)
            if runtime.failure == .trackingLost, runtime.sourceRejection == "stale_source" {
                evidence?.recordFailure(.trackingLost, cause: .poseSourceStale)
            }
            return false
        }
        guard let fence = followTurnStopFence, fence.operationGeneration == owner,
              fence.sourceGeneration == runtime.generation,
              sample.sourceTimestamp! > fence.acknowledgementUptime,
              sample.sourceTimestamp! > (fence.highestSourceTimestamp ?? -.infinity),
              sample.frameID!.sequence > (fence.highestSequence ?? 0) else {
            runtime.fail(.trackingLost); return false
        }
        if let health = sourceHealth?(), health.trackingQuality != .normal || health.generation != runtime.generation {
            runtime.fail(.trackingLost); return false
        }
        let validationDate = now()
        if runtime.requiresKnownAck {
            guard let ack = runtime.lastAck else { runtime.fail(.commsLost); return false }
            let age = validationDate.timeIntervalSince(ack)
            guard age.isFinite, age >= 0, age <= guardLayer.watchdogTimeout else {
                runtime.fail(.commsLost); return false
            }
        }
        switch guardLayer.evaluate(forwardClearance: currentForwardClearance(), lastAckAt: runtime.lastAck,
            now: validationDate, feedback: nil, requireFreshAck: requireFreshAck ?? runtime.progress.hasSentCommand,
            checkForwardObstacle: false) {
        case .go: return true
        case .stopCommsLost: runtime.fail(.commsLost)
        case .stopTipping: runtime.fail(.tipping)
        case .stopObstacle: runtime.fail(.obstacle)
        }
        return false
    }

    /// At most one outstanding read per owner. The read has no motor/runtime
    /// authority, and is never joined on interruption. Late completion only fills
    /// its own immutable-owner slot; it cannot clear or wake a newer owner's slot.
    private func interruptibleFollowTurnAck(owner: UInt, deadline: Double = .infinity,
                                           progress: () -> FollowTurnWaitingProgress,
                                           interrupted: () -> Bool) async -> (completed: Bool, value: Date?) {
        let read: FollowTurnAckRead
        if let existing = followTurnAckRead, existing.owner == owner {
            read = existing
        } else {
            read = FollowTurnAckRead(owner: owner)
            followTurnAckRead = read
            let getter = currentLastAck
            Task { @MainActor [weak self] in
                read.value = await getter()
                read.completed = true
                guard let self, self.followTurnAckRead === read, self.operationGeneration == owner else { return }
                self.wakeFollowTurnSourceWaiters()
            }
        }
        while true {
            guard !Task.isCancelled, operationGeneration == owner, !interrupted(), sourceNow() < deadline else {
                return (false, nil)
            }
            if read.completed {
                if followTurnAckRead === read { followTurnAckRead = nil }
                return (true, read.value)
            }
            var wake = deadline
            if let sample = followTurnSourceGate.latest, let timestamp = sample.sourceTimestamp {
                wake = min(wake, (timestamp + 0.500).nextUp)
            }
            if let remaining = progress().remaining(at: now()) { wake = min(wake, sourceNow() + remaining) }
            if let authorization = FollowRecoveryScope.authorization {
                wake = min(wake, sourceNow() + max(0, authorization.deadline - authorization.now()))
            }
            await waitForFollowTurnSource(until: wake, while: { !read.completed })
        }
    }

    private func refreshFollowTurnRuntime(_ runtime: FollowTurnRuntimeState, owner: UInt,
                                         evidence: FollowMotionOperationEvidence?, requireFreshAck: Bool? = nil,
                                         deadline: Double = .infinity, stopTriggered: () -> Bool = { false }) async -> Bool {
        // Freeze/check the existing progress epoch before the suspension; an ACK
        // return or a cached source read must never start a replacement epoch.
        runtime.expire(at: now())
        guard runtime.failure == nil, !Task.isCancelled, operationGeneration == owner,
              evidence?.fenced != true, !stopUnconfirmed, recoveryAuthorized else { return false }
        let ack = await interruptibleFollowTurnAck(owner: owner, deadline: deadline, progress: { runtime.progress }) {
            runtime.expire(at: self.now())
            return stopTriggered() || !self.validateFollowTurnRuntime(runtime, owner: owner, evidence: evidence,
                requireFreshAck: false)
        }
        guard ack.completed else { return false }
        guard !Task.isCancelled, operationGeneration == owner, evidence?.fenced != true,
              !stopUnconfirmed, recoveryAuthorized else { return false }
        runtime.lastAck = ack.value
        return validateFollowTurnRuntime(runtime, owner: owner, evidence: evidence, requireFreshAck: requireFreshAck)
    }

    /// A shared one-burst execution boundary. Stop runs independently of caller
    /// cancellation; a replaced owner delegates cleanup to the replacing stop.
    func executeFollowTurnBurst(_ command: WheelCommand, requestedBudget: Double,
                                purpose: FollowMotionPurpose,
                                runtime: FollowTurnRuntimeState? = nil,
                                plannedYaw: Double? = nil, sendValidUntil: Double? = nil) async throws -> FollowTurnBurstExecutionReceipt {
        let owner = operationGeneration
        let evidence = FollowMotionTaskScope.evidence
        var ownedSubscription: UUID?
        var observation: FollowTurnBurstObservation?
        var sourceTriggeredStop: Double?
        var stopAdmissionUptime: Double?
        var completedSend: FollowTurnBurstSendReceipt?
        if let target = evidence?.targetYaw {
            if followTurnSourceTask == nil {
                startFollowTurnSourceEvents()
                ownedSubscription = followTurnSourceSubscriptionID
            }
            if let start = followTurnSourceGate.latest {
                observation = .init(targetYaw: target,
                    tolerance: runtime?.tolerance ?? FollowTurnBurstPlanner.Profile(purpose: purpose == .followAlignment ? .alignment : .scan).tolerance,
                    start: start, uptime: sourceNow(), preferPositiveHalfTurn: command.right > command.left)
            }
        }
        defer {
            if let ownedSubscription, followTurnSourceSubscriptionID == ownedSubscription {
                followTurnSourceTask?.cancel()
                followTurnSourceTask = nil
                followTurnSourceSubscriptionID = nil
            }
        }
        let observerID = UUID()
        if observation != nil {
            followTurnSourceObservers[observerID] = { sample in
                guard self.pendingFollowTurnBurst != nil || completedSend != nil else { return }
                if observation?.observe(sample, at: self.sourceNow()) == true, sourceTriggeredStop == nil {
                    sourceTriggeredStop = self.sourceNow()
                    if let evidence { evidence.burstTrace?.obligation(at: self.sourceNow(),
                        reason: observation?.triggerReason ?? "source", pending: self.pendingFollowTurnBurst != nil,
                        evidence: evidence, latch: self.stopUnconfirmed) }
                }
            }
        }
        defer { followTurnSourceObservers.removeValue(forKey: observerID) }
        var execution = try await FollowTurnBurstExecutor.execute(command: command, budget: requestedBudget,
            send: {
                let receipt = await self.sendFollowTurnBurst($0, requestedBudget: $1, purpose: purpose,
                    runtime: runtime, plannedYaw: plannedYaw, sendValidUntil: sendValidUntil)
                completedSend = receipt
                return receipt
            },
            waitRemaining: { receipt in
                var enteredWait = false
                var waitStart: Double?
                var requestedWait = 0.0
                defer {
                    let waitEnd = self.sourceNow()
                    evidence?.burstTrace?.additionalWait(requested: requestedWait, start: waitStart ?? waitEnd, end: waitEnd)
                    if let evidence { evidence.burstTrace?.waitEnded(reason: observation?.triggerReason ??
                        (runtime?.failure.map { String(describing: $0) } ??
                            (Task.isCancelled || evidence.fenced || self.operationGeneration != owner ? "fence" :
                                (self.sourceNow() >= receipt.deadline ? "budget_expiry" : "health_or_authority"))),
                        evidence: evidence, latch: self.stopUnconfirmed) }
                    if enteredWait, let evidence {
                        evidence.scanTrace?.endWait("pulse_wait", evidence: evidence, latch: self.stopUnconfirmed,
                            interrupted: Task.isCancelled || evidence.fenced || self.operationGeneration != owner)
                    }
                }
                while self.sourceNow() < receipt.deadline {
                    runtime?.expire(at: self.now())
                    if runtime?.failure != nil { return }
                    if sourceTriggeredStop != nil { return }
                    if let runtime, !(await self.refreshFollowTurnRuntime(runtime, owner: owner, evidence: evidence,
                        deadline: receipt.deadline, stopTriggered: { sourceTriggeredStop != nil })) { return }
                    guard !Task.isCancelled, self.operationGeneration == owner, !self.stopUnconfirmed,
                          evidence?.fenced != true, self.recoveryAuthorized else { return }
                    var deadline = receipt.deadline
                    if let remaining = runtime?.progress.remaining(at: self.now()) {
                        deadline = min(deadline, self.sourceNow() + remaining)
                    }
                    if var current = observation {
                        guard let sample = self.followTurnSourceGate.latest,
                              sample.rejection(at: self.sourceNow(), expectedGeneration: current.generation,
                                  requireEnriched: true) == nil else { return }
                        if let health = self.sourceHealth?(), health.trackingQuality != .normal || health.generation != current.generation {
                            return
                        }
                        if current.observe(sample, at: self.sourceNow()) { return }
                        observation = current
                        deadline = min(deadline, (sample.sourceTimestamp! + 0.500).nextUp)
                    }
                    if let authorization = FollowRecoveryScope.authorization {
                        deadline = min(deadline, self.sourceNow() + max(0, authorization.deadline - authorization.now()))
                    }
                    if !enteredWait, let evidence {
                        waitStart = self.sourceNow()
                        requestedWait = max(0, deadline - self.sourceNow())
                        evidence.scanTrace?.beginWait("pulse_wait", duration: max(0, deadline - self.sourceNow()),
                            evidence: evidence, latch: self.stopUnconfirmed)
                        enteredWait = true
                    }
                    await self.waitForFollowTurnSource(until: deadline)
                }
            },
            stop: {
                guard self.operationGeneration == owner else { throw CancellationError() }
                runtime?.expire(at: self.now())
                if purpose != .followReady, completedSend?.definitePreSendExpiry == true, runtime?.failure == nil,
                   !Task.isCancelled, evidence?.fenced != true, self.recoveryAuthorized,
                   !self.stopUnconfirmed, let evidence {
                    evidence.recordFailure(.rotationResolutionInsufficient, cause: .burstPreSendExpired)
                    if !evidence.emittedFailure, let failure = evidence.failure(source: .stream) {
                        evidence.scanTrace?.failure(.rotationResolutionInsufficient, evidence: evidence,
                            latch: self.stopUnconfirmed)
                        self.deliverFollowFailure(failure)
                        evidence.emittedFailure = true
                    }
                }
                if let runtime, let reason = runtime.failure, let evidence,
                   !Task.isCancelled, !evidence.fenced, !evidence.emittedFailure {
                    // Preserve the actual terminal cause while stop is still
                    // pending. A later failed stop overrides motion permission,
                    // but must not erase this captured primary failure.
                    let snapshot = runtime.progress.watchdog.diagnosticSnapshot(
                        distanceToGoal: runtime.progress.distanceToGoal, now: self.now())
                    if let sample = self.followTurnSourceGate.latest {
                        evidence.scanTrace?.observed(snapshot, previous: snapshot,
                            sample: .init(sample: sample, uptime: self.sourceNow(), expectedGeneration: runtime.generation))
                    }
                    self.finish(.failed(reason))
                }
                stopAdmissionUptime = self.sourceNow()
                if let evidence {
                    evidence.burstTrace?.obligation(at: self.sourceNow(),
                        reason: runtime?.failure.map { String(describing: $0) } ?? (evidence.fenced ? "fence" : "budget_expiry"),
                        pending: false, evidence: evidence, latch: self.stopUnconfirmed)
                    evidence.burstTrace?.stopAdmission(at: self.sourceNow())
                }
                let confirmation = Task { @MainActor in
                    guard self.operationGeneration == owner else { throw CancellationError() }
                    try await FollowTurnSourceScope.$required.withValue(true) {
                        try await self.rotationStop(origin: "pulse")
                    }
                    guard self.operationGeneration == owner else { throw CancellationError() }
                    return self.followTurnStopFence
                }
                return try await confirmation.value
            })
        execution.stopObligationUptime = min(execution.send.stopObligationUptime ?? .infinity,
            sourceTriggeredStop ?? .infinity, execution.send.deadline, stopAdmissionUptime ?? .infinity)
        if let evidence, let fence = execution.confirmedStopFence, let obligation = execution.stopObligationUptime {
            evidence.burstTrace?.stopConfirmed(fence, obligation: obligation, evidence: evidence, latch: stopUnconfirmed)
        }
        return execution
    }

    /// Internal sender seam; Task 4 supplies planner/source authority and serialized stop ownership.
    func sendFollowTurnBurst(_ command: WheelCommand, requestedBudget: Double,
                              purpose: FollowMotionPurpose,
                              runtime: FollowTurnRuntimeState? = nil,
                              plannedYaw: Double? = nil, sendValidUntil: Double? = nil) async -> FollowTurnBurstSendReceipt {
        let owner = operationGeneration
        let evidence = FollowMotionTaskScope.evidence
        let readyRecovery = purpose == .followReady ? readyRecoveryPulse : nil
        let bypassAcquisitionDepth = permitsAcquisitionDepthBypass(command, purpose: purpose, evidence: evidence)
        let isRecoveryArc = (purpose == .followScan || purpose == .followAlignment) &&
            command.left > 0 && command.right > 0
        if isRecoveryArc { runtime?.requiresKnownAck = true }
        let requireFreshAckForSend = isRecoveryArc || (runtime?.progress.hasSentCommand ?? false)
        if let runtime, !validateFollowTurnRuntime(runtime, owner: owner, evidence: evidence) {
            let entry = sourceNow()
            return .init(sendEntryUptime: entry, deadline: entry + requestedBudget, responseUptime: entry,
                result: .init(receipt: .unknown, failure: FollowTurnBurstTransportDenial.fenced), stopObligation: true)
        }
        var entry = sourceNow()
        var deadline = entry + requestedBudget
        var depthValidUntil = Double.infinity
        guard pendingFollowTurnBurst == nil else {
            return .init(sendEntryUptime: entry, deadline: deadline, responseUptime: entry,
                result: .init(receipt: .init(httpStatus: nil, acknowledged: false, acknowledgementUTC: nil,
                    attempts: 0, outcome: "fenced"), failure: FollowTurnBurstTransportDenial.fenced),
                stopObligation: false)
        }
        guard entry.isFinite, requestedBudget.isFinite, requestedBudget > 0, requestedBudget <= 0.080,
              deadline.isFinite, deadline > entry else {
            return .init(sendEntryUptime: entry, deadline: deadline, responseUptime: entry,
                result: .init(receipt: .init(httpStatus: nil, acknowledged: false, acknowledgementUTC: nil,
                    attempts: 0, outcome: "invalid_budget"), failure: FollowTurnBurstTransportDenial.invalidBudget),
                stopObligation: false)
        }
        let fence = FollowTurnBurstFence()
        let grant = evidence?.context.request?.searchClearance
        let grantObserver = UUID()
        grant?.revocationObservers[grantObserver] = { [weak grant] in
            if let failure = runtime?.failure { evidence?.recordFailure(failure) }
            if let grant { self.recordSearchClearanceRejection(grant, evidence: evidence) }
            fence.inhibit(at: self.sourceNow())
        }
        defer { grant?.revocationObservers.removeValue(forKey: grantObserver) }
        // Raw projection completes independently of pose callbacks. Subscribe
        // before arming; new hazards may only revoke this immutable fence, never
        // refresh the original depth/clearance expiry or reauthorize a veto.
        let removeDepthObserver: (() -> Void)? = (grant == nil || bypassAcquisitionDepth) ? nil : observeDepthSafety? { snapshot in
            guard self.operationGeneration == owner, evidence?.fenced != true else { return }
            let observation = snapshot.map { DepthSafetyEvaluator.evaluate($0, command: command,
                now: self.sourceNow(), operatorClearedRotation: true) }
                ?? .unavailable(.missingRawDepth, sampleAge: .infinity, motionClass: .classify(command))
            guard case .allow(let accepted, _) = self.guardLayer.evaluate(command: command, depthSafety: observation),
                  accepted == command else {
                let failure: NavigationFailure
                switch observation.state {
                case .stop: failure = .obstacle
                case .unavailable(let reason): failure = .depthUnavailable(reason)
                case .caution: failure = .obstacle
                case .clear: failure = .depthUnavailable(.malformedDepth)
                }
                evidence?.recordFailure(failure)
                runtime?.fail(failure)
                fence.inhibit(at: self.sourceNow())
                return
            }
        }
        defer { removeDepthObserver?() }
        let arming = FollowTurnBurstArming()
        evidence?.burstTrace?.prepareSender(budget: requestedBudget, uptime: entry, settleWait: runtime?.settleWait ?? 0.300)
        let drain = FollowTurnBurstDrain()
        let transportCapture = FollowTurnTransportCapture()
        // The shared follow executor supplies the frozen target through captured
        // operation evidence. Generic senders never subscribe to follow source.
        var observation: FollowTurnBurstObservation?
        var ownedSubscription: UUID?
        var sourceStopped = false
        if (purpose == .followAlignment || purpose == .followScan), let target = evidence?.targetYaw {
            if followTurnSourceTask == nil {
                startFollowTurnSourceEvents()
                ownedSubscription = followTurnSourceSubscriptionID
            }
            if let start = followTurnSourceGate.latest {
                observation = .init(targetYaw: target,
                    tolerance: runtime?.tolerance ?? FollowTurnBurstPlanner.Profile(purpose: purpose == .followAlignment ? .alignment : .scan).tolerance,
                    start: start, uptime: entry, preferPositiveHalfTurn: command.right > command.left)
            }
        }
        // Production supplies the actual planning source captured BEFORE any
        // planner/pulse diagnostics. Standalone sender seams bootstrap source first.
        let preparedYaw = plannedYaw ?? followTurnSourceGate.latest?.pose?.yaw
        let recovery = FollowRecoveryScope.authorization
        // Validate controller-owned state before arming, and publish only its
        // expiring value facts to the transport. No MainActor round trip belongs
        // in the tiny burst budget. Source ingress can refresh or revoke it.
        func refreshTransportAuthority() -> Bool {
            guard !Task.isCancelled, self.operationGeneration == owner,
                  evidence?.fenced != true, !self.stopUnconfirmed else { return false }
            if let recovery, !recovery.authorized(at: recovery.now()) { return false }
            if let runtime, !self.validateFollowTurnRuntime(runtime, owner: owner, evidence: evidence,
                requireFreshAck: requireFreshAckForSend) { return false }
            let time = self.sourceNow()
            let date = self.now()
            if let readyRecovery {
                guard self.readyRecoveryPulse?.id == readyRecovery.id,
                      readyRecovery.permits(command, sample: self.readFollowPose(), now: time),
                      self.operationGeneration == owner, evidence?.fenced != true else { return false }
            }
            guard self.validateSearchClearance(evidence?.context.request, evidence: evidence) else { return false }
            var expiry = min(sendValidUntil ?? .infinity, depthValidUntil)
            if let readyRecovery { expiry = min(expiry, readyRecovery.expiresAt) }
            if let grant { expiry = min(expiry, grant.expiresAt) }
            if let runtime {
                guard let timestamp = self.followTurnSourceGate.latest?.sourceTimestamp else { return false }
                expiry = min(expiry, (timestamp + 0.500).nextUp)
                if let remaining = runtime.progress.remaining(at: date) {
                    expiry = min(expiry, time + remaining)
                }
                if requireFreshAckForSend, let ack = runtime.lastAck {
                    let remaining = self.guardLayer.watchdogTimeout - date.timeIntervalSince(ack)
                    expiry = min(expiry, (time + remaining).nextUp)
                }
            }
            if let recovery {
                expiry = min(expiry, time + max(0, recovery.deadline - recovery.now()))
            }
            return fence.publishValidity(from: time, untilExclusive: expiry)
        }
        defer {
            if let ownedSubscription, followTurnSourceSubscriptionID == ownedSubscription {
                followTurnSourceTask?.cancel()
                followTurnSourceTask = nil
                followTurnSourceSubscriptionID = nil
            }
        }
        let observerID = UUID()
        let readyObserverID = UUID()
        if let readyRecovery {
            if followTurnSourceTask == nil {
                startFollowTurnSourceEvents()
                ownedSubscription = followTurnSourceSubscriptionID
            }
            followTurnSourceObservers[readyObserverID] = { sample in
                if !readyRecovery.permits(command, sample: sample, now: self.sourceNow()) {
                    fence.inhibit(at: self.sourceNow())
                }
            }
        }
        defer { followTurnSourceObservers.removeValue(forKey: readyObserverID) }
        if observation != nil {
            followTurnSourceObservers[observerID] = { sample in
                if let grant, let request = evidence?.context.request {
                    guard grant.validate(sample: sample, session: request.sessionGeneration, now: self.sourceNow()) else {
                        fence.inhibit(at: self.sourceNow()); return
                    }
                }
                guard arming.epoch != nil else { return }
                if grant != nil && !bypassAcquisitionDepth {
                    let freshDepth = FollowSearchRotationScope.$authorized.withValue(true) { self.depthSafetyObservation(command) }
                    guard case .allow(let accepted, _) = self.guardLayer.evaluate(command: command, depthSafety: freshDepth),
                          accepted == command else {
                        let failure: NavigationFailure
                        switch freshDepth.state {
                        case .stop: failure = .obstacle
                        case .unavailable(let reason): failure = .depthUnavailable(reason)
                        case .caution: failure = .obstacle
                        case .clear: failure = .depthUnavailable(.malformedDepth)
                        }
                        evidence?.recordFailure(failure)
                        runtime?.fail(failure)
                        fence.inhibit(at: self.sourceNow()); return
                    }
                }
                runtime?.observe(sample, uptime: self.sourceNow(), date: self.now())
                if runtime?.failure != nil { fence.inhibit(at: self.sourceNow()); return }
                guard self.operationGeneration == owner, evidence?.fenced != true,
                       !self.stopUnconfirmed, self.recoveryAuthorized else { fence.inhibit(at: self.sourceNow()); return }
                guard sample.rejection(at: self.sourceNow(), expectedGeneration: observation?.generation,
                           requireEnriched: true) == nil else { fence.inhibit(at: self.sourceNow()); return }
                if observation?.observe(sample, at: self.sourceNow()) == true {
                    sourceStopped = true
                    if let reason = observation?.triggerReason.flatMap(FollowTurnTargetStop.init(rawValue:)) {
                        fence.inhibitForTarget(reason, at: self.sourceNow())
                    } else { fence.inhibit(at: self.sourceNow()) }
                    if let evidence { evidence.burstTrace?.obligation(at: self.sourceNow(),
                        reason: observation?.triggerReason ?? "source", pending: true, evidence: evidence, latch: self.stopUnconfirmed) }
                } else if !refreshTransportAuthority() {
                    fence.inhibit(at: self.sourceNow())
                }
            }
        }
        defer { followTurnSourceObservers.removeValue(forKey: observerID) }
        let authorization = FollowTurnBurstAuthorization(operationID: evidence?.context.controllerOperationID ?? UInt64(owner),
            preparedFence: fence, uptime: transportUptime,
            didEnterAttempt: { transportCapture.record($0) },
            transportCapture: transportCapture)
        let selected = purpose == .followAlignment || purpose == .followScan || purpose == .followReady ? authorization : nil
        let monitor = Task { @MainActor in
            guard let epoch = await arming.wait(), !Task.isCancelled else { return }
            let deadline = epoch.deadline
            runtime?.expire(at: now())
            if runtime?.failure != nil { fence.inhibit(at: sourceNow()); return }
            if sourceNow() >= deadline {
                fence.inhibit(at: deadline)
                if let evidence { evidence.burstTrace?.obligation(at: deadline, observedAt: sourceNow(), reason: "budget_expiry", pending: true,
                    evidence: evidence, latch: stopUnconfirmed) }
                if runtime == nil { return }
            }
            if var observation {
                while !Task.isCancelled, fence.authorized || runtime != nil {
                    runtime?.expire(at: now())
                    if runtime?.failure != nil { fence.inhibit(at: sourceNow()); return }
                    if sourceStopped { return }
                    if let runtime, !(await refreshFollowTurnRuntime(runtime, owner: owner, evidence: evidence,
                        requireFreshAck: requireFreshAckForSend)) {
                        fence.inhibit(at: sourceNow()); return
                    }
                    if sourceStopped { return }
                    guard operationGeneration == owner, !stopUnconfirmed,
                           evidence?.fenced != true, recoveryAuthorized else { fence.inhibit(at: sourceNow()); return }
                    if sourceNow() >= deadline {
                        fence.inhibit(at: deadline)
                        if let evidence { evidence.burstTrace?.obligation(at: deadline, observedAt: sourceNow(), reason: "budget_expiry", pending: true,
                            evidence: evidence, latch: stopUnconfirmed) }
                        if runtime == nil { return }
                    }
                    guard let latest = followTurnSourceGate.latest,
                          latest.rejection(at: sourceNow(), expectedGeneration: observation.generation,
                               requireEnriched: true) == nil else { fence.inhibit(at: sourceNow()); return }
                    if let health = sourceHealth?(), health.trackingQuality != .normal || health.generation != observation.generation {
                        fence.inhibit(at: sourceNow()); return
                    }
                    if let sample = followTurnSourceGate.latest, observation.observe(sample, at: sourceNow()) {
                        fence.inhibit(at: sourceNow())
                        // Crossing already fences every retry. Do not introduce a
                        // new monitor sleep after that stop obligation; the shared
                        // epoch is still observed at ingress and checked on drain.
                        return
                    }
                    var wake = sourceNow() < deadline ? deadline : .infinity
                    wake = min(wake, (latest.sourceTimestamp! + 0.500).nextUp)
                    if let remaining = runtime?.progress.remaining(at: now()) { wake = min(wake, sourceNow() + remaining) }
                    if isRecoveryArc, let ack = runtime?.lastAck {
                        wake = min(wake, (sourceNow() + max(0, guardLayer.watchdogTimeout - now().timeIntervalSince(ack))).nextUp)
                    }
                    if let authorization = FollowRecoveryScope.authorization {
                        wake = min(wake, sourceNow() + max(0, authorization.deadline - authorization.now()))
                    }
                    await waitForFollowTurnSource(until: wake)
                }
            } else {
                await sleep(.seconds(max(0, deadline - sourceNow())))
                guard !Task.isCancelled, sourceNow() >= deadline else { return }
                fence.inhibit(at: deadline) // Only this immutable operation token; never motors or a newer owner.
            }
        }
        // Deadline/source-clock wake ownership is independent of the actual
        // ACK getter: that actor suspension must not pause budget inhibition.
        let deadlineMonitor: Task<Void, Never>? = runtime.map { runtime in
            Task { @MainActor in
                guard let epoch = await arming.wait(), !Task.isCancelled else { return }
                let deadline = epoch.deadline
                while !Task.isCancelled {
                    guard operationGeneration == owner, !stopUnconfirmed, evidence?.fenced != true,
                          recoveryAuthorized else { fence.inhibit(at: sourceNow()); return }
                    runtime.expire(at: now())
                    if runtime.failure != nil { fence.inhibit(at: sourceNow()); return }
                    if isRecoveryArc, !validateFollowTurnRuntime(runtime, owner: owner, evidence: evidence,
                        requireFreshAck: true) { fence.inhibit(at: sourceNow()); return }
                    if sourceStopped { return }
                    if sourceNow() >= deadline {
                        fence.inhibit(at: deadline)
                        if let evidence { evidence.burstTrace?.obligation(at: deadline, observedAt: sourceNow(), reason: "budget_expiry", pending: true,
                            evidence: evidence, latch: stopUnconfirmed) }
                    }
                    let validationUptime = sourceNow()
                    guard let sample = followTurnSourceGate.latest,
                          sample.rejection(at: validationUptime, expectedGeneration: runtime.generation, requireEnriched: true) == nil else {
                        let rejection = followTurnSourceGate.latest?.rejection(at: validationUptime,
                            expectedGeneration: runtime.generation, requireEnriched: true)
                        runtime.failTracking(rejection)
                        if runtime.failure == .trackingLost, runtime.sourceRejection == "stale_source" {
                            evidence?.recordFailure(.trackingLost, cause: .poseSourceStale)
                        }
                        fence.inhibit(at: sourceNow()); return
                    }
                    if let health = sourceHealth?(), health.trackingQuality != .normal || health.generation != runtime.generation {
                        runtime.fail(.trackingLost); fence.inhibit(at: sourceNow()); return
                    }
                    var wake = sourceNow() < deadline ? deadline : .infinity
                    wake = min(wake, (sample.sourceTimestamp! + 0.500).nextUp)
                    if let remaining = runtime.progress.remaining(at: now()) { wake = min(wake, sourceNow() + remaining) }
                    if isRecoveryArc, let ack = runtime.lastAck {
                        wake = min(wake, (sourceNow() + max(0, guardLayer.watchdogTimeout - now().timeIntervalSince(ack))).nextUp)
                    }
                    if let authorization = FollowRecoveryScope.authorization {
                        wake = min(wake, sourceNow() + max(0, authorization.deadline - authorization.now()))
                    }
                    await waitForFollowTurnSource(until: wake)
                }
            }
        }
        defer {
            monitor.cancel()
            deadlineMonitor?.cancel()
            arming.finish()
        }
        var response = entry
        let result = await withTaskCancellationHandler {
            await FollowTurnBurstTransportScope.$authorization.withValue(selected) {
                if let evidence { evidence.scanTrace?.beginSend(command, evidence: evidence, latch: stopUnconfirmed) }
                // Logging/setup may invoke synchronous callbacks. Revalidate the
                // current owner, cached ACK, source and health after all of it.
                guard !Task.isCancelled, operationGeneration == owner, evidence?.fenced != true,
                      !stopUnconfirmed, recoveryAuthorized, pendingFollowTurnBurst == nil,
                      evidence?.targetYaw == nil || followTurnSourceGate.latest?.pose?.yaw == preparedYaw,
                      runtime.map({ validateFollowTurnRuntime($0, owner: owner, evidence: evidence,
                          requireFreshAck: requireFreshAckForSend) }) ?? true else {
                    return RoverCommandDiagnosticResult(receipt: .init(httpStatus: nil, acknowledged: false,
                        acknowledgementUTC: nil, attempts: 0, outcome: Task.isCancelled ? "cancelled" : "fenced"),
                        failure: Task.isCancelled ? FollowTurnBurstTransportDenial.cancelled : .fenced)
                }
                if let target = evidence?.targetYaw, let start = followTurnSourceGate.latest {
                    observation = .init(targetYaw: target,
                        tolerance: runtime?.tolerance ?? FollowTurnBurstPlanner.Profile(purpose: purpose == .followAlignment ? .alignment : .scan).tolerance,
                        start: start, uptime: sourceNow(), preferPositiveHalfTurn: command.right > command.left)
                }
                runtime?.progress.hasSentCommand = true
                evidence?.recordStop(.pending)
                guard refreshTransportAuthority() else {
                    fence.inhibit(at: sourceNow())
                    return RoverCommandDiagnosticResult(receipt: .init(httpStatus: nil, acknowledged: false,
                        acknowledgementUTC: nil, attempts: 0, outcome: "fenced"),
                        failure: FollowTurnBurstTransportDenial.fenced)
                }
                entry = sourceNow()
                deadline = entry + requestedBudget
                // The final clock/provider read can synchronously replace an owner
                // in injected environments. No stale operation may arm afterward.
                guard !Task.isCancelled, operationGeneration == owner, evidence?.fenced != true,
                      !stopUnconfirmed, recoveryAuthorized,
                      evidence?.targetYaw == nil || followTurnSourceGate.latest?.pose?.yaw == preparedYaw else {
                    return RoverCommandDiagnosticResult(receipt: .init(httpStatus: nil, acknowledged: false,
                        acknowledgementUTC: nil, attempts: 0, outcome: Task.isCancelled ? "cancelled" : "fenced"),
                        failure: Task.isCancelled ? FollowTurnBurstTransportDenial.cancelled : .fenced)
                }
                let safeCommand: WheelCommand
                let depthReadStarted = sourceNow()
                if let readyRecovery, readyRecovery.stage != .probe {
                    guard self.readyRecoveryPulse?.id == readyRecovery.id,
                          readyRecovery.permits(command, sample: readFollowPose(), now: sourceNow()) else {
                        return .init(receipt: .unknown, failure: FollowTurnBurstTransportDenial.fenced)
                    }
                    // Fresh forward sensing is required even for the narrow
                    // retained-path return / pre-ready in-place search policy.
                    // It is never represented as measured rear coverage.
                    let live = boundedReadyDepthObservation(.init(left: 0.25, right: 0.25))
                    recordDepthObservation(live, disposition: "ready_recovery_return_or_scan")
                    if case .unavailable = live.state {
                        return .init(receipt: .unknown, failure: DepthSafetyStop(failure: recordDepthRejection(live)))
                    }
                    depthValidUntil = depthReadStarted + DepthSafetyEvaluator.maximumRawDepthAge - live.sampleAge
                    guard live.sampleAge.isFinite, live.sampleAge >= 0, sourceNow() < depthValidUntil else {
                        return .init(receipt: .unknown, failure: DepthSafetyStop(failure: .depthUnavailable(.staleRawDepth)))
                    }
                    deadline = min(deadline, depthValidUntil, readyRecovery.expiresAt)
                    safeCommand = command
                } else if permitsAcquisitionDepthBypass(command, purpose: purpose, evidence: evidence) {
                    safeCommand = command
                    recordAcquisitionDepthBypass(evidence, disposition: "sender")
                    if let grant { deadline = min(deadline, grant.expiresAt) }
                } else {
                    let boundedReady = purpose == .followReady &&
                        (readyRecovery != nil || boundedReadyGeneration == owner)
                    let decision = ReadyMotionDepthScope.$active.withValue(boundedReady) {
                        exactDepthDecision(command, disposition: "sender")
                    }
                    switch decision {
                    case .allow(let limited, let observation):
                        safeCommand = limited
                        // Freeze the original evidence expiry. New pose callbacks
                        // cannot renew it, even while the transport actor is queued.
                        depthValidUntil = depthReadStarted + DepthSafetyEvaluator.maximumRawDepthAge - observation.sampleAge
                        guard observation.sampleAge.isFinite, observation.sampleAge >= 0,
                              depthValidUntil.isFinite, sourceNow() < depthValidUntil else {
                            let stale = DepthSafetyObservation.unavailable(.staleRawDepth,
                                sampleAge: observation.sampleAge + max(0, sourceNow() - depthReadStarted),
                                motionClass: .classify(limited))
                            recordDepthObservation(stale, disposition: "expired_before_send")
                            let failure = recordDepthRejection(stale)
                            runtime?.fail(failure)
                            return .init(receipt: .init(httpStatus: nil, acknowledged: false,
                                acknowledgementUTC: nil, attempts: 0, outcome: "depth_safety_stop"), failure: DepthSafetyStop(failure: failure))
                        }
                        deadline = min(deadline, depthValidUntil)
                        if let grant { deadline = min(deadline, grant.expiresAt) }
                    case .stopDepth(let observation):
                        let recoverableProbeObstacle = readyRecovery?.stage == .probe &&
                            (observation.state == .stop || observation.state == .caution)
                        let failure: NavigationFailure = recoverableProbeObstacle ? .obstacle : recordDepthRejection(observation)
                        runtime?.fail(failure)
                        fence.inhibit(at: entry)
                        return RoverCommandDiagnosticResult(receipt: .init(httpStatus: nil, acknowledged: false,
                            acknowledgementUTC: nil, attempts: 0, outcome: "depth_safety_stop"), failure: DepthSafetyStop(failure: failure))
                    }
                }
                // Providers/logging can reenter or advance clocks. Validate all
                // current source/ACK/forward gates again before arming an arc.
                let runtimeValid = runtime.map { validateFollowTurnRuntime($0, owner: owner, evidence: evidence,
                    requireFreshAck: isRecoveryArc || requireFreshAckForSend) } ?? true
                var arcGeneralSafe = true
                if isRecoveryArc {
                    let clearance = currentForwardClearance()
                    let date = now()
                    let age = runtime?.lastAck.map { date.timeIntervalSince($0) }
                    arcGeneralSafe = !clearance.isNaN && clearance >= guardLayer.stopDistance &&
                        age.map { $0 >= 0 && $0 <= RoverConfig.commsWatchdogTimeout } == true
                    if !arcGeneralSafe, operationGeneration == owner, evidence?.fenced != true {
                        let reason: NavigationFailure = !clearance.isNaN && clearance >= guardLayer.stopDistance ? .commsLost : .obstacle
                        runtime?.fail(reason)
                        evidence?.recordFailure(reason)
                    }
                }
                // The depth provider can invoke callbacks; do not arm a replaced owner.
                let currentAuthority = refreshTransportAuthority()
                guard currentAuthority, runtimeValid, arcGeneralSafe, recoveryAuthorized, !Task.isCancelled, operationGeneration == owner, evidence?.fenced != true,
                      !stopUnconfirmed, pendingFollowTurnBurst == nil,
                      evidence?.targetYaw == nil || followTurnSourceGate.latest?.pose?.yaw == preparedYaw else {
                    return RoverCommandDiagnosticResult(receipt: .init(httpStatus: nil, acknowledged: false,
                        acknowledgementUTC: nil, attempts: 0, outcome: "fenced"), failure: FollowTurnBurstTransportDenial.fenced)
                }
                guard let epoch = fence.arm(entry: entry, budget: deadline - entry) else {
                    return RoverCommandDiagnosticResult(receipt: .init(httpStatus: nil, acknowledged: false,
                        acknowledgementUTC: nil, attempts: 0, outcome: "invalid_budget"),
                        failure: FollowTurnBurstTransportDenial.invalidBudget)
                }
                pendingFollowTurnBurst = fence
                pendingFollowTurnBurstDrain = drain
                arming.arm(epoch)
                evidence?.burstTrace?.senderEntry(entry, deadline: deadline, budget: deadline - entry)
                var result = await sendFollowBurstCommand(safeCommand)
                // Capture/cancel synchronously at the actual sender return, not
                // after the task-local wrapper's next actor resumption.
                response = sourceNow()
                // A queued transport can return before the monitor observes the
                // immutable grant deadline. Reconcile that earlier limiting
                // authority before generic burst/depth expiry attribution.
                if let grant, let request = evidence?.context.request,
                   response >= grant.expiresAt, grant.expiresAt < entry + requestedBudget,
                   grant.expiresAt <= depthValidUntil,
                   result.receipt.attempts == 0, transportCapture.attempts.isEmpty,
                   let denial = result.failure as? FollowTurnBurstTransportDenial,
                   denial == .expired || denial == .fenced,
                   !Task.isCancelled, operationGeneration == owner, evidence?.fenced != true,
                   !stopUnconfirmed, recoveryAuthorized, runtime?.failure == nil,
                   !grant.validate(sample: currentPoseSample?(), session: request.sessionGeneration, now: response) {
                    recordSearchClearanceRejection(grant, evidence: evidence)
                    let failure = NavigationFailure.depthUnavailable(.blindSweptVolume)
                    runtime?.fail(failure)
                    result = .init(receipt: .init(httpStatus: nil, acknowledged: false,
                        acknowledgementUTC: nil, attempts: 0, outcome: "search_clearance_stop"),
                        failure: DepthSafetyStop(failure: failure))
                }
                if response >= depthValidUntil, depthValidUntil < entry + requestedBudget,
                   result.receipt.attempts == 0, transportCapture.attempts.isEmpty,
                   let denial = result.failure as? FollowTurnBurstTransportDenial,
                   denial == .expired || denial == .fenced,
                   !Task.isCancelled, operationGeneration == owner, evidence?.fenced != true, runtime?.failure == nil {
                    let stale = DepthSafetyObservation.unavailable(.staleRawDepth,
                        sampleAge: DepthSafetyEvaluator.maximumRawDepthAge + response - depthValidUntil,
                        motionClass: .classify(safeCommand))
                    recordDepthObservation(stale, disposition: "expired_at_transport")
                    let failure = recordDepthRejection(stale)
                    runtime?.fail(failure)
                    result = .init(receipt: .init(httpStatus: nil, acknowledged: false,
                        acknowledgementUTC: nil, attempts: 0, outcome: "depth_safety_stop"), failure: DepthSafetyStop(failure: failure))
                }
                if let evidence, fence.status?.stopObligation == true || response >= deadline {
                    evidence.burstTrace?.obligation(at: fence.stopObligationUptime ?? min(response, deadline),
                        observedAt: response,
                        reason: sourceStopped ? (observation?.triggerReason ?? "source") :
                            (response >= deadline ? "budget_expiry" : "fence_or_health"),
                        pending: false, evidence: evidence, latch: stopUnconfirmed)
                }
                evidence?.burstTrace?.senderResponse(entry: entry, deadline: deadline, response: response,
                    attempts: transportCapture.attempts, timings: transportCapture.timings,
                    obligated: fence.status?.stopObligation == true, result: result, targetStop: fence.targetStop)
                monitor.cancel()
                deadlineMonitor?.cancel()
                drain.finish()
                if let evidence {
                    evidence.recordCommandReceipt(result.receipt)
                    evidence.scanTrace?.endSend(evidence: evidence, latch: stopUnconfirmed,
                        outcome: result.failure == nil ? "acknowledged" : "failed")
                }
                return result
            }
        } onCancel: { fence.inhibit() }
        if arming.epoch == nil {
            // A preparation denial is observable but has no actual sender epoch.
            evidence?.recordCommandReceipt(result.receipt)
            if let evidence {
                evidence.scanTrace?.endSend(evidence: evidence, latch: stopUnconfirmed, outcome: result.receipt.outcome)
            }
        }
        if response >= deadline { fence.inhibit(at: deadline) }
        if pendingFollowTurnBurst === fence {
            pendingFollowTurnBurst = nil
            pendingFollowTurnBurstDrain = nil
        }
        return .init(transportAttempts: transportCapture.attempts, sendEntryUptime: entry, deadline: deadline,
            responseUptime: response, result: result, stopObligation: fence.status?.stopObligation == true,
            stopObligationUptime: fence.stopObligationUptime, targetStop: fence.targetStop)
    }

    /// Task 2 boundary seam. It remains disconnected from the motor burst executor until Task 4.
    func prepareFollowTurnSource(progress: FollowTurnWaitingProgress = .init()) async -> FollowTurnSourceResult {
        operationGeneration &+= 1
        let owner = operationGeneration
        startFollowTurnSourceEvents()
        let subscriptionID = followTurnSourceSubscriptionID
        defer {
            if followTurnSourceSubscriptionID == subscriptionID {
                followTurnSourceTask?.cancel()
                followTurnSourceTask = nil
                followTurnSourceSubscriptionID = nil
            }
        }
        do { try await FollowTurnSourceScope.$required.withValue(true) { try await confirmStop() } }
        catch { return .failed(.commandFailed) }
        guard let fence = followTurnStopFence, operationGeneration == owner else { return .cancelled }
        return await awaitFollowTurnSource(after: fence, progress: progress)
    }

    private func startFollowTurnSourceEvents() {
        followTurnSourceTask?.cancel()
        followTurnSourceTask = nil
        followTurnSourceSubscriptionID = nil
        guard let sourceEvents else { return }
        followTurnSourceSubscriptionID = UUID()
        if let sample = currentPoseSample?() { followTurnSourceGate.ingest(sample) }
        let events = sourceEvents()
        followTurnSourceTask = Task { @MainActor [weak self] in
            for await sample in events {
                guard !Task.isCancelled, let self else { break }
                self.ingestFollowTurnSource(sample)
            }
        }
    }

    /// Synchronous source ingress; immutable events are processed before the
    /// cached latest sample can be replaced by another capture.
    func ingestFollowTurnSource(_ sample: NavigationPoseSample) {
        followTurnSourceGate.ingest(sample)
        for observer in followTurnSourceObservers.values { observer(sample) }
        wakeFollowTurnSourceWaiters()
    }

    private func wakeFollowTurnSourceWaiters() {
        let waiters = followTurnSourceWaiters.values
        followTurnSourceWaiters.removeAll()
        for waiter in waiters { waiter.resume() }
    }

    private func waitForFollowTurnSource(until deadline: TimeInterval, while needsWait: @escaping () -> Bool = { true }) async {
        let id = UUID()
        let token = FollowTurnSourceWaitToken()
        await withTaskCancellationHandler {
            guard !Task.isCancelled else { return }
            let timer = Task { @MainActor in
                guard !Task.isCancelled, token.isActive else { return }
                guard needsWait() else {
                    followTurnSourceWaiters.removeValue(forKey: id)?.resume()
                    return
                }
                let remaining = deadline - sourceNow()
                if remaining > 0 { await sleep(.seconds(remaining)) }
                followTurnSourceWaiters.removeValue(forKey: id)?.resume()
            }
            await withCheckedContinuation { followTurnSourceWaiters[id] = $0 }
            token.cancel()
            timer.cancel()
        } onCancel: {
            token.cancel()
            Task { @MainActor [weak self] in self?.followTurnSourceWaiters.removeValue(forKey: id)?.resume() }
        }
    }

    func awaitFollowTurnSource(after fence: FollowTurnStopFence,
                               progress: FollowTurnWaitingProgress,
                               runtime: FollowTurnRuntimeState? = nil) async -> FollowTurnSourceResult {
        let evidence = FollowMotionTaskScope.evidence
        let minimumSettle = runtime?.settleWait ?? 0.300
        var enteredSettle = false
        var evaluated: NavigationPoseSample?
        defer {
            if let evidence, evaluated == nil {
                evidence.burstTrace?.sourceGate(fence: fence, sample: followTurnSourceGate.latest, uptime: sourceNow(),
                    reason: followTurnSourceGate.diagnosticRejection(after: fence, at: sourceNow(), minimumSettle: minimumSettle) ?? "authority_or_runtime_failure",
                    evidence: evidence, latch: stopUnconfirmed, minimumSettle: minimumSettle)
            }
            if enteredSettle, let evidence {
                if let evaluated, let runtime {
                    evidence.scanTrace?.settled()
                    evidence.scanTrace?.evaluation(.init(sample: evaluated, uptime: sourceNow(),
                        expectedGeneration: fence.sourceGeneration),
                        snapshot: runtime.progress.watchdog.diagnosticSnapshot(
                            distanceToGoal: runtime.progress.distanceToGoal, now: now()),
                        evidence: evidence, latch: stopUnconfirmed)
                } else {
                    evidence.scanTrace?.endWait("settle", evidence: evidence, latch: stopUnconfirmed,
                        interrupted: Task.isCancelled || evidence.fenced || operationGeneration != fence.operationGeneration)
                }
            }
        }
        if let expected = FollowRecoveryScope.authorization?.expectedGeneration, expected != fence.sourceGeneration {
            return .failed(.trackingLost)
        }
        while true {
            guard !Task.isCancelled, operationGeneration == fence.operationGeneration,
                  followTurnStopFence?.identity == fence.identity,
                  FollowMotionTaskScope.evidence?.fenced != true, recoveryAuthorized else { return .cancelled }
            guard !stopUnconfirmed else { return .failed(.commandFailed) }
            let uptime = sourceNow()
            runtime?.expire(at: now())
            if let failure = runtime?.failure { return .failed(failure) }
            if runtime == nil, progress.expired(at: now()) { return .failed(.stalled) }
            if let health = sourceHealth?(), health.trackingQuality != .normal || health.generation != fence.sourceGeneration {
                return .failed(.trackingLost)
            }
            guard let sample = followTurnSourceGate.latest,
                   sample.rejection(at: uptime, expectedGeneration: fence.sourceGeneration, requireEnriched: true) == nil else {
                let rejection = followTurnSourceGate.latest?.rejection(at: uptime,
                    expectedGeneration: fence.sourceGeneration, requireEnriched: true)
                runtime?.failTracking(rejection)
                if rejection == "stale_source" { evidence?.recordFailure(.trackingLost, cause: .poseSourceStale) }
                if let evidence {
                    evidence.scanTrace?.unavailablePost(sample: followTurnSourceGate.latest.map {
                        .init(sample: $0, uptime: uptime, expectedGeneration: fence.sourceGeneration)
                    }, evidence: evidence, latch: stopUnconfirmed)
                }
                return .failed(.trackingLost)
            }
            let read = await interruptibleFollowTurnAck(owner: fence.operationGeneration, progress: { runtime?.progress ?? progress }) {
                runtime?.expire(at: self.now())
                if let health = self.sourceHealth?(), health.trackingQuality != .normal || health.generation != fence.sourceGeneration {
                    return true
                }
                return self.followTurnStopFence?.identity != fence.identity || evidence?.fenced == true ||
                    !self.recoveryAuthorized || self.stopUnconfirmed || runtime?.failure != nil ||
                    (runtime == nil && progress.expired(at: self.now())) ||
                    self.followTurnSourceGate.latest?.rejection(at: self.sourceNow(),
                        expectedGeneration: fence.sourceGeneration, requireEnriched: true) != nil
            }
            if !read.completed { continue }
            let ack = read.value
            guard !Task.isCancelled, operationGeneration == fence.operationGeneration,
                  followTurnStopFence?.identity == fence.identity,
                  FollowMotionTaskScope.evidence?.fenced != true, recoveryAuthorized else { return .cancelled }
            runtime?.expire(at: now())
            if let failure = runtime?.failure { return .failed(failure) }
            if runtime == nil, progress.expired(at: now()) { return .failed(.stalled) }
            guard !stopUnconfirmed else { return .failed(.commandFailed) }
            if let health = sourceHealth?(), health.trackingQuality != .normal || health.generation != fence.sourceGeneration {
                return .failed(.trackingLost)
            }
            let validationUptime = sourceNow()
            guard let newest = followTurnSourceGate.latest,
                  newest.rejection(at: validationUptime, expectedGeneration: fence.sourceGeneration, requireEnriched: true) == nil else {
                let rejection = followTurnSourceGate.latest?.rejection(at: validationUptime,
                    expectedGeneration: fence.sourceGeneration, requireEnriched: true)
                runtime?.failTracking(rejection)
                if rejection == "stale_source" { evidence?.recordFailure(.trackingLost, cause: .poseSourceStale) }
                if let evidence {
                    evidence.scanTrace?.unavailablePost(sample: followTurnSourceGate.latest.map {
                        .init(sample: $0, uptime: sourceNow(), expectedGeneration: fence.sourceGeneration)
                    }, evidence: evidence, latch: stopUnconfirmed)
                }
                return .failed(.trackingLost)
            }
            let decision = guardLayer.evaluate(forwardClearance: currentForwardClearance(), lastAckAt: ack,
                now: now(), feedback: nil, requireFreshAck: progress.hasSentCommand, checkForwardObstacle: false)
            switch decision {
            case .go: break
            case .stopCommsLost: return .failed(.commsLost)
            case .stopTipping: return .failed(.tipping)
            case .stopObstacle: return .failed(.obstacle)
            }
            if let selected = followTurnSourceGate.consume(after: fence, at: sourceNow(), minimumSettle: minimumSettle) {
                evaluated = selected
                if let evidence { evidence.burstTrace?.sourceGate(fence: fence, sample: selected, uptime: sourceNow(),
                    reason: nil, evidence: evidence, latch: stopUnconfirmed, minimumSettle: minimumSettle) }
                return .sample(selected)
            }
            var deadline = (newest.sourceTimestamp! + 0.500).nextUp
            let settleDeadline = fence.acknowledgementUptime +
                (sourceNow() < fence.acknowledgementUptime + minimumSettle ? minimumSettle : 0.300)
            if sourceNow() < settleDeadline { deadline = min(deadline, settleDeadline) }
            if let remaining = (runtime?.progress ?? progress).remaining(at: now()) {
                deadline = min(deadline, sourceNow() + remaining)
            }
            if let authorization = FollowRecoveryScope.authorization {
                deadline = min(deadline, sourceNow() + max(0, authorization.deadline - authorization.now()))
            }
            if !enteredSettle, sourceNow() < settleDeadline, let evidence {
                evidence.scanTrace?.beginWait("settle", duration: settleDeadline - sourceNow(),
                    evidence: evidence, latch: stopUnconfirmed)
                enteredSettle = true
            }
            if let evidence { evidence.burstTrace?.sourceGate(fence: fence, sample: newest, uptime: sourceNow(),
                reason: followTurnSourceGate.diagnosticRejection(after: fence, at: sourceNow(), minimumSettle: minimumSettle),
                evidence: evidence, latch: stopUnconfirmed, minimumSettle: minimumSettle) }
            await waitForFollowTurnSource(until: deadline)
        }
    }

    func readFollowPose() -> NavigationPoseSample {
        if let currentPoseSample { return currentPoseSample() ?? .unavailable }
        return .legacy(currentPose())
    }

    private var recoveryAuthorized: Bool {
        guard let authorization = FollowRecoveryScope.authorization else { return true }
        let time = authorization.now()
        let allowed = authorization.authorized(at: time) && FollowRecoveryScope.heading?.stageHeading.isFinite != false
        FollowMotionTaskScope.evidence?.recoveryAuthorizationTime = time
        FollowMotionTaskScope.evidence?.recoveryAuthorizationOutcome = allowed ? "authorized" :
            (!time.isFinite || !authorization.deadline.isFinite ? "nonfinite_deadline_clock" :
                (time >= authorization.deadline ? "deadline_expired" :
                    (Task.isCancelled ? "cancelled" : "ownership_health_or_heading_fenced")))
        return allowed
    }

    private func followPoseRejection(_ sample: NavigationPoseSample, at time: TimeInterval,
                                     expectedGeneration: UInt64? = nil) -> String? {
        if let recovery = FollowRecoveryScope.authorization {
            guard sample.source != "legacy_unknown", sample.frameID != nil,
                  sample.sourceTimestamp != nil, sample.trackingQuality != nil else { return "missing_recovery_provenance" }
            return sample.rejection(at: time, expectedGeneration: recovery.expectedGeneration)
        }
        return sample.rejection(at: time, expectedGeneration: expectedGeneration)
    }

    private func drive(to initialGoal: Vec2, stoppingAtForwardClearance targetStopDistance: Double?,
                       isFollowGoal: Bool) async -> NavigationResult {
        var goal = initialGoal
        var hasSentCommand = false
        var consecutiveCommandFailures = 0
        var progressWatchdog = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.05)
        while !Task.isCancelled {
            guard let pose = currentPose() else {
                liveFollowGoal = nil
                try? await stopRover()
                path = []
                let result = NavigationResult.failed(.trackingLost)
                finish(result)
                return result
            }
            if isFollowGoal, let live = liveFollowGoal, live != goal {
                // The goal moved with the person. Distance-to-goal no longer measures this
                // drive's progress across the change, so restart the stall window.
                goal = live
                progressWatchdog.reset()
            }
            let distanceToGoal = pose.position.distance(to: goal)
            let targetApproachDecision = targetStopDistance.map {
                Self.visualTargetApproachDecision(distanceToGoal: distanceToGoal,
                                                  forwardClearance: currentForwardClearance(),
                                                  stopDistance: $0)
            } ?? .inactive

            if case .arrived = targetApproachDecision, let targetStopDistance {
                liveFollowGoal = nil
                try? await stopRover()
                let result = NavigationResult.arrived
                finish(result)
                RuntimeFileLog.append("nav_target_reached", fields: [
                    "brake_trigger_distance": Self.formatMeters(
                        targetStopDistance + RoverConfig.visualTargetBrakeLeadDistance
                    ),
                    "clearance": Self.formatMeters(currentForwardClearance()),
                    "distance_to_goal": Self.formatMeters(distanceToGoal),
                    "stop_distance": Self.formatMeters(targetStopDistance)
                ])
                return result
            }

            // Replan every ~1s to fold in newly meshed obstacles.
            replanCounter += 1
            if replanCounter % 10 == 0,
               let failure = planAndStore(from: pose.position, to: goal, policy: activePolicy) {
                liveFollowGoal = nil
                try? await stopRover()
                path = []
                let result = NavigationResult.failed(failure)
                finish(result)
                return result
            }

            // Safety gate.
            let lastAck = await currentLastAck()
            if FollowMotionTaskScope.evidence != nil,
               Task.isCancelled || FollowMotionTaskScope.evidence?.fenced == true { break }
            let now = Date()
            let decision = guardLayer.evaluate(forwardClearance: currentForwardClearance(),
                                               lastAckAt: lastAck,
                                               now: now,
                                               feedback: nil,
                                               requireFreshAck: hasSentCommand,
                                               checkForwardObstacle: targetApproachDecision == .inactive)
            switch decision {
            case .go:
                break
            case .stopObstacle(let clearance):
                liveFollowGoal = nil
                try? await stopRover()
                if isFollowGoal {
                    let result = NavigationResult.failed(.obstacle)
                    finish(result)
                    RuntimeFileLog.append("nav_safety_stop", fields: [
                        "reason": "obstacle",
                        "clearance": String(format: "%.2f", clearance),
                        "state": "follow_obstacle"
                    ])
                    return result
                }
                state = Self.stateAfterObstacleStop(pose: pose, goal: goal, clearance: clearance)
                if case .failed = state { publishSafetyState(.failed(.obstacle)) }
                RuntimeFileLog.append("nav_safety_stop", fields: [
                    "reason": "obstacle",
                    "clearance": String(format: "%.2f", clearance),
                    "state": state.description
                ])
                activePolicy = nil
                return state == .arrived ? .arrived : .failed(.obstacle)
            case .stopCommsLost:
                liveFollowGoal = nil
                try? await stopRover()
                let result = NavigationResult.failed(.commsLost)
                finish(result)
                RuntimeFileLog.append("nav_safety_stop", fields: [
                    "reason": "comms_lost",
                    "ack_age": Self.ackAgeField(lastAckAt: lastAck, now: now)
                ])
                return result
            case .stopTipping:
                liveFollowGoal = nil
                try? await stopRover()
                let result = NavigationResult.failed(.tipping)
                finish(result)
                RuntimeFileLog.append("nav_safety_stop", fields: ["reason": "tipping"])
                return result
            }

            let out = pursuit.step(pose: pose, path: path)
            if out.reachedGoal {
                liveFollowGoal = nil
                try? await stopRover()
                let result = NavigationResult.arrived
                finish(result)
                return result
            }
            let command = targetApproachDecision == .approach
                ? Self.visualTargetApproachCommand(out.command,
                                                   forwardClearance: currentForwardClearance(),
                                                   stopDistance: targetStopDistance ?? 0)
                : out.command
            let isCommanded = abs(command.left) > 0.01 || abs(command.right) > 0.01
            if progressWatchdog.observe(distanceToGoal: distanceToGoal,
                                        now: now,
                                        commanded: isCommanded) {
                liveFollowGoal = nil
                try? await stopRover()
                let result = NavigationResult.failed(.stalled)
                finish(result)
                RuntimeFileLog.append("nav_safety_stop", fields: [
                    "reason": "no_goal_progress",
                    "distance_to_goal": Self.formatMeters(distanceToGoal),
                    "timeout": "2.50"
                ])
                return result
            }
            var telemetry = Self.driveTelemetryFields(pose: pose,
                                                       goal: goal,
                                                       command: command,
                                                       consecutiveCommandFailures: consecutiveCommandFailures)
            telemetry["forward_clearance"] = Self.formatMeters(currentForwardClearance())
            telemetry["path_points"] = "\(path.count)"
            telemetry["target_approach_slowed"] = command == out.command ? "false" : "true"
            RuntimeFileLog.append("nav_drive_tick", fields: telemetry)
            let safeCommand: WheelCommand
            switch exactDepthDecision(command, disposition: "pursuit") {
            case .allow(let limited, _): safeCommand = limited
            case .stopDepth(let observation):
                let failure = recordDepthRejection(observation)
                liveFollowGoal = nil
                let cleanupOwner = operationGeneration
                do { try await rotationStop(origin: "cleanup") } catch {
                    guard operationGeneration == cleanupOwner, !Task.isCancelled,
                          FollowMotionTaskScope.evidence?.fenced != true else { return .cancelled }
                    finish(.failed(.commandFailed))
                    return .failed(.commandFailed)
                }
                finish(.failed(failure))
                return .failed(failure)
            }
            guard !Task.isCancelled, FollowMotionTaskScope.evidence?.fenced != true, !stopUnconfirmed else { break }
            do {
                try await sendCommand(safeCommand)
                consecutiveCommandFailures = 0
                hasSentCommand = true
            } catch {
                // stopAndConfirm cancels this task; URLSession may surface that as
                // URLError.cancelled rather than CancellationError. Let the stop
                // confirmation own the outcome instead of publishing a send failure.
                if Task.isCancelled { break }
                consecutiveCommandFailures += 1
                RuntimeFileLog.append("nav_command_send_failed", fields: Self.driveTelemetryFields(
                    pose: pose,
                    goal: goal,
                    command: command,
                    consecutiveCommandFailures: consecutiveCommandFailures
                ).merging([
                    "error": error.localizedDescription,
                    "max_failures": "1"
                ]) { current, _ in current })
                liveFollowGoal = nil
                try? await stopRover()
                state = Self.stateAfterCommandFailure(error)
                publishSafetyState(.failed(.commandFailed))
                RuntimeFileLog.append("nav_command_failed", fields: [
                    "error": error.localizedDescription,
                    "state": state.description
                ])
                activePolicy = nil
                return .failed(.commandFailed)
            }
            await sleep(.seconds(RoverConfig.commandInterval))
        }
        liveFollowGoal = nil
        try? await stopRover()
        activePolicy = nil
        return .cancelled
    }

    private func planAndStore(
        from: Vec2,
        to: Vec2,
        policy: (any PathAdmissibilityPolicy)?
    ) -> NavigationFailure? {
        guard let p = makePlan(from, to) else { return .noPath }
        if let policy,
           case .rejected(let violation) = policy.evaluate(path: [from] + p + [to]) {
            return .pathRejected(violation)
        }
        path = p
        return nil
    }

    private func performRotate(to targetYaw: Double, mode: RotationMode,
                               followProfile: FollowScanRotationProfile? = nil,
                               sourceGeneration: UInt64? = nil) async -> NavigationResult {
        var targetYaw = targetYaw
        var recoveryResolved = false
        FollowMotionTaskScope.evidence?.targetYaw = FollowRecoveryScope.heading == nil ? targetYaw : nil
        let pulsed = mode == .scan || mode == .followScan
        let profile = mode == .followScan ? followProfile : nil
        let angularTolerance = profile?.angularTolerance ?? (pulsed ? RoverConfig.scanTurnYawTolerance : 0.05)
        let ownedGeneration = operationGeneration
        var hasSentCommand = false
        var progressWatchdog = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.05)
        while !Task.isCancelled {
            guard recoveryAuthorized else { break }
            let evidence = FollowMotionTaskScope.evidence
            let trace = evidence?.scanTrace
            var lastAck: Date?
            let requiresSource = mode == .followScan || FollowRecoveryScope.authorization != nil
            if requiresSource {
                trace?.enter("ack_read")
                lastAck = await currentLastAck()
                guard !Task.isCancelled, evidence?.fenced != true, !stopUnconfirmed,
                      operationGeneration == ownedGeneration, recoveryAuthorized else { break }
            }
            let sourceTime = requiresSource ? sourceNow() : 0
            let controlSample = requiresSource ? readFollowPose() : .legacy(currentPose())
            guard !requiresSource || followPoseRejection(controlSample, at: sourceTime, expectedGeneration: sourceGeneration) == nil,
                  let pose = controlSample.pose else {
                if let evidence = FollowMotionTaskScope.evidence {
                    let rejected = currentPoseSample == nil ? nil : RotationPoseDiagnosticSample(
                        sample: controlSample, uptime: sourceTime, expectedGeneration: sourceGeneration)
                    evidence.scanTrace?.unavailablePost(sample: rejected, evidence: evidence, latch: stopUnconfirmed)
                }
                try? await rotationStop(origin: "cleanup")
                let result = NavigationResult.failed(.trackingLost)
                finish(result)
                return result
            }
            if mode == .followScan, let recovery = FollowRecoveryScope.heading, !recoveryResolved {
                let delta: Double
                switch FollowReacquisitionPlanner.resolveAbsoluteStage(stageHeading: recovery.stageHeading, actualYaw: pose.yaw,
                    maximumSegment: recovery.maximumSegment) {
                case .turn(let resolvedDelta, let target):
                    delta = resolvedDelta
                    targetYaw = target
                case .stageArrived:
                    delta = 0
                    targetYaw = FollowReacquisitionPlanner.wrap(pose.yaw)
                case .unavailable, .exhausted:
                    return .cancelled
                }
                recoveryResolved = true
                evidence?.targetYaw = targetYaw
                if let previous = evidence?.recovery {
                    evidence?.recovery = .init(postStopSource: previous.postStopSource, resolutionSource: controlSample,
                        stageHeading: recovery.stageHeading, segmentHeading: targetYaw, requestedDelta: delta,
                        arrivalSource: nil, segmentArrived: false, stageArrived: nil,
                        postStopReadUptime: previous.postStopReadUptime, resolutionReadUptime: sourceTime)
                }
            }
            let error = normalizeAngle(targetYaw - pose.yaw)
            let sample = currentPoseSample != nil && mode == .followScan
                ? RotationPoseDiagnosticSample(sample: controlSample, uptime: sourceTime, expectedGeneration: sourceGeneration)
                : trace?.sample(yaw: pose.yaw)
            if let evidence, let sample {
                trace?.evaluation(sample, snapshot: progressWatchdog.diagnosticSnapshot(distanceToGoal: abs(error), now: now()),
                    evidence: evidence, latch: stopUnconfirmed)
            }
            if abs(error) <= angularTolerance {
                try? await rotationStop(origin: "final")
                if requiresSource && (Task.isCancelled || evidence?.fenced == true || stopUnconfirmed || !recoveryAuthorized) { return .cancelled }
                let result = NavigationResult.arrived
                finish(result)
                return result
            }

            if !requiresSource {
                trace?.enter("ack_read")
                lastAck = await currentLastAck()
            }
            if evidence != nil && (Task.isCancelled || evidence?.fenced == true || stopUnconfirmed) { break }
            let now = now()
            let decision = guardLayer.evaluate(forwardClearance: currentForwardClearance(),
                                               lastAckAt: lastAck,
                                               now: now,
                                               feedback: nil,
                                               requireFreshAck: hasSentCommand,
                                               checkForwardObstacle: false)
            switch decision {
            case .go:
                break
            case .stopObstacle(let clearance):
                try? await rotationStop(origin: "cleanup")
                state = .failed(Self.obstacleMessage(clearance: clearance))
                publishSafetyState(.failed(.obstacle))
                RuntimeFileLog.append("nav_safety_stop", fields: [
                    "reason": "obstacle_while_rotating",
                    "clearance": String(format: "%.2f", clearance)
                ])
                return .failed(.obstacle)
            case .stopCommsLost:
                try? await rotationStop(origin: "cleanup")
                let result = NavigationResult.failed(.commsLost)
                finish(result)
                RuntimeFileLog.append("nav_safety_stop", fields: [
                    "reason": "comms_lost_while_rotating",
                    "ack_age": Self.ackAgeField(lastAckAt: lastAck, now: now)
                ])
                return result
            case .stopTipping:
                try? await rotationStop(origin: "cleanup")
                let result = NavigationResult.failed(.tipping)
                finish(result)
                RuntimeFileLog.append("nav_safety_stop", fields: ["reason": "tipping_while_rotating"])
                return result
            }

            let cmd: WheelCommand
            if let profile {
                // Follow-only fixed breakaway pulse, including just outside tolerance.
                let speed = profile.wheelCap
                let signed = error > 0 ? speed : -speed
                cmd = WheelCommand(left: -signed, right: signed)
            } else {
                cmd = RotationCommand.command(forYawError: error)
            }
            let before = progressWatchdog.diagnosticSnapshot(distanceToGoal: abs(error), now: now)
            let stalled = progressWatchdog.observe(distanceToGoal: abs(error), now: now, commanded: true)
            if let sample {
                trace?.observed(progressWatchdog.diagnosticSnapshot(distanceToGoal: abs(error), now: now), previous: before, sample: sample)
            }
            if stalled {
                trace?.enter("watchdog")
                try? await rotationStop(origin: "cleanup")
                let result = NavigationResult.failed(.stalled)
                finish(result)
                RuntimeFileLog.append("nav_safety_stop", fields: [
                    "reason": "no_yaw_progress",
                    "target_yaw_deg": Self.formatDegrees(targetYaw),
                    "yaw_error_deg": Self.formatDegrees(error),
                    "timeout": "2.50"
                ])
                return result
            }
            RuntimeFileLog.append("nav_rotate_tick", fields: [
                "pose_x": Self.formatMeters(pose.position.x),
                "pose_y": Self.formatMeters(pose.position.y),
                "pose_yaw_deg": Self.formatDegrees(pose.yaw),
                "target_yaw_deg": Self.formatDegrees(targetYaw),
                "yaw_error_deg": Self.formatDegrees(error),
                "mode": mode == .followScan ? "follow_scan_pulse" : (pulsed ? "scan_pulse" : "continuous"),
                "wheel_left": Self.formatMeters(cmd.left),
                "wheel_right": Self.formatMeters(cmd.right)
            ])
            if let evidence, let sample {
                trace?.pulse(sample, command: cmd, evidence: evidence, latch: stopUnconfirmed)
                trace?.beginSend(cmd, evidence: evidence, latch: stopUnconfirmed)
            }
            if FollowRecoveryScope.authorization != nil {
                guard recoveryAuthorized, operationGeneration == ownedGeneration,
                      !Task.isCancelled, evidence?.fenced != true, !stopUnconfirmed else { break }
                if followPoseRejection(controlSample, at: sourceNow()) != nil {
                    try? await rotationStop(origin: "cleanup")
                    let result = NavigationResult.failed(.trackingLost)
                    finish(result)
                    return result
                }
            }
            let safeCommand: WheelCommand
            switch exactDepthDecision(cmd, disposition: "legacy_turn") {
            case .allow(let limited, _): safeCommand = limited
            case .stopDepth(let observation):
                let failure = recordDepthRejection(observation)
                let cleanupOwner = operationGeneration
                do { try await rotationStop(origin: "cleanup") } catch {
                    guard operationGeneration == cleanupOwner, !Task.isCancelled,
                          FollowMotionTaskScope.evidence?.fenced != true else { return .cancelled }
                    finish(.failed(.commandFailed))
                    return .failed(.commandFailed)
                }
                finish(.failed(failure))
                return .failed(failure)
            }
            guard !Task.isCancelled, operationGeneration == ownedGeneration,
                  evidence?.fenced != true, !stopUnconfirmed else { break }
            do {
                try await sendCommand(safeCommand)
                if let evidence { trace?.endSend(evidence: evidence, latch: stopUnconfirmed, outcome: "acknowledged") }
                hasSentCommand = true
            } catch {
                if let evidence { trace?.endSend(evidence: evidence, latch: stopUnconfirmed,
                    outcome: Task.isCancelled ? "cancelled" : "failed") }
                // A cancelled motion send is not a failed motor-stop confirmation.
                if Task.isCancelled { break }
                try? await rotationStop(origin: "cleanup")
                state = Self.stateAfterCommandFailure(error)
                publishSafetyState(.failed(.commandFailed))
                RuntimeFileLog.append("nav_command_failed", fields: [
                    "error": error.localizedDescription,
                    "state": state.description
                ])
                return .failed(.commandFailed)
            }
            if mode == .followScan && (Task.isCancelled || evidence?.fenced == true || stopUnconfirmed || !recoveryAuthorized) { break }
            if pulsed {
                if let evidence { trace?.beginWait("pulse_wait", duration: profile?.pulseWait ?? RoverConfig.scanTurnPulseDuration,
                    evidence: evidence, latch: stopUnconfirmed) }
                await sleep(.seconds(profile?.pulseWait ?? RoverConfig.scanTurnPulseDuration))
                if let evidence { trace?.endWait("pulse_wait", evidence: evidence, latch: stopUnconfirmed, interrupted: Task.isCancelled) }
                if mode == .followScan && (Task.isCancelled || evidence?.fenced == true || stopUnconfirmed || !recoveryAuthorized) { break }
                do {
                    try await rotationStop(origin: "pulse")
                } catch {
                    // External stop owns an independent, noncancelled acknowledgement.
                    // Do not publish success here: confirmStop still fails closed if that fails.
                    if Task.isCancelled { return .cancelled }
                    stopUnconfirmed = true
                    FollowMotionTaskScope.evidence?.recordStop(.failed)
                    let result = NavigationResult.failed(.commandFailed)
                    finish(result)
                    return result
                }
                if mode == .followScan && (Task.isCancelled || evidence?.fenced == true || stopUnconfirmed || !recoveryAuthorized) { break }
                RuntimeFileLog.append("nav_scan_turn_settle", fields: [
                    "settle_seconds": String(format: "%.2f", RoverConfig.scanTurnSettleDuration)
                ])
                if let evidence { trace?.beginWait("settle", duration: profile?.settleWait ?? RoverConfig.scanTurnSettleDuration,
                    evidence: evidence, latch: stopUnconfirmed) }
                await sleep(.seconds(profile?.settleWait ?? RoverConfig.scanTurnSettleDuration))
                if mode == .followScan && (Task.isCancelled || evidence?.fenced == true || stopUnconfirmed || !recoveryAuthorized) {
                    if let evidence { trace?.endWait("settle", evidence: evidence, latch: stopUnconfirmed, interrupted: true) }
                    break
                }
                trace?.settled()
            } else {
                await sleep(.seconds(RoverConfig.commandInterval))
            }
        }
        try? await rotationStop(origin: "cleanup")
        return .cancelled
    }

    private func prepareFollowTurnDepth(_ command: WheelCommand, runtime: FollowTurnRuntimeState, owner: UInt,
                                       evidence: FollowMotionOperationEvidence?, waitSpent: inout Bool) async throws
        -> (command: WheelCommand, arc: Bool) {
        func owned() -> Bool {
            !Task.isCancelled && operationGeneration == owner && evidence?.fenced != true &&
                !stopUnconfirmed && recoveryAuthorized
        }
        guard owned() else { throw CancellationError() }
        if permitsAcquisitionDepthBypass(command, purpose: evidence?.context.purpose, evidence: evidence) {
            recordAcquisitionDepthBypass(evidence, disposition: "turn")
            guard owned() else { throw CancellationError() }
            return (command, false)
        }
        var decision = exactDepthDecision(command, disposition: "turn")
        guard owned() else { throw CancellationError() }
        if case .stopDepth(let initial) = decision,
           initial.state == .unavailable(.missingRawDepth) || initial.state == .unavailable(.staleRawDepth), !waitSpent {
            // Every calibrated burst starts stopped. Keep the original snapshot
            // age; retries call the real provider and never manufacture freshness.
            guard pendingFollowTurnBurst == nil, evidence?.stopOutcome == .confirmed else { throw CancellationError() }
            waitSpent = true
            let deadline = sourceNow() + 0.6
            for _ in 0..<12 {
                guard owned(), sourceNow() < deadline else { break }
                await sleep(.milliseconds(50))
                guard owned() else { throw CancellationError() }
                if sourceNow() >= deadline { break }
                guard await refreshFollowTurnRuntime(runtime, owner: owner, evidence: evidence,
                    requireFreshAck: true, deadline: deadline) else {
                    if owned(), runtime.failure == nil, sourceNow() >= deadline { break }
                    throw CancellationError()
                }
                guard owned() else { throw CancellationError() }
                decision = exactDepthDecision(command, disposition: "stopped_retry")
                guard owned() else { throw CancellationError() }
                if case .stopDepth(let observation) = decision,
                   observation.state == .unavailable(.missingRawDepth) || observation.state == .unavailable(.staleRawDepth) {
                    continue
                }
                break
            }
        }
        guard owned() else { throw CancellationError() }
        switch decision {
        case .allow(let safe, _): return (safe, false)
        case .stopDepth(let rejected):
            guard evidence?.context.request?.isSearchSweep != true,
                  evidence?.context.request?.searchClearance == nil,
                  rejected.state == .unavailable(.blindSweptVolume), DepthSafetyMotionClass.classify(command) == .rotating else {
                runtime.fail(recordDepthRejection(rejected)); throw DepthSafetyStop(failure: runtime.failure!)
            }
            guard depthRecoveryAttempts < 2 else {
                evidence?.recordFailure(.depthUnavailable(.blindSweptVolume), cause: .depthRecoveryExhausted)
                runtime.fail(.depthUnavailable(.blindSweptVolume))
                throw DepthSafetyStop(failure: .depthUnavailable(.blindSweptVolume))
            }
            // Keep the proven outer breakaway magnitude, but use a shallow arc:
            // the old 1:2 wheel ratio sweeps outside a forward depth frustum.
            let outer = RoverConfig.minimumRotateWheelSpeed
            let arc = command.right > command.left
                ? WheelCommand(left: outer * 0.8, right: outer)
                : WheelCommand(left: outer, right: outer * 0.8)
            let arcDecision = exactDepthDecision(arc, disposition: "recovery_arc")
            guard owned() else { throw CancellationError() }
            guard case .allow(let safe, _) = arcDecision else {
                if case .stopDepth(let observation) = arcDecision { runtime.fail(recordDepthRejection(observation)) }
                throw DepthSafetyStop(failure: runtime.failure ?? .depthUnavailable(.blindSweptVolume))
            }
            guard await refreshFollowTurnRuntime(runtime, owner: owner, evidence: evidence, requireFreshAck: true) else {
                throw CancellationError()
            }
            guard owned() else { throw CancellationError() }
            let clearance = currentForwardClearance()
            let date = now()
            guard owned() else { throw CancellationError() }
            guard let ack = runtime.lastAck, date.timeIntervalSince(ack) >= 0,
                  date.timeIntervalSince(ack) <= RoverConfig.commsWatchdogTimeout else {
                runtime.fail(.commsLost); throw DepthSafetyStop(failure: .commsLost)
            }
            guard !clearance.isNaN, clearance >= guardLayer.stopDistance else {
                runtime.fail(.obstacle); throw DepthSafetyStop(failure: .obstacle)
            }
            return (safe, true)
        }
    }

    /// Controller-owned initial acquisition only. This is a motion policy, never
    /// camera evidence or an implicit operator clearance grant. Existing source,
    /// transport, stop, sweep and ownership fences remain authoritative.
    private func permitsAcquisitionDepthBypass(_ command: WheelCommand, purpose: FollowMotionPurpose?,
                                                 evidence: FollowMotionOperationEvidence?) -> Bool {
        guard purpose == .followScan || purpose == .followAlignment,
              let evidence, activeFollowEvidence === evidence,
              evidence.ownedGeneration == operationGeneration, !evidence.fenced,
              evidence.context.purpose == purpose,
              let request = evidence.context.request, request.purpose == purpose,
              request.allowsAcquisitionRotationWithoutDepth,
              let angle = evidence.context.requestedRotation, angle.isFinite, angle != 0, abs(angle) <= .pi,
              recoveryAuthorized,
              command.left.isFinite, command.right.isFinite, command.left != 0,
              command.left == -command.right,
              abs(command.left) <= FollowTurnBurstPlanner.Profile(purpose: .scan).fixedWheelMagnitude else { return false }
        return true
    }

    private func recordAcquisitionDepthBypass(_ evidence: FollowMotionOperationEvidence?, disposition: String) {
        let fields: [String: FollowDiagnosticValue] = [
            "depth_policy": .string("pre_ready_acquisition_pure_rotation_bypass"),
            "depth_candidate_disposition": .string(disposition)]
        evidence?.depthDiagnosticFields = fields
        diagnosticEmitter?.emit(.init(event: "follow_depth.acquisition_rotation_bypassed", context: .init(
            sessionGeneration: evidence?.context.request?.sessionGeneration,
            operationID: evidence?.context.controllerOperationID, purpose: evidence?.context.purpose?.rawValue,
            phase: evidence?.context.request?.phase, reason: "ready_signal_not_completed"), payload: fields))
    }

    private func recordSearchClearanceRejection(_ grant: FollowSearchClearanceGrant,
                                               evidence: FollowMotionOperationEvidence?) {
        evidence?.searchClearanceDiagnosticFields = grant.diagnosticFields
        evidence?.recordFailure(.depthUnavailable(.blindSweptVolume), cause: grant.revocationCause)
    }

    private func validateSearchClearance(_ request: FollowMotionRequestContext?,
                                         evidence: FollowMotionOperationEvidence? = nil) -> Bool {
        guard let request, let grant = request.searchClearance else { return true }
        let evidence = evidence ?? FollowMotionTaskScope.evidence
        if request.purpose != .followScan && request.purpose != .followAlignment {
            grant.revoke(detail: "non_rotation_purpose", now: sourceNow())
        }
        if let health = sourceHealth?(), health.trackingQuality != .normal || health.generation != grant.sourceGeneration {
            grant.revoke(cause: .searchClearanceSourceInvalid, detail: "source_health_changed", now: sourceNow())
        }
        let valid = grant.validate(sample: currentPoseSample?(), session: request.sessionGeneration, now: sourceNow())
        if !valid { recordSearchClearanceRejection(grant, evidence: evidence) }
        return valid
    }

    private func searchDepthObservation(_ command: WheelCommand) -> DepthSafetyObservation {
        let evidence = FollowMotionTaskScope.evidence
        let request = evidence?.context.request
        if let grant = request?.searchClearance, command.left != -command.right {
            grant.revoke(detail: "translation_requested", now: sourceNow())
        }
        guard validateSearchClearance(request, evidence: evidence) else {
            return .unavailable(.blindSweptVolume, sampleAge: .infinity, motionClass: .classify(command))
        }
        return FollowSearchRotationScope.$authorized.withValue(request?.searchClearance != nil) {
            depthSafetyObservation(command)
        }
    }

    /// Caution changes curvature visibility and stopping distance; authorize the
    /// exact scaled wheels too. A second scaling is rejected instead of iterated.
    private func exactDepthDecision(_ command: WheelCommand, disposition: String) -> ObstacleGuard.DepthDecision {
        let observation = searchDepthObservation(command)
        recordDepthObservation(observation, disposition: disposition)
        switch guardLayer.evaluate(command: command, depthSafety: observation) {
        case .stopDepth: return .stopDepth(observation)
        case .allow(let limited, _) where limited == command: return .allow(command, depthSafety: observation)
        case .allow(let limited, _):
            let exact = searchDepthObservation(limited)
            recordDepthObservation(exact, disposition: disposition + "_scaled")
            if case .allow(let accepted, _) = guardLayer.evaluate(command: limited, depthSafety: exact), accepted == limited {
                return .allow(limited, depthSafety: exact)
            }
            return .stopDepth(exact)
        }
    }

    private func recordDepthObservation(_ observation: DepthSafetyObservation, disposition: String) {
        func finite(_ value: Double) -> FollowDiagnosticValue { value.isFinite ? .number(value) : .null }
        var fields: [String: FollowDiagnosticValue] = [
            "depth_state": .string(observation.state.telemetryReason), "depth_age_s": finite(observation.sampleAge),
            "depth_coverage_provenance": .string(observation.coverageProvenance),
            "depth_clearance_m": finite(observation.clearance), "depth_support_count": .number(Double(observation.supportCount)),
            "depth_motion_class": .string(observation.motionClass.rawValue), "depth_candidate_disposition": .string(disposition),
            "depth_required_stopping_distance_m": finite(observation.requiredStoppingDistance)]
        let coverage = observation.coverage
        let coverageFields: [String: FollowDiagnosticValue] = [
            "depth_coverage_horizon_m": coverage.map { finite($0.horizon) } ?? .null,
            "depth_coverage_fraction": coverage.map { finite($0.fraction) } ?? .null,
            "depth_required_coverage_fraction": .number(DepthSafetyEvaluator.minimumSweptVolumeCoverage),
            "depth_required_points": coverage.map { .number(Double($0.requiredPoints)) } ?? .null,
            "depth_covered_points": coverage.map { .number(Double($0.coveredPoints)) } ?? .null,
            "depth_outside_view_points": coverage.map { .number(Double($0.outsideViewPoints)) } ?? .null,
            "depth_missing_sample_points": coverage.map { .number(Double($0.missingDepthPoints)) } ?? .null,
            "depth_occluded_points": coverage.map { .number(Double($0.occludedPoints)) } ?? .null,
            "depth_projection_available": coverage.map { .bool($0.projectionAvailable) } ?? .null
        ]
        fields.merge(coverageFields) { _, new in new }
        let evidence = FollowMotionTaskScope.evidence
        evidence?.depthDiagnosticFields = fields
        diagnosticEmitter?.emit(.init(event: "follow_depth.candidate", context: .init(
            sessionGeneration: evidence?.context.request?.sessionGeneration,
            operationID: evidence?.context.controllerOperationID, purpose: evidence?.context.purpose?.rawValue,
            phase: evidence?.context.request?.phase, reason: observation.state.telemetryReason), payload: fields))
    }

    private func recordDepthRejection(_ observation: DepthSafetyObservation) -> NavigationFailure {
        let failure: NavigationFailure
        switch observation.state {
        case .stop: failure = .obstacle
        case .unavailable(let reason): failure = .depthUnavailable(reason)
        case .caution: failure = .obstacle
        case .clear: failure = .depthUnavailable(.malformedDepth)
        }
        FollowMotionTaskScope.evidence?.recordFailure(failure)
        return failure
    }

    private func boundedReadyDepthObservation(_ command: WheelCommand) -> DepthSafetyObservation {
        ReadyMotionDepthScope.$active.withValue(true) { depthSafetyObservation(command) }
    }

    private func finish(_ result: NavigationResult) {
        activePolicy = nil
        liveFollowGoal = nil
        switch result {
        case .arrived:
            state = .arrived
            publishSafetyState(.idle)
        case .cancelled, .reobservationRequired:
            state = .idle
            publishSafetyState(.idle)
        case .failed(let failure):
            state = .failed(Self.message(for: failure))
            publishSafetyState(.failed(failure))
        }
    }

    private func publishSafetyState(_ newState: NavigationSafetyState, evidence captured: FollowMotionOperationEvidence? = nil) {
        if case .failed(let reason) = newState {
            if let evidence = captured ?? FollowMotionTaskScope.evidence {
                evidence.recordFailure(reason)
                evidence.scanTrace?.failure(reason, evidence: evidence, latch: stopUnconfirmed)
                if let failure = evidence.failure(source: .stream) {
                    deliverFollowFailure(failure)
                    evidence.emittedFailure = true
                }
            } else {
                deliverFollowFailure(.init(context: .unknown, reason: reason, stopOutcome: .unknown, source: .stream))
            }
        }
        guard safetyState != newState else { return }
        safetyState = newState
        onSafetyStateChange?(newState)
        for continuation in safetyStateContinuations.values { continuation.yield(newState) }
    }

    static func message(for failure: NavigationFailure) -> String {
        switch failure {
        case .noPose: "No ARKit pose yet — move the device to establish tracking."
        case .noPath: "No path to goal."
        case .pathRejected: "Path rejected by navigation policy."
        case .obstacle: "Obstacle ahead."
        case .depthUnavailable(let reason): reason.followMotionMessage
        case .commsLost: "Rover command link lost."
        case .tipping: "Rover may be tipping."
        case .stalled: "Navigation stalled."
        case .rotationResolutionInsufficient: "Rotation stopped: observed response is too coarse for the remaining angle."
        case .commandFailed: "Rover command failed."
        case .trackingLost: "ARKit tracking lost."
        case .cancelled: "Navigation cancelled."
        }
    }

    static func stateAfterObstacleStop(pose: Pose2D, goal: Vec2, clearance: Double) -> State {
        if pose.position.distance(to: goal) <= obstacleArrivalDistance {
            return .arrived
        }
        return .failed(obstacleMessage(clearance: clearance))
    }

    static func visualTargetApproachDecision(distanceToGoal: Double,
                                             forwardClearance: Double,
                                             stopDistance: Double) -> VisualTargetApproachDecision {
        guard distanceToGoal <= RoverConfig.visualTargetApproachDistance else { return .inactive }
        let brakeTriggerDistance = stopDistance + RoverConfig.visualTargetBrakeLeadDistance
        guard forwardClearance.isFinite,
              forwardClearance <= brakeTriggerDistance else {
            return .approach
        }
        return .arrived
    }

    static func visualTargetApproachCommand(_ command: WheelCommand,
                                            forwardClearance: Double,
                                            stopDistance: Double) -> WheelCommand {
        guard forwardClearance.isFinite,
              forwardClearance > stopDistance + RoverConfig.visualTargetBrakeLeadDistance,
              forwardClearance <= RoverConfig.visualTargetSlowdownDistance,
              command.left >= 0,
              command.right >= 0 else {
            return command
        }

        let peak = max(command.left, command.right)
        guard peak > RoverConfig.visualTargetApproachMaxWheelSpeed else { return command }
        let scale = RoverConfig.visualTargetApproachMaxWheelSpeed / peak
        return WheelCommand(left: command.left * scale, right: command.right * scale)
    }

    static func stateAfterCommandFailure(_ error: Error) -> State {
        .failed("Rover command failed: \(error.localizedDescription)")
    }

    static func driveTelemetryFields(pose: Pose2D,
                                     goal: Vec2,
                                     command: WheelCommand,
                                     consecutiveCommandFailures: Int) -> [String: String] {
        [
            "goal_x": formatMeters(goal.x),
            "goal_y": formatMeters(goal.y),
            "pose_x": formatMeters(pose.position.x),
            "pose_y": formatMeters(pose.position.y),
            "pose_yaw_deg": formatDegrees(pose.yaw),
            "distance_to_goal": formatMeters(pose.position.distance(to: goal)),
            "wheel_left": formatMeters(command.left),
            "wheel_right": formatMeters(command.right),
            "command_failures": "\(consecutiveCommandFailures)"
        ]
    }

    static func ackAgeField(lastAckAt: Date?, now: Date = Date()) -> String {
        guard let lastAckAt else { return "none" }
        return String(format: "%.2f", now.timeIntervalSince(lastAckAt))
    }

    private static func obstacleMessage(clearance: Double) -> String {
        String(format: "Obstacle ahead at %.2f m.", clearance)
    }

    private static func formatMeters(_ value: Double) -> String {
        String(format: "%.2f", value)
    }

    private static func formatDegrees(_ radians: Double) -> String {
        String(format: "%.0f", radians * 180 / .pi)
    }
}

extension FollowNavigationController.State: CustomStringConvertible {
    public var description: String {
        switch self {
        case .idle: return "idle"
        case .planning: return "planning"
        case .driving: return "driving"
        case .arrived: return "arrived"
        case .failed(let reason): return "failed: \(reason)"
        }
    }
}
