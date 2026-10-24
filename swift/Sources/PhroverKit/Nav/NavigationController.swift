import Foundation
import RoverNav

/// Coarse motion-safety state for observers that only care whether the rover is moving
/// and, if it stopped on a failure, which kind (Silent Search's operator UI).
public enum NavigationSafetyState: Equatable, Sendable {
    case idle
    case moving
    case failed(NavigationFailure)
}

/// Autonomy orchestrator. Runs the closed loop:
///
///   ARKit pose ─┐
///   LiDAR mesh ─┼─► CostmapBuilder ─► AStarPlanner ─► path
///               │                                      │
///   depth ──► ObstacleGuard ──(safe?)──► PursuitController.step(pose, path) ─► WheelCommand ─► RoverControl
///
/// The loop is closed on ARKit visual-inertial pose (no wheel encoders). Replans
/// periodically so newly-seen obstacles (from the growing mesh) are respected.
@Observable
@MainActor
public final class NavigationController {
    public enum State: Equatable, Sendable { case idle, planning, driving, arrived, failed(String) }
    public private(set) var state: State = .idle {
        didSet {
            if !mirroringFollowState { publishSafetyState(Self.safetyState(for: state, policyViolation: lastPolicyViolation)) }
        }
    }
    public private(set) var path: [Vec2] = []
    public private(set) var safetyState: NavigationSafetyState = .idle
    private var safetyStateContinuations: [UUID: AsyncStream<NavigationSafetyState>.Continuation] = [:]
    /// Constrains every plan (initial and replans) of the current navigation, if set.
    private var activePolicy: (any PathAdmissibilityPolicy)?
    /// Why the latest plan was rejected by `activePolicy`, if it was.
    private var lastPolicyViolation: PathPolicyViolation?

    // Follow owns its source-evidence/burst runtime; all admission is serialized by
    // this controller's operation tokens alongside ordinary navigation and scans.
    private var mirroringFollowState = false
    private var followNavigation: FollowNavigationController
    private var followOperation: OperationToken?
    private var followNeedsConfirmedStop = false
    private var followFailureContinuations: [UUID: AsyncStream<FollowMotionFailureDelivery>.Continuation] = [:]

    private let ar: ARSessionManager
    private let control: RoverControl
    private var genericStop: () async throws -> Void
    private let planner = AStarPlanner()
    private let pursuit = PursuitController(params: .init(
        wheelBase: RoverConfig.wheelBase,
        goalTolerance: 0.2,
        minimumRotateWheelSpeed: RoverConfig.minimumRotateWheelSpeed))
    private let guardLayer: ObstacleGuard
    private let safetyFeedback: () -> RoverFeedback?
    private let depthSafetyObservation: (WheelCommand) -> DepthSafetyObservation
    private let planningReadinessSnapshot: () -> PlanningReadinessSnapshot

    private struct OperationToken: Equatable, Sendable {
        let generation: UInt64
    }

    private struct OperationHandoff {
        let token: OperationToken
        let predecessor: Task<Void, Never>?
    }

    private var loop: Task<Void, Never>?
    private var loopOwner: OperationToken?
    private var operationGeneration: UInt64 = 0
    private var relativeHeadingOwner: OperationToken?
    private var replanCounter = 0
    private let trackingRecoveryTimeout: TimeInterval
    private let scanDepthRecoveryTimeout: TimeInterval

    public convenience init(
        ar: ARSessionManager,
        control: RoverControl,
        trackingRecoveryTimeout: TimeInterval = RoverConfig.navigationTrackingRecoveryTimeout,
        scanDepthRecoveryTimeout: TimeInterval = RoverConfig.scanDepthRecoveryTimeout
    ) {
        self.init(
            ar: ar,
            control: control,
            trackingRecoveryTimeout: trackingRecoveryTimeout,
            scanDepthRecoveryTimeout: scanDepthRecoveryTimeout,
            guardLayer: ObstacleGuard(),
            safetyFeedback: { nil },
            depthSafetyObservation: { ar.depthSafetyObservation(for: $0) },
            planningReadinessSnapshot: { ar.planningReadiness }
        )
    }

    init(
        ar: ARSessionManager,
        control: RoverControl,
        trackingRecoveryTimeout: TimeInterval,
        scanDepthRecoveryTimeout: TimeInterval,
        guardLayer: ObstacleGuard,
        safetyFeedback: @escaping () -> RoverFeedback?,
        depthSafetyObservation: @escaping (WheelCommand) -> DepthSafetyObservation,
        planningReadinessSnapshot: @escaping () -> PlanningReadinessSnapshot = {
            PlanningReadinessSnapshot(
                sessionGeneration: 0,
                normalObservationStreak: 3,
                trustedMeshRevision: 1
            )
        }
    ) {
        self.followNavigation = FollowNavigationController(ar: ar, control: control, depthSafetyObservation: depthSafetyObservation)
        self.ar = ar
        self.control = control
        self.genericStop = { try await control.stop() }
        self.trackingRecoveryTimeout = trackingRecoveryTimeout
        self.scanDepthRecoveryTimeout = scanDepthRecoveryTimeout
        self.guardLayer = guardLayer
        self.safetyFeedback = safetyFeedback
        self.depthSafetyObservation = depthSafetyObservation
        self.planningReadinessSnapshot = planningReadinessSnapshot
        configureFollowFailureDelivery()
    }

    convenience init(currentPose: @escaping () -> Pose2D?,
         forwardClearance: @escaping () -> Double,
         depthSafetyObservation: ((WheelCommand) -> DepthSafetyObservation)? = nil,
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
        self.init(ar: ARSessionManager(), control: RoverControl())
        self.genericStop = stopRover
        self.followNavigation = FollowNavigationController(
            currentPose: currentPose, forwardClearance: forwardClearance, depthSafetyObservation: depthSafetyObservation, plan: plan,
            readySignalCostmap: readySignalCostmap, lastAckAt: lastAckAt,
            sendCommand: sendCommand, stopRover: stopRover, sleep: sleep, now: now,
            sendCommandReceipt: sendCommandReceipt, stopRoverReceipt: stopRoverReceipt,
            diagnosticEmitter: diagnosticEmitter, poseSample: poseSample, sourceNow: sourceNow,
            sourceEvents: sourceEvents, sourceStopSnapshot: sourceStopSnapshot,
            sourceHighWater: sourceHighWater, sourceHealth: sourceHealth,
            turnPoseEvidence: turnPoseEvidence, transportUptime: transportUptime)
        configureFollowFailureDelivery()
    }

    func performFollowMotion(_ request: FollowMotionRequest, context: FollowMotionRequestContext?) async -> FollowMotionResult {
        let handoff = claimOperation()
        followOperation = handoff.token
        followNeedsConfirmedStop = true
        activePolicy = nil
        lastPolicyViolation = nil
        let resultTask = Task { @MainActor in
            await handoff.predecessor?.value
            guard self.isOperationActive(handoff.token) else {
                await self.finishCancelledOperationIfCurrent(handoff.token)
                self.clearLoopIfCurrent(handoff.token)
                return FollowMotionResult(result: .cancelled, context: .unknown, failure: nil)
            }
            self.endAnyRelativeHeadingMeasurementIfCurrent(handoff.token)
            self.followNavigation.onStateChange = { [weak self] state, path in
                guard let self, self.isCurrentOperation(handoff.token) else { return }
                self.mirrorFollowState(state, path: path)
            }
            self.followNavigation.onSafetyStateChange = { [weak self] state in
                guard let self, self.isCurrentOperation(handoff.token) else { return }
                self.mirrorFollowSafetyState(state)
            }
            let result = await self.followNavigation.performFollowMotion(request, context: context)
            if self.isCurrentOperation(handoff.token) {
                self.mirroringFollowState = true
                defer { self.mirroringFollowState = false }
                switch result.result {
                case .arrived: self.state = .arrived; self.publishSafetyState(.idle)
                case .cancelled, .reobservationRequired: self.state = .idle; self.publishSafetyState(.idle)
                case .failed(let failure):
                    self.state = .failed(FollowNavigationController.message(for: failure))
                    self.publishSafetyState(.failed(failure))
                }
                self.clearLoopIfCurrent(handoff.token)
            }
            return result
        }
        let task = Task {
            await withTaskCancellationHandler {
                _ = await resultTask.value
            } onCancel: { resultTask.cancel() }
        }
        installLoop(task, operation: handoff.token)
        return await withTaskCancellationHandler {
            await resultTask.value
        } onCancel: { task.cancel(); resultTask.cancel() }
    }

    /// Follow display changes never parse typed failure reasons out of UI text.
    func mirrorFollowState(_ state: FollowNavigationController.State, path: [Vec2]) {
        self.path = path
        mirroringFollowState = true
        defer { mirroringFollowState = false }
        switch state {
        case .idle: self.state = .idle
        case .planning: self.state = .planning
        case .driving: self.state = .driving
        case .arrived: self.state = .arrived
        case .failed(let message): self.state = .failed(message)
        }
    }

    func mirrorFollowSafetyState(_ state: NavigationSafetyState) { publishSafetyState(state) }

    public func navigateForFollow(to goal: Vec2, stoppingAtForwardClearance clearance: Double) async -> NavigationResult {
        await performFollowMotion(.following(goal, clearance), context: nil).result
    }

    public func navigateForFollowReadySignal() async -> NavigationResult {
        await performFollowMotion(.ready, context: nil).result
    }

    public func rotateForFollowScan(by angle: Double) async -> NavigationResult {
        await performFollowMotion(.scan(angle), context: nil).result
    }

    public func rotateForFollowAlignment(by angle: Double) async -> NavigationResult {
        await performFollowMotion(.alignment(angle), context: nil).result
    }

    func updateFollowGoal(_ goal: Vec2) -> Bool {
        guard let owner = followOperation, isCurrentOperation(owner),
              loop?.isCancelled == false else { return false }
        return followNavigation.updateFollowGoal(goal)
    }

    func inhibitFollowScanContinuation(origin: FollowMotionStopOrigin) {
        followNavigation.inhibitFollowScanContinuation(origin: origin)
    }

    var followSourceUptime: TimeInterval { followNavigation.followSourceUptime }
    func ingestFollowTurnSource(_ sample: NavigationPoseSample) { followNavigation.ingestFollowTurnSource(sample) }
    func readFollowPose() -> NavigationPoseSample { followNavigation.readFollowPose() }
    func followMotionFailures() -> AsyncStream<FollowMotionFailureDelivery> {
        let id = UUID()
        return AsyncStream { continuation in
            followFailureContinuations[id] = continuation
            if case .failed(let reason) = safetyState {
                continuation.yield(.init(context: .unknown, reason: reason,
                    stopOutcome: followNavigation.hasUnconfirmedStop ? .failed : .unknown, source: .stream))
            }
            continuation.onTermination = { @Sendable [weak self] _ in
                Task { @MainActor in self?.followFailureContinuations[id] = nil }
            }
        }
    }

    private func configureFollowFailureDelivery() {
        followNavigation.onFailure = { [weak self] failure in self?.deliverFollowFailure(failure) }
    }

    private func deliverFollowFailure(_ failure: FollowMotionFailureDelivery) {
        for continuation in followFailureContinuations.values { continuation.yield(failure) }
    }

    public func stopAndConfirm() async throws {
        let handoff = claimOperation()
        followNeedsConfirmedStop = true
        let stop = Task { @MainActor in
            await handoff.predecessor?.value
            // The successor waits for this installed task, so an explicitly
            // requested confirmation must run even if ownership changed while
            // the predecessor was draining. Returning early would falsely
            // acknowledge a stop that never reached the transport.
            self.endAnyRelativeHeadingMeasurementIfCurrent(handoff.token)
            do {
                try await self.followNavigation.stopAndConfirm()
                guard self.isCurrentOperation(handoff.token) else { return }
                self.followNeedsConfirmedStop = false
                self.state = .idle
                self.path = []
            } catch {
                self.setStateIfCurrent(.failed("Rover command failed: stop could not be confirmed."), operation: handoff.token)
                throw error
            }
        }
        let task = Task { _ = try? await stop.value }
        installLoop(task, operation: handoff.token)
        do {
            try await stop.value
            clearLoopIfCurrent(handoff.token)
        } catch {
            clearLoopIfCurrent(handoff.token)
            throw error
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

    /// Drive to `goal` and wait for the outcome. `policy`, if given, must admit every plan —
    /// the initial one and each replan — or navigation stops with `.pathRejected`. A newer
    /// navigation, rotation or stop supersedes this one and it returns `.cancelled`.
    public func navigateAndWait(
        to goal: Vec2,
        stoppingAtForwardClearance clearance: Double? = nil,
        policy: (any PathAdmissibilityPolicy)? = nil
    ) async -> NavigationResult {
        let (task, token) = startNavigation(to: goal, stoppingAtForwardClearance: clearance, policy: policy)
        await awaitOwnedOperation(task)
        return navigationResult(for: token)
    }

    /// `rotate(by:)`, returning the typed outcome.
    public func rotateAndWait(by angle: Double) async -> NavigationResult {
        let (task, token) = startRotation(by: angle)
        await awaitOwnedOperation(task)
        return navigationResult(for: token)
    }

    /// Stop, and return once the rover has been told to stop.
    public func cancelAndWait() async {
        await stopAndWait()
    }

    /// Every change of `safetyState`, starting with the current one.
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

    private func publishSafetyState(_ newState: NavigationSafetyState) {
        guard safetyState != newState else { return }
        safetyState = newState
        // Engine-owned failures retain their captured operation attribution. Ordinary
        // navigation failures use unknown attribution so an active follow observer
        // can stop safely without borrowing the preceding follow operation's identity.
        // A pending confirmed stop also belongs to the engine and reports its own failure.
        if case .failed(let reason) = newState, followOperation == nil, !followNeedsConfirmedStop {
            deliverFollowFailure(.init(context: .unknown, reason: reason, stopOutcome: .unknown, source: .stream))
        }
        for continuation in safetyStateContinuations.values { continuation.yield(newState) }
    }

    private func navigationResult(for token: OperationToken) -> NavigationResult {
        guard isCurrentOperation(token) else { return .cancelled }
        switch state {
        case .arrived: return .arrived
        case .failed(let message):
            return .failed(Self.navigationFailure(forMessage: message, policyViolation: lastPolicyViolation))
        case .idle, .planning, .driving: return .cancelled
        }
    }

    static func safetyState(for state: State, policyViolation: PathPolicyViolation?) -> NavigationSafetyState {
        switch state {
        case .idle, .arrived: .idle
        case .planning, .driving: .moving
        case .failed(let message): .failed(navigationFailure(forMessage: message, policyViolation: policyViolation))
        }
    }

    /// Typed reason for a failure state. The failure sites set human-readable messages; this
    /// maps them back so callers (Silent Search) can branch without parsing text themselves.
    static func navigationFailure(forMessage message: String,
                                  policyViolation: PathPolicyViolation?) -> NavigationFailure {
        if message.hasPrefix(Self.pathPolicyRejectionMessage), let policyViolation {
            return .pathRejected(policyViolation)
        }
        if message.hasPrefix("No ARKit pose") { return .noPose }
        if message.hasPrefix("No path to goal") || message.hasPrefix(Self.pathPolicyRejectionMessage) {
            return .noPath
        }
        if message.hasPrefix("Rover command link lost") { return .commsLost }
        if message.hasPrefix("Rover may be tipping") { return .tipping }
        if message.hasPrefix("Rover command failed") || message.hasPrefix("Rover stop could not be confirmed") {
            return .commandFailed
        }
        if message.hasPrefix("Rotation stopped: observed response") { return .rotationResolutionInsufficient }
        if message.hasPrefix("Navigation stalled") || message.hasPrefix("Scan turn moved")
            || message.hasPrefix("Rover moved unexpectedly") {
            return .stalled
        }
        if message.hasPrefix("Obstacle") || message.hasPrefix("Depth safety")
            || message.hasPrefix("I can’t safely see") {
            return .obstacle
        }
        // What remains is AR tracking, relative-heading and fresh-frame readiness.
        return .trackingLost
    }

    static let pathPolicyRejectionMessage = "Path rejected by navigation policy."

    public func assessGoal(_ goal: Vec2) -> NavigationGoalAssessment {
        guard let start = ar.pose?.position else {
            return NavigationGoalAssessment(
                goal: goal,
                isReachable: false,
                pathDistance: .infinity,
                rejectionReason: .missingPose
            )
        }
        guard planningReadinessSnapshot().isPoseReady,
              Self.isNavigationObservationUsable(ar.latestObservation) else {
            return NavigationGoalAssessment(
                goal: goal,
                isReachable: false,
                pathDistance: .infinity,
                rejectionReason: .trackingUnstable
            )
        }
        let costmap = CostmapBuilder.build(from: ar.meshAnchors, center: start)
        let planningResult = planner.assess(from: start, to: goal, in: costmap)
        guard case .path(let candidatePath) = planningResult else {
            guard case .rejected(let reason) = planningResult else {
                preconditionFailure("Unexpected planner result")
            }
            return NavigationGoalAssessment(
                goal: goal,
                isReachable: false,
                pathDistance: .infinity,
                rejectionReason: Self.navigationRejection(for: reason)
            )
        }
        let distance = zip(candidatePath, candidatePath.dropFirst()).reduce(0) {
            $0 + $1.0.distance(to: $1.1)
        }
        return NavigationGoalAssessment(goal: goal, isReachable: true, pathDistance: distance)
    }

    @discardableResult
    private func startNavigation(
        to goal: Vec2,
        stoppingAtForwardClearance: Double?,
        policy: (any PathAdmissibilityPolicy)?
    ) -> (Task<Void, Never>, OperationToken) {
        let handoff = claimOperation()
        activePolicy = policy
        lastPolicyViolation = nil
        state = .planning
        let task = Task {
            await handoff.predecessor?.value
            guard self.isOperationActive(handoff.token) else {
                await self.finishCancelledOperationIfCurrent(handoff.token)
                self.clearLoopIfCurrent(handoff.token)
                return
            }
            self.endAnyRelativeHeadingMeasurementIfCurrent(handoff.token)
            guard await self.stopTransportIfCurrent(handoff.token),
                  self.isOperationActive(handoff.token) else {
                await self.finishCancelledOperationIfCurrent(handoff.token)
                self.clearLoopIfCurrent(handoff.token)
                return
            }
            let readinessOutcome = await self.waitForInitialPlanningContext(operation: handoff.token)
            guard readinessOutcome == .ready,
                  self.isOperationActive(handoff.token),
                  let start = self.ar.pose?.position else {
                if self.isOperationActive(handoff.token) {
                    let message = readinessOutcome == .trackingTimeout
                        ? "AR tracking did not recover in time."
                        : "AR tracking or mesh did not become ready for planning."
                    self.setStateIfCurrent(
                        .failed(message),
                        operation: handoff.token
                    )
                }
                self.clearLoopIfCurrent(handoff.token)
                return
            }
            switch self.planAndStore(from: start, to: goal) {
            case .path:
                break
            case .rejected(let reason):
                self.setStateIfCurrent(
                    .failed(self.lastPolicyViolation == nil ? "No path to goal." : Self.pathPolicyRejectionMessage),
                    operation: handoff.token
                )
                RuntimeFileLog.append("nav_planning_failed", fields: [
                    "rejection_reason": self.lastPolicyViolation == nil
                        ? Self.navigationRejection(for: reason).rawValue
                        : "path_policy_rejected"
                ])
                self.clearLoopIfCurrent(handoff.token)
                return
            }
            RuntimeFileLog.append("nav_goal_start", fields: [
                "goal_x": Self.formatMeters(goal.x),
                "goal_y": Self.formatMeters(goal.y),
                "pose_x": Self.formatMeters(start.x),
                "pose_y": Self.formatMeters(start.y),
                "distance_to_goal": Self.formatMeters(start.distance(to: goal)),
                "target_stop_clearance": stoppingAtForwardClearance.map(Self.formatMeters) ?? "none"
            ])
            self.setStateIfCurrent(.driving, operation: handoff.token)
            await self.drive(
                to: goal,
                stoppingAtForwardClearance: stoppingAtForwardClearance,
                operation: handoff.token
            )
            if Task.isCancelled {
                await self.finishCancelledOperationIfCurrent(handoff.token)
            }
            self.clearLoopIfCurrent(handoff.token)
        }
        installLoop(task, operation: handoff.token)
        return (task, handoff.token)
    }

    public func preparePlanningContext(
        requiring requirement: PlanningContextRequirement
    ) async -> PlanningRecoveryOutcome {
        await stopAndWait()
        guard !Task.isCancelled else { return .cancelled }
        let initial = planningReadinessSnapshot()
        let generation = ar.sessionGeneration
        let deadline = Date().addingTimeInterval(trackingRecoveryTimeout)
        var reportedPoseReady = false
        RuntimeFileLog.append("nav_planning_recovery_started", fields: [
            "requirement": requirement.rawValue,
            "session_generation": "\(generation)",
            "pose_streak": "\(initial.normalObservationStreak)",
            "mesh_revision": "\(initial.trustedMeshRevision)",
            "timeout": Self.formatSeconds(trackingRecoveryTimeout)
        ])

        while Date() < deadline {
            guard !Task.isCancelled else {
                return planningRecoveryFailed(.cancelled, requirement: requirement)
            }
            guard ar.sessionGeneration == generation else {
                return planningRecoveryFailed(.sessionGenerationChanged, requirement: requirement)
            }
            let snapshot = planningReadinessSnapshot()
            if snapshot.isPoseReady,
               Self.isNavigationObservationUsable(ar.latestObservation) {
                if !reportedPoseReady {
                    reportedPoseReady = true
                    RuntimeFileLog.append("nav_planning_pose_ready", fields: [
                        "requirement": requirement.rawValue,
                        "pose_streak": "\(snapshot.normalObservationStreak)",
                        "session_generation": "\(generation)"
                    ])
                }
                if requirement == .stablePose {
                    return .ready
                }
                if snapshot.trustedMeshRevision > initial.trustedMeshRevision {
                    RuntimeFileLog.append("nav_planning_mesh_refreshed", fields: [
                        "requirement": requirement.rawValue,
                        "previous_revision": "\(initial.trustedMeshRevision)",
                        "current_revision": "\(snapshot.trustedMeshRevision)"
                    ])
                    return .ready
                }
            }
            try? await Task.sleep(for: .seconds(RoverConfig.navigationTrackingPollInterval))
        }
        let outcome: PlanningRecoveryOutcome = requirement == .stablePose
            ? .trackingTimeout
            : (reportedPoseReady ? .meshTimeout : .trackingTimeout)
        return planningRecoveryFailed(outcome, requirement: requirement)
    }

    public func recoverPlanningContext() async -> PlanningRecoveryOutcome {
        await preparePlanningContext(requiring: .refreshedTrustedMesh)
    }

    /// Rotate in place by `angle` radians (CCW positive, matching `Pose2D.yaw`) and wait
    /// for it to finish. A pure turn, no path planning — used by the mission agent to scan
    /// for something not currently in view (e.g. up to a full `2 * .pi` look-around).
    /// Uses the same command cadence/watchdog as `drive()`, but does not treat forward
    /// clearance as a hard stop because this is an in-place search turn, not forward motion.
    public func rotate(by angle: Double) async {
        let (task, _) = startRotation(by: angle)
        await awaitOwnedOperation(task)
    }

    private func startRotation(by angle: Double) -> (Task<Void, Never>, OperationToken) {
        let handoff = claimOperation()
        let task = Task {
            await handoff.predecessor?.value
            guard self.isOperationActive(handoff.token) else {
                await self.finishCancelledOperationIfCurrent(handoff.token)
                self.clearLoopIfCurrent(handoff.token)
                return
            }
            self.endAnyRelativeHeadingMeasurementIfCurrent(handoff.token)
            guard await self.stopTransportIfCurrent(handoff.token),
                  self.isOperationActive(handoff.token) else {
                await self.finishCancelledOperationIfCurrent(handoff.token)
                self.clearLoopIfCurrent(handoff.token)
                return
            }
            guard let startYaw = self.ar.pose?.yaw else {
                self.setStateIfCurrent(
                    .failed("No ARKit pose yet — move the device to establish tracking."),
                    operation: handoff.token
                )
                self.clearLoopIfCurrent(handoff.token)
                return
            }
            let targetYaw = normalizeAngle(startYaw + angle)
            self.setStateIfCurrent(.driving, operation: handoff.token)
            await self.performContinuousRotation(to: targetYaw, operation: handoff.token)
            if Task.isCancelled {
                await self.finishCancelledOperationIfCurrent(handoff.token)
            }
            self.clearLoopIfCurrent(handoff.token)
        }
        installLoop(task, operation: handoff.token)
        return (task, handoff.token)
    }

    /// Rotate in short pulses for camera-based target search. Stopping between pulses
    /// prevents a 30-degree scan step from sweeping past the object before detection can
    /// process a stable frame.
    public func rotateForScan(by angle: Double) async {
        let handoff = claimOperation()
        let task = Task {
            await handoff.predecessor?.value
            guard self.isOperationActive(handoff.token) else {
                await self.finishCancelledOperationIfCurrent(handoff.token)
                self.clearLoopIfCurrent(handoff.token)
                return
            }
            self.endAnyRelativeHeadingMeasurementIfCurrent(handoff.token)
            guard await self.stopTransportIfCurrent(handoff.token),
                  self.isOperationActive(handoff.token),
                  await self.waitForUsableTracking(operation: handoff.token),
                  self.isOperationActive(handoff.token),
                  let startPose = self.ar.pose else {
                if self.isOperationActive(handoff.token) {
                    self.setStateIfCurrent(
                        .failed("AR tracking is not ready — keep the phone still and try again."),
                        operation: handoff.token
                    )
                } else {
                    await self.finishCancelledOperationIfCurrent(handoff.token)
                }
                self.clearLoopIfCurrent(handoff.token)
                return
            }

            guard self.beginRelativeHeadingMeasurementIfCurrent(handoff.token),
                  await self.waitForReliableRelativeHeading(operation: handoff.token) != nil,
                  self.isOperationActive(handoff.token) else {
                self.endRelativeHeadingMeasurementIfOwned(by: handoff.token)
                if self.isOperationActive(handoff.token) {
                    self.setStateIfCurrent(
                        .failed("Relative heading is not ready — keep the phone still and try again."),
                        operation: handoff.token
                    )
                } else {
                    await self.finishCancelledOperationIfCurrent(handoff.token)
                }
                self.clearLoopIfCurrent(handoff.token)
                return
            }

            self.setStateIfCurrent(.driving, operation: handoff.token)
            await self.performScanRotation(
                from: startPose,
                requestedScanAngle: normalizeAngle(angle),
                operation: handoff.token
            )
            self.endRelativeHeadingMeasurementIfOwned(by: handoff.token)
            if Task.isCancelled {
                await self.finishCancelledOperationIfCurrent(handoff.token)
            } else {
                guard await self.stopTransportForActiveOperation(handoff.token) else {
                    await self.finishCancelledOperationIfCurrent(handoff.token)
                    self.clearLoopIfCurrent(handoff.token)
                    return
                }
            }
            self.clearLoopIfCurrent(handoff.token)
        }
        installLoop(task, operation: handoff.token)
        await awaitOwnedOperation(task)
    }

    /// Stop and clear the current goal.
    public func cancel() {
        let handoff = claimOperation()
        // Without this, an external cancel (e.g. a hard-stop bypassing the brain) leaves
        // `state` at `.driving` forever, so anything polling `state == .driving` to know
        // when motion has settled never returns.
        state = .idle
        installStopOnlyOperation(handoff)
    }

    public func stopAndWait() async {
        let handoff = claimOperation()
        state = .idle
        let task = makeStopOnlyOperation(handoff)
        installLoop(task, operation: handoff.token)
        await task.value
    }

    private func claimOperation() -> OperationHandoff {
        if followOperation != nil {
            followNavigation.inhibitForHandoff()
            followOperation = nil
        }
        operationGeneration &+= 1
        let token = OperationToken(generation: operationGeneration)
        let predecessor = loop
        predecessor?.cancel()
        loop = nil
        loopOwner = nil
        return OperationHandoff(token: token, predecessor: predecessor)
    }

    private func installLoop(_ task: Task<Void, Never>, operation: OperationToken) {
        guard isCurrentOperation(operation) else {
            task.cancel()
            return
        }
        loop = task
        loopOwner = operation
    }

    private func clearLoopIfCurrent(_ operation: OperationToken) {
        guard loopOwner == operation else { return }
        loop = nil
        loopOwner = nil
    }

    private func isCurrentOperation(_ operation: OperationToken) -> Bool {
        operationGeneration == operation.generation
    }

    private func isOperationActive(_ operation: OperationToken) -> Bool {
        isCurrentOperation(operation) && !Task.isCancelled
    }

    private func setStateIfCurrent(_ newState: State, operation: OperationToken) {
        guard isCurrentOperation(operation) else { return }
        state = newState
    }

    private func awaitOwnedOperation(_ task: Task<Void, Never>) async {
        await withTaskCancellationHandler {
            await task.value
        } onCancel: {
            task.cancel()
        }
    }

    private func installStopOnlyOperation(_ handoff: OperationHandoff) {
        let task = makeStopOnlyOperation(handoff)
        installLoop(task, operation: handoff.token)
    }

    private func makeStopOnlyOperation(_ handoff: OperationHandoff) -> Task<Void, Never> {
        Task {
            await handoff.predecessor?.value
            guard self.isCurrentOperation(handoff.token) else { return }
            self.endAnyRelativeHeadingMeasurementIfCurrent(handoff.token)
            _ = await self.stopTransportIfCurrent(handoff.token)
            self.clearLoopIfCurrent(handoff.token)
        }
    }

    @discardableResult
    private func stopTransportIfCurrent(_ operation: OperationToken) async -> Bool {
        guard isCurrentOperation(operation) else { return false }
        let stopTask = Task { @MainActor [weak self] in
            guard self?.isCurrentOperation(operation) == true else { return false }
            if let self, self.followNeedsConfirmedStop {
                do {
                    try await self.followNavigation.stopAndConfirm()
                    guard self.isCurrentOperation(operation) else { return false }
                    self.followNeedsConfirmedStop = false
                } catch {
                    self.setStateIfCurrent(.failed("Rover command failed: stop could not be confirmed."), operation: operation)
                    return false
                }
            } else {
                try? await self?.genericStop()
            }
            return self?.isCurrentOperation(operation) == true
        }
        return await stopTask.value
    }

    private func stopTransportForActiveOperation(_ operation: OperationToken) async -> Bool {
        guard isOperationActive(operation),
              await stopTransportIfCurrent(operation) else { return false }
        return isOperationActive(operation)
    }

    private func finishCancelledOperationIfCurrent(_ operation: OperationToken) async {
        guard isCurrentOperation(operation) else { return }
        endRelativeHeadingMeasurementIfOwned(by: operation)
        guard await stopTransportIfCurrent(operation) else { return }
        setStateIfCurrent(.idle, operation: operation)
    }

    private func beginRelativeHeadingMeasurementIfCurrent(_ operation: OperationToken) -> Bool {
        guard isCurrentOperation(operation) else { return false }
        ar.beginRelativeHeadingMeasurement()
        relativeHeadingOwner = operation
        return true
    }

    private func endRelativeHeadingMeasurementIfOwned(by operation: OperationToken) {
        guard relativeHeadingOwner == operation else { return }
        ar.endRelativeHeadingMeasurement()
        relativeHeadingOwner = nil
    }

    private func endAnyRelativeHeadingMeasurementIfCurrent(_ operation: OperationToken) {
        guard isCurrentOperation(operation), relativeHeadingOwner != nil else { return }
        ar.endRelativeHeadingMeasurement()
        relativeHeadingOwner = nil
    }

    // MARK: - Loop

    private func drive(to goal: Vec2,
                       stoppingAtForwardClearance targetStopDistance: Double?,
                       operation: OperationToken) async {
        var hasSentCommand = false
        var consecutiveCommandFailures = 0
        var progressWatchdog = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.05)
        while isOperationActive(operation) {
            guard await waitForUsableTracking(operation: operation),
                  isOperationActive(operation),
                  let pose = ar.pose else {
                if await stopTransportForActiveOperation(operation) {
                    setStateIfCurrent(.failed("AR tracking did not recover in time."), operation: operation)
                    RuntimeFileLog.append("nav_safety_stop", fields: ["reason": "tracking_unavailable"])
                }
                return
            }
            let distanceToGoal = pose.position.distance(to: goal)
            let isTargetApproach = targetStopDistance != nil
                && distanceToGoal <= RoverConfig.visualTargetApproachDistance

            // Replan every ~1s to fold in newly meshed obstacles.
            replanCounter += 1
            if replanCounter % 10 == 0,
               case .rejected(let rejection) = planAndStore(from: pose.position, to: goal) {
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(
                    .failed(lastPolicyViolation == nil ? "No path to goal after replanning." : Self.pathPolicyRejectionMessage),
                    operation: operation
                )
                RuntimeFileLog.append("nav_safety_stop", fields: [
                    "reason": lastPolicyViolation == nil ? "replan_failed" : "path_policy_rejected",
                    "rejection_reason": Self.navigationRejection(for: rejection).rawValue,
                    "distance_to_goal": Self.formatMeters(distanceToGoal)
                ])
                return
            }

            // Safety gate.
            let lastAck = await control.lastAckAt
            guard isOperationActive(operation) else { return }
            let now = Date()
            let decision = guardLayer.evaluate(forwardClearance: .infinity,
                                               lastAckAt: lastAck,
                                               now: now,
                                               feedback: nil,
                                               requireFreshAck: hasSentCommand,
                                               checkForwardObstacle: false)
            switch decision {
            case .go:
                break
            case .stopObstacle(let clearance):
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(
                    Self.stateAfterObstacleStop(pose: pose, goal: goal, clearance: clearance),
                    operation: operation
                )
                RuntimeFileLog.append("nav_safety_stop", fields: [
                    "reason": "obstacle",
                    "clearance": String(format: "%.2f", clearance),
                    "state": state.description
                ])
                return
            case .stopCommsLost:
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(.failed("Rover command link lost."), operation: operation)
                RuntimeFileLog.append("nav_safety_stop", fields: [
                    "reason": "comms_lost",
                    "ack_age": Self.ackAgeField(lastAckAt: lastAck, now: now)
                ])
                return
            case .stopTipping:
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(.failed("Rover may be tipping."), operation: operation)
                RuntimeFileLog.append("nav_safety_stop", fields: ["reason": "tipping"])
                return
            }

            let out = pursuit.step(pose: pose, path: path)
            if out.reachedGoal {
                // Probe the forward envelope without sending motion so stale or blind depth
                // cannot turn an unsafe stop into a successful arrival.
                let arrivalProbe = WheelCommand(left: 0.02, right: 0.02)
                let arrivalSafety = depthSafetyObservation(arrivalProbe)
                switch guardLayer.evaluate(command: arrivalProbe, depthSafety: arrivalSafety) {
                case .allow(_, _):
                    guard await stopTransportForActiveOperation(operation) else { return }
                    setStateIfCurrent(.arrived, operation: operation)
                case .stopDepth(let observation):
                    if observation.state == .unavailable(.staleRawDepth) {
                        let depthVersion = ar.depthSnapshotVersion
                        guard await stopTransportForActiveOperation(operation) else { return }
                        RuntimeFileLog.append("nav_arrival_depth_retry_started", fields: [
                            "depth_age": Self.formatSeconds(observation.sampleAge),
                            "depth_version": String(depthVersion),
                            "timeout_seconds": Self.formatSeconds(scanDepthRecoveryTimeout)
                        ])
                        let depthRefreshed = await waitForNewDepthSnapshot(
                            after: depthVersion,
                            operation: operation
                        )
                        guard isOperationActive(operation) else { return }
                        if depthRefreshed {
                            RuntimeFileLog.append("nav_arrival_depth_fresh_snapshot", fields: [
                                "previous_version": String(depthVersion),
                                "depth_version": String(ar.depthSnapshotVersion),
                                "depth_timestamp": ar.depthSnapshotTimestamp.map(Self.formatSeconds) ?? "none"
                            ])
                            continue
                        }
                        RuntimeFileLog.append("nav_arrival_depth_retry_timeout", fields: [
                            "depth_version": String(depthVersion)
                        ])
                    } else {
                        guard await stopTransportForActiveOperation(operation) else { return }
                    }
                    let reason = observation.state.telemetryReason
                    setStateIfCurrent(
                        .failed("Depth safety stop at arrival: \(reason)."),
                        operation: operation
                    )
                    RuntimeFileLog.append("nav_safety_stop", fields: [
                        "reason": "depth_safety_at_arrival",
                        "depth_state": reason,
                        "depth_age": Self.formatSeconds(observation.sampleAge),
                        "support_count": "\(observation.supportCount)"
                    ])
                }
                return
            }
            let plannedCommand: WheelCommand
            if isTargetApproach {
                let maximum = max(abs(out.command.left), abs(out.command.right))
                let scale = maximum > RoverConfig.visualTargetApproachMaxWheelSpeed
                    ? RoverConfig.visualTargetApproachMaxWheelSpeed / maximum
                    : 1
                plannedCommand = WheelCommand(
                    left: out.command.left * scale,
                    right: out.command.right * scale
                )
            } else {
                plannedCommand = out.command
            }
            let depthVisibleCommand = Self.depthVisibleForwardCommand(plannedCommand)
            let command: WheelCommand
            let depthSafety: DepthSafetyObservation
            if DepthSafetyMotionClass.classify(depthVisibleCommand) == .rotating {
                guard let recoveredCommand = await rotationSafetyCommand(
                    depthVisibleCommand,
                    hasSentCommand: hasSentCommand,
                    operation: operation
                ) else { return }
                guard isOperationActive(operation) else { return }
                command = recoveredCommand
                depthSafety = depthSafetyObservation(recoveredCommand)
            } else {
                depthSafety = depthSafetyObservation(depthVisibleCommand)
                switch guardLayer.evaluate(command: depthVisibleCommand, depthSafety: depthSafety) {
                case .allow(let safeCommand, _):
                    command = safeCommand
                case .stopDepth(let observation):
                    if observation.state == .unavailable(.staleRawDepth) {
                        let depthVersion = ar.depthSnapshotVersion
                        guard await stopTransportForActiveOperation(operation) else { return }
                        RuntimeFileLog.append("nav_depth_retry_started", fields: [
                            "depth_age": Self.formatSeconds(observation.sampleAge),
                            "depth_version": String(depthVersion),
                            "timeout_seconds": Self.formatSeconds(scanDepthRecoveryTimeout)
                        ])
                        let depthRefreshed = await waitForNewDepthSnapshot(
                            after: depthVersion,
                            operation: operation
                        )
                        guard isOperationActive(operation) else { return }
                        guard depthRefreshed else {
                            setStateIfCurrent(
                                .failed("Depth safety stop: stale_raw_depth."),
                                operation: operation
                            )
                            RuntimeFileLog.append("nav_depth_retry_timeout", fields: [
                                "depth_version": String(depthVersion)
                            ])
                            RuntimeFileLog.append("nav_safety_stop", fields: [
                                "reason": "depth_safety",
                                "depth_state": observation.state.telemetryReason,
                                "clearance": Self.formatMeters(observation.clearance),
                                "depth_age": Self.formatSeconds(observation.sampleAge),
                                "support_count": "\(observation.supportCount)",
                                "motion_class": observation.motionClass.rawValue
                            ])
                            return
                        }
                        RuntimeFileLog.append("nav_depth_fresh_snapshot", fields: [
                            "previous_version": String(depthVersion),
                            "depth_version": String(ar.depthSnapshotVersion),
                            "depth_timestamp": ar.depthSnapshotTimestamp.map(Self.formatSeconds) ?? "none"
                        ])
                        continue
                    }
                    guard await stopTransportForActiveOperation(operation) else { return }
                    let reason = observation.state.telemetryReason
                    setStateIfCurrent(.failed("Depth safety stop: \(reason)."), operation: operation)
                    RuntimeFileLog.append("nav_safety_stop", fields: [
                        "reason": "depth_safety",
                        "depth_state": reason,
                        "clearance": Self.formatMeters(observation.clearance),
                        "depth_age": Self.formatSeconds(observation.sampleAge),
                        "support_count": "\(observation.supportCount)",
                        "motion_class": observation.motionClass.rawValue
                    ])
                    return
                }
            }
            let isCommanded = abs(command.left) > 0.01 || abs(command.right) > 0.01
            if progressWatchdog.observe(distanceToGoal: distanceToGoal,
                                        now: now,
                                        commanded: isCommanded) {
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(.failed("Navigation stalled."), operation: operation)
                RuntimeFileLog.append("nav_safety_stop", fields: [
                    "reason": "no_goal_progress",
                    "distance_to_goal": Self.formatMeters(distanceToGoal),
                    "timeout": "2.50"
                ])
                return
            }
            var telemetry = Self.driveTelemetryFields(pose: pose,
                                                       goal: goal,
                                                       command: command,
                                                       consecutiveCommandFailures: consecutiveCommandFailures)
            telemetry["depth_safety_state"] = depthSafety.state.telemetryReason
            telemetry["depth_safety_clearance"] = Self.formatMeters(depthSafety.clearance)
            telemetry["depth_safety_age"] = Self.formatSeconds(depthSafety.sampleAge)
            telemetry["depth_safety_support"] = "\(depthSafety.supportCount)"
            telemetry["path_points"] = "\(path.count)"
            telemetry["target_approach_slowed"] = plannedCommand == out.command ? "false" : "true"
            telemetry["depth_visible_curvature_limited"] = depthVisibleCommand == plannedCommand
                ? "false"
                : "true"
            RuntimeFileLog.append("nav_drive_tick", fields: telemetry)
            do {
                guard isOperationActive(operation) else { return }
                try await control.sendNavigation(command)
                guard isOperationActive(operation) else { return }
                consecutiveCommandFailures = 0
                hasSentCommand = true
            } catch {
                guard isOperationActive(operation) else { return }
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
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(Self.stateAfterCommandFailure(error), operation: operation)
                RuntimeFileLog.append("nav_command_failed", fields: [
                    "error": error.localizedDescription,
                    "state": state.description
                ])
                return
            }
            do {
                try await Task.sleep(for: .seconds(RoverConfig.commandInterval))
            } catch {
                break
            }
        }
        await finishCancelledOperationIfCurrent(operation)
    }

    private func waitForUsableTracking(operation: OperationToken) async -> Bool {
        guard isOperationActive(operation) else { return false }
        if isPlanningPoseReady() { return true }
        guard await stopTransportForActiveOperation(operation) else { return false }
        let deadline = Date().addingTimeInterval(trackingRecoveryTimeout)
        while isOperationActive(operation), Date() < deadline {
            if isPlanningPoseReady() { return true }
            do {
                try await Task.sleep(for: .seconds(RoverConfig.navigationTrackingPollInterval))
            } catch {
                return false
            }
        }
        return false
    }

    private func isPlanningPoseReady() -> Bool {
        planningReadinessSnapshot().isPoseReady
            && Self.isNavigationObservationUsable(ar.latestObservation)
    }

    private func waitForInitialPlanningContext(
        operation: OperationToken
    ) async -> PlanningRecoveryOutcome {
        let deadline = Date().addingTimeInterval(trackingRecoveryTimeout)
        var sawReadyPose = false
        while isOperationActive(operation), Date() < deadline {
            let snapshot = planningReadinessSnapshot()
            if isPlanningPoseReady() {
                sawReadyPose = true
                if snapshot.trustedMeshRevision > 0 { return .ready }
            }
            try? await Task.sleep(for: .seconds(RoverConfig.navigationTrackingPollInterval))
        }
        guard isOperationActive(operation), !Task.isCancelled else { return .cancelled }
        return sawReadyPose ? .meshTimeout : .trackingTimeout
    }

    static func isNavigationObservationUsable(
        _ observation: PoseObservation?,
        uptime: TimeInterval = ProcessInfo.processInfo.systemUptime
    ) -> Bool {
        guard let observation, observation.trackingQuality == .normal else { return false }
        return uptime - observation.timestamp <= RoverConfig.navigationTrackingFreshness
    }

    @discardableResult
    private func planAndStore(from: Vec2, to: Vec2) -> PathPlanningResult {
        let costmap = CostmapBuilder.build(from: ar.meshAnchors, center: from)
        let result = planner.assess(from: from, to: to, in: costmap)
        guard case .path(let p) = result else {
            path.removeAll()
            lastPolicyViolation = nil
            return result
        }
        if let activePolicy,
           case .rejected(let violation) = activePolicy.evaluate(path: [from] + p + [to]) {
            path.removeAll()
            lastPolicyViolation = violation
            RuntimeFileLog.append("nav_path_policy_rejected", fields: ["violation": "\(violation)"])
            return .rejected(.noConnectedPath)
        }
        lastPolicyViolation = nil
        path = p
        return result
    }

    private func planningRecoveryFailed(
        _ outcome: PlanningRecoveryOutcome,
        requirement: PlanningContextRequirement
    ) -> PlanningRecoveryOutcome {
        let snapshot = planningReadinessSnapshot()
        RuntimeFileLog.append("nav_planning_recovery_failed", fields: [
            "requirement": requirement.rawValue,
            "outcome": outcome.rawValue,
            "session_generation": "\(ar.sessionGeneration)",
            "pose_streak": "\(snapshot.normalObservationStreak)",
            "mesh_revision": "\(snapshot.trustedMeshRevision)",
            "tracking_quality": Self.trackingQualityDescription(ar.latestObservation?.trackingQuality)
        ])
        return outcome
    }

    private static func navigationRejection(for failure: PathPlanningFailure) -> NavigationGoalRejection {
        switch failure {
        case .startOutsideMap: .startOutsideMap
        case .goalOutsideMap: .goalOutsideMap
        case .startBlocked: .startBlocked
        case .goalBlocked: .goalBlocked
        case .noConnectedPath: .noConnectedPath
        }
    }

    private static func trackingQualityDescription(_ quality: PoseTrackingQuality?) -> String {
        switch quality {
        case .normal: "normal"
        case .limited: "limited"
        case .unavailable, nil: "unavailable"
        }
    }

    private func performContinuousRotation(to targetYaw: Double,
                                           operation: OperationToken) async {
        var hasSentCommand = false
        // A commanded turn whose yaw error stops shrinking (wheels stalled, rover wedged)
        // fails closed instead of driving the motors indefinitely.
        var progressWatchdog = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.05)
        while isOperationActive(operation) {
            guard let pose = ar.pose else {
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(
                    .failed("AR tracking pose became unavailable during rotation."),
                    operation: operation
                )
                return
            }
            let error = normalizeAngle(targetYaw - pose.yaw)
            if abs(error) <= 0.05 {
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(.arrived, operation: operation)
                return
            }
            let plannedCommand = RotationCommand.command(forYawError: error)
            guard let cmd = await rotationSafetyCommand(
                plannedCommand,
                hasSentCommand: hasSentCommand,
                operation: operation
            ) else { return }
            guard isOperationActive(operation) else { return }
            if progressWatchdog.observe(distanceToGoal: abs(error), now: Date(), commanded: true) {
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(.failed("Navigation stalled."), operation: operation)
                RuntimeFileLog.append("nav_safety_stop", fields: [
                    "reason": "no_yaw_progress",
                    "target_yaw_deg": Self.formatDegrees(targetYaw),
                    "yaw_error_deg": Self.formatDegrees(error),
                    "timeout": "2.50"
                ])
                return
            }
            RuntimeFileLog.append("nav_rotate_tick", fields: [
                "pose_x": Self.formatMeters(pose.position.x),
                "pose_y": Self.formatMeters(pose.position.y),
                "pose_yaw_deg": Self.formatDegrees(pose.yaw),
                "target_yaw_deg": Self.formatDegrees(targetYaw),
                "yaw_error_deg": Self.formatDegrees(error),
                "mode": "continuous",
                "heading_source": "arkit",
                "wheel_left": Self.formatMeters(cmd.left),
                "wheel_right": Self.formatMeters(cmd.right)
            ])
            do {
                try await control.sendNavigation(cmd)
                guard isOperationActive(operation) else { return }
                hasSentCommand = true
            } catch {
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(Self.stateAfterCommandFailure(error), operation: operation)
                RuntimeFileLog.append("nav_command_failed", fields: [
                    "error": error.localizedDescription,
                    "state": state.description
                ])
                return
            }
            do {
                try await Task.sleep(for: .seconds(RoverConfig.commandInterval))
            } catch {
                break
            }
        }
        await finishCancelledOperationIfCurrent(operation)
    }

    private func performScanRotation(from scanStartPose: Pose2D,
                                     requestedScanAngle: Double,
                                     operation: OperationToken) async {
        var scanPulseCount = 0
        var hasSentCommand = false

        while isOperationActive(operation) {
            guard let pose = ar.pose else {
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(
                    .failed("AR tracking pose became unavailable during rotation."),
                    operation: operation
                )
                return
            }
            let measurement = ar.relativeHeadingMeasurement()
            guard measurement.reliability == .reliable else {
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(
                    .failed(Self.relativeHeadingFailureMessage(measurement.reliability)),
                    operation: operation
                )
                RuntimeFileLog.append("nav_scan_heading_unreliable", fields: [
                    "reason": Self.relativeHeadingReliabilityDescription(measurement.reliability),
                    "sample_age": measurement.sampleAge.map(Self.formatSeconds) ?? "none"
                ])
                return
            }

            if Self.scanTurnReachedRelativeTarget(
                requestedAngle: requestedScanAngle,
                accumulatedAngle: measurement.accumulatedAngle,
                tolerance: RoverConfig.scanTurnYawTolerance
            ) {
                guard await stopTransportForActiveOperation(operation) else { return }
                let directedTurn = requestedScanAngle >= 0
                    ? measurement.accumulatedAngle
                    : -measurement.accumulatedAngle
                RuntimeFileLog.append("nav_scan_completed", fields: [
                    "requested_deg": Self.formatDegrees(requestedScanAngle),
                    "accumulated_deg": Self.formatDegrees(measurement.accumulatedAngle),
                    "overshoot_deg": Self.formatDegrees(max(0, directedTurn - abs(requestedScanAngle))),
                    "pulses": "\(scanPulseCount)"
                ])
                setStateIfCurrent(.arrived, operation: operation)
                return
            }

            if Self.scanTurnMovedOppositeDirection(
                requestedAngle: requestedScanAngle,
                accumulatedAngle: measurement.accumulatedAngle
            ) {
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(
                    .failed("Scan turn moved opposite the requested direction."),
                    operation: operation
                )
                RuntimeFileLog.append("nav_scan_opposite_direction", fields: [
                    "requested_deg": Self.formatDegrees(requestedScanAngle),
                    "accumulated_deg": Self.formatDegrees(measurement.accumulatedAngle)
                ])
                return
            }

            if Self.scanPulseLimitReached(scanPulseCount) {
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(
                    .failed("Scan turn could not establish a reliable heading."),
                    operation: operation
                )
                RuntimeFileLog.append("nav_scan_pulse_limit", fields: [
                    "pulses": "\(scanPulseCount)",
                    "requested_deg": Self.formatDegrees(requestedScanAngle),
                    "accumulated_deg": Self.formatDegrees(measurement.accumulatedAngle),
                    "pose_yaw_deg": Self.formatDegrees(pose.yaw)
                ])
                return
            }

            let directedTurn = requestedScanAngle >= 0
                ? measurement.accumulatedAngle
                : -measurement.accumulatedAngle
            let remaining = max(0, abs(requestedScanAngle) - directedTurn)
            let error = requestedScanAngle >= 0 ? remaining : -remaining
            let plannedCommand = RotationCommand.command(forYawError: error)
            guard let command = await rotationSafetyCommand(
                plannedCommand,
                hasSentCommand: hasSentCommand,
                operation: operation
            ) else { return }
            guard isOperationActive(operation) else { return }
            let pulseDuration = Self.scanPulseDuration()
            RuntimeFileLog.append("nav_rotate_tick", fields: [
                "pose_x": Self.formatMeters(pose.position.x),
                "pose_y": Self.formatMeters(pose.position.y),
                "pose_yaw_deg": Self.formatDegrees(pose.yaw),
                "requested_deg": Self.formatDegrees(requestedScanAngle),
                "accumulated_deg": Self.formatDegrees(measurement.accumulatedAngle),
                "sample_age": measurement.sampleAge.map(Self.formatSeconds) ?? "none",
                "pulse": "\(scanPulseCount + 1)",
                "pulse_seconds": Self.formatSeconds(pulseDuration),
                "mode": "scan_pulse",
                "heading_source": "gravity_axis_rotation_rate",
                "wheel_left": Self.formatMeters(command.left),
                "wheel_right": Self.formatMeters(command.right)
            ])

            do {
                try await control.sendNavigation(command)
                guard isOperationActive(operation) else { return }
                hasSentCommand = true
            } catch {
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(Self.stateAfterCommandFailure(error), operation: operation)
                RuntimeFileLog.append("nav_command_failed", fields: [
                    "error": error.localizedDescription,
                    "state": state.description
                ])
                return
            }

            scanPulseCount += 1
            let frameBeforePulse = ar.frameSequence
            do {
                try await Task.sleep(for: .seconds(pulseDuration))
            } catch is CancellationError {
                return
            } catch {
                return
            }
            guard await stopTransportForActiveOperation(operation) else { return }
            RuntimeFileLog.append("nav_scan_turn_settle", fields: [
                "pulse_seconds": Self.formatSeconds(pulseDuration),
                "settle_seconds": Self.formatSeconds(RoverConfig.scanTurnSettleDuration)
            ])
            do {
                try await Task.sleep(for: .seconds(RoverConfig.scanTurnSettleDuration))
            } catch is CancellationError {
                return
            } catch {
                return
            }
            guard isOperationActive(operation) else { return }

            guard let freshPose = await waitForFreshScanPose(
                afterFrame: frameBeforePulse,
                operation: operation
            ), isOperationActive(operation) else {
                if Task.isCancelled { return }
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(
                    .failed("ARKit did not provide a fresh tracked frame after scan turn."),
                    operation: operation
                )
                RuntimeFileLog.append("nav_scan_frame_unavailable", fields: [
                    "timeout_seconds": Self.formatSeconds(RoverConfig.scanFrameFreshnessTimeout)
                ])
                return
            }

            let settledMeasurement = ar.relativeHeadingMeasurement()
            guard settledMeasurement.reliability == .reliable else {
                guard await stopTransportForActiveOperation(operation) else { return }
                setStateIfCurrent(
                    .failed(Self.relativeHeadingFailureMessage(settledMeasurement.reliability)),
                    operation: operation
                )
                RuntimeFileLog.append("nav_scan_heading_unreliable", fields: [
                    "reason": Self.relativeHeadingReliabilityDescription(settledMeasurement.reliability),
                    "sample_age": settledMeasurement.sampleAge.map(Self.formatSeconds) ?? "none"
                ])
                return
            }

            let arRotation = normalizeAngle(freshPose.yaw - scanStartPose.yaw)
            let disagreement = abs(normalizeAngle(
                arRotation - settledMeasurement.accumulatedAngle
            ))
            RuntimeFileLog.append("nav_scan_frame_ready", fields: [
                "frame": "\(ar.frameSequence)",
                "pose_x": Self.formatMeters(freshPose.position.x),
                "pose_y": Self.formatMeters(freshPose.position.y),
                "pose_yaw_deg": Self.formatDegrees(freshPose.yaw),
                "ar_rotation_deg": Self.formatDegrees(arRotation),
                "accumulated_deg": Self.formatDegrees(settledMeasurement.accumulatedAngle),
                "disagreement_deg": Self.formatDegrees(disagreement)
            ])

            guard Self.isSettledScanPoseConsistent(
                from: scanStartPose,
                to: freshPose,
                accumulatedAngle: settledMeasurement.accumulatedAngle
            ) else {
                guard await stopTransportForActiveOperation(operation) else { return }
                let translation = scanStartPose.position.distance(to: freshPose.position)
                if translation >= RoverConfig.scanPoseJumpTranslation {
                    setStateIfCurrent(
                        .failed("Rover moved unexpectedly during an in-place scan."),
                        operation: operation
                    )
                    RuntimeFileLog.append("nav_scan_translation_jump", fields: [
                        "meters": Self.formatMeters(translation)
                    ])
                } else {
                    setStateIfCurrent(
                        .failed("AR and inertial scan headings disagreed after settling."),
                        operation: operation
                    )
                    RuntimeFileLog.append("nav_scan_heading_disagreement", fields: [
                        "ar_rotation_deg": Self.formatDegrees(arRotation),
                        "accumulated_deg": Self.formatDegrees(settledMeasurement.accumulatedAngle),
                        "disagreement_deg": Self.formatDegrees(disagreement)
                    ])
                }
                return
            }
        }
        _ = await stopTransportForActiveOperation(operation)
    }

    private func rotationSafetyCommand(_ command: WheelCommand,
                                       hasSentCommand: Bool,
                                       operation: OperationToken) async -> WheelCommand? {
        guard isOperationActive(operation) else { return nil }
        guard await rotationGeneralSafetyAllowsMotion(
            command,
            requireFreshAck: hasSentCommand,
            operation: operation
        ) else { return nil }
        guard isOperationActive(operation) else { return nil }

        let depthSafety = depthSafetyObservation(command)
        switch guardLayer.evaluate(command: command, depthSafety: depthSafety) {
        case .allow(let safeCommand, _):
            return safeCommand
        case .stopDepth(let observation):
            var stoppingObservation = observation
            if Self.shouldWaitForFreshRotationDepth(observation) {
                guard await stopTransportForActiveOperation(operation) else { return nil }
                var depthVersion = ar.depthSnapshotVersion
                let deadline = Date().addingTimeInterval(scanDepthRecoveryTimeout)
                RuntimeFileLog.append("nav_scan_depth_retry_started", fields: [
                    "depth_state": observation.state.telemetryReason,
                    "depth_version": String(depthVersion),
                    "timeout_seconds": Self.formatSeconds(scanDepthRecoveryTimeout),
                ])
                while true {
                    let depthRefreshed = await waitForNewDepthSnapshot(
                        after: depthVersion,
                        deadline: deadline,
                        operation: operation
                    )
                    guard isOperationActive(operation) else { return nil }
                    guard depthRefreshed else {
                        guard await stopTransportForActiveOperation(operation) else { return nil }
                        let failureMessage = stoppingObservation.state == .unavailable(.blindSweptVolume)
                            ? "I can’t safely see the space needed to turn. Reposition the rover or camera and try again."
                            : "Depth safety stop while rotating: \(stoppingObservation.state.telemetryReason)."
                        setStateIfCurrent(.failed(failureMessage), operation: operation)
                        RuntimeFileLog.append("nav_scan_depth_retry_timeout", fields: [
                            "depth_state": stoppingObservation.state.telemetryReason,
                            "depth_age": Self.formatSeconds(stoppingObservation.sampleAge),
                            "depth_version": String(depthVersion),
                        ])
                        RuntimeFileLog.append("nav_safety_stop", fields: [
                            "reason": "rotation_depth_timeout",
                            "depth_state": stoppingObservation.state.telemetryReason,
                        ])
                        return nil
                    }

                    let previousDepthVersion = depthVersion
                    depthVersion = ar.depthSnapshotVersion
                    let retriedSafety = depthSafetyObservation(command)
                    switch guardLayer.evaluate(command: command, depthSafety: retriedSafety) {
                    case .allow(let safeCommand, let safeObservation):
                        RuntimeFileLog.append("nav_scan_depth_fresh_snapshot", fields: [
                            "previous_version": String(previousDepthVersion),
                            "depth_version": String(depthVersion),
                            "depth_timestamp": ar.depthSnapshotTimestamp.map(Self.formatSeconds) ?? "none",
                        ])
                        guard await rotationGeneralSafetyAllowsMotion(
                            safeCommand,
                            requireFreshAck: true,
                            operation: operation
                        ) else { return nil }
                        guard isOperationActive(operation) else { return nil }
                        RuntimeFileLog.append("nav_scan_depth_original_authorized", fields: [
                            "depth_state": safeObservation.state.telemetryReason,
                            "support_count": String(safeObservation.supportCount),
                        ])
                        return safeCommand
                    case .stopDepth(let retriedObservation):
                        stoppingObservation = retriedObservation
                    }

                    guard stoppingObservation.state == .unavailable(.staleRawDepth) else {
                        break
                    }
                    RuntimeFileLog.append("nav_scan_depth_retry_candidate_rejected", fields: [
                        "depth_state": stoppingObservation.state.telemetryReason,
                        "depth_age": Self.formatSeconds(stoppingObservation.sampleAge),
                        "previous_version": String(previousDepthVersion),
                        "depth_version": String(depthVersion),
                    ])
                }
                if stoppingObservation.state == .unavailable(.blindSweptVolume) {
                    let arcCommand = RotationCommand.depthVisibleArc(matching: command)
                    let arcSafety = depthSafetyObservation(arcCommand)
                    switch guardLayer.evaluate(
                        command: arcCommand,
                        depthSafety: arcSafety
                    ) {
                    case .allow(let safeArcCommand, let safeObservation):
                        guard await rotationGeneralSafetyAllowsMotion(
                            safeArcCommand,
                            requireFreshAck: true,
                            operation: operation
                        ) else { return nil }
                        guard isOperationActive(operation) else { return nil }
                        RuntimeFileLog.append("nav_scan_depth_visible_arc", fields: [
                            "original_depth_state": observation.state.telemetryReason,
                            "arc_depth_state": safeObservation.state.telemetryReason,
                            "arc_support_count": "\(safeObservation.supportCount)",
                            "wheel_left": Self.formatMeters(safeArcCommand.left),
                            "wheel_right": Self.formatMeters(safeArcCommand.right)
                        ])
                        return safeArcCommand
                    case .stopDepth(let arcObservation):
                        stoppingObservation = arcObservation
                        RuntimeFileLog.append("nav_scan_depth_visible_arc_rejected", fields: [
                            "original_depth_state": observation.state.telemetryReason,
                            "arc_depth_state": arcObservation.state.telemetryReason,
                            "arc_support_count": "\(arcObservation.supportCount)"
                        ])
                    }
                }
            }
            guard await stopTransportForActiveOperation(operation) else { return nil }
            if stoppingObservation.state == .unavailable(.blindSweptVolume) {
                setStateIfCurrent(
                    .failed("I can’t safely see the space needed to turn. Reposition the rover or camera and try again."),
                    operation: operation
                )
            } else {
                setStateIfCurrent(
                    .failed("Depth safety stop while rotating: \(stoppingObservation.state.telemetryReason)."),
                    operation: operation
                )
            }
            RuntimeFileLog.append("nav_safety_stop", fields: [
                "reason": "depth_safety_while_rotating",
                "depth_state": stoppingObservation.state.telemetryReason,
                "depth_age": Self.formatSeconds(stoppingObservation.sampleAge),
                "support_count": "\(stoppingObservation.supportCount)",
                "motion_class": stoppingObservation.motionClass.rawValue
            ])
            return nil
        }
    }

    private static func shouldWaitForFreshRotationDepth(
        _ observation: DepthSafetyObservation
    ) -> Bool {
        guard observation.motionClass == .rotating else { return false }
        switch observation.state {
        case .unavailable(.blindSweptVolume), .unavailable(.staleRawDepth):
            return true
        default:
            return false
        }
    }

    private func waitForNewDepthSnapshot(
        after depthVersion: UInt64,
        deadline: Date? = nil,
        operation: OperationToken
    ) async -> Bool {
        let deadline = deadline ?? Date().addingTimeInterval(scanDepthRecoveryTimeout)
        while ar.depthSnapshotVersion <= depthVersion,
              Date() < deadline,
              isOperationActive(operation) {
            do {
                try await Task.sleep(for: .seconds(RoverConfig.scanDepthRecoveryPollInterval))
            } catch {
                return false
            }
        }
        return isOperationActive(operation) && ar.depthSnapshotVersion > depthVersion
    }

    private func rotationGeneralSafetyAllowsMotion(
        _ command: WheelCommand,
        requireFreshAck: Bool,
        operation: OperationToken
    ) async -> Bool {
        guard isOperationActive(operation) else { return false }
        let lastAck = await control.lastAckAt
        guard isOperationActive(operation) else { return false }
        let now = Date()
        let motionClass = DepthSafetyMotionClass.classify(command)
        let decision = guardLayer.evaluate(
            forwardClearance: .infinity,
            lastAckAt: lastAck,
            now: now,
            feedback: safetyFeedback(),
            requireFreshAck: requireFreshAck,
            checkForwardObstacle: motionClass != .rotating
        )
        switch decision {
        case .go:
            return true
        case .stopObstacle(let clearance):
            guard await stopTransportForActiveOperation(operation) else { return false }
            setStateIfCurrent(.failed(Self.obstacleMessage(clearance: clearance)), operation: operation)
            RuntimeFileLog.append("nav_safety_stop", fields: [
                "reason": "obstacle_while_rotating",
                "clearance": String(format: "%.2f", clearance)
            ])
        case .stopCommsLost:
            guard await stopTransportForActiveOperation(operation) else { return false }
            setStateIfCurrent(.failed("Rover command link lost."), operation: operation)
            RuntimeFileLog.append("nav_safety_stop", fields: [
                "reason": "comms_lost_while_rotating",
                "ack_age": Self.ackAgeField(lastAckAt: lastAck, now: now)
            ])
        case .stopTipping:
            guard await stopTransportForActiveOperation(operation) else { return false }
            setStateIfCurrent(.failed("Rover may be tipping."), operation: operation)
            RuntimeFileLog.append("nav_safety_stop", fields: ["reason": "tipping_while_rotating"])
        }
        return false
    }

    private func waitForReliableRelativeHeading(
        operation: OperationToken
    ) async -> RelativeHeadingMeasurement? {
        let deadline = Date().addingTimeInterval(RoverConfig.scanFrameFreshnessTimeout)
        while isOperationActive(operation), Date() < deadline {
            let measurement = ar.relativeHeadingMeasurement()
            if measurement.reliability == .reliable {
                return measurement
            }
            if case .unreliable(let reason) = measurement.reliability,
               reason != .notStarted,
               reason != .staleSample {
                return nil
            }
            do {
                try await Task.sleep(for: .seconds(RoverConfig.scanFramePollInterval))
            } catch {
                return nil
            }
        }
        return nil
    }

    private func waitForFreshScanPose(
        afterFrame baseline: UInt64,
        operation: OperationToken
    ) async -> Pose2D? {
        let deadline = Date().addingTimeInterval(RoverConfig.scanFrameFreshnessTimeout)
        while isOperationActive(operation), Date() < deadline {
            let currentFrame = ar.frameSequence
            let pose = ar.pose
            if Self.isScanFrameReady(
                baselineFrame: baseline,
                currentFrame: currentFrame,
                isTrackingNormal: ar.isTrackingNormal,
                hasPose: pose != nil
            ), let pose {
                return pose
            }
            do {
                try await Task.sleep(for: .seconds(RoverConfig.scanFramePollInterval))
            } catch {
                return nil
            }
        }
        return nil
    }

    static func isScanFrameReady(baselineFrame: UInt64,
                                 currentFrame: UInt64,
                                 isTrackingNormal: Bool,
                                 hasPose: Bool) -> Bool {
        currentFrame > baselineFrame && isTrackingNormal && hasPose
    }

    static func scanTurnReachedRelativeTarget(requestedAngle: Double,
                                              accumulatedAngle: Double,
                                              tolerance: Double) -> Bool {
        let directedTurn = requestedAngle >= 0 ? accumulatedAngle : -accumulatedAngle
        return directedTurn >= max(0, abs(requestedAngle) - tolerance)
    }

    static func scanTurnMovedOppositeDirection(requestedAngle: Double,
                                               accumulatedAngle: Double) -> Bool {
        let directedTurn = requestedAngle >= 0 ? accumulatedAngle : -accumulatedAngle
        return directedTurn <= -RoverConfig.scanMinimumProgressYaw
    }

    static func isSettledScanPoseConsistent(from start: Pose2D,
                                            to current: Pose2D,
                                            accumulatedAngle: Double) -> Bool {
        guard start.position.distance(to: current.position) < RoverConfig.scanPoseJumpTranslation else {
            return false
        }
        let arRotation = normalizeAngle(current.yaw - start.yaw)
        return abs(normalizeAngle(arRotation - accumulatedAngle))
            <= RoverConfig.scanHeadingAgreementTolerance
    }

    static func scanPulseLimitReached(_ pulseCount: Int) -> Bool {
        pulseCount >= RoverConfig.scanTurnMaxPulseCount
    }

    static func scanPulseDuration() -> TimeInterval {
        RoverConfig.scanTurnPulseDuration
    }

    static func stateAfterObstacleStop(pose _: Pose2D, goal _: Vec2, clearance: Double) -> State {
        .failed(obstacleMessage(clearance: clearance))
    }

    static func depthVisibleForwardCommand(_ command: WheelCommand) -> WheelCommand {
        guard DepthSafetyMotionClass.classify(command) == .curved else { return command }
        let outerSpeed = max(command.left, command.right)
        guard outerSpeed > 0 else { return command }
        let minimumInnerSpeed = outerSpeed * 0.75
        if command.left < command.right {
            return WheelCommand(left: max(command.left, minimumInnerSpeed), right: command.right)
        }
        return WheelCommand(left: command.left, right: max(command.right, minimumInnerSpeed))
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

    private static func relativeHeadingReliabilityDescription(
        _ reliability: RelativeHeadingReliability
    ) -> String {
        switch reliability {
        case .reliable:
            "reliable"
        case .unreliable(let reason):
            reason.rawValue
        }
    }

    private static func relativeHeadingFailureMessage(
        _ reliability: RelativeHeadingReliability
    ) -> String {
        "Relative heading became unreliable (\(relativeHeadingReliabilityDescription(reliability)))."
    }

    private static func formatSeconds(_ value: TimeInterval) -> String {
        String(format: "%.3f", value)
    }

    private static func formatMeters(_ value: Double) -> String {
        String(format: "%.2f", value)
    }

    private static func formatDegrees(_ radians: Double) -> String {
        String(format: "%.0f", radians * 180 / .pi)
    }
}

extension NavigationController.State {
    var isMotionActive: Bool {
        switch self {
        case .planning, .driving:
            true
        case .idle, .arrived, .failed:
            false
        }
    }
}

extension NavigationController.State: CustomStringConvertible {
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
