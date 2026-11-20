import Foundation
import RoverNav

struct FollowPursuitPlan: Sendable {
    static let joinMargin = 0.02
    let path: [Vec2]
    var map: Costmap? = nil
    var rejection: PathPlanningFailure? = nil
    var diagnostics: [String: String] = [:]
    var planningMode: FollowPlanningMode = .mesh
    var isDirectDepthRoute: Bool { planningMode == .reactiveDepth && map == nil && path.count == 1 && rejection == nil }
    var endpointTolerance: Double { max(map?.resolution ?? 0, 1e-6) }

    func joined(from position: Vec2) -> Self? {
        guard let map else { return self }
        guard let closest = path.indices.min(by: { path[$0].distance(to: position) < path[$1].distance(to: position) }) else { return nil }
        // Join an already planned route using its traversability policy. Soft
        // inflation is a preference cost, not a new obstacle under the rover.
        for index in closest..<path.count where map.isSegmentClear(from: position, to: path[index],
            margin: Self.joinMargin, blockingCost: Costmap.lethalThreshold) {
            return .init(path: Array(path[index...]), map: map)
        }
        return nil
    }

    /// Replace only the final leg using the same captured collision map. This
    /// creates a plan for the latest goal; it does not authorize the old goal.
    func retargeted(to goal: Vec2, from capturedStart: Vec2) -> Self? {
        if isDirectDepthRoute {
            guard goal.x.isFinite, goal.y.isFinite else { return nil }
            return .init(path: [goal], planningMode: .reactiveDepth)
        }
        guard rejection == nil, let map, !path.isEmpty,
              goal.x.isFinite, goal.y.isFinite,
              path.allSatisfy({ $0.x.isFinite && $0.y.isFinite }) else { return nil }
        let prefix = path.dropLast()
        let anchor = prefix.last ?? capturedStart
        guard map.isSegmentClear(from: anchor, to: goal, margin: Self.joinMargin) else { return nil }
        return .init(path: Array(prefix) + [goal], map: map)
    }
}

struct FollowPursuitSendFailure: Error {
    let reason: NavigationFailure
    var definitelyUnsent = false
    var authorizationExpired = false
    var targetAuthorityChanged = false
}

/// One follow owner, one worker, and a newest-only goal mailbox. Planning never
/// suspends the moving control loop. A result is committed only for its exact
/// goal revision and source generation, before its original deadline.
@MainActor
final class FollowPursuitDriver {
    struct Environment {
        let plan: @MainActor (Vec2, Vec2) async -> FollowPursuitPlan?
        let pose: @MainActor () -> NavigationPoseSample
        let owned: @MainActor () -> Bool
        let uptime: @MainActor () -> Double
        let date: @MainActor () -> Date
        let acknowledgement: @MainActor () async -> Date?
        let clearance: @MainActor () -> Double
        let depth: @MainActor (WheelCommand) -> DepthSafetyObservation
        /// Must check fresh depth for the exact (possibly speed-limited) command
        /// at transport admission, within the supplied original expiry.
        let send: @MainActor (WheelCommand, Double) async throws -> Void
        let stop: @MainActor () async throws -> Void
        let sleep: @MainActor (Duration) async -> Void
        let publishPath: @MainActor ([Vec2]) -> Void
        let moving: @MainActor () -> Void
        let emit: @MainActor (String, [String: FollowDiagnosticValue]) -> Void
        var admissionDuration: @MainActor () -> Double? = { nil }
        var depthNotificationsEnabled = false
        var acknowledgementSnapshot: (@MainActor () -> Date?)? = nil
        var targetSample: @MainActor () -> FollowTargetSpeedSample? = { nil }
    }

    private final class Job {
        let revision: UInt64
        let goal: Vec2
        let source: NavigationPoseSample
        let started: Double
        var completed = false
        var result: FollowPursuitPlan?
        var task: Task<Void, Never>?
        init(revision: UInt64, goal: Vec2, source: NavigationPoseSample, started: Double) {
            self.revision = revision; self.goal = goal; self.source = source; self.started = started
        }

        func stillBlocksStart(at current: NavigationPoseSample) -> Bool {
            guard result?.rejection == .startBlocked, let map = result?.map,
                  let generation = source.frameID?.generation,
                  current.frameID?.generation == generation,
                  let capturedPose = source.pose, let currentPose = current.pose else { return false }
            let capturedCell = map.worldToCell(capturedPose.position)
            let currentCell = map.worldToCell(currentPose.position)
            return capturedCell.cx == currentCell.cx && capturedCell.cy == currentCell.cy &&
                map.isBlocked(currentCell.cx, currentCell.cy)
        }

        func retargeted(to goal: Vec2, revision: UInt64, current: NavigationPoseSample) -> Job? {
            guard let generation = source.frameID?.generation,
                  current.frameID?.generation == generation, let capturedPose = source.pose,
                  let result, result.rejection == nil, let endpoint = result.path.last,
                  endpoint.distance(to: self.goal) <= result.endpointTolerance,
                  let updated = result.retargeted(to: goal, from: capturedPose.position) else { return nil }
            let job = Job(revision: revision, goal: goal, source: source, started: started)
            job.result = updated
            job.completed = true
            return job
        }
    }

    private let environment: Environment
    private let pursuit = PursuitController(params: .init(wheelBase: RoverConfig.wheelBase,
        goalTolerance: 0.2, minimumRotateWheelSpeed: RoverConfig.minimumRotateWheelSpeed))
    private var guardLayer: ObstacleGuard {
        .init(stopDistance: FollowPlanningScope.minimumObstacleClearance ?? 0.45)
    }
    private var job: Job?
    private var active = true
    private(set) var perceptionPaused = false
    private var perceptionPauseBoundary: Double?
    private var requiresCurrentRouteAfterPause = false
    private var resumedAfterPerceptionPause = false
    private var perceptionPauseWaiters: [CheckedContinuation<Bool, Never>] = []
    private(set) var stationaryStopConfirmed = false

    func pauseForPerception() async -> Bool {
        guard owned else { return false }
        if !perceptionPaused { perceptionPauseBoundary = nil }
        perceptionPaused = true
        wakeDepthWaiter()
        if perceptionPauseBoundary != nil { return true }
        return await withCheckedContinuation { perceptionPauseWaiters.append($0) }
    }

    func canResumePerception(timestamp: Double) -> Bool {
        guard owned, perceptionPaused, let boundary = perceptionPauseBoundary,
              timestamp.isFinite, timestamp > boundary, timestamp <= environment.uptime(),
              environment.uptime() - timestamp <= 0.5 else { return false }
        return true
    }

    func resumePerception(timestamp: Double) -> Bool {
        guard canResumePerception(timestamp: timestamp) else { return false }
        perceptionPaused = false
        perceptionPauseBoundary = nil
        requiresCurrentRouteAfterPause = true
        resumedAfterPerceptionPause = true
        wakeDepthWaiter()
        return true
    }

    private func completePerceptionPause(_ confirmed: Bool) {
        let waiters = perceptionPauseWaiters
        perceptionPauseWaiters.removeAll()
        for waiter in waiters { waiter.resume(returning: confirmed) }
    }
    private var depthRevision: UInt64 = 0
    private var depthPublishedAt: Double?
    private var depthWaiter: (id: UUID, continuation: CheckedContinuation<Void, Never>)?

    func depthPublished() {
        depthRevision &+= 1
        depthPublishedAt = environment.uptime()
        wakeDepthWaiter()
    }

    private func wakeDepthWaiter(id: UUID? = nil) {
        guard let waiter = depthWaiter, id == nil || waiter.id == id else { return }
        depthWaiter = nil
        waiter.continuation.resume()
    }

    private func waitForDepth(after revision: UInt64, seconds: Double) async {
        guard owned, seconds > 0 else { return }
        guard environment.depthNotificationsEnabled else {
            await environment.sleep(.seconds(seconds)); return
        }
        guard depthRevision == revision else { return }
        let id = UUID()
        var timer: Task<Void, Never>?
        defer { timer?.cancel() }
        await withTaskCancellationHandler {
            await withCheckedContinuation { continuation in
                guard owned, depthRevision == revision else { continuation.resume(); return }
                depthWaiter = (id, continuation)
                timer = Task { [weak self, environment] in
                    await environment.sleep(.seconds(seconds))
                    guard !Task.isCancelled else { return }
                    self?.wakeDepthWaiter(id: id)
                }
            }
        } onCancel: {
            Task { @MainActor [weak self] in self?.wakeDepthWaiter(id: id) }
        }
    }
    private var lastPlanStarted = -Double.infinity
    private(set) var revision: UInt64 = 0
    private(set) var goal: Vec2
    private(set) var acceptingGoals = true
    static let planningTimeout = 1.0
    static let initialAdmissionTimeout = 2.0
    static let depthRefreshLimit = 0.6
    static let maximumUnmappedPoseShift = 0.02
    // Cold-start reserve only; transport still enforces exact expiry. Observed
    // evaluation/send durations raise this budget instead of assuming 100 ms.
    static let minimumFreshnessMargin = 0.02

    /// Shared with native admission: early warnings authorize only bounded,
    /// slow forward arcs, and only after exact-command depth is clear.
    nonisolated static func isSlowObstacleArc(_ command: WheelCommand) -> Bool {
        let peak = max(command.left, command.right)
        let inner = min(command.left, command.right)
        return peak.isFinite && inner.isFinite && peak > 0 && peak <= 0.12 &&
            inner / peak >= 0.15 - 1e-9 && inner / peak <= 0.65 + 1e-9
    }
    private(set) var failureCause: FollowMotionFailureCause?

    init(goal: Vec2, environment: Environment) { self.goal = goal; self.environment = environment }

    func updateGoal(_ goal: Vec2) -> Bool {
        guard active, acceptingGoals, goal.x.isFinite, goal.y.isFinite else { return false }
        if self.goal != goal { self.goal = goal; revision &+= 1 }
        return true
    }

    func inhibit() {
        active = false; acceptingGoals = false
        completePerceptionPause(false)
        wakeDepthWaiter()
        job?.task?.cancel(); job = nil
    }

    private var owned: Bool { active && !Task.isCancelled && environment.owned() }

    private func beginPlan(from source: NavigationPoseSample, reason: String) {
        guard job == nil, let pose = source.pose else { return }
        let next = Job(revision: revision, goal: goal, source: source, started: environment.uptime())
        job = next
        lastPlanStarted = next.started
        environment.emit("planning_started", ["goal_x": .number(goal.x), "goal_z": .number(goal.y),
            "revision": .number(Double(revision)), "reason": .string(reason)])
        let plan = environment.plan
        next.task = Task { [weak self] in
            let result = await plan(pose.position, next.goal)
            guard let self, self.owned, self.job === next else { return }
            next.result = result
            next.completed = true
        }
    }

    func run() async -> NavigationResult {
        defer { inhibit() }
        let began = environment.uptime()
        let initial = environment.pose()
        guard initial.pose != nil else { return .failed(.noPose) }
        guard goal.x.isFinite, goal.y.isFinite, began.isFinite,
              initial.rejection(at: began) == nil else { return .failed(.trackingLost) }
        let generation = initial.frameID?.generation
        var route: FollowPursuitPlan?
        var routeRevision: UInt64?
        var routeSource: NavigationPoseSample?
        var routePlanStartedAt: Double?
        var routeGoal = goal
        var lastPlan = began
        var sent = false
        var movementActive = false
        var requiresTargetAuthority = FollowTargetSpeedScope.feed != nil
        var targetRefresh: (after: Double, deadline: Double)?
        var targetRefreshes = 0
        var acceleration = FollowPursuitAcceleration()
        var refreshDeadline: Double?
        var refreshReason = DepthSafetyUnavailableReason.staleRawDepth
        var expiryRecoveries = 0
        var refreshAfter: (frame: ARFrameID?, time: Double)?
        var progress = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.05)
        var turnProgress = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.05)
        var previousTurnYaw: Double?
        var previousTurnDirection = 0.0
        var directedTurnTravel = 0.0
        var progressStoppedAt: Date?
        var progressStoppedDuration = 0.0
        func markProgressStopped() { progressStoppedAt = progressStoppedAt ?? environment.date() }
        func progressTime(_ date: Date) -> Date {
            date.addingTimeInterval(-progressStoppedDuration - (progressStoppedAt.map { max(0, date.timeIntervalSince($0)) } ?? 0))
        }
        func resumeProgress(at date: Date) {
            if let stopped = progressStoppedAt { progressStoppedDuration += max(0, date.timeIntervalSince(stopped)) }
            progressStoppedAt = nil
        }
        var previousPosition: Vec2?
        var previousGoal: Vec2?
        var approach = 0.0
        var lastDiagnostic = -Double.infinity
        var maximumAdmissionDuration = 0.0
        var healthySince: Double?
        var lastHealthySend: Double?
        var lastHealthyCapture: Double?
        var healthySends = 0
        var refreshCaptureCount = 0
        var refreshFirstCapture: Double?
        var refreshLastCapture: Double?
        var stoppedAcknowledgement: Date?
        var avoidanceSide: DepthObstacleSide?
        var avoidanceUntil = -Double.infinity
        var passagePreference: DepthPassageGuidance?
        var passageSlowUntil = -Double.infinity
        var steeringResponse = FollowSteeringResponse()
        var alignmentRequested: Double?
        var correctiveTurn: (direction: Double, yaw: Double, deadline: Double)?
        var alignmentApproach: (direction: Double, origin: Vec2, deadline: Double, command: WheelCommand)?
        var approachCompleted = false
        var alignmentAttempts = 0
        var creepResponse = FollowCreepResponse()
        var creepHoldRequested = false

        func recordAcceptance(_ reason: String, job: Job?, source: NavigationPoseSample, at time: Double,
                              event: String = "plan_acceptance_rejected", includeGeometry: Bool = true) {
            let capturedPlan: FollowPursuitPlan?
            if let job { capturedPlan = job.result } else { capturedPlan = route }
            let capture = FollowPursuitPlanDiagnostics(plan: capturedPlan,
                capturedSource: job?.source ?? routeSource, currentSource: source,
                capturedGoal: job?.goal ?? (route == nil ? nil : routeGoal), latestGoal: goal,
                expectedGeneration: generation, capturedRevision: job?.revision ?? routeRevision,
                latestRevision: revision, planStartedAt: job?.started ?? routePlanStartedAt,
                evaluatedAt: time, startupDeadline: began + Self.initialAdmissionTimeout, sent: sent)
            var fields = capture.fields(reason: reason, includeGeometry: includeGeometry)
            // Keep the existing revision field for planning_superseded consumers.
            fields["revision"] = job.map { .number(Double($0.revision)) } ?? .null
            environment.emit(event, fields)
        }

        func resetHealthyDelivery() {
            healthySince = nil; lastHealthySend = nil; lastHealthyCapture = nil; healthySends = 0
        }

        func recoverExpiredCommand(expiry: Double, rawDepth: Bool = false) async -> NavigationFailure? {
            resetHealthyDelivery()
            refreshCaptureCount = 0; refreshFirstCapture = nil; refreshLastCapture = nil
            guard expiryRecoveries < 2 else {
                failureCause = .pursuitRecoveryExhausted
                return .depthUnavailable(.staleRawDepth)
            }
            failureCause = rawDepth ? .pursuitDepthRefreshTimeout : .pursuitAuthorizationExpired
            let stopRequestedAt = environment.uptime()
            let existingRecoveryDeadline = refreshAfter == nil ? nil : refreshDeadline
            expiryRecoveries += 1
            environment.emit(rawDepth ? "raw_depth_refresh_started" : "authorization_expired", ["recovery_attempt": .number(Double(expiryRecoveries)),
                "refresh_deadline_s": existingRecoveryDeadline.map { .number($0) } ?? .null,
                "command_valid_until_s": expiry.isFinite ? .number(expiry) : .null,
                "evaluated_at_s": .number(environment.uptime()), "reason": .string(rawDepth ? "raw_depth_freshness" : "command_authorization_expired"),
                "transport_attempts": .number(0)])
            if !stationaryStopConfirmed {
                do { try await environment.stop() } catch { return .commandFailed }
                stationaryStopConfirmed = true
            }
            markProgressStopped()
            movementActive = false
            acceleration.reset()
            stoppedAcknowledgement = nil
            refreshAfter = (environment.pose().frameID, environment.uptime())
            // Recovery accepts only captures after this stop. Give those captures
            // the full bounded window instead of spending it on stop transport.
            // Startup cannot outlive its original admission deadline, and an
            // unsent retry within this recovery cannot renew the established window.
            refreshDeadline = existingRecoveryDeadline ?? min(sent ? .infinity : began + Self.initialAdmissionTimeout,
                refreshAfter!.time + Self.depthRefreshLimit)
            environment.emit("depth_refresh_stop_confirmed", [
                "confirmed_stop_s": .number(refreshAfter!.time),
                "stop_elapsed_s": .number(refreshAfter!.time - stopRequestedAt),
                "refresh_deadline_s": .number(refreshDeadline!)])
            return nil
        }

        while owned {
            let tick = environment.uptime()
            guard tick.isFinite, tick >= began else { return .failed(.trackingLost) }
            let source = environment.pose()
            let sourceRejection = source.rejection(at: tick, expectedGeneration: generation)
            guard sourceRejection == nil, let pose = source.pose else {
                recordAcceptance("source_" + (sourceRejection ?? "missing_pose"), job: job, source: source, at: tick)
                return .failed(.trackingLost)
            }
            if !sent, tick >= began + Self.initialAdmissionTimeout {
                recordAcceptance("startup_timeout", job: job, source: source, at: tick)
                return .failed(.noPath)
            }
            if let refreshDeadline, tick >= refreshDeadline { return .failed(.depthUnavailable(refreshReason)) }
            if perceptionPaused {
                if !stationaryStopConfirmed {
                    do { try await environment.stop() } catch { return .failed(.commandFailed) }
                    stationaryStopConfirmed = true
                    movementActive = false
                    acceleration.reset()
                    stoppedAcknowledgement = nil
                }
                if perceptionPauseBoundary == nil {
                    markProgressStopped()
                    perceptionPauseBoundary = environment.uptime()
                    completePerceptionPause(true)
                    environment.emit("perception_pause_confirmed", [:])
                }
                await waitForDepth(after: depthRevision, seconds: 0.02)
                continue
            }
            if resumedAfterPerceptionPause {
                resumedAfterPerceptionPause = false
                if sent, let old = job, tick >= old.started + Self.planningTimeout {
                    // A completed candidate can age out while deliberately
                    // stationary. Discard it once on resume, rather than accept
                    // stale geometry or restart the entire pursuit owner.
                    recordAcceptance("expired_plan_after_perception_pause", job: old, source: source, at: tick,
                        event: "planning_superseded", includeGeometry: false)
                    old.task?.cancel()
                    job = nil
                    route = nil
                    routeRevision = nil
                }
            }
            if let boundary = refreshAfter {
                guard let timestamp = source.sourceTimestamp, timestamp > boundary.time,
                      let frame = source.frameID, let previous = boundary.frame,
                      frame.generation == previous.generation, frame.sequence > previous.sequence else {
                    await environment.sleep(.milliseconds(20)); continue
                }
            }
            if let job {
                guard tick >= job.started, tick < job.started + Self.planningTimeout else {
                    recordAcceptance(tick < job.started ? "planning_clock_regressed" : "planning_timeout", job: job, source: source, at: tick)
                    return .failed(.noPath)
                }
                if job.completed {
                    self.job = nil
                    // A moving target invalidates a route or goal rejection, but
                    // cannot clear the unchanged starting cell before any send.
                    if let rejected = job.result?.rejection,
                       job.revision == revision || (!sent && job.stillBlocksStart(at: source)) {
                        switch rejected {
                        case .startBlocked:
                            failureCause = job.result?.diagnostics["start_geometry_conflict"] == "true"
                                ? .pursuitMapGeometryConflict : .pursuitStartBlocked
                        case .goalBlocked: failureCause = .pursuitGoalBlocked
                        case .noConnectedPath: failureCause = .pursuitDisconnectedPath
                        case .startOutsideMap, .goalOutsideMap: failureCause = .pursuitOutsideMap
                        }
                        var fields = (job.result?.diagnostics ?? [:]).mapValues { FollowDiagnosticValue.string($0) }
                        fields["planning_rejection"] = .string(rejected.rawValue)
                        fields["planning_goal_superseded"] = .bool(job.revision != revision)
                        environment.emit("planning_rejected", fields)
                        recordAcceptance("planner_" + rejected.rawValue, job: job, source: source, at: tick)
                        return .failed(.noPath)
                    }
                    var candidate = job
                    if job.revision != revision,
                       let updated = job.retargeted(to: goal, revision: revision, current: source) {
                        recordAcceptance("latest_goal_tail_verified", job: job, source: source, at: tick,
                            event: "planning_retargeted", includeGeometry: false)
                        candidate = updated
                    }
                    let job = candidate
                    if job.revision == revision {
                        guard job.source.frameID?.generation == generation else {
                            recordAcceptance("captured_generation_mismatch", job: job, source: source, at: tick)
                            return .failed(.noPath)
                        }
                        guard let result = job.result else {
                            recordAcceptance("planner_returned_nil", job: job, source: source, at: tick)
                            return .failed(.noPath)
                        }
                        guard !result.path.isEmpty else {
                            recordAcceptance("empty_path", job: job, source: source, at: tick)
                            return .failed(.noPath)
                        }
                        guard result.path.allSatisfy({ $0.x.isFinite && $0.y.isFinite }) else {
                            recordAcceptance("nonfinite_waypoint", job: job, source: source, at: tick)
                            return .failed(.noPath)
                        }
                        guard result.path.last!.distance(to: job.goal) <= result.endpointTolerance else {
                            recordAcceptance("endpoint_mismatch", job: job, source: source, at: tick)
                            return .failed(.noPath)
                        }
                        guard let joined = result.joined(from: pose.position) else {
                            recordAcceptance("path_join_blocked", job: job, source: source, at: tick)
                            return .failed(.noPath)
                        }
                        if result.map == nil, !result.isDirectDepthRoute, let origin = job.source.pose,
                           origin.position.distance(to: pose.position) > Self.maximumUnmappedPoseShift {
                            recordAcceptance("unmapped_pose_shift", job: job, source: source, at: tick)
                            return .failed(.trackingLost)
                        }
                        route = joined; routeRevision = job.revision; routeGoal = job.goal; lastPlan = tick
                        routeSource = job.source; routePlanStartedAt = job.started
                        environment.publishPath(joined.path)
                        environment.emit("planning_completed", ["elapsed_s": .number(tick - job.started),
                            "revision": .number(Double(job.revision))])
                    } else {
                        recordAcceptance("goal_revision_superseded", job: job, source: source, at: tick,
                            event: "planning_superseded", includeGeometry: false)
                    }
                }
            }
            guard owned else { return .cancelled }
            if job == nil, tick - lastPlanStarted >= 0.5,
               refreshAfter == nil || route == nil || routeRevision != revision,
               route == nil || routeRevision != revision || tick - lastPlan >= 1 {
                beginPlan(from: source, reason: route == nil ? "startup" : (routeRevision != revision ? "live_goal" : "periodic"))
            }

            // While confirmed stopped, an async actor round trip on every depth
            // publication can consume its send margin before we even evaluate it.
            // Reuse only a real post-stop read with its original watchdog expiry;
            // never manufacture an acknowledgement or renew its timestamp.
            let ackReadStarted = environment.uptime()
            let cachedAge = stoppedAcknowledgement.map { environment.date().timeIntervalSince($0) }
            let reuseAck = environment.acknowledgementSnapshot == nil && refreshAfter != nil && !movementActive && cachedAge.map {
                $0.isFinite && $0 >= 0 && $0 + max(Self.minimumFreshnessMargin, maximumAdmissionDuration + 0.01) < RoverConfig.commsWatchdogTimeout
            } == true
            let ack: Date?
            if let snapshot = environment.acknowledgementSnapshot { ack = snapshot() }
            else if reuseAck { ack = stoppedAcknowledgement }
            else {
                ack = await environment.acknowledgement()
                stoppedAcknowledgement = refreshAfter != nil && !movementActive ? ack : nil
            }
            let ackReadDuration = environment.uptime() - ackReadStarted
            guard owned else { return .cancelled }
            let current = environment.pose()
            let time = environment.uptime()
            guard time.isFinite, time >= tick else { return .failed(.trackingLost) }
            if !sent, time >= began + Self.initialAdmissionTimeout {
                recordAcceptance("startup_timeout", job: job, source: current, at: time)
                return .failed(.noPath)
            }
            if let job, time >= job.started + Self.planningTimeout {
                recordAcceptance("planning_timeout", job: job, source: current, at: time)
                return .failed(.noPath)
            }
            let currentRejection = current.rejection(at: time, expectedGeneration: generation)
            guard currentRejection == nil, let currentPose = current.pose else {
                recordAcceptance("source_" + (currentRejection ?? "missing_pose"), job: job, source: current, at: time)
                return .failed(.trackingLost)
            }
            guard let acknowledgedAt = ack else { return .failed(.commsLost) }
            let ackAge = environment.date().timeIntervalSince(acknowledgedAt)
            guard ackAge.isFinite, ackAge >= 0, ackAge <= RoverConfig.commsWatchdogTimeout else {
                return .failed(.commsLost)
            }
            switch guardLayer.evaluate(forwardClearance: environment.clearance(), lastAckAt: ack,
                now: environment.date(), feedback: nil, requireFreshAck: true, checkForwardObstacle: true) {
            case .go: break
            case .stopObstacle: return .failed(.obstacle)
            case .stopCommsLost: return .failed(.commsLost)
            case .stopTipping: return .failed(.tipping)
            }
            guard let route, (sent && !requiresCurrentRouteAfterPause) || routeRevision == revision else {
                await environment.sleep(.milliseconds(20)); continue
            }
            if let refreshDeadline, time >= refreshDeadline { return .failed(.depthUnavailable(refreshReason)) }
            guard let joined = route.joined(from: currentPose.position) else {
                recordAcceptance("active_route_join_blocked", job: nil, source: current, at: time)
                return .failed(.noPath)
            }
            let targetSample = environment.targetSample()
            if let sample = targetSample, let approach = sample.lastSeenApproach,
               approach.reachedLimit(position: currentPose.position, observedAt: sample.timestamp, now: time) {
                environment.emit("last_seen_limit_reached", ["observed_at_s": .number(sample.timestamp),
                    "deadline_s": .number(sample.validUntil)])
                return .arrived
            }
            let targetSpeed = FollowTargetSpeed.evaluate(targetSample, pose: current,
                now: time, maximum: pursuit.params.maxLinear)
            if targetSample != nil { requiresTargetAuthority = true }
            if route.planningMode == .reactiveDepth, requiresTargetAuthority {
                if let refresh = targetRefresh, time >= refresh.deadline {
                    failureCause = .pursuitTargetRefreshTimeout
                    environment.emit("target_wait_expired", ["deadline_s": .number(refresh.deadline),
                        "confirmed_stop_s": .number(refresh.after),
                        "target_capture_s": targetSample.map { .number($0.timestamp) } ?? .null])
                    return .failed(.trackingLost)
                }
                let postStopTarget = targetRefresh.map { refresh in
                    targetSample.map { $0.timestamp > refresh.after } ?? false
                } ?? true
                if targetSpeed == nil || !postStopTarget {
                    if targetRefresh == nil {
                        guard targetRefreshes < 2 else {
                            failureCause = .pursuitTargetRecoveryExhausted
                            return .failed(.trackingLost)
                        }
                        targetRefreshes += 1
                        // Never substitute a waypoint when target authority
                        // expires. Stop once; only a new post-stop measurement can
                        // resume this owner, within a non-renewable recovery window.
                        let stopStarted = environment.uptime()
                        if !stationaryStopConfirmed {
                            do { try await environment.stop() } catch { return .failed(.commandFailed) }
                            stationaryStopConfirmed = true
                        }
                        guard owned else { return .cancelled }
                        let stoppedAt = environment.uptime()
                        guard stoppedAt.isFinite, stoppedAt >= stopStarted else { return .failed(.trackingLost) }
                        // Delivery/stop latency cannot consume the interval reserved
                        // for a newly captured, independently verified person.
                        // This deadline is created once, never renewed by misses.
                        let deadline = stoppedAt + 0.5
                        movementActive = false
                        markProgressStopped()
                        acceleration.reset()
                        resetHealthyDelivery()
                        stoppedAcknowledgement = nil
                        targetRefresh = (stoppedAt, deadline)
                        environment.emit("target_wait_started", ["deadline_s": .number(deadline),
                            "stop_started_s": .number(stopStarted), "confirmed_stop_s": .number(stoppedAt),
                            "post_stop_window_s": .number(0.5)])
                    }
                    await environment.sleep(.milliseconds(20))
                    continue
                }
                if targetRefresh != nil {
                    environment.emit("target_wait_resumed", [:])
                    targetRefresh = nil
                }
            }
            let targetExpiry = targetSpeed == nil ? Double.infinity : targetSample!.validUntil
            var parameters = pursuit.params
            if let targetSpeed {
                parameters.maxLinear = targetSpeed.maximumWheelSpeed
                parameters.goalTolerance = 0.05
                parameters.slowdownRadius = parameters.goalTolerance
            }
            // Centre the measured bearing at a fixed lookahead, not at the
            // person's full range: a two-point path otherwise selects its remote
            // endpoint and weakens curvature as the person walks farther away.
            // No angular deadband: even a sub-five-degree offset steers.
            // The fresh target-speed policy still owns stand-off braking.
            let steeringPath: [Vec2]
            var centeringBearing: Double?
            if route.planningMode == .reactiveDepth, targetSpeed != nil, let targetSample {
                // Prioritise heading over translation. The former 0.5 m arc
                // produced almost equal PWM requests under depth caution and
                // little measured yaw response. Bound the sharper arc through
                // the existing angular/wheel caps and exact-command depth gate.
                parameters.lookahead = 0.15
                // Scale the yaw cap with the target-speed ceiling as well, so
                // slowing near stand-off does not change the requested arc.
                parameters.maxAngular *= parameters.maxLinear / pursuit.params.maxLinear
                let delta = targetSample.position - currentPose.position
                let heading = atan2(delta.y, delta.x) - currentPose.yaw
                centeringBearing = atan2(sin(heading), cos(heading))
                let distance = delta.length
                let aim = currentPose.position + delta * (min(parameters.lookahead, distance) / max(distance, 1e-9))
                steeringPath = [currentPose.position, aim]
                parameters.rotateInPlaceAngle = .pi / 2
            } else {
                steeringPath = joined.path
            }
            let correctionEligible = route.planningMode == .reactiveDepth &&
                targetSample?.centersContinuously == true && targetSample?.lastSeenApproach == nil &&
                targetSample?.holdsTranslation != true && (targetSpeed?.maximumWheelSpeed ?? 0) > 0
            let creepEligible = correctionEligible &&
                (targetSpeed?.range ?? .infinity) <= (targetSample?.standOff ?? 0) + 0.25 &&
                (targetSpeed?.maximumWheelSpeed ?? .infinity) <= 0.1
            if creepHoldRequested {
                creepHoldRequested = false
                if creepEligible {
                    do { try await environment.stop() } catch { return .failed(.commandFailed) }
                    stationaryStopConfirmed = true
                    guard owned else { return .cancelled }
                    environment.emit("near_target_settled_hold", [:])
                    return .arrived
                }
            }
            if let approach = alignmentApproach,
               !correctionEligible || environment.uptime() >= approach.deadline ||
                currentPose.position.distance(to: approach.origin) >= 0.15 {
                do { try await environment.stop() } catch { return .failed(.commandFailed) }
                stationaryStopConfirmed = true
                movementActive = false
                markProgressStopped()
                guard owned else { return .cancelled }
                alignmentApproach = nil
                let needsTurn = correctionEligible &&
                    (centeringBearing ?? 0) * approach.direction > Double.pi / 36
                approachCompleted = needsTurn
                alignmentRequested = needsTurn ? approach.direction : nil
                environment.emit("steering_approach_completed", [
                    "travel_m": .number(currentPose.position.distance(to: approach.origin))])
            }
            if let turn = correctiveTurn {
                let travel = atan2(sin(currentPose.yaw - turn.yaw), cos(currentPose.yaw - turn.yaw)) * turn.direction
                if !correctionEligible || environment.uptime() >= turn.deadline || travel >= 0.3 ||
                    (centeringBearing ?? 0) * turn.direction <= Double.pi / 36 {
                    do { try await environment.stop() } catch { return .failed(.commandFailed) }
                    stationaryStopConfirmed = true
                    movementActive = false
                    markProgressStopped()
                    correctiveTurn = nil
                    steeringResponse.reset()
                    previousPosition = nil; previousGoal = nil
                    environment.emit("steering_alignment_completed", ["yaw_travel_rad": .number(travel)])
                    guard owned else { return .cancelled }
                    // A bounded correction with no response must not become
                    // another unbounded sequence of motor requests.
                    if correctionEligible, travel < Double.pi / 60 { return .failed(.stalled) }
                    continue
                }
            } else if let direction = alignmentRequested {
                alignmentRequested = nil
                steeringResponse.reset()
                if correctionEligible {
                    guard alignmentAttempts < 2 else { return .failed(.stalled) }
                    do { try await environment.stop() } catch { return .failed(.commandFailed) }
                    stationaryStopConfirmed = true
                    movementActive = false
                    markProgressStopped()
                    guard owned else { return .cancelled }
                    if !approachCompleted {
                        let peak = min(0.12, targetSpeed?.maximumWheelSpeed ?? 0)
                        let candidates = [
                            WheelCommand(left: peak, right: peak),
                            direction > 0 ? WheelCommand(left: peak * 0.85, right: peak) :
                                WheelCommand(left: peak, right: peak * 0.85)
                        ]
                        for candidate in candidates where peak > 0 {
                            let probe = environment.depth(candidate)
                            let compatible = probe.passage == nil || probe.passage?.accepts(candidate) == true ||
                                probe.passage?.permitsFlankAvoidance(candidate, side: probe.obstacleSide) == true
                            let approved = probe.state == .clear && probe.sampleAge.isFinite &&
                                probe.sampleAge >= 0 && probe.sampleAge < DepthSafetyEvaluator.maximumRawDepthAge - Self.minimumFreshnessMargin &&
                                compatible &&
                                !(probe.obstacleSide == .left && candidate.left <= candidate.right) &&
                                !(probe.obstacleSide == .right && candidate.right <= candidate.left) &&
                                !((probe.obstacleSide == .front || probe.obstacleSide == .both) &&
                                    !Self.isSlowObstacleArc(candidate))
                            environment.emit("steering_approach_probe", [
                                "left": .number(candidate.left), "right": .number(candidate.right),
                                "approved": .bool(approved)])
                            if approved {
                                alignmentApproach = (direction, currentPose.position, environment.uptime() + 0.8, candidate)
                                environment.emit("steering_approach_started", [
                                    "deadline_s": .number(alignmentApproach!.deadline), "maximum_travel_m": .number(0.15)])
                                break
                            }
                        }
                        if alignmentApproach != nil { continue }
                    }
                    approachCompleted = false
                    alignmentAttempts += 1
                    correctiveTurn = (direction, currentPose.yaw, environment.uptime() + 0.8)
                    environment.emit("steering_alignment_started", [
                        "direction": .number(direction), "attempt": .number(Double(alignmentAttempts)),
                        "deadline_s": .number(correctiveTurn!.deadline)])
                    continue
                }
            }
            let holdingTurn = correctiveTurn != nil ||
                (route.planningMode == .reactiveDepth && targetSpeed != nil && targetSample?.holdsTranslation == true)
            let output = PursuitController(params: parameters).step(pose: currentPose, path: steeringPath)
            var desiredCommand = output.command
            if alignmentApproach != nil, targetSpeed.map({ $0.range <= $0.stopDistance }) == true { return .arrived }
            if let approach = alignmentApproach {
                desiredCommand = approach.command
            } else if let turn = correctiveTurn {
                let wheel = turn.direction * RoverConfig.minimumRotateWheelSpeed
                desiredCommand = .init(left: -wheel, right: wheel)
            } else if holdingTurn, let bearing = centeringBearing {
                let completion = targetSample?.centersContinuously == true
                    ? FollowHoldingTurn.continuousCompletionAngle : FollowHoldingTurn.completionAngle
                if abs(bearing) <= completion { return .arrived }
                let wheel = (bearing < 0 ? -1.0 : 1.0) * RoverConfig.minimumRotateWheelSpeed
                desiredCommand = .init(left: -wheel, right: wheel)
            } else if targetSpeed.map({ $0.range <= $0.stopDistance }) == true { return .arrived }
            if targetSample?.lastSeenApproach != nil,
               desiredCommand.left <= 0 || desiredCommand.right <= 0 {
                // Last-seen translation never spins or reverses on remembered
                // evidence. End this leg; the stopped search owns turning.
                return .arrived
            }
            if output.reachedGoal && !holdingTurn {
                // A pending new goal must not be discarded by old-goal arrival.
                if routeRevision == revision { return .arrived }
                if movementActive {
                    do { try await environment.stop() } catch { return .failed(.commandFailed) }
                    stationaryStopConfirmed = true
                    markProgressStopped()
                    movementActive = false
                    acceleration.reset()
                }
                await environment.sleep(.milliseconds(20)); continue
            }

            let distanceToGoal = currentPose.position.distance(to: joined.path.last ?? routeGoal)
            let approaching = targetSpeed == nil ? FollowPursuitAcceleration.approaching(desiredCommand,
                distance: distanceToGoal, params: pursuit.params) : desiredCommand
            let bypassStartupRamp = targetSpeed != nil
            // A verified target already supplies a fresh distance/velocity speed
            // ceiling. Do not add another crawl ramp on startup or recovery.
            // Depth and exact-command transport checks still run below.
            var safeCommand = bypassStartupRamp ? approaching : acceleration.limited(approaching, at: environment.uptime())
            var unavailable: DepthSafetyUnavailableReason?
            var admissionBudgetOnly = false
            var usingObstacleArc = false
            var usingPassage = false
            var depthExpiry = -Double.infinity
            var depthEvaluationCost = 0.0
            let assessmentStarted = environment.uptime()
            let observedDepthRevision = depthRevision
            let notificationLag: FollowDiagnosticValue = depthPublishedAt.map {
                let elapsed = assessmentStarted - $0
                return elapsed.isFinite && elapsed >= 0 ? .number(elapsed) : .null
            } ?? .null
            enum ArcSelection {
                case approved
                case blocked
                case depthUnavailable(DepthSafetyUnavailableReason)
            }
            func selectPassage(_ original: DepthSafetyObservation, retained: Bool) -> ArcSelection {
                guard let passage = original.passage, let targetSpeed else { return .blocked }
                let limit = min(targetSpeed.maximumWheelSpeed, original.speedLimit ?? .infinity)
                guard limit.isFinite, limit > 0 else { return .blocked }
                let candidate = retained && passage.accepts(safeCommand) && max(safeCommand.left, safeCommand.right) <= limit
                    ? safeCommand : passage.command(maximum: limit)
                var probe = original
                var expiry = depthExpiry
                if candidate != safeCommand {
                    let cost = max(0.001, depthEvaluationCost)
                    let margin = max(Self.minimumFreshnessMargin, maximumAdmissionDuration + 0.01, cost + 0.01)
                    guard depthExpiry - environment.uptime() > cost + margin else {
                        admissionBudgetOnly = original.state == .clear && depthExpiry > environment.uptime()
                        return .depthUnavailable(.staleRawDepth)
                    }
                    let began = environment.uptime()
                    probe = environment.depth(candidate)
                    depthEvaluationCost = max(depthEvaluationCost, environment.uptime() - began)
                    expiry = began + DepthSafetyEvaluator.maximumRawDepthAge - probe.sampleAge
                }
                if case .unavailable(let reason) = probe.state { return .depthUnavailable(reason) }
                guard probe.sampleAge.isFinite, probe.sampleAge >= 0 else { return .depthUnavailable(.malformedDepth) }
                guard expiry - environment.uptime() > Self.minimumFreshnessMargin else {
                    admissionBudgetOnly = probe.state == .clear && expiry > environment.uptime()
                    return .depthUnavailable(.staleRawDepth)
                }
                let observedPassage = probe.observedPassage ?? probe.passage
                guard probe.state == .clear, observedPassage?.accepts(candidate) == true,
                      !(probe.obstacleSide == .left && candidate.left <= candidate.right),
                      !(probe.obstacleSide == .right && candidate.right <= candidate.left),
                      case .allow(let accepted, _) = guardLayer.evaluate(command: candidate, depthSafety: probe),
                      accepted == candidate else { return .blocked }
                safeCommand = candidate
                depthExpiry = expiry
                usingPassage = true
                passagePreference = probe.passage
                return .approved
            }
            func forwardArc(side: DepthObstacleSide, speedLimit: Double?) -> WheelCommand? {
                guard route.planningMode == .reactiveDepth,
                      let targetSpeed, let bearing = centeringBearing,
                      abs(bearing) <= .pi / 2,
                       safeCommand.left > 0, safeCommand.right > 0 else { return nil }
                guard side == .left || side == .right else { return nil }
                let originalLimit = speedLimit.flatMap { $0.isFinite && $0 > 0 ? $0 : nil } ?? 0.12
                let peak = min(0.12, targetSpeed.maximumWheelSpeed, originalLimit)
                guard peak > 0 else { return nil }
                // Positive lateral is rover-right. A left obstacle requires
                // left wheel faster than right, regardless of target bearing.
                // If the verified person is in the escape direction, strengthen
                // centering from 10° to 30° instead of holding a fixed curve as
                // their torso drifts out of view. Never steer toward the flank.
                let centersTowardTarget = (side == .right && bearing > 0) || (side == .left && bearing < 0)
                let centering = targetSample?.centersContinuously == true && centersTowardTarget
                    ? min(1, max(0, (abs(bearing) - .pi / 18) / (.pi / 9))) : 0
                let ratio = 0.65 - 0.5 * centering
                return side == .right
                    ? WheelCommand(left: peak * ratio, right: peak)
                    : WheelCommand(left: peak, right: peak * ratio)
            }
            func selectForwardArc(awayFrom original: DepthSafetyObservation, side: DepthObstacleSide,
                                  alternative: WheelCommand? = nil) -> ArcSelection {
                guard let candidate = alternative ?? forwardArc(side: side, speedLimit: original.speedLimit) else { return .blocked }
                // A directional preference is useful even if this probe cannot
                // fit. Recovery should assess the intended arc first, not repeat
                // cruise plus arc. This stores no authorization or expiry.
                if original.obstacleSide != nil, original.sampleAge.isFinite,
                   original.sampleAge >= 0, original.sampleAge <= DepthSafetyEvaluator.maximumRawDepthAge {
                    avoidanceSide = side
                    avoidanceUntil = environment.uptime() + 0.5
                }
                // Reuse only this tick's exact-command assessment. A retained
                // direction is a candidate preference, never cached depth approval.
                var probe = original
                var expiry = depthExpiry
                if candidate != safeCommand {
                    let probeCost = max(0.001, depthEvaluationCost)
                    let projectedMargin = max(Self.minimumFreshnessMargin,
                        maximumAdmissionDuration + 0.01, probeCost + 0.01)
                    guard depthExpiry - environment.uptime() > probeCost + projectedMargin else {
                        admissionBudgetOnly = original.state == .clear && depthExpiry > environment.uptime()
                        environment.emit("obstacle_arc_depth_wait", [
                            "reason": .string("insufficient_probe_budget"),
                            "remaining_s": .number(depthExpiry - environment.uptime()),
                            "required_s": .number(probeCost + projectedMargin),
                            "obstacle_side": original.obstacleSide.map { .string($0.rawValue) } ?? .null])
                        return .depthUnavailable(.staleRawDepth)
                    }
                    let capturedAt = environment.uptime()
                    probe = environment.depth(candidate)
                    depthEvaluationCost = max(depthEvaluationCost, environment.uptime() - capturedAt)
                    expiry = capturedAt + DepthSafetyEvaluator.maximumRawDepthAge - probe.sampleAge
                }
                if case .unavailable(let reason) = probe.state { return .depthUnavailable(reason) }
                guard probe.sampleAge.isFinite, probe.sampleAge >= 0 else {
                    return .depthUnavailable(.malformedDepth)
                }
                guard expiry - environment.uptime() > Self.minimumFreshnessMargin else {
                    admissionBudgetOnly = probe.state == .clear && expiry > environment.uptime()
                    environment.emit("obstacle_arc_depth_wait", ["reason": .string("probe_expired"),
                        "remaining_s": .number(expiry - environment.uptime())])
                    return .depthUnavailable(.staleRawDepth)
                }
                guard case .allow(let accepted, _) = guardLayer.evaluate(command: candidate, depthSafety: probe),
                       accepted == candidate, probe.state == .clear,
                       probe.passage == nil || probe.passage?.accepts(candidate) == true ||
                        probe.passage?.permitsFlankAvoidance(candidate, side: probe.obstacleSide) == true,
                       !((probe.obstacleSide == .front || probe.obstacleSide == .both) &&
                         !Self.isSlowObstacleArc(candidate)),
                       !(probe.obstacleSide == .left && candidate.left <= candidate.right),
                       !(probe.obstacleSide == .right && candidate.right <= candidate.left) else { return .blocked }
                // Native transport independently checks this exact command
                // and obstacle-side evidence again immediately before HTTP.
                safeCommand = candidate
                depthExpiry = expiry
                usingObstacleArc = true
                passagePreference = nil
                avoidanceSide = side
                // This retains a steering preference, never depth authority.
                // Every tick and transport admission still checks the exact arc.
                if original.obstacleSide != nil { avoidanceUntil = environment.uptime() + 0.5 }
                return .approved
            }
            let retainingAvoidance = environment.uptime() < avoidanceUntil && avoidanceSide != nil
            if environment.uptime() >= passageSlowUntil { passagePreference = nil }
            // Retain the speed ceiling through detection gaps, never depth
            // approval. Every steering preference is checked on current depth.
            if route.planningMode == .reactiveDepth, !holdingTurn,
               safeCommand.left > 0, safeCommand.right > 0,
               environment.uptime() < passageSlowUntil {
                let peak = max(safeCommand.left, safeCommand.right)
                if peak > DepthPassageGuidance.maximumWheelSpeed {
                    let scale = DepthPassageGuidance.maximumWheelSpeed / peak
                    safeCommand = .init(left: safeCommand.left * scale, right: safeCommand.right * scale)
                }
            }
            let preferringPassage = route.planningMode == .reactiveDepth && !holdingTurn &&
                safeCommand.left > 0 && safeCommand.right > 0 && targetSpeed != nil && passagePreference != nil
            if preferringPassage, let passagePreference, let targetSpeed {
                safeCommand = passagePreference.command(maximum: targetSpeed.maximumWheelSpeed)
            } else if retainingAvoidance, let candidate = forwardArc(side: avoidanceSide!, speedLimit: nil) {
                // Assess the command we intend to send first, instead of spending
                // a depth evaluation on pursuit before repeating it for this arc.
                safeCommand = candidate
            }
            do {
                let readStarted = environment.uptime()
                let observation = environment.depth(safeCommand)
                depthEvaluationCost = environment.uptime() - readStarted
                depthExpiry = readStarted + DepthSafetyEvaluator.maximumRawDepthAge - observation.sampleAge
                // Missing boundaries retain direction only within the original
                // bounded window. This is a candidate, never cached clearance.
                if observation.passage != nil { passageSlowUntil = environment.uptime() + 1.5 }
                if route.planningMode == .reactiveDepth, safeCommand.left > 0, safeCommand.right > 0,
                   observation.state == .clear || observation.state == .caution || observation.state == .stop {
                    let frontal = observation.obstacleSide == .front || observation.obstacleSide == .both
                    if frontal, observation.state == .stop {
                        environment.emit("obstacle_stop", ["obstacle_side": .string(observation.obstacleSide!.rawValue)])
                        return .failed(.obstacle)
                    }
                    if observation.obstacleSide == nil, observation.state != .clear, observation.passage == nil {
                        guard observation.state == .caution, let targetSpeed,
                              targetSample?.centersContinuously == true else { return .failed(.obstacle) }
                        let peak = min(0.12, targetSpeed.maximumWheelSpeed, observation.speedLimit ?? 0.12)
                        guard peak.isFinite, peak > 0 else { return .failed(.obstacle) }
                        let scale = peak / max(safeCommand.left, safeCommand.right)
                        let candidates = [
                            WheelCommand(left: safeCommand.left * scale, right: safeCommand.right * scale),
                            WheelCommand(left: peak, right: peak),
                            WheelCommand(left: peak * 0.85, right: peak),
                            WheelCommand(left: peak, right: peak * 0.85)
                        ]
                        var selection = ArcSelection.blocked
                        for candidate in candidates {
                            selection = selectForwardArc(awayFrom: observation,
                                side: candidate.left > candidate.right ? .left : .right, alternative: candidate)
                            environment.emit("early_warning_candidate", [
                                "left": .number(candidate.left), "right": .number(candidate.right),
                                "result": .string({
                                    switch selection {
                                    case .approved: return "approved"
                                    case .blocked: return "blocked"
                                    case .depthUnavailable: return "depth_unavailable"
                                    }
                                }())])
                            if case .blocked = selection { continue }
                            break
                        }
                        switch selection {
                        case .approved: break
                        case .blocked: return .failed(.obstacle)
                        case .depthUnavailable(let reason): unavailable = reason
                        }
                    }
                    if usingObstacleArc || unavailable != nil {
                        // The bounded early-warning probe already selected an
                        // exact command or handed off to existing depth recovery.
                    } else if !frontal, observation.passage != nil, targetSpeed != nil,
                              observation.passage?.conflicts(with: observation.obstacleSide) != true {
                        switch selectPassage(observation, retained: preferringPassage) {
                        case .approved: break
                        case .blocked: return .failed(.obstacle)
                        case .depthUnavailable(let reason): unavailable = reason
                        }
                    } else if observation.obstacleSide != nil || retainingAvoidance {
                        let side: DepthObstacleSide
                        if observation.obstacleSide == .left || observation.obstacleSide == .right {
                            side = observation.obstacleSide!
                        } else {
                            side = retainingAvoidance ? avoidanceSide! : ((centeringBearing ?? 0) >= 0 ? .right : .left)
                        }
                        var selection = selectForwardArc(awayFrom: observation, side: side)
                        // Early frontal/bilateral occupancy is a warning. Check
                        // the other direction only if the first fresh probe was
                        // blocked; missing freshness must stop for recovery.
                        if frontal, case .blocked = selection {
                            selection = selectForwardArc(awayFrom: observation, side: side == .left ? .right : .left)
                        }
                        switch selection {
                        case .approved: break
                        case .blocked:
                            environment.emit("obstacle_arc_rejected", ["reason": .string("escape_not_approved")])
                            return .failed(.obstacle)
                        case .depthUnavailable(let reason): unavailable = reason
                        }
                    } else if observation.state != .clear {
                        // Unknown obstacle direction is not permission to guess.
                        return .failed(.obstacle)
                    }
                }
                if !usingObstacleArc, !usingPassage, unavailable == nil {
                    switch guardLayer.evaluate(command: safeCommand, depthSafety: observation) {
                    case .allow(let limited, _):
                        // Transport still validates these exact wheels before HTTP.
                        safeCommand = limited
                    case .stopDepth(let rejected):
                        switch rejected.state {
                        case .unavailable(let reason): unavailable = reason
                        case .stop, .caution: return .failed(.obstacle)
                        case .clear: unavailable = .malformedDepth
                        }
                    }
                }
            }
            if unavailable == nil, !depthExpiry.isFinite || environment.uptime() >= depthExpiry { unavailable = .staleRawDepth }
            if let unavailable {
                if admissionBudgetOnly, refreshAfter != nil {
                    // No new capture is counted. Already approved post-stop
                    // captures remain evidence within the fixed recovery window.
                    environment.emit("depth_stability_preserved", [
                        "capture_count": .number(Double(refreshCaptureCount)),
                        "reason": .string("admission_budget_only")])
                } else {
                    refreshCaptureCount = 0; refreshFirstCapture = nil; refreshLastCapture = nil
                }
                if sent, refreshAfter == nil, unavailable == .staleRawDepth || unavailable == .missingRawDepth {
                    if let failure = await recoverExpiredCommand(expiry: depthExpiry, rawDepth: true) { return .failed(failure) }
                    continue
                }
                guard (!sent || refreshAfter != nil), unavailable == .staleRawDepth || unavailable == .missingRawDepth else {
                    return .failed(.depthUnavailable(unavailable))
                }
                refreshReason = unavailable
                if refreshDeadline == nil {
                    failureCause = .pursuitDepthRefreshTimeout
                    refreshDeadline = min(began + Self.initialAdmissionTimeout, environment.uptime() + Self.depthRefreshLimit)
                    environment.emit("depth_wait_started", ["deadline_s": .number(refreshDeadline!), "reason": .string(unavailable.rawValue)])
                }
                guard owned else { return .cancelled }
                await waitForDepth(after: observedDepthRevision,
                    seconds: min(0.02, max(0, (refreshDeadline ?? .infinity) - environment.uptime())))
                continue
            }
            // Leave enough time for transport admission. While refreshing, require
            // raw depth captured after the confirmed stop, not just a new pose.
            // The remaining work is one exact-command check, not a replay of
            // the entire assessment (ack read plus every candidate arc).
            let remaining = depthExpiry - environment.uptime()
            let requiredMargin = max(Self.minimumFreshnessMargin,
                max(maximumAdmissionDuration, depthEvaluationCost) + 0.01)
            let captureTime = depthExpiry - DepthSafetyEvaluator.maximumRawDepthAge
            let capturedBeforeStop = refreshAfter.map({ captureTime <= $0.time }) == true
            if refreshAfter != nil {
                if capturedBeforeStop {
                    refreshCaptureCount = 0; refreshFirstCapture = nil; refreshLastCapture = nil
                } else if refreshLastCapture.map({ captureTime > $0 + 1e-6 }) ?? true {
                    // Depth stability describes fresh, approved post-stop
                    // captures, independently of this tick's send budget.
                    // A slow assessment must still inhibit sending, but must
                    // not erase valid evidence from earlier captures.
                    refreshFirstCapture = refreshFirstCapture ?? captureTime
                    refreshLastCapture = captureTime
                    refreshCaptureCount += 1
                }
            }
            if remaining < requiredMargin || capturedBeforeStop {
                if refreshAfter == nil {
                    if let failure = await recoverExpiredCommand(expiry: depthExpiry, rawDepth: true) { return .failed(failure) }
                }
                environment.emit("freshness_margin_wait", ["remaining_s": .number(remaining),
                    "acknowledgement_read_s": .number(ackReadDuration), "acknowledgement_reused": .bool(reuseAck),
                    "acknowledgement_snapshot": .bool(environment.acknowledgementSnapshot != nil),
                    "depth_notification_to_evaluation_s": notificationLag,
                    "required_s": .number(requiredMargin), "observed_admission_s": .number(maximumAdmissionDuration),
                    "depth_capture_s": .number(captureTime),
                    "confirmed_stop_s": refreshAfter.map { .number($0.time) } ?? .null,
                    "waiting_for_post_stop_depth": .bool(refreshAfter.map { captureTime <= $0.time } ?? false)])
                await waitForDepth(after: observedDepthRevision,
                    seconds: min(0.02, max(0, (refreshDeadline ?? .infinity) - environment.uptime())))
                continue
            }
            if refreshAfter != nil {
                guard refreshCaptureCount >= 3, captureTime - refreshFirstCapture! >= 0.06 else {
                    environment.emit("depth_stability_wait", ["capture_count": .number(Double(refreshCaptureCount)),
                        "capture_span_s": .number(captureTime - refreshFirstCapture!),
                        "deadline_s": .number(refreshDeadline!)])
                    await waitForDepth(after: observedDepthRevision, seconds: min(0.02, max(0, refreshDeadline! - environment.uptime())))
                    continue
                }
            }
            guard owned else { return .cancelled }
            let sendRevision = revision
            let finalSource = environment.pose()
            let sendTime = environment.uptime()
            guard sendTime.isFinite, sendTime >= time else { return .failed(.trackingLost) }
            if let rejection = finalSource.rejection(at: sendTime, expectedGeneration: generation) {
                recordAcceptance("source_" + rejection, job: nil, source: finalSource, at: sendTime)
                return .failed(.trackingLost)
            }
            if finalSource.pose != current.pose { await environment.sleep(.milliseconds(20)); continue }
            if !sent || requiresCurrentRouteAfterPause, routeRevision != revision { continue }
            if let refreshDeadline, sendTime >= refreshDeadline { return .failed(.depthUnavailable(refreshReason)) }
            if !sent, sendTime >= began + Self.initialAdmissionTimeout {
                recordAcceptance("startup_timeout", job: job, source: finalSource, at: sendTime)
                return .failed(.noPath)
            }
            if holdingTurn {
                markProgressStopped()
                previousPosition = nil; previousGoal = nil
                if stationaryStopConfirmed {
                    turnProgress.reset(); previousTurnYaw = nil; directedTurnTravel = 0
                }
                if let previousTurnYaw {
                    let delta = currentPose.yaw - previousTurnYaw
                    directedTurnTravel += atan2(sin(delta), cos(delta)) * previousTurnDirection
                }
                previousTurnYaw = currentPose.yaw
                previousTurnDirection = safeCommand.right > safeCommand.left ? 1 : -1
                if turnProgress.observe(distanceToGoal: -directedTurnTravel, now: environment.date(), commanded: true) {
                    return .failed(.stalled)
                }
            } else {
                turnProgress.reset(); previousTurnYaw = nil; directedTurnTravel = 0
                if let previousPosition, let previousGoal {
                    approach += previousPosition.distance(to: previousGoal) - currentPose.position.distance(to: previousGoal)
                }
                previousPosition = currentPose.position; previousGoal = routeGoal
                // The pre-turn approach has its own non-renewable time/travel
                // bounds; do not spend the pursuit stall budget twice.
                if alignmentApproach == nil,
                   progress.observe(distanceToGoal: -approach, now: progressTime(environment.date()), commanded: true) { return .failed(.stalled) }
            }
            let ackExpiry = ack.map { sendTime + RoverConfig.commsWatchdogTimeout - environment.date().timeIntervalSince($0) } ?? sendTime
            let expiry = min(alignmentApproach?.deadline ?? .infinity, correctiveTurn?.deadline ?? .infinity, depthExpiry, finalSource.sourceTimestamp.map { $0 + 0.5 } ?? .infinity,
                ackExpiry, targetExpiry, sent ? .infinity : began + Self.initialAdmissionTimeout, refreshDeadline ?? .infinity)
            guard sendTime < expiry else {
                if sendTime >= targetExpiry { continue }
                if let failure = await recoverExpiredCommand(expiry: expiry) { return .failed(failure) }
                continue
            }
            if sendTime - lastDiagnostic >= 0.5 {
                lastDiagnostic = sendTime
                environment.emit("drive_tick", ["goal_x": .number(routeGoal.x), "goal_z": .number(routeGoal.y),
                    "planning_mode": .string(route.planningMode.rawValue),
                    "acknowledgement_snapshot": .bool(environment.acknowledgementSnapshot != nil),
                    "depth_notification_to_evaluation_s": notificationLag,
                    "acknowledgement_read_s": .number(ackReadDuration), "acknowledgement_reused": .bool(reuseAck),
                    "requested_goal_x": .number(goal.x), "requested_goal_z": .number(goal.y),
                    "revision": .number(Double(revision)), "path_revision": .number(Double(routeRevision ?? 0)),
                    "planning_pending": .bool(job != nil), "depth_valid_until_s": .number(depthExpiry),
                    "desired_command_left": .number(desiredCommand.left), "desired_command_right": .number(desiredCommand.right),
                    "forward_acceleration_mps2": .number(FollowPursuitAcceleration.acceleration),
                    "target_range_m": targetSpeed.map { .number($0.range) } ?? .null,
                    "target_authority": .string(targetSample?.lastSeenApproach == nil ? "measured" : "last_seen_approach"),
                    "target_capture_s": targetSample.map { .number($0.timestamp) } ?? .null,
                    "target_radial_velocity_mps": targetSpeed.map { .number($0.radialVelocity) } ?? .null,
                    "target_speed_ceiling": targetSpeed.map { .number($0.maximumWheelSpeed) } ?? .null,
                    "target_speed_valid_until_s": targetSpeed.map { _ in .number(targetExpiry) } ?? .null,
                    "catchup_ramp_bypassed": .bool(targetSpeed?.bypassesStartupRamp == true),
                    "target_speed_direct": .bool(bypassStartupRamp),
                    "target_centering_bearing_rad": centeringBearing.map { .number($0) } ?? .null,
                    "target_centering_lookahead_m": centeringBearing == nil ? .null : .number(parameters.lookahead),
                    "reactive_obstacle_arc": .bool(usingObstacleArc),
                    "passage_centering": .bool(usingPassage),
                    "passage_slow_hold": .bool(environment.uptime() < passageSlowUntil),
                    "passage_width_m": passagePreference.map { .number($0.width) } ?? .null,
                    "passage_center_lateral_m": passagePreference.map { .number($0.centerLateral) } ?? .null,
                    "holding_turn": .bool(holdingTurn),
                    "corrective_alignment": .bool(correctiveTurn != nil),
                    "distance_to_goal_m": .number(distanceToGoal),
                    "approach_command_left": .number(approaching.left), "approach_command_right": .number(approaching.right),
                    "required_send_margin_s": .number(requiredMargin),
                    "observed_admission_s": .number(maximumAdmissionDuration),
                    "command_left": .number(safeCommand.left), "command_right": .number(safeCommand.right)])
            }
            environment.moving()
            guard owned, !perceptionPaused, revision == sendRevision else { continue }
            guard environment.uptime() < expiry else {
                if environment.uptime() >= targetExpiry { continue }
                if let failure = await recoverExpiredCommand(expiry: expiry) { return .failed(failure) }
                continue
            }
            let sendStarted = environment.uptime()
            let sendDate = environment.date()
            stationaryStopConfirmed = false
            do { try await environment.send(safeCommand, expiry) }
            catch {
                guard owned else { return .cancelled }
                if let rejected = error as? FollowPursuitSendFailure {
                    if rejected.definitelyUnsent, rejected.targetAuthorityChanged { continue }
                    if rejected.definitelyUnsent, perceptionPaused { continue }
                    if rejected.definitelyUnsent, rejected.authorizationExpired {
                        if environment.uptime() >= targetExpiry { continue }
                        if let failure = await recoverExpiredCommand(expiry: expiry) { return .failed(failure) }
                        continue
                    }
                    if rejected.definitelyUnsent, revision != sendRevision { continue }
                    return .failed(rejected.reason)
                }
                if error as? FollowTurnBurstTransportDenial == .expired { return .failed(.depthUnavailable(.staleRawDepth)) }
                return .failed(.commandFailed)
            }
            maximumAdmissionDuration = max(maximumAdmissionDuration,
                environment.admissionDuration() ?? (environment.uptime() - sendStarted))
            creepHoldRequested = creepResponse.observe(command: safeCommand, position: currentPose.position,
                now: environment.uptime(), eligible: creepEligible && !holdingTurn && refreshAfter == nil)
            if let direction = steeringResponse.observe(command: safeCommand, yaw: currentPose.yaw,
                bearing: centeringBearing ?? 0, now: environment.uptime(),
                eligible: correctionEligible && alignmentApproach == nil && !holdingTurn && refreshAfter == nil) {
                alignmentRequested = direction
            }
            sent = true
            if !holdingTurn && alignmentApproach == nil { resumeProgress(at: sendDate) }
            requiresCurrentRouteAfterPause = false
            acceleration.sent(safeCommand, at: sendStarted)
            movementActive = true
            if refreshDeadline != nil { environment.emit("depth_wait_recovered", [:]) }
            let completedAt = environment.uptime()
            if refreshDeadline == nil, safeCommand.left != 0 || safeCommand.right != 0, completedAt < depthExpiry,
               depthExpiry - sendStarted >= requiredMargin + 0.02,
               lastHealthyCapture.map({ captureTime > $0 + 1e-6 }) ?? true {
                if let previous = lastHealthySend, completedAt - previous > 0.3 { resetHealthyDelivery() }
                healthySince = healthySince ?? completedAt
                lastHealthySend = completedAt
                lastHealthyCapture = captureTime
                healthySends += 1
                if expiryRecoveries > 0, healthySends >= 10, completedAt - healthySince! >= 2 {
                    environment.emit("recovery_budget_replenished", ["healthy_send_count": .number(Double(healthySends)),
                        "healthy_duration_s": .number(completedAt - healthySince!),
                        "previous_recovery_count": .number(Double(expiryRecoveries))])
                    expiryRecoveries = 0
                    resetHealthyDelivery()
                }
            } else { resetHealthyDelivery() }
            refreshDeadline = nil
            refreshAfter = nil
            failureCause = nil
            // New publication wakes the control loop; missing publication cannot
            // postpone its next check past the original command validity deadline.
            await waitForDepth(after: observedDepthRevision,
                seconds: min(0.1, max(0, expiry - environment.uptime())))
        }
        return .cancelled
    }
}
