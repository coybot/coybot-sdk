import Foundation
import RoverNav

struct FollowPursuitPlan: Sendable {
    static let joinMargin = 0.02
    let path: [Vec2]
    var map: Costmap? = nil
    var rejection: PathPlanningFailure? = nil
    var diagnostics: [String: String] = [:]
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
    private let guardLayer = ObstacleGuard()
    private var job: Job?
    private var active = true
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
    private(set) var failureCause: FollowMotionFailureCause?

    init(goal: Vec2, environment: Environment) { self.goal = goal; self.environment = environment }

    func updateGoal(_ goal: Vec2) -> Bool {
        guard active, acceptingGoals, goal.x.isFinite, goal.y.isFinite else { return false }
        if self.goal != goal { self.goal = goal; revision &+= 1 }
        return true
    }

    func inhibit() {
        active = false; acceptingGoals = false
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
        var acceleration = FollowPursuitAcceleration()
        var refreshDeadline: Double?
        var refreshReason = DepthSafetyUnavailableReason.staleRawDepth
        var expiryRecoveries = 0
        var refreshAfter: (frame: ARFrameID?, time: Double)?
        var progress = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.05)
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
            do { try await environment.stop() } catch { return .commandFailed }
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
                        if result.map == nil, let origin = job.source.pose, origin.position.distance(to: pose.position) > Self.maximumUnmappedPoseShift {
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
            guard let route, sent || routeRevision == revision else {
                await environment.sleep(.milliseconds(20)); continue
            }
            if let refreshDeadline, time >= refreshDeadline { return .failed(.depthUnavailable(refreshReason)) }
            guard let joined = route.joined(from: currentPose.position) else {
                recordAcceptance("active_route_join_blocked", job: nil, source: current, at: time)
                return .failed(.noPath)
            }
            let output = pursuit.step(pose: currentPose, path: joined.path)
            if output.reachedGoal {
                // A pending new goal must not be discarded by old-goal arrival.
                if routeRevision == revision { return .arrived }
                if movementActive {
                    do { try await environment.stop() } catch { return .failed(.commandFailed) }
                    movementActive = false
                    acceleration.reset()
                }
                await environment.sleep(.milliseconds(20)); continue
            }

            let distanceToGoal = currentPose.position.distance(to: joined.path.last ?? routeGoal)
            let approaching = FollowPursuitAcceleration.approaching(output.command,
                distance: distanceToGoal, params: pursuit.params)
            var safeCommand = acceleration.limited(approaching, at: environment.uptime())
            var unavailable: DepthSafetyUnavailableReason?
            var depthExpiry = -Double.infinity
            let assessmentStarted = environment.uptime()
            let observedDepthRevision = depthRevision
            let notificationLag: FollowDiagnosticValue = depthPublishedAt.map {
                let elapsed = assessmentStarted - $0
                return elapsed.isFinite && elapsed >= 0 ? .number(elapsed) : .null
            } ?? .null
            do {
                let readStarted = environment.uptime()
                let observation = environment.depth(safeCommand)
                depthExpiry = readStarted + DepthSafetyEvaluator.maximumRawDepthAge - observation.sampleAge
                switch guardLayer.evaluate(command: safeCommand, depthSafety: observation) {
                case .allow(let limited, _):
                    // ObstacleGuard scales both wheels proportionally. Do not
                    // spend a second full evaluation here: transport must still
                    // validate this exact command against current depth before HTTP.
                    safeCommand = limited
                case .stopDepth(let rejected):
                    switch rejected.state {
                    case .unavailable(let reason): unavailable = reason
                    case .stop, .caution: return .failed(.obstacle)
                    case .clear: unavailable = .malformedDepth
                    }
                }
            }
            if unavailable == nil, !depthExpiry.isFinite || environment.uptime() >= depthExpiry { unavailable = .staleRawDepth }
            if let unavailable {
                refreshCaptureCount = 0; refreshFirstCapture = nil; refreshLastCapture = nil
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
            let remaining = depthExpiry - environment.uptime()
            let requiredMargin = max(Self.minimumFreshnessMargin,
                max(maximumAdmissionDuration, environment.uptime() - assessmentStarted) + 0.01)
            let captureTime = depthExpiry - DepthSafetyEvaluator.maximumRawDepthAge
            if remaining < requiredMargin || refreshAfter.map({ captureTime <= $0.time }) == true {
                refreshCaptureCount = 0; refreshFirstCapture = nil; refreshLastCapture = nil
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
                if refreshLastCapture.map({ captureTime > $0 + 1e-6 }) ?? true {
                    refreshFirstCapture = refreshFirstCapture ?? captureTime
                    refreshLastCapture = captureTime
                    refreshCaptureCount += 1
                }
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
            if !sent, routeRevision != revision { continue }
            if let refreshDeadline, sendTime >= refreshDeadline { return .failed(.depthUnavailable(refreshReason)) }
            if !sent, sendTime >= began + Self.initialAdmissionTimeout {
                recordAcceptance("startup_timeout", job: job, source: finalSource, at: sendTime)
                return .failed(.noPath)
            }
            if let previousPosition, let previousGoal {
                approach += previousPosition.distance(to: previousGoal) - currentPose.position.distance(to: previousGoal)
            }
            previousPosition = currentPose.position; previousGoal = routeGoal
            if progress.observe(distanceToGoal: -approach, now: environment.date(), commanded: true) { return .failed(.stalled) }
            let ackExpiry = ack.map { sendTime + RoverConfig.commsWatchdogTimeout - environment.date().timeIntervalSince($0) } ?? sendTime
            let expiry = min(depthExpiry, finalSource.sourceTimestamp.map { $0 + 0.5 } ?? .infinity,
                ackExpiry, sent ? .infinity : began + Self.initialAdmissionTimeout, refreshDeadline ?? .infinity)
            guard sendTime < expiry else {
                if let failure = await recoverExpiredCommand(expiry: expiry) { return .failed(failure) }
                continue
            }
            if sendTime - lastDiagnostic >= 0.5 {
                lastDiagnostic = sendTime
                environment.emit("drive_tick", ["goal_x": .number(routeGoal.x), "goal_z": .number(routeGoal.y),
                    "acknowledgement_snapshot": .bool(environment.acknowledgementSnapshot != nil),
                    "depth_notification_to_evaluation_s": notificationLag,
                    "acknowledgement_read_s": .number(ackReadDuration), "acknowledgement_reused": .bool(reuseAck),
                    "requested_goal_x": .number(goal.x), "requested_goal_z": .number(goal.y),
                    "revision": .number(Double(revision)), "path_revision": .number(Double(routeRevision ?? 0)),
                    "planning_pending": .bool(job != nil), "depth_valid_until_s": .number(depthExpiry),
                    "desired_command_left": .number(output.command.left), "desired_command_right": .number(output.command.right),
                    "forward_acceleration_mps2": .number(FollowPursuitAcceleration.acceleration),
                    "distance_to_goal_m": .number(distanceToGoal),
                    "approach_command_left": .number(approaching.left), "approach_command_right": .number(approaching.right),
                    "required_send_margin_s": .number(requiredMargin),
                    "observed_admission_s": .number(maximumAdmissionDuration),
                    "command_left": .number(safeCommand.left), "command_right": .number(safeCommand.right)])
            }
            environment.moving()
            guard owned, revision == sendRevision else { continue }
            guard environment.uptime() < expiry else {
                if let failure = await recoverExpiredCommand(expiry: expiry) { return .failed(failure) }
                continue
            }
            let sendStarted = environment.uptime()
            do { try await environment.send(safeCommand, expiry) }
            catch {
                guard owned else { return .cancelled }
                if let rejected = error as? FollowPursuitSendFailure {
                    if rejected.definitelyUnsent, rejected.authorizationExpired {
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
            sent = true
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
