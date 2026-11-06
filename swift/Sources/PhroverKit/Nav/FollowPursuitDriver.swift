import Foundation
import RoverNav

struct FollowPursuitPlan: Sendable {
    let path: [Vec2]
    var map: Costmap? = nil
    var rejection: PathPlanningFailure? = nil
    var diagnostics: [String: String] = [:]

    func joined(from position: Vec2) -> Self? {
        guard let map else { return self }
        guard let closest = path.indices.min(by: { path[$0].distance(to: position) < path[$1].distance(to: position) }) else { return nil }
        for index in closest..<path.count where map.isSegmentClear(from: position, to: path[index], margin: 0.02) {
            return .init(path: Array(path[index...]), map: map)
        }
        return nil
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
    }

    private let environment: Environment
    private let pursuit = PursuitController(params: .init(wheelBase: RoverConfig.wheelBase,
        goalTolerance: 0.2, minimumRotateWheelSpeed: RoverConfig.minimumRotateWheelSpeed))
    private let guardLayer = ObstacleGuard()
    private var job: Job?
    private var active = true
    private var depthRevision: UInt64 = 0
    private var depthWaiter: (id: UUID, continuation: CheckedContinuation<Void, Never>)?

    func depthPublished() {
        depthRevision &+= 1
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
        var routeGoal = goal
        var lastPlan = began
        var sent = false
        var movementActive = false
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
            guard source.rejection(at: tick, expectedGeneration: generation) == nil,
                  let pose = source.pose else { return .failed(.trackingLost) }
            if !sent, tick >= began + Self.initialAdmissionTimeout { return .failed(.noPath) }
            if let refreshDeadline, tick >= refreshDeadline { return .failed(.depthUnavailable(refreshReason)) }
            if let boundary = refreshAfter {
                guard let timestamp = source.sourceTimestamp, timestamp > boundary.time,
                      let frame = source.frameID, let previous = boundary.frame,
                      frame.generation == previous.generation, frame.sequence > previous.sequence else {
                    await environment.sleep(.milliseconds(20)); continue
                }
            }
            if let job {
                guard tick >= job.started, tick < job.started + Self.planningTimeout else { return .failed(.noPath) }
                if job.completed {
                    self.job = nil
                    if job.revision == revision {
                        if let rejected = job.result?.rejection {
                            switch rejected {
                            case .startBlocked: failureCause = .pursuitStartBlocked
                            case .goalBlocked: failureCause = .pursuitGoalBlocked
                            case .noConnectedPath: failureCause = .pursuitDisconnectedPath
                            case .startOutsideMap, .goalOutsideMap: failureCause = .pursuitOutsideMap
                            }
                            var fields = (job.result?.diagnostics ?? [:]).mapValues { FollowDiagnosticValue.string($0) }
                            fields["planning_rejection"] = .string(rejected.rawValue)
                            environment.emit("planning_rejected", fields)
                            return .failed(.noPath)
                        }
                        guard job.source.frameID?.generation == generation,
                              let result = job.result, !result.path.isEmpty,
                              result.path.allSatisfy({ $0.x.isFinite && $0.y.isFinite }),
                              result.path.last!.distance(to: job.goal) <= max(result.map?.resolution ?? 0, 1e-6) else { return .failed(.noPath) }
                        guard let joined = result.joined(from: pose.position) else { return .failed(.noPath) }
                        if result.map == nil, let origin = job.source.pose, origin.position.distance(to: pose.position) > 0.02 {
                            return .failed(.trackingLost)
                        }
                        route = joined; routeRevision = job.revision; routeGoal = job.goal; lastPlan = tick
                        environment.publishPath(joined.path)
                        environment.emit("planning_completed", ["elapsed_s": .number(tick - job.started),
                            "revision": .number(Double(job.revision))])
                    } else {
                        environment.emit("planning_superseded", ["revision": .number(Double(job.revision))])
                    }
                }
            }
            guard owned else { return .cancelled }
            if job == nil, tick - lastPlanStarted >= 0.5,
               refreshAfter == nil || route == nil || routeRevision != revision,
               route == nil || routeRevision != revision || tick - lastPlan >= 1 {
                beginPlan(from: source, reason: route == nil ? "startup" : (routeRevision != revision ? "live_goal" : "periodic"))
            }

            let ack = await environment.acknowledgement()
            guard owned else { return .cancelled }
            let current = environment.pose()
            let time = environment.uptime()
            guard time.isFinite, time >= tick else { return .failed(.trackingLost) }
            if !sent, time >= began + Self.initialAdmissionTimeout { return .failed(.noPath) }
            if let job, time >= job.started + Self.planningTimeout { return .failed(.noPath) }
            guard current.rejection(at: time, expectedGeneration: generation) == nil,
                  let currentPose = current.pose else { return .failed(.trackingLost) }
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
            guard let joined = route.joined(from: currentPose.position) else { return .failed(.noPath) }
            let output = pursuit.step(pose: currentPose, path: joined.path)
            if output.reachedGoal {
                // A pending new goal must not be discarded by old-goal arrival.
                if routeRevision == revision { return .arrived }
                if movementActive {
                    do { try await environment.stop() } catch { return .failed(.commandFailed) }
                    movementActive = false
                }
                await environment.sleep(.milliseconds(20)); continue
            }

            var safeCommand = output.command
            var unavailable: DepthSafetyUnavailableReason?
            var depthExpiry = -Double.infinity
            let assessmentStarted = environment.uptime()
            let observedDepthRevision = depthRevision
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
            guard finalSource.rejection(at: sendTime, expectedGeneration: generation) == nil else { return .failed(.trackingLost) }
            if finalSource.pose != current.pose { await environment.sleep(.milliseconds(20)); continue }
            if !sent, routeRevision != revision { continue }
            if let refreshDeadline, sendTime >= refreshDeadline { return .failed(.depthUnavailable(refreshReason)) }
            if !sent, sendTime >= began + Self.initialAdmissionTimeout { return .failed(.noPath) }
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
                    "requested_goal_x": .number(goal.x), "requested_goal_z": .number(goal.y),
                    "revision": .number(Double(revision)), "path_revision": .number(Double(routeRevision ?? 0)),
                    "planning_pending": .bool(job != nil), "depth_valid_until_s": .number(depthExpiry),
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
