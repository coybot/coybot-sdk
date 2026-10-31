import Foundation
import RoverNav

struct FollowPursuitPlan: Sendable {
    let path: [Vec2]
    var map: Costmap? = nil

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
        let send: @MainActor (WheelCommand, Double) async throws -> Void
        let stop: @MainActor () async throws -> Void
        let sleep: @MainActor (Duration) async -> Void
        let publishPath: @MainActor ([Vec2]) -> Void
        let moving: @MainActor () -> Void
        let emit: @MainActor (String, [String: FollowDiagnosticValue]) -> Void
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
    private(set) var revision: UInt64 = 0
    private(set) var goal: Vec2
    private(set) var acceptingGoals = true
    static let planningTimeout = 1.0
    static let initialAdmissionTimeout = 2.0
    static let depthRefreshLimit = 0.6

    init(goal: Vec2, environment: Environment) { self.goal = goal; self.environment = environment }

    func updateGoal(_ goal: Vec2) -> Bool {
        guard active, acceptingGoals, goal.x.isFinite, goal.y.isFinite else { return false }
        if self.goal != goal { self.goal = goal; revision &+= 1 }
        return true
    }

    func inhibit() {
        active = false; acceptingGoals = false
        job?.task?.cancel(); job = nil
    }

    private var owned: Bool { active && !Task.isCancelled && environment.owned() }

    private func beginPlan(from source: NavigationPoseSample, reason: String) {
        guard job == nil, let pose = source.pose else { return }
        let next = Job(revision: revision, goal: goal, source: source, started: environment.uptime())
        job = next
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
        var progress = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.05)
        var previousPosition: Vec2?
        var previousGoal: Vec2?
        var approach = 0.0
        var lastDiagnostic = -Double.infinity

        while owned {
            let tick = environment.uptime()
            guard tick.isFinite, tick >= began else { return .failed(.trackingLost) }
            let source = environment.pose()
            guard source.rejection(at: tick, expectedGeneration: generation) == nil,
                  let pose = source.pose else { return .failed(.trackingLost) }
            if !sent, tick >= began + Self.initialAdmissionTimeout { return .failed(.noPath) }
            if let refreshDeadline, tick >= refreshDeadline { return .failed(.depthUnavailable(refreshReason)) }
            if let job {
                guard tick >= job.started, tick < job.started + Self.planningTimeout else { return .failed(.noPath) }
                if job.completed {
                    self.job = nil
                    if job.revision == revision {
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
            if job == nil, route == nil || routeRevision != revision || tick - lastPlan >= 1 {
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
            depthAssessment: for attempt in 0..<2 {
                let readStarted = environment.uptime()
                let observation = environment.depth(safeCommand)
                depthExpiry = readStarted + DepthSafetyEvaluator.maximumRawDepthAge - observation.sampleAge
                switch guardLayer.evaluate(command: safeCommand, depthSafety: observation) {
                case .allow(let limited, _):
                    if limited == safeCommand { break depthAssessment }
                    guard attempt == 0 else { return .failed(.obstacle) }
                    safeCommand = limited
                case .stopDepth(let rejected):
                    switch rejected.state {
                    case .unavailable(let reason): unavailable = reason
                    case .stop, .caution: return .failed(.obstacle)
                    case .clear: unavailable = .malformedDepth
                    }
                    break depthAssessment
                }
            }
            if unavailable == nil, !depthExpiry.isFinite || environment.uptime() >= depthExpiry { unavailable = .staleRawDepth }
            if let unavailable {
                guard !sent, unavailable == .staleRawDepth || unavailable == .missingRawDepth else {
                    return .failed(.depthUnavailable(unavailable))
                }
                refreshReason = unavailable
                if refreshDeadline == nil {
                    refreshDeadline = min(began + Self.initialAdmissionTimeout, environment.uptime() + Self.depthRefreshLimit)
                    environment.emit("depth_wait_started", ["deadline_s": .number(refreshDeadline!), "reason": .string(unavailable.rawValue)])
                }
                guard owned else { return .cancelled }
                await environment.sleep(.milliseconds(20)); continue
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
            guard sendTime < expiry else { return .failed(.depthUnavailable(.staleRawDepth)) }
            if sendTime - lastDiagnostic >= 0.5 {
                lastDiagnostic = sendTime
                environment.emit("drive_tick", ["goal_x": .number(routeGoal.x), "goal_z": .number(routeGoal.y),
                    "requested_goal_x": .number(goal.x), "requested_goal_z": .number(goal.y),
                    "revision": .number(Double(revision)), "path_revision": .number(Double(routeRevision ?? 0)),
                    "planning_pending": .bool(job != nil), "depth_valid_until_s": .number(depthExpiry),
                    "command_left": .number(safeCommand.left), "command_right": .number(safeCommand.right)])
            }
            if refreshDeadline != nil { environment.emit("depth_wait_recovered", [:]) }
            environment.moving()
            guard owned, revision == sendRevision else { continue }
            guard environment.uptime() < expiry else { return .failed(.depthUnavailable(.staleRawDepth)) }
            do { try await environment.send(safeCommand, expiry) }
            catch {
                guard owned else { return .cancelled }
                if let rejected = error as? FollowPursuitSendFailure {
                    if rejected.definitelyUnsent, revision != sendRevision { continue }
                    return .failed(rejected.reason)
                }
                if error as? FollowTurnBurstTransportDenial == .expired { return .failed(.depthUnavailable(.staleRawDepth)) }
                return .failed(.commandFailed)
            }
            sent = true
            movementActive = true
            refreshDeadline = nil
            await environment.sleep(.milliseconds(100))
        }
        return .cancelled
    }
}
