import Foundation
import RoverNav

struct FollowObstacleDetourRequest {
    let id: UUID
    let generation: UInt64
    let deadline: Double
    let preferredSide: Double
    let target: Vec2
}

/// One attempt per follow session. A new goal cannot renew the maneuver budget.
@MainActor
final class FollowObstacleDetourContext {
    private var used = false
    let request: (NavigationPoseSample) -> FollowObstacleDetourRequest?
    let began: (FollowObstacleDetourRequest) -> Void
    let target: () -> Vec2?

    init(request: @escaping (NavigationPoseSample) -> FollowObstacleDetourRequest?,
         began: @escaping (FollowObstacleDetourRequest) -> Void, target: @escaping () -> Vec2?) {
        self.request = request; self.began = began; self.target = target
    }

    func begin(at source: NavigationPoseSample) -> FollowObstacleDetourRequest? {
        guard !used, let request = request(source) else { return nil }
        used = true
        began(request)
        return request
    }
}

enum FollowObstacleDetourScope {
    @TaskLocal static var context: FollowObstacleDetourContext?
    @TaskLocal static var active = false
}

/// Stop/look/short advance/turn back. Uses the existing transport and pure-turn
/// controller; never infers free space from the missing side of an image.
@MainActor
final class FollowObstacleDetour {
    static let duration = 12.0
    static let advanceDistance = 0.25
    static let maximumExcursion = 0.35
    private let request: FollowObstacleDetourRequest
    private let control: FollowPursuitDriver.Environment
    private let turn: @MainActor (Double) async -> NavigationResult
    private let stopFence: () -> FollowTurnStopFence?
    private let target: () -> Vec2?
    private var origin: Pose2D?
    private var advanceOrigin: Pose2D?

    init(request: FollowObstacleDetourRequest, control: FollowPursuitDriver.Environment,
         turn: @escaping @MainActor (Double) async -> NavigationResult,
         stopFence: @escaping () -> FollowTurnStopFence?, target: @escaping () -> Vec2?) {
        self.request = request; self.control = control; self.turn = turn
        self.stopFence = stopFence; self.target = target
    }

    /// Also used at native transport admission. Deliberately does not call
    /// control.owned(), which itself evaluates this recovery authorization.
    func withinBounds() -> Bool {
        let time = control.uptime(), source = control.pose()
        guard time.isFinite, time < request.deadline,
              source.rejection(at: time, expectedGeneration: request.generation, requireEnriched: true) == nil,
              let pose = source.pose else { return false }
        if let origin, pose.position.distance(to: origin.position) > Self.maximumExcursion { return false }
        if let advanceOrigin {
            let delta = pose.position - advanceOrigin.position
            let forward = delta.x * cos(advanceOrigin.yaw) + delta.y * sin(advanceOrigin.yaw)
            let lateral = -delta.x * sin(advanceOrigin.yaw) + delta.y * cos(advanceOrigin.yaw)
            guard forward >= -0.03, forward <= 0.30, abs(lateral) <= 0.08,
                  abs(FollowReacquisitionPlanner.wrap(pose.yaw - advanceOrigin.yaw)) <= .pi / 18 else { return false }
        }
        return true
    }

    func run() async -> NavigationResult {
        emit("started")
        let result = await maneuver()
        // Caller cancellation retains the controller's independent stop owner.
        // Otherwise confirm the final stop, including on timeout/blocked space.
        if control.owned(), !Task.isCancelled {
            do { try await control.stop() }
            catch { return .failed(.commandFailed) }
        }
        emit("finished", ["result": .string(String(describing: result))])
        return result
    }

    private func maneuver() async -> NavigationResult {
        guard request.deadline.isFinite, request.preferredSide == -1 || request.preferredSide == 1,
              request.target.x.isFinite, request.target.y.isFinite else { return .failed(.trackingLost) }
        do { try await control.stop() } catch { return .failed(.commandFailed) }
        guard let stopped = await settledPose(), let initial = stopped.pose else { return failure() }
        origin = initial
        var selected: Pose2D?
        for side in [request.preferredSide, -request.preferredSide] {
            if side != request.preferredSide {
                let result = await rotate(to: initial.yaw)
                guard result == .arrived else { return result }
            }
            for step in 1...3 {
                let heading = FollowReacquisitionPlanner.wrap(initial.yaw + side * Double(step) * .pi / 12)
                emit("inspect", ["side": .number(side), "heading_rad": .number(heading)])
                let result = await rotate(to: heading)
                guard result == .arrived else { return result }
                switch await corridor() {
                case .clear:
                    selected = control.pose().pose
                case .blocked: continue
                case .failed(let reason): return .failed(reason)
                }
                break
            }
            if selected != nil { break }
        }
        guard let selected else { return .failed(.obstacle) }
        advanceOrigin = selected
        emit("advance", ["distance_m": .number(Self.advanceDistance)])
        let advancing = await advance()
        guard advancing == .arrived else { return advancing }
        do { try await control.stop() } catch { return .failed(.commandFailed) }
        guard let stopped = await settledPose(), let pose = stopped.pose else { return failure() }
        advanceOrigin = nil
        let point = target() ?? request.target
        guard point.x.isFinite, point.y.isFinite else { return .failed(.trackingLost) }
        if point.distance(to: pose.position) > 0.1 {
            let heading = atan2(point.y - pose.position.y, point.x - pose.position.x)
            emit("return_to_target", ["heading_rad": .number(heading)])
            let result = await rotate(to: heading)
            guard result == .arrived else { return result }
        }
        // Returning to a remembered direction is not target acquisition. The
        // coordinator must obtain a fresh full-frame post-stop target match.
        return .reobservationRequired
    }

    private func failure() -> NavigationResult {
        if Task.isCancelled { return .cancelled }
        if control.pose().rejection(at: control.uptime(), expectedGeneration: request.generation, requireEnriched: true) != nil {
            return .failed(.trackingLost)
        }
        if control.uptime() < request.deadline, withinBounds(), !control.owned() { return .cancelled }
        return .failed(.obstacle)
    }

    private func settledPose() async -> NavigationPoseSample? {
        guard let fence = stopFence(), fence.sourceGeneration == request.generation else { return nil }
        while control.owned(), !Task.isCancelled, withinBounds() {
            let source = control.pose(), time = control.uptime()
            if time >= fence.acknowledgementUptime + 0.3,
               let timestamp = source.sourceTimestamp, timestamp > fence.acknowledgementUptime,
               let frame = source.frameID, frame.sequence > (fence.highestSequence ?? 0) { return source }
            await control.sleep(.milliseconds(20))
        }
        return nil
    }

    private func rotate(to heading: Double) async -> NavigationResult {
        guard control.owned(), !Task.isCancelled, withinBounds() else { return failure() }
        let result = await turn(heading)
        guard result == .arrived else { return control.uptime() >= request.deadline ? .failed(.obstacle) : result }
        guard withinBounds(), let pose = control.pose().pose,
              abs(FollowReacquisitionPlanner.wrap(heading - pose.yaw)) <= .pi / 36 else { return .failed(.trackingLost) }
        return .arrived
    }

    private enum Corridor { case clear, blocked, failed(NavigationFailure) }
    private func corridor() async -> Corridor {
        guard let fence = stopFence() else { return .failed(.commandFailed) }
        let deadline = min(request.deadline, control.uptime() + 0.6)
        var captures = 0, first: Double?, last: Double?
        while control.owned(), !Task.isCancelled, withinBounds(), control.uptime() < deadline {
            let time = control.uptime()
            let observation = control.depth(.init(left: 0.12, right: 0.12))
            if case .unavailable(let reason) = observation.state { return .failed(.depthUnavailable(reason)) }
            guard observation.sampleAge.isFinite, observation.sampleAge >= 0,
                  observation.sampleAge < DepthSafetyEvaluator.maximumRawDepthAge else { return .failed(.depthUnavailable(.staleRawDepth)) }
            guard observation.state == .clear, !observation.clearance.isNaN,
                  observation.clearance >= Self.advanceDistance + 0.35,
                  control.clearance() >= Self.advanceDistance + 0.35 else { return .blocked }
            let capture = time - observation.sampleAge
            if capture > fence.acknowledgementUptime,
               last.map({ capture > $0 + 1e-4 }) ?? true {
                first = first ?? capture; last = capture; captures += 1
            }
            if captures >= 3, let first, capture - first >= 0.06 { return .clear }
            await control.sleep(.milliseconds(30))
        }
        return .failed(.depthUnavailable(.staleRawDepth))
    }

    private func advance() async -> NavigationResult {
        guard let start = advanceOrigin else { return .failed(.trackingLost) }
        var progress = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.03)
        let guardLayer = ObstacleGuard(stopDistance: FollowPlanningScope.minimumObstacleClearance ?? 0.45)
        var refreshUsed = false
        var refreshDeadline: Double?
        var refreshFence: FollowTurnStopFence?
        var refreshCaptures = 0
        var firstRefreshCapture: Double?
        var lastRefreshCapture: Double?
        while control.owned(), !Task.isCancelled, withinBounds() {
            if let refreshDeadline, control.uptime() >= refreshDeadline {
                return .failed(.depthUnavailable(.staleRawDepth))
            }
            let source = control.pose(), time = control.uptime()
            guard source.rejection(at: time, expectedGeneration: request.generation, requireEnriched: true) == nil,
                  let pose = source.pose, let sourceTime = source.sourceTimestamp else { return .failed(.trackingLost) }
            let delta = pose.position - start.position
            let distance = delta.x * cos(start.yaw) + delta.y * sin(start.yaw)
            if distance >= Self.advanceDistance - 0.02 { return .arrived }
            let ack = await control.acknowledgement()
            guard let ack else { return .failed(.commsLost) }
            let age = control.date().timeIntervalSince(ack)
            guard age.isFinite, age >= 0, age <= RoverConfig.commsWatchdogTimeout else { return .failed(.commsLost) }
            switch guardLayer.evaluate(forwardClearance: control.clearance(), lastAckAt: ack, now: control.date(), feedback: nil) {
            case .go: break
            case .stopObstacle: return .failed(.obstacle)
            case .stopCommsLost: return .failed(.commsLost)
            case .stopTipping: return .failed(.tipping)
            }
            let observedAt = control.uptime()
            let observation = control.depth(.init(left: 0.12, right: 0.12))
            guard observation.sampleAge.isFinite, observation.sampleAge >= 0 else { return .failed(.depthUnavailable(.staleRawDepth)) }
            let depthExpiry = observedAt + DepthSafetyEvaluator.maximumRawDepthAge - observation.sampleAge
            let capture = observedAt - observation.sampleAge
            let ageOnly = observation.state == .unavailable(.staleRawDepth) ||
                ((observation.state == .clear || observation.state == .caution) &&
                    control.uptime() + FollowPursuitDriver.minimumFreshnessMargin >= depthExpiry)
            let waitingForPostStopCapture = (observation.state == .clear || observation.state == .caution) &&
                (refreshFence.map { capture <= $0.acknowledgementUptime } ?? false)
            if ageOnly || waitingForPostStopCapture {
                refreshCaptures = 0
                firstRefreshCapture = nil
                lastRefreshCapture = nil
                if refreshDeadline == nil {
                    guard !refreshUsed else { return .failed(.depthUnavailable(.staleRawDepth)) }
                    refreshUsed = true
                    do { try await control.stop() } catch { return .failed(.commandFailed) }
                    guard let fence = stopFence(), fence.sourceGeneration == request.generation else {
                        return .failed(.commandFailed)
                    }
                    refreshFence = fence
                    refreshDeadline = min(request.deadline, control.uptime() + FollowPursuitDriver.depthRefreshLimit)
                    emit("depth_refresh_started")
                }
                await control.sleep(.milliseconds(20))
                continue
            }
            guard case .allow(let command, _) = guardLayer.evaluate(command: .init(left: 0.12, right: 0.12), depthSafety: observation) else {
                if case .unavailable(let reason) = observation.state { return .failed(.depthUnavailable(reason)) }
                return .failed(.obstacle)
            }
            if refreshDeadline != nil {
                if lastRefreshCapture.map({ capture > $0 + 1e-4 }) ?? true {
                    firstRefreshCapture = firstRefreshCapture ?? capture
                    lastRefreshCapture = capture
                    refreshCaptures += 1
                }
                guard refreshCaptures >= 3, let firstRefreshCapture, capture - firstRefreshCapture >= 0.06 else {
                    await control.sleep(.milliseconds(20))
                    continue
                }
            }
            let expiry = min(request.deadline, observedAt + DepthSafetyEvaluator.maximumRawDepthAge - observation.sampleAge,
                sourceTime + 0.5, control.uptime() + RoverConfig.commsWatchdogTimeout - control.date().timeIntervalSince(ack))
            guard control.owned(), withinBounds(), control.uptime() + 0.02 < expiry else { return .failed(.depthUnavailable(.staleRawDepth)) }
            if refreshDeadline != nil {
                // Only confirmed stationary time is excluded from the progress clock.
                progress = DriveProgressWatchdog(timeout: 2.5, minimumProgress: 0.03)
                refreshDeadline = nil
                refreshFence = nil
                emit("depth_refresh_recovered")
            }
            if progress.observe(distanceToGoal: Self.advanceDistance - distance, now: control.date(), commanded: true) { return .failed(.stalled) }
            control.moving()
            do { try await control.send(command, expiry) }
            catch { return .failed((error as? FollowPursuitSendFailure)?.reason ?? .commandFailed) }
            await control.sleep(.seconds(min(0.05, max(0, expiry - control.uptime()))))
        }
        return failure()
    }

    private func emit(_ stage: String, _ fields: [String: FollowDiagnosticValue] = [:]) {
        control.emit("obstacle_detour." + stage, fields.merging(["deadline_s": .number(request.deadline)]) { first, _ in first })
    }
}
