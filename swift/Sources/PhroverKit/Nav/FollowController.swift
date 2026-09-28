import Foundation
import RoverNav

/// The actuation seam `FollowController` drives, in vehicle-class-neutral unicycle terms
/// (forward m/s + yaw rate rad/s) rather than wheel speeds.
///
/// Exists so one follow loop serves both the real chassis and the Godot sim: on device an
/// adapter converts to `RoverControl`'s wheel command, and the sim's own `phrover_drive`
/// op already speaks `{v, w}` directly. `NavigationController` does not need this because
/// its unit of work is a whole goal-to-goal drive; following is a per-tick servo, so the
/// substitutable thing is the command sink, not the trip.
@MainActor
public protocol RoverDrive: AnyObject {
    func send(v: Double, w: Double) async throws
    func stop() async throws
    /// Nearest obstacle distance in the forward cone (m); `.infinity` when unknown.
    var forwardClearance: Double { get }
    /// Timestamp of the last successful command round-trip, for the comms watchdog.
    func lastAck() async -> Date?
}

/// Drives the real WAVE ROVER: ARKit supplies clearance, `RoverControl` the link.
@MainActor
public final class LocalRoverDrive: RoverDrive {
    private let ar: ARSessionManager
    private let control: RoverControl
    private let wheelBase: Double
    private let maxWheelSpeed: Double

    public init(ar: ARSessionManager,
                control: RoverControl,
                wheelBase: Double = RoverConfig.wheelBase,
                maxWheelSpeed: Double = RoverConfig.maxWheelSpeed) {
        self.ar = ar
        self.control = control
        self.wheelBase = wheelBase
        self.maxWheelSpeed = maxWheelSpeed
    }

    public var forwardClearance: Double { ar.forwardClearance }

    public func send(v: Double, w: Double) async throws {
        let wheels = DifferentialDrive.wheels(v: v, w: w,
                                              wheelBase: wheelBase,
                                              maxWheelSpeed: maxWheelSpeed)
        try await control.sendNavigation(wheels)
    }

    public func stop() async throws {
        try await control.stop()
    }

    public func lastAck() async -> Date? {
        await control.lastAckAt
    }
}

/// Follows a moving target: tracker → `FollowPolicy` → drive, with its own safety gate and
/// loss state machine.
///
/// A sibling of `NavigationController`, not an extension of it. Three of that class's
/// mechanisms are built on the goal standing still and all three are load-bearing for
/// ordinary navigation: `DriveProgressWatchdog` fails a drive whose distance to goal stops
/// shrinking (which is what a correctly held stand-off looks like), `PursuitController`'s
/// `reachedGoal` ends the trip on arrival (following never arrives), and
/// `visualTargetApproachDecision` latches `.arrived` at a stand-off. What is reused instead
/// is everything below the loop: `FollowPolicy`, `PursuitController`, `ObstacleGuard`,
/// `RotationCommand`.
@Observable
@MainActor
public final class FollowController {
    public enum State: Equatable, Sendable {
        case idle
        /// Locked to a spec, nothing matching seen yet.
        case acquiring
        case following
        /// Target lost; turning toward its last known bearing to re-find it.
        case searching
        /// Gave up. Carries the reason for the operator-facing message.
        case ended(String)
    }

    public private(set) var state: State = .idle
    public private(set) var track: TargetTrack?
    /// Latest policy output, for UI and logging.
    public private(set) var rangeError: Double = 0
    /// Times the lock re-attached to a candidate the association gate could not vouch for.
    /// Non-zero means the rover may be following someone else — surfaced rather than
    /// hidden, because without appearance re-identification nothing downstream can tell.
    public var unverifiedReacquisitions: Int { tracker.unverifiedReacquisitions }

    private let perception: RoverPerception
    private let drive: RoverDrive
    private let tracker: TargetTracker
    private var policy: FollowPolicy
    private let guardLayer: ObstacleGuard
    private let pursuit: PursuitController
    private let maxDuration: TimeInterval
    private let acquireTimeout: TimeInterval
    private let tickInterval: TimeInterval

    private var loop: Task<Void, Never>?

    public init(perception: RoverPerception,
                drive: RoverDrive,
                policy: FollowPolicy.Params = .ground(),
                tracker: TargetTracker = TargetTracker(),
                maxDuration: TimeInterval = RoverConfig.maxFollowDuration,
                acquireTimeout: TimeInterval = RoverConfig.followAcquireTimeout,
                tickInterval: TimeInterval = RoverConfig.commandInterval) {
        self.perception = perception
        self.drive = drive
        self.tracker = tracker
        self.policy = FollowPolicy(params: policy)
        self.guardLayer = ObstacleGuard()
        self.pursuit = PursuitController(params: .init(
            maxLinear: policy.maxSpeed,
            wheelBase: RoverConfig.wheelBase,
            goalTolerance: 0.05,
            minimumRotateWheelSpeed: RoverConfig.minimumRotateWheelSpeed))
        self.maxDuration = maxDuration
        self.acquireTimeout = acquireTimeout
        self.tickInterval = tickInterval
    }

    /// Start following whatever matches `spec`. Returns immediately — following is a mode,
    /// not a trip, so the caller keeps running while this loop holds station.
    public func follow(_ spec: TargetSpec, seed: Vec2? = nil) {
        cancel()
        tracker.lock(to: spec, seed: seed)
        policy.reset()
        state = .acquiring
        RuntimeFileLog.append("follow_started", fields: [
            "query": spec.query,
            "standoff": String(format: "%.2f", policy.params.standoff),
            "max_duration": String(format: "%.0f", maxDuration)
        ])
        loop = Task { await run() }
    }

    /// What is being followed right now, or `nil` if nothing is.
    public var followingSpec: TargetSpec? { tracker.spec }

    /// `RoverFollowing` spelling of `cancel()`.
    public func stopFollowing() { cancel() }

    /// Stop following and release the lock.
    public func cancel() {
        loop?.cancel()
        loop = nil
        tracker.release()
        track = nil
        Task { try? await drive.stop() }
        state = .idle
    }

    // MARK: - Loop

    private func run() async {
        let startedAt = Date()
        var hasSentCommand = false

        while !Task.isCancelled {
            let now = Date()
            if now.timeIntervalSince(startedAt) > maxDuration {
                await end("I've been following for a while — stopping there.", reason: "time_budget")
                return
            }
            guard let pose = perception.pose else {
                await end("I lost my bearings.", reason: "no_pose")
                return
            }

            tracker.update(observations: observations(), now: now)
            track = tracker.track

            switch tracker.state {
            case .acquiring:
                if now.timeIntervalSince(startedAt) > acquireTimeout {
                    await end("I couldn't find them to follow.", reason: "never_acquired")
                    return
                }
                try? await drive.stop()
                try? await Task.sleep(for: .seconds(tickInterval))
                continue
            case .lost:
                if await search(from: pose) { continue }
                await end("I lost you — say follow me again.", reason: "target_lost")
                return
            case .idle:
                await end("Nothing to follow.", reason: "no_spec")
                return
            case .tracking, .coasting:
                break
            }

            guard let current = tracker.track else { continue }
            state = tracker.state == .coasting ? .searching : .following

            let out = policy.step(pose: pose, track: current, now: now)
            rangeError = out.rangeError

            let distanceToTarget = pose.position.distance(to: out.predictedTarget)
            let lastAck = await drive.lastAck()
            let decision = guardLayer.evaluate(
                forwardClearance: drive.forwardClearance,
                lastAckAt: lastAck,
                now: now,
                feedback: nil,
                requireFreshAck: hasSentCommand,
                checkForwardObstacle: !Self.clearanceIsTheTarget(
                    forwardClearance: drive.forwardClearance,
                    distanceToTarget: distanceToTarget))

            switch decision {
            case .go:
                break
            case .stopObstacle(let clearance):
                await end(String(format: "Something's in the way — %.1f m ahead.", clearance),
                          reason: "obstacle")
                return
            case .stopCommsLost:
                await end("Rover command link lost.", reason: "comms_lost")
                return
            case .stopTipping:
                await end("Rover may be tipping.", reason: "tipping")
                return
            }

            let command = Self.unicycleCommand(for: out.motion, pose: pose, pursuit: pursuit)
            RuntimeFileLog.append("follow_tick", fields: [
                "state": state.description,
                "track_id": "\(current.id)",
                "range_error": String(format: "%.2f", out.rangeError),
                "bearing_error_deg": String(format: "%.0f", out.bearingError * 180 / .pi),
                "stale": out.isStale ? "true" : "false",
                "clearance": drive.forwardClearance.isFinite
                    ? String(format: "%.2f", drive.forwardClearance) : "inf",
                "v": String(format: "%.2f", command.v),
                "w": String(format: "%.2f", command.w)
            ])

            do {
                if command.v == 0 && command.w == 0 {
                    try await drive.stop()
                } else {
                    try await drive.send(v: command.v, w: command.w)
                }
                hasSentCommand = true
            } catch {
                await end("Rover command failed: \(error.localizedDescription)",
                          reason: "command_failed")
                return
            }

            try? await Task.sleep(for: .seconds(tickInterval))
        }
        try? await drive.stop()
    }

    /// Turn toward the target's last known bearing in scan pulses, the same pulse-and-settle
    /// cadence `NavigationController.rotateForScan` uses — spinning continuously sweeps past
    /// the target before detection gets a stable frame. Returns true if the target came back.
    private func search(from pose: Pose2D) async -> Bool {
        guard let last = tracker.track else { return false }
        state = .searching
        let bearing = normalizeAngle(
            atan2(last.position.y - pose.position.y, last.position.x - pose.position.x) - pose.yaw)
        RuntimeFileLog.append("follow_search", fields: [
            "bearing_deg": String(format: "%.0f", bearing * 180 / .pi)
        ])

        // Same proportional rotation the nav loop's scan turns use, so a search pulse
        // here behaves identically to one there — including the minimum wheel speed that
        // overcomes the chassis's static friction.
        let turn = Self.unicycleCommand(for: .turnInPlace(yawError: bearing),
                                        pose: pose,
                                        pursuit: pursuit)
        for _ in 0..<RoverConfig.followSearchPulses {
            if Task.isCancelled { return false }
            try? await drive.send(v: 0, w: turn.w)
            try? await Task.sleep(for: .seconds(RoverConfig.scanTurnPulseDuration))
            try? await drive.stop()
            try? await Task.sleep(for: .seconds(RoverConfig.scanTurnSettleDuration))
            tracker.update(observations: observations())
            if tracker.state == .tracking {
                track = tracker.track
                policy.reset()
                RuntimeFileLog.append("follow_reacquired", fields: [:])
                return true
            }
        }
        return false
    }

    private func observations() -> [TargetObservation] {
        perception.detectObjects().compactMap { object in
            guard let world = perception.groundPoint(of: object) else {
                return nil
            }
            return TargetObservation(label: object.label,
                                     confidence: object.confidence,
                                     worldPoint: world)
        }
    }

    private func end(_ message: String, reason: String) async {
        try? await drive.stop()
        tracker.release()
        state = .ended(message)
        RuntimeFileLog.append("follow_ended", fields: ["reason": reason, "message": message])
    }

    // MARK: - Pure helpers

    /// Whether the forward-clearance reading is explained by the target being followed.
    ///
    /// The person you are following *is* the forward obstacle, so `ObstacleGuard`'s hard
    /// stop has to be relaxed — but only for them. If the cone reads something at a range
    /// the target does not account for (a cart swinging in, a wall), the stop must still
    /// fire. `NavigationController` makes the same trade for a locked visual target; this
    /// is the moving-target version of it, and it is deliberately a range agreement rather
    /// than a blanket suppression.
    static func clearanceIsTheTarget(forwardClearance: Double,
                                     distanceToTarget: Double,
                                     tolerance: Double = RoverConfig.followClearanceMatchTolerance) -> Bool {
        guard forwardClearance.isFinite else { return false }
        return abs(forwardClearance - distanceToTarget) <= tolerance
    }

    /// Convert a policy motion into a unicycle command, reusing `PursuitController` for the
    /// translating cases so path following stays a single implementation.
    static func unicycleCommand(for motion: FollowPolicy.Motion,
                                pose: Pose2D,
                                pursuit: PursuitController) -> (v: Double, w: Double) {
        switch motion {
        case .hold:
            return (0, 0)

        case .turnInPlace(let yawError):
            let command = RotationCommand.command(forYawError: yawError)
            return wheelsToUnicycle(command)

        case .advance(let goal, let speed):
            let out = pursuit.step(pose: pose, path: [pose.position, goal])
            var command = wheelsToUnicycle(out.command)
            command.v = min(command.v, speed)
            return command

        case .retreat(let goal, let speed):
            // Only reachable for classes that allow reverse; a ground rover's policy does
            // not, because it has no rear sensing (see FollowPolicy).
            let heading = atan2(goal.y - pose.position.y, goal.x - pose.position.x)
            let error = normalizeAngle(heading - pose.yaw)
            return (-min(speed, RoverConfig.maxWheelSpeed), error * 0.5)

        case .orbit:
            // Fixed-wing geometry; a ground rover has no use for it. The aerial stacks
            // implement orbit through their own loiter primitive.
            return (0, 0)
        }
    }

    private static func wheelsToUnicycle(_ command: WheelCommand) -> (v: Double, w: Double) {
        (v: (command.left + command.right) / 2,
         w: (command.right - command.left) / RoverConfig.wheelBase)
    }
}

extension FollowController: RoverFollowing {
    public var followState: State { state }
}

extension FollowController.State: CustomStringConvertible {
    public var description: String {
        switch self {
        case .idle: return "idle"
        case .acquiring: return "acquiring"
        case .following: return "following"
        case .searching: return "searching"
        case .ended(let reason): return "ended: \(reason)"
        }
    }
}
