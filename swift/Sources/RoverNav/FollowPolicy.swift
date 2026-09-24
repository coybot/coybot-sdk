import Foundation

/// A moving target being followed, as estimated by a tracker (see PhroverKit's
/// `TargetTracker`). Position/velocity are in the nav plane, same frame as `Pose2D`.
///
/// `id` is the *identity* of the thing being followed, not of a detection: it must survive
/// occlusion and re-acquisition, and a change of `id` mid-follow means the tracker swapped
/// targets — the failure mode vision-only following is most prone to, and the one the sim
/// tests assert against directly.
public struct TargetTrack: Equatable, Sendable {
    public var id: Int
    public var position: Vec2
    /// World-frame velocity (m/s). `.zero` for a track seen only once.
    public var velocity: Vec2
    public var lastSeenAt: Date
    /// Detector confidence of the most recent observation, 0...1.
    public var confidence: Double

    public init(id: Int,
                position: Vec2,
                velocity: Vec2 = .zero,
                lastSeenAt: Date = Date(),
                confidence: Double = 1.0) {
        self.id = id
        self.position = position
        self.velocity = velocity
        self.lastSeenAt = lastSeenAt
        self.confidence = confidence
    }

    /// Where the target is expected to be `seconds` from its last observation — used both
    /// to lead the goal point and to coast through a short detection dropout.
    public func predictedPosition(after seconds: TimeInterval) -> Vec2 {
        position + velocity * seconds
    }
}

/// The follow control law, shared across vehicle classes.
///
/// Deliberately pure and stateless apart from the deadband latch, so it unit-tests on plain
/// macOS alongside `PursuitController` / `AStarPlanner` with no ARKit, no sim and no I/O.
/// The surrounding loop (`PhroverKit.FollowController`) owns perception, actuation, safety
/// and the loss state machine; this type only answers "given where the target is and where
/// I am, what should I be doing right now".
///
/// Three geometries, keyed to the kinematics classes in `eco/drone/common/vehicle_class.py`
/// (never to a vehicle-type string — that file's own rule):
///   - `.unicycle`  — ground rover: hold a stand-off on the target bearing, turn in place
///                    when badly off-bearing.
///   - `.holonomic` — multirotor: same stand-off, but it can translate while yawed, so it
///                    never needs to turn in place first.
///   - `.orbit`     — fixed-wing: cannot hover, so "following" is a circle whose centre is
///                    slaved to the target's predicted position.
public struct FollowPolicy: Sendable {
    public enum Geometry: Sendable, Equatable {
        case unicycle
        case holonomic
        case orbit
    }

    public struct Params: Sendable, Equatable {
        public var geometry: Geometry
        /// Desired distance to hold from the target (m).
        public var standoff: Double
        /// Range error (m) below which translation stops. Must be < `deadbandExit`.
        public var deadbandEnter: Double
        /// Range error (m) above which translation resumes. The gap between the two is what
        /// stops the vehicle oscillating forward/back around the stand-off every tick — the
        /// single most important number in this type.
        public var deadbandExit: Double
        /// Bearing error (rad) inside which the target counts as centred.
        public var bearingTolerance: Double
        /// Bearing error (rad) above which a unicycle turns in place instead of arcing.
        public var rotateInPlaceAngle: Double
        /// Speed cap (m/s) — the vehicle class's own limit.
        public var maxSpeed: Double
        /// Commanded speed per metre of range error (1/s).
        public var speedGain: Double
        /// How far ahead of the last observation to predict the target (s). Also how long a
        /// dropout is coasted before the caller's loss machine takes over.
        public var leadTime: TimeInterval
        /// Age (s) beyond which the track is reported stale.
        public var maxTrackAge: TimeInterval
        /// Whether the vehicle may reverse to restore stand-off when the target closes in.
        public var allowsReverse: Bool
        /// `.orbit` geometry only: circle radius (m). Must be at or above the airframe's own
        /// minimum turn radius — see `search_patterns.turn_radius_m`.
        public var orbitRadius: Double

        public init(geometry: Geometry,
                    standoff: Double,
                    deadbandEnter: Double,
                    deadbandExit: Double,
                    bearingTolerance: Double = 0.15,
                    rotateInPlaceAngle: Double = 0.7,
                    maxSpeed: Double,
                    speedGain: Double = 0.5,
                    leadTime: TimeInterval = 0.5,
                    maxTrackAge: TimeInterval = 1.0,
                    allowsReverse: Bool = false,
                    orbitRadius: Double = 0) {
            self.geometry = geometry
            self.standoff = standoff
            self.deadbandEnter = deadbandEnter
            self.deadbandExit = deadbandExit
            self.bearingTolerance = bearingTolerance
            self.rotateInPlaceAngle = rotateInPlaceAngle
            self.maxSpeed = maxSpeed
            self.speedGain = speedGain
            self.leadTime = leadTime
            self.maxTrackAge = maxTrackAge
            self.allowsReverse = allowsReverse
            self.orbitRadius = orbitRadius
        }
    }

    public enum Motion: Equatable, Sendable {
        /// In the band and centred — send no motion at all.
        case hold
        /// Centre the target without translating. `yawError` is CCW-positive radians.
        case turnInPlace(yawError: Double)
        /// Drive toward `goal` at `speed`, closing the stand-off.
        case advance(goal: Vec2, speed: Double)
        /// Restore stand-off by reversing (only when `allowsReverse`).
        case retreat(goal: Vec2, speed: Double)
        /// Circle `center` at `radius`, sensor held on the centre.
        case orbit(center: Vec2, radius: Double)
    }

    public struct Output: Equatable, Sendable {
        public var motion: Motion
        /// Signed range error: positive = too far away, negative = too close.
        public var rangeError: Double
        /// Signed bearing error to the target, CCW-positive radians.
        public var bearingError: Double
        /// The target's predicted position at `leadTime` past its last sighting.
        public var predictedTarget: Vec2
        /// The track has not been observed within `maxTrackAge` — the caller's loss state
        /// machine should take over. Motion is still reported (an aircraft cannot stop, and
        /// a ground vehicle coasting one tick on a fresh prediction is better than a jerk).
        public var isStale: Bool
    }

    /// Below this closing speed a target counts as stationary, and a vehicle in the band
    /// stops rather than creeping after sensor noise.
    static let pacingThreshold = 0.05

    public private(set) var params: Params
    /// Deadband latch. Mutated by `step`, exactly like `DriveProgressWatchdog.observe`.
    private var isHolding = false

    public init(params: Params) {
        self.params = params
    }

    /// Current stand-off band state — exposed for logging and for tests asserting that the
    /// latch is what suppresses oscillation, rather than luck in the sampling.
    public var isInBand: Bool { isHolding }

    public mutating func step(pose: Pose2D, track: TargetTrack, now: Date = Date()) -> Output {
        let sinceSeen = now.timeIntervalSince(track.lastSeenAt)
        let predicted = track.predictedPosition(after: sinceSeen + params.leadTime)
        let toTarget = predicted - pose.position
        let range = toTarget.length
        let rangeError = range - params.standoff
        // A target sitting exactly on the vehicle has no meaningful bearing; hold the
        // current heading rather than spinning on numerical noise.
        let bearingError = range > 1e-3
            ? normalizeAngle(atan2(toTarget.y, toTarget.x) - pose.yaw)
            : 0
        let isStale = sinceSeen > params.maxTrackAge

        updateBandLatch(rangeError: rangeError)

        let lineOfSight = range > 1e-3
            ? Vec2(toTarget.x / range, toTarget.y / range)
            : pose.forward
        let motion = self.motion(pose: pose,
                                 predicted: predicted,
                                 track: track,
                                 lineOfSight: lineOfSight,
                                 rangeError: rangeError,
                                 bearingError: bearingError)
        return Output(motion: motion,
                      rangeError: rangeError,
                      bearingError: bearingError,
                      predictedTarget: predicted,
                      isStale: isStale)
    }

    /// Drop the latch — call when the track is lost or re-acquired so the next follow leg
    /// starts from a clean band state rather than inheriting the last one.
    public mutating func reset() {
        isHolding = false
    }

    // MARK: - Internals

    private mutating func updateBandLatch(rangeError: Double) {
        let error = abs(rangeError)
        if isHolding {
            if error >= params.deadbandExit { isHolding = false }
        } else {
            if error <= params.deadbandEnter { isHolding = true }
        }
    }

    /// Proportional correction PLUS the target's own speed along the line of sight.
    ///
    /// Without the feedforward term a pure P-controller cannot hold a stand-off behind
    /// anything that keeps moving: it settles wherever `gain * error` happens to equal the
    /// target's speed, which for a rover behind a 0.4 m/s walk is about 0.8 m too far back
    /// — measured in the Godot depot before this term existed. Only the receding component
    /// is fed forward; a target closing in is handled by the deadband and, for a vehicle
    /// that cannot reverse, by stopping.
    private func speed(forRangeError rangeError: Double,
                       track: TargetTrack,
                       lineOfSight: Vec2) -> Double {
        let closing = track.velocity.x * lineOfSight.x + track.velocity.y * lineOfSight.y
        return min(params.maxSpeed, abs(rangeError) * params.speedGain + max(0, closing))
    }

    private func motion(pose: Pose2D,
                        predicted: Vec2,
                        track: TargetTrack,
                        lineOfSight: Vec2,
                        rangeError: Double,
                        bearingError: Double) -> Motion {
        // A fixed-wing has no hold and no reverse: min airspeed is well above zero, so the
        // only way to "stay with" a target is to keep circling it. Band state is irrelevant.
        if params.geometry == .orbit {
            return .orbit(center: predicted, radius: params.orbitRadius)
        }

        if isHolding {
            // In the band, the deadband suppresses the range CORRECTION — not motion
            // itself. A vehicle that stops dead behind a target that keeps walking has to
            // start again the moment the gap reopens, which is stick-slip: measured at 36
            // start/stop cycles a minute behind a steady walk. Pacing the target at its
            // own speed holds the stand-off without hunting for it, and still comes to a
            // genuine stop the moment the target does.
            let closing = track.velocity.x * lineOfSight.x + track.velocity.y * lineOfSight.y
            if closing > Self.pacingThreshold {
                return .advance(goal: predicted - lineOfSight * params.standoff,
                                speed: min(params.maxSpeed, closing))
            }
            return abs(bearingError) > params.bearingTolerance
                ? .turnInPlace(yawError: bearingError)
                : .hold
        }

        let speed = self.speed(forRangeError: rangeError, track: track, lineOfSight: lineOfSight)

        if rangeError < 0 {
            // Target has closed inside the stand-off. A ground rover does NOT back away:
            // phrover_manager.gd's person governor documents three rejected retreat/dodge
            // designs and settles on a plain stop as the only provably safe response — a
            // reversing rover with no rear sensing loses that tail-chase anyway. A
            // multirotor can hold station off-axis, so it is allowed to restore stand-off.
            guard params.allowsReverse else {
                return abs(bearingError) > params.bearingTolerance
                    ? .turnInPlace(yawError: bearingError)
                    : .hold
            }
            let away = Vec2(-lineOfSight.x, -lineOfSight.y)
            return .retreat(goal: predicted + away * params.standoff, speed: speed)
        }

        // Unicycle: arcing toward a target that is far off-boresight wastes ground and can
        // lose it out of a narrow camera FOV. Square up first.
        if params.geometry == .unicycle, abs(bearingError) > params.rotateInPlaceAngle {
            return .turnInPlace(yawError: bearingError)
        }

        return .advance(goal: predicted - lineOfSight * params.standoff, speed: speed)
    }
}

// MARK: - Per-class defaults

extension FollowPolicy.Params {
    /// WAVE ROVER / phrover. `maxSpeed` matches `PursuitController.Params.maxLinear`; note
    /// this is well below walking pace (0.8-1.3 m/s), so the rover holds station with
    /// someone deliberately walking slowly and falls behind anyone else. Stand-off is set
    /// above the Godot depot's person governor release distance (`PERSON_SAFE_DIST`, 2.4 m)
    /// so following does not sit permanently inside a guard the brain cannot override.
    public static func ground(standoff: Double = 2.5, maxSpeed: Double = 0.35) -> Self {
        .init(geometry: .unicycle,
              standoff: standoff,
              deadbandEnter: 0.25,
              deadbandExit: 0.60,
              maxSpeed: maxSpeed,
              speedGain: 0.5,
              leadTime: 0.5,
              maxTrackAge: 1.0,
              allowsReverse: false)
    }

    /// Multirotor (quadcopter, crazyflie): can hover and translate while yawed, and at
    /// 3.0 m/s comfortably outruns a walking person.
    public static func multirotor(standoff: Double = 5.0, maxSpeed: Double = 3.0) -> Self {
        .init(geometry: .holonomic,
              standoff: standoff,
              deadbandEnter: 0.5,
              deadbandExit: 1.5,
              maxSpeed: maxSpeed,
              speedGain: 0.6,
              leadTime: 0.8,
              maxTrackAge: 1.5,
              allowsReverse: true)
    }

    /// Fixed-wing: `orbitRadius` must come from the airframe's own minimum turn radius
    /// (`search_patterns.turn_radius_m` — roughly max_speed_mps / max_yaw_rate_radps, ~42 m
    /// for the shipped FIXEDWING class). A tighter circle is not suboptimal, it is
    /// unflyable. `maxTrackAge` is generous because the aircraft keeps orbiting the last
    /// estimate regardless; losing the target does not make it stop.
    public static func fixedWing(orbitRadius: Double, maxSpeed: Double = 25.0) -> Self {
        .init(geometry: .orbit,
              standoff: orbitRadius,
              deadbandEnter: 0,
              deadbandExit: .infinity,
              maxSpeed: maxSpeed,
              speedGain: 1.0,
              leadTime: 2.0,
              maxTrackAge: 5.0,
              allowsReverse: false,
              orbitRadius: orbitRadius)
    }
}
