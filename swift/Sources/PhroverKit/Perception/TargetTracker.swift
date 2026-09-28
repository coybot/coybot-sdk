import Foundation
import RoverNav

/// What the operator (or a brain) asked to follow, as free text — "me", "the person in the
/// hat", "the cart". Grounding quality is whatever the detector in use can deliver: the
/// bundled COCO `Detector` can only answer at class granularity, so an attribute in the
/// query narrows nothing on its own. It is still carried verbatim because a smarter
/// `RoverPerception.groundObject(query:)` (or a cloud VLM pointing at a pixel) can use it,
/// and because the rover speaks it back when confirming the lock.
public struct TargetSpec: Equatable, Sendable {
    public var query: String
    public init(query: String) {
        self.query = query
    }
}

/// One candidate sighting, already grounded to the nav plane by the caller.
///
/// The tracker takes world points rather than image points on purpose: unprojection needs
/// ARKit (or the sim's IPC), and keeping it on the caller's side of this boundary leaves
/// the association logic — the part that actually decides whether the rover keeps following
/// the right person — unit-testable with no ARKit, no CoreML and no sim.
public struct TargetObservation: Equatable, Sendable {
    public var label: String
    public var confidence: Float
    public var worldPoint: Vec2

    public init(label: String, confidence: Float, worldPoint: Vec2) {
        self.label = label
        self.confidence = confidence
        self.worldPoint = worldPoint
    }
}

/// Turns a stream of per-frame detections into one persistent target track.
///
/// The detector emits no track ids — the bundled `Detector` reports a bare COCO class per
/// frame — so identity has to be reconstructed here, by gating candidates against where the
/// track is predicted to be. That gate is the whole defence against following the wrong
/// person: without appearance re-identification, two people passing within the gate radius
/// are indistinguishable to this code. `unverifiedReacquisitions` counts every time the
/// lock was re-attached to a candidate the gate could not vouch for, so a caller (and a
/// test) can tell a clean follow from a lucky one.
@MainActor
public final class TargetTracker {
    public struct Params: Sendable {
        /// Candidates further than this from the predicted position are not the target (m).
        public var associationRadius: Double
        /// Smoothing on position updates — 1 trusts each measurement completely.
        public var positionAlpha: Double
        /// Smoothing on the velocity estimate. Low, because a single noisy unprojection
        /// otherwise throws the predicted position metres off.
        public var velocityAlpha: Double
        /// Largest velocity worth believing (m/s); beyond this the measurement is an
        /// association error or an unprojection glitch, not motion.
        public var maxSpeed: Double
        /// How long the lock coasts unseen before it will re-attach to an ungated candidate.
        public var reacquireAfter: TimeInterval
        /// How long the lock coasts unseen before it is declared lost.
        public var loseAfter: TimeInterval
        /// Detections below this confidence are ignored entirely.
        public var minimumConfidence: Float

        public init(associationRadius: Double = 0.8,
                    positionAlpha: Double = 0.7,
                    velocityAlpha: Double = 0.35,
                    maxSpeed: Double = 3.0,
                    reacquireAfter: TimeInterval = 1.5,
                    loseAfter: TimeInterval = 10,
                    minimumConfidence: Float = 0.5) {
            self.associationRadius = associationRadius
            self.positionAlpha = positionAlpha
            self.velocityAlpha = velocityAlpha
            self.maxSpeed = maxSpeed
            self.reacquireAfter = reacquireAfter
            self.loseAfter = loseAfter
            self.minimumConfidence = minimumConfidence
        }
    }

    public enum State: Equatable, Sendable {
        /// Locked to a spec, nothing matching seen yet.
        case acquiring
        /// Seen this tick (or recently enough to be trusted).
        case tracking
        /// Not seen this tick; the track is being predicted forward.
        case coasting
        /// Unseen past `loseAfter`. The last track is retained so the caller can search
        /// toward the last known bearing.
        case lost
        /// No spec locked.
        case idle
    }

    public private(set) var state: State = .idle
    public private(set) var spec: TargetSpec?
    public private(set) var track: TargetTrack?
    /// Times the lock re-attached to a candidate outside the association gate — i.e. the
    /// tracker assumed identity rather than establishing it.
    public private(set) var unverifiedReacquisitions = 0

    private let params: Params
    private var nextID = 1
    private var lastObservedAt: Date?
    private var seed: Vec2?

    public init(params: Params = Params()) {
        self.params = params
    }

    /// Begin following whatever matches `spec`. `seed` biases the initial pick toward a
    /// point the operator or a brain indicated (the person centred in frame, the pixel a
    /// VLM pointed at); without it, the highest-confidence match wins.
    public func lock(to spec: TargetSpec, seed: Vec2? = nil) {
        self.spec = spec
        self.seed = seed
        track = nil
        lastObservedAt = nil
        unverifiedReacquisitions = 0
        state = .acquiring
        RuntimeFileLog.append("follow_target_locked", fields: [
            "query": spec.query,
            "seed": seed.map { String(format: "%.2f,%.2f", $0.x, $0.y) } ?? "none"
        ])
    }

    public func release() {
        spec = nil
        seed = nil
        track = nil
        lastObservedAt = nil
        state = .idle
    }

    /// Fold this tick's detections into the track. Returns the current track, predicted
    /// forward when nothing was seen, or `nil` if nothing has ever been locked onto.
    @discardableResult
    public func update(observations: [TargetObservation], now: Date = Date()) -> TargetTrack? {
        guard let spec else { return nil }

        let candidates = observations.filter {
            $0.confidence >= params.minimumConfidence
                && MissionAgent.visualQueryMatches(query: spec.query, label: $0.label)
        }
        // Choosing *who* (the first pick, and any unvouched re-acquisition) goes by how
        // specifically the label fits the query — the one with the hat, not merely a
        // person. Staying on them once locked goes by position (the gate below), so a
        // detector that stops seeing the hat for a frame does not break the track.
        let specific = MissionAgent.mostSpecificMatches(candidates, query: spec.query, label: \.label)

        guard let existing = track else {
            if let picked = initialPick(from: specific) {
                track = TargetTrack(id: nextID,
                                    position: picked.worldPoint,
                                    velocity: .zero,
                                    lastSeenAt: now,
                                    confidence: Double(picked.confidence))
                nextID += 1
                lastObservedAt = now
                state = .tracking
                RuntimeFileLog.append("follow_target_acquired", fields: [
                    "label": picked.label,
                    "x": String(format: "%.2f", picked.worldPoint.x),
                    "y": String(format: "%.2f", picked.worldPoint.y)
                ])
            }
            return track
        }

        let unseenFor = now.timeIntervalSince(lastObservedAt ?? existing.lastSeenAt)
        let predicted = existing.predictedPosition(after: unseenFor)

        if let gated = nearest(to: predicted, among: candidates, within: params.associationRadius) {
            apply(gated, to: existing, now: now, unseenFor: unseenFor)
            return track
        }

        // Nothing inside the gate. While the lock is still fresh, coast on the prediction
        // rather than grabbing whatever else is in frame — that grab is exactly how a
        // follow silently transfers to a passer-by.
        if unseenFor >= params.reacquireAfter,
           let fallback = nearest(to: predicted, among: specific, within: .infinity) {
            unverifiedReacquisitions += 1
            RuntimeFileLog.append("follow_target_reacquired_unverified", fields: [
                "unseen_for": String(format: "%.2f", unseenFor),
                "distance_from_prediction": String(format: "%.2f",
                                                    fallback.worldPoint.distance(to: predicted)),
                "count": "\(unverifiedReacquisitions)"
            ])
            apply(fallback, to: existing, now: now, unseenFor: unseenFor)
            return track
        }

        state = unseenFor >= params.loseAfter ? .lost : .coasting
        return track
    }

    /// Seconds since the track was last actually observed (as opposed to predicted).
    public func secondsSinceSeen(now: Date = Date()) -> TimeInterval? {
        guard let lastObservedAt else { return nil }
        return now.timeIntervalSince(lastObservedAt)
    }

    // MARK: - Internals

    private func initialPick(from candidates: [TargetObservation]) -> TargetObservation? {
        guard !candidates.isEmpty else { return nil }
        if let seed {
            return candidates.min { $0.worldPoint.distance(to: seed) < $1.worldPoint.distance(to: seed) }
        }
        return candidates.max { $0.confidence < $1.confidence }
    }

    private func nearest(to point: Vec2,
                         among candidates: [TargetObservation],
                         within radius: Double) -> TargetObservation? {
        candidates
            .filter { $0.worldPoint.distance(to: point) <= radius }
            .min { $0.worldPoint.distance(to: point) < $1.worldPoint.distance(to: point) }
    }

    private func apply(_ observation: TargetObservation,
                       to existing: TargetTrack,
                       now: Date,
                       unseenFor: TimeInterval) {
        var updated = existing
        let position = Vec2(
            existing.position.x + (observation.worldPoint.x - existing.position.x) * params.positionAlpha,
            existing.position.y + (observation.worldPoint.y - existing.position.y) * params.positionAlpha
        )

        // Velocity only means anything across a real time gap; two detections in the same
        // millisecond would otherwise divide by ~0 and produce a nonsense prediction.
        if unseenFor > 0.02 {
            let measured = (position - existing.position) * (1 / unseenFor)
            var blended = Vec2(
                existing.velocity.x + (measured.x - existing.velocity.x) * params.velocityAlpha,
                existing.velocity.y + (measured.y - existing.velocity.y) * params.velocityAlpha
            )
            let speed = blended.length
            if speed > params.maxSpeed, speed > 0 {
                blended = blended * (params.maxSpeed / speed)
            }
            updated.velocity = blended
        }

        updated.position = position
        updated.lastSeenAt = now
        updated.confidence = Double(observation.confidence)
        track = updated
        lastObservedAt = now
        state = .tracking
    }
}
