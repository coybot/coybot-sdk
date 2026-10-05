import Foundation

struct RelativeHeadingSample: Equatable, Sendable {
    let timestamp: TimeInterval
    let rotationRate: SIMD3<Double>
    let gravity: SIMD3<Double>
}

enum RelativeHeadingReliability: Equatable, Sendable {
    enum UnreliableReason: String, Equatable, Sendable {
        case notStarted = "not_started"
        case nonFiniteSample = "non_finite_sample"
        case invalidGravity = "invalid_gravity"
        case nonMonotonicTimestamp = "non_monotonic_timestamp"
        case sampleGap = "sample_gap"
        case staleSample = "stale_sample"
        case sessionGenerationChanged = "session_generation_changed"
        case trackingInterrupted = "tracking_interrupted"
    }

    case reliable
    case unreliable(UnreliableReason)
}

struct RelativeHeadingMeasurement: Equatable, Sendable {
    let accumulatedAngle: Double
    let sampleAge: TimeInterval?
    let reliability: RelativeHeadingReliability
}

struct RelativeHeadingTracker: Sendable {
    private let maximumIntegrationGap: TimeInterval
    private let maximumSampleAge: TimeInterval
    private let acceptedGravityMagnitude: ClosedRange<Double>

    private(set) var accumulatedAngle = 0.0
    private var previousSample: RelativeHeadingSample?
    private var previousRate: Double?
    private var reliability: RelativeHeadingReliability = .unreliable(.notStarted)

    init(
        maximumIntegrationGap: TimeInterval = RoverConfig.relativeHeadingMaximumIntegrationGap,
        maximumSampleAge: TimeInterval = RoverConfig.relativeHeadingMaximumSampleAge,
        acceptedGravityMagnitude: ClosedRange<Double> = RoverConfig.relativeHeadingAcceptedGravityMagnitude
    ) {
        self.maximumIntegrationGap = maximumIntegrationGap
        self.maximumSampleAge = maximumSampleAge
        self.acceptedGravityMagnitude = acceptedGravityMagnitude
    }

    mutating func reset() {
        accumulatedAngle = 0
        previousSample = nil
        previousRate = nil
        reliability = .unreliable(.notStarted)
    }

    mutating func invalidate(reason: RelativeHeadingReliability.UnreliableReason) {
        previousSample = nil
        previousRate = nil
        reliability = .unreliable(reason)
    }

    @discardableResult
    mutating func ingest(_ sample: RelativeHeadingSample) -> Bool {
        if case .unreliable(let reason) = reliability, reason != .notStarted {
            return false
        }
        guard isFinite(sample) else {
            reliability = .unreliable(.nonFiniteSample)
            return false
        }

        let gravityMagnitude = magnitude(sample.gravity)
        guard acceptedGravityMagnitude.contains(gravityMagnitude) else {
            reliability = .unreliable(.invalidGravity)
            return false
        }

        if let previousSample {
            let interval = sample.timestamp - previousSample.timestamp
            guard interval > 0 else {
                reliability = .unreliable(.nonMonotonicTimestamp)
                return false
            }
            guard interval <= maximumIntegrationGap + 1e-9 else {
                reliability = .unreliable(.sampleGap)
                return false
            }
        }

        let normalizedGravity = sample.gravity / gravityMagnitude
        let rate = dot(sample.rotationRate, normalizedGravity)
        guard rate.isFinite else {
            reliability = .unreliable(.nonFiniteSample)
            return false
        }

        if let previousSample, let previousRate {
            accumulatedAngle += (previousRate + rate) * 0.5
                * (sample.timestamp - previousSample.timestamp)
        }
        self.previousSample = sample
        self.previousRate = rate
        reliability = .reliable
        return true
    }

    func measurement(at timestamp: TimeInterval) -> RelativeHeadingMeasurement {
        guard let previousSample else {
            return RelativeHeadingMeasurement(
                accumulatedAngle: accumulatedAngle,
                sampleAge: nil,
                reliability: reliability
            )
        }
        let age = max(0, timestamp - previousSample.timestamp)
        let currentReliability: RelativeHeadingReliability = age <= maximumSampleAge
            ? reliability
            : .unreliable(.staleSample)
        return RelativeHeadingMeasurement(
            accumulatedAngle: accumulatedAngle,
            sampleAge: age,
            reliability: currentReliability
        )
    }

    private func isFinite(_ sample: RelativeHeadingSample) -> Bool {
        sample.timestamp.isFinite
            && sample.rotationRate.x.isFinite
            && sample.rotationRate.y.isFinite
            && sample.rotationRate.z.isFinite
            && sample.gravity.x.isFinite
            && sample.gravity.y.isFinite
            && sample.gravity.z.isFinite
    }

    private func magnitude(_ vector: SIMD3<Double>) -> Double {
        sqrt(dot(vector, vector))
    }

    private func dot(_ lhs: SIMD3<Double>, _ rhs: SIMD3<Double>) -> Double {
        lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z
    }
}

struct RelativeHeadingIngestResult: Sendable {
    let accepted: Bool
    let measurement: RelativeHeadingMeasurement
}

struct RelativeHeadingInvalidationResult: Sendable {
    let wasActive: Bool
    let measurement: RelativeHeadingMeasurement
}

/// Keeps Core Motion ingestion independent of UI, ARKit, and telemetry work on the main actor.
final class RelativeHeadingTrackerStore: @unchecked Sendable {
    private let lock = NSLock()
    private var tracker = RelativeHeadingTracker()
    private var isMeasurementActive = false

    func beginMeasurement() {
        lock.lock()
        defer { lock.unlock() }
        tracker.reset()
        isMeasurementActive = true
    }

    func endMeasurement() {
        lock.lock()
        defer { lock.unlock() }
        isMeasurementActive = false
        tracker.reset()
    }

    func invalidate(
        reason: RelativeHeadingReliability.UnreliableReason,
        at timestamp: TimeInterval
    ) -> RelativeHeadingInvalidationResult {
        lock.lock()
        defer { lock.unlock() }
        let wasActive = isMeasurementActive
        isMeasurementActive = false
        if wasActive {
            tracker.invalidate(reason: reason)
        } else {
            tracker.reset()
        }
        return RelativeHeadingInvalidationResult(
            wasActive: wasActive,
            measurement: tracker.measurement(at: timestamp)
        )
    }

    func ingest(_ sample: RelativeHeadingSample) -> RelativeHeadingIngestResult? {
        lock.lock()
        defer { lock.unlock() }
        guard isMeasurementActive else { return nil }
        let accepted = tracker.ingest(sample)
        return RelativeHeadingIngestResult(
            accepted: accepted,
            measurement: tracker.measurement(at: sample.timestamp)
        )
    }

    func measurement(at timestamp: TimeInterval) -> RelativeHeadingMeasurement {
        lock.lock()
        defer { lock.unlock() }
        return tracker.measurement(at: timestamp)
    }
}
