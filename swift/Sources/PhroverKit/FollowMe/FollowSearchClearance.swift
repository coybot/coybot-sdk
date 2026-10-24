import Foundation
import RoverNav

/// Shared model for the initial operator-confirmed, pure-rotation search.
/// Distances are outside the conservative full-rotation swept footprint, not
/// from the camera or chassis center. Camera pose is an observed bound, not a
/// calibrated chassis displacement or a guarantee of physical clearance.
enum FollowSearchClearancePolicy {
    static let initialClearance = 0.50
    static let retainedClearance = 0.30
    static let maximumObservedDisplacement = initialClearance - retainedClearance
    // Float camera transforms round 0.20 m upward by about 3 nm. This only
    // absorbs representation noise at the inclusive boundary, not pose drift.
    static let displacementComparisonTolerance = 1e-8
    static let lifetime: TimeInterval = 60

    static func sweptRadius(geometry: RoverCollisionGeometry, cellResolution: Double) -> Double {
        // Full chassis rotation plus lateral margin, then the voxel half-diagonal.
        hypot(geometry.length / 2, geometry.width / 2 + geometry.lateralSafetyMargin)
            + cellResolution / sqrt(2)
    }
}

/// An explicit operator assertion of 50 cm outside the full turning footprint.
/// Capture only when the operator confirms on the phone.
/// This assertion supplements unseen rotation sectors; it is not sensor evidence.
@MainActor
public final class FollowSearchClearanceConfirmation {
    fileprivate let grant: FollowSearchClearanceGrant
    private var consumed = false

    init(sourceGeneration: UInt64, origin: Vec2, confirmedAt: TimeInterval) {
        grant = FollowSearchClearanceGrant(sourceGeneration: sourceGeneration, origin: origin, confirmedAt: confirmedAt)
    }

    func consume(session: UInt64) -> FollowSearchClearanceGrant? {
        guard !consumed else { return nil }
        consumed = true
        grant.session = session
        return grant
    }
}

@MainActor
final class FollowSearchClearanceGrant: Equatable {
    nonisolated static func == (lhs: FollowSearchClearanceGrant, rhs: FollowSearchClearanceGrant) -> Bool { lhs === rhs }
    let sourceGeneration: UInt64
    let origin: Vec2
    let confirmedAt: TimeInterval
    var expiresAt: TimeInterval { confirmedAt + FollowSearchClearancePolicy.lifetime }
    fileprivate var session: UInt64?
    private(set) var revoked = false
    private(set) var revocationCause: FollowMotionFailureCause?
    private(set) var revocationDetail: String?
    private(set) var revokedAt: TimeInterval?
    private var observedDisplacement: Double?
    private var observedSample: NavigationPoseSample?
    private var observedAt: TimeInterval?
    var diagnosticFields: [String: FollowDiagnosticValue] {
        func number(_ value: Double?) -> FollowDiagnosticValue {
            value.flatMap { $0.isFinite ? .number($0) : nil } ?? .null
        }
        return [
            "search_clearance_reason": revocationCause.map { .string($0.rawValue) } ?? .null,
            "search_clearance_detail": revocationDetail.map { .string($0) } ?? .null,
            "search_clearance_confirmed_at_s": number(confirmedAt),
            "search_clearance_expires_at_s": number(expiresAt),
            "search_clearance_revoked_at_s": number(revokedAt),
            "search_clearance_observed_at_s": number(observedAt),
            "search_clearance_observed_displacement_m": number(observedDisplacement),
            "search_clearance_displacement_limit_m": .number(FollowSearchClearancePolicy.maximumObservedDisplacement),
            "search_clearance_retained_gap_m": .number(FollowSearchClearancePolicy.retainedClearance),
            "search_clearance_source_generation": .number(Double(sourceGeneration)),
            "search_clearance_observed_generation": observedSample?.frameID.map { .number(Double($0.generation)) } ?? .null,
            "search_clearance_observed_frame": observedSample?.frameID.map { .string("\($0.generation):\($0.sequence)") } ?? .null,
            "search_clearance_source_timestamp_s": number(observedSample?.sourceTimestamp)
        ]
    }
    var revocationObservers: [UUID: @MainActor () -> Void] = [:]

    init(sourceGeneration: UInt64, origin: Vec2, confirmedAt: TimeInterval) {
        self.sourceGeneration = sourceGeneration
        self.origin = origin
        self.confirmedAt = confirmedAt
    }

    func revoke(cause: FollowMotionFailureCause = .searchClearanceInvalidated,
                detail: String? = nil, now: TimeInterval = ProcessInfo.processInfo.systemUptime) {
        guard !revoked else { return }
        revocationCause = cause
        revocationDetail = detail
        revokedAt = now
        revoked = true
        for observer in Array(revocationObservers.values) { observer() }
    }

    /// Called synchronously by AR ingress from capture onward, including before
    /// the confirmation has been consumed by a follow session.
    func observeIngress(_ observation: PoseObservation, now: TimeInterval) {
        let quality: ARTrackingQuality = switch observation.trackingQuality {
        case .normal: .normal
        case .limited: .limited
        case .unavailable: .unavailable
        }
        _ = validateSource(sample: .init(pose: observation.pose,
            frameID: .init(generation: observation.sessionGeneration, sequence: observation.frameSequence),
            sourceTimestamp: observation.timestamp, trackingQuality: quality), now: now)
    }

    @discardableResult
    func validate(sample: NavigationPoseSample?, session: UInt64, now: TimeInterval) -> Bool {
        guard self.session == session else { revoke(cause: .searchClearanceInvalidated, detail: "session_mismatch", now: now); return false }
        return validateSource(sample: sample, now: now)
    }

    private func validateSource(sample: NavigationPoseSample?, now: TimeInterval) -> Bool {
        guard !revoked else { return false }
        observedSample = sample
        observedAt = now
        observedDisplacement = sample?.pose.map { hypot($0.position.x - origin.x, $0.position.y - origin.y) }
        guard confirmedAt.isFinite, now.isFinite, now >= confirmedAt,
              origin.x.isFinite, origin.y.isFinite else {
            revoke(cause: .searchClearanceInvalidated, detail: "invalid_confirmation", now: now); return false
        }
        guard now < expiresAt else {
            revoke(cause: .searchClearanceExpired, now: now); return false
        }
        if let rejection = sample?.rejection(at: now, expectedGeneration: sourceGeneration, requireEnriched: true) ?? (sample == nil ? "missing_source" : nil) {
            revoke(cause: .searchClearanceSourceInvalid, detail: rejection, now: now); return false
        }
        guard let observedDisplacement, observedDisplacement <= FollowSearchClearancePolicy.maximumObservedDisplacement + FollowSearchClearancePolicy.displacementComparisonTolerance else {
            revoke(cause: .searchClearanceSpaceExceeded, now: now); return false
        }
        return true
    }

}

/// Scoped only around the real depth provider after validating the request grant.
/// General mission depth evaluation never inherits this rotation permission.
enum FollowSearchRotationScope {
    @TaskLocal static var authorized = false
}

extension ARSessionManager {
    /// Returns nil when current AR provenance cannot bind an operator assertion.
    public func confirmInitialFollowSearchClearance() -> FollowSearchClearanceConfirmation? {
        let now = ProcessInfo.processInfo.systemUptime
        guard acceptsFollowSearchClearance, trackingQuality == .normal, let snapshot = latestSnapshot,
              snapshot.id.generation == sessionGeneration, snapshot.id.sequence == frameSequence else { return nil }
        let sample = NavigationPoseSample(snapshot: snapshot)
        guard sample.rejection(at: now, expectedGeneration: sessionGeneration, requireEnriched: true) == nil,
              let pose = sample.pose else { return nil }
        let confirmation = FollowSearchClearanceConfirmation(sourceGeneration: sessionGeneration,
            origin: pose.position, confirmedAt: now)
        registerFollowSearchClearance(confirmation.grant)
        return confirmation
    }
}
