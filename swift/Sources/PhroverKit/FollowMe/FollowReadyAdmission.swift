import Foundation

enum FollowMotionOutcome: Sendable, Equatable {
    case navigation(NavigationResult)
    case notStarted(FollowReadyDeferral)
    case scanDeferred(FollowScanDeferral)
}

enum FollowScanDeferral: String, Sendable {
    case newerFramePending = "newer_frame_pending"
    case insufficientFreshness = "insufficient_freshness_headroom"
    case perceptionUnavailable = "perception_unavailable"
    case ownershipChanged = "ownership_changed"
}

enum FollowReadyDeferral: String, Sendable {
    case clearance, heading, observation, ownership
}

enum FollowReadyAdmissionDecision {
    case accepted
    case deferred(FollowReadyDeferral)
    case staleObservation
}

enum FollowReadyAdmissionBoundary: String {
    case legacyCall = "legacy_signal_call", controllerFirstSend = "controller_first_send"
    case recoveryProbe = "recovery_probe"
    case recoveryProbeSend = "recovery_probe_send"
}

struct FollowReadyAdmissionToken: Equatable {
    let generation: UInt64
    let operation: UInt64
}

/// Synchronous MainActor authorization; the controller invokes it at first send.
@MainActor
final class FollowReadyAdmission {
    let allowsPathRecovery: Bool
    let authorize: @MainActor (NavigationPoseSample?, Double?) -> FollowReadyAdmissionDecision
    private(set) var deferred: FollowReadyDeferral?
    private(set) var boundary: FollowReadyAdmissionBoundary?
    private(set) var controllerSample: NavigationPoseSample?
    private(set) var controllerReadUptime: Double?
    var observationSnapshot: FollowAdmissionSnapshot?
    var rejectionCondition: String?
    var observationValidUntil: Double?
    private var recoveryRefreshStarted: Double?
    private var recoveryRefreshEnded = false
    static let recoveryRefreshLimit = 0.6
    var recoveryRefreshDeadline: Double? { recoveryRefreshStarted.map { $0 + Self.recoveryRefreshLimit } }

    /// Called only by the controller after a confirmed stop, before any probe.
    /// A single window belongs to this admission; it cannot be renewed.
    func beginRecoveryRefresh(at uptime: Double) {
        guard recoveryRefreshStarted == nil, !recoveryRefreshEnded, uptime.isFinite else { return }
        recoveryRefreshStarted = uptime
    }
    func isRefreshingRecovery(at uptime: Double) -> Bool {
        guard !recoveryRefreshEnded, let start = recoveryRefreshStarted else { return false }
        return uptime.isFinite && uptime >= start && uptime < start + Self.recoveryRefreshLimit
    }
    func endRecoveryRefresh() { recoveryRefreshEnded = true }
    init(_ authorize: @escaping @MainActor () -> FollowReadyAdmissionDecision) {
        allowsPathRecovery = false
        self.authorize = { _, _ in authorize() }
    }
    init(allowsPathRecovery: Bool = false,
         validating authorize: @escaping @MainActor (NavigationPoseSample?, Double?) -> FollowReadyAdmissionDecision) {
        self.allowsPathRecovery = allowsPathRecovery
        self.authorize = authorize
    }
    func admit(boundary: FollowReadyAdmissionBoundary = .legacyCall,
               sample: NavigationPoseSample? = nil, readUptime: Double? = nil,
               retryStaleObservation: Bool = false) -> Bool {
        self.boundary = boundary
        controllerSample = sample
        controllerReadUptime = readUptime
        switch authorize(sample, readUptime) {
        case .accepted: return true
        case .staleObservation:
            if retryStaleObservation, let readUptime, isRefreshingRecovery(at: readUptime) {
                return false
            }
            deferred = .observation
            return false
        case .deferred(let reason): deferred = reason; return false
        }
    }
}

enum FollowReadyAdmissionScope {
    @TaskLocal static var current: FollowReadyAdmission?
}
