import Foundation

enum FollowMotionOutcome: Sendable, Equatable {
    case navigation(NavigationResult)
    case notStarted(FollowReadyDeferral)
}

enum FollowReadyDeferral: String, Sendable {
    case clearance, heading, observation, ownership
}

enum FollowReadyAdmissionDecision {
    case accepted
    case deferred(FollowReadyDeferral)
}

enum FollowReadyAdmissionBoundary: String {
    case legacyCall = "legacy_signal_call", controllerFirstSend = "controller_first_send"
}

struct FollowReadyAdmissionToken: Equatable {
    let generation: UInt64
    let operation: UInt64
}

/// Synchronous MainActor authorization; the controller invokes it at first send.
@MainActor
final class FollowReadyAdmission {
    let authorize: @MainActor (NavigationPoseSample?, Double?) -> FollowReadyAdmissionDecision
    private(set) var deferred: FollowReadyDeferral?
    private(set) var boundary: FollowReadyAdmissionBoundary?
    private(set) var controllerSample: NavigationPoseSample?
    private(set) var controllerReadUptime: Double?
    var observationSnapshot: FollowAdmissionSnapshot?
    var rejectionCondition: String?
    init(_ authorize: @escaping @MainActor () -> FollowReadyAdmissionDecision) {
        self.authorize = { _, _ in authorize() }
    }
    init(validating authorize: @escaping @MainActor (NavigationPoseSample?, Double?) -> FollowReadyAdmissionDecision) {
        self.authorize = authorize
    }
    func admit(boundary: FollowReadyAdmissionBoundary = .legacyCall,
               sample: NavigationPoseSample? = nil, readUptime: Double? = nil) -> Bool {
        self.boundary = boundary
        controllerSample = sample
        controllerReadUptime = readUptime
        switch authorize(sample, readUptime) {
        case .accepted: return true
        case .deferred(let reason): deferred = reason; return false
        }
    }
}

enum FollowReadyAdmissionScope {
    @TaskLocal static var current: FollowReadyAdmission?
}
