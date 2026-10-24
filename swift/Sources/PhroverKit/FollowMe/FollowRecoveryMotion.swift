import Foundation

/// Synchronous episode authorization, also carried through incomplete alignment/readiness.
struct FollowRecoveryAuthorization: Sendable {
    let episodeID: UUID
    let expectedGeneration: UInt64
    let deadline: TimeInterval
    let now: @MainActor @Sendable () -> TimeInterval
    let canContinue: @MainActor @Sendable () -> Bool
    @MainActor var authorized: Bool {
        authorized(at: now())
    }
    @MainActor func authorized(at time: Double) -> Bool {
        return time.isFinite && deadline.isFinite && time < deadline && !Task.isCancelled && canContinue()
    }
}

struct FollowRecoveryHeadingRequest: Sendable {
    let stageHeading: Double
    let authorization: FollowRecoveryAuthorization
    /// Correlation only; the coordinator still owns cursor advancement.
    var stageIndex: Int? = nil
    var segmentIndex: Int? = nil
    var maximumSegment: Double = .pi / 6
}

struct FollowRecoverySegmentEvidence: Sendable {
    let postStopSource: NavigationPoseSample
    let resolutionSource: NavigationPoseSample?
    let stageHeading: Double
    let segmentHeading: Double?
    let requestedDelta: Double?
    let arrivalSource: NavigationPoseSample?
    let segmentArrived: Bool
    let stageArrived: Bool?
    var postStopReadUptime: Double? = nil
    var resolutionReadUptime: Double? = nil
    var arrivalReadUptime: Double? = nil
}

enum FollowRecoveryScope {
    @TaskLocal static var authorization: FollowRecoveryAuthorization?
    @TaskLocal static var heading: FollowRecoveryHeadingRequest?
}

@MainActor
protocol FollowMeAbsoluteHeadingMotion: FollowMeMotion {
    func recoveryPoseSample() -> NavigationPoseSample
    func performRecovery(_ request: FollowRecoveryHeadingRequest, context: FollowMotionRequestContext) async -> FollowMotionResult
}

extension FollowMeAbsoluteHeadingMotion {
    func recoveryPoseSample() -> NavigationPoseSample { .unavailable }
}

extension FollowMeMotion {
    func performRecoveryHeading(_ request: FollowRecoveryHeadingRequest, context: FollowMotionRequestContext) async -> FollowMotionResult {
        if let companion = self as? any FollowMeAbsoluteHeadingMotion {
            return await companion.performRecovery(request, context: context)
        }
        return await stopForUnavailableRecovery(context: context)
    }

    func stopForUnavailableRecovery(context: FollowMotionRequestContext) async -> FollowMotionResult {
        // Relative-only providers cannot certify the source boundary. Confirm stop, never turn.
        let operation = FollowMotionOperationContext(request: context, controllerOperationID: nil, purpose: context.purpose, profile: nil)
        do {
            try await stopAndConfirm()
            return .init(result: .cancelled, context: operation, failure: nil, stopOutcome: .confirmed)
        } catch {
            return .init(result: .failed(.commandFailed), context: operation,
                failure: .init(context: operation, reason: .commandFailed, stopOutcome: .failed, source: .result), stopOutcome: .failed)
        }
    }
}
