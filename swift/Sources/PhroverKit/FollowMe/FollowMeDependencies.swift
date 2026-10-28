import Foundation
import RoverNav

public enum FollowPerceptionIssue: String, Equatable, Sendable {
    case noFrames, staleFrame, poseUnavailable, depthUnavailable, trackingLimited, trackingUnavailable

    public var message: String {
        switch self {
        case .noFrames: "No camera frames received. Check the AR session and camera access."
        case .staleFrame: "Camera frame stale or timestamp invalid. Check camera delivery and inference latency."
        case .poseUnavailable: "Rover pose unavailable. Wait for AR tracking to recover."
        case .depthUnavailable: "Depth unavailable. Check LiDAR visibility and depth support."
        case .trackingLimited: "AR tracking limited. Move slowly and point the camera at a well-lit, textured scene."
        case .trackingUnavailable: "AR tracking unavailable. Check the AR session and camera access."
        }
    }
}

/// Best-effort live ARKit diagnostics, never an input to pose or person projection.
public enum FollowTrackingReason: String, Equatable, Sendable {
    case initializing, excessiveMotion, insufficientFeatures, relocalizing, notAvailable, unknown

    public var action: String {
        switch self {
        case .initializing: "Wait for AR initialization while pointing at a well-lit, textured scene."
        case .excessiveMotion: "Move the camera slowly and keep it steady."
        case .insufficientFeatures: "Point the camera at a well-lit, textured scene."
        case .relocalizing: "Return the camera to a previously observed scene and wait for relocalization."
        case .notAvailable, .unknown: "Check the AR session and camera access."
        }
    }
}

public struct FollowFrameBatch: @unchecked Sendable {
    public let frameID: ARFrameID
    public let timestamp: TimeInterval
    public let pose: Pose2D?
    public let depthAvailable: Bool
    public let people: [FollowPersonObservation]
    /// Detections that failed the full-observation rules but still give a direction; see
    /// `FollowBearingObservation`. Empty from providers that do not supply them.
    public let partialPeople: [FollowBearingObservation]
    public let trackingQuality: ARTrackingQuality?
    public let trackingReason: FollowTrackingReason?
    /// Detector wall-clock duration in seconds; nil means not measured.
    public let inferenceDuration: TimeInterval?
    public let perceptionDiagnostics: FollowPerceptionDiagnostics?
    /// Delivery diagnostics only; capture timestamp remains the freshness authority.
    var publishedUptime: TimeInterval?
    var ingestedUptime: TimeInterval?

    public init(frameID: ARFrameID, timestamp: TimeInterval, pose: Pose2D?,
                depthAvailable: Bool, people: [FollowPersonObservation],
                trackingQuality: ARTrackingQuality? = nil, trackingReason: FollowTrackingReason? = nil,
                inferenceDuration: TimeInterval? = nil, perceptionDiagnostics: FollowPerceptionDiagnostics? = nil,
                partialPeople: [FollowBearingObservation] = []) {
        self.partialPeople = partialPeople
        self.frameID = frameID
        self.timestamp = timestamp
        self.pose = pose
        self.depthAvailable = depthAvailable
        self.people = people
        self.trackingQuality = trackingQuality
        self.trackingReason = trackingReason
        self.inferenceDuration = inferenceDuration
        self.perceptionDiagnostics = perceptionDiagnostics
    }
}

public enum FollowPerceptionEvent: Sendable {
    case frame(FollowFrameBatch)
    case interrupted
    case failed(String)
}

@MainActor
public protocol FollowMePerception {
    var detectorReady: Bool { get }
    var personLabelAvailable: Bool { get }
    func events() -> AsyncStream<FollowPerceptionEvent>
}

@MainActor
public protocol FollowMeMotion {
    func rotateForScan(by angle: Double) async -> NavigationResult
    func alignTowardPerson(by angle: Double) async -> NavigationResult
    func signalReady() async -> NavigationResult
    func navigate(to goal: Vec2, stoppingAtForwardClearance clearance: Double) async -> NavigationResult
    func stopAndConfirm() async throws
    func safetyStates() -> AsyncStream<NavigationSafetyState>
}

/// Optional internal companion. Original public conformers acquire no requirements.
@MainActor
protocol FollowMeSourceClockMotion: FollowMeMotion {
    var sourceUptime: TimeInterval? { get }
}

@MainActor
protocol FollowMeContextualMotion: FollowMeSourceClockMotion {
    func inhibitScanContinuation(origin: FollowMotionStopOrigin)
    func perform(_ request: FollowMotionRequest, context: FollowMotionRequestContext) async -> FollowMotionResult
    func motionFailures() -> AsyncStream<FollowMotionFailureDelivery>
    /// Move the running follow-goal drive without stopping it. False means "not applied";
    /// the caller stops and relaunches as before.
    func updateFollowGoal(_ goal: Vec2) -> Bool
}

extension FollowMeMotion {
    func performContextual(_ request: FollowMotionRequest, context: FollowMotionRequestContext,
                           admission: FollowReadyAdmission? = nil,
                           recovery: FollowRecoveryAuthorization? = nil) async -> FollowMotionResult {
        await FollowRecoveryScope.$authorization.withValue(recovery ?? FollowRecoveryScope.authorization) {
            await FollowReadyAdmissionScope.$current.withValue(admission ?? FollowReadyAdmissionScope.current) {
                if FollowRecoveryScope.authorization != nil, !(self is any FollowMeAbsoluteHeadingMotion) {
                    return await stopForUnavailableRecovery(context: context)
                }
                if let contextual = self as? any FollowMeContextualMotion { return await contextual.perform(request, context: context) }
                return await performLegacy(request, context: context)
            }
        }
    }

    fileprivate func performLegacy(_ request: FollowMotionRequest, context: FollowMotionRequestContext) async -> FollowMotionResult {
        let result: NavigationResult
        switch request {
        case .scan(let angle): result = await rotateForScan(by: angle)
        case .alignment(let angle): result = await alignTowardPerson(by: angle)
        case .ready:
            if let admission = FollowReadyAdmissionScope.current, !admission.admit() {
                return .init(result: .cancelled, context: .init(request: context, controllerOperationID: nil,
                    purpose: nil, profile: nil), failure: nil, deferred: admission.deferred)
            }
            result = await signalReady()
        case .following(let goal, let clearance): result = await navigate(to: goal, stoppingAtForwardClearance: clearance)
        }
        let operation = FollowMotionOperationContext(request: context, controllerOperationID: nil, purpose: nil,
            profile: nil, requestedRotation: request.requestedRotation)
        let failure: FollowMotionFailureDelivery?
        if case .failed(let reason) = result {
            failure = .init(context: operation, reason: reason, stopOutcome: .unknown, source: .result)
        } else { failure = nil }
        return .init(result: result, context: operation, failure: failure)
    }
}

extension FollowMeContextualMotion {
    var sourceUptime: TimeInterval? { nil }
    func inhibitScanContinuation(origin: FollowMotionStopOrigin) {}
    func updateFollowGoal(_ goal: Vec2) -> Bool { false }

    func perform(_ request: FollowMotionRequest, context: FollowMotionRequestContext) async -> FollowMotionResult {
        await performLegacy(request, context: context)
    }
    func motionFailures() -> AsyncStream<FollowMotionFailureDelivery> {
        let states = safetyStates()
        return AsyncStream { continuation in
            let observer = Task { @MainActor in
                for await state in states {
                    if case .failed(let reason) = state {
                        continuation.yield(.init(context: .unknown, reason: reason, stopOutcome: .unknown, source: .stream))
                    }
                }
                continuation.finish()
            }
            continuation.onTermination = { _ in observer.cancel() }
        }
    }
}

@MainActor
public protocol FollowMeClock {
    var now: TimeInterval { get }
    func sleep(seconds: Double) async
}
