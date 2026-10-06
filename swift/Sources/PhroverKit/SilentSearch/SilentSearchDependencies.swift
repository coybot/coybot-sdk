import Foundation

public typealias SilentSearchInstant = Int64

@MainActor
public protocol SilentSearchClock: AnyObject {
    var wallNowMilliseconds: Int64 { get }
    var monotonicNow: SilentSearchInstant { get }
    func sleep(until deadline: SilentSearchInstant) async throws
}

public enum SilentSearchTrackingReadiness: Equatable, Sendable {
    case unavailable
    case limited(sessionGeneration: UInt64)
    case normal(sessionGeneration: UInt64)
}

public struct SilentSearchReadiness: Equatable, Sendable {
    public let tracking: SilentSearchTrackingReadiness
    public let lidarAvailable: Bool
    public let generationValid: Bool
    public let detectorLoaded: Bool
    public let detectorLabelAvailable: Bool
    public let commandLinkAvailable: Bool

    public init(
        tracking: SilentSearchTrackingReadiness,
        lidarAvailable: Bool = true,
        generationValid: Bool = true,
        detectorLoaded: Bool,
        detectorLabelAvailable: Bool = true,
        commandLinkAvailable: Bool
    ) {
        self.tracking = tracking
        self.lidarAvailable = lidarAvailable
        self.generationValid = generationValid
        self.detectorLoaded = detectorLoaded
        self.detectorLabelAvailable = detectorLabelAvailable
        self.commandLinkAvailable = commandLinkAvailable
    }

    public static let notReady = SilentSearchReadiness(
        tracking: .unavailable,
        detectorLoaded: false,
        commandLinkAvailable: false
    )

    public static func ready(sessionGeneration: UInt64) -> SilentSearchReadiness {
        SilentSearchReadiness(
            tracking: .normal(sessionGeneration: sessionGeneration),
            detectorLoaded: true,
            commandLinkAvailable: true
        )
    }

    public var missingRequirements: [SilentSearchReadinessRequirement] {
        var missing: [SilentSearchReadinessRequirement] = []
        if case .normal = tracking {} else { missing.append(.tracking) }
        if !lidarAvailable { missing.append(.lidar) }
        if !generationValid { missing.append(.generation) }
        if !detectorLoaded { missing.append(.detector) }
        if !detectorLabelAvailable { missing.append(.detectorLabel) }
        if !commandLinkAvailable { missing.append(.commandLink) }
        return missing
    }

    public var sessionGeneration: UInt64? {
        guard case let .normal(generation) = tracking else { return nil }
        return generation
    }
}

@MainActor
public protocol SilentSearchReadinessChecking: AnyObject {
    var snapshot: SilentSearchReadiness { get }
}

public enum SilentSearchCalibrationEvent: Equatable, Sendable {
    case feedback(SilentSearchCalibrationFeedback)
    case progress(context: SilentSearchCalibrationFrameContext, acceptedFrameCount: Int)
    case rejected(SharedMissionCalibrationDiagnostic)
    case accepted(SharedMissionFrame)
}

@MainActor
public protocol SilentSearchCalibrating: AnyObject {
    func events(markerID: String, sessionGeneration: UInt64) -> AsyncStream<SilentSearchCalibrationEvent>
    func cancel()
}

@MainActor
public protocol SilentSearchOpticalExchanging: AnyObject {
    func present(payload: Data) async throws
    func scan(until deadline: SilentSearchInstant) async throws -> Data
    func cancel()
}

@MainActor
public protocol SilentSearchExploring: AnyObject {
    func nextCandidate() async -> SectorExplorerSelection
    func markVisited(_ stableID: String)
    func markRejected(_ stableID: String, reason: SectorFrontierRejectionReason)
}

public enum SilentSearchTargetObservationResult: Equatable, Sendable {
    case pending
    case confirmed(TargetConfirmation)
}

@MainActor
public protocol SilentSearchTargetObserving: AnyObject {
    func observeNextFrame(until deadline: SilentSearchInstant) async -> SilentSearchTargetObservationResult
}

public enum SilentSearchMotionPolicy: Equatable, Sendable {
    case sectorConstrained(SearchSector)
    case unrestrictedConvergence
}

public enum SilentSearchMotionFailure: Equatable, Sendable {
    case noPose
    case noPath
    case pathRejected(PathPolicyViolation)
    case obstacle
    case commandLink
    case tipping
    case stalled
    case tracking
    case positionToleranceExceeded
    case headingToleranceExceeded
}

public enum SilentSearchMotionResult: Equatable, Sendable {
    case arrived
    case failed(SilentSearchMotionFailure)
    case cancelled
}

@MainActor
public protocol SilentSearchMotion: AnyObject {
    func navigate(to target: MissionPoint, policy: SilentSearchMotionPolicy) async -> SilentSearchMotionResult
    func rotate(to heading: Double, tolerance: Double) async -> SilentSearchMotionResult
    func stop() async
    var currentMissionPose: MissionPose? { get }
    var currentMissionPath: [MissionPoint] { get }
}

public enum SilentSearchSafetyFailure: Equatable, Sendable {
    case transport
    case reactiveSafety
}

public enum SilentSearchSafetyEvent: Equatable, Sendable {
    case trackingNormal(generation: UInt64)
    case trackingLimited(generation: UInt64)
    case generationChanged
    case transportFailed
    case reactiveSafetyFailed
    case operatorStop
}

@MainActor
public protocol SilentSearchSafetyMonitoring: AnyObject {
    func events() -> AsyncStream<SilentSearchSafetyEvent>
}

public protocol SilentSearchEventSink: AnyObject, Sendable {
    func record(event: String, fields: [String: String])
}

@MainActor
public struct SilentSearchDependencies {
    public let clock: any SilentSearchClock
    public let readiness: any SilentSearchReadinessChecking
    public let calibration: any SilentSearchCalibrating
    public let opticalExchange: any SilentSearchOpticalExchanging
    public let explorer: any SilentSearchExploring
    public let targetObserver: any SilentSearchTargetObserving
    public let motion: any SilentSearchMotion
    public let safety: any SilentSearchSafetyMonitoring
    public let events: any SilentSearchEventSink

    public init(
        clock: any SilentSearchClock,
        readiness: any SilentSearchReadinessChecking,
        calibration: any SilentSearchCalibrating,
        opticalExchange: any SilentSearchOpticalExchanging,
        explorer: any SilentSearchExploring,
        targetObserver: any SilentSearchTargetObserving,
        motion: any SilentSearchMotion,
        safety: any SilentSearchSafetyMonitoring,
        events: any SilentSearchEventSink
    ) {
        self.clock = clock
        self.readiness = readiness
        self.calibration = calibration
        self.opticalExchange = opticalExchange
        self.explorer = explorer
        self.targetObserver = targetObserver
        self.motion = motion
        self.safety = safety
        self.events = events
    }
}

@MainActor
public final class RuntimeSilentSearchClock: SilentSearchClock {
    public init() {}

    public var wallNowMilliseconds: Int64 {
        Int64((Date().timeIntervalSince1970 * 1_000).rounded(.down))
    }

    public var monotonicNow: SilentSearchInstant {
        let value = DispatchTime.now().uptimeNanoseconds
        return value > UInt64(Int64.max) ? Int64.max : Int64(value)
    }

    public func sleep(until deadline: SilentSearchInstant) async throws {
        let remaining = deadline.subtractingReportingOverflow(monotonicNow)
        guard !remaining.overflow, remaining.partialValue > 0 else { return }
        try await Task.sleep(for: .nanoseconds(remaining.partialValue))
    }
}
