import Foundation

public struct SilentSearchMission: Equatable, Sendable {
    public let id: UUID
    public let role: RoverRole
    public let targetLabel: String
    public let searchDurationSeconds: UInt32
    public let markerID: String

    public init?(
        id: UUID = UUID(),
        role: RoverRole,
        targetLabel: String,
        searchDurationSeconds: UInt32,
        markerID: String
    ) {
        guard !targetLabel.isEmpty,
              targetLabel == targetLabel.trimmingCharacters(in: .whitespacesAndNewlines),
              searchDurationSeconds > 0,
              SharedMissionCalibrator.isValidMarkerID(markerID) else {
            return nil
        }
        self.id = id
        self.role = role
        self.targetLabel = targetLabel
        self.searchDurationSeconds = searchDurationSeconds
        self.markerID = markerID
    }
}

public enum SilentSearchHandshakeStep: String, Equatable, Sendable {
    case ready
    case presenting
    case scanning
}

public enum SilentSearchRendezvousStep: String, Equatable, Sendable {
    case ready
    case waiting
    case rotating
    case presenting
    case scanning
}

public enum SilentSearchTerminalResult: Equatable, Sendable {
    case success
    case notFound
    case operatorStopped
    case operatorAborted
    case partnerTimeout
    case protocolFailure(OpticalProtocolRejection)
    case calibrationInvalidated
    case motionFailure(SilentSearchMotionFailure)
    case safetyFailure(SilentSearchSafetyFailure)
}

public enum SilentSearchPhase: Equatable, Sendable {
    case setup
    case calibrating
    case handshake(SilentSearchHandshakeStep)
    case waitingForSearch
    case searching
    case returning
    case rendezvous(SilentSearchRendezvousStep)
    case waitingForConvergence
    case converging
    case terminal(SilentSearchTerminalResult)

    var telemetryName: String {
        switch self {
        case .setup: "setup"
        case .calibrating: "calibrating"
        case .handshake: "handshake"
        case .waitingForSearch: "waiting_for_search"
        case .searching: "searching"
        case .returning: "returning"
        case .rendezvous: "rendezvous"
        case .waitingForConvergence: "waiting_for_convergence"
        case .converging: "converging"
        case .terminal: "terminal"
        }
    }
}

public enum SilentSearchReadinessRequirement: String, Equatable, Sendable {
    case tracking
    case lidar
    case generation
    case detector
    case detectorLabel
    case commandLink
}

public enum SilentSearchCoordinatorDiagnostic: Equatable, Sendable {
    case notReady([SilentSearchReadinessRequirement])
    case calibrationRejected(SharedMissionCalibrationDiagnostic)
    case opticalTimedOut
}

public enum SilentSearchTransitionError: Error, Equatable, Sendable {
    case illegal(from: SilentSearchPhase, to: SilentSearchPhase)
}
