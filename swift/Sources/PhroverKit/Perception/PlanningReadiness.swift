import Foundation

struct PlanningReadinessSnapshot: Equatable, Sendable {
    let sessionGeneration: UInt64
    var normalObservationStreak: Int
    var trustedMeshRevision: UInt64

    var isPoseReady: Bool { normalObservationStreak >= 3 }
}

struct PlanningReadinessTracker: Sendable {
    private(set) var snapshot: PlanningReadinessSnapshot

    init(sessionGeneration: UInt64 = 0) {
        snapshot = PlanningReadinessSnapshot(
            sessionGeneration: sessionGeneration,
            normalObservationStreak: 0,
            trustedMeshRevision: 0
        )
    }

    mutating func reset(sessionGeneration: UInt64) {
        self = PlanningReadinessTracker(sessionGeneration: sessionGeneration)
    }

    mutating func suspend() {
        snapshot.normalObservationStreak = 0
    }

    mutating func ingest(_ observation: PoseObservation) {
        guard observation.sessionGeneration == snapshot.sessionGeneration else { return }
        if observation.trackingQuality == .normal {
            snapshot.normalObservationStreak += 1
        } else {
            snapshot.normalObservationStreak = 0
        }
    }

    @discardableResult
    mutating func recordTrustedMeshUpdate() -> Bool {
        guard snapshot.isPoseReady else { return false }
        snapshot.trustedMeshRevision &+= 1
        return true
    }
}
