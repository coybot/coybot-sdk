import Foundation

/// Read-only progress emitted by a deterministic room-transition mission for diagnostics.
public enum RoomTransitionDebugState: Equatable, Sendable {
    case idle
    case scanning(step: Int, total: Int, openingCount: Int, candidateCount: Int)
    case candidateFound(id: DoorwayCandidateID, reachable: Bool)
    case unreachable(id: DoorwayCandidateID)
    case approaching(id: DoorwayCandidateID)
    case confirmingCrossing(id: DoorwayCandidateID)
    case completed(doorwayID: DoorwayID, roomID: RoomID)
    case exhausted
    case failed(reason: String)
}
