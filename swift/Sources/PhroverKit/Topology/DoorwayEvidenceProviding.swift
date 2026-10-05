import Foundation

public protocol DoorwayEvidenceProviding: Sendable {
    func boostValues(
        forFrame frame: Data,
        candidates: [DoorwayCandidate]
    ) async -> [DoorwayCandidateID: Double]
}
