import Foundation
import RoverNav

public struct SectorFrontierObservation: Equatable, Sendable {
    public let localCentroid: Vec2
    public let width: Double

    public init(localCentroid: Vec2, width: Double) {
        self.localCentroid = localCentroid
        self.width = width
    }
}

public enum SectorFrontierStatus: Equatable, Sendable {
    case available
    case visited
    case rejected
}

public enum SectorFrontierRejectionReason: Equatable, Sendable {
    case unreachable
    case outOfSector
    case pathPolicy(PathPolicyViolation)
    case missionRejected(String)
}

public struct SectorFrontierCandidate: Equatable, Sendable {
    public let stableID: String
    public let localCentroid: Vec2
    public let missionCentroid: MissionPoint
    public let width: Double
    public let status: SectorFrontierStatus
    public let rejectionReason: SectorFrontierRejectionReason?
    public let safePath: [Vec2]?
    public let pathLength: Double?
}

public enum SectorExplorerSelection: Equatable, Sendable {
    case candidate(SectorFrontierCandidate)
    case exhausted
}

public final class SectorExplorer {
    public private(set) var candidates: [SectorFrontierCandidate] = []

    private struct Record {
        var localCentroid: Vec2
        var missionCentroid: MissionPoint
        var width: Double
        var status: SectorFrontierStatus
        var persistentRejection: SectorFrontierRejectionReason?
    }

    private struct NewObservation {
        let index: Int
        let observation: SectorFrontierObservation
        let missionCentroid: MissionPoint
    }

    private let frame: SharedMissionFrame
    private let sector: SearchSector
    private let policy: any PathAdmissibilityPolicy
    private let planner: (Vec2, Vec2, Costmap?) -> [Vec2]?
    private let events: (any SilentSearchEventSink)?
    private var records: [String: Record] = [:]
    private var nextID = 1

    public init(
        frame: SharedMissionFrame,
        sector: SearchSector,
        policy: any PathAdmissibilityPolicy,
        events: (any SilentSearchEventSink)? = nil
    ) {
        self.frame = frame
        self.sector = sector
        self.policy = policy
        self.events = events
        self.planner = { start, goal, costmap in
            guard let costmap else { return nil }
            return AStarPlanner().plan(from: start, to: goal, in: costmap)
        }
    }

    init(
        frame: SharedMissionFrame,
        sector: SearchSector,
        policy: any PathAdmissibilityPolicy,
        events: (any SilentSearchEventSink)? = nil,
        planner: @escaping (Vec2, Vec2) -> [Vec2]?
    ) {
        self.frame = frame
        self.sector = sector
        self.policy = policy
        self.events = events
        self.planner = { start, goal, _ in planner(start, goal) }
    }

    public func rebuild(costmap: Costmap, observed: ObservedGrid, from start: Vec2) {
        let observations = FrontierFinder.candidates(costmap: costmap, observed: observed).map {
            SectorFrontierObservation(localCentroid: $0.centroid, width: $0.widthMeters)
        }
        rebuild(observations: observations, from: start, costmap: costmap)
    }

    public func rebuild(observations: [SectorFrontierObservation], from start: Vec2) {
        rebuild(observations: observations, from: start, costmap: nil)
    }

    public func nextCandidate() -> SectorExplorerSelection {
        let eligible = candidates.filter {
            $0.status == .available && $0.rejectionReason == nil && $0.pathLength != nil
        }
        guard let selected = eligible.sorted(by: Self.ranksBefore).first else {
            events?.record(event: "silent_search_frontier_exhausted", fields: [:])
            return .exhausted
        }
        events?.record(event: "silent_search_frontier_ranked", fields: [
            "frontier_id": selected.stableID,
            "path_length_mm": "\(Int(((selected.pathLength ?? 0) * 1_000).rounded()))",
        ])
        return .candidate(selected)
    }

    public func markVisited(_ stableID: String) {
        guard var record = records[stableID] else { return }
        record.status = .visited
        record.persistentRejection = nil
        records[stableID] = record
        refreshCurrentCandidate(stableID)
        events?.record(event: "silent_search_frontier_visited", fields: ["frontier_id": stableID])
    }

    public func markRejected(_ stableID: String, reason: SectorFrontierRejectionReason) {
        guard var record = records[stableID] else { return }
        record.status = .rejected
        record.persistentRejection = reason
        records[stableID] = record
        refreshCurrentCandidate(stableID)
        events?.record(event: "silent_search_frontier_rejected", fields: [
            "frontier_id": stableID,
            "reason": String(describing: reason),
        ])
    }

    private func rebuild(
        observations: [SectorFrontierObservation],
        from start: Vec2,
        costmap: Costmap?
    ) {
        let newObservations: [NewObservation] = observations.enumerated().compactMap { index, observation in
            guard observation.localCentroid.x.isFinite,
                  observation.localCentroid.y.isFinite,
                  observation.width.isFinite,
                  observation.width >= 0,
                  let mission = frame.missionPoint(from: observation.localCentroid) else {
                return nil
            }
            return NewObservation(index: index, observation: observation, missionCentroid: mission)
        }

        struct Pair {
            let oldID: String
            let new: NewObservation
            let distance: Double
        }

        var pairs: [Pair] = []
        for (oldID, old) in records {
            for new in newObservations {
                let dx = old.missionCentroid.x - new.missionCentroid.x
                let dy = old.missionCentroid.y - new.missionCentroid.y
                let distance = hypot(dx, dy)
                if distance < 0.30 {
                    pairs.append(Pair(oldID: oldID, new: new, distance: distance))
                }
            }
        }
        pairs.sort {
            if $0.distance != $1.distance { return $0.distance < $1.distance }
            let lhsID = Self.numericID($0.oldID)
            let rhsID = Self.numericID($1.oldID)
            if lhsID != rhsID { return lhsID < rhsID }
            if $0.new.missionCentroid.x != $1.new.missionCentroid.x {
                return $0.new.missionCentroid.x < $1.new.missionCentroid.x
            }
            return $0.new.missionCentroid.y < $1.new.missionCentroid.y
        }

        var matchedOld = Set<String>()
        var matchedNew = Set<Int>()
        var assignment: [Int: String] = [:]
        for pair in pairs where !matchedOld.contains(pair.oldID) && !matchedNew.contains(pair.new.index) {
            matchedOld.insert(pair.oldID)
            matchedNew.insert(pair.new.index)
            assignment[pair.new.index] = pair.oldID
        }

        let unmatched = newObservations.filter { assignment[$0.index] == nil }.sorted {
            if $0.missionCentroid.x != $1.missionCentroid.x {
                return $0.missionCentroid.x < $1.missionCentroid.x
            }
            if $0.missionCentroid.y != $1.missionCentroid.y {
                return $0.missionCentroid.y < $1.missionCentroid.y
            }
            return $0.observation.width < $1.observation.width
        }
        for new in unmatched {
            let stableID = "frontier_\(nextID)"
            nextID += 1
            assignment[new.index] = stableID
        }

        var rebuilt: [SectorFrontierCandidate] = []
        for new in newObservations {
            guard let stableID = assignment[new.index] else { continue }
            var record = records[stableID] ?? Record(
                localCentroid: new.observation.localCentroid,
                missionCentroid: new.missionCentroid,
                width: new.observation.width,
                status: .available,
                persistentRejection: nil
            )
            record.localCentroid = new.observation.localCentroid
            record.missionCentroid = new.missionCentroid
            record.width = new.observation.width
            records[stableID] = record
            rebuilt.append(candidate(stableID: stableID, record: record, start: start, costmap: costmap))
        }
        candidates = rebuilt.sorted { Self.numericID($0.stableID) < Self.numericID($1.stableID) }
        for candidate in candidates {
            events?.record(event: "silent_search_frontier_discovered", fields: [
                "frontier_id": candidate.stableID,
                "x_mm": "\(Int((candidate.missionCentroid.x * 1_000).rounded()))",
                "y_mm": "\(Int((candidate.missionCentroid.y * 1_000).rounded()))",
            ])
            if let rejection = candidate.rejectionReason {
                events?.record(event: "silent_search_frontier_rejected", fields: [
                    "frontier_id": candidate.stableID,
                    "reason": String(describing: rejection),
                ])
            }
        }
    }

    private func candidate(
        stableID: String,
        record: Record,
        start: Vec2,
        costmap: Costmap?
    ) -> SectorFrontierCandidate {
        var rejection = record.persistentRejection
        var path: [Vec2]?
        var pathLength: Double?

        if record.status == .available {
            let inSector = switch sector {
            case .west: record.missionCentroid.x < -SilentSearchGeometry.centerBandHalfWidth
            case .east: record.missionCentroid.x > SilentSearchGeometry.centerBandHalfWidth
            }
            if !inSector {
                rejection = .outOfSector
            } else if let planned = planner(start, record.localCentroid, costmap) {
                let evaluatedPath = [start] + planned + [record.localCentroid]
                switch policy.evaluate(path: evaluatedPath) {
                case .admissible:
                    path = planned
                    pathLength = Self.length(of: evaluatedPath)
                case .rejected(let violation):
                    rejection = .pathPolicy(violation)
                }
            } else {
                rejection = .unreachable
            }
        }

        return SectorFrontierCandidate(
            stableID: stableID,
            localCentroid: record.localCentroid,
            missionCentroid: record.missionCentroid,
            width: record.width,
            status: record.status,
            rejectionReason: rejection,
            safePath: path,
            pathLength: pathLength
        )
    }

    private func refreshCurrentCandidate(_ stableID: String) {
        guard let index = candidates.firstIndex(where: { $0.stableID == stableID }),
              let record = records[stableID] else { return }
        let previous = candidates[index]
        candidates[index] = SectorFrontierCandidate(
            stableID: stableID,
            localCentroid: record.localCentroid,
            missionCentroid: record.missionCentroid,
            width: record.width,
            status: record.status,
            rejectionReason: record.persistentRejection,
            safePath: previous.safePath,
            pathLength: previous.pathLength
        )
    }

    private static func length(of path: [Vec2]) -> Double {
        zip(path, path.dropFirst()).reduce(0) { $0 + $1.0.distance(to: $1.1) }
    }

    private static func ranksBefore(_ lhs: SectorFrontierCandidate, _ rhs: SectorFrontierCandidate) -> Bool {
        let lhsLength = lhs.pathLength ?? .infinity
        let rhsLength = rhs.pathLength ?? .infinity
        if lhsLength != rhsLength { return lhsLength < rhsLength }
        if lhs.width != rhs.width { return lhs.width > rhs.width }
        if lhs.missionCentroid.x != rhs.missionCentroid.x {
            return lhs.missionCentroid.x < rhs.missionCentroid.x
        }
        if lhs.missionCentroid.y != rhs.missionCentroid.y {
            return lhs.missionCentroid.y < rhs.missionCentroid.y
        }
        return lhs.stableID < rhs.stableID
    }

    private static func numericID(_ stableID: String) -> Int {
        Int(stableID.dropFirst("frontier_".count)) ?? .max
    }
}
