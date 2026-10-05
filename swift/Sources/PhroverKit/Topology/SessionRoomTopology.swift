import Foundation
import RoverNav

@MainActor
public final class SessionRoomTopology: RoomTopologyManaging {
    private static let minimumDoorwayWidth = 0.65
    private static let candidateMatchDistance = 0.5
    private static let candidateMatchAngle = 35.0 * Double.pi / 180.0
    private static let crossingClearance = 0.35
    private static let requiredBeyondPlanePoses = 3
    private static let maximumObservationStep = 0.75

    private struct PendingTransition {
        let candidate: DoorwayCandidate
        let sourceRoomID: RoomID
        var lastFrameSequence: UInt64?
        var lastTimestamp: TimeInterval?
        var lastPose: Pose2D
        var hasApproachSideEvidence: Bool
        var consecutiveBeyondPlanePoses = 0
        var isReadyForConfirmation = false
    }

    private struct CandidateOrientation {
        let direction: Vec2
        let signedDistance: Double
        let flipped: Bool
    }

    private var sessionGeneration: UInt64?
    private var currentRoomID: RoomID?
    private var rooms: [RoomID: Room] = [:]
    private var doorways: [DoorwayID: Doorway] = [:]
    private var candidates: [DoorwayCandidateID: DoorwayCandidate] = [:]
    private var pendingTransition: PendingTransition?
    private var nextRoomNumber = 1
    private var nextDoorwayNumber = 1
    private var nextCandidateNumber = 1
    private let telemetry: RoomTopologyTelemetrySink

    public init(
        telemetry: @escaping RoomTopologyTelemetrySink = { event, fields in
            RuntimeFileLog.append(event, fields: fields)
        }
    ) {
        self.telemetry = telemetry
    }

    public var snapshot: RoomTopologySnapshot {
        RoomTopologySnapshot(
            sessionGeneration: sessionGeneration,
            currentRoomID: currentRoomID,
            rooms: rooms.values.sorted { $0.id < $1.id },
            doorways: doorways.values.sorted { $0.id < $1.id },
            candidates: candidates.values.sorted { $0.id < $1.id },
            pendingTransitionCandidateID: pendingTransition?.candidate.id
        )
    }

    public func startSession(generation: UInt64, initialPose: Pose2D) {
        clearSessionState()
        sessionGeneration = generation
        let roomID = makeRoomID()
        rooms[roomID] = Room(id: roomID, representativePoses: [initialPose])
        currentRoomID = roomID
        telemetry("room_session_started", [
            "generation": String(generation),
            "room_id": roomID.rawValue,
        ])
    }

    public func ingestStablePose(_ pose: Pose2D, sessionGeneration generation: UInt64) {
        guard generation == sessionGeneration, let currentRoomID else { return }
        rooms[currentRoomID]?.representativePoses.append(pose)
    }

    public func reset(forSessionGeneration generation: UInt64) {
        clearSessionState()
        sessionGeneration = generation
        telemetry("room_session_reset", ["generation": String(generation)])
    }

    public func shortestDoorwayPath(
        from startRoomID: RoomID,
        to destinationRoomID: RoomID
    ) -> [DoorwayRouteStep]? {
        guard sessionGeneration != nil,
              rooms[startRoomID] != nil,
              rooms[destinationRoomID] != nil else {
            return nil
        }
        guard startRoomID != destinationRoomID else { return [] }

        var visited: Set<RoomID> = [startRoomID]
        var queue: [(roomID: RoomID, route: [DoorwayRouteStep])] = [
            (startRoomID, []),
        ]
        var nextIndex = 0

        while nextIndex < queue.count {
            let current = queue[nextIndex]
            nextIndex += 1

            let nextSteps = doorways.values.compactMap { doorway -> DoorwayRouteStep? in
                guard let nextRoomID = oppositeRoom(to: current.roomID, through: doorway) else {
                    return nil
                }
                return DoorwayRouteStep(
                    doorwayID: doorway.id,
                    fromRoomID: current.roomID,
                    toRoomID: nextRoomID
                )
            }.sorted { $0.doorwayID < $1.doorwayID }

            for step in nextSteps where visited.insert(step.toRoomID).inserted {
                let route = current.route + [step]
                if step.toRoomID == destinationRoomID {
                    return route
                }
                queue.append((step.toRoomID, route))
            }
        }

        return nil
    }

    public func refreshCandidates(
        from frontiers: [Frontier],
        referencePose: Pose2D
    ) -> [DoorwayCandidate] {
        guard currentRoomID != nil else { return [] }

        let incidentCandidates = incidentDoorwayCandidates()
        let priorCandidates = (incidentCandidates + candidates.values).reduce(
            into: [DoorwayCandidateID: DoorwayCandidate]()
        ) { result, candidate in
            result[candidate.id] = candidate
        }.values.sorted { $0.id < $1.id }
        var matchedIDs: Set<DoorwayCandidateID> = []
        var refreshed = Dictionary(
            incidentCandidates.map { ($0.id, $0) },
            uniquingKeysWith: { first, _ in first }
        )

        for (index, frontier) in frontiers.enumerated() {
            var fields = Self.frontierTelemetryFields(frontier, index: index)
            guard Self.isFinite(frontier.centroid) else {
                fields["reason"] = "invalid_centroid"
                telemetry("doorway_frontier_rejected", fields)
                continue
            }
            guard frontier.widthMeters.isFinite,
                  frontier.widthMeters >= Self.minimumDoorwayWidth else {
                fields["reason"] = "below_safe_width"
                telemetry("doorway_frontier_rejected", fields)
                continue
            }
            guard let rawDirection = frontier.outwardDirection else {
                fields["reason"] = "missing_direction"
                telemetry("doorway_frontier_rejected", fields)
                continue
            }
            guard let orientation = candidateOrientation(
                planePoint: frontier.centroid,
                rawDirection: rawDirection,
                referencePose: referencePose
            ) else {
                fields["reason"] = "invalid_direction"
                telemetry("doorway_frontier_rejected", fields)
                continue
            }
            let direction = orientation.direction
            fields["raw_direction_x"] = Self.format(rawDirection.x)
            fields["raw_direction_y"] = Self.format(rawDirection.y)
            fields["reference_x"] = Self.format(referencePose.position.x)
            fields["reference_y"] = Self.format(referencePose.position.y)
            fields["signed_distance"] = Self.format(orientation.signedDistance)
            fields["direction_flipped"] = String(orientation.flipped)
            fields["direction_x"] = Self.format(direction.x)
            fields["direction_y"] = Self.format(direction.y)
            let beyondPlaneGoal = frontier.centroid + direction * 0.60
            guard Self.isFinite(beyondPlaneGoal),
                  beyondPlaneGoal.distance(to: frontier.centroid) > 1e-9 else {
                fields["reason"] = "invalid_beyond_plane_goal"
                telemetry("doorway_frontier_rejected", fields)
                continue
            }

            let match = priorCandidates
                .filter { !matchedIDs.contains($0.id) }
                .filter {
                    $0.planePoint.distance(to: frontier.centroid) <= Self.candidateMatchDistance
                        && angleBetween($0.outwardDirection, direction) <= Self.candidateMatchAngle
                }
                .min {
                    let leftDistance = $0.planePoint.distance(to: frontier.centroid)
                    let rightDistance = $1.planePoint.distance(to: frontier.centroid)
                    return leftDistance == rightDistance ? $0.id < $1.id : leftDistance < rightDistance
                }
            let id = match?.id ?? makeCandidateID()
            if match != nil { matchedIDs.insert(id) }
            refreshed[id] = DoorwayCandidate(
                id: id,
                planePoint: frontier.centroid,
                outwardDirection: direction,
                widthMeters: frontier.widthMeters,
                frontierCellCount: frontier.cellCount,
                doorwayID: match?.doorwayID,
                oppositeRoomID: match?.oppositeRoomID
            )
            fields["candidate_id"] = id.rawValue
            fields["beyond_x"] = Self.format(frontier.centroid.x + direction.x * 0.60)
            fields["beyond_y"] = Self.format(frontier.centroid.y + direction.y * 0.60)
            telemetry("doorway_frontier_admitted", fields)
        }

        candidates = refreshed
        return candidates.values.sorted { $0.id < $1.id }
    }

    public func rankedCandidates(
        assessments: [DoorwayCandidateAssessment],
        visualBoosts: [DoorwayCandidateID: Double] = [:],
        excluding excludedCandidateIDs: Set<DoorwayCandidateID> = []
    ) -> [RankedDoorwayCandidate] {
        let assessmentByID = Dictionary(
            assessments.map { ($0.candidateID, $0) },
            uniquingKeysWith: { first, _ in first }
        )
        let ranked = candidates.values.compactMap { candidate -> RankedDoorwayCandidate? in
            guard !excludedCandidateIDs.contains(candidate.id),
                  let assessment = assessmentByID[candidate.id] else {
                return nil
            }
            return RankedDoorwayCandidate(
                candidate: candidate,
                assessment: assessment,
                visualBoost: min(1, max(0, visualBoosts[candidate.id] ?? 0))
            )
        }.sorted(by: ranksBefore)

        for (index, item) in ranked.enumerated() {
            telemetry("doorway_candidate_ranked", [
                "candidate_id": item.candidate.id.rawValue,
                "rank": String(index + 1),
                "reachable": String(item.assessment.isReachable),
            ])
        }
        return ranked
    }

    @discardableResult
    public func beginTransition(
        candidateID: DoorwayCandidateID,
        approachPose: Pose2D
    ) -> TransitionStartResult {
        guard pendingTransition == nil else { return .rejected(.alreadyActive) }
        guard let sourceRoomID = currentRoomID else { return .rejected(.missingCurrentRoom) }
        guard let candidate = candidates[candidateID] else { return .rejected(.missingCandidate) }
        guard let orientation = candidateOrientation(
            planePoint: candidate.planePoint,
            rawDirection: candidate.outwardDirection,
            referencePose: approachPose
        ) else {
            return .rejected(.invalidGeometry)
        }
        guard abs(orientation.signedDistance) > 1e-9 else {
            return .rejected(.invalidApproachSide)
        }
        if orientation.flipped {
            let corrected = DoorwayCandidate(
                id: candidate.id,
                planePoint: candidate.planePoint,
                outwardDirection: orientation.direction,
                widthMeters: candidate.widthMeters,
                frontierCellCount: candidate.frontierCellCount,
                doorwayID: candidate.doorwayID,
                oppositeRoomID: candidate.oppositeRoomID
            )
            candidates[candidateID] = corrected
            telemetry("room_transition_orientation_corrected", [
                "candidate_id": candidateID.rawValue,
                "raw_direction_x": Self.format(candidate.outwardDirection.x),
                "raw_direction_y": Self.format(candidate.outwardDirection.y),
                "corrected_direction_x": Self.format(orientation.direction.x),
                "corrected_direction_y": Self.format(orientation.direction.y),
                "reference_x": Self.format(approachPose.position.x),
                "reference_y": Self.format(approachPose.position.y),
                "signed_distance": Self.format(orientation.signedDistance),
                "direction_flipped": String(orientation.flipped),
            ])
            return .correctedOrientation(corrected)
        }

        pendingTransition = PendingTransition(
            candidate: candidate,
            sourceRoomID: sourceRoomID,
            lastPose: approachPose,
            hasApproachSideEvidence: true
        )
        telemetry("room_transition_started", [
            "candidate_id": candidateID.rawValue,
            "from_room_id": sourceRoomID.rawValue,
        ])
        return .started
    }

    public func observeTransition(
        _ observation: TransitionObservation
    ) -> TransitionObservationResult {
        guard var transition = pendingTransition,
              observation.sessionGeneration == sessionGeneration,
              observation.isTrackingNormal,
              transition.lastFrameSequence.map({ observation.frameSequence > $0 }) ?? true,
              transition.lastTimestamp.map({ observation.timestamp > $0 }) ?? true else {
            return .ignored
        }

        transition.lastFrameSequence = observation.frameSequence
        transition.lastTimestamp = observation.timestamp
        let distanceFromLastPose = transition.lastPose.position.distance(to: observation.pose.position)
        transition.lastPose = observation.pose
        let signedDistance = signedDistance(
            from: observation.pose.position,
            toPlaneAt: transition.candidate.planePoint,
            normal: transition.candidate.outwardDirection
        )

        if distanceFromLastPose > Self.maximumObservationStep + 1e-9 {
            transition.hasApproachSideEvidence = signedDistance < 0
            transition.consecutiveBeyondPlanePoses = 0
            transition.isReadyForConfirmation = false
            pendingTransition = transition
            return .ignored
        }

        if signedDistance < 0 {
            transition.hasApproachSideEvidence = true
        }
        if transition.hasApproachSideEvidence,
           signedDistance >= Self.crossingClearance {
            transition.consecutiveBeyondPlanePoses += 1
        } else {
            transition.consecutiveBeyondPlanePoses = 0
        }
        transition.isReadyForConfirmation = transition.consecutiveBeyondPlanePoses
            >= Self.requiredBeyondPlanePoses
        pendingTransition = transition
        return transition.isReadyForConfirmation ? .readyForConfirmation : .observing
    }

    @discardableResult
    public func confirmTransition() -> RoomID? {
        guard let transition = pendingTransition,
              transition.isReadyForConfirmation,
              currentRoomID == transition.sourceRoomID else {
            return nil
        }

        let destinationRoomID: RoomID
        let doorway: Doorway
        if let doorwayID = transition.candidate.doorwayID,
           let knownDoorway = doorways[doorwayID],
           let oppositeRoomID = oppositeRoom(to: transition.sourceRoomID, through: knownDoorway) {
            destinationRoomID = oppositeRoomID
            doorway = knownDoorway
            rooms[destinationRoomID]?.representativePoses.append(transition.lastPose)
        } else {
            destinationRoomID = makeRoomID()
            rooms[destinationRoomID] = Room(
                id: destinationRoomID,
                representativePoses: [transition.lastPose]
            )
            doorway = Doorway(
                id: makeDoorwayID(),
                candidateID: transition.candidate.id,
                planePoint: transition.candidate.planePoint,
                normalFromFirstRoom: transition.candidate.outwardDirection,
                widthMeters: transition.candidate.widthMeters,
                frontierCellCount: transition.candidate.frontierCellCount,
                firstRoomID: transition.sourceRoomID,
                secondRoomID: destinationRoomID
            )
            doorways[doorway.id] = doorway
        }

        currentRoomID = destinationRoomID
        pendingTransition = nil
        candidates = Dictionary(
            incidentDoorwayCandidates().map { ($0.id, $0) },
            uniquingKeysWith: { first, _ in first }
        )
        telemetry("doorway_crossed", [
            "doorway_id": doorway.id.rawValue,
            "from_room_id": transition.sourceRoomID.rawValue,
            "to_room_id": destinationRoomID.rawValue,
        ])
        telemetry("room_transition_completed", [
            "candidate_id": transition.candidate.id.rawValue,
            "room_id": destinationRoomID.rawValue,
        ])
        return destinationRoomID
    }

    public func rejectTransition(reason: String) {
        guard let transition = pendingTransition else { return }
        pendingTransition = nil
        telemetry("room_transition_rejected", [
            "candidate_id": transition.candidate.id.rawValue,
            "reason": reason,
        ])
    }

    public func abandonTransition() {
        guard let transition = pendingTransition else { return }
        pendingTransition = nil
        telemetry("room_transition_rejected", [
            "candidate_id": transition.candidate.id.rawValue,
            "reason": "abandoned",
        ])
    }

    private func ranksBefore(_ lhs: RankedDoorwayCandidate, _ rhs: RankedDoorwayCandidate) -> Bool {
        let lhsUnexplored = lhs.candidate.doorwayID == nil
        let rhsUnexplored = rhs.candidate.doorwayID == nil
        if lhsUnexplored != rhsUnexplored { return lhsUnexplored }
        if lhs.assessment.isReachable != rhs.assessment.isReachable {
            return lhs.assessment.isReachable
        }
        let lhsQuality = geometricQuality(lhs.candidate)
        let rhsQuality = geometricQuality(rhs.candidate)
        if lhsQuality != rhsQuality { return lhsQuality > rhsQuality }
        if lhs.visualBoost != rhs.visualBoost { return lhs.visualBoost > rhs.visualBoost }
        if lhs.assessment.pathDistance != rhs.assessment.pathDistance {
            return lhs.assessment.pathDistance < rhs.assessment.pathDistance
        }
        return lhs.candidate.id < rhs.candidate.id
    }

    private func geometricQuality(_ candidate: DoorwayCandidate) -> Double {
        let widthQuality = 1 - min(abs(candidate.widthMeters - 1.0), 1.0)
        return widthQuality + Double(candidate.frontierCellCount) / 1_000.0
    }

    private func incidentDoorwayCandidates() -> [DoorwayCandidate] {
        guard let currentRoomID else { return [] }
        return doorways.values.compactMap { doorway in
            let normal: Vec2
            let oppositeRoomID: RoomID
            if doorway.firstRoomID == currentRoomID {
                normal = doorway.normalFromFirstRoom
                oppositeRoomID = doorway.secondRoomID
            } else if doorway.secondRoomID == currentRoomID {
                normal = doorway.normalFromFirstRoom * -1
                oppositeRoomID = doorway.firstRoomID
            } else {
                return nil
            }
            return DoorwayCandidate(
                id: doorway.candidateID,
                planePoint: doorway.planePoint,
                outwardDirection: normal,
                widthMeters: doorway.widthMeters,
                frontierCellCount: doorway.frontierCellCount,
                doorwayID: doorway.id,
                oppositeRoomID: oppositeRoomID
            )
        }.sorted { $0.id < $1.id }
    }

    private func oppositeRoom(to roomID: RoomID, through doorway: Doorway) -> RoomID? {
        if doorway.firstRoomID == roomID { return doorway.secondRoomID }
        if doorway.secondRoomID == roomID { return doorway.firstRoomID }
        return nil
    }

    private func signedDistance(from point: Vec2, toPlaneAt planePoint: Vec2, normal: Vec2) -> Double {
        let offset = point - planePoint
        return offset.x * normal.x + offset.y * normal.y
    }

    private func candidateOrientation(
        planePoint: Vec2,
        rawDirection: Vec2,
        referencePose: Pose2D
    ) -> CandidateOrientation? {
        guard Self.isFinite(planePoint),
              Self.isFinite(referencePose.position),
              let normalizedDirection = normalized(rawDirection) else {
            return nil
        }
        let distance = signedDistance(
            from: referencePose.position,
            toPlaneAt: planePoint,
            normal: normalizedDirection
        )
        guard distance.isFinite else { return nil }
        return CandidateOrientation(
            direction: distance >= 0 ? normalizedDirection * -1 : normalizedDirection,
            signedDistance: distance,
            flipped: distance >= 0
        )
    }

    private static func frontierTelemetryFields(
        _ frontier: Frontier,
        index: Int
    ) -> [String: String] {
        [
            "frontier_index": String(index),
            "centroid_x": format(frontier.centroid.x),
            "centroid_y": format(frontier.centroid.y),
            "width_m": format(frontier.widthMeters),
            "cell_count": String(frontier.cellCount),
            "direction_available": String(frontier.outwardDirection != nil),
            "direction_x": frontier.outwardDirection.map { format($0.x) } ?? "none",
            "direction_y": frontier.outwardDirection.map { format($0.y) } ?? "none",
        ]
    }

    private static func isFinite(_ point: Vec2) -> Bool {
        point.x.isFinite && point.y.isFinite
    }

    private static func format(_ value: Double) -> String {
        String(format: "%.2f", value)
    }

    private func normalized(_ vector: Vec2) -> Vec2? {
        guard Self.isFinite(vector) else { return nil }
        let length = vector.length
        guard length.isFinite, length > 1e-9 else { return nil }
        let result = vector * (1 / length)
        return Self.isFinite(result) ? result : nil
    }

    private func angleBetween(_ lhs: Vec2, _ rhs: Vec2) -> Double {
        acos(min(1, max(-1, lhs.x * rhs.x + lhs.y * rhs.y)))
    }

    private func clearSessionState() {
        currentRoomID = nil
        rooms.removeAll()
        doorways.removeAll()
        candidates.removeAll()
        pendingTransition = nil
        nextRoomNumber = 1
        nextDoorwayNumber = 1
        nextCandidateNumber = 1
    }

    private func makeRoomID() -> RoomID {
        defer { nextRoomNumber += 1 }
        return RoomID("room_\(nextRoomNumber)")
    }

    private func makeCandidateID() -> DoorwayCandidateID {
        defer { nextCandidateNumber += 1 }
        return DoorwayCandidateID("doorway_candidate_\(nextCandidateNumber)")
    }

    private func makeDoorwayID() -> DoorwayID {
        defer { nextDoorwayNumber += 1 }
        return DoorwayID("doorway_\(nextDoorwayNumber)")
    }
}
