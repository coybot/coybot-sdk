import Foundation
import RoverNav

public struct RoomID: RawRepresentable, Hashable, Comparable, Sendable {
    public let rawValue: String

    public init(rawValue: String) {
        self.rawValue = rawValue
    }

    public init(_ rawValue: String) {
        self.rawValue = rawValue
    }

    public static func < (lhs: RoomID, rhs: RoomID) -> Bool {
        lhs.rawValue < rhs.rawValue
    }
}

public struct DoorwayID: RawRepresentable, Hashable, Comparable, Sendable {
    public let rawValue: String

    public init(rawValue: String) {
        self.rawValue = rawValue
    }

    public init(_ rawValue: String) {
        self.rawValue = rawValue
    }

    public static func < (lhs: DoorwayID, rhs: DoorwayID) -> Bool {
        lhs.rawValue < rhs.rawValue
    }
}

public struct DoorwayCandidateID: RawRepresentable, Hashable, Comparable, Sendable {
    public let rawValue: String

    public init(rawValue: String) {
        self.rawValue = rawValue
    }

    public init(_ rawValue: String) {
        self.rawValue = rawValue
    }

    public static func < (lhs: DoorwayCandidateID, rhs: DoorwayCandidateID) -> Bool {
        lhs.rawValue < rhs.rawValue
    }
}

public struct Room: Equatable, Sendable {
    public let id: RoomID
    public var representativePoses: [Pose2D]

    public init(id: RoomID, representativePoses: [Pose2D]) {
        self.id = id
        self.representativePoses = representativePoses
    }
}

public struct Doorway: Equatable, Sendable {
    public let id: DoorwayID
    public let candidateID: DoorwayCandidateID
    public let planePoint: Vec2
    public let normalFromFirstRoom: Vec2
    public let widthMeters: Double
    public let frontierCellCount: Int
    public let firstRoomID: RoomID
    public let secondRoomID: RoomID

    public init(id: DoorwayID,
                candidateID: DoorwayCandidateID,
                planePoint: Vec2,
                normalFromFirstRoom: Vec2,
                widthMeters: Double,
                frontierCellCount: Int,
                firstRoomID: RoomID,
                secondRoomID: RoomID) {
        self.id = id
        self.candidateID = candidateID
        self.planePoint = planePoint
        self.normalFromFirstRoom = normalFromFirstRoom
        self.widthMeters = widthMeters
        self.frontierCellCount = frontierCellCount
        self.firstRoomID = firstRoomID
        self.secondRoomID = secondRoomID
    }
}

public struct DoorwayRouteStep: Equatable, Sendable {
    public let doorwayID: DoorwayID
    public let fromRoomID: RoomID
    public let toRoomID: RoomID

    public init(doorwayID: DoorwayID, fromRoomID: RoomID, toRoomID: RoomID) {
        self.doorwayID = doorwayID
        self.fromRoomID = fromRoomID
        self.toRoomID = toRoomID
    }

    public var reversed: DoorwayRouteStep {
        DoorwayRouteStep(
            doorwayID: doorwayID,
            fromRoomID: toRoomID,
            toRoomID: fromRoomID
        )
    }
}

public struct TransitionObservation: Equatable, Sendable {
    public let pose: Pose2D
    public let frameSequence: UInt64
    public let timestamp: TimeInterval
    public let isTrackingNormal: Bool
    public let sessionGeneration: UInt64

    public init(pose: Pose2D,
                frameSequence: UInt64,
                timestamp: TimeInterval,
                isTrackingNormal: Bool,
                sessionGeneration: UInt64) {
        self.pose = pose
        self.frameSequence = frameSequence
        self.timestamp = timestamp
        self.isTrackingNormal = isTrackingNormal
        self.sessionGeneration = sessionGeneration
    }
}

public enum TransitionObservationResult: Equatable, Sendable {
    case ignored
    case observing
    case readyForConfirmation
}

public enum TransitionStartRejectionReason: String, Equatable, Sendable {
    case alreadyActive = "already_active"
    case missingCurrentRoom = "missing_current_room"
    case missingCandidate = "missing_candidate"
    case invalidGeometry = "invalid_geometry"
    case invalidApproachSide = "invalid_approach_side"
}

public enum TransitionStartResult: Equatable, Sendable {
    case started
    case correctedOrientation(DoorwayCandidate)
    case rejected(TransitionStartRejectionReason)
}

public struct DoorwayCandidate: Equatable, Sendable {
    public let id: DoorwayCandidateID
    public let planePoint: Vec2
    public let outwardDirection: Vec2
    public let widthMeters: Double
    public let frontierCellCount: Int
    public let doorwayID: DoorwayID?
    public let oppositeRoomID: RoomID?

    public init(id: DoorwayCandidateID,
                planePoint: Vec2,
                outwardDirection: Vec2,
                widthMeters: Double,
                frontierCellCount: Int,
                doorwayID: DoorwayID? = nil,
                oppositeRoomID: RoomID? = nil) {
        self.id = id
        self.planePoint = planePoint
        self.outwardDirection = outwardDirection
        self.widthMeters = widthMeters
        self.frontierCellCount = frontierCellCount
        self.doorwayID = doorwayID
        self.oppositeRoomID = oppositeRoomID
    }

    // 0.35 m crossing margin + 0.20 m navigation arrival tolerance + 0.05 m allowance.
    public func beyondPlaneGoal(clearance: Double = 0.60) -> Vec2 {
        planePoint + outwardDirection * clearance
    }
}

public struct DoorwayCandidateAssessment: Equatable, Sendable {
    public let candidateID: DoorwayCandidateID
    public let isReachable: Bool
    public let beyondPlaneGoal: Vec2
    public let pathDistance: Double

    public init(candidateID: DoorwayCandidateID,
                isReachable: Bool,
                beyondPlaneGoal: Vec2,
                pathDistance: Double) {
        self.candidateID = candidateID
        self.isReachable = isReachable
        self.beyondPlaneGoal = beyondPlaneGoal
        self.pathDistance = pathDistance
    }
}

public struct RankedDoorwayCandidate: Equatable, Sendable {
    public let candidate: DoorwayCandidate
    public let assessment: DoorwayCandidateAssessment
    public let visualBoost: Double

    public init(candidate: DoorwayCandidate,
                assessment: DoorwayCandidateAssessment,
                visualBoost: Double) {
        self.candidate = candidate
        self.assessment = assessment
        self.visualBoost = visualBoost
    }
}

public struct RoomTopologySnapshot: Equatable, Sendable {
    public let sessionGeneration: UInt64?
    public let currentRoomID: RoomID?
    public let rooms: [Room]
    public let doorways: [Doorway]
    public let candidates: [DoorwayCandidate]
    public let pendingTransitionCandidateID: DoorwayCandidateID?

    public init(sessionGeneration: UInt64?,
                currentRoomID: RoomID?,
                rooms: [Room],
                doorways: [Doorway],
                candidates: [DoorwayCandidate],
                pendingTransitionCandidateID: DoorwayCandidateID? = nil) {
        self.sessionGeneration = sessionGeneration
        self.currentRoomID = currentRoomID
        self.rooms = rooms
        self.doorways = doorways
        self.candidates = candidates
        self.pendingTransitionCandidateID = pendingTransitionCandidateID
    }
}

public typealias RoomTopologyTelemetrySink = (_ event: String, _ fields: [String: String]) -> Void

@MainActor
public protocol RoomTopologyManaging: AnyObject {
    var snapshot: RoomTopologySnapshot { get }

    func startSession(generation: UInt64, initialPose: Pose2D)
    func ingestStablePose(_ pose: Pose2D, sessionGeneration generation: UInt64)
    func reset(forSessionGeneration generation: UInt64)
    func shortestDoorwayPath(from startRoomID: RoomID, to destinationRoomID: RoomID)
        -> [DoorwayRouteStep]?
    func refreshCandidates(from frontiers: [Frontier], referencePose: Pose2D) -> [DoorwayCandidate]
    func rankedCandidates(
        assessments: [DoorwayCandidateAssessment],
        visualBoosts: [DoorwayCandidateID: Double],
        excluding excludedCandidateIDs: Set<DoorwayCandidateID>
    ) -> [RankedDoorwayCandidate]
    @discardableResult
    func beginTransition(
        candidateID: DoorwayCandidateID,
        approachPose: Pose2D
    ) -> TransitionStartResult
    func observeTransition(_ observation: TransitionObservation) -> TransitionObservationResult
    @discardableResult
    func confirmTransition() -> RoomID?
    func rejectTransition(reason: String)
    func abandonTransition()
}

public extension RoomTopologyManaging {
    func shortestDoorwayPath(from startRoomID: RoomID, to destinationRoomID: RoomID)
        -> [DoorwayRouteStep]? {
        nil
    }
}
