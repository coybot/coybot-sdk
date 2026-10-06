import Foundation

public enum OpticalMessageKind: String, CaseIterable, Codable, Sendable {
    case offer
    case accept
    case searchCommit
    case searchAck
    case status
    case decision
    case converge
    case convergeAck
}

public struct OpticalPose: Equatable, Codable, Sendable {
    public let x: Int32
    public let y: Int32
    public let headingMillidegrees: Int32

    public init(x: Int32, y: Int32, headingMillidegrees: Int32) {
        self.x = x
        self.y = y
        self.headingMillidegrees = headingMillidegrees
    }

    enum CodingKeys: String, CodingKey { case x, y; case headingMillidegrees = "h" }
}

public struct OfferBody: Equatable, Codable, Sendable {
    public let searchDurationSeconds: UInt16
    public let centerHalfWidthMillimeters: Int32
    public let targetLabel: String
    public let markerWidthMillimeters: Int32
    public let roverARendezvous: OpticalPose
    public let roverBRendezvous: OpticalPose

    public init(searchDurationSeconds: UInt16, centerHalfWidthMillimeters: Int32,
                targetLabel: String, markerWidthMillimeters: Int32,
                roverARendezvous: OpticalPose, roverBRendezvous: OpticalPose) {
        self.searchDurationSeconds = searchDurationSeconds
        self.centerHalfWidthMillimeters = centerHalfWidthMillimeters
        self.targetLabel = targetLabel
        self.markerWidthMillimeters = markerWidthMillimeters
        self.roverARendezvous = roverARendezvous
        self.roverBRendezvous = roverBRendezvous
    }

    enum CodingKeys: String, CodingKey {
        case searchDurationSeconds = "d"
        case centerHalfWidthMillimeters = "e"
        case targetLabel = "n"
        case markerWidthMillimeters = "w"
        case roverARendezvous = "ra"
        case roverBRendezvous = "rb"
    }
}

public struct AcceptBody: Equatable, Codable, Sendable {
    public let offerHash: String
    public let roverBWallTimeMilliseconds: Int64

    public init(offerHash: String, roverBWallTimeMilliseconds: Int64) {
        self.offerHash = offerHash
        self.roverBWallTimeMilliseconds = roverBWallTimeMilliseconds
    }

    enum CodingKeys: String, CodingKey { case offerHash = "h"; case roverBWallTimeMilliseconds = "u" }
}

public struct SearchCommitBody: Equatable, Codable, Sendable {
    public let deadlineMilliseconds: Int64
    public let acceptanceHash: String
    public let startMilliseconds: Int64

    public init(deadlineMilliseconds: Int64, acceptanceHash: String, startMilliseconds: Int64) {
        self.deadlineMilliseconds = deadlineMilliseconds
        self.acceptanceHash = acceptanceHash
        self.startMilliseconds = startMilliseconds
    }

    enum CodingKeys: String, CodingKey {
        case deadlineMilliseconds = "d"
        case acceptanceHash = "h"
        case startMilliseconds = "s"
    }
}

public struct HashAcknowledgementBody: Equatable, Codable, Sendable {
    public let hash: String
    public init(hash: String) { self.hash = hash }
    enum CodingKeys: String, CodingKey { case hash = "h" }
}

public struct StatusBody: Equatable, Codable, Sendable {
    public let found: Bool
    public let previousStatusHash: String?
    public let label: String?
    public let x: Int32?
    public let y: Int32?
    public let confidenceBasisPoints: UInt16?
    public let sampleCount: UInt16?

    public init(found: Bool, previousStatusHash: String? = nil, label: String? = nil,
                x: Int32? = nil, y: Int32? = nil, confidenceBasisPoints: UInt16? = nil,
                sampleCount: UInt16? = nil) {
        self.found = found
        self.previousStatusHash = previousStatusHash
        self.label = label
        self.x = x
        self.y = y
        self.confidenceBasisPoints = confidenceBasisPoints
        self.sampleCount = sampleCount
    }

    enum CodingKeys: String, CodingKey {
        case found = "f"; case previousStatusHash = "p"; case label = "n"
        case x, y; case confidenceBasisPoints = "c"; case sampleCount = "z"
    }
}

public enum OpticalDecisionOutcome: String, Codable, Sendable {
    case found
    case notFound
    case conflict
}

public struct DecisionBody: Equatable, Codable, Sendable {
    public let roverAStatusHash: String
    public let roverBStatusHash: String
    public let outcome: OpticalDecisionOutcome
    public let x: Int32?
    public let y: Int32?

    public init(roverAStatusHash: String, roverBStatusHash: String,
                outcome: OpticalDecisionOutcome, x: Int32? = nil, y: Int32? = nil) {
        self.roverAStatusHash = roverAStatusHash
        self.roverBStatusHash = roverBStatusHash
        self.outcome = outcome
        self.x = x
        self.y = y
    }

    enum CodingKeys: String, CodingKey {
        case roverAStatusHash = "a"; case roverBStatusHash = "b"; case outcome = "o"; case x, y
    }
}

public struct ConvergeBody: Equatable, Codable, Sendable {
    public let decisionHash: String
    public let releaseMilliseconds: Int64
    public let x: Int32?
    public let y: Int32?

    public init(decisionHash: String, releaseMilliseconds: Int64, x: Int32? = nil, y: Int32? = nil) {
        self.decisionHash = decisionHash
        self.releaseMilliseconds = releaseMilliseconds
        self.x = x
        self.y = y
    }

    enum CodingKeys: String, CodingKey { case decisionHash = "h"; case releaseMilliseconds = "s"; case x, y }
}

public enum OpticalMessageBody: Equatable, Sendable {
    case offer(OfferBody)
    case accept(AcceptBody)
    case searchCommit(SearchCommitBody)
    case searchAck(HashAcknowledgementBody)
    case status(StatusBody)
    case decision(DecisionBody)
    case converge(ConvergeBody)
    case convergeAck(HashAcknowledgementBody)
}

public struct OpticalMessage: Equatable, Sendable {
    public static let protocolVersion = 1

    public let version: Int
    public let missionID: UUID
    public let kind: OpticalMessageKind
    public let sequence: UInt64
    public let role: RoverRole
    public let markerID: String
    public let timestampMilliseconds: Int64
    public let body: OpticalMessageBody

    public init(version: Int = protocolVersion, missionID: UUID, kind: OpticalMessageKind,
                sequence: UInt64, role: RoverRole, markerID: String,
                timestampMilliseconds: Int64, body: OpticalMessageBody) {
        self.version = version
        self.missionID = missionID
        self.kind = kind
        self.sequence = sequence
        self.role = role
        self.markerID = markerID
        self.timestampMilliseconds = timestampMilliseconds
        self.body = body
    }
}
