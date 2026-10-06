import Foundation

public struct OpticalProtocolContext: Equatable, Sendable {
    public let missionID: UUID
    public let markerID: String
    public let localRole: RoverRole

    public init(missionID: UUID, markerID: String, localRole: RoverRole) {
        self.missionID = missionID
        self.markerID = markerID
        self.localRole = localRole
    }
}

public enum OpticalProtocolPhase: Equatable, Sendable {
    case readyToSendOffer
    case awaitingOffer
    case awaitingAccept
    case readyToSendAccept
    case readyToSendSearchCommit
    case awaitingSearchCommit
    case awaitingSearchAck
    case readyToSendSearchAck
    case searchScheduled(startMilliseconds: Int64, deadlineMilliseconds: Int64)
    case readyToSendStatus
    case awaitingAStatus
    case readyToSendBStatus
    case awaitingBStatus
    case readyToSendDecision
    case awaitingDecision
    case readyToSendConverge
    case awaitingConverge
    case readyToSendConvergeAck
    case awaitingConvergeAck
    case convergenceScheduled(outcome: OpticalDecisionOutcome, releaseMilliseconds: Int64,
                              x: Int32?, y: Int32?)
    case terminalNotFound
    case terminalConflict
}

public enum OpticalProtocolRejection: Error, Equatable, Sendable {
    case codec(OpticalMessageCodecError)
    case unexpectedPhase
    case wrongMission
    case wrongMarker
    case wrongRole
    case sequenceNotIncreasing(last: UInt64, received: UInt64)
    case clockDisagreement
    case invalidSchedule
    case invalidLinkedHash
    case invalidDecision
    case convergenceAfterConflict
    case noOutgoingMessage
}

public struct OpticalSequenceTracker: Equatable, Sendable {
    public private(set) var lastAccepted: UInt64 = 0
    public init() {}

    public func validate(_ sequence: UInt64) throws {
        guard sequence > lastAccepted else {
            throw OpticalProtocolRejection.sequenceNotIncreasing(last: lastAccepted, received: sequence)
        }
    }

    public mutating func accept(_ sequence: UInt64) { lastAccepted = sequence }
}

public struct OutboundOpticalSequence: Equatable, Sendable {
    public private(set) var nextSequence: UInt64 = 1
    public init() {}

    public mutating func allocate() -> UInt64 {
        defer { nextSequence += 1 }
        return nextSequence
    }
}

public struct OpticalProtocolSession: Sendable {
    public let context: OpticalProtocolContext
    public private(set) var phase: OpticalProtocolPhase
    public private(set) var incomingSequence = OpticalSequenceTracker()
    public private(set) var outgoingSequence = OutboundOpticalSequence()

    private let codec = OpticalMessageCodec()
    private let events: (any SilentSearchEventSink)?
    private var lastOutgoing: Data?
    private var offer: OfferBody?
    private var offerHash: String?
    private var acceptanceHash: String?
    private var searchCommitHash: String?
    private var searchStart: Int64?
    private var searchDeadline: Int64?
    private var roverAStatus: StatusBody?
    private var roverAStatusHash: String?
    private var roverBStatus: StatusBody?
    private var roverBStatusHash: String?
    private var decision: DecisionBody?
    private var decisionHash: String?
    private var convergenceRelease: Int64?
    private var convergenceHash: String?

    private static let maximumHandshakeClockDifferenceMilliseconds: Int64 = 15_000

    public init(context: OpticalProtocolContext, events: (any SilentSearchEventSink)? = nil) {
        self.context = context
        self.events = events
        phase = context.localRole == .a ? .readyToSendOffer : .awaitingOffer
    }

    public mutating func retryOutgoing() throws -> Data {
        guard let lastOutgoing else { throw OpticalProtocolRejection.noOutgoingMessage }
        return lastOutgoing
    }

    public mutating func prepareOutgoing(body: OpticalMessageBody, at nowMilliseconds: Int64) throws -> Data {
        let kind = kind(of: body)
        try validateOutgoing(kind: kind, body: body, at: nowMilliseconds)
        let sequence = outgoingSequence.nextSequence
        let message = OpticalMessage(missionID: context.missionID, kind: kind, sequence: sequence,
                                     role: context.localRole, markerID: context.markerID,
                                     timestampMilliseconds: nowMilliseconds, body: body)
        let data: Data
        do { data = try codec.encode(message) }
        catch let error as OpticalMessageCodecError { throw OpticalProtocolRejection.codec(error) }
        outgoingSequence.allocate()
        applyOutgoing(body: body, data: data)
        lastOutgoing = data
        recordProtocol(direction: "outgoing", kind: kind, sequence: sequence, outcome: "accepted")
        return data
    }

    public mutating func receive(_ data: Data, at nowMilliseconds: Int64) throws {
        var decoded: OpticalMessage?
        do {
            let message: OpticalMessage
            do { message = try codec.decode(data) }
            catch let error as OpticalMessageCodecError { throw OpticalProtocolRejection.codec(error) }
            decoded = message

            if phase == .terminalConflict, message.kind == .converge {
                throw OpticalProtocolRejection.convergenceAfterConflict
            }
            guard message.missionID == context.missionID else { throw OpticalProtocolRejection.wrongMission }
            guard message.markerID == context.markerID else { throw OpticalProtocolRejection.wrongMarker }
            guard expectedIncomingKind == message.kind else { throw OpticalProtocolRejection.unexpectedPhase }
            guard message.role != context.localRole else { throw OpticalProtocolRejection.wrongRole }
            do { try codec.validateTimestamp(of: message, nowMilliseconds: nowMilliseconds) }
            catch let error as OpticalMessageCodecError { throw OpticalProtocolRejection.codec(error) }
            try incomingSequence.validate(message.sequence)
            try validateIncoming(message, at: nowMilliseconds)
            incomingSequence.accept(message.sequence)
            applyIncoming(message, data: data)
            recordProtocol(direction: "incoming", kind: message.kind,
                           sequence: message.sequence, outcome: "accepted")
        } catch {
            recordProtocol(
                direction: "incoming",
                kind: decoded?.kind,
                sequence: decoded?.sequence,
                outcome: "rejected",
                reason: String(describing: error)
            )
            throw error
        }
    }

    public mutating func beginRendezvous() throws {
        guard case .searchScheduled = phase else { throw OpticalProtocolRejection.unexpectedPhase }
        phase = context.localRole == .a ? .readyToSendStatus : .awaitingAStatus
    }

    public static func decision(roverA: StatusBody, roverB: StatusBody,
                                roverAHash: String, roverBHash: String) -> DecisionBody {
        switch (roverA.found, roverB.found) {
        case (false, false):
            return DecisionBody(roverAStatusHash: roverAHash, roverBStatusHash: roverBHash, outcome: .notFound)
        case (true, false):
            return DecisionBody(roverAStatusHash: roverAHash, roverBStatusHash: roverBHash,
                                outcome: .found, x: roverA.x, y: roverA.y)
        case (false, true):
            return DecisionBody(roverAStatusHash: roverAHash, roverBStatusHash: roverBHash,
                                outcome: .found, x: roverB.x, y: roverB.y)
        case (true, true):
            let dx = Int64(roverA.x!) - Int64(roverB.x!)
            let dy = Int64(roverA.y!) - Int64(roverB.y!)
            guard abs(dx) <= 500, abs(dy) <= 500, dx * dx + dy * dy <= 500 * 500 else {
                return DecisionBody(roverAStatusHash: roverAHash, roverBStatusHash: roverBHash, outcome: .conflict)
            }
            return DecisionBody(roverAStatusHash: roverAHash, roverBStatusHash: roverBHash, outcome: .found,
                                x: median(roverA.x!, roverB.x!), y: median(roverA.y!, roverB.y!))
        }
    }

    private var expectedIncomingKind: OpticalMessageKind? {
        switch phase {
        case .awaitingOffer: .offer
        case .awaitingAccept: .accept
        case .awaitingSearchCommit: .searchCommit
        case .awaitingSearchAck: .searchAck
        case .searchScheduled where context.localRole == .b: .searchCommit
        case .awaitingAStatus, .awaitingBStatus: .status
        case .awaitingDecision: .decision
        case .awaitingConverge: .converge
        case .awaitingConvergeAck: .convergeAck
        default: nil
        }
    }

    private func validateOutgoing(kind: OpticalMessageKind, body: OpticalMessageBody,
                                  at now: Int64) throws {
        if phase == .terminalConflict, kind == .converge {
            throw OpticalProtocolRejection.convergenceAfterConflict
        }
        let validPhase: Bool
        switch (phase, kind) {
        case (.readyToSendOffer, .offer), (.readyToSendAccept, .accept),
             (.readyToSendSearchCommit, .searchCommit), (.awaitingSearchAck, .searchCommit),
             (.readyToSendSearchAck, .searchAck),
             (.readyToSendStatus, .status), (.readyToSendBStatus, .status),
             (.readyToSendDecision, .decision), (.readyToSendConverge, .converge),
             (.readyToSendConvergeAck, .convergeAck): validPhase = true
        default: validPhase = false
        }
        guard validPhase else { throw OpticalProtocolRejection.unexpectedPhase }
        switch body {
        case let .accept(value):
            guard value.offerHash == offerHash else { throw OpticalProtocolRejection.invalidLinkedHash }
            guard value.roverBWallTimeMilliseconds == now else { throw OpticalProtocolRejection.clockDisagreement }
        case let .searchCommit(value):
            guard value.acceptanceHash == acceptanceHash else { throw OpticalProtocolRejection.invalidLinkedHash }
            try validateSchedule(start: value.startMilliseconds, deadline: value.deadlineMilliseconds, at: now)
        case let .searchAck(value):
            guard value.hash == searchCommitHash else { throw OpticalProtocolRejection.invalidLinkedHash }
        case let .status(value):
            if context.localRole == .a {
                guard value.previousStatusHash == nil else { throw OpticalProtocolRejection.invalidLinkedHash }
            } else {
                guard value.previousStatusHash == roverAStatusHash else { throw OpticalProtocolRejection.invalidLinkedHash }
            }
            if value.found, value.label != offer?.targetLabel { throw OpticalProtocolRejection.invalidDecision }
        case let .decision(value):
            guard let roverAStatus, let roverBStatus, let roverAStatusHash, let roverBStatusHash,
                  value == Self.decision(roverA: roverAStatus, roverB: roverBStatus,
                                         roverAHash: roverAStatusHash, roverBHash: roverBStatusHash) else {
                throw OpticalProtocolRejection.invalidDecision
            }
        case let .converge(value):
            guard let decision, value.decisionHash == decisionHash else { throw OpticalProtocolRejection.invalidLinkedHash }
            if decision.outcome == .conflict { throw OpticalProtocolRejection.convergenceAfterConflict }
            guard isAtLeast(value.releaseMilliseconds, milliseconds: 30_000, after: now) else {
                throw OpticalProtocolRejection.invalidSchedule
            }
            guard (decision.outcome == .found && value.x == decision.x && value.y == decision.y) ||
                    (decision.outcome == .notFound && value.x == nil && value.y == nil) else {
                throw OpticalProtocolRejection.invalidDecision
            }
        case let .convergeAck(value):
            guard value.hash == convergenceHash else { throw OpticalProtocolRejection.invalidLinkedHash }
        case .offer: break
        }
    }

    private func validateIncoming(_ message: OpticalMessage, at now: Int64) throws {
        switch message.body {
        case let .offer(value):
            guard absDifference(now, message.timestampMilliseconds) <= Self.maximumHandshakeClockDifferenceMilliseconds else {
                throw OpticalProtocolRejection.clockDisagreement
            }
            guard message.role == .a else { throw OpticalProtocolRejection.wrongRole }
            _ = value
        case let .accept(value):
            guard message.role == .b else { throw OpticalProtocolRejection.wrongRole }
            guard value.offerHash == offerHash else { throw OpticalProtocolRejection.invalidLinkedHash }
            guard value.roverBWallTimeMilliseconds == message.timestampMilliseconds,
                  absDifference(now, value.roverBWallTimeMilliseconds) <= Self.maximumHandshakeClockDifferenceMilliseconds else {
                throw OpticalProtocolRejection.clockDisagreement
            }
        case let .searchCommit(value):
            guard message.role == .a else { throw OpticalProtocolRejection.wrongRole }
            if case let .searchScheduled(previousStart, _) = phase {
                guard now < previousStart else { throw OpticalProtocolRejection.invalidSchedule }
            }
            guard value.acceptanceHash == acceptanceHash else { throw OpticalProtocolRejection.invalidLinkedHash }
            try validateSchedule(start: value.startMilliseconds, deadline: value.deadlineMilliseconds, at: now)
        case let .searchAck(value):
            guard message.role == .b else { throw OpticalProtocolRejection.wrongRole }
            guard value.hash == searchCommitHash else { throw OpticalProtocolRejection.invalidLinkedHash }
            guard let searchStart, isAtLeast(searchStart, milliseconds: 5_000, after: now) else {
                throw OpticalProtocolRejection.invalidSchedule
            }
        case let .status(value):
            if message.role == .a {
                guard phase == .awaitingAStatus, value.previousStatusHash == nil else {
                    throw OpticalProtocolRejection.wrongRole
                }
            } else {
                guard phase == .awaitingBStatus, value.previousStatusHash == roverAStatusHash else {
                    throw OpticalProtocolRejection.invalidLinkedHash
                }
            }
            if value.found, value.label != offer?.targetLabel { throw OpticalProtocolRejection.invalidDecision }
        case let .decision(value):
            guard message.role == .a, let roverAStatus, let roverBStatus,
                  let roverAStatusHash, let roverBStatusHash,
                  value == Self.decision(roverA: roverAStatus, roverB: roverBStatus,
                                         roverAHash: roverAStatusHash, roverBHash: roverBStatusHash) else {
                throw OpticalProtocolRejection.invalidDecision
            }
        case let .converge(value):
            guard message.role == .a, let decision, value.decisionHash == decisionHash else {
                throw OpticalProtocolRejection.invalidLinkedHash
            }
            if decision.outcome == .conflict { throw OpticalProtocolRejection.convergenceAfterConflict }
            guard isAtLeast(value.releaseMilliseconds, milliseconds: 30_000, after: now) else {
                throw OpticalProtocolRejection.invalidSchedule
            }
            guard (decision.outcome == .found && value.x == decision.x && value.y == decision.y) ||
                    (decision.outcome == .notFound && value.x == nil && value.y == nil) else {
                throw OpticalProtocolRejection.invalidDecision
            }
        case let .convergeAck(value):
            guard message.role == .b, value.hash == convergenceHash else {
                throw OpticalProtocolRejection.invalidLinkedHash
            }
            guard let convergenceRelease, isAtLeast(convergenceRelease, milliseconds: 5_000, after: now) else {
                throw OpticalProtocolRejection.invalidSchedule
            }
        }
    }

    private mutating func applyOutgoing(body: OpticalMessageBody, data: Data) {
        switch body {
        case let .offer(value):
            offer = value; offerHash = codec.messageLinkHash(for: data); phase = .awaitingAccept
        case .accept:
            acceptanceHash = codec.messageLinkHash(for: data); phase = .awaitingSearchCommit
        case let .searchCommit(value):
            searchStart = value.startMilliseconds; searchDeadline = value.deadlineMilliseconds
            searchCommitHash = codec.messageLinkHash(for: data); phase = .awaitingSearchAck
        case .searchAck:
            phase = .searchScheduled(startMilliseconds: searchStart!, deadlineMilliseconds: searchDeadline!)
        case let .status(value):
            if context.localRole == .a {
                roverAStatus = value; roverAStatusHash = codec.messageLinkHash(for: data); phase = .awaitingBStatus
            } else {
                roverBStatus = value; roverBStatusHash = codec.messageLinkHash(for: data); phase = .awaitingDecision
            }
        case let .decision(value):
            decision = value; decisionHash = codec.messageLinkHash(for: data)
            phase = value.outcome == .conflict ? .terminalConflict : .readyToSendConverge
        case let .converge(value):
            convergenceRelease = value.releaseMilliseconds; convergenceHash = codec.messageLinkHash(for: data)
            phase = .awaitingConvergeAck
        case .convergeAck:
            phase = terminalPhaseForAcknowledgedConvergence()
        }
    }

    private mutating func applyIncoming(_ message: OpticalMessage, data: Data) {
        switch message.body {
        case let .offer(value):
            offer = value; offerHash = codec.messageLinkHash(for: data); phase = .readyToSendAccept
        case .accept:
            acceptanceHash = codec.messageLinkHash(for: data); phase = .readyToSendSearchCommit
        case let .searchCommit(value):
            searchStart = value.startMilliseconds; searchDeadline = value.deadlineMilliseconds
            searchCommitHash = codec.messageLinkHash(for: data); phase = .readyToSendSearchAck
        case .searchAck:
            phase = .searchScheduled(startMilliseconds: searchStart!, deadlineMilliseconds: searchDeadline!)
        case let .status(value):
            if message.role == .a {
                roverAStatus = value; roverAStatusHash = codec.messageLinkHash(for: data); phase = .readyToSendBStatus
            } else {
                roverBStatus = value; roverBStatusHash = codec.messageLinkHash(for: data); phase = .readyToSendDecision
            }
        case let .decision(value):
            decision = value; decisionHash = codec.messageLinkHash(for: data)
            phase = value.outcome == .conflict ? .terminalConflict : .awaitingConverge
        case let .converge(value):
            convergenceRelease = value.releaseMilliseconds; convergenceHash = codec.messageLinkHash(for: data)
            phase = .readyToSendConvergeAck
        case .convergeAck:
            phase = terminalPhaseForAcknowledgedConvergence()
        }
    }

    private func validateSchedule(start: Int64, deadline: Int64, at now: Int64) throws {
        guard let offer, isAtLeast(start, milliseconds: 30_000, after: now) else {
            throw OpticalProtocolRejection.invalidSchedule
        }
        let expected = start.addingReportingOverflow(Int64(offer.searchDurationSeconds) * 1_000)
        guard !expected.overflow, deadline == expected.partialValue else {
            throw OpticalProtocolRejection.invalidSchedule
        }
    }

    private func terminalPhaseForAcknowledgedConvergence() -> OpticalProtocolPhase {
        guard let decision, let convergenceRelease else { return .terminalConflict }
        if decision.outcome == .notFound { return .terminalNotFound }
        return .convergenceScheduled(outcome: decision.outcome, releaseMilliseconds: convergenceRelease,
                                     x: decision.x, y: decision.y)
    }

    private func kind(of body: OpticalMessageBody) -> OpticalMessageKind {
        switch body {
        case .offer: .offer
        case .accept: .accept
        case .searchCommit: .searchCommit
        case .searchAck: .searchAck
        case .status: .status
        case .decision: .decision
        case .converge: .converge
        case .convergeAck: .convergeAck
        }
    }

    private func recordProtocol(direction: String, kind: OpticalMessageKind?, sequence: UInt64?,
                                outcome: String, reason: String? = nil) {
        var fields = [
            "direction": direction,
            "kind": kind?.rawValue ?? "unknown",
            "marker": context.markerID,
            "mission": context.missionID.uuidString.lowercased(),
            "outcome": outcome,
            "role": context.localRole.rawValue,
            "sequence": sequence.map(String.init) ?? "unknown",
        ]
        if let reason { fields["reason"] = reason }
        events?.record(event: "silent_search_protocol", fields: fields)
    }

    private func absDifference(_ lhs: Int64, _ rhs: Int64) -> UInt64 {
        let result = lhs >= rhs
            ? lhs.subtractingReportingOverflow(rhs)
            : rhs.subtractingReportingOverflow(lhs)
        return result.overflow ? .max : UInt64(result.partialValue)
    }

    private func isAtLeast(_ later: Int64, milliseconds: Int64, after earlier: Int64) -> Bool {
        let boundary = earlier.addingReportingOverflow(milliseconds)
        return !boundary.overflow && later >= boundary.partialValue
    }

    private static func median(_ lhs: Int32, _ rhs: Int32) -> Int32 {
        Int32((Int64(lhs) + Int64(rhs)) / 2)
    }
}
