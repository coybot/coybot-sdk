import XCTest
@testable import PhroverKit

final class OpticalProtocolSessionTests: XCTestCase {
    private let missionID = UUID(uuidString: "10000000-0000-0000-0000-000000000001")!
    private let markerID = "SILENT_SEARCH_01"
    private let now: Int64 = 1_786_406_400_000

    func testHandshakeProducesLinkedMessagesAndAcknowledgedSchedule() throws {
        var roverA = OpticalProtocolSession(context: context(.a))
        var roverB = OpticalProtocolSession(context: context(.b))

        let offer = try roverA.prepareOutgoing(
            body: .offer(OfferBody(searchDurationSeconds: 180, centerHalfWidthMillimeters: 250,
                targetLabel: "chair", markerWidthMillimeters: 200,
                roverARendezvous: OpticalPose(x: -600, y: -800, headingMillidegrees: 0),
                roverBRendezvous: OpticalPose(x: 600, y: -800, headingMillidegrees: 0))),
            at: now
        )
        try roverB.receive(offer, at: now + 2_000)
        let accept = try roverB.prepareOutgoing(
            body: .accept(AcceptBody(offerHash: OpticalMessageCodec().messageLinkHash(for: offer),
                                     roverBWallTimeMilliseconds: now + 2_000)),
            at: now + 2_000
        )
        try roverA.receive(accept, at: now + 4_000)
        let acceptHash = OpticalMessageCodec().messageLinkHash(for: accept)
        let commit = try roverA.prepareOutgoing(
            body: .searchCommit(SearchCommitBody(deadlineMilliseconds: now + 214_000,
                acceptanceHash: acceptHash, startMilliseconds: now + 34_000)),
            at: now + 4_000
        )
        try roverB.receive(commit, at: now + 4_000)
        let acknowledgement = try roverB.prepareOutgoing(
            body: .searchAck(HashAcknowledgementBody(hash: OpticalMessageCodec().messageLinkHash(for: commit))),
            at: now + 5_000
        )
        try roverA.receive(acknowledgement, at: now + 29_000)

        XCTAssertEqual(roverA.phase, .searchScheduled(startMilliseconds: now + 34_000,
                                                       deadlineMilliseconds: now + 214_000))
        XCTAssertEqual(roverB.phase, roverA.phase)
    }

    func testRetryIsByteIdenticalAndDoesNotAdvanceSequence() throws {
        var roverA = OpticalProtocolSession(context: context(.a))
        let offer = try roverA.prepareOutgoing(body: offerBody(), at: now)

        XCTAssertEqual(try roverA.retryOutgoing(), offer)
        XCTAssertEqual(roverA.outgoingSequence.nextSequence, 2)
        XCTAssertThrowsError(try roverA.prepareOutgoing(body: offerBody(), at: now + 1)) {
            XCTAssertEqual($0 as? OpticalProtocolRejection, .unexpectedPhase)
        }
        XCTAssertEqual(roverA.outgoingSequence.nextSequence, 2)
    }

    func testInvalidScanDoesNotConsumeIncomingSequenceOrChangePhase() throws {
        var roverB = OpticalProtocolSession(context: context(.b))
        let wrongMission = OpticalMessage(
            missionID: UUID(uuidString: "20000000-0000-0000-0000-000000000002")!, kind: .offer,
            sequence: 1, role: .a, markerID: markerID, timestampMilliseconds: now, body: offerBody()
        )
        let invalid = try OpticalMessageCodec().encode(wrongMission)

        XCTAssertThrowsError(try roverB.receive(invalid, at: now)) {
            XCTAssertEqual($0 as? OpticalProtocolRejection, .wrongMission)
        }
        XCTAssertEqual(roverB.phase, .awaitingOffer)
        XCTAssertEqual(roverB.incomingSequence.lastAccepted, 0)

        var roverA = OpticalProtocolSession(context: context(.a))
        try roverB.receive(roverA.prepareOutgoing(body: offerBody(), at: now), at: now)
        XCTAssertEqual(roverB.incomingSequence.lastAccepted, 1)
    }

    func testBoundRoverBStillRejectsAnotherMission() throws {
        let boundID = UUID(uuidString: "30000000-0000-0000-0000-000000000003")!
        var roverA = OpticalProtocolSession(context: OpticalProtocolContext(
            missionID: boundID, markerID: markerID, localRole: .a))
        var roverB = OpticalProtocolSession(context: OpticalProtocolContext(
            missionID: boundID, markerID: markerID, localRole: .b))
        try roverB.receive(roverA.prepareOutgoing(body: offerBody(), at: now), at: now)
        let wrong = OpticalMessage(
            missionID: UUID(), kind: .offer, sequence: 2, role: .a,
            markerID: markerID, timestampMilliseconds: now, body: offerBody())

        XCTAssertThrowsError(try roverB.receive(OpticalMessageCodec().encode(wrong), at: now)) {
            XCTAssertEqual($0 as? OpticalProtocolRejection, .wrongMission)
        }
    }

    func testStructuredTelemetryRecordsKindSequenceOutcomeAndRejectionWithoutPayload() throws {
        let sink = RecordingProtocolSink()
        var roverA = OpticalProtocolSession(context: context(.a), events: sink)
        var roverB = OpticalProtocolSession(context: context(.b), events: sink)
        let offer = try roverA.prepareOutgoing(body: offerBody(), at: now)
        try roverB.receive(offer, at: now)

        let wrongMission = OpticalMessage(
            missionID: UUID(uuidString: "20000000-0000-0000-0000-000000000002")!, kind: .offer,
            sequence: 2, role: .a, markerID: markerID, timestampMilliseconds: now, body: offerBody()
        )
        var invalidSession = OpticalProtocolSession(context: context(.b), events: sink)
        XCTAssertThrowsError(try invalidSession.receive(OpticalMessageCodec().encode(wrongMission), at: now))

        XCTAssertEqual(sink.entries[0].fields["direction"], "outgoing")
        XCTAssertEqual(sink.entries[0].fields["kind"], "offer")
        XCTAssertEqual(sink.entries[0].fields["sequence"], "1")
        XCTAssertEqual(sink.entries[1].fields["outcome"], "accepted")
        XCTAssertEqual(sink.entries[2].fields["outcome"], "rejected")
        XCTAssertEqual(sink.entries[2].fields["reason"], "wrongMission")
        XCTAssertTrue(sink.entries.allSatisfy { $0.fields["payload"] == nil && $0.fields["image"] == nil })
    }

    func testOfferAndAcceptanceClockBoundariesAreExact() throws {
        var roverA = OpticalProtocolSession(context: context(.a))
        let offer = try roverA.prepareOutgoing(body: offerBody(), at: now)
        var atBoundary = OpticalProtocolSession(context: context(.b))
        XCTAssertNoThrow(try atBoundary.receive(offer, at: now + 15_000))
        var beyondBoundary = OpticalProtocolSession(context: context(.b))
        XCTAssertThrowsError(try beyondBoundary.receive(offer, at: now + 15_001)) {
            XCTAssertEqual($0 as? OpticalProtocolRejection, .clockDisagreement)
        }

        var roverB = OpticalProtocolSession(context: context(.b))
        try roverB.receive(offer, at: now)
        let accept = try roverB.prepareOutgoing(body: .accept(AcceptBody(
            offerHash: OpticalMessageCodec().messageLinkHash(for: offer),
            roverBWallTimeMilliseconds: now)), at: now)
        var acceptedA = roverA
        XCTAssertNoThrow(try acceptedA.receive(accept, at: now + 15_000))
        XCTAssertThrowsError(try roverA.receive(accept, at: now + 15_001)) {
            XCTAssertEqual($0 as? OpticalProtocolRejection, .clockDisagreement)
        }
    }

    func testScheduleAndAcknowledgementRejectOneMillisecondOutsideBoundaries() throws {
        var sessions = try sessionsThroughAcceptance()
        let acceptHash = OpticalMessageCodec().messageLinkHash(for: try sessions.b.retryOutgoing())
        XCTAssertThrowsError(try sessions.a.prepareOutgoing(body: .searchCommit(SearchCommitBody(
            deadlineMilliseconds: now + 209_999, acceptanceHash: acceptHash,
            startMilliseconds: now + 29_999)), at: now)) {
            XCTAssertEqual($0 as? OpticalProtocolRejection, .invalidSchedule)
        }

        let commit = try sessions.a.prepareOutgoing(body: .searchCommit(SearchCommitBody(
            deadlineMilliseconds: now + 210_000, acceptanceHash: acceptHash,
            startMilliseconds: now + 30_000)), at: now)
        try sessions.b.receive(commit, at: now)
        let ack = try sessions.b.prepareOutgoing(body: .searchAck(HashAcknowledgementBody(
            hash: OpticalMessageCodec().messageLinkHash(for: commit))), at: now)
        var boundaryA = sessions.a
        XCTAssertNoThrow(try boundaryA.receive(ack, at: now + 25_000))
        XCTAssertThrowsError(try sessions.a.receive(ack, at: now + 25_001)) {
            XCTAssertEqual($0 as? OpticalProtocolRejection, .invalidSchedule)
        }
    }

    func testReceiverRetainsThirtySecondScheduleValidation() throws {
        var sessions = try sessionsThroughAcceptance()
        let acceptHash = OpticalMessageCodec().messageLinkHash(for: try sessions.b.retryOutgoing())
        let commit = try sessions.a.prepareOutgoing(body: .searchCommit(SearchCommitBody(
            deadlineMilliseconds: now + 210_000, acceptanceHash: acceptHash,
            startMilliseconds: now + 30_000)), at: now)

        XCTAssertNoThrow(try sessions.b.receive(commit, at: now))
    }

    func testLateSearchAcknowledgementCanBeReplacedWithoutCorruptingSequenceOrPhase() throws {
        var sessions = try sessionsThroughAcceptance()
        let acceptHash = OpticalMessageCodec().messageLinkHash(for: try sessions.b.retryOutgoing())
        let initialCommit = try sessions.a.prepareOutgoing(body: .searchCommit(SearchCommitBody(
            deadlineMilliseconds: now + 210_000, acceptanceHash: acceptHash,
            startMilliseconds: now + 30_000)), at: now)
        try sessions.b.receive(initialCommit, at: now)
        let lateAcknowledgement = try sessions.b.prepareOutgoing(body: .searchAck(HashAcknowledgementBody(
            hash: OpticalMessageCodec().messageLinkHash(for: initialCommit))), at: now)

        XCTAssertThrowsError(try sessions.a.receive(lateAcknowledgement, at: now + 25_001)) {
            XCTAssertEqual($0 as? OpticalProtocolRejection, .invalidSchedule)
        }
        XCTAssertEqual(sessions.a.phase, .awaitingSearchAck)
        XCTAssertEqual(sessions.a.incomingSequence.lastAccepted, 1)
        XCTAssertEqual(sessions.a.outgoingSequence.nextSequence, 3)

        let replacementStart = now + 55_001
        let replacementCommit = try sessions.a.prepareOutgoing(body: .searchCommit(SearchCommitBody(
            deadlineMilliseconds: replacementStart + 180_000, acceptanceHash: acceptHash,
            startMilliseconds: replacementStart)), at: now + 25_001)
        XCTAssertEqual(try OpticalMessageCodec().decode(replacementCommit).sequence, 3)
        XCTAssertEqual(sessions.a.phase, .awaitingSearchAck)
        XCTAssertEqual(sessions.a.outgoingSequence.nextSequence, 4)
        XCTAssertEqual(try sessions.a.retryOutgoing(), replacementCommit)

        try sessions.b.receive(replacementCommit, at: now + 25_001)
        let replacementAcknowledgement = try sessions.b.prepareOutgoing(body: .searchAck(HashAcknowledgementBody(
            hash: OpticalMessageCodec().messageLinkHash(for: replacementCommit))), at: now + 25_001)
        try sessions.a.receive(replacementAcknowledgement, at: replacementStart - 5_000)

        let expected = OpticalProtocolPhase.searchScheduled(
            startMilliseconds: replacementStart, deadlineMilliseconds: replacementStart + 180_000)
        XCTAssertEqual(sessions.a.phase, expected)
        XCTAssertEqual(sessions.b.phase, expected)
        XCTAssertEqual(sessions.a.incomingSequence.lastAccepted, 3)
        XCTAssertEqual(sessions.b.incomingSequence.lastAccepted, 3)
    }

    func testRendezvousDeterministicallyHandlesEveryOutcome() throws {
        let foundA = StatusBody(found: true, label: "chair", x: -100, y: 200,
                                confidenceBasisPoints: 9500, sampleCount: 3)
        let foundBNear = StatusBody(found: true, previousStatusHash: String(repeating: "0", count: 64),
                                    label: "chair", x: 300, y: 400,
                                    confidenceBasisPoints: 9600, sampleCount: 4)
        let foundBFar = StatusBody(found: true, previousStatusHash: String(repeating: "0", count: 64),
                                   label: "chair", x: 401, y: 600,
                                   confidenceBasisPoints: 9700, sampleCount: 5)
        let notFoundA = StatusBody(found: false)

        let soleA = try completeRendezvous(aStatus: foundA, bStatusTemplate: StatusBody(found: false,
            previousStatusHash: String(repeating: "0", count: 64)))
        XCTAssertEqual(soleA.a.phase, .convergenceScheduled(outcome: .found,
            releaseMilliseconds: now + 130_000, x: -100, y: 200))

        let soleB = try completeRendezvous(aStatus: notFoundA, bStatusTemplate: foundBNear)
        XCTAssertEqual(soleB.a.phase, .convergenceScheduled(outcome: .found,
            releaseMilliseconds: now + 130_000, x: 300, y: 400))

        let dual = try completeRendezvous(aStatus: foundA, bStatusTemplate: foundBNear)
        XCTAssertEqual(dual.a.phase, .convergenceScheduled(outcome: .found,
            releaseMilliseconds: now + 130_000, x: 100, y: 300))

        let conflict = try completeRendezvous(aStatus: foundA, bStatusTemplate: foundBFar)
        XCTAssertEqual(conflict.a.phase, .terminalConflict)
        XCTAssertEqual(conflict.b.phase, .terminalConflict)

        let neither = try completeRendezvous(aStatus: notFoundA, bStatusTemplate: StatusBody(found: false,
            previousStatusHash: String(repeating: "0", count: 64)))
        XCTAssertEqual(neither.a.phase, .terminalNotFound)
        XCTAssertEqual(neither.b.phase, .terminalNotFound)
    }

    func testConvergenceIsRejectedAfterConflictWithoutChangingStateOrSequence() throws {
        let foundA = StatusBody(found: true, label: "chair", x: -100, y: 200,
                                confidenceBasisPoints: 9500, sampleCount: 3)
        let foundBFar = StatusBody(found: true, previousStatusHash: String(repeating: "0", count: 64),
                                   label: "chair", x: 401, y: 600,
                                   confidenceBasisPoints: 9700, sampleCount: 5)
        var sessions = try completeRendezvous(aStatus: foundA, bStatusTemplate: foundBFar)
        let nextSequence = sessions.a.outgoingSequence.nextSequence

        XCTAssertThrowsError(try sessions.a.prepareOutgoing(body: .converge(ConvergeBody(
            decisionHash: String(repeating: "a", count: 64),
            releaseMilliseconds: now + 130_000, x: 0, y: 0)), at: now + 100_000)) {
            XCTAssertEqual($0 as? OpticalProtocolRejection, .convergenceAfterConflict)
        }
        XCTAssertEqual(sessions.a.phase, .terminalConflict)
        XCTAssertEqual(sessions.a.outgoingSequence.nextSequence, nextSequence)
    }

    func testSequenceTrackerAllowsGapsAndRejectsReplay() throws {
        var tracker = OpticalSequenceTracker()
        try tracker.validate(5)
        tracker.accept(5)
        XCTAssertNoThrow(try tracker.validate(9))
        XCTAssertThrowsError(try tracker.validate(5)) {
            XCTAssertEqual($0 as? OpticalProtocolRejection, .sequenceNotIncreasing(last: 5, received: 5))
        }
        XCTAssertThrowsError(try tracker.validate(4))
    }

    private func sessionsThroughAcceptance() throws -> (a: OpticalProtocolSession, b: OpticalProtocolSession) {
        var a = OpticalProtocolSession(context: context(.a))
        var b = OpticalProtocolSession(context: context(.b))
        let offer = try a.prepareOutgoing(body: offerBody(), at: now)
        try b.receive(offer, at: now)
        let accept = try b.prepareOutgoing(body: .accept(AcceptBody(
            offerHash: OpticalMessageCodec().messageLinkHash(for: offer),
            roverBWallTimeMilliseconds: now)), at: now)
        try a.receive(accept, at: now)
        return (a, b)
    }

    private func scheduledSessions() throws -> (a: OpticalProtocolSession, b: OpticalProtocolSession) {
        var sessions = try sessionsThroughAcceptance()
        let accept = try sessions.b.retryOutgoing()
        let commit = try sessions.a.prepareOutgoing(body: .searchCommit(SearchCommitBody(
            deadlineMilliseconds: now + 210_000,
            acceptanceHash: OpticalMessageCodec().messageLinkHash(for: accept),
            startMilliseconds: now + 30_000)), at: now)
        try sessions.b.receive(commit, at: now)
        let ack = try sessions.b.prepareOutgoing(body: .searchAck(HashAcknowledgementBody(
            hash: OpticalMessageCodec().messageLinkHash(for: commit))), at: now)
        try sessions.a.receive(ack, at: now + 25_000)
        return sessions
    }

    private func completeRendezvous(aStatus: StatusBody,
                                    bStatusTemplate: StatusBody) throws
        -> (a: OpticalProtocolSession, b: OpticalProtocolSession) {
        var sessions = try scheduledSessions()
        try sessions.a.beginRendezvous()
        try sessions.b.beginRendezvous()
        let aData = try sessions.a.prepareOutgoing(body: .status(aStatus), at: now + 100_000)
        try sessions.b.receive(aData, at: now + 100_000)
        let aHash = OpticalMessageCodec().messageLinkHash(for: aData)
        let bStatus = StatusBody(found: bStatusTemplate.found, previousStatusHash: aHash,
            label: bStatusTemplate.label, x: bStatusTemplate.x, y: bStatusTemplate.y,
            confidenceBasisPoints: bStatusTemplate.confidenceBasisPoints,
            sampleCount: bStatusTemplate.sampleCount)
        let bData = try sessions.b.prepareOutgoing(body: .status(bStatus), at: now + 100_000)
        try sessions.a.receive(bData, at: now + 100_000)
        let decision = OpticalProtocolSession.decision(roverA: aStatus, roverB: bStatus,
            roverAHash: aHash, roverBHash: OpticalMessageCodec().messageLinkHash(for: bData))
        let decisionData = try sessions.a.prepareOutgoing(body: .decision(decision), at: now + 100_000)
        try sessions.b.receive(decisionData, at: now + 100_000)
        if decision.outcome == .conflict { return sessions }
        let converge = try sessions.a.prepareOutgoing(body: .converge(ConvergeBody(
            decisionHash: OpticalMessageCodec().messageLinkHash(for: decisionData),
            releaseMilliseconds: now + 130_000, x: decision.x, y: decision.y)), at: now + 100_000)
        try sessions.b.receive(converge, at: now + 100_000)
        let ack = try sessions.b.prepareOutgoing(body: .convergeAck(HashAcknowledgementBody(
            hash: OpticalMessageCodec().messageLinkHash(for: converge))), at: now + 100_000)
        try sessions.a.receive(ack, at: now + 125_000)
        return sessions
    }

    private func offerBody() -> OpticalMessageBody {
        .offer(OfferBody(searchDurationSeconds: 180, centerHalfWidthMillimeters: 250,
            targetLabel: "chair", markerWidthMillimeters: 200,
            roverARendezvous: OpticalPose(x: -600, y: -800, headingMillidegrees: 0),
            roverBRendezvous: OpticalPose(x: 600, y: -800, headingMillidegrees: 0)))
    }

    private func context(_ role: RoverRole) -> OpticalProtocolContext {
        OpticalProtocolContext(missionID: missionID, markerID: markerID, localRole: role)
    }
}

private final class RecordingProtocolSink: SilentSearchEventSink, @unchecked Sendable {
    struct Entry { let event: String; let fields: [String: String] }
    private(set) var entries: [Entry] = []
    func record(event: String, fields: [String: String]) { entries.append(Entry(event: event, fields: fields)) }
}
