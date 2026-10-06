import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class TwoRoverSilentSearchSimulationTests: XCTestCase {
    private let wallNow: Int64 = 1_786_406_400_000
    private let missionID = UUID(uuidString: "00000000-0000-0000-0000-000000000013")!

    func testRoverBBindsToIncomingOfferMission() async throws {
        let simulation = try await simulation(aTarget: nil, bTarget: nil, independentMissionIDs: true)
        let originalBMissionID = try XCTUnwrap(simulation.b.mission?.id)

        XCTAssertTrue(simulation.a.startHandshake())
        XCTAssertTrue(simulation.b.startHandshake())
        await eventually { simulation.a.phase == .waitingForSearch && simulation.b.phase == .waitingForSearch }

        XCTAssertNotEqual(originalBMissionID, missionID)
        XCTAssertEqual(simulation.b.mission?.id, missionID)
    }

    func testRoverBRejectsWrongMissionAfterBindingToOffer() async throws {
        let simulation = try await simulation(aTarget: nil, bTarget: nil, independentMissionIDs: true)
        simulation.aHarness.optical.shouldRelay = { payload in
            (try? OpticalMessageCodec().decode(payload).kind) != .searchCommit
        }
        XCTAssertTrue(simulation.a.startHandshake())
        XCTAssertTrue(simulation.b.startHandshake())
        await eventually {
            simulation.b.mission?.id == self.missionID &&
                simulation.aHarness.optical.presentedPayloads.contains {
                    (try? OpticalMessageCodec().decode($0).kind) == .searchCommit
                }
        }
        let commitPayload = try XCTUnwrap(simulation.aHarness.optical.presentedPayloads.first {
            (try? OpticalMessageCodec().decode($0).kind) == .searchCommit
        })
        let commit = try OpticalMessageCodec().decode(commitPayload)
        let wrongPayload = try OpticalMessageCodec().encode(OpticalMessage(
            missionID: UUID(), kind: commit.kind, sequence: commit.sequence, role: commit.role,
            markerID: commit.markerID, timestampMilliseconds: commit.timestampMilliseconds,
            body: commit.body
        ))

        simulation.bHarness.optical.sendToScanner(wrongPayload)
        await eventually { simulation.b.phase == .terminal(.protocolFailure(.wrongMission)) }
    }

    func testRendezvousOpticalTimeoutRetriesByteIdenticalStatusAndSupportsAbort() async throws {
        let simulation = try await simulation(aTarget: nil, bTarget: nil)
        XCTAssertTrue(simulation.a.startHandshake())
        XCTAssertTrue(simulation.b.startHandshake())
        await eventually { simulation.a.phase == .waitingForSearch && simulation.b.phase == .waitingForSearch }
        simulation.clock.advance(nanoseconds: 35_000_000_000)
        await eventually { simulation.a.phase == .searching && simulation.b.phase == .searching }
        await eventually {
            simulation.aHarness.motion.stopCount > 0 && simulation.bHarness.motion.stopCount > 0
        }
        simulation.aHarness.optical.shouldRelay = { payload in
            (try? OpticalMessageCodec().decode(payload).kind) != .status
        }
        simulation.aHarness.optical.suspendPresent = true
        simulation.clock.advance(nanoseconds: 750_000_000)
        await eventually { self.statusPayloads(in: simulation.aHarness).count == 1 }
        await eventually { simulation.aHarness.optical.suspendedPresentationCount == 1 }
        XCTAssertEqual(simulation.aHarness.optical.suspendedPresentationCount, 1)
        let timeoutDeadline = simulation.clock.monotonicNow + 30_000_000_000
        await eventually { simulation.clock.pendingDeadlines.contains(timeoutDeadline) }
        XCTAssertTrue(simulation.clock.pendingDeadlines.contains(timeoutDeadline),
                      "now=\(simulation.clock.monotonicNow) deadlines=\(simulation.clock.pendingDeadlines)")

        simulation.clock.advance(nanoseconds: 29_999_000_000)
        await taskTurn()
        XCTAssertNil(simulation.a.diagnostic)
        simulation.clock.advance(nanoseconds: 1_000_000)
        await eventually { simulation.a.diagnostic == .opticalTimedOut }
        XCTAssertEqual(simulation.a.diagnostic, .opticalTimedOut)
        let first = try XCTUnwrap(statusPayloads(in: simulation.aHarness).first)

        XCTAssertTrue(simulation.a.retryOpticalExchange(), "phase=\(simulation.a.phase) diagnostic=\(String(describing: simulation.a.diagnostic))")
        await eventually { self.statusPayloads(in: simulation.aHarness).count == 2 }
        let retried = statusPayloads(in: simulation.aHarness)
        XCTAssertEqual(retried.count, 2)
        XCTAssertEqual(retried.last, first)
        await simulation.a.abort()
        XCTAssertEqual(simulation.a.phase, .terminal(.operatorAborted))
    }

    func testTrackingRecoveryResumesInterruptedRendezvousRotationAndExchange() async throws {
        let simulation = try await simulation(aTarget: nil, bTarget: nil)
        XCTAssertTrue(simulation.a.startHandshake())
        XCTAssertTrue(simulation.b.startHandshake())
        await eventually { simulation.a.phase == .waitingForSearch && simulation.b.phase == .waitingForSearch }
        simulation.clock.advance(nanoseconds: 35_000_000_000)
        await eventually { simulation.a.phase == .searching && simulation.b.phase == .searching }
        await eventually {
            simulation.aHarness.motion.stopCount > 0 && simulation.bHarness.motion.stopCount > 0
        }
        simulation.aHarness.motion.suspendRotation = true
        simulation.clock.advance(nanoseconds: 750_000_000)
        await eventually { simulation.a.phase == .rendezvous(.rotating) }
        let rotationsBeforeRecovery = simulation.aHarness.motion.rotationRequests.count

        simulation.aHarness.safety.send(.trackingLimited(generation: 1))
        await eventually { simulation.aHarness.motion.stopCount >= 3 }
        simulation.aHarness.motion.suspendRotation = false
        simulation.aHarness.safety.send(.trackingNormal(generation: 1))
        await eventually { simulation.aHarness.motion.rotationRequests.count > rotationsBeforeRecovery }
        await eventually {
            simulation.a.phase == .terminal(.notFound) && simulation.b.phase == .terminal(.notFound)
        }

        XCTAssertEqual(simulation.a.phase, .terminal(.notFound))
        XCTAssertEqual(simulation.b.phase, .terminal(.notFound))
    }

    func testSoleFinderCompletesRoleOrderedProtocolAndBothReachFixedStandOffs() async throws {
        let simulation = try await simulation(aTarget: point(-1, 2), bTarget: nil)

        await simulation.startAndReachRendezvous()

        XCTAssertEqual(simulation.messageKinds, [.offer, .accept, .searchCommit, .searchAck,
                                                  .status, .status, .decision, .converge, .convergeAck])
        XCTAssertEqual(simulation.a.phase, .waitingForConvergence)
        XCTAssertEqual(simulation.b.phase, .waitingForConvergence)
        XCTAssertFalse(simulation.aHarness.motion.navigationRequests.contains { $0.1 == .unrestrictedConvergence })
        XCTAssertFalse(simulation.bHarness.motion.navigationRequests.contains { $0.1 == .unrestrictedConvergence })

        simulation.clock.advance(nanoseconds: 35_000_000_000)
        await eventually { simulation.a.phase == .terminal(.success) && simulation.b.phase == .terminal(.success) }

        XCTAssertEqual(simulation.aHarness.motion.navigationRequests.last?.0, point(-1.6, 2))
        XCTAssertEqual(simulation.bHarness.motion.navigationRequests.last?.0, point(-0.4, 2))
        XCTAssertEqual(simulation.aHarness.motion.navigationRequests.last?.1, .unrestrictedConvergence)
        XCTAssertEqual(simulation.bHarness.motion.navigationRequests.last?.1, .unrestrictedConvergence)
        XCTAssertEqual(simulation.aHarness.motion.rotationRequests.last?.heading ?? .nan,
                       -.pi / 2, accuracy: 0.0001)
        XCTAssertEqual(simulation.bHarness.motion.rotationRequests.last?.heading ?? .nan,
                       .pi / 2, accuracy: 0.0001)
        let aHeadings = simulation.aHarness.motion.rotationRequests.map(\.heading)
        let bHeadings = simulation.bHarness.motion.rotationRequests.map(\.heading)
        let expectedA: [Double] = [.pi / 2, -.pi / 2, .pi / 2, .pi / 2, -.pi / 2, -.pi / 2]
        let expectedB: [Double] = [.pi / 2, -.pi / 2, .pi / 2, .pi / 2, -.pi / 2, .pi / 2]
        XCTAssertEqual(aHeadings, expectedA)
        XCTAssertEqual(bHeadings, expectedB)
    }

    func testSoleBFinderSelectsBReport() async throws {
        let simulation = try await simulation(aTarget: nil, bTarget: point(1, 3))
        await simulation.startAndReachRendezvous()
        simulation.clock.advance(nanoseconds: 35_000_000_000)
        await eventually { simulation.a.phase == .terminal(.success) && simulation.b.phase == .terminal(.success) }

        XCTAssertEqual(simulation.aHarness.motion.navigationRequests.last?.0, point(0.4, 3))
        XCTAssertEqual(simulation.bHarness.motion.navigationRequests.last?.0, point(1.6, 3))
    }

    func testTrackingRecoveryRecreatesWaitingForConvergenceActionWhenReleaseExpired() async throws {
        let simulation = try await simulation(aTarget: point(-1, 2), bTarget: nil)
        await simulation.startAndReachRendezvous()
        simulation.aHarness.safety.send(.trackingLimited(generation: 1))
        simulation.bHarness.safety.send(.trackingLimited(generation: 1))
        await eventually { simulation.aHarness.motion.stopCount >= 3 && simulation.bHarness.motion.stopCount >= 3 }

        simulation.clock.advance(nanoseconds: 35_000_000_000)
        simulation.aHarness.safety.send(.trackingNormal(generation: 1))
        simulation.bHarness.safety.send(.trackingNormal(generation: 1))
        await eventually { simulation.a.phase == .terminal(.success) && simulation.b.phase == .terminal(.success) }
    }

    func testDualFinderUsesMedianAndNeitherFinderCommitsNotFoundWithoutMovement() async throws {
        let dual = try await simulation(aTarget: point(-0.1, 2), bTarget: point(0.3, 2.2))
        await dual.startAndReachRendezvous()
        dual.clock.advance(nanoseconds: 35_000_000_000)
        await eventually { dual.a.phase == .terminal(.success) && dual.b.phase == .terminal(.success) }
        XCTAssertEqual(dual.aHarness.motion.navigationRequests.last?.0, point(-0.5, 2.1))
        XCTAssertEqual(dual.bHarness.motion.navigationRequests.last?.0, point(0.7, 2.1))

        let neither = try await simulation(aTarget: nil, bTarget: nil)
        await neither.startAndReachRendezvous()
        await eventually { neither.a.phase == .terminal(.notFound) && neither.b.phase == .terminal(.notFound) }
        XCTAssertFalse(neither.aHarness.motion.navigationRequests.contains { $0.1 == .unrestrictedConvergence })
        XCTAssertFalse(neither.bHarness.motion.navigationRequests.contains { $0.1 == .unrestrictedConvergence })
    }

    func testConflictingFindersFailWithoutConvergence() async throws {
        let simulation = try await simulation(aTarget: point(-1, 2), bTarget: point(1, 2))
        await simulation.startAndReachRendezvous()

        await eventually {
            simulation.a.phase == .terminal(.protocolFailure(.convergenceAfterConflict)) &&
                simulation.b.phase == .terminal(.protocolFailure(.convergenceAfterConflict))
        }
        XCTAssertFalse(simulation.messageKinds.contains(.converge))
        XCTAssertFalse(simulation.aHarness.motion.navigationRequests.contains { $0.1 == .unrestrictedConvergence })
    }

    func testEveryMissingRendezvousMessageTimesOutSixtySecondsAfterSearchDeadline() async throws {
        let missingMessages: [(OpticalMessageKind, RoverRole)] = [
            (.status, .a), (.status, .b), (.decision, .a), (.converge, .a), (.convergeAck, .b),
        ]
        for (missingKind, missingRole) in missingMessages {
            let simulation = try await simulation(aTarget: point(-1, 2), bTarget: nil)
            var dropped = false
            simulation.aHarness.optical.shouldRelay = { payload in
                let message = try? OpticalMessageCodec().decode(payload)
                if message?.kind == missingKind, message?.role == missingRole, !dropped {
                    dropped = true
                    return false
                }
                return true
            }
            simulation.bHarness.optical.shouldRelay = simulation.aHarness.optical.shouldRelay
            await simulation.startAndReachRendezvous(waitForProtocol: false)

            let partnerDeadline: Int64 = 215_000_000_000
            simulation.clock.advance(nanoseconds: partnerDeadline - 1_000_000 - simulation.clock.monotonicNow)
            await taskTurn()
            XCTAssertNotEqual(simulation.a.phase, .terminal(.partnerTimeout), "\(missingKind) \(missingRole)")
            XCTAssertNotEqual(simulation.b.phase, .terminal(.partnerTimeout), "\(missingKind) \(missingRole)")
            simulation.clock.advance(nanoseconds: 1_000_000)
            await eventually {
                simulation.a.phase == .terminal(.partnerTimeout) || simulation.b.phase == .terminal(.partnerTimeout)
            }
        }
    }

    func testEveryMissingHandshakeMessageTimesOutWithoutStartingSearch() async throws {
        let missingMessages: [(OpticalMessageKind, RoverRole)] = [
            (.offer, .a), (.accept, .b), (.searchCommit, .a), (.searchAck, .b),
        ]
        for (missingKind, missingRole) in missingMessages {
            let simulation = try await simulation(aTarget: nil, bTarget: nil)
            var dropped = false
            let filter: (Data) -> Bool = { payload in
                let message = try? OpticalMessageCodec().decode(payload)
                if message?.kind == missingKind, message?.role == missingRole, !dropped {
                    dropped = true
                    return false
                }
                return true
            }
            simulation.aHarness.optical.shouldRelay = filter
            simulation.bHarness.optical.shouldRelay = filter
            XCTAssertTrue(simulation.a.startHandshake())
            XCTAssertTrue(simulation.b.startHandshake())
            await taskTurn()
            simulation.clock.advance(nanoseconds: 30_000_000_000)
            await eventually {
                simulation.a.diagnostic == .opticalTimedOut || simulation.b.diagnostic == .opticalTimedOut
            }
            XCTAssertFalse(simulation.a.phase == .searching && simulation.b.phase == .searching)
        }
    }

    func testDuplicateAndOutOfOrderRendezvousMessagesAreRejected() async throws {
        let duplicate = try await simulation(aTarget: point(-1, 2), bTarget: nil)
        var duplicated = false
        duplicate.aHarness.optical.shouldRelay = { payload in
            if (try? OpticalMessageCodec().decode(payload).kind) == .status, !duplicated {
                duplicated = true
                duplicate.bHarness.optical.sendToScanner(payload)
            }
            return true
        }
        await duplicate.startAndReachRendezvous()
        await eventually { duplicate.a.isTerminal || duplicate.b.isTerminal }
        XCTAssertTrue(duplicate.a.hasProtocolFailure || duplicate.b.hasProtocolFailure)

        let outOfOrder = try await simulation(aTarget: point(-1, 2), bTarget: nil)
        XCTAssertTrue(outOfOrder.a.startHandshake())
        XCTAssertTrue(outOfOrder.b.startHandshake())
        await eventually { outOfOrder.a.phase == .waitingForSearch && outOfOrder.b.phase == .waitingForSearch }
        let oldAcknowledgement = outOfOrder.bHarness.optical.presentedPayloads.first {
            (try? OpticalMessageCodec().decode($0).kind) == .searchAck
        }!
        outOfOrder.bHarness.optical.sendToScanner(oldAcknowledgement)
        outOfOrder.clock.advance(nanoseconds: 35_000_000_000)
        await eventually { outOfOrder.a.phase == .searching && outOfOrder.b.phase == .searching }
        await eventually { outOfOrder.aHarness.motion.stopCount > 0 && outOfOrder.bHarness.motion.stopCount > 0 }
        outOfOrder.clock.advance(nanoseconds: 750_000_000)
        await eventually { outOfOrder.b.isTerminal }
        XCTAssertTrue(outOfOrder.b.hasProtocolFailure)
    }

    func testDeadlineCompletesNotFoundAndSafetyStopCanResetRendezvous() async throws {
        let deadline = try await simulation(aTarget: nil, bTarget: nil)
        let west = SectorFrontierCandidate(stableID: "west", localCentroid: Vec2(-1, 0),
            missionCentroid: point(-1, 0), width: 1, status: .available, rejectionReason: nil,
            safePath: [Vec2(-1, 0)], pathLength: 1)
        let east = SectorFrontierCandidate(stableID: "east", localCentroid: Vec2(1, 0),
            missionCentroid: point(1, 0), width: 1, status: .available, rejectionReason: nil,
            safePath: [Vec2(1, 0)], pathLength: 1)
        deadline.aHarness.explorer.selection = .candidate(west)
        deadline.bHarness.explorer.selection = .candidate(east)
        deadline.aHarness.motion.suspendNavigation = true
        deadline.bHarness.motion.suspendNavigation = true
        XCTAssertTrue(deadline.a.startHandshake())
        XCTAssertTrue(deadline.b.startHandshake())
        await eventually { deadline.a.phase == .waitingForSearch && deadline.b.phase == .waitingForSearch }
        deadline.clock.advance(nanoseconds: 35_000_000_000)
        await eventually { deadline.a.phase == .searching && deadline.b.phase == .searching }
        await eventually { deadline.aHarness.motion.stopCount > 0 && deadline.bHarness.motion.stopCount > 0 }
        deadline.clock.advance(nanoseconds: 750_000_000)
        await eventually {
            deadline.aHarness.motion.navigationRequests.count == 1 &&
                deadline.bHarness.motion.navigationRequests.count == 1
        }
        deadline.aHarness.motion.suspendNavigation = false
        deadline.bHarness.motion.suspendNavigation = false
        deadline.clock.advance(nanoseconds: 119_250_000_000)
        await eventually { deadline.a.phase == .terminal(.notFound) && deadline.b.phase == .terminal(.notFound) }

        let stopped = try await simulation(aTarget: point(-1, 2), bTarget: nil)
        await stopped.startAndReachRendezvous()
        stopped.aHarness.safety.send(.operatorStop)
        stopped.bHarness.safety.send(.reactiveSafetyFailed)
        await eventually {
            stopped.a.phase == .terminal(.operatorStopped) &&
                stopped.b.phase == .terminal(.safetyFailure(.reactiveSafety))
        }
        await stopped.a.reset()
        XCTAssertEqual(stopped.a.phase, .setup)
        XCTAssertNil(stopped.a.mission)
        XCTAssertNil(stopped.a.targetConfirmation)
        XCTAssertFalse(stopped.aHarness.motion.navigationRequests.contains { $0.1 == .unrestrictedConvergence })
    }

    func testUnsafeStandOffFailsWithoutAlternateAndFoundWaitsForHeadingSuccess() async throws {
        let unsafe = try await simulation(aTarget: point(-1, 2), bTarget: nil)
        unsafe.aHarness.motion.results = [.arrived, .failed(.obstacle)]
        await unsafe.startAndReachRendezvous()
        unsafe.clock.advance(nanoseconds: 35_000_000_000)
        await eventually { unsafe.a.phase == .terminal(.motionFailure(.obstacle)) }
        XCTAssertEqual(unsafe.aHarness.motion.navigationRequests.filter { $0.1 == .unrestrictedConvergence }.count, 1)

        let heading = try await simulation(aTarget: point(-1, 2), bTarget: nil)
        heading.aHarness.motion.updatePoseOnArrival = false
        await heading.startAndReachRendezvous()
        heading.clock.advance(nanoseconds: 35_000_000_000)
        await eventually { if case .terminal = heading.a.phase { true } else { false } }
        XCTAssertNotEqual(heading.a.phase, .terminal(.success))
    }

    private func simulation(aTarget: MissionPoint?, bTarget: MissionPoint?,
                            independentMissionIDs: Bool = false) async throws -> Simulation {
        let clock = ManualSilentSearchClock(wallNowMilliseconds: wallNow)
        let opticalA = FakeSilentSearchOpticalExchange()
        let opticalB = FakeSilentSearchOpticalExchange()
        opticalA.peer = opticalB
        opticalB.peer = opticalA
        let aHarness = SilentSearchTestHarness(clock: clock, optical: opticalA)
        let bHarness = SilentSearchTestHarness(clock: clock, optical: opticalB)
        for harness in [aHarness, bHarness] {
            harness.readiness.snapshot = .ready(sessionGeneration: 1)
            harness.motion.updatePoseOnArrival = true
        }
        aHarness.targetObserver.result = observation(aTarget)
        bHarness.targetObserver.result = observation(bTarget)
        let a = aHarness.coordinator()
        let b = bHarness.coordinator()
        a.configure(mission(.a))
        b.configure(mission(.b, id: independentMissionIDs ? UUID() : missionID))
        XCTAssertTrue(a.startCalibration())
        XCTAssertTrue(b.startCalibration())
        let frame = SharedMissionFrame(localOrigin: .zero, localNorthHeading: 0, sessionGeneration: 1)!
        aHarness.calibration.send(.accepted(frame))
        bHarness.calibration.send(.accepted(frame))
        await eventually { a.phase == .handshake(.ready) && b.phase == .handshake(.ready) }
        return Simulation(clock: clock, aHarness: aHarness, bHarness: bHarness, a: a, b: b)
    }

    private func mission(_ role: RoverRole, id: UUID? = nil) -> SilentSearchMission {
        SilentSearchMission(id: id ?? missionID, role: role, targetLabel: "chair",
                            searchDurationSeconds: 120, markerID: "SILENT_SEARCH_01")!
    }

    private func observation(_ target: MissionPoint?) -> SilentSearchTargetObservationResult {
        guard let target else { return .pending }
        return .confirmed(TargetConfirmation(label: "chair", coordinate: target,
                                              sampleCount: 3, meanConfidence: 0.95))
    }

    private func point(_ x: Double, _ y: Double) -> MissionPoint { MissionPoint(x: x, y: y)! }

    private func statusPayloads(in harness: SilentSearchTestHarness) -> [Data] {
        harness.optical.presentedPayloads.filter {
            guard let message = try? OpticalMessageCodec().decode($0) else { return false }
            return message.kind == .status && message.role == .a
        }
    }

    private func taskTurn() async { await Task.yield(); await Task.yield() }

    private func eventually(_ condition: @escaping @MainActor () -> Bool) async {
        for _ in 0..<1_000 where !condition() { await Task.yield() }
    }
}

@MainActor
private struct Simulation {
    let clock: ManualSilentSearchClock
    let aHarness: SilentSearchTestHarness
    let bHarness: SilentSearchTestHarness
    let a: SilentSearchCoordinator
    let b: SilentSearchCoordinator

    var messageKinds: [OpticalMessageKind] {
        let codec = OpticalMessageCodec()
        return (aHarness.optical.presentedPayloads + bHarness.optical.presentedPayloads)
            .compactMap { try? codec.decode($0).kind }
            .sorted { order($0) < order($1) }
    }

    func startAndReachRendezvous(waitForProtocol: Bool = true) async {
        XCTAssertTrue(a.startHandshake())
        XCTAssertTrue(b.startHandshake())
        await eventually { a.phase == .waitingForSearch && b.phase == .waitingForSearch }
        clock.advance(nanoseconds: 35_000_000_000)
        await eventually { a.phase == .searching && b.phase == .searching }
        await eventually { aHarness.motion.stopCount > 0 && bHarness.motion.stopCount > 0 }
        clock.advance(nanoseconds: 750_000_000)
        if waitForProtocol {
            await eventually {
                (a.phase == .waitingForConvergence && b.phase == .waitingForConvergence) ||
                    (a.isTerminal && b.isTerminal)
            }
        } else {
            await eventually { a.isRendezvousOrLater && b.isRendezvousOrLater }
        }
    }

    private func eventually(_ condition: @escaping @MainActor () -> Bool) async {
        for _ in 0..<300 where !condition() { await Task.yield() }
    }

    private func order(_ kind: OpticalMessageKind) -> Int {
        [.offer, .accept, .searchCommit, .searchAck, .status, .decision, .converge, .convergeAck]
            .firstIndex(of: kind) ?? 99
    }
}

private extension SilentSearchCoordinator {
    var isTerminal: Bool { if case .terminal = phase { true } else { false } }
    var hasProtocolFailure: Bool {
        if case .terminal(.protocolFailure) = phase { true } else { false }
    }
    var isRendezvousOrLater: Bool {
        switch phase {
        case .rendezvous, .waitingForConvergence, .converging, .terminal: true
        default: false
        }
    }
}
