import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class RoomSessionCoordinatorTests: XCTestCase {
    func testResetOrdersAwaitedStopAbandonTopologyThenARWithOneGeneration() async {
        var events: [String] = []
        let motion = CoordinatorMotion { events.append("stop") }
        let topology = CoordinatorTopology(events: { events.append($0) })
        let ar = CoordinatorAR(events: { events.append($0) })
        let coordinator = RoomSessionCoordinator(
            motion: motion,
            topology: topology,
            arSession: ar,
            initialGeneration: 20
        )

        let generation = await coordinator.resetSession()

        XCTAssertEqual(generation, 21)
        XCTAssertEqual(events, ["stop", "abandon", "topology.reset.21", "ar.reset.21"])
    }

    func testResetDoesNotAbandonOrResetUntilStopCompletes() async {
        var events: [String] = []
        let motion = CoordinatorMotion { events.append("stop") }
        motion.suspendStop = true
        let topology = CoordinatorTopology(events: { events.append($0) })
        let ar = CoordinatorAR(events: { events.append($0) })
        let coordinator = RoomSessionCoordinator(
            motion: motion,
            topology: topology,
            arSession: ar,
            initialGeneration: 8
        )

        let reset = Task { await coordinator.resetSession() }
        await Task.yield()

        XCTAssertEqual(events, ["stop"])
        XCTAssertEqual(coordinator.sessionGeneration, 8)

        motion.completeStop()
        let generation = await reset.value

        XCTAssertEqual(generation, 9)
        XCTAssertEqual(events, ["stop", "abandon", "topology.reset.9", "ar.reset.9"])
    }

    func testStartsTopologyOnceFromFirstFreshNormalMatchingObservation() async {
        let motion = CoordinatorMotion {}
        let topology = CoordinatorTopology(events: { _ in })
        let ar = CoordinatorAR(events: { _ in })
        let coordinator = RoomSessionCoordinator(
            motion: motion,
            topology: topology,
            arSession: ar
        )
        let generation = await coordinator.resetSession()

        ar.publish(observation(frame: 1, timestamp: 1, quality: .limited, generation: generation))
        ar.publish(observation(frame: 2, timestamp: 2, quality: .normal, generation: generation + 1))
        ar.publish(observation(frame: 0, timestamp: 0, quality: .normal, generation: generation))
        XCTAssertTrue(topology.startCalls.isEmpty)

        let first = observation(frame: 3, timestamp: 3, quality: .normal, generation: generation)
        ar.publish(first)
        ar.publish(observation(frame: 4, timestamp: 4, quality: .normal, generation: generation))

        XCTAssertEqual(topology.startCalls.count, 1)
        XCTAssertEqual(topology.startCalls.first?.generation, generation)
        XCTAssertEqual(topology.startCalls.first?.pose, first.pose)
    }

    func testFeedsEveryFreshNormalMatchingPoseAfterStartingTopologyOnce() async {
        let topology = CoordinatorTopology(events: { _ in })
        let ar = CoordinatorAR(events: { _ in })
        let coordinator = RoomSessionCoordinator(
            motion: CoordinatorMotion {},
            topology: topology,
            arSession: ar
        )
        let generation = await coordinator.resetSession()
        let first = observation(frame: 1, timestamp: 1, quality: .normal, generation: generation)
        let second = observation(frame: 4, timestamp: 4, quality: .normal, generation: generation)

        ar.publish(first)
        ar.publish(observation(frame: 2, timestamp: 2, quality: .limited, generation: generation))
        ar.publish(observation(frame: 3, timestamp: 3, quality: .normal, generation: generation + 1))
        ar.publish(second)

        XCTAssertEqual(topology.startCalls.map(\.pose), [first.pose])
        XCTAssertEqual(topology.stablePoseCalls.map(\.pose), [second.pose])
        XCTAssertEqual(topology.stablePoseCalls.map(\.generation), [generation])
    }

    func testExternalARResetSynchronouslyAbandonsAndBindsClearedTopology() async {
        var events: [String] = []
        let topology = CoordinatorTopology(events: { events.append($0) })
        let ar = CoordinatorAR(events: { events.append($0) })
        let coordinator = RoomSessionCoordinator(
            motion: CoordinatorMotion {},
            topology: topology,
            arSession: ar
        )
        _ = await coordinator.resetSession()
        events.removeAll()

        ar.notifyReset(generation: 9)

        XCTAssertEqual(coordinator.sessionGeneration, 9)
        XCTAssertEqual(events, ["abandon", "topology.reset.9"])
        ar.publish(observation(frame: 1, timestamp: 1, quality: .normal, generation: 9))
        XCTAssertEqual(topology.startCalls.map(\.generation), [9])
    }

    func testDirectARSessionManagerResetSynchronouslyClearsConcreteTopology() {
        let ar = ARSessionManager()
        let topology = SessionRoomTopology()
        topology.startSession(
            generation: 1,
            initialPose: Pose2D(position: Vec2(1, 2), yaw: 0)
        )
        let coordinator = RoomSessionCoordinator(
            motion: CoordinatorMotion {},
            topology: topology,
            arSession: ar,
            initialGeneration: 1
        )

        ar.resetTracking(generation: 12, runSession: false)

        XCTAssertEqual(coordinator.sessionGeneration, 12)
        XCTAssertEqual(topology.snapshot.sessionGeneration, 12)
        XCTAssertNil(topology.snapshot.currentRoomID)
        XCTAssertTrue(topology.snapshot.rooms.isEmpty)
    }

    func testDoesNotStartTopologyBeforeCoordinatorAllocatesGeneration() {
        let motion = CoordinatorMotion {}
        let topology = CoordinatorTopology(events: { _ in })
        let ar = CoordinatorAR(events: { _ in })
        _ = RoomSessionCoordinator(motion: motion, topology: topology, arSession: ar)

        ar.publish(observation(frame: 1, timestamp: 1, quality: .normal, generation: 0))

        XCTAssertTrue(topology.startCalls.isEmpty)
    }

    func testRepeatedAppStartupDoesNotResetActiveTopology() async {
        var events: [String] = []
        let motion = CoordinatorMotion { events.append("stop") }
        let topology = CoordinatorTopology(events: { events.append($0) })
        let ar = CoordinatorAR(events: { events.append($0) })
        let coordinator = RoomSessionCoordinator(motion: motion, topology: topology, arSession: ar)

        await coordinator.startIfNeeded()
        await coordinator.startIfNeeded()

        XCTAssertEqual(coordinator.sessionGeneration, 1)
        XCTAssertEqual(events, ["stop", "abandon", "topology.reset.1", "ar.reset.1"])
    }

    func testSecondResetRejectsOldAndMismatchedSamplesAndStartsOnce() async {
        let motion = CoordinatorMotion {}
        let topology = CoordinatorTopology(events: { _ in })
        let ar = CoordinatorAR(events: { _ in })
        let coordinator = RoomSessionCoordinator(motion: motion, topology: topology, arSession: ar)
        let oldGeneration = await coordinator.resetSession()
        ar.publish(observation(frame: 1, timestamp: 1, quality: .normal, generation: oldGeneration))
        XCTAssertEqual(topology.startCalls.count, 1)

        let generation = await coordinator.resetSession()
        ar.publish(observation(frame: 2, timestamp: 2, quality: .normal, generation: oldGeneration))
        ar.publish(observation(frame: 1, timestamp: 1, quality: .normal, generation: generation + 1))
        ar.publish(observation(frame: 1, timestamp: 1, quality: .limited, generation: generation))
        XCTAssertEqual(topology.startCalls.count, 1)

        ar.publish(observation(frame: 2, timestamp: 2, quality: .normal, generation: generation))
        ar.publish(observation(frame: 3, timestamp: 3, quality: .normal, generation: generation))

        XCTAssertEqual(topology.startCalls.map(\.generation), [oldGeneration, generation])
    }

    private func observation(frame: UInt64,
                             timestamp: TimeInterval,
                             quality: PoseTrackingQuality,
                             generation: UInt64) -> PoseObservation {
        PoseObservation(
            pose: Pose2D(position: Vec2(Double(frame), 0), yaw: 0),
            frameSequence: frame,
            timestamp: timestamp,
            trackingQuality: quality,
            sessionGeneration: generation
        )
    }
}

@MainActor
private final class CoordinatorMotion: RoverMotion {
    var state: NavigationController.State = .idle
    var suspendStop = false
    let onStop: () -> Void
    private var stopContinuation: CheckedContinuation<Void, Never>?

    init(onStop: @escaping () -> Void) { self.onStop = onStop }
    func navigate(to goal: Vec2) {}
    func rotate(by angle: Double) async {}
    func cancel() { state = .idle }
    func stopAndWait() async {
        onStop()
        if suspendStop {
            await withCheckedContinuation { stopContinuation = $0 }
        }
        state = .idle
    }
    func completeStop() {
        suspendStop = false
        stopContinuation?.resume()
        stopContinuation = nil
    }
}

@MainActor
private final class CoordinatorAR: RoomSessionARManaging {
    var latestObservation: PoseObservation?
    var observationHandler: ((PoseObservation) -> Void)?
    var onReset: ((UInt64) -> Void)?
    let events: (String) -> Void

    init(events: @escaping (String) -> Void) { self.events = events }
    func resetTracking(generation: UInt64) {
        events("ar.reset.\(generation)")
        onReset?(generation)
    }
    func notifyReset(generation: UInt64) { onReset?(generation) }
    func publish(_ observation: PoseObservation) {
        latestObservation = observation
        observationHandler?(observation)
    }
}

@MainActor
private final class CoordinatorTopology: RoomTopologyManaging {
    var snapshot = RoomTopologySnapshot(
        sessionGeneration: nil,
        currentRoomID: nil,
        rooms: [],
        doorways: [],
        candidates: []
    )
    private(set) var startCalls: [(generation: UInt64, pose: Pose2D)] = []
    private(set) var stablePoseCalls: [(generation: UInt64, pose: Pose2D)] = []
    let events: (String) -> Void

    init(events: @escaping (String) -> Void) { self.events = events }
    func startSession(generation: UInt64, initialPose: Pose2D) {
        startCalls.append((generation, initialPose))
    }
    func ingestStablePose(_ pose: Pose2D, sessionGeneration generation: UInt64) {
        stablePoseCalls.append((generation, pose))
    }
    func reset(forSessionGeneration generation: UInt64) { events("topology.reset.\(generation)") }
    func refreshCandidates(from frontiers: [Frontier], referencePose: Pose2D) -> [DoorwayCandidate] { [] }
    func rankedCandidates(assessments: [DoorwayCandidateAssessment],
                          visualBoosts: [DoorwayCandidateID: Double],
                          excluding excludedCandidateIDs: Set<DoorwayCandidateID>) -> [RankedDoorwayCandidate] { [] }
    func beginTransition(
        candidateID: DoorwayCandidateID,
        approachPose: Pose2D
    ) -> TransitionStartResult { .rejected(.missingCandidate) }
    func observeTransition(_ observation: TransitionObservation) -> TransitionObservationResult { .ignored }
    func confirmTransition() -> RoomID? { nil }
    func rejectTransition(reason: String) {}
    func abandonTransition() { events("abandon") }
}
