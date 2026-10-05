import XCTest
import PhroverKit
import RoverNav
@testable import PhroverCloud

@MainActor
final class CloudDoorwayEvidenceProviderTests: XCTestCase {
    func testExploreChoiceBoostsOnlyTheKnownCandidate() async {
        let brain = StubRoverBrain(result: .success(BrainOutput(
            decision: .explore(candidateId: "doorway_candidate_2")
        )))
        let provider = CloudDoorwayEvidenceProvider(brain: brain)
        let candidates = [candidate(1), candidate(2)]

        let boosts = await provider.boostValues(
            forFrame: Data([0x01, 0x02]),
            candidates: candidates
        )

        XCTAssertEqual(boosts, [
            DoorwayCandidateID("doorway_candidate_1"): 0,
            DoorwayCandidateID("doorway_candidate_2"): 1,
        ])
        XCTAssertEqual(brain.seenContexts.count, 1)
        XCTAssertEqual(brain.seenContexts[0].frameJPEG, Data([0x01, 0x02]))
        XCTAssertEqual(
            brain.seenContexts[0].explorationCandidates.map(\.id),
            ["doorway_candidate_1", "doorway_candidate_2"]
        )
    }

    func testNonExploreChoiceProvidesNoEvidence() async {
        let provider = CloudDoorwayEvidenceProvider(brain: StubRoverBrain(
            result: .success(BrainOutput(decision: .done))
        ))

        let boosts = await provider.boostValues(forFrame: Data([0x01]), candidates: [candidate(1)])

        XCTAssertTrue(boosts.isEmpty)
    }

    func testUnknownExploreChoiceProvidesNoEvidence() async {
        let provider = CloudDoorwayEvidenceProvider(brain: StubRoverBrain(
            result: .success(BrainOutput(decision: .explore(candidateId: "doorway_candidate_99")))
        ))

        let boosts = await provider.boostValues(forFrame: Data([0x01]), candidates: [candidate(1)])

        XCTAssertTrue(boosts.isEmpty)
    }

    func testBrainErrorProvidesNoEvidence() async {
        let provider = CloudDoorwayEvidenceProvider(brain: StubRoverBrain(result: .failure(TestError.failed)))

        let boosts = await provider.boostValues(forFrame: Data([0x01]), candidates: [candidate(1)])

        XCTAssertTrue(boosts.isEmpty)
    }

    func testOfflineStateProvidesNoEvidenceWithoutCallingBrain() async {
        let brain = StubRoverBrain(result: .success(BrainOutput(
            decision: .explore(candidateId: "doorway_candidate_1")
        )))
        let provider = CloudDoorwayEvidenceProvider(brain: brain, isOnline: { false })

        let boosts = await provider.boostValues(forFrame: Data([0x01]), candidates: [candidate(1)])

        XCTAssertTrue(boosts.isEmpty)
        XCTAssertTrue(brain.seenContexts.isEmpty)
    }

    func testTimeoutProvidesNoEvidenceWithoutWaitingForProductionDeadline() async {
        let brain = StubRoverBrain(
            result: .success(BrainOutput(decision: .explore(candidateId: "doorway_candidate_1"))),
            delay: .seconds(1)
        )
        let provider = CloudDoorwayEvidenceProvider(
            brain: brain,
            timeout: .milliseconds(10)
        )
        let clock = ContinuousClock()
        let start = clock.now

        let boosts = await provider.boostValues(forFrame: Data([0x01]), candidates: [candidate(1)])

        XCTAssertTrue(boosts.isEmpty)
        XCTAssertLessThan(start.duration(to: clock.now), .milliseconds(500))
    }

    private func candidate(_ number: Int) -> DoorwayCandidate {
        DoorwayCandidate(
            id: DoorwayCandidateID("doorway_candidate_\(number)"),
            planePoint: Vec2(Double(number), 0),
            outwardDirection: Vec2(1, 0),
            widthMeters: 0.9,
            frontierCellCount: 4
        )
    }
}

@MainActor
private final class StubRoverBrain: RoverBrain {
    let result: Result<BrainOutput, Error>
    let delay: Duration?
    private(set) var seenContexts: [MissionContext] = []

    init(result: Result<BrainOutput, Error>, delay: Duration? = nil) {
        self.result = result
        self.delay = delay
    }

    func nextAction(_ context: MissionContext) async throws -> BrainOutput {
        seenContexts.append(context)
        if let delay {
            try await Task.sleep(for: delay)
        }
        return try result.get()
    }
}

private enum TestError: Error {
    case failed
}
