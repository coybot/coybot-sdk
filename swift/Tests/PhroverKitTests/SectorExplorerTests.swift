import XCTest
import RoverNav
@testable import PhroverKit

final class SectorExplorerTests: XCTestCase {
    func testProductionRebuildComposesFrontierFinderAStarAndSectorPolicy() {
        let frame = SharedMissionFrame(localOrigin: .zero, localNorthHeading: .pi / 2, sessionGeneration: 1)!
        let policy = SectorPathPolicy(sector: .west, frame: frame)
        let explorer = SectorExplorer(frame: frame, sector: .west, policy: policy)
        let map = Costmap(width: 20, height: 20, resolution: 0.1, origin: Vec2(-2, -1))
        var observed = ObservedGrid(matching: map)
        for y in 0..<20 {
            for x in 0...9 { observed.markObserved(at: map.cellCenter(x, y)) }
        }

        explorer.rebuild(costmap: map, observed: observed, from: Vec2(-1.8, 0))

        guard case .candidate(let candidate) = explorer.nextCandidate() else {
            return XCTFail("expected generated frontier to be reachable")
        }
        XCTAssertLessThan(candidate.missionCentroid.x, -0.25)
        XCTAssertNotNil(candidate.safePath)
    }

    func testObservationsRetainIdentityAcrossReorderingStrictlyWithinThirtyCentimeters() {
        let explorer = makeExplorer()
        explorer.rebuild(observations: [observation(-1, 0), observation(-1, 1)], from: start)

        explorer.rebuild(observations: [observation(-0.8, 1), observation(-0.8, 0)], from: start)

        XCTAssertEqual(explorer.candidates.map(\.stableID), ["frontier_1", "frontier_2"])
        XCTAssertEqual(explorer.candidates.map(\.missionCentroid.y), [0, 1])

        explorer.rebuild(observations: [observation(-0.5, 0)], from: start)
        XCTAssertEqual(explorer.candidates.single?.stableID, "frontier_3")
    }

    func testGreedyIdentityMatchingUsesDistanceBeforeOldIDAndNewCoordinates() {
        let explorer = makeExplorer()
        explorer.rebuild(observations: [observation(-1.4, 0), observation(-1, 0)], from: start)

        explorer.rebuild(observations: [observation(-1.19, 0), observation(-1.11, 0)], from: start)

        XCTAssertEqual(explorer.candidates.map(\.stableID), ["frontier_1", "frontier_2"])
        XCTAssertEqual(explorer.candidates[0].missionCentroid.x, -1.19, accuracy: 0.001)
        XCTAssertEqual(explorer.candidates[1].missionCentroid.x, -1.11, accuracy: 0.001)
    }

    func testUnmatchedObservationsReceiveIDsInMissionCoordinateThenWidthOrder() {
        let explorer = makeExplorer()

        explorer.rebuild(observations: [
            observation(-0.5, 1, width: 0.8),
            observation(-1, 2, width: 0.4),
            observation(-1, 2, width: 0.3),
            observation(-1, 1, width: 0.9),
        ], from: start)

        XCTAssertEqual(explorer.candidates.map(\.width), [0.9, 0.3, 0.4, 0.8])
        XCTAssertEqual(explorer.candidates.map(\.stableID), ["frontier_1", "frontier_2", "frontier_3", "frontier_4"])
    }

    func testVisitedAndRejectedStatePersistsAcrossRebuilds() {
        let explorer = makeExplorer()
        explorer.rebuild(observations: [observation(-1, 0), observation(-1, 1)], from: start)
        explorer.markVisited("frontier_1")
        explorer.markRejected("frontier_2", reason: .missionRejected("scan_failed"))

        explorer.rebuild(observations: [observation(-0.9, 0), observation(-0.9, 1)], from: start)

        XCTAssertEqual(explorer.candidates.map(\.status), [.visited, .rejected])
        XCTAssertEqual(explorer.nextCandidate(), .exhausted)
    }

    func testSelectionExcludesDestinationPathAndReachabilityFailures() {
        let policy = TestPathPolicy { path in
            path.contains(Vec2(-1.5, 9)) ? .rejected(.outsideSector(pointIndex: 1)) : .admissible
        }
        let explorer = makeExplorer(policy: policy) { start, goal in
            if goal.x == 3 { return nil }
            if goal.x == 2 { return [start, Vec2(-1.5, 9), goal] }
            return [start, goal]
        }

        explorer.rebuild(observations: [
            observation(0, 0),
            observation(-1, 3),
            observation(-1, 2),
            observation(-1, 1),
        ], from: start)

        guard case .candidate(let selected) = explorer.nextCandidate() else {
            return XCTFail("expected one safe candidate")
        }
        XCTAssertEqual(selected.missionCentroid, MissionPoint(x: -1, y: 1)!)
        XCTAssertEqual(explorer.candidates.map(\.rejectionReason), [
            nil,
            .pathPolicy(.outsideSector(pointIndex: 1)),
            .unreachable,
            .outOfSector,
        ])
    }

    func testRankingUsesLengthWidthMissionCoordinatesThenStableIDAndExhaustsExplicitly() {
        let explorer = makeExplorer()
        explorer.rebuild(observations: [
            observation(-1, 0, width: 0.2),
            observation(-1, 0, width: 0.2),
            observation(-1, 0, width: 0.5),
            observation(-1.5, 0, width: 1),
        ], from: start)

        var selectedIDs: [String] = []
        while case .candidate(let candidate) = explorer.nextCandidate() {
            selectedIDs.append(candidate.stableID)
            explorer.markVisited(candidate.stableID)
        }

        XCTAssertEqual(selectedIDs, ["frontier_1", "frontier_4", "frontier_2", "frontier_3"])
        XCTAssertEqual(explorer.nextCandidate(), .exhausted)
    }

    func testStructuredTelemetryRecordsDiscoveryRejectionRankingVisitAndExhaustion() {
        let sink = RecordingExplorerSink()
        let explorer = SectorExplorer(
            frame: Self.frame,
            sector: .west,
            policy: UnrestrictedPathPolicy(),
            events: sink,
            planner: { start, goal in goal == .zero ? nil : [start, goal] }
        )
        explorer.rebuild(observations: [observation(0, 0), observation(-1, 0)], from: start)

        guard case let .candidate(candidate) = explorer.nextCandidate() else {
            return XCTFail("expected candidate")
        }
        explorer.markVisited(candidate.stableID)
        XCTAssertEqual(explorer.nextCandidate(), .exhausted)

        XCTAssertTrue(sink.events.contains("silent_search_frontier_discovered"))
        XCTAssertTrue(sink.events.contains("silent_search_frontier_rejected"))
        XCTAssertTrue(sink.events.contains("silent_search_frontier_ranked"))
        XCTAssertTrue(sink.events.contains("silent_search_frontier_visited"))
        XCTAssertTrue(sink.events.contains("silent_search_frontier_exhausted"))
    }

    private func makeExplorer(
        policy: any PathAdmissibilityPolicy = UnrestrictedPathPolicy(),
        planner: @escaping (Vec2, Vec2) -> [Vec2]? = { [$0, $1] }
    ) -> SectorExplorer {
        SectorExplorer(
            frame: Self.frame,
            sector: .west,
            policy: policy,
            planner: planner
        )
    }

    private func observation(_ x: Double, _ y: Double, width: Double = 0.5) -> SectorFrontierObservation {
        SectorFrontierObservation(
            localCentroid: Self.frame.localPoint(from: MissionPoint(x: x, y: y)!),
            width: width
        )
    }

    private static let frame = SharedMissionFrame(
        localOrigin: .zero,
        localNorthHeading: 0,
        sessionGeneration: 1
    )!
    private var start: Vec2 { Self.frame.localPoint(from: MissionPoint(x: -2, y: 0)!) }
}

private final class RecordingExplorerSink: SilentSearchEventSink, @unchecked Sendable {
    private(set) var events: [String] = []
    func record(event: String, fields: [String: String]) { events.append(event) }
}

private struct TestPathPolicy: PathAdmissibilityPolicy {
    let evaluation: @Sendable ([Vec2]) -> PathAdmissibilityResult

    init(_ evaluation: @escaping @Sendable ([Vec2]) -> PathAdmissibilityResult) {
        self.evaluation = evaluation
    }

    func evaluate(path: [Vec2]) -> PathAdmissibilityResult { evaluation(path) }
}

private extension Array {
    var single: Element? { count == 1 ? self[0] : nil }
}
