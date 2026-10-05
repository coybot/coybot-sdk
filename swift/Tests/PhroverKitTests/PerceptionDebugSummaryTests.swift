import XCTest
import CoreGraphics
@testable import PhroverKit

final class PerceptionDebugSummaryTests: XCTestCase {
    func testVisibleObjectSummaryShowsTopLabelsAndConfidence() {
        let objects = [
            PerceivedObject(label: "chair", confidence: 0.752, normalizedPoint: CGPoint(x: 0.1, y: 0.2)),
            PerceivedObject(label: "table", confidence: 0.934, normalizedPoint: CGPoint(x: 0.3, y: 0.4)),
            PerceivedObject(label: "person", confidence: 0.611, normalizedPoint: CGPoint(x: 0.5, y: 0.6)),
        ]

        XCTAssertEqual(PerceptionDebugSummary.visibleObjects(objects), "table 93%, chair 75%, person 61%")
    }

    func testVisibleObjectSummaryReportsNoneWhenEmpty() {
        XCTAssertEqual(PerceptionDebugSummary.visibleObjects([]), "none")
    }

    func testNavigationSummaryRetainsScanEvidenceThroughCompletedTransition() {
        var summary = NavigationDebugSummary()
        XCTAssertEqual(summary.openingsText, "—")
        XCTAssertEqual(summary.doorwayCandidatesText, "—")
        XCTAssertEqual(summary.targetText, "none")
        XCTAssertEqual(summary.transitionText, "idle")

        summary.apply(.scanning(step: 3, total: 12, openingCount: 2, candidateCount: 1))
        XCTAssertEqual(summary.openingsText, "2")
        XCTAssertEqual(summary.doorwayCandidatesText, "1")
        XCTAssertEqual(summary.transitionText, "scanning 3/12")

        let candidateID = DoorwayCandidateID("doorway_candidate_1")
        summary.apply(.candidateFound(id: candidateID, reachable: true))
        XCTAssertEqual(summary.doorwayCandidatesText, "1 (reachable)")
        XCTAssertEqual(summary.targetText, "doorway_candidate_1")
        XCTAssertEqual(summary.transitionText, "candidate found")

        summary.apply(.approaching(id: candidateID))
        XCTAssertEqual(summary.openingsText, "2")
        XCTAssertEqual(summary.doorwayCandidatesText, "1 (reachable)")
        XCTAssertEqual(summary.transitionText, "approaching")

        summary.apply(.confirmingCrossing(id: candidateID))
        XCTAssertEqual(summary.transitionText, "confirming crossing")

        summary.apply(.completed(doorwayID: DoorwayID("doorway_1"), roomID: RoomID("room_2")))
        XCTAssertEqual(summary.targetText, "doorway_1")
        XCTAssertEqual(summary.transitionText, "completed: room_2")
    }

    func testNavigationSummaryShowsUnreachableFailureAndExhaustion() {
        var summary = NavigationDebugSummary()
        let candidateID = DoorwayCandidateID("doorway_candidate_4")
        summary.apply(.scanning(step: 0, total: 12, openingCount: 3, candidateCount: 1))
        summary.apply(.candidateFound(id: candidateID, reachable: false))
        summary.apply(.unreachable(id: candidateID))

        XCTAssertEqual(summary.openingsText, "3")
        XCTAssertEqual(summary.doorwayCandidatesText, "1 (unreachable)")
        XCTAssertEqual(summary.targetText, "doorway_candidate_4")
        XCTAssertEqual(summary.transitionText, "unreachable")

        summary.apply(.failed(reason: "tracking_unavailable"))
        XCTAssertEqual(summary.transitionText, "failed: tracking unavailable")

        summary.apply(.exhausted)
        XCTAssertEqual(summary.targetText, "none")
        XCTAssertEqual(summary.transitionText, "exhausted")

        summary.apply(.idle)
        XCTAssertEqual(summary.openingsText, "—")
        XCTAssertEqual(summary.doorwayCandidatesText, "—")
        XCTAssertEqual(summary.transitionText, "idle")
    }
}
