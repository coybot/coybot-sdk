import XCTest
@testable import RoverNav

final class CostmapTests: XCTestCase {
    func testWorldCellRoundTrip() {
        let map = Costmap(width: 20, height: 20, resolution: 0.1, origin: Vec2(-1, -1))
        let c = map.worldToCell(Vec2(0.0, 0.0))
        XCTAssertEqual(c.cx, 10)
        XCTAssertEqual(c.cy, 10)
        let center = map.cellCenter(c.cx, c.cy)
        // Cell center should be within half a cell of the queried point.
        XCTAssertLessThanOrEqual(center.distance(to: Vec2(0, 0)), 0.1)
    }

    func testOutOfBoundsIsBlocked() {
        let map = Costmap(width: 5, height: 5, resolution: 1.0)
        XCTAssertTrue(map.isBlocked(-1, 0))
        XCTAssertTrue(map.isBlocked(5, 5))
        XCTAssertFalse(map.isBlocked(2, 2))
    }

    func testMarkAndInflate() {
        var map = Costmap(width: 21, height: 21, resolution: 0.1, origin: Vec2(-1.05, -1.05))
        map.markObstacle(at: Vec2(0, 0))
        let c = map.worldToCell(Vec2(0, 0))
        XCTAssertTrue(map.isBlocked(c.cx, c.cy))

        map.inflate(radius: 0.25) // ~2-3 cells
        // Immediate neighbor should now be hard-blocked (inner ring).
        XCTAssertTrue(map.isBlocked(c.cx + 1, c.cy))
        // A cell far away stays free.
        XCTAssertFalse(map.isBlocked(c.cx + 10, c.cy))
        // A mid-range cell picked up some cost gradient.
        XCTAssertGreaterThan(map.cost(c.cx + 2, c.cy), 0)
    }

    func testSegmentClearanceRejectsTouchedOccupiedOrInflatedCells() {
        var map = Costmap(width: 10, height: 10, resolution: 0.1)
        map.setCost(5, 5, Costmap.lethal)
        XCTAssertTrue(map.isSegmentClear(from: Vec2(0.15, 0.35), to: Vec2(0.85, 0.35), margin: 0.02))
        XCTAssertFalse(map.isSegmentClear(from: Vec2(0.15, 0.55), to: Vec2(0.85, 0.55), margin: 0))

        map.setCost(5, 5, 1)
        XCTAssertFalse(map.isSegmentClear(from: Vec2(0.15, 0.48), to: Vec2(0.85, 0.48), margin: 0.02),
            "Even a low-cost inflated cell must block the follow readiness probe")
    }

    func testSegmentClearanceFailsClosedAtMapBoundaryAndForInvalidInputs() {
        let map = Costmap(width: 10, height: 10, resolution: 0.1)
        XCTAssertFalse(map.isSegmentClear(from: Vec2(0.02, 0.5), to: Vec2(0.2, 0.5), margin: 0.02))
        XCTAssertFalse(map.isSegmentClear(from: Vec2(.nan, 0.5), to: Vec2(0.2, 0.5), margin: 0))
        XCTAssertFalse(map.isSegmentClear(from: Vec2(0.2, 0.5), to: Vec2(0.3, 0.5), margin: -0.01))
    }
}
