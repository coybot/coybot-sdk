import XCTest
@testable import RoverNav

final class AStarPlannerTests: XCTestCase {
    /// 5m x 5m room at 0.1m resolution.
    private func emptyRoom() -> Costmap {
        Costmap(width: 50, height: 50, resolution: 0.1, origin: .zero)
    }

    func testStraightLineInEmptyMap() {
        let map = emptyRoom()
        let path = AStarPlanner().plan(from: Vec2(0.25, 0.25), to: Vec2(4.5, 0.25), in: map)
        XCTAssertNotNil(path)
        XCTAssertEqual(path!.first!.distance(to: Vec2(0.25, 0.25)), 0, accuracy: 0.1)
        XCTAssertLessThanOrEqual(path!.last!.distance(to: Vec2(4.5, 0.25)), 0.15)
    }

    func testRoutesAroundWallWithGap() {
        var map = emptyRoom()
        // Vertical wall at x≈2.5m spanning y=0..4m, leaving a gap near y=4.2m.
        let wallX = 2.5
        var y = 0.0
        while y < 4.0 {
            map.markObstacle(at: Vec2(wallX, y))
            y += 0.05
        }
        map.inflate(radius: 0.15)

        let path = AStarPlanner().plan(from: Vec2(0.5, 0.5), to: Vec2(4.5, 0.5), in: map)
        XCTAssertNotNil(path, "planner must find a route through the gap")

        // Path must never pass through a blocked cell.
        for p in path! {
            let c = map.worldToCell(p)
            XCTAssertFalse(map.isBlocked(c.cx, c.cy), "path crosses an obstacle at \(p)")
        }
        // And it must detour upward past the wall (max y clearly above the start row).
        let maxY = path!.map(\.y).max()!
        XCTAssertGreaterThan(maxY, 3.9, "path should go up to the gap")
    }

    func testUnreachableGoalReturnsNil() {
        var map = emptyRoom()
        // Fully enclose the goal region behind a sealed wall.
        var y = 0.0
        while y <= 5.0 {
            map.markObstacle(at: Vec2(2.5, y))
            y += 0.05
        }
        map.inflate(radius: 0.15)
        let path = AStarPlanner().plan(from: Vec2(0.5, 0.5), to: Vec2(4.5, 0.5), in: map)
        XCTAssertNil(path)
    }

    func testGoalInsideObstacleReturnsNil() {
        var map = emptyRoom()
        map.markObstacle(at: Vec2(2.5, 2.5))
        let path = AStarPlanner().plan(from: Vec2(0.5, 0.5), to: Vec2(2.5, 2.5), in: map)
        XCTAssertNil(path)
    }

    func testAssessmentReportsStartOutsideMap() {
        XCTAssertEqual(
            AStarPlanner().assess(from: Vec2(-0.1, 0.5), to: Vec2(1, 1), in: emptyRoom()),
            .rejected(.startOutsideMap)
        )
    }

    func testAssessmentReportsGoalOutsideMap() {
        XCTAssertEqual(
            AStarPlanner().assess(from: Vec2(0.5, 0.5), to: Vec2(5.1, 1), in: emptyRoom()),
            .rejected(.goalOutsideMap)
        )
    }

    func testAssessmentReportsBlockedStart() {
        var map = emptyRoom()
        map.markObstacle(at: Vec2(0.5, 0.5))

        XCTAssertEqual(
            AStarPlanner().assess(from: Vec2(0.5, 0.5), to: Vec2(1, 1), in: map),
            .rejected(.startBlocked)
        )
    }

    func testAssessmentReportsBlockedGoal() {
        var map = emptyRoom()
        map.markObstacle(at: Vec2(1, 1))

        XCTAssertEqual(
            AStarPlanner().assess(from: Vec2(0.5, 0.5), to: Vec2(1, 1), in: map),
            .rejected(.goalBlocked)
        )
    }

    func testAssessmentReportsDisconnectedPath() {
        var map = emptyRoom()
        var y = 0.0
        while y <= 5.0 {
            map.markObstacle(at: Vec2(2.5, y))
            y += 0.05
        }
        map.inflate(radius: 0.15)

        XCTAssertEqual(
            AStarPlanner().assess(from: Vec2(0.5, 0.5), to: Vec2(4.5, 0.5), in: map),
            .rejected(.noConnectedPath)
        )
    }
}
