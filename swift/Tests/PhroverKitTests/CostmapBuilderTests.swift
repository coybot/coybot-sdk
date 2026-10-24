import XCTest
import RoverNav
@testable import PhroverKit

final class CostmapBuilderTests: XCTestCase {
    @MainActor
    func testNoMeshCannotProduceAnAuthoritativeReadyMap() async {
        let map = await CostmapBuilder.buildAsync(from: [], center: .zero)
        XCTAssertNil(map, "Removing the final mesh anchor must not turn unknown space into a clear path")
    }

    func testSnapshotBuildPreservesFloorLowObstaclesInflationAndObservedCoverage() {
        let vertices = Array(repeating: SIMD3<Float>(-0.75, 0.02, -0.75), count: 20)
            + [SIMD3<Float>(0.25, 0.18, 0.25), SIMD3<Float>(0.75, 2.40, -0.75)]
        let result = CostmapBuilder.buildWithObserved(from: .init(worldVertices: vertices),
            center: .zero, params: .init(size: 2))
        let floor = result.map.worldToCell(Vec2(-0.75, -0.75))
        let obstacle = result.map.worldToCell(Vec2(0.25, 0.25))
        let overhead = result.map.worldToCell(Vec2(0.75, -0.75))
        XCTAssertEqual(result.map.cost(floor.cx, floor.cy), 0)
        XCTAssertEqual(result.map.cost(obstacle.cx, obstacle.cy), Costmap.lethal)
        XCTAssertGreaterThan(result.map.cost(obstacle.cx + 1, obstacle.cy), 0)
        XCTAssertEqual(result.map.cost(overhead.cx, overhead.cy), 0)
        XCTAssertTrue(result.observed.isObserved(floor.cx, floor.cy))
        XCTAssertTrue(result.observed.isObserved(overhead.cx, overhead.cy))
    }

    func testFloorEstimateUsesGeometryAndRetainsLowFurniture() {
        let heights: [Float] = Array(repeating: 0.02, count: 20)
            + [0.18, 0.22, 0.75, 2.40]
        let floor = CostmapBuilder.estimatedFloorHeight(fromWorldHeights: heights)

        XCTAssertEqual(floor, 0.02, accuracy: 0.001)
        XCTAssertTrue(CostmapBuilder.isObstacleHeight(0.18, floorY: floor))
        XCTAssertTrue(CostmapBuilder.isObstacleHeight(0.75, floorY: floor))
        XCTAssertFalse(CostmapBuilder.isObstacleHeight(0.02, floorY: floor))
        XCTAssertFalse(CostmapBuilder.isObstacleHeight(2.40, floorY: floor))
    }
}
