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

    func testGeometryFloorCanBlockReadyCorridorThatAstralHeuristicMissed() {
        // Differential fixture, not a replay of the device mesh (not logged).
        // Astral a326011 estimates floor from anchor translation minus 2 m;
        // Coybot retains its geometry-derived floor from 4a885f6.
        let vertices = Array(repeating: SIMD3<Float>(-0.75, 0.02, -0.75), count: 20)
            + [SIMD3<Float>(-0.35, 0.18, 0.15)]
        let params = CostmapBuilder.Params(size: 2)
        let current = CostmapBuilder.buildWithObserved(from: .init(worldVertices: vertices),
            center: .zero, params: params).map
        var astral = Costmap(width: current.width, height: current.height,
            resolution: current.resolution, origin: current.origin)
        let legacyFloorY: Float = 0 - 2.0 // identity mesh-anchor transform
        for point in vertices {
            let height = point.y - legacyFloorY
            if height > Float(params.floorBand), height < Float(params.ceilingBand) {
                astral.markObstacle(at: .init(Double(point.x), Double(point.z)))
            }
        }
        astral.inflate(radius: params.inflationRadius)
        let direction = Vec2(cos(1.446), sin(1.446))
        let goal = direction * 0.10
        let goalCell = current.worldToCell(goal)
        let startCell = current.worldToCell(.zero)
        XCTAssertEqual(current.cost(startCell.cx, startCell.cy), 0)
        XCTAssertEqual(current.cost(goalCell.cx, goalCell.cy), 0)
        // Keep Astral's exact corridor margin on both sides of the comparison:
        // the floor policy alone is enough to change admission.
        XCTAssertTrue(astral.isSegmentClear(from: .zero, to: direction * 0.12, margin: 0.02))
        XCTAssertFalse(current.isSegmentClear(from: .zero, to: direction * 0.12, margin: 0.02))
    }
}
