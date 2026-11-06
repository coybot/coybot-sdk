import XCTest
import RoverNav
@testable import PhroverKit

final class CostmapBuilderTests: XCTestCase {
    private func recordedHeightFixture(raisedObstacle: Bool = false) -> [SIMD3<Float>] {
        // Synthetic spatial support using the recorded elevations; not a replay
        // of the complete device mesh, which was not captured.
        var vertices = Array(repeating: SIMD3<Float>(-4, -1.2131299, -4), count: 1000)
        for x in -8...8 { for z in -8...8 {
            vertices.append(SIMD3(Float(x) * 0.1 + 0.01, -1.084145, Float(z) * 0.1 + 0.01))
        } }
        if raisedObstacle { vertices.append(SIMD3(0.05, -0.90, 0.05)) }
        return vertices
    }

    func testSupportedLocalGroundDoesNotBecomeObstacleFromDistantLowerMesh() {
        let result = CostmapBuilder.buildWithObserved(from: .init(worldVertices: recordedHeightFixture(), expectedFloorY: -1.10), center: .zero)
        XCTAssertEqual(result.diagnostics["raw_start_cost"], "0")
        XCTAssertEqual(result.diagnostics["inflated_start_cost"], "0")
    }

    func testLocalGroundCorrectionPreservesRaisedObstacleAndRejectsUnsupportedPlatform() {
        let obstacle = CostmapBuilder.buildWithObserved(from: .init(worldVertices: recordedHeightFixture(raisedObstacle: true), expectedFloorY: -1.10), center: .zero)
        XCTAssertEqual(obstacle.diagnostics["raw_start_cost"], "255")
        for reference in [Float(-1.2131299), nil] {
            let unsupported = CostmapBuilder.buildWithObserved(from: .init(worldVertices: recordedHeightFixture(), expectedFloorY: reference), center: .zero)
            XCTAssertEqual(unsupported.diagnostics["raw_start_cost"], "255")
        }
    }

    func testLocalGroundRejectsOneSidedSteepAndMultipleLevelEvidence() {
        let base = recordedHeightFixture()
        let oneSided = base.filter { $0.x >= 0 }
        let steep = base.map { SIMD3<Float>($0.x, -1.10 + $0.x * 0.12, $0.z) }
        let multiple = base + [SIMD3<Float>(0.41, -1.213, 0.41), SIMD3<Float>(0.51, -1.213, 0.41), SIMD3<Float>(0.61, -1.213, 0.41)]
        for vertices in [oneSided, steep, multiple] {
            let assessment = LocalGroundPlane.assess(vertices: vertices, center: .zero, expectedFloorY: -1.10)
            XCTAssertNil(assessment.plane)
            XCTAssertEqual(assessment.diagnostics["local_floor_status"], "rejected")
        }
    }

    func testLocalPlaneDoesNotExtrapolateOrHideLocalDepression() throws {
        var vertices = recordedHeightFixture()
        vertices.append(SIMD3<Float>(2, -1.084145, 0.05))
        vertices.append(SIMD3<Float>(0.05, -1.213, 0.05))
        let assessment = LocalGroundPlane.assess(vertices: vertices, center: .zero, expectedFloorY: -1.10)
        let plane = try XCTUnwrap(assessment.plane)
        XCTAssertNil(plane.height(at: Vec2(2, 0.05)))
        let result = CostmapBuilder.buildWithObserved(from: .init(worldVertices: vertices, expectedFloorY: -1.10), center: .zero)
        XCTAssertEqual(result.diagnostics["raw_start_cost"], "255", "A depression under the rover is not free floor")
        let outside = result.map.worldToCell(Vec2(2, 0.05))
        XCTAssertTrue(result.map.isBlocked(outside.cx, outside.cy))
    }

    func testMountedFloorReferenceNeedsStableDistinctCaptures() throws {
        let valid: [(time: Double, height: Double)] = [(10, -0.8), (10.1, -0.801), (10.2, -0.799)]
        XCTAssertEqual(try XCTUnwrap(LocalGroundPlane.mountedReference(captures: valid, cameraHeight: 0.3)), -1.1, accuracy: 0.001)
        XCTAssertNil(LocalGroundPlane.mountedReference(captures: Array(repeating: valid[0], count: 20), cameraHeight: 0.3))
        XCTAssertNil(LocalGroundPlane.mountedReference(captures: [(10, -0.8), (10.1, -0.8), (10.2, -0.7)], cameraHeight: 0.3))
        XCTAssertNil(LocalGroundPlane.mountedReference(captures: valid, cameraHeight: .nan))
    }

    func testInvalidMeshVertexCannotProduceFreePursuitPath() async {
        let result = await CostmapBuilder.planCaptured(.init(worldVertices: [SIMD3<Float>(.nan, 0, 0)]),
            start: .zero, goal: Vec2(0.5, 0))
        XCTAssertNil(result)
    }

    func testFollowPlanRetainsBlockedStartAndGoalEvidence() async throws {
        for startBlocked in [true, false] {
            let x: Float = startBlocked ? 0.05 : 0.55
            let vertices = Array(repeating: SIMD3<Float>(-0.75, 0.02, -0.75), count: 20)
                + [SIMD3<Float>(x, 0.18, 0.05)]
            let result = await CostmapBuilder.planCaptured(.init(worldVertices: vertices), start: .zero, goal: Vec2(0.5, 0))
            let plan = try XCTUnwrap(result)
            XCTAssertEqual(plan.rejection, startBlocked ? .startBlocked : .goalBlocked)
            XCTAssertTrue(plan.path.isEmpty)
            XCTAssertNotNil(plan.diagnostics["estimated_floor_y_m"])
            XCTAssertEqual(plan.diagnostics["reason"], startBlocked ? "start_blocked" : "goal_blocked")
        }
    }

    func testStartCellEvidenceDistinguishesDirectObstacleFromInflation() {
        for (x, rawCost) in [(Float(0.05), "255"), (Float(0.15), "0")] {
            let vertices = Array(repeating: SIMD3<Float>(-0.75, 0.02, -0.75), count: 20)
                + [SIMD3<Float>(x, 0.18, 0.05)]
            let result = CostmapBuilder.buildWithObserved(from: .init(worldVertices: vertices), center: .zero,
                params: .init(size: 2))
            XCTAssertEqual(result.diagnostics["raw_start_cost"], rawCost)
            XCTAssertEqual(result.diagnostics["inflated_start_cost"], "255")
            XCTAssertEqual(result.diagnostics["start_blocking_vertex_count"], "1")
            XCTAssertFalse(result.diagnostics["start_blocking_samples_world_xyz_m", default: ""].isEmpty)
            let cell = result.map.worldToCell(.zero)
            XCTAssertTrue(result.map.isBlocked(cell.cx, cell.cy), "Diagnostics must not clear real obstacles")
        }
    }

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
