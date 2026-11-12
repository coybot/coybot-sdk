import XCTest
import RoverNav
import simd
@testable import PhroverKit

final class GroundDepthReconciliationTests: XCTestCase {
    private func vertices(raised: Bool = false) -> [SIMD3<Float>] {
        var result: [SIMD3<Float>] = []
        for x in -10...10 { for z in -10...10 {
            result.append(SIMD3(Float(x) * 0.1 + 0.05, -0.84, Float(z) * 0.1 + 0.05))
        } }
        // Overlapping lower reconstruction, including the starting cell.
        for x in 0...2 { for z in 0...2 { result.append(SIMD3(Float(x) * 0.1 + 0.05, -0.92, Float(z) * 0.1 + 0.05)) } }
        result += Array(repeating: SIMD3<Float>(4, -0.92, 4), count: 100)
        // Like the device samples: below the obstacle cutoff relative to the
        // classified floor, but above it relative to the lower global fallback.
        result.append(SIMD3(0.05, -0.75, 0.05))
        if raised { result.append(SIMD3(0.05, -0.68, 0.05)) }
        return result
    }

    private func projection(_ depth: Float, cameraX: Float = 0) -> DepthSafetyProjection {
        var camera = simd_float4x4(simd_quatf(angle: -.pi / 2, axis: SIMD3(1, 0, 0)))
        camera.columns.3 = SIMD4(cameraX, 0.5, 0, 1)
        return .init(width: 64, height: 64, fx: 32, fy: 32, cx: 31.5, cy: 31.5,
            depthValues: Array(repeating: depth, count: 4096), cameraFromWorld: simd_inverse(camera),
            roverOrigin: .zero, roverRight: SIMD3(1, 0, 0), roverForward: SIMD3(0, 0, 1))
    }

    private func evidence(live: Float = 1.34, older: Float = 1.34, now: Double = 10.05, cameraX: Float = 0) -> GroundDepthEvidence? {
        var snapshot = DepthSafetySnapshot(timestamp: 10, cells: [], cellResolution: 0.04,
            validSampleRatio: 1, geometry: RoverConfig.collisionGeometry, projection: projection(live, cameraX: cameraX), failureReason: nil)
        snapshot.history = [.init(timestamp: 9.9, projection: projection(older, cameraX: cameraX), cells: [], cellResolution: 0.04)]
        return GroundDepthEvidence(snapshot: snapshot, at: now)
    }

    func testFreshFloorDepthRefutesLowerGhostWithoutRemovingRaisedObstacle() throws {
        let depth = try XCTUnwrap(evidence())
        let result = CostmapBuilder.buildWithObserved(from: .init(worldVertices: vertices(),
            classifiedFloorY: -0.84, groundDepth: depth), center: .zero)
        XCTAssertEqual(result.diagnostics["local_floor_reference_source"], "classified_floor")
        XCTAssertEqual(result.diagnostics["local_floor_status"], "accepted")
        XCTAssertEqual(result.diagnostics["raw_start_cost"], "0")
        XCTAssertEqual(result.diagnostics["inflated_start_cost"], "0")
        XCTAssertGreaterThan(Int(result.diagnostics["lower_mesh_refuted_cells"] ?? "0") ?? 0, 0)
        let obstacle = CostmapBuilder.buildWithObserved(from: .init(worldVertices: vertices(raised: true),
            classifiedFloorY: -0.84, groundDepth: depth), center: .zero)
        XCTAssertEqual(obstacle.diagnostics["raw_start_cost"], "255")
    }

    func testLowerMissingStaleOccludedAndInconsistentDepthRemainBlocked() {
        for (name, depth) in [("missing", nil), ("invalid", evidence(live: .nan)),
                              ("zero", evidence(live: 0)), ("stale", evidence(now: 10.26)),
                              ("occluded", evidence(live: 1.1, older: 1.1)),
                              ("inconsistent", evidence(older: 1.42)), ("outside", evidence(cameraX: 5)),
                              ("drop", evidence(live: 1.42, older: 1.42))] {
            let result = CostmapBuilder.buildWithObserved(from: .init(worldVertices: vertices(),
                classifiedFloorY: -0.84, groundDepth: depth), center: .zero)
            XCTAssertEqual(result.diagnostics["raw_start_cost"], "255", name)
            XCTAssertEqual(result.diagnostics["local_floor_status"], "accepted", "Keep the supported floor; localize the \(name) conflict")
            XCTAssertEqual(result.diagnostics["start_geometry_conflict"], name == "drop" ? "false" : "true", name)
        }
    }

    func testDepthCannotReplaceIndependentFloorAuthorityOrOutliveCaptureWindow() {
        XCTAssertNil(evidence(now: 9.99))
        XCTAssertNil(evidence(now: 10.26))
        let result = CostmapBuilder.buildWithObserved(from: .init(worldVertices: vertices(),
            expectedFloorY: -0.84, groundDepth: evidence()), center: .zero)
        XCTAssertEqual(result.diagnostics["raw_start_cost"], "255")
        XCTAssertEqual(result.diagnostics["local_floor_rejection"], "multiple_floor_levels")
    }
}
