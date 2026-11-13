import XCTest
import RoverNav
@testable import PhroverKit

final class LocalFloorReplayTests: XCTestCase {
    private let mount: Float = -0.9811032
    private let classified: Float = -0.9652927

    func testRecordedMarginalFloorRequiresIndependentCorroboration() throws {
        let vertices = recordedCells
        // Both single-reference fits reject this geometry at 70% support.
        for reference in [mount, classified] {
            XCTAssertNil(LocalGroundPlane.assess(vertices: vertices, center: .zero, expectedFloorY: reference).plane)
        }
        let result = assess(vertices)
        let plane = try XCTUnwrap(result.plane)
        XCTAssertEqual(result.diagnostics["local_floor_reference_source"], "classified_floor")
        XCTAssertEqual(result.diagnostics["local_floor_support_policy"], "corroborated_marginal_support")
        let inliers = try XCTUnwrap(result.diagnostics["local_floor_inlier_cells"].flatMap(Double.init))
        let eligible = try XCTUnwrap(result.diagnostics["local_floor_reference_consistent_cells"].flatMap(Double.init))
        XCTAssertGreaterThanOrEqual(inliers / eligible, 0.68)
        XCTAssertLessThan(inliers / eligible, 0.70)
        XCTAssertEqual(plane.height, Double(classified), accuracy: 0.02)
        XCTAssertLessThanOrEqual(hypot(plane.xSlope, plane.zSlope), 0.08)
        XCTAssertLessThanOrEqual(try XCTUnwrap(result.diagnostics["local_floor_rms_m"].flatMap(Double.init)), 0.01)
        for reference: Float? in [nil, .nan, classified - 0.021] {
            let unsupported = LocalGroundPlane.assess(vertices: vertices, center: .zero,
                expectedFloorY: reference, classifiedFloorY: classified)
            XCTAssertNil(unsupported.plane)
            XCTAssertEqual(unsupported.diagnostics["local_floor_support_policy"], "standard")
        }
    }

    func testRecordedStartSamplesAreNotObstaclesAgainstCorroboratedFloor() {
        // Logged start-blocking points relative to the Nov 13 11:44 rover pose.
        // These are about 8 cm above the local floor but 17 cm above the global
        // estimate. This is a reduced geometry replay, not the complete map.
        let blockers = [SIMD3<Float>(0.087299, -0.879088, -0.071910), SIMD3<Float>(0.034368, -0.879088, 0.012350)]
        let vertices = recordedCells + blockers + Array(repeating: SIMD3<Float>(-4, -1.0490329, -4), count: 1000)
        let result = build(vertices)
        XCTAssertEqual(result.diagnostics["local_floor_status"], "accepted")
        XCTAssertEqual(result.diagnostics["raw_start_cost"], "0")
        for height: Float in [0.115, 0.145, 0.3] {
            let obstacle = build(vertices + [SIMD3<Float>(0.05, classified + height, 0.05)])
            XCTAssertEqual(obstacle.diagnostics["raw_start_cost"], "255", "Raised geometry remains blocked")
        }
        let lower = build(vertices + [SIMD3<Float>(0.05, classified - 0.2, 0.05)])
        XCTAssertEqual(lower.diagnostics["raw_start_cost"], "255", "Unverified lower geometry remains a hazard")
    }

    func testCorroborationDoesNotReplaceSurroundingSupportOrSlopeChecks() {
        let base = recordedCells
        for vertices in [base.filter { $0.x >= 0 }, base.map { SIMD3<Float>($0.x, classified + $0.x * 0.12, $0.z) }, Array(base.prefix(20))] {
            XCTAssertNil(assess(vertices).plane)
        }
        var competing: [SIMD3<Float>] = []
        for x in -8...8 { for z in -8...8 {
            competing.append(SIMD3(Float(x) * 0.1 + 0.01, classified + ((x + z) % 2 == 0 ? -0.025 : 0.025),
                Float(z) * 0.1 + 0.01))
        } }
        XCTAssertNil(assess(competing).plane, "Two competing surfaces cannot become one clear floor")
    }

    private func assess(_ vertices: [SIMD3<Float>]) -> (plane: LocalGroundPlane?, diagnostics: [String: String]) {
        LocalGroundPlane.assess(vertices: vertices, center: .zero, expectedFloorY: mount, classifiedFloorY: classified)
    }

    private func build(_ vertices: [SIMD3<Float>]) -> (map: Costmap, observed: ObservedGrid, diagnostics: [String: String]) {
        CostmapBuilder.buildWithObserved(from: .init(worldVertices: vertices,
            expectedFloorY: mount, classifiedFloorY: classified), center: .zero)
    }

    // Greedily minimized from all 361 logged reduced fit cells at 2026-11-13
    // 19:44:00Z. Every retained cell is needed to preserve strict rejection and
    // corroborated acceptance. Coordinates are rover-relative; no imagery.
    private var recordedCells: [SIMD3<Float>] {
        let capture = """
        -0.27668428,-0.95759827,-0.4180621;-0.07436538,-0.95759827,-0.60514796;-0.061133146,-0.95759827,-0.7848582
        -0.01799512,-0.95759827,0.42907727;-0.014397144,-0.95759827,0.3992939;-5.2452087e-06,-0.95759827,0.28015983
        0.5177138,-0.9629506,-0.5034331;0.51971865,-0.96468586,-0.01971984;0.5243223,-0.9654428,0.19234228
        0.5259119,-0.9635969,-0.32114208;0.54514337,-0.95759827,0.43667042;0.6044706,-0.9638755,-0.22100556
        0.6064782,-0.96561074,0.26269794;0.60907066,-0.9646324,-0.008931279;0.61526346,-0.9635434,-0.31035268
        0.6172714,-0.9652787,0.17334986;0.61825144,-0.95759827,0.41528404;0.6218494,-0.95759827,0.38550067
        0.6234621,-0.96418965,-0.12806356;0.62806594,-0.9649466,0.08399749;0.62965405,-0.96310073,-0.4294821
        0.69180906,-1.020128,-0.6938999;0.7020184,-0.96446836,-0.027919292;0.70662534,-0.9652252,0.1841315
        0.7128136,-0.9641363,-0.117274165;0.7174188,-0.96489316,0.09478259;0.7183957,-0.95759827,0.3367275
        0.72561085,-0.9655396,0.27709413;0.7749634,-0.99277276,-0.38167346;0.7918315,-0.99908763,-0.47029006
        0.79395294,-0.992557,-0.28872514;0.7965511,-1.0204769,-0.5603743;0.8221574,-0.9652648,0.2283206
        0.86631334,-0.9922809,0.11285174;0.87710726,-0.9923742,0.023501277;0.8833034,-1.0199373,-0.27793133
        0.89467156,-0.99908763,-0.06503129;0.8976952,-1.0203147,-0.39706528;0.898695,-1.0045507,-0.15519953
        0.9402721,-1.0200568,0.0009138584;0.9556637,-1.0196753,0.12364566
        """
        return capture.split(whereSeparator: { $0 == ";" || $0.isNewline }).map { cell in
            let values = cell.split(separator: ",").map { Float($0)! }
            return SIMD3(values[0], values[1], values[2])
        }
    }
}
