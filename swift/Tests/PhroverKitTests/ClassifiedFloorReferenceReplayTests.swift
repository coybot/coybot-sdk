import XCTest
import RoverNav
@testable import PhroverKit

final class ClassifiedFloorReferenceReplayTests: XCTestCase {
    private let mount: Float = -1.2537597
    private let classified: Float = -1.1875758
    private let global: Float = -1.2773969
    private let sampleHeight: Float = -1.110777

    func testRecordedFloorConsensusSurvivesMountDisagreement() throws {
        let vertices = recordedCells
        let strict = LocalGroundPlane.assess(vertices: vertices, center: .zero, expectedFloorY: classified)
        XCTAssertNil(strict.plane)
        XCTAssertEqual(strict.diagnostics["local_floor_rejection"], "inconsistent_surface")
        for reference: Float? in [mount, nil] {
            let result = LocalGroundPlane.assess(vertices: vertices, center: .zero,
                expectedFloorY: reference, classifiedFloorY: classified)
            let plane = try XCTUnwrap(result.plane)
            XCTAssertEqual(result.diagnostics["local_floor_support_policy"], "classified_reference_consensus")
            XCTAssertEqual(plane.height, Double(classified), accuracy: 0.01)
            XCTAssertEqual(Double(sampleHeight) - plane.height, 0.077, accuracy: 0.001)
            XCTAssertNil(plane.height(at: Vec2(2, 0)), "Consensus must stay inside observed local support")
        }
    }

    func testRecordedBlockingSamplesUseLocalFloorAndRaisedObstaclesStayBlocked() {
        let blockers = [SIMD3<Float>(-0.0079323, sampleHeight, 0.0719252), SIMD3<Float>(0.0244947, sampleHeight, 0.0601122)]
        let vertices = recordedCells + blockers + Array(repeating: SIMD3<Float>(-4, global, -4), count: 1000)
        let oldReference = build(vertices, classifiedFloor: nil)
        XCTAssertEqual(oldReference.diagnostics["raw_start_cost"], "255", "Low global tail creates the recorded false blockage")
        let corrected = build(vertices, classifiedFloor: classified)
        XCTAssertEqual(corrected.diagnostics["local_floor_support_policy"], "classified_reference_consensus")
        XCTAssertEqual(corrected.diagnostics["raw_start_cost"], "0")
        for offset: Float in [0.115, 0.145, 0.3, -0.2] {
            let obstacle = build(vertices + [SIMD3<Float>(0.05, classified + offset, 0.05)], classifiedFloor: classified)
            XCTAssertEqual(obstacle.diagnostics["raw_start_cost"], "255", "Raised objects and unverified depressions remain blocked")
        }
    }

    func testConsensusRequiresFreshStableClassifiedEvidence() throws {
        var tracker = ClassifiedFloorTracker()
        let id = UUID()
        let plane = try XCTUnwrap(ClassifiedFloorTracker.Plane(origin: SIMD3(0, Double(classified), 0),
            normal: SIMD3(0, 1, 0), boundary: [Vec2(-2, -2), Vec2(2, -2), Vec2(2, 2), Vec2(-2, 2)]))
        for time in [1.0, 1.1] { tracker.record(id: id, timestamp: time, plane: plane) }
        XCTAssertNil(assess(recordedCells, reference: tracker.reference(at: .zero, now: 1.1)).plane)
        tracker.record(id: id, timestamp: 1.2, plane: plane)
        XCTAssertNotNil(assess(recordedCells, reference: tracker.reference(at: .zero, now: 1.2)).plane)
        XCTAssertNil(assess(recordedCells, reference: tracker.reference(at: .zero, now: 3.3)).plane)
        tracker.remove([id])
        XCTAssertNil(assess(recordedCells, reference: tracker.reference(at: .zero, now: 1.3)).plane)
    }

    func testConsensusRejectsSparseOneSidedAndConflictingSupport() {
        let base = recordedCells
        XCTAssertNil(assess(base, reference: .nan).plane)
        XCTAssertNil(assess(base, reference: classified + 0.05).plane)
        // Removing one floor inlier preserves the marginal support ratio while
        // dropping below the consensus branch's 48-inlier minimum.
        for vertices in [Array(base.dropFirst()), base.filter { $0.x >= 0 },
                         base.map { SIMD3<Float>($0.x, classified + 0.12 * $0.x, $0.z) }] {
            XCTAssertNil(assess(vertices, reference: classified).plane)
        }
        var competing: [SIMD3<Float>] = []
        for x in -8...8 { for z in -8...8 {
            competing.append(SIMD3(Float(x) * 0.1 + 0.01, classified + ((x + z) % 2 == 0 ? -0.025 : 0.025),
                Float(z) * 0.1 + 0.01))
        } }
        XCTAssertNil(assess(competing, reference: classified).plane)
    }

    func testMarginalConsensusCannotTradeAwayHeightAgreementOrResidual() {
        let shifted = recordedCells.map { point in
            abs(point.y - classified) < 0.005 ? SIMD3<Float>(point.x, point.y + 0.012, point.z) : point
        }
        let mismatch = assess(shifted, reference: classified)
        XCTAssertNil(mismatch.plane)
        XCTAssertEqual(mismatch.diagnostics["classified_consensus_rejection"], "slope_or_mount_mismatch")
        let noisy = recordedCells.enumerated().map { index, point in
            abs(point.y - classified) < 0.005 ? SIMD3<Float>(point.x, point.y + (index % 2 == 0 ? 0.008 : -0.008), point.z) : point
        }
        let residual = assess(noisy, reference: classified)
        XCTAssertNil(residual.plane)
        XCTAssertEqual(residual.diagnostics["classified_consensus_rejection"], "high_residual")
    }

    private func assess(_ vertices: [SIMD3<Float>], reference: Float?) -> (plane: LocalGroundPlane?, diagnostics: [String: String]) {
        LocalGroundPlane.assess(vertices: vertices, center: .zero, expectedFloorY: mount, classifiedFloorY: reference)
    }

    private func build(_ vertices: [SIMD3<Float>], classifiedFloor: Float?) -> (map: Costmap, observed: ObservedGrid, diagnostics: [String: String]) {
        CostmapBuilder.buildWithObserved(from: .init(worldVertices: vertices,
            expectedFloorY: mount, classifiedFloorY: classifiedFloor), center: .zero)
    }

    // Greedily minimized from the reduced estimator capture at 2026-11-13
    // 21:24:35Z. Retains standard-fit rejection and classified-consensus success.
    // Coordinates are rover-relative; this is not the complete device mesh.
    private var recordedCells: [SIMD3<Float>] {
        let capture = """
        -0.27748847,-1.1875752,-0.11776489;-0.09223938,-1.1875757,0.5908131;-0.089986086,-1.1875755,0.35651642
        -0.07968235,-1.1875756,0.483378;-0.06723404,-1.1875755,-0.41152152;-0.056930304,-1.1875756,-0.28466004
        0.01744914,-1.1875753,0.36907363;0.027439833,-1.1875752,0.29882622;0.101914406,-1.1875753,-0.42525983
        0.103171825,-1.1875752,0.40254897;0.10559535,-1.1875752,0.36523986;0.10760236,-1.1875751,-0.6172693
        0.114471436,-1.1875752,-0.53269494;0.14695048,-1.1875755,-0.22951567;0.17843866,-1.2208275,-0.793287
        0.17937875,-1.1875755,-0.19211704;0.18396282,-1.2307769,-0.8924459;0.18592405,-1.1875753,-0.31450292
        0.20874381,-1.1875753,-0.2557431;0.20934963,-1.1875752,-0.4127026;0.20945859,-1.1875751,0.37476152
        0.2129991,-1.1875752,0.5095821;0.21503758,-1.187575,-0.60471207;0.2163279,-1.1875751,0.45933592
        0.2219069,-1.1875751,-0.52013785;0.27247643,-1.1875752,-0.31969133;0.2790048,-1.2474747,-0.86530405
        0.28587365,-1.2227271,-0.78072995;0.30113435,-1.1875751,-0.3347075;0.30433655,-1.1875751,0.49475402
        0.30472732,-1.1875752,0.58695215;0.30648112,-1.187575,-0.5270069;0.31345916,-1.187575,0.3450315
        0.3165586,-1.1875751,-0.40033785;0.31903815,-1.1875749,-0.63444215;0.38322282,-1.2121586,0.67989427
        0.40135908,-1.187575,-0.4070146;0.4023459,-1.1875751,0.69614404;0.40546203,-1.1875749,0.38384354
        0.40822816,-1.187575,-0.3224404;0.411772,-1.187575,0.50731105;0.41391635,-1.1875749,-0.51444983
        0.46219158,-1.2307769,0.78636736;0.48475218,-1.2230564,-0.69046754;0.49291158,-1.2126322,0.45815474
        0.5011723,-1.2307769,0.74075454;0.50535965,-1.1875747,-0.43674466;0.5168445,-1.1875752,-0.033299327
        0.54206777,-1.2349483,0.5392944;0.55133367,-1.2307769,0.46843094;0.5705998,-1.2307769,0.68164545
        0.58553624,-1.1875751,0.8124438;0.58660865,-1.2147104,0.30156332;0.58776474,-1.1875753,0.26317745
        0.5990565,-1.2231547,-0.59333605;0.60087514,-1.2307769,0.5892666;0.60141873,-1.1875751,-0.040168524
        0.6024895,-1.187575,-0.15736455;0.608397,-1.1875751,0.83187014;0.60881925,-1.2307769,0.6353857
        0.6108117,-1.2307769,0.40086728;0.6185918,-1.1875752,0.17126721;0.6455102,-1.2307769,0.31224734
        0.6998404,-1.187575,0.90957516;0.7019849,-1.1875749,-0.11218572;0.7055546,-1.187575,0.9028507
        0.7705674,-1.2398133,-0.0539068;0.79686296,-1.239145,0.007806599;0.85269225,-1.2307769,0.09461695
        """
        return capture.split(whereSeparator: { $0 == ";" || $0.isNewline }).map { cell in
            let values = cell.split(separator: ",").map { Float($0)! }
            return SIMD3(values[0], values[1], values[2])
        }
    }
}
