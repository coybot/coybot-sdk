import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowPursuitPlanDiagnosticsTests: XCTestCase {
    private func capture(_ plan: FollowPursuitPlan, quality: ARTrackingQuality? = .normal) -> FollowPursuitPlanDiagnostics {
        let source = NavigationPoseSample(pose: .init(position: .zero, yaw: 0),
            frameID: .init(generation: 7, sequence: 42), sourceTimestamp: 10, trackingQuality: quality)
        return .init(plan: plan, capturedSource: source, currentSource: source,
            capturedGoal: Vec2(5, 0), latestGoal: Vec2(5, 0), expectedGeneration: 7,
            capturedRevision: 0, latestRevision: 0, planStartedAt: 10, evaluatedAt: 10.25,
            startupDeadline: 12, sent: false)
    }

    func testSerializedCaptureReconstructsExactMapAndReplaysJoinFailure() throws {
        var map = Costmap(width: 80, height: 80, resolution: 0.1, origin: Vec2(-1, -1))
        map.setCost(15, 10, 1)
        map.setCost(16, 10, Costmap.lethalThreshold)
        map.setCost(20, 20, 252)
        map.setCost(21, 20, 255)
        let plan = FollowPursuitPlan(path: [Vec2(5, 0)], map: map)
        XCTAssertNil(plan.joined(from: .zero))
        var json: String?
        let emitter = FollowDiagnosticEmitter(streamID: "replay-test", monotonic: { 10.25 }, utc: { Date() }) { _, fields in
            json = fields["payload"]
        }
        emitter.emit(.init(event: "follow_pursuit.plan_acceptance_rejected",
            payload: capture(plan).fields(reason: "path_join_blocked", includeGeometry: true)))
        let data = try XCTUnwrap(json?.data(using: .utf8))
        let fields = try XCTUnwrap(JSONSerialization.jsonObject(with: data) as? [String: Any])
        let encoded = try XCTUnwrap(fields["map_cells_base64"] as? String)
        let cells = Array(try XCTUnwrap(Data(base64Encoded: encoded)))
        let origin = try XCTUnwrap(fields["map_origin_xz"] as? [String: Double])
        var replayMap = Costmap(width: try XCTUnwrap(fields["map_width"] as? Int),
            height: try XCTUnwrap(fields["map_height"] as? Int),
            resolution: try XCTUnwrap(fields["map_resolution_m"] as? Double),
            origin: Vec2(try XCTUnwrap(origin["x"]), try XCTUnwrap(origin["z"])))
        XCTAssertEqual(cells.count, replayMap.width * replayMap.height)
        for (index, cost) in cells.enumerated() { replayMap.setCost(index % replayMap.width, index / replayMap.width, cost) }
        let points = try XCTUnwrap(fields["waypoints_xz"] as? [[String: Double]])
        let replayPath = try points.map { Vec2(try XCTUnwrap($0["x"]), try XCTUnwrap($0["z"])) }
        XCTAssertEqual(replayMap.cells, map.cells)
        XCTAssertEqual(replayPath, plan.path)
        XCTAssertNil(FollowPursuitPlan(path: replayPath, map: replayMap).joined(from: .zero))
        XCTAssertEqual(fields["acceptance_reason"] as? String, "path_join_blocked")
        XCTAssertEqual(fields["join_blocking_cost"] as? Int, Int(Costmap.lethalThreshold))
        XCTAssertNil(fields["reason"] as? String, "Envelope reason must not be confused with the acceptance reason")
    }

    func testOversizedGeometryIsExplicitlyOmittedRatherThanSilentlyTruncated() {
        let map = Costmap(width: 129, height: 129, resolution: 0.1)
        let plan = FollowPursuitPlan(path: Array(repeating: Vec2(5, 0), count: 513), map: map)
        let fields = capture(plan).fields(reason: "path_join_blocked", includeGeometry: true)
        XCTAssertEqual(fields["map_capture_status"], .string("omitted_cell_limit"))
        XCTAssertEqual(fields["map_cells_base64"], .null)
        XCTAssertEqual(fields["waypoints_capture_status"], .string("omitted_waypoint_limit"))
        XCTAssertEqual(fields["waypoints_xz"], .null)
        XCTAssertEqual(fields["waypoint_count"], .number(513))
        XCTAssertEqual(fields["path_endpoint_xz"], .object(["x": .number(5), "z": .number(0)]))
    }

    func testNonfiniteWaypointRemainsExplicitAndDoesNotBreakJSONSerialization() throws {
        let plan = FollowPursuitPlan(path: [Vec2(.nan, .infinity)])
        var json: String?
        let emitter = FollowDiagnosticEmitter(streamID: "invalid-test", monotonic: { 10 }, utc: { Date() }) { _, fields in
            json = fields["payload"]
        }
        emitter.emit(.init(event: "test", payload: capture(plan).fields(reason: "nonfinite_waypoint", includeGeometry: true)))
        let fields = try XCTUnwrap(JSONSerialization.jsonObject(with: XCTUnwrap(json?.data(using: .utf8))) as? [String: Any])
        let points = try XCTUnwrap(fields["waypoints_xz"] as? [[String: Any]])
        XCTAssertEqual(points.first?["x_availability"] as? String, "nonfinite")
        XCTAssertEqual(points.first?["z_availability"] as? String, "nonfinite")
    }

    func testTrackingQualityDistinguishesMissingFromUnavailableForReplay() throws {
        for quality: ARTrackingQuality? in [nil, .unavailable, .limited, .normal] {
            let fields = capture(.init(path: [Vec2(5, 0)]), quality: quality)
                .fields(reason: "source_unhealthy_tracking", includeGeometry: true)
            guard case .object(let source)? = fields["current_source"] else { return XCTFail("Missing source") }
            XCTAssertEqual(source["tracking_quality"], quality.map { .string(String(describing: $0)) } ?? .null)
        }
    }
}
