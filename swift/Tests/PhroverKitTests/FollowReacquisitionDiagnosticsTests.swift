import XCTest
import RoverNav
@testable import PhroverKit

@MainActor
final class FollowReacquisitionDiagnosticsTests: XCTestCase {
    func testFrozenHeadingFallbackEmitsExactSignedReturnAndUnavailableWorldGeometry() throws {
        let anchor = FollowReliableMemory(position: nil, pairedPose: nil, bearing: 0.3,
            frameID: .init(generation: 2, sequence: 8), timestamp: 1, rawPersonID: 17,
            association: .acceptedPendingContinuity, pairedYaw: .pi - 0.3)
        let current = FollowRecoveryPose(pose: .init(position: Vec2(3, 4), yaw: 0),
            frameID: .init(generation: 2, sequence: 12), timestamp: 2, trackingQuality: .normal)
        let episode = FollowReacquisitionEpisode(firstLoss: 2, anchor: anchor).selectingCenter(current: current, now: 2)
        let sink = FollowDiagnosticRecordingSink()
        let emitter = FollowDiagnosticEmitter(streamID: "fallback", monotonic: { 3 }, utc: { Date() }, sink: sink.append)
        emitter.emit(.init(event: "follow_recovery.center_selected", payload:
            FollowReacquisitionDiagnostics.payload(episode, now: 3, stop: "confirmed")))
        let fields = try XCTUnwrap(sink.records.first?.fields["payload"])
        let json = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(fields.utf8)) as? [String: Any])
        XCTAssertEqual(json["center_source"] as? String, "historical_paired_view_heading")
        XCTAssertEqual(json["center_fallback_reason"] as? String, "world_direction_unavailable_zero_or_nonfinite")
        XCTAssertEqual(json["initial_return_delta_rad"] as? Double, -.pi)
        XCTAssertEqual(json["center_heading_rad"] as? Double, -.pi)
        XCTAssertEqual(json["anchor_raw_person_id"] as? Int, 17)
        XCTAssertEqual(json["anchor_raw_id_scope"] as? String, "frame_local_not_identity")
        XCTAssertEqual(json["anchor_association"] as? String, "acceptedPendingContinuity")
        XCTAssertEqual(json["anchor_age_s"] as? Double, 2)
        XCTAssertEqual(json["anchor_timestamp_s"] as? Double, 1)
        XCTAssertTrue(json["anchor_rover_x"] is NSNull)
        XCTAssertTrue(json["anchor_person_x"] is NSNull)
        XCTAssertTrue(json["measured_movement_rad"] is NSNull)
        XCTAssertTrue(json["center_source_read_uptime_s"] is NSNull, "Source time is not read time")
        XCTAssertEqual(json["center_rover_x"] as? Double, 3)
        XCTAssertEqual(json["center_rover_z"] as? Double, 4)
    }

    func testUnavailableEpisodeNeverFabricatesZeroGeometryOrMovement() throws {
        let episode = FollowReacquisitionEpisode(firstLoss: 1, anchor: nil)
        let fields = FollowReacquisitionDiagnostics.payload(episode, now: 4, stop: "pending")
        XCTAssertEqual(fields["remaining_s"], .number(7))
        XCTAssertEqual(fields["unavailable_reason"], .string("missing_reliable_memory"))
        for key in ["anchor_person_x", "anchor_person_z", "anchor_yaw_rad", "anchor_bearing_rad",
                    "center_heading_rad", "initial_return_delta_rad", "requested_movement_rad", "measured_movement_rad"] {
            XCTAssertEqual(fields[key], .null, key)
            XCTAssertEqual(fields[key + "_availability"], .string("unavailable"), key)
        }
    }
}
