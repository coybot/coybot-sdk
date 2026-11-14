import Foundation
import RoverNav

/// Temporary, bounded evidence for replaying pursuit acceptance failures. No
/// extra sensor reads, collision evaluations, images or raw mesh are captured.
@MainActor
struct FollowPursuitPlanDiagnostics {
    static let maximumWaypoints = 512
    static let maximumMapCells = 16_384

    let plan: FollowPursuitPlan?
    let capturedSource: NavigationPoseSample?
    let currentSource: NavigationPoseSample
    let capturedGoal: Vec2?
    let latestGoal: Vec2
    let expectedGeneration: UInt64?
    let capturedRevision: UInt64?
    let latestRevision: UInt64
    let planStartedAt: Double?
    let evaluatedAt: Double
    let startupDeadline: Double
    let sent: Bool

    func fields(reason: String, includeGeometry: Bool) -> [String: FollowDiagnosticValue] {
        let endpoint = plan?.path.last
        var fields: [String: FollowDiagnosticValue] = [
            "diagnostic_tag": .string("[DEBUG-pursuit-acceptance]"),
            "capture_schema_version": .number(1),
            "acceptance_reason": .string(reason),
            "coordinate_convention": .string("x=world_X; z=world_Z; metres"),
            "captured_source": Self.source(capturedSource),
            "current_source": Self.source(currentSource),
            "captured_goal_xz": Self.point(capturedGoal),
            "latest_goal_xz": Self.point(latestGoal),
            "expected_ar_generation": expectedGeneration.map { .string(String($0)) } ?? .null,
            "captured_revision": capturedRevision.map { .string(String($0)) } ?? .null,
            "latest_revision": .string(String(latestRevision)),
            "plan_started_at_s": planStartedAt.map { .number($0) } ?? .null,
            "evaluated_at_s": .number(evaluatedAt),
            "planning_elapsed_s": planStartedAt.map { .number(evaluatedAt - $0) } ?? .null,
            "planning_deadline_s": planStartedAt.map { .number($0 + FollowPursuitDriver.planningTimeout) } ?? .null,
            "startup_deadline_s": .number(startupDeadline),
            "has_sent_motion": .bool(sent),
            "plan_available": .bool(plan != nil),
            "planning_mode": plan.map { .string($0.planningMode.rawValue) } ?? .null,
            "waypoint_count": plan.map { .number(Double($0.path.count)) } ?? .null,
            "path_endpoint_xz": Self.point(endpoint),
            "endpoint_tolerance_m": plan.map { .number($0.endpointTolerance) } ?? .null,
            "endpoint_error_m": endpoint.flatMap { end in capturedGoal.map { .number(end.distance(to: $0)) } } ?? .null,
            "join_margin_m": .number(FollowPursuitPlan.joinMargin),
            "join_blocking_cost": .number(Double(Costmap.lethalThreshold)),
            "join_evaluation": .string(reason == "path_join_blocked" || reason == "active_route_join_blocked" ? "blocked" : "not_reached_or_not_required"),
            "unmapped_pose_shift_limit_m": .number(FollowPursuitDriver.maximumUnmappedPoseShift),
            "pose_displacement_m": capturedSource?.pose.flatMap { captured in
                currentSource.pose.map { .number(captured.position.distance(to: $0.position)) }
            } ?? .null,
            "map_available": .bool(plan?.map != nil)
        ]
        if let map = plan?.map {
            fields["map_width"] = .number(Double(map.width))
            fields["map_height"] = .number(Double(map.height))
            fields["map_resolution_m"] = .number(map.resolution)
            fields["map_origin_xz"] = Self.point(map.origin)
            fields["map_lethal_threshold"] = .number(Double(Costmap.lethalThreshold))
            // Preserve every cost, including soft inflation. Segment clearance
            // and A* do not necessarily interpret those costs identically.
            fields["map_cells_encoding"] = .string("base64_uint8_row_major")
            let captureMap = includeGeometry && map.cells.count <= Self.maximumMapCells
            fields["map_capture_status"] = .string(!includeGeometry ? "summary_only" : (captureMap ? "complete" : "omitted_cell_limit"))
            fields["map_cells_base64"] = captureMap ? .string(Data(map.cells).base64EncodedString()) : .null
        } else {
            fields["map_capture_status"] = .string("unavailable")
        }
        if let plan {
            let capturePath = includeGeometry && plan.path.count <= Self.maximumWaypoints
            fields["waypoints_capture_status"] = .string(!includeGeometry ? "summary_only" : (capturePath ? "complete" : "omitted_waypoint_limit"))
            fields["waypoints_xz"] = capturePath ? .array(plan.path.map { Self.point($0) }) : .null
            fields["planner_rejection"] = plan.rejection.map { .string($0.rawValue) } ?? .null
        } else {
            fields["waypoints_capture_status"] = .string("unavailable")
        }
        return fields
    }

    private static func point(_ point: Vec2?) -> FollowDiagnosticValue {
        point.map { .object(["x": .number($0.x), "z": .number($0.y)]) } ?? .null
    }

    private static func source(_ sample: NavigationPoseSample?) -> FollowDiagnosticValue {
        guard let sample else { return .null }
        return .object([
            "position_xz": point(sample.pose?.position),
            "yaw_rad": sample.pose.map { .number($0.yaw) } ?? .null,
            "ar_generation": sample.frameID.map { .string(String($0.generation)) } ?? .null,
            "frame_sequence": sample.frameID.map { .string(String($0.sequence)) } ?? .null,
            "source_timestamp_s": sample.sourceTimestamp.map { .number($0) } ?? .null,
            "tracking_quality": sample.trackingQuality.map { .string(String(describing: $0)) } ?? .null,
            "provenance": .string(sample.source)
        ])
    }
}
