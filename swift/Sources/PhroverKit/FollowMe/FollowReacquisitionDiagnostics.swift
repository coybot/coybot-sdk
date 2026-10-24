import Foundation

/// Formats frozen decisions only. Never reads a pose provider or authorizes motion.
enum FollowReacquisitionDiagnostics {
    static func payload(_ episode: FollowReacquisitionEpisode, now: Double,
                        stop: String, centerSource: NavigationPoseSample? = nil,
                        centerReadUptime: Double? = nil) -> [String: FollowDiagnosticValue] {
        let anchor = episode.anchor
        let center = episode.center
        let offset = FollowReacquisitionPlanner.offsets.indices.contains(episode.stageIndex)
            ? FollowReacquisitionPlanner.offsets[episode.stageIndex] : nil
        let fields: [String: FollowDiagnosticValue] = [
            "episode_id": .string(episode.id.uuidString), "first_loss_s": .number(episode.firstLoss),
            "deadline_s": .number(episode.deadline), "elapsed_s": .number(now - episode.firstLoss),
            "remaining_s": .number(max(0, episode.deadline - now)), "deadline_clock": .string("system_uptime"),
            "stop_outcome": .string(stop), "anchor_frame_id": frame(anchor?.frameID),
            "anchor_association": anchor.map { .string($0.association.rawValue) } ?? .null,
            "anchor_generation": number(anchor.map { Double($0.frameID.generation) }),
            "anchor_timestamp_s": number(anchor?.timestamp), "anchor_age_s": number(anchor.map { now - $0.timestamp }),
            "anchor_raw_person_id": number(anchor?.rawPersonID.map(Double.init)),
            "anchor_raw_id_scope": .string("frame_local_not_identity"),
            "anchor_person_x": number(anchor?.position?.x), "anchor_person_z": number(anchor?.position?.y),
            "anchor_rover_x": number(anchor?.pairedPose?.position.x), "anchor_rover_z": number(anchor?.pairedPose?.position.y),
            "anchor_yaw_rad": number(anchor?.pairedYaw), "anchor_bearing_rad": number(anchor?.bearing),
            "anchor_view_heading_rad": number(anchor.flatMap { memory in
                guard let yaw = memory.pairedYaw, let bearing = memory.bearing else { return nil }
                return FollowReacquisitionPlanner.wrap(yaw + bearing)
            }),
            "anchor_validity": .string(anchor == nil ? "unavailable" : "last_reliable_accepted_not_fresh_authority"),
            "anchor_pairing": .string(anchor?.pairedPose == nil ? "unavailable" : "same_observation"),
            "center_heading_rad": number(center?.heading),
            "center_source": .string(center.map { $0.source == .worldFromPostStopPose
                ? "world_from_post_stop_pose" : "historical_paired_view_heading" } ?? "unavailable"),
            "center_fallback_reason": center?.source == .historicalPairedViewHeading
                ? .string("world_direction_unavailable_zero_or_nonfinite") : .null,
            "center_initial_yaw_rad": number(center?.sample.pose.yaw),
            "initial_return_delta_rad": number(center.map { FollowReacquisitionPlanner.wrap($0.heading - $0.sample.pose.yaw) }),
            "center_source_frame_id": frame(center?.sample.frameID),
            "center_source_timestamp_s": number(center?.sample.timestamp),
            "center_source_generation": number(center.map { Double($0.sample.frameID.generation) }),
            "center_source_read_uptime_s": number(centerReadUptime),
            "center_source_age_at_selection_s": number(centerReadUptime.flatMap { read in center.map { read - $0.sample.timestamp } }),
            "center_source_identity": centerSource.map { .string($0.source) } ?? .null,
            "center_rover_x": number(center?.sample.pose.position.x), "center_rover_z": number(center?.sample.pose.position.y),
            "stage_index": .number(Double(episode.stageIndex)), "segment_index": .number(Double(episode.segmentIndex)),
            "stage_name": .string(episode.stageIndex == 0 ? "center_return" : (offset == nil ? "exhausted" : "offset")),
            "stage_offset_rad": number(offset),
            "stage_target_rad": number(center.flatMap { center in offset.map { FollowReacquisitionPlanner.wrap(center.heading + $0) } }),
            "pass_exhausted": .bool(offset == nil), "unavailable_reason": episode.unavailableReason.map { .string($0) } ?? .null,
            "measured_movement_rad": .null, "requested_movement_rad": .null
        ]
        return withAvailability(fields)
    }

    static func number(_ value: Double?) -> FollowDiagnosticValue { value.map { .number($0) } ?? .null }
    static func snapshot(memory: FollowReliableMemory?, provisional: FollowPersonObservation?,
                         episode: FollowReacquisitionEpisode?, healthy: Bool, currentFrame: ARFrameID?) -> [String: FollowDiagnosticValue] {
        ["recovery_active": .bool(episode != nil),
         "reliable_frame_id": frame(memory?.frameID), "reliable_timestamp_s": number(memory?.timestamp),
         "reliable_person_x": number(memory?.position?.x), "reliable_person_z": number(memory?.position?.y),
         "reliable_association": memory.map { .string($0.association.rawValue) } ?? .null,
         "reliable_availability": .string(memory == nil ? "unavailable" : "last_valid_not_current_authority"),
         "provisional_frame_id": frame(episode == nil ? nil : provisional?.frameID),
         "provisional_person_x": number(episode == nil ? nil : provisional?.position.x),
         "provisional_person_z": number(episode == nil ? nil : provisional?.position.y),
         "provisional_raw_person_id": number(episode == nil ? nil : provisional?.rawPersonID.map(Double.init)),
         "current_target_availability": .string(!healthy ? "unavailable_unhealthy_perception" :
            (provisional?.frameID == currentFrame && provisional != nil ? "current_matched_lock" : "unavailable_no_current_match")),
         "snapshot_boundary": .string("captured_decision_not_reassociated")]
    }
    @MainActor
    static func controllerPayload(_ evidence: FollowMotionOperationEvidence) -> [String: FollowDiagnosticValue] {
        let segment = evidence.recovery
        var fields = segmentPayload(segment, expectedGeneration: evidence.recoveryExpectedGeneration)
        fields["episode_id"] = evidence.recoveryEpisodeID.map { .string($0.uuidString) } ?? .null
        fields["deadline_s"] = number(evidence.recoveryDeadline)
        fields["deadline_clock"] = .string(evidence.recoveryDeadline == nil ? "unknown" : "system_uptime")
        fields["stage_target_rad"] = number(segment?.stageHeading ?? evidence.recoveryStageHeading)
        fields["expected_generation"] = number(evidence.recoveryExpectedGeneration.map(Double.init))
        fields["authorization_checked_uptime_s"] = number(evidence.recoveryAuthorizationTime)
        fields["authorization_outcome"] = evidence.recoveryAuthorizationOutcome.map { .string($0) } ?? .null
        fields["authorization_remaining_s"] = number(evidence.recoveryAuthorizationTime.flatMap { time in
            evidence.recoveryDeadline.map { max(0, $0 - time) }
        })
        return withAvailability(fields)
    }

    static func segmentPayload(_ segment: FollowRecoverySegmentEvidence?, expectedGeneration: UInt64? = nil) -> [String: FollowDiagnosticValue] {
        let before = segment?.resolutionSource?.pose?.yaw
        let after = segment?.arrivalSource?.pose?.yaw
        var fields: [String: FollowDiagnosticValue] = [
            "stage_target_rad": number(segment?.stageHeading), "segment_target_rad": number(segment?.segmentHeading),
            "requested_movement_rad": number(segment?.requestedDelta),
            "measured_pre_yaw_rad": number(before), "measured_post_yaw_rad": number(after),
            "measured_movement_rad": number(before.flatMap { before in after.map { FollowReacquisitionPlanner.wrap($0 - before) } }),
            "stage_error_rad": number(after.flatMap { yaw in segment.map { FollowReacquisitionPlanner.wrap($0.stageHeading - yaw) } }),
            "segment_error_rad": number(after.flatMap { yaw in segment?.segmentHeading.map { FollowReacquisitionPlanner.wrap($0 - yaw) } }),
            "segment_arrived": segment.map { .bool($0.segmentArrived) } ?? .null,
            "stage_arrived": segment?.stageArrived.map { .bool($0) } ?? .null,
            "movement_measurement_source": .string("ar_visual_inertial"),
            "movement_scope": .string("segment_resolution_to_final_post_stop_not_cumulative_coverage")
        ]
        for (name, sample, read) in [("controller_post_stop", segment?.postStopSource, segment?.postStopReadUptime),
                                     ("controller_resolution", segment?.resolutionSource, segment?.resolutionReadUptime),
                                     ("controller_arrival", segment?.arrivalSource, segment?.arrivalReadUptime)] {
            fields[name + "_frame_id"] = frame(sample?.frameID)
            fields[name + "_generation"] = number(sample?.frameID.map { Double($0.generation) })
            fields[name + "_timestamp_s"] = number(sample?.sourceTimestamp)
            fields[name + "_read_uptime_s"] = number(read)
            fields[name + "_age_s"] = number(read.flatMap { read in sample?.sourceTimestamp.map { read - $0 } })
            fields[name + "_source_identity"] = sample.map { .string($0.source) } ?? .null
            fields[name + "_tracking"] = sample?.trackingQuality.map { .string(String(describing: $0)) } ?? .null
            fields[name + "_availability"] = .string(sample == nil ? "unavailable" :
                (read.flatMap { sample?.rejection(at: $0, expectedGeneration: expectedGeneration) }
                    ?? (read == nil ? "unknown_read_time" : "available")))
            fields[name + "_rover_x"] = number(sample?.pose?.position.x)
            fields[name + "_rover_z"] = number(sample?.pose?.position.y)
            fields[name + "_yaw_rad"] = number(sample?.pose?.yaw)
        }
        if let first = segment?.resolutionSource?.frameID, let last = segment?.arrivalSource?.frameID {
            fields["segment_pose_pairing"] = .string(first == last ? "same_frame" : "independently_sampled")
        } else { fields["segment_pose_pairing"] = .string("unknown") }
        return withAvailability(fields)
    }

    private static func withAvailability(_ fields: [String: FollowDiagnosticValue]) -> [String: FollowDiagnosticValue] {
        var result = fields
        for (key, value) in fields {
            switch value {
            case .null: result[key + "_availability"] = .string("unavailable")
            case .number(let value): result[key + "_availability"] = .string(value.isFinite ? "available" : "nonfinite")
            default: break
            }
        }
        return result
    }

    static func frame(_ value: ARFrameID?) -> FollowDiagnosticValue {
        value.map { .string("\($0.generation):\($0.sequence)") } ?? .null
    }
}
