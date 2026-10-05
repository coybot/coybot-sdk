# Tracking and Costmap Recovery Design

## Context

The August 11 iPhone 15 Pro trace after the fresh-depth retry deployment contains three refrigerator missions. Two found a valid target approach path but failed because ARKit remained `limited` beyond the current 1.5-second tracking-recovery window. The third ran while tracking was limited and reported `path_distance=inf` for the direct target, every stand-off goal, three map openings, and four short visual-ray steps. The trace does not expose whether those plans failed because a cell was outside the costmap, blocked, or disconnected.

No `nav_scan_depth_retry_*` event occurred in these missions, so this design does not change the depth retry.

## Goals

- Begin planning and motion only from a stable, fresh AR pose.
- Prevent limited-quality AR mesh updates from contaminating the navigation costmap.
- Recover once from a transient all-unreachable map after stable tracking returns.
- Emit a typed reason for every planner rejection.
- Preserve fail-closed behavior: the rover remains stopped whenever tracking or planning confidence is insufficient.

## Non-goals

- Do not weaken depth, obstacle, communications, tipping, or tracking freshness limits.
- Do not reset the AR session or invalidate world-space mission and room-topology state.
- Do not plan or move from a single momentary normal frame.
- Do not retry planning indefinitely or silently clear obstacles.

## Stable Planning Readiness

`ARSessionManager` will maintain a planning-readiness state derived from accepted pose observations and mesh updates:

- A normal streak increments only for fresh `.normal` observations and resets immediately on `.limited` or `.unavailable` observations.
- Planning becomes pose-ready after three consecutive normal observations.
- Mesh anchors are accepted only while the three-observation normal streak is satisfied. Limited/unavailable callbacks and one-frame normal flickers do not mutate the trusted mesh collection.
- A monotonic trusted-mesh revision advances when a normal-quality mesh add/update is accepted.

`NavigationController` will expose a bounded `recoverPlanningContext()` operation through `RoverMotion`. It sends/awaits a stop, waits up to 3.0 seconds for the three-observation normal streak, then waits within that same deadline for a trusted-mesh revision newer than the revision captured at recovery start. It returns a typed result: ready, tracking timeout, mesh timeout, cancelled, or session generation changed.

The entire deadline is shared; pose recovery and mesh refresh do not each receive a new timeout.

## Navigation Start and Tracking Recovery

Initial planning must move behind the planning-readiness gate. `startNavigation` may claim the operation synchronously, but its asynchronous operation must stop first, await a stable pose and at least one trusted mesh snapshot, then build a path. Unlike explicit recovery from an all-unreachable map, ordinary initial planning does not require a mesh revision newer than the operation start. No wheel command may be emitted before both readiness and planning succeed.

The normal drive loop will use the same three-observation readiness rule and the 3.0-second bounded recovery window. A fleeting normal frame cannot resume motion. Timeout remains a terminal safety failure and records the last tracking quality, normal-streak count, observation age, and elapsed recovery time.

## Typed Planner Outcomes

Add a public, sendable rejection enum to `NavigationGoalAssessment`:

- `missingPose`
- `trackingUnstable`
- `startOutsideMap`
- `goalOutsideMap`
- `startBlocked`
- `goalBlocked`
- `noConnectedPath`

Reachable assessments have no rejection reason. Unreachable assessments must contain exactly one reason. `AStarPlanner` will expose a typed assessment entry point while preserving its existing `plan(...) -> [Vec2]?` convenience API. Precondition checks distinguish bounds and blocked cells; exhaustion of the A* frontier produces `noConnectedPath`.

Every `mission_target_approach_goal_rejected` and `mission_target_staging_candidate` event will include `rejection_reason`. Navigation planning and replanning failures will log the same typed reason.

## One-shot Costmap Recovery

When `approachVisualTarget` finds no reachable direct, stand-off, opening, or incremental-ray candidate:

1. It verifies that no planning-context recovery has already been attempted for this target approach.
2. It invokes `motion.recoverPlanningContext()` while the rover remains stopped.
3. On success, it reacquires the visual target from the new stable frame, rebuilds all assessments from the refreshed trusted mesh, and retries candidate selection once.
4. On tracking timeout, mesh timeout, cancellation, or session-generation change, it terminates through the corresponding existing fail-closed mission path.
5. If the refreshed map still has no reachable candidate, it emits `mission_target_staging_exhausted` with the dominant typed rejection reason and fails `No safe route toward target.`

This recovery never clears the costmap and never reuses the object goal derived from limited tracking.

## Telemetry

Add ordered events around the recovery boundary:

- `nav_planning_recovery_started` with session generation, pose streak, mesh revision, and timeout.
- `nav_planning_pose_ready` after three consecutive normal observations.
- `nav_planning_mesh_refreshed` with previous/current revision.
- `nav_planning_recovery_failed` with typed outcome and final tracking/mesh fields.
- `mission_target_planning_retry` with mission, target, and original dominant rejection reason.

These fields make the physical trace distinguish tracking instability, stale mapping, blocked endpoints, and disconnected free space without logging sensor payloads.

## Testing

### ARSessionManager

- Normal, limited, normal does not satisfy the three-observation readiness gate.
- Three consecutive fresh normal observations do satisfy it.
- Limited-quality mesh callbacks do not change trusted mesh anchors or revision.
- A normal-quality mesh update advances the revision.

### NavigationController

- Recovery remains stopped through tracking flicker and resumes only after three normal observations plus a newer trusted mesh revision.
- Recovery that becomes normal just after 1.5 seconds but before 3.0 seconds succeeds.
- Tracking timeout and mesh timeout send no motion.
- Initial navigation never plans or drives before readiness.
- Typed planner outcomes cover every bounds, blocked-cell, and disconnected-path case.

### MissionAgent

- An all-unreachable visual target invokes planning recovery exactly once, reacquires the target, and retries with the refreshed assessments.
- A still-unreachable refreshed map fails once without returning to the brain loop.
- Tracking, mesh, cancellation, transport, and session-generation failures retain one terminal mission result.

Run focused suites, the complete SDK scheme, `git diff --check`, a signed iPhone 15 Pro build, and a new physical mission trace. Runtime acceptance requires stable-readiness events followed by either safe navigation progress or a typed bounded failure; mission success must not be inferred from safety-only evidence.

## Rejected Alternatives

- **Only raise the tracking timeout:** permits planning from limited-quality pose/mesh and leaves planner failures opaque.
- **Clear or reset the AR session:** invalidates world-space target and room-topology state and can create unsafe continuity assumptions.
- **Retry indefinitely:** consumes battery and can resume unpredictably after the operator believes the mission has stopped.
