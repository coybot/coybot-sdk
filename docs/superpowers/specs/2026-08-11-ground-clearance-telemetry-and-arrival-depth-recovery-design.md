# Ground Clearance Telemetry and Arrival Depth Recovery Design

## Context

The post-install runtime trace from the paired iPhone 15 Pro shows two distinct signals:

- geometry-aware depth safety allowed the clear-floor navigation run;
- the mission later failed at arrival because its raw-depth snapshot was 0.408 seconds old, beyond the 0.25-second freshness limit.

The trace also shows the legacy image-space `forward_clearance` value dropping to 0.12–0.22 m when the low-mounted camera viewed the floor. That value samples the tenth percentile of a fixed central depth rectangle and does not classify floor in rover coordinates. It is already excluded from the main navigation obstacle gate, but its logs and telemetry misleadingly look like an active ground stop.

## Goal

Remove the obsolete ground-sensitive clearance signal and recover from a transient stale raw-depth snapshot at arrival without weakening geometry-aware collision safety.

## Non-goals

- Do not disable `DepthSafetyEvaluator`.
- Do not treat stale, missing, malformed, or blind depth as clear.
- Do not change the collision-height band, obstacle support threshold, swept-volume coverage, stopping distance, or caution speed.
- Do not weaken communications, tipping, tracking, planning, or progress-watchdog safety.
- Do not change object-depth sampling or visual target grounding.

## Design

### Remove legacy forward-clearance state

Remove `ARSessionManager.forwardClearance`, its fixed image-space percentile calculation, its periodic `forward_clearance` log, and navigation telemetry derived from that value. Remove the associated dead visual-target helper functions and their tests if no production caller exists.

The general navigation safety gate will continue checking communications and tipping, with geometry-aware obstacle decisions supplied separately by `DepthSafetyEvaluator`. No active obstacle protection is removed because the legacy forward-clearance veto is already invoked with `checkForwardObstacle: false`.

### Retry stale depth at arrival

When pursuit reports that the goal has been reached, navigation will stop the transport before evaluating arrival safety. If the arrival probe returns `stale_raw_depth`, navigation will wait up to the existing depth-recovery timeout for a newer depth snapshot, then restart the loop and re-evaluate arrival against that snapshot.

The retry is limited to `stale_raw_depth`. A missing, malformed, blind, invalid-calibration, hazard-stop, or caution-without-safe-command result remains a fail-closed arrival failure. If no newer snapshot arrives before the deadline, navigation fails with the existing stale-depth safety reason and emits retry-timeout telemetry.

Stopping before the wait guarantees that no motor command is authorized by stale depth. Restarting the loop rechecks operation ownership, tracking, planning, communications, pose, and depth instead of carrying stale state across the asynchronous wait.

## Telemetry

Add arrival-specific events for retry start, fresh-snapshot recovery, and timeout. Include the previous and current depth versions, sample age, and timeout where applicable. Retain the final `nav_safety_stop` record when recovery fails.

Remove the misleading `forward_clearance` event and `forward_clearance` field from drive telemetry. Geometry-aware `depth_safety_state`, `depth_safety_clearance`, `depth_safety_age`, and `depth_safety_support` remain the authoritative obstacle evidence.

## Testing

- Add an arrival regression where the initial observation is `stale_raw_depth`, a newer eligible snapshot arrives, and navigation reports arrival without sending a movement command.
- Add an arrival timeout regression proving stale depth still fails closed when no new snapshot arrives.
- Preserve the existing missing-depth arrival regression.
- Remove or update tests that cover dead legacy forward-clearance helpers.
- Run focused navigation and depth-safety tests, the full SDK test suite, and `git diff --check`.
- Build and install the signed app on the paired iPhone 15 Pro.

## Physical Acceptance

With the rover stationary on clear floor and the camera fixed at 30 cm:

- a transient stale depth frame at arrival recovers when a fresh frame becomes available;
- no `forward_clearance` telemetry is emitted;
- geometry-aware depth telemetry remains present;
- missing or persistently stale depth still fails closed;
- a real obstacle in the collision band still produces caution or stop.
