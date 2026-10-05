# Fail-Closed Visual Navigation Recovery Design

## Goal

Allow a visible-target mission such as "Go to refrigerator" to recover from a transient stale depth frame and an unreachable direct target goal without weakening any motion safety gate.

## Observed Failure

The August 11 iPhone 15 Pro runtime trace shows two consecutive recovery gaps:

1. A visual-target scan attempts both rotation directions. Both rotations stop immediately with `stale_raw_depth`, even though a newer depth frame may become available within the existing recovery interval.
2. A later mission recognizes the refrigerator and receives successful on-device brain decisions, but every direct stand-off goal is unreachable. The brain-driven path does not use the staged approach already available to offline object fallback, repeats the unreachable target, and eventually selects an opening that fails with `No path to goal.`

Speech recognition, on-device reasoning, and rover HTTP transport succeeded during this trace. They are outside this fix.

## Safety Contract

- Navigation remains fail-closed.
- A stale depth observation never authorizes motion.
- Before retrying a rotation, the rover stops and waits for a strictly newer depth snapshot.
- The original rotation command is re-evaluated against the new snapshot. Motion proceeds only when every existing general-safety and depth-safety check allows it.
- A timeout, cancellation, session change, or unsafe refreshed observation leaves the rover stopped.
- Target staging uses only goals that the current planner reports as reachable.
- Costmap occupancy, obstacle thresholds, stand-off distances, and depth thresholds are not relaxed.

## Design

### Rotation depth freshness recovery

`NavigationController.rotationSafetyCommand` will classify both `blind_swept_volume` and `stale_raw_depth` as depth conditions that can justify waiting for a new snapshot while the rover is stopped. The existing bounded `waitForNewDepthSnapshot` helper remains the only freshness wait.

After a new snapshot arrives, the controller re-runs depth evaluation for the exact requested rotation. If it becomes safe, the controller repeats the existing general-safety check with a fresh rover acknowledgement before returning the command. If refreshed depth is still stale or otherwise unsafe, the controller reports the final depth reason and returns no command. The depth-visible arc fallback remains limited to a fresh `blind_swept_volume` result; stale depth does not select an alternate motion shape.

Telemetry will distinguish the initial and refreshed depth states so a future device log proves whether freshness recovery succeeded or failed.

### Brain-driven visual-target staging

The visual-target navigation branch driven by a successful brain decision will use the same staged approach policy as offline object fallback when no reachable direct or stand-off goal exists.

The staged approach:

1. Reassesses the visible target against the current pose and costmap.
2. Prefers an unexplored doorway candidate that is planner-reachable and makes measurable progress toward the target.
3. If no such doorway is available, considers bounded incremental points along the current-to-target ray.
4. Navigates to one selected staging point at a time, waits for a fresh tracked perception frame, reacquires the target, and reassesses its approach goal.
5. Stops with `No safe route toward target.` when no planner-confirmed progress candidate exists or the bounded attempt budget is exhausted.

The shared behavior must preserve mission cancellation and room-mapping session generation checks. A brain-driven mission must not silently continue thinking after staged navigation has produced a terminal failure.

## Error Handling

- Stale rotation depth with no fresh snapshot: stop and fail with a depth freshness reason.
- Refreshed but unsafe rotation depth: stop and report the refreshed safety state.
- No reachable direct target goal and no reachable staging candidate: stop and publish a terminal navigation failure.
- Failed staging motion: try another bounded candidate only for existing recoverable target-approach failures; otherwise stop immediately.
- Mission cancellation or room-mapping session reset: cancel motion and return without publishing stale success.

## Testing

Add focused regressions before production changes:

1. A rotation receives `stale_raw_depth`, a newer snapshot becomes clear, and exactly one safe rotation command is eventually sent.
2. A rotation receives `stale_raw_depth`, no newer snapshot arrives, and no motion command is sent.
3. A brain-driven visible-target decision has no reachable direct stand-off goal, uses a reachable staging candidate, reacquires the target, and completes navigation.
4. A brain-driven visible-target decision has no reachable direct or staging goal, terminates once with `No safe route toward target.`, and does not ask the brain to select an unreachable opening.

Run the focused navigation and mission-agent tests, the complete iOS SDK test scheme, `git diff --check`, and a signed iPhone build. Install the verified build on the iPhone 15 Pro and collect a new physical-device trace before claiming runtime recovery.

## Scope

This change does not modify speech capture, Apple Intelligence timeout policy, object detection thresholds, rover networking, costmap construction, or general navigation safety thresholds. Existing unrelated and uncommitted worktree changes remain preserved.
