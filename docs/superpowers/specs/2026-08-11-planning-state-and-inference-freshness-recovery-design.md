# Planning State and Inference Freshness Recovery Design

## Context

The post-install iPhone 15 Pro trace exposes two independent orchestration defects.

First, `NavigationController` now enters `.planning` synchronously while its owned task stops the rover, waits for stable AR readiness, and builds a path. `MissionAgent.waitForMotionToSettle()` waits only while the state is `.driving`, so it treats `.planning` as terminal. The brain then issues another decision that replaces the active navigation operation before it can drive.

Second, visual target resolution performs synchronous Vision/CoreML inference on the main actor. That work can exceed the 0.5-second navigation-observation freshness window and delay AR frame ingestion. The planning recovery operation correctly observes three normal frames and a newer trusted mesh, but the current sequence runs target inference after recovery. Subsequent goal assessments therefore reject the now-aged observation as `tracking_unstable` before A* evaluates the refreshed costmap.

## Goals

- Treat planning and driving as active motion states throughout mission orchestration.
- Prevent a new brain decision from replacing navigation while path planning is still active.
- Ensure visual-target goal assessment begins from a fresh, stable AR pose after synchronous inference completes.
- Preserve the existing three-normal-frame gate, 0.5-second freshness limit, trusted-mesh rules, three-second timeout, session-generation checks, and one refreshed-map retry.
- Keep every timeout and cancellation path fail-closed with no motor command emitted from an unready planning context.

## Non-goals

- Do not move Vision/CoreML inference off the main actor in this change.
- Do not increase or remove the navigation freshness limit.
- Do not reset the AR session, clear the costmap, or reuse a target across a session-generation change.
- Do not add additional brain retries or map-refresh retries.
- Do not alter depth, obstacle, communications, tipping, or command-watchdog safety behavior.

## Active Motion Semantics

Add one canonical state predicate owned by `NavigationController.State`:

- `.planning` and `.driving` are active.
- `.idle`, `.arrived`, and `.failed` are settled.

`MissionAgent.waitForMotionToSettle()` will use this predicate rather than matching `.driving` directly. It will continue checking mission cancellation and session generation while either active state is present. This keeps the state-machine knowledge local and prevents future callers from independently defining incomplete active-state sets.

The wait has no new timeout. Navigation operations already have bounded readiness, planning, transport, and drive termination paths. A caller cancellation or session-generation change remains authoritative and stops the rover before returning.

## Planning Context Interface

Replace the single-purpose recovery operation with one planning-context interface:

```swift
public enum PlanningContextRequirement: Equatable, Sendable {
    case stablePose
    case refreshedTrustedMesh
}

func preparePlanningContext(
    requiring requirement: PlanningContextRequirement
) async -> PlanningRecoveryOutcome
```

The interface hides stop coordination, observation polling, deadline management, mesh revision tracking, cancellation, and telemetry.

- `stablePose` sends and awaits stop, then requires three consecutive normal observations whose latest sample is within the existing freshness limit.
- `refreshedTrustedMesh` captures the trusted-mesh revision at entry and, within the same three-second deadline, requires both stable pose readiness and a newer trusted-mesh revision.

`NavigationController` is the production adapter. The default `RoverMotion` adapter returns `.ready` so existing non-AR fakes remain source-compatible; focused mission tests override the method to script outcomes.

The current `recoverPlanningContext()` name may remain temporarily as a source-compatible wrapper for `.refreshedTrustedMesh`, but new production call sites must use the explicit requirement. The wrapper should be removed only in a separate cleanup after all consumers migrate.

## Visual Target Data Flow

### Initial visual approach

1. Detect, lock, and unproject the visual target into the current AR world coordinate system.
2. If the first direct or stand-off assessment returns `trackingUnstable`, call `preparePlanningContext(requiring: .stablePose)`.
3. On `.ready`, reassess the already-unprojected world goal without running detector inference again.
4. On timeout, cancellation, or session-generation change, terminate through the existing fail-closed mission path.

The world goal may be retained only within the same AR session generation. The readiness wait refreshes the rover start pose used for planning; it does not reinterpret the target using a different camera frame.

### All-unreachable refreshed-map retry

1. When every direct, stand-off, opening, and incremental goal is unreachable, verify the retry has not already been consumed.
2. Reacquire and unproject the target first. This is the only detector inference in the retry path.
3. Call `preparePlanningContext(requiring: .refreshedTrustedMesh)` after inference completes.
4. On `.ready`, rebuild all goal assessments immediately from the fresh pose and newer trusted mesh without invoking detector inference again.
5. If the refreshed assessments are still unreachable, fail once with `No safe route toward target.` and the dominant typed planner rejection.

Reacquisition before readiness is safe only because the resulting world goal is generation-bound. Any session-generation change invalidates the goal and terminates the mission.

## Error Handling

- `trackingTimeout`: stop and report that AR tracking did not recover.
- `meshTimeout`: stop and report that the navigation map did not refresh.
- `cancelled`: preserve the existing mission cancellation result without further perception or planning.
- `sessionGenerationChanged`: discard the world goal, stop, and preserve the existing session-change terminal path.
- Transport stop failure: preserve the transport failure rather than translating it into a tracking or map failure.

No non-ready outcome may proceed to A*, navigation start, or a motor command.

## Telemetry

Retain the existing planning recovery events and add the planning requirement to their fields:

- `nav_planning_recovery_started`: `requirement=stable_pose|refreshed_trusted_mesh`.
- `nav_planning_pose_ready`: unchanged readiness fields.
- `nav_planning_mesh_refreshed`: emitted only for the refreshed-mesh requirement.
- `nav_planning_recovery_failed`: include requirement and typed outcome.

Add `mission_motion_wait_started` when a mission begins waiting, with the initial state. `motion_settled` remains terminal-only; it must never contain `state=planning` or `state=driving`.

For visual retry, retain `mission_target_planning_retry` and add `inference_completed_before_readiness=true`. This provides a trace-level acceptance signal without logging images or sensor payloads.

## Testing

### Mission motion waiting

- A navigation fake that remains `.planning` must not settle or trigger another brain decision.
- Transition from `.planning` to `.driving` to `.arrived` must produce one decision and one navigation call.
- Cancellation during `.planning` must stop motion and return promptly.
- Session-generation change during `.planning` must stop motion and terminate through the session-change path.

### Planning context

- `stablePose` must ignore normal-limited-normal flicker and succeed only after three consecutive fresh normal observations.
- `stablePose` must not require a newer mesh revision.
- `refreshedTrustedMesh` must use one shared deadline and require a revision newer than the entry revision.
- Tracking timeout, mesh timeout, cancellation, session change, and transport failure must emit no navigation motor command.

### Visual target recovery

- A scripted slow detector must age the pre-inference frame beyond 0.5 seconds; after three fresh frames, assessment must reach A* rather than return `trackingUnstable`.
- The all-unreachable retry must call target detection before refreshed-map readiness and must not call detection afterward.
- A refreshed reachable assessment must navigate once without another brain decision.
- A still-unreachable refreshed assessment must fail once without a second planning-context recovery.

### Runtime acceptance

A new iPhone 15 Pro trace must show:

- no `motion_settled state=planning` or `motion_settled state=driving` events;
- one mission decision while navigation remains active;
- detector completion before the final planning-context readiness event;
- either `nav_drive_tick` progress or a typed A* rejection after readiness;
- no post-recovery batch where every candidate is rejected solely as `tracking_unstable` while tracking remains normal.

## Rejected Alternatives

### Move inference off the main actor

This is the stronger long-term architecture because AR frame ingestion could continue during inference. It requires async perception interfaces, detector isolation, pixel-buffer lifetime decisions, and broad mission call-site changes. It is intentionally deferred until the bounded orchestration fix is verified physically.

### Increase or remove the freshness limit

This would mask main-actor starvation by accepting older poses. It weakens the fail-closed invariant and does not prevent premature `.planning` settlement.

### Treat `trackingUnstable` as a reachable planner result

This would conflate readiness failure with map reachability and could permit motion from stale tracking. Readiness remains an explicit asynchronous prerequisite instead.
