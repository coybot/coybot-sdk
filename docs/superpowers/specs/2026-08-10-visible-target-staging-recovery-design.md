# Visible Target Staging Recovery

**Date:** 2026-08-10

## Problem

The offline object mission can detect and depth-project a requested object while the
cost-map planner cannot reach the object surface or any collinear stand-off point. The
2026-08-10 device trace demonstrated this for `Go to refrigerator`: speech completed,
the refrigerator matched at 91 percent, the rover link returned HTTP 200, and every
stand-off from 0.30 m through 1.20 m was unreachable. `MissionAgent` then submitted the
known-unreachable surface goal, failed immediately with `No path to goal`, and changed
the Talk UI from `Thinking` back to `Ready` before `On it` could render.

The same trace contained four exploration openings and measured only 1.11 m of forward
clearance for a target approximately 5.07 m away. A visible object is therefore not proof
that a direct path is available; the rover may need to move through an opening and
reacquire it from a new pose.

## Behavior

When a locked visual target has no reachable direct or stand-off goal:

1. Keep the original target query locked.
2. Refresh current exploration openings.
3. Assess each unexplored opening with the existing cost-map planner.
4. Retain only reachable openings that reduce distance to the projected target.
5. Navigate to the candidate that provides the greatest target-distance reduction, using
   path distance as the tie-breaker.
6. Wait for navigation to settle and for a newer normally tracked camera frame.
7. Reacquire and reproject the locked target, then retry normal stand-off planning.
8. Repeat for at most three distinct staging candidates.
9. If no progress-making candidate exists, or the bounded attempts are exhausted, stop
   and publish an explicit navigation failure.

The mission enters `acting` before staging motion begins, so the Talk UI displays
`On it...`. Emergency stop, depth safety, tracking validity, session generation, and rover
transport failure remain authoritative.

## Ownership

`MissionAgent` owns target-aware recovery because it knows the locked visual query and
mission budget. `NavigationController` remains responsible only for assessing and driving
to a supplied world goal. `ARSessionManager`, `CostmapBuilder`, and `FrontierFinder` remain
unchanged; they continue to provide target depth, occupancy, and opening candidates.

The recovery should be a small mission helper that returns one of: target reached,
staging required, cancelled, session changed, or exhausted. It must not duplicate doorway
crossing or room-topology state for a same-room command.

## Telemetry

Add structured events:

- `mission_target_staging_started`: target, projected goal, candidate count, attempt budget.
- `mission_target_staging_candidate`: candidate ID, reachability, path distance, target
  distance before and after, and rank.
- `mission_target_staging_selected`: candidate ID and staging goal.
- `mission_target_staging_completed`: resulting pose and fresh frame sequence.
- `mission_target_staging_exhausted`: structured reason and attempts used.

Existing navigation telemetry continues to record pose, goal, distance, wheel command,
request URL, and HTTP status.

## Failure Handling

- No reachable candidate that reduces target distance: fail without issuing the known
  unreachable object goal.
- Staging navigation failure: preserve the concrete motion failure and stop.
- Missing fresh tracked frame: stop rather than using stale detections.
- Target absent after staging: use the existing bounded slow visual scan before selecting
  another staging candidate.
- Mission cancellation or session-generation change: stop active motion immediately.
- Repeated candidate: reject it so recovery cannot loop in place.

## Testing

Use test-first development:

- A mission test reproduces the device failure: all collinear target goals are unreachable,
  one opening is reachable and reduces target distance, and the rover stages there before
  reacquiring and reaching the refrigerator.
- Reject a reachable opening that moves farther from the target.
- Prefer greater target progress, then shorter path distance.
- Never reuse a staging candidate.
- Stop after three staging attempts.
- Preserve cancellation, session-reset, motion-failure, target-lock, and explicit-return
  behavior.
- Run the focused tests, complete iOS SDK suite, signed device build, installation, and a
  physical `Go to refrigerator` trial with pulled logs.

## Non-Goals

- Driving directly through occupied or unknown cells.
- Replacing the cost-map planner.
- Persisting room maps across app launches.
- Treating a same-room staging move as a confirmed doorway crossing.
- Changing the 0.90 visual confidence threshold or 0.30 m preferred stop distance.
