# Follow-Me Physical Device Acceptance

**Status:** Formal acceptance pending. User-run sessions on iPhone 15 Pro and iPhone 15 Pro Max supplied diagnostic logs, and the latest signed build has been installed on iPhone 15 Pro. The full checklist below has not been completed.

Automated simulator tests do not establish LiDAR projection accuracy or prove that the physical rover stops when commanded. The WAVE ROVER stop command is `T:1` with `L:0` and `R:0`; an HTTP acknowledgement alone is not evidence that the wheels stopped. Before using follow mode around people, test the current build on a supervised, LiDAR-capable iPhone mounted on the rover in a clear space:

1. With cloud connectivity disabled, press and hold the Talk microphone and say `follow me`. Verify rear-camera acquisition stops on a person or after at most 360°.
2. Verify the rover maintains approximately 1.5 m distance and does not drive backward when the person is too close.
3. Introduce a second person in the image centre. Verify the original track remains locked, or the rover pauses on ambiguity.
4. Briefly hide the person. Verify forward motion stops before reacquisition rotation; keep the person lost and verify follow mode stops within 10 seconds.
5. In each active state, press Stop Following; also test spoken `stop`. Verify motor stop acknowledgement and no delayed restart.
6. Leave Talk, background the app, and trigger a navigation failure. Verify the rover stops before another mode can drive.

Record the tested phone, rover, build revision, pass/fail per step, and redacted runtime-log evidence here after the supervised run. Do not attach camera images or raw transcripts.

## Pre-ready acquisition rotation policy

Initial search, alignment, and reacquisition use bounded in-place turns without
depth/obstacle checks until the ready signal has completed successfully with a
confirmed stop. A brief person detection or verification loss does not end this
acquisition policy. The diagnostic event is
`follow_depth.acquisition_rotation_bypassed`.

The exception requires an explicit per-request acquisition flag and equal/opposite
wheel speeds. Ready-signal translation and forward following still require depth
and clearance. After ready completion, subsequent alignment/reacquisition also
uses normal depth checks. Tracking, source freshness, ownership, stop confirmation,
burst bounds, and the recovery deadline continue to apply during acquisition.

For device acceptance, lose the person briefly before the ready signal and verify
reacquisition remains in-place; then repeat after ready completion and verify the
normal depth gate applies. Full physical acceptance of this policy remains pending.

## Alignment after a stop-latency spike

A complete, confirmed alignment overshoot may authorize one opposite correction
using the strictly smaller measured overshoot ceiling, even when the retained
HTTP/stop-overhead model yields no positive budget. This is an estimated probe,
not a claim that acknowledgement timing measures physical motor-on duration.
Further repeats require a complete, correctly directed small response whose
sampled heading span fits the remaining corridor and whose budget stays within
the overshoot ceiling. Missing evidence, zero progress or another overshoot
cannot authorize another latency-model exception. The 0.05-radian alignment
tolerance and original stop/source/watchdog checks remain in force.

The diagnostic selection reason is `bounded_overshoot_correction`. A simulator
replay covers the October 24 iPhone 15 Pro Max alignment failure. On-device
acceptance still needs to verify convergence and stop behavior; this change does
not eliminate network or firmware response latency.

## Ready-map disagreement recovery

Before the ready signal, a mapped start/goal/corridor obstruction can enter a
bounded live-depth check. Missing maps, stale maps and invalid pose shifts still
stop; they are not classified as obstacle disagreements.

* Live depth must be fresh and clear before a forward probe. The target is 3 cm,
  with a 5 cm measured-position limit, using 20 ms host pulses and confirmed stops.
  This uses the existing 0.25 wheel-command breakaway magnitude at a lower duty
  cycle; the command is not a calibrated physical speed guarantee.
* If the probe remains clear, its displacement counts toward the original 10 cm
  ready signal. The actual ready admission is checked again before continuing.
* A confirmed obstruction returns only along the newly probed straight segment,
  with a three-second return allowance, 1 cm lateral / 0.05 rad heading limits,
  and a 5 mm origin tolerance. No forward movement means no reverse allowance.
  This is retained-path evidence, not rear-camera visibility.
* After returning, a bounded pre-ready in-place search checks live forward depth
  for a clearer heading. Two distinct clear depth observations are required.
  The search is limited to 90 degrees; the whole recovery to 12 seconds and 32
  pulses. At most two recoveries are allowed per follow session.
* Unknown depth, tracking loss, cancellation, failed stops or geometry-limit
  violations terminate recovery. Finding a clear heading requests a new person
  observation/reacquisition; it never claims the ready signal has completed.

The UI shows `Checking floor` during recovery; `follow_ready.path_recovery` logs
`probe`, `backtrack`, `scan`, `clear`, and `clear_heading` decisions. Simulator
coverage includes clear/blocked/unknown paths, retained-path bounds, source reset,
cancelled ownership, failed stops and the coordinator's perception-outage handoff.
Physical probe distance, reverse stop behavior and clear-heading reacquisition
still require device acceptance.

### Bounded ready-motion depth coverage

Every bounded ready signal and its recovery probe check visibility through the stopping corridor (at least 30 cm),
without requiring the additional 35 cm cruise look-ahead. Collision detection
still evaluates the original hazard horizon, and 90% coverage is still required
inside the stopping corridor. This scope is restricted to equal, positive wheel
commands at or below the ready-signal magnitude. The ordinary map-clear ready path
uses the same policy as the recovery path; it does not require a preceding map
failure or probe. The permission expires with the ready operation; ordinary
following does not inherit it. Telemetry labels it
`bounded_ready_motion_stopping_corridor`.

`follow_depth.candidate` now records the visibility horizon, required stopping
distance, covered/required points, coverage fraction, outside-view points, missing
depth samples and occluded points. Rejection before the first probe is logged as
`ready_recovery_assessment`. This distinguishes missing distant look-ahead from
genuinely unseen near-chassis space, which remains blocked. The original device
log did not contain raw depth or coverage counts; synthetic projection tests
verify the policy correction, not that the device's near field is visible.
When rejected coverage includes points outside the camera view, the ready failure
message displays the measured visible percentage and required percentage instead
of implying that every such failure is a detected obstacle.

## Search freshness and dispatch accounting

Completed perception delivery retains only the newest frame plus the strongest
pending structural-health transition. Tracking/pose faults outrank depth-only
loss because initial acquisition may tolerate missing depth. Terminal AR events
close the subscription and cannot be overwritten by later frames. Selection
before inference preserves queued safety losses, and detections always retain
their original image/depth/pose pairing and capture timestamp.

Initial search requires 150 ms of freshness remaining within the existing 500 ms
observation limit, checked both before scheduling and immediately before motion
dispatch. A newer pending observation is processed first. Unsent deferrals neither
spend a controller attempt nor count as an interrupted turn. Waiting for an
admissible capture is limited to two seconds from the first deferral; new
near-expiry frames do not renew that deadline. Actual dispatched interruptions
remain subject to the existing eight-interruption bound.

`follow_search.admission_wait` and `follow_search.dispatch_deferred` distinguish
freshness deferrals from `follow_search.sweep_interrupted`. The once-per-second
`follow_perception_latency` record reports capture-to-inference-start time,
inference time, capture-to-publication time and coalesced completed-frame count.
Device acceptance should confirm that searching resumes on fresh observations,
does not spend interruption allowance without dispatch, and still stops on real
tracking/source loss.
