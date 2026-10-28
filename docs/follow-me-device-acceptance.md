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
* Before the first recovery pulse, missing/stale depth can wait while stopped for
  up to 600 ms, polling the real provider every 50 ms. The 250 ms raw-depth age
  limit is unchanged. New stale captures do not renew the wait; fresh data at or
  after its deadline cannot authorize movement. The original three-second probe
  and twelve-second recovery deadlines include this waiting time. Cancellation,
  tracking/source reset, invalid calibration, malformed depth and blind coverage
  are not retried. Only the initial assessment gets this refresh allowance;
  subsequent pulse, return and search depth failures remain terminal.
* This pre-probe refresh also tolerates ordinary person-frame age expiry while
  the controller remains confirmed stopped, including during reacquisition.
  Fresh target association, heading, clearance and depth must all pass again
  before the first pulse. Waiting for perception shares the same fixed 600 ms
  deadline; it never starts a second window. The first sender also carries the
  observation/window expiry through queueing. Structural perception faults,
  invalid clocks and ownership changes remain terminal, and once any recovery
  pulse starts, perception loss retains the immediate stop policy.
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
The same event records `depth_wait_started`, `depth_wait_recovered`, or
`depth_wait_expired`, including elapsed time, poll count, reason and fixed deadlines.

### Bounded ready-motion depth coverage

> The 90% requirement below applies under `full_volume`. Follow-me now defaults
> to `astral_compatible`, which measures and logs coverage but does not require
> it; every other check here is unchanged. See `docs/follow-depth-policy.md`.

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

## Search calibration delivery ordering

A synchronous stop-boundary pose read can overtake older queued pose events.
Such delivery order is not usable on its own, but a complete bounded AR-ingress
archive can reconcile it when every collected control sample exactly matches its
original generation, sequence, source timestamp and yaw witness. Archived capture
times are retained; delayed delivery and duplicate reads do not invent fresh
frames or additional travelled angle.

Unhealthy/stale control reads, regressing collection clocks, conflicting sample
values, incomplete archives and invalid ingress chronology still reject calibration.
This covers the iPhone trace with control delivery `2506 → 2508 → 2507 → 2508`;
it does not disable the calibration evidence requirement.

## Shared five-degree alignment acceptance

Follow alignment now accepts an inclusive 5° residual in either direction. The
coordinator's default alignment and ready-admission tolerance comes from the same
follow turn profile. Search retains 7°; generic rotation keeps its own tolerance.
Acceptance still requires the confirmed-stop/settled-source fence, a fresh matched
person and valid clearance. It does not authorize a ready movement by heading alone.

This restores Astral's October 9 acceptance fix from stash commits `afa1fba` and
`bfaa867`, which was absent from the committed `6eb91e4` source used by the port.
The October 27 iPhone 15 Pro run (03:35:22Z on October 28, alignment operation 14)
settled at 4.9776° and subsequently read 4.7922° residual. Both satisfy 5°, but the
old 0.05-radian gate rejected them and requested an unresolvable additional pulse.
Regression coverage preserves the actual 187.411 ms response overhead, validates
both signs and the 5°/5.1° boundary, and requires the real adapter's stop/source
fence before accepting without another pulse.

## Perception and map timing

`follow_perception_delivery` reports capture-to-processing, publication-to-ingress
and ingress-to-processing durations, at most once per second. These timestamps
are diagnostics only and never replace the capture timestamp used for freshness.
`follow_perception_latency` also separates projection/publication cost from inference;
`nav_ready_map_snapshot` separates main-actor mesh capture from worker/delivery time.
Repeated identical `silent_search_safety` events are logged only on transitions;
every safety event is still processed. No mesh obstacle is erased based solely on
its proximity to the rover; a blocked start still requires live-depth recovery.

## Mounted camera calibration

The operator reports a rear camera lens approximately 30 cm above the floor and
7–10 cm behind the midpoint between the front and rear axles. The depth projection
uses `cameraHeight = 0.30 m`, `forwardOffset = -0.085 m` as a provisional midpoint.
Negative forward offset means behind the chassis center along the camera-aligned
rover-forward axis. Lateral offset and heading alignment remain unmeasured defaults.

For the configured 30 cm chassis length, the leading boundary is now 23.5 cm
ahead of the lens rather than 15 cm (22–25 cm over the reported offset range).
Synthetic projection tests cover the range and retain near-obstacle stopping.
They also show that ordinary-FOV coverage can remain below 90% after this correction;
the offset is not a visibility exemption or proof that this mount passes on-device.
Collision dimensions, the height band and the required coverage fraction are unchanged.

## Detailed visibility diagnostics and assembly sections

On a `blind_swept_volume` assessment, `follow_depth.candidate` and the retained
failure evidence now include:

- `depth_coverage_points`: section name, left/center/right leading boundary (or
  rotational boundary), travel distance, rover-relative lateral/height/forward
  position, projected pixel, camera depth, measured depth, support classification,
  and outside edges (`left`, `right`, `top`, `bottom`, or behind/too near the camera).
- `depth_projection`: the captured depth image dimensions, scaled intrinsics,
  effective camera offset/height, capture uptime, and rover axes/origin in world
  coordinates. `depth_projection_camera_from_world_column_major` preserves the
  matching 4×4 transform for replaying visibility with that same captured pose.
- `depth_geometry_sections` and `depth_geometry_lateral_margin_m`: the exact
  assembly model used, rather than an assumed camera-centered box.
- `depth_coverage_components` and `depth_limiting_component_fraction`: complete
  required/covered counts per section. Every component must independently meet
  90%; a large visible component cannot hide a small invisible component.
- `depth_coverage_boundaries`: counts for each sampled boundary. Explicit section
  models require each entering boundary to meet 90% too, so visible front samples
  cannot conceal a blind side. Legacy box admission keeps its existing policy.

Pixels are in **raw depth-image coordinates**, not the rotated/cropped UI preview.
Nonfinite or unavailable measurements are null. Point diagnostics retain at most
256 points, preferentially keeping uncovered points when full; truncation is
explicit. Aggregate and component decisions always use all required points.
Detailed points are emitted only for visibility failure, not every clear command.
No image or full depth map is attached to these events.

`RoverCollisionGeometry.sections` supports measured, contiguous height bands for
the chassis, mount and phone, each with its own footprint and center offset. The
same sections govern coverage and collision contact, including voxels straddling
height boundaries and operator-confirmed rotation's radial clearance. Curved
section motion checks entering side/rear edges using their local velocity, rather
than assuming every offset part only enters space along its front edge.
Missing/gapped/overlapping, oversized or invalid sections fail
validation. Reverse, freshness and confirmed-stop requirements remain unchanged.

The production profile still reports `conservative_bounding_box`: only lens height
and rearward offset have been supplied. Chassis height, upper-assembly footprint,
offsets and top height are needed before enabling narrower sections. Photos alone
do not establish those dimensions, and this diagnostic build does not claim that
the observed 57% visibility failure is resolved.
