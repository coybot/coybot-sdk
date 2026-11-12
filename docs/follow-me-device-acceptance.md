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

## Spoken readiness in PhroverOperator

The operator app supplies `stationaryReadyAnnouncement` to `FollowMeCoordinator`.
After person validation and alignment, it confirms stop and waits for a fresh
post-stop matched observation. It queues the phrase **“I'm ready to follow”** on
the iPhone using on-device speech synthesis. The synthesizer manages its own audio
session so a preceding record-only microphone session does not suppress output.

This mode does not call the forward ready-motion controller, map preflight or
floor-probe recovery. It preserves the person-clearance gate, post-stop source
fence, and a new-frame departure baseline before waiting for the person to move.
Normal pursuit still requires its path, depth, pose, ownership and transport
safety checks. The cue is requested once per follow session; cancelled or failed
stops do not announce readiness. `follow_ready.announcement_requested` records
the request, not proof that the phone audibly played it.

While spoken readiness is confirmed stopped and waiting for its post-stop match
or distinct departure-baseline frame, pure frame-age expiry preserves that
phase and stop fence. It does not restart alignment or repeat stop confirmation.
The 500 ms observation limit, two-second perception-outage deadline and any
existing ten-second reacquisition deadline remain unchanged. Tracking, pose,
depth and ownership faults still take their normal stop paths; the exception
does not apply to an active alignment or legacy ready-motion request.

The November 9 pre-departure replay covers loss during alignment, reacquisition,
spoken readiness, then 350 ms-old observations delivered 550 ms apart with a
representative 250 ms stop delay. It failed with `Person lost` before the fix and
now reaches `waitingForMovement` without sending translation. This is a
synthetic timing-pattern replay through the real controller, not physical
acceptance. On device, verify one readiness cue, a fresh baseline, and eventual
departure after a brief perception interruption.

SDK clients that omit the announcement callback retain the legacy motion signal.
The probe-specific acceptance sections below apply to that legacy mode.

## Stationary reacquisition after delayed perception

The November 9 run found a person again near the end of reacquisition but did
not restore following before the original ten-second deadline. The coordinator
replay uses the recorded 392–425 ms observation ages and a representative
350 ms stop-confirmation delay with synthetic accepted person observations.
It reproduces `Person lost` before the fix; this is a timing-pattern replay, not
a replay of raw camera images or the physical rover.

After a detection stop has drained all motion, pure frame-age expiry can wait
without requesting another stop. This requires a retained stop fence in the
same AR generation, a valid source clock, no scan/alignment/movement task, and
no pending or failed stop. The original perception-outage and reacquisition
deadlines remain active. Tracking, pose and depth faults still request a stop.

A post-departure recovery can reuse that stop only after a fresh matching frame
clears the latest stop fence and its settling interval. It cannot use a
pre-stop, consumed, stale or conflicting observation to resume movement. Once
movement is launched, frame expiry requires a new stop again.

On device, briefly lose the person after departure, return into view before the
deadline, and verify that following resumes without repeated stationary stop
requests. Also verify that a sustained perception outage still terminates, and
that stale perception after resumed movement stops the rover. Physical
acceptance remains pending.

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

### Significant wrong-way response

After a complete, confirmed-stopped response, the turn executor compares signed
measured yaw travel with the actual commanded turn direction. Opposite travel
larger than the purpose's existing angular tolerance (5 degrees for alignment,
7 degrees for scan) terminates with `turn_response_opposite_direction`. Smaller
measurement reversals retain the existing response/progress checks. The executor
does not infer reversed wiring, flip command signs, extend its watchdog, or send
another corrective pulse after this rejection. Stop failure still takes priority
over the direction diagnostic.

The November 9 replay covers the captured wrong-way yaw sequence, both turn signs,
and wraparound. Device verification must establish why the physical response
disagreed with the request; this check prevents repeated wrong-way commands but
does not establish a corrected physical turn calibration.

### Reconciling lower mesh against a classified floor

The November 12 11:21 local attempt had a classified floor at -0.84197 m and
a fitted plane at -0.84226 m, with 195 inliers (175 required). Eighty-four lower
mesh cells vetoed the entire plane, causing fallback to a global estimate about
7.3 cm lower. That changed some near-rover samples from below the obstacle-height
cutoff into blocked cells. The screenshot alone cannot establish whether lower
geometry is stale or a real drop.

For independently classified, supported floor fits, lower geometry is now a
local hazard rather than a veto on the whole plane. A lower mesh cell can be
discarded only when two captured raw-depth frames, at least 50 ms apart and both
at most 250 ms old at planning capture, affirm the expected floor. Each must
support the center and four inset corners, cover at least three distinct pixels,
and reconstruct nearby world hits within 2 cm of the plane. Lower observations,
occlusion, missing/invalid depth, insufficient resolution, inconsistent frames,
and out-of-view cells stay blocked. This is capture-time map evidence, not an
extension of transport-time depth authorization.

Lower surfaces outside the supported plane bounds but within its local 1.2 m
radius can only be marked hazardous, never cleared by extrapolation. Raised
mesh geometry is retained even when live depth contradicts lower mesh in the
same cell. Mount-only fits retain their original multiple-level rejection.

Diagnostics distinguish `lower_mesh_refuted_cells`, `lower_mesh_confirmed_cells`,
and `lower_mesh_unresolved_cells`. If unresolved lower geometry alone blocks the
start, the message is "Map geometry conflicts near the rover" and asks for a
rescan. Known obstacles and depth-confirmed lower surfaces retain the blocked-map
message; failed stop confirmation still takes priority. Tests use controlled
geometry and depth, not the uncaptured raw pixels from the November 12 trial.

### Gradual forward requests and moving-target replans

Normal arrival now also tapers forward wheel requests ahead of the stopping
boundary. With the current pursuit settings, the cap decreases from 0.35 m/s
at 60 cm from the path endpoint to 0.04 m/s at 30 cm, before the 20 cm arrival
gate. It scales both wheels together and never raises an already lower request.
The previous acceleration ramp controls any subsequent speed increase. This is
anticipatory slowdown while perception and command authority remain valid, not
a sequence of extra movement commands after a stop/cancellation request. Actual
crawl and braking behavior need verification on the rover.

The November 12 10:55 trial confirms the acceleration ramp and moving retargets
were active. Its abrupt stops were depth-refresh stops, not normal goal arrival:
depth delivery delays preceded the first stop, tracking stayed normal, and the
operation eventually exhausted recovery attempts. This trace does not establish
that braking vibration caused perception latency. Depth/obstacle/ownership faults
still stop immediately; they cannot be softened by prolonging invalid motion.
`distance_to_goal_m` and the `approach_command_*` fields expose the normal taper.

The November 12 10:22–10:24 local trials got into pursuit. Logs show repeated
depth-refresh stops and restarts directly at the 0.35 m/s command cap. In the
last attempt the active route remained at revision 0 while the requested goal
advanced to revision 10; the rover then received in-place turn commands and
ended with insufficient measured progress.

Forward/curved pursuit requests now start with a maximum wheel magnitude of
0.10 m/s and increase by at most 0.25 m/s per second. Both wheels are scaled
together to preserve curvature, and the existing cruise cap still applies.
Only successful sends advance the ramp. Scheduling gaps contribute at most
200 ms of ramp time, and confirmed stops reset it. Lower requested speeds and
safety stops take effect immediately. Pure-turn breakaway commands retain their
existing policy. These are wheel-request limits, not physical acceleration or
gearbox calibration. The limited command is evaluated before admission and
checked again at transport.

Completed moving replans may now retarget their final leg to the newest goal,
using the existing captured-map segment verification and current-pose join.
Original plan deadlines, generation checks and send-revision fencing remain.
Unsafe tails still cannot replace a validated route. Diagnostics include
desired and final wheel commands plus `forward_acceleration_mps2`.

Regression tests reproduce abrupt startup and moving-replan starvation before
the changes. Device acceptance must verify smooth physical breakaway and route
updates while the person moves. Depth freshness and recovery exhaustion can
still require immediate stops; this change does not authorize blind continuous
motion or extend those recovery deadlines.

### Geometry-backed fallback for a disagreeing mount reference

Follow planning can now retry a rejected mount-referenced local floor fit using
independent ARKit-classified floor geometry. Horizontal plane detection is enabled
on LiDAR devices as well. Only horizontal anchors classified as floor and below
the camera contribute; arbitrary horizontal/table planes do not provide this
reference. Legacy non-LiDAR object grounding remains separate.

The classified-floor tracker requires three distinct geometry updates spanning
at least 150 ms, all within two seconds, agreeing in height within 2 cm at the
rover position. Each observed boundary must contain the rover center with a
25 cm inset. Competing overlapping floor levels, steep/invalid geometry, stale
tracking, removed or reclassified anchors, interruption, pause and reset revoke
the reference. Merely reading the same anchor on later camera frames does not
renew the evidence. Tracker storage is bounded to 32 anchors and three updates
per anchor, with at most 128 boundary vertices.

The independent reference is captured with the planning snapshot. The local
mesh must still pass the existing spatial support, 70% candidate consistency,
slope, reference agreement, residual and surrounding-support checks. Lower-level
conflicts are handled locally as described above. Raised geometry is still classified against the accepted plane;
the starting cell is never cleared. Without sufficiently stable classified floor
evidence, a rejected mount fit retains the existing global-floor fallback.

Diagnostics retain `mounted_floor_reference_y_m` and the original
`mounted_floor_rejection` when the classified fallback is attempted. They also
record `classified_floor_reference_y_m` and `local_floor_reference_source`
(`mounted_camera` or `classified_floor`). On device, verify that a new session
collects stable floor anchors and uses the fallback when the mount disagrees.
This does not guarantee that the November 12 blocking samples become traversable:
some remain over 10 cm above even the independently fitted floor.

### Depth control cost and November 11 follow-up

The November 11 11:24 local run recovered once, then exhausted the next 600 ms
depth window with roughly 59–60 ms validity remaining against a 60.7 ms reserve.
Cached acknowledgement reads were already only microseconds. The 11:23 run also
had large camera/inference publication delays, ending with an 817 ms-old frame.

The live AR depth provider now omits diagnostic-only full-volume coverage under
the existing Astral-compatible follow policy. Collision cells, remembered hazards,
stopping distance, exact-command transport checks and freshness limits are still
evaluated. Full-volume policy always evaluates coverage. Direct diagnostic
evaluation defaults to including it; omitted coverage and the counterfactual
full-volume result remain unknown/null, never reported as passing.

An empty-hazard, 20-history-frame simulator benchmark measured about 0.562 ms
per evaluation with shadow coverage versus 0.019 ms without it in Debug. This
isolates diagnostic work; it is not a device latency guarantee or proof that the
entire camera-staleness failure is resolved. Pipeline summaries now include
thermal state (0 nominal, 1 fair, 2 serious, 3 critical) and low-power mode.

The 11:22 map capture is a separate unresolved physical/calibration question:
the floor was near Y=0, the mount-derived reference was near Y=0.256, and a
sample at (-2.057022, 1.001897, -0.402796) lay about 8.7 cm from the rover's
planar position. The raw start was clear but obstacle inflation blocked it.
A regression preserves that rejection; the high sample is not reclassified
as floor or removed as presumed self geometry. Confirm the rigid mount, actual
lens height and nearby geometry before changing this classification.

Release validation was attempted but coordinator/recovery tests failed or hung
despite the targeted depth/AR/map/pursuit suites passing. The experimental
scheduling changes to those test helpers were reverted. Do not treat Release as
validated or install it based on the targeted pass. The full Debug suite passed
1,534 tests without retries; the next device build remains Debug.

### Small non-progressing search responses

The November 10 16:12 local search stopped after a complete response measured
-0.005944669 rad (-0.34 degrees), following 27 positive pulses. The executor
previously rejected every negative search response, even below the separate
significant-wrong-way threshold. A controller replay using the recorded yaw
endpoints and send/stop timing reproduces this failure.

Initial directed search now tolerates small complete non-progressing responses
within the original 2.5-second progress watchdog. Pulse budget and wheel speed
remain unchanged. Negative net travel is subtracted from sweep progress, never
clamped into positive credit. Retreat more than the existing scan tolerance
from the best directed sweep stops as wrong-way motion, including accumulated
small reversals. A non-progressing response whose sampled absolute travel exceeds
that tolerance is rejected rather than treated as noise. Missing evidence,
significant wrong-way responses and failed stop confirmation remain terminal.

The regression covers both directions, later measured arrival, persistent stall,
accumulated reversal and large out-and-back travel. It does not establish whether
the device's small response was mechanical settling or AR noise. Verify continued
search and eventual person acquisition on the device; no motor-sign reversal or
physical motor-on duration is inferred from HTTP acknowledgements.

### Local floor support amid raised clutter

The November 10 15:31 local attempt had a raw starting-cell cost of 255. Local
floor fitting reported 66 inliers among 83–84 mount-consistent candidate cells,
but required 206 because it counted all 294 local cells, including raised
clutter, in the consistency denominator. This could reject a supported floor
solely because more wall or furniture cells were present.

The fit now requires 70% agreement among mount-consistent candidates, still with
at least 24 cells. The surrounding-support, maximum slope, mount agreement,
residual, lower-level conflict and bounded-extrapolation checks are unchanged.
Excluded raised geometry remains input to obstacle classification; neither the
starting cell nor any presumed chassis region is cleared.

A controlled regression with a surrounding floor and dominant raised clutter
failed before this change and passes afterward. Actual 11.5, 14.5 and 18 cm
obstacles still block the starting cell. Competing candidate surfaces, one-sided
support, lower levels, steep surfaces and missing mount references remain rejected.
This is a synthetic regression, not an exact replay of the uncaptured device mesh.

Rejected fits now capture up to 512 reduced cells as
`local_floor_fit_cells_dx_y_dz_m` (relative X/Z, absolute world Y), with explicit
capture status, plus candidate plane height/slopes when available. These inputs
allow the next device rejection to be replayed. Physical verification must check
the configured 30 cm camera height and -8.5 cm forward offset; these values were
not recalibrated from the log. A candidate may still fail the independent mount,
geometry or lower-level checks after the consistency denominator is corrected.

### Joining planned routes through soft-cost cells

The November 10 14:57 local capture failed before the first pursuit command:
the planner produced a route, but the joining check treated the rover's cost-14
cell as blocked. Planning permits costs below 253; joining previously required
every touched cell to have zero cost. The minimized controller replay preserves
the recorded pose, seven waypoints and the single start-cell cost that triggers
the failure.

Joining an existing follow route now uses the planner's threshold of 253 while
retaining the 2 cm segment margin, map bounds and conservative cell-intersection
check. Costs 253–255 remain blocked. Other clearance checks, including a newly
retargeted final leg, retain the zero-soft-cost requirement. Diagnostics record
`join_blocking_cost` alongside the captured costmap. Tests cover soft and hard
start cells, nearby hard cells inside the margin, diagonal corner contact and
map edges. Physical acceptance still requires a follow trial on the mounted rover.

### Stopped depth recovery acknowledgement reads

The November 9 19:28 local pursuit attempt received clear post-stop depth,
but only 34–40 ms of its 250 ms validity remained at admission. The measured
admission reserve was 57.6 ms, so the original 600 ms recovery window expired.
The log establishes the freshness shortfall, not the exact source of the delay.

A deterministic pursuit replay combines the logged 149 ms publication age and
47.6 ms admission cost with a simulated 60 ms asynchronous acknowledgement-read
delay. Repeating that read on every stopped depth update reproduces the timeout.
While confirmed stopped, pursuit now reuses a real post-stop acknowledgement
read with its original watchdog expiry. It rereads when that evidence cannot
cover the admission reserve, and resumes ordinary reads after movement resumes.
Missing, future, nonfinite or expired acknowledgements cannot authorize pursuit.

The 250 ms raw-depth validity, 600 ms recovery deadline, three distinct post-stop
captures and transport-time validation remain required. The replay now resumes;
physical verification must establish whether acknowledgement scheduling is the
device bottleneck. `acknowledgement_read_s` and `acknowledgement_reused` on pursuit
drive/wait diagnostics distinguish that from intrinsically late depth delivery.

### Bounded undershoot finishing

The November 9 16:08 local alignment trace stopped roughly eight degrees short
after a valid 80 ms request. Its 182 ms observed stop overhead made the inverse
model negative, while the measured heading span was just too large to repeat
the previous request. A minimized replay retains the logged endpoints/extrema,
source evaluations and transport timing.

For alignment only, a complete improving undershoot can authorize one smaller
finishing probe when the remaining error is above five and at most ten degrees,
the inverse model is nonpositive, and the previous pulse does not fit the
remaining corridor. The current error must agree with the last settled source
within the existing tolerance; large out-and-back excursions, prior overshoot
corrections and incomplete evidence cannot grant this exception.

The request is bounded by both `previousBudget * remainingExcess / headingSpan`
and `remainingExcess / retainedResponseRate`, with no minimum pulse floor. It is
strictly smaller than the observed request. Once used, its ceiling cannot be
renewed or increased. Subsequent requests require a fresh complete response
with same-direction improvement; no progress, wrong-way movement or overshoot
outside the arrival tolerance terminates. Confirmed arrival still requires the
unchanged five-degree tolerance. Source, ownership, stop, depth, watchdog and
recovery-deadline checks remain active.

Telemetry labels the initial exception `bounded_undershoot_finishing` and
records `finishing_correction_budget_s` and `finishing_correction_ceiling_s`.
These are host request budgets, not a guarantee of physical motor-on time or
turn distance. Physical acceptance must verify the smaller pulse's actual
response and stopping behavior on the mounted rover.

### Confirmed overshoot correction

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

## Ready-probe decisions and completion races

The October 29 09:48:54 PDT run failed during floor-check recovery before pursuit.
AR tracking was reported normal and depth was fresh/clear. Seven probe sends
were acknowledged; the eighth proposal failed before sending. The old Boolean
authorization mapped every rejection to `trackingLost`, including motion bounds.
The rejecting controller pose was not logged, so that run cannot establish the
exact boundary that failed.

Probe authorization now distinguishes permitted motion, goal already reached,
invalid source, command/clock errors, expiry, and individual movement bounds.
A healthy pose at 3–5 cm forward travel within the existing lateral/heading
limits requests a stopped recheck rather than another pulse or a tracking failure.
No-op rechecks do not increment pulse counters, fabricate a stop/source fence,
renew deadlines, or create a return-path grant. Completion still requires a real
earlier forward pulse, fresh depth and source, and the original probe deadline.

The check is repeated after acknowledgement reads and at transport/source-observer
boundaries. A transport-denied completion can be retried as a no-op only when both
the receipt and captured attempt history prove zero HTTP attempts. Unknown,
entered, or contradictory outcomes retain their failure/stop handling.

`follow_ready.recovery_decision` records the boundary, authority ID, decision,
actual source frame/time/age/tracking state, original/current poses, along and
lateral displacement, heading change, and original limits/deadline. Proposed
probe events carry the same pose metrics. Geometry failures receive floor-check
messages; `trackingLost` is reserved for invalid pose-source evidence in this path.
Limits remain 3 cm probe goal, 5 cm maximum travel, 1 cm lateral deviation and
0.05 rad heading deviation. Physical rover acceptance remains pending.

### Stable stopped probe baseline and confirmation

Before the first probe, collect three distinct, normal-tracking pose captures
after entry's confirmed stop. Capture times must be at least 50 ms apart, within
a fixed 300 ms collection window. The position baseline is the component-wise
median; yaw uses wrapped offsets from the initial heading. Every sample must be
within 4 mm and 0.01 rad of that baseline. The median may not shift more than
5 mm or 0.01 rad from the initial pose. These are conservative stability gates,
not a claim about ARKit measurement accuracy. Failure reports “pose unstable for
floor check.” Baseline collection consumes the original startup/probe budgets.

The baseline is frozen once probing starts. A backward/lateral violation observed
in the stopped geometry loop collects three newer samples, at least 50 ms apart,
within 300 ms and the original stage deadline. All must meet the original movement
limits to continue. Persistent or mixed in/out readings fail. Tracking faults,
owner changes, failed stops and active-pulse/transport vetoes remain terminal.
No nonzero pulse occurs while collecting confirmation samples.

`baseline_accepted`, `baseline_unstable`, `stopped_pose_timeout` and
`pose_recheck_recovered` reasons in `follow_ready.path_recovery` record sample
counts, capture span, median pose and maximum variation. Geometry records include
both backward-limit and lateral-limit flags, even when only one is the primary
failure reason.

Physical calibration still requires a rigidly mounted phone: record stationary
baseline variation, then compare repeated controlled straight drives with the
configured body-forward axis and available wheel telemetry. Do not infer a
mounting correction from one failed probe or re-anchor after movement. The
November 4 trace (10.028 mm then 10.037 mm lateral displacement) establishes a
persistent AR-measured boundary violation, not whether the chassis really slid.

### Probe response and measured settling

The November 4 16:15 PST run accepted a stable baseline, then measured 11.66 mm
before pulse two and 74.08 mm afterward. Both motor and stop requests were
acknowledged. Those acknowledgements do not establish physical motor duration.

Probe and straight return wheel commands now use magnitude 0.12 rather than
0.25; turning and the main ready signal retain their existing settings. The
requested host pulse remains 20 ms. Firmware-timed motor execution is not assumed.

After each translation pulse's confirmed stop, the controller polls fresh
same-generation poses. Three distinct samples spaced at least 50 ms apart must
be within 2 mm and 0.005 rad of the newest pose, with at least 150 ms elapsed
since stop return. The sliding window is bounded to 600 ms and the original
stage deadline. Cached frames or ongoing movement cannot authorize another pulse.
Travel/heading violations remain hard failures during settling. Existing stopped
backward/lateral corroboration runs before subsequent motion.

After a settled forward pulse, retain its largest observed displacement from its
pre-send pose. Another pulse requires room for 1.5 times the largest response
plus 5 mm, inside the original 50 mm limit. These conservative allowances are
heuristics, not a proof of physical stopping distance. An unfit response fails
explicitly instead of issuing another pulse. The 30 mm completion goal, fixed
origin, stage deadlines and maximum pulse count remain unchanged.

`follow_ready.probe_pulse_timing` reports send entry/response, stop acknowledgement,
return time and pose capture time, explicitly labeling physical motor duration
unknown. `pulse_settle_sample`, `pulse_settled`, `pulse_response_fits` and
`pulse_response_unfit` reasons report settling and remaining-travel decisions.
Physical verification must check low-speed motion, delayed coast and stop response;
AR estimates alone cannot distinguish all chassis motion from pose-estimation error.
