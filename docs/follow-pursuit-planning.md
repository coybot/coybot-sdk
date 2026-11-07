# Background follow-pursuit planning

## Device evidence

The October 28 run completed all ten ready pulses, with depth evaluations taking
2.1–2.9 ms, then failed on its first pursuit check at 21:18:26 PDT. Depth was
already 309 ms old. A perception frame spent 263 ms awaiting ingestion during
the handoff. Production pursuit still synchronously built its costmap and A*
path on the control actor, unlike the asynchronous ready-map path.

## Production planning path

`CostmapBuilder.planFollowAsync` copies local vertex bytes and transforms on the
main actor. Coordinate transformation, costmap construction and A* search run in
a detached worker requested at utility priority. ARKit/Metal buffers never cross
into the worker. Only the byte copy remains on the session actor; its time is
logged separately. Depth processing requests user-initiated priority. These
priorities are scheduling hints, not real-time guarantees.

`FollowPursuitDriver` owns one planning job and a newest-only goal revision. The
same path handles startup, periodic replans (one second after the last accepted
plan), and live-goal updates. A job has a fixed one-second deadline. Results from
replaced goals, owners or AR generations cannot authorize motion. Initial planning
and admission share a two-second outer limit, so goal churn cannot wait forever.

Planning starts, including startup and stopped refresh, are spaced at least 500 ms apart. Goal
updates still coalesce to the newest revision; safety checks and driving continue
while a replan is throttled or pending.
Periodic rebuilding is suppressed during stopped refresh when the existing plan
already matches the current goal. Goal changes still require a matching plan;
the original startup and worker deadlines are not extended.

Only the exact current goal revision can commit a plan. Before first movement,
the source must be healthy/fresh, the path endpoint must match the requested goal,
and the current pose must connect to the captured map's path. The control loop
rechecks pose, central clearance, communications and depth after acknowledgement
reads. Transport admission checks the owner, target revision, pose and depth again;
the original depth/pose/acknowledgement expiry is not renewed in the sender queue.

Before the first send, a completed `start_blocked` rejection can still terminate
startup after a goal update if its captured map shows the rover in the same
blocked cell and the AR generation is unchanged. The starting-cell obstruction
does not depend on the person's updated goal. Its cause and map diagnostics are
retained instead of degrading to a generic startup timeout. This exception never
commits an obsolete route, reuses goal-specific rejections, or applies after a send.
Local-floor fits report reference-consistent, seed, inlier and required support
counts for the assessment stages reached; counts not yet computed are omitted.
This lets a blocked start be investigated without relaxing geometry checks.

While replanning an already moving drive, the last committed path remains active
under the normal safety checks. New plans are installed atomically; worker waits
do not suspend the control loop. If the old goal is reached while a replacement
is pending, the rover confirms a stop rather than continuing its previous command.
The in-loop stop does not join the pursuit task itself. A moving replan connects
to a reachable forward portion of its result, rather than requiring a straight
line back to its obsolete start point. Planning failure/timeout stops the operation.
Live goal changes do not reset the
progress watchdog: progress measures actual approach between observed positions.

For the asynchronous production planner, `updateFollowGoal` returning true means
the new goal was queued, not that planning already succeeded. Synchronous injected
planning closures retain their legacy behavior for existing adapters/tests;
production follow-me always uses the snapshot worker. Ordinary navigation and
Silent Search remain on their existing controller paths.

## Stopped depth refresh

Entry to pursuit follows a confirmed stop. Before any movement command has been
sent, transient missing/stale depth may wait for up to 600 ms. Every retry reads
the real provider, revalidates current authority and source, and uses the same
deadline. Fresh depth at or after that deadline cannot authorize a command.
The original 250 ms raw-depth age limit remains in force.

Production pursuit subscribes to depth publication for the lifetime of its driver.
After a send it wakes on a new publication, or at the earlier of 100 ms and the
original command validity deadline. A version check prevents losing publications
that arrive during sending. Cancellation/owner inhibition wakes any pending wait;
observer and timer cleanup cannot wake a replacement operation. Injected providers
without publication support retain timed polling. Missing publication never
extends the original safety deadline.

Hard depth faults and hazards do not get this grace. After motion has begun,
missing or stale raw depth requests an immediate confirmed stop before entering
the bounded refresh window; it never permits continuing on cached evidence.
The existing `astral_compatible`/`full_volume` policy selection is preserved.

An expired command authorization is distinct from a stale depth observation.
When an expired command is proven unsent (zero attempts in both the receipt and
transport capture), pursuit confirms a new stop before re-observing. It requires
a newer same-generation frame captured after that stop, rechecks all normal
guards, and computes a new command with new authority. The old command's expiry
is never extended. The fixed 600 ms refresh window starts at the first confirmed
stop in that recovery, giving post-stop captures the full window to stabilize.
Repeated expiry before a successful send cannot renew this deadline, including
time spent confirming another stop. At most two recoveries are allowed before sustained
healthy delivery replenishes the allowance. Uncertain sends, failed stops, source faults and hazards still
terminate. The original startup and progress deadlines remain in force.

Raw-depth and command-expiry recovery share the two-recovery limit. Before a
replacement send, both pose and depth must be captured after the confirmed stop.
Recovery also requires at least three distinct qualifying depth captures spanning
60 ms, within the same fixed 600 ms window. Missing/stale depth or inadequate
margin resets that qualification streak. Polling the same cached capture does
not count. This avoids declaring stability from just one usable frame.
The sending reserve is `max(20 ms, max(observed admission time, current depth
assessment time) + 10 ms)`, measured after evaluation. Production admission time
runs from sender entry to the recorded HTTP attempt, excluding response/ACK
latency. Only successful sends teach this budget; an unsent scheduling stall
does not permanently demand a reserve larger than the entire depth lifetime.
The 20 ms cold-start reserve and 10 ms allowance are explicit heuristics, not
latency guarantees. Otherwise the rover stays stopped and waits within the
same 600 ms window. Transport admission
still enforces the original expiry. `depth_wait_recovered` is emitted only after
a successful send, rather than before the transport attempt.

Depth ingestion precomputes the intrinsic ray factors per row/column and avoids
temporary per-cluster extrema arrays. All original depth pixels, Float world
transforms, height thresholds and collision cells are preserved. The device
timing events remain necessary to measure the benefit under real sensor load.

Terminal causes distinguish a fresh-depth/margin timeout, command-authorization
expiry, and exhausted recovery attempts. Stop failure retains highest priority.

The allowance is not replenished by a single successful recovery command. It
requires at least two seconds and ten successful nonzero sends with distinct
advancing depth captures, no send gap over 300 ms, and at least 20 ms beyond
the measured admission reserve at send entry. Each send must complete before
the original depth expiry. Any failed freshness check or recovery breaks the
streak. `recovery_budget_replenished` logs the healthy duration, send count and
previous recovery count. Rapid repeated failures still exhaust the budget.

In caution, the driver accepts ObstacleGuard's proportional wheel-speed reduction
without repeating the full depth evaluation. The production transport MUST still
evaluate fresh depth for the exact reduced command before HTTP admission, within
the original deadline; a new obstacle or further required speed reduction rejects
the send. This removes one evaluation, not the last safety boundary.

Historical-hazard evaluation skips live-camera projection only for cells already
retained as hazards in the current evaluation. Visible historical points are never
cached as evidence that a neighboring hidden point is safe. Hazard union semantics
are preserved. The remembered-projection metric now counts actual live sight
queries, excluding earlier world-coordinate transforms and duplicate retained cells.

Retargeting after an unsuccessful send is allowed without another stop only when
the receipt and captured attempt history both prove zero HTTP attempts. An
attempted or uncertain send failure terminates and confirms a stop, even if the
goal changed. Every nonzero command marks stop evidence pending and retains its
receipt; cancellation cannot reuse the initial admission stop as proof that a
later moving command stopped. Expired episode authority still leaves the current
owner responsible for its cleanup stop.

## Diagnostics and verification

- `nav_follow_plan_snapshot`: mesh-copy and worker/delivery durations, vertex count,
  and planning outcome.
- `follow_pursuit.planning_started`, `planning_completed`, `planning_superseded`:
  operation-correlated planning lifecycle and revision.
- `follow_pursuit.depth_wait_started`, `depth_wait_recovered`: bounded refresh.
- `follow_pursuit.authorization_expired`: original command deadline, evaluation
  time, zero-attempt evidence and fixed recovery deadline.
- `follow_pursuit.depth_refresh_stop_confirmed`: expiry recovery has confirmed stop.
- `follow_pursuit.raw_depth_refresh_started`, `freshness_margin_wait`: raw-depth
  recovery and the available/required sending margin.
- `follow_pursuit.depth_stability_wait`: qualifying capture count, span and fixed
  refresh deadline before resuming.
- `nav_follow_plan_rejected`: explicit map/planner rejection and requested geometry;
  failures remain terminal rather than bypassing mapped obstacles.

Blocked plans retain `PathPlanningFailure` through the driver and controller to
the UI. Start blocked, goal blocked, disconnected path and outside-map failures
have distinct follow messages. Stop failure continues to override all of them.
The rejection log includes the raw/inflated start cost, cell indices, resolution,
inflation radius, obstacle height bands, global floor estimate, local height-sample
count and local fifth-percentile height. Up to 16 representative blocking seed
points are recorded in world XYZ metres, plus the existing nearby cell samples.
Blocking seeds follow Costmap's actual hard axial-neighbor inflation ring, not an
assumption that the entire inflation radius is lethal.

The November 5 trace showed a global floor near -1.213 m, local height percentile
near -1.116 m, and blocking points near -1.084 m. A global percentile alone can
therefore classify a locally supported floor as occupied. Those few logged points
do not prove the real surface is floor rather than self-mesh or an obstacle.

Follow planning now optionally fits a local plane, corroborated by mounting
height. The mounted reference requires normal tracking, current pose/depth within
250 ms, and at least three camera heights spaced 50 ms apart over 150 ms from
the last 500 ms, with at most 30 mm variation. It uses the configured camera
height (currently 0.30 m), not the global mesh percentile.

Plane fitting uses the lowest point per 10 cm spatial cell in the 0.25–1.2 m
annulus around the start. It requires at least 24 inlier cells, 70% spatial-cell
support, support in all four quadrants extending at least 25 cm each way, slope
at most 0.08, RMS residual at most 10 mm, and agreement within 40 mm of the mount
reference. Three or more cells on a conflicting lower level reject the plane.
These conservative thresholds must be validated on the physical mount.

The plane is used only inside its measured support bounds and 1.2 m radius;
elsewhere the global estimate remains. Raised obstacles retain the existing
height-band/inflation checks. Vertices more than 50 mm below a validated plane
are also blocked as possible depressions. Missing or ambiguous evidence falls
back to the original map; no start-cell clearing or speculative self-mesh masking
is performed. Diagnostics report the global and selected floor, mount reference,
support, residual, slope, bounds and rejection reason. Spoken readiness remains
the operator app's confirmation mode; no blocked-start escape motion is introduced.
- `depth_pipeline_timing` (at most once per second): source timestamp,
  capture-to-submit, submit-to-worker, projection, worker-to-publication and total
  capture-to-publication times. Worker-to-publication includes actor scheduling,
  memory attachment and delivery. These measurements locate publication delays;
  the existing device trace alone does not identify the upstream bottleneck.
- `follow_pursuit.drive_tick`: committed/requested goal, revisions, pending worker,
  wheels and original depth expiry, at most twice per second.

Tests cover off-actor work on an immutable snapshot, the recorded 309 ms startup
depth failure recovering while stopped, fixed deadlines, hard faults, source and
owner replacement, superseded goals, periodic planning with live safety checks,
retargeting progress, and the actual navigation facade/transport admission seam.
Physical follow-me acceptance remains pending after simulator/build verification.

### Verification results

- Full SDK simulator suite: **1,421 passed**, zero failures/skips/retries.
- Correctness review findings were fixed with controller-level regressions for
  in-loop stop completion, uncertain/zero-attempt sends, forward path joining,
  episode-expiry cleanup, and immutable stop/command receipt evidence.
- Signed generic-iOS build succeeded. Installed on **iPhone 15 Pro at 22:34 PDT
  on October 28, 2026**. A physical ready-to-follow run remains the acceptance check.
