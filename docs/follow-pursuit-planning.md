# Background follow-pursuit planning

## Device evidence

The October 28 run completed all ten ready pulses, with depth evaluations taking
2.1–2.9 ms, then failed on its first pursuit check at 21:18:26 PDT. Depth was
already 309 ms old. A perception frame spent 263 ms awaiting ingestion during
the handoff. Production pursuit still synchronously built its costmap and A*
path on the control actor, unlike the asynchronous ready-map path.

## Production planning path

`CostmapBuilder.planFollowAsync` copies ARKit mesh geometry into a value snapshot
on the main actor. The snapshot's costmap construction and A* search run in a
detached worker. ARKit/Metal buffers never cross into the worker. The small mesh
copy remains on the session actor; its time is logged separately.

`FollowPursuitDriver` owns one planning job and a newest-only goal revision. The
same path handles startup, periodic replans (one second after the last accepted
plan), and live-goal updates. A job has a fixed one-second deadline. Results from
replaced goals, owners or AR generations cannot authorize motion. Initial planning
and admission share a two-second outer limit, so goal churn cannot wait forever.

Only the exact current goal revision can commit a plan. Before first movement,
the source must be healthy/fresh, the path endpoint must match the requested goal,
and the current pose must connect to the captured map's path. The control loop
rechecks pose, central clearance, communications and depth after acknowledgement
reads. Transport admission checks the owner, target revision, pose and depth again;
the original depth/pose/acknowledgement expiry is not renewed in the sender queue.

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

Hard depth faults and hazards do not get this grace. After motion has begun,
stale depth remains a stop/failure; it never permits continuing on cached evidence.
The existing `astral_compatible`/`full_volume` policy selection is preserved.

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
