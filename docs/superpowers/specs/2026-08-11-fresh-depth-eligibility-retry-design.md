# Fresh Depth Eligibility Retry Design

## Context

The post-fix iPhone 15 Pro trace for the second `Go to refrigerator` attempt proved that a newer raw-depth snapshot is not necessarily fresh enough to authorize rotation. The controller stopped before rotation, observed `depthSnapshotVersion` advance, and then rejected the replacement observation as `stale_raw_depth` with a sample age of `0.369s`. The current safety limit remains `0.25s`.

## Goal

Recover a transient sequence of stale rotation-depth observations without weakening the existing fail-closed age limit. Rotation may resume only after the normal depth-safety guard accepts an observation. If no acceptable observation arrives within the bounded recovery window, the rover must remain stopped and the navigation operation must fail.

## Non-goals

- Do not increase the maximum raw-depth age.
- Do not authorize motion merely because the snapshot version changed.
- Do not add an unbounded retry or restart the entire mission.
- Do not change translation, obstacle-clearance, communication, tracking, or tipping safety rules.

## Selected Design

Replace the one-snapshot rotation retry with a bounded eligibility loop in `NavigationController`:

1. Stop the transport before waiting.
2. Record the starting depth-snapshot version and a monotonic deadline using the existing rotation recovery timeout.
3. Wait for a newer snapshot version.
4. Evaluate the candidate through the existing depth guard using the original rotation command.
5. If the guard allows the command, rerun the general rotation safety checks and return the safe command.
6. If the candidate is still `stale_raw_depth`, keep the transport stopped and wait for another newer version while time remains.
7. If the candidate is another depth stop, preserve the existing blind-volume arc fallback or fail with the existing safety reason.
8. If the deadline expires while observations remain stale, emit timeout telemetry and return no command.

The loop treats guard eligibility, not version advancement, as freshness. It never sends a motor command between failed observations.

## Telemetry

Keep `nav_scan_depth_retry_started` and add enough fields to distinguish:

- a newer but still stale observation;
- the observation that passed the guard;
- a deadline exhausted with the final depth state and sample age.

No high-frequency telemetry is required beyond one event per rejected replacement snapshot because the retry window is bounded and short.

## Tests

Add focused controller tests before changing production code:

1. Initial stale observation, first newer snapshot still stale, second newer snapshot accepted: exactly one rotation command is eventually sent.
2. Initial stale observation and every replacement remains stale through the deadline: no rotation command is sent and navigation fails closed.
3. Existing communication and tipping veto tests continue to prove that a depth recovery cannot bypass other safety layers.
4. Existing blind swept-volume recovery behavior remains unchanged.

Run the focused navigation safety suite, the full SDK suite, `git diff --check`, a signed device build, and another iPhone 15 Pro mission trace before claiming runtime recovery.

## Rejected Alternatives

- **Increase the `0.25s` age limit:** this would authorize older sensor data and violate the requested fail-closed policy.
- **Accept any version increase:** the physical trace disproves version advancement as a sufficient safety condition.
- **Retry the whole mission:** this adds nondeterminism and movement planning without repairing the safety boundary that rejected the rotation.
