# Follow-me depth policy

`RoverConfig.followDepthPolicy` selects how much depth evidence a follow-me
motion needs. It applies only to depth reads made by `FollowNavigationController`.
Ordinary navigation and Silent Search always use `fullVolume`.

| Policy | Visibility requirement | Default |
| --- | --- | --- |
| `astral_compatible` | None, as in Astral | Yes, for the current WAVE ROVER mount |
| `full_volume` | At least 90% of motion-volume samples visible, live or remembered | No |

## Why the default is `astral_compatible`

With the conservative collision box (30 × 25 cm footprint, 8 cm lateral margin,
4–50 cm height band) and a lens about 30 cm high and 8.5 cm behind the axle
midpoint, the camera cannot see enough of the near motion volume to pass 90%.
Device runs reported 35 of 81 samples `outside_view`, with no missing or
occluded depth. Remembered visibility (`docs/remembered-depth-visibility.md`)
helps once the rover has moved, but not from a stationary start.

Switch back to `full_volume` once measured height sections make it passable on
the rover. The diagnostics below show whether it would.

## What each policy checks

Both policies keep every other gate:

| Check | Astral (`a326011`) | `astral_compatible` | `full_volume` |
| --- | --- | --- | --- |
| Central-region clearance ≥ 0.45 m (`ObstacleGuard`, 10th percentile, middle half of width × middle third of height) on the ready-signal drive, pursuit and recovery arcs | Yes | Yes, same code | Yes, same code |
| Same central clearance on in-place turns and the ≤ 3 cm ready-path recovery probe pulses | Turns: no. Probe: not in Astral | No | No |
| Ready signal fails on unmeasured central clearance | Yes | Yes | Yes |
| Depth older than 0.25 s stops motion | No | Yes | Yes |
| Missing or malformed raw depth stops motion | No (pursuit reads it as infinite clearance) | Yes | Yes |
| Visible obstacle in the swept path stops motion (30 cm minimum, speed-dependent) | No | Yes | Yes |
| Caution speed limit (0.12) near a visible obstacle | No | Yes | Yes |
| Remembered obstacle the camera can no longer see stops motion | No | Yes | Yes |
| Visible obstacle inside the turning radius stops a turn | No (turns skip depth) | Yes | Yes |
| Depth gate never authorizes reverse (no rear depth). The ready-path recovery backtrack retraces only its own ≤ 5 cm probe segment, outside this gate | Not used | Yes | Yes |
| 90% motion-volume visibility | No | **No** | Yes |

The `ObstacleGuard` central-clearance gate already ran in Coybot's follow
controller before this policy existed. It is the same sampler as Astral's
`ARSessionManager.forwardClearance(fromDepthMap:)`, on smoothed depth, with the
same 0.45 m stop. `astral_compatible` changes only the visibility row.

## Known gap

Without the visibility requirement, space the camera cannot see is not checked.
A low object beside the front wheels, or directly under the camera's lower field
of view, can be missed if it was never in view. That is the trade Astral made.
Remembered obstacles narrow the gap: something seen in the last 2.0 s still
stops the rover after it leaves the view.

## Diagnostics

Every `follow_depth.candidate` event records:

- `depth_visibility_policy`: `astral_compatible` or `full_volume`.
- `depth_full_volume_would_block`: whether the 90% requirement failed for that
  command, measured under either policy. It is null when the evaluator stopped
  before measuring coverage, for example on a visible obstacle.
- The existing coverage fields: fraction, outside-view, missing and occluded
  counts, and remembered points.

`depth_coverage_provenance` is `astral_compatible` when coverage was measured
but not enforced. Per-point detail is emitted only for `blind_swept_volume`
assessments, so it does not appear under `astral_compatible`. Aggregate counts
and per-section fractions do.
