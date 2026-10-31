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

Both policies keep the other gates wherever those gates already apply:

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

### Existing acquisition-turn exemption

Before the ready signal completes, the coordinator can authorize bounded pure
search/alignment turns through the existing `allowsAcquisitionRotationWithoutDepth`
path. Those turns bypass the depth provider under **either** policy: the depth
freshness, missing-depth and visible/remembered-depth obstacle rows above do not
apply to that exemption. They still retain pose, ownership, transport, bounded-turn
and stop checks. The exemption predates this policy and emits
`follow_depth.acquisition_rotation_bypassed`, rather than `follow_depth.candidate`.
The compatibility policy does not expand it or permit depth-free translation.

## Known gap

Without the visibility requirement, space the camera cannot see is not checked.
A low object beside the front wheels, or directly under the camera's lower field
of view, can be missed if it was never in view. That is the trade Astral made.
Remembered obstacles narrow the gap: something seen in the last 2.0 s still
stops the rover after it leaves the view.

## Diagnostics

Every `follow_depth.candidate` event records:

- `depth_visibility_policy`: `astral_compatible` or `full_volume`.
  This also records the selected policy when no depth snapshot is available;
  missing depth remains a rejection.
- `depth_full_volume_would_block`: whether the 90% requirement failed for that
  command, measured under either policy. It is null when the evaluator stopped
  before measuring coverage, for example on a visible obstacle.
- The existing coverage fields: fraction, outside-view, missing and occluded
  counts, and remembered points.

`depth_coverage_provenance` is `astral_compatible` when coverage was measured
but not enforced. Per-point detail is emitted only for `blind_swept_volume`
assessments, so it does not appear under `astral_compatible`. Aggregate counts
and per-section fractions do.

## Local integration verification (October 28, 2026)

- Imported the PDF's implementation from `7f56469`, including remembered-depth
  support from `81e46e3`.
- Replayed the captured 58% projection (47 of 81 points) with synthetic clear
  depth: `full_volume` rejects visibility, while `astral_compatible` permits the
  same geometry and reports `depth_full_volume_would_block = true`. This is a
  projection/policy regression, not a claim about unobserved physical obstacles.
- Hardened tracking-loss ingress so coalescing cannot retain pre-loss memory or
  republish a pre-loss projection; verified cached depth is revoked immediately.
- Missing-depth and pre-send/transport-expiry diagnostics retain the selected
  policy without weakening their rejection.
- Full SDK suite: 1,398 passed with no retries. After the final expiry-label
  correction, all 41 depth-recovery tests passed with no retries.
- Signed iPhone 15 Pro build and installation succeeded at 19:10 PDT.
  Physical rover acceptance of this mode remains pending.

## Depth-evaluation latency fix

The October 28 device run at 20:27:12 PDT passed compatibility-mode visibility
(53/81 points, including remembered support) but expired depth before the first
probe send. The sender read a 211.945 ms-old sample, spent about 57.004 ms in
evaluation/setup, and correctly rejected its resulting 268.949 ms age.

The evaluator now rejects distant live cells before detailed swept-footprint
intersection and rejects distant remembered cells in their original planar frame
before world transformation. The enclosing radius includes the maximum assembly
radius, the **full caution horizon**, and voxel padding; it is not the smaller
ready-visibility horizon. Historical frame filtering has an outward numerical
allowance and falls back to the original checks for non-orthonormal bases.

The original capture age, 250 ms freshness limit, stopping/caution distances,
remembered-hazard rules and final transport expiry gates remain authoritative.
This changes calculation cost, not the amount of stale evidence motion accepts.

Additional `follow_depth.candidate` diagnostics:

- `depth_evaluation_s`: elapsed evaluator time, excluding provider/logging work.
- `depth_collision_candidates`: cells reaching detailed collision checks.
- `depth_remembered_projections`: retained cells reaching world transformation.

These are diagnostic measurements only; unavailable counts are null. A synthetic
8,100-cell scene with 20 retained frames took 133–155 ms before pruning and about
5 ms afterward in the simulator. That is not a device latency guarantee. Tests
assert bounded work and unchanged nearby hazards rather than a flaky wall-clock
performance threshold. The next physical run must confirm the latency improvement.

Verification: 1,400 SDK simulator tests passed with no retries; correctness and
standards reviews found no blockers. The signed update was installed on iPhone
15 Pro at 20:56 PDT on October 28.
