# Central-region depth clearance vs. motion-volume visibility

> Historical finding from the full-volume device runs. Follow-me now defaults to
> the named `astral_compatible` policy described in [Follow-me depth policy](follow-depth-policy.md):
> coverage is measured but its fraction is not enforced for follow-owned depth
> reads. Ordinary navigation and Silent Search retain `full_volume`. The
> comparison and device measurements below describe the earlier full-volume behavior.

## Summary

The two approaches answer different questions:

- **Central-region depth clearance:** How close is something in front of the camera?
- **Motion-volume visibility:** Can the camera verify enough of the space the rover needs to move through?

This distinction explains how the inspected Astral implementation could allow
movement with the same mounting arrangement while Coybot reports insufficient
visibility. It does not establish that all Astral versions use the same policy.

## 1. Central-region depth clearance: the inspected Astral implementation

Astral samples a rectangular region near the center of the LiDAR depth image:

- The middle half of the image width.
- The middle third of the image height.
- Valid, finite depth readings greater than 5 cm.

It sorts those readings and uses the **10th percentile** as the forward-clearance
distance. Navigation compares that distance against its stopping thresholds.

For example, if the central region mostly sees a wall two metres away, the result
can be approximately two metres of clearance. Smaller foreground objects can
affect the result if they occupy enough of the sampled region.

### Strength

The check is simple and relatively tolerant of mounting position.

### Limitation

It does not establish that the rover's entire path is visible. A low obstacle
near a wheel, an obstacle to one side, or an object immediately below the camera
may lie outside the sampled region.

**Source inspected:** Astral SDK `swift/Sources/PhroverKit/Perception/ARSessionManager.swift`,
`forwardClearance(fromDepthMap:)`, in local `main` at `a326011`.

## 2. Motion-volume visibility: Coybot

Coybot uses a collision model and the proposed wheel command to generate sample
points along the boundary of the space being checked. It projects those points
into the captured depth image using camera calibration and pose.

For each point, it checks:

1. Whether the point projects inside the camera's field of view and is sufficiently
   far in front of the camera.
2. Whether a valid depth reading exists at the projected pixel.
3. Whether the depth reading reaches at least as far as the point, within the
   evaluator's tolerance, or a closer surface occludes it.

The visibility gate requires **at least 90% of the sample checks to pass**.
Separately, the collision check looks for occupied depth cells that intersect the
rover's motion and applies stopping-distance rules.

**The 90% is a fraction of sampled checks.** It is not a measured percentage of
every cubic centimetre of the volume, and it is not a probability that movement
is safe.

### Current ready-motion policy

- Bounded straight ready motion checks at least a **30 cm stopping horizon**.
- It omits the additional 35 cm cruise look-ahead used for ordinary forward motion.
- Depth must still meet the **250 ms maximum raw-depth age** requirement.
- A short commanded movement does not automatically exempt the stopping corridor
  from visibility checks.

### Section-based geometry support

Coybot also supports separate, contiguous height sections for the chassis, mount,
and phone. When configured, each section and its entering boundary groups must
independently meet the coverage requirement. A large visible section cannot hide
a smaller invisible section through averaging.

The production profile still uses the conservative bounding box pending further
physical measurements. Detailed point diagnostics are available to identify the
remaining invisible regions. Diagnostic truncation does not truncate the points
used for safety decisions.

**Sources:**

- `swift/Sources/PhroverKit/Perception/DepthSafetyEvaluator.swift`
- `swift/Sources/PhroverKit/Perception/DepthSafetyModels.swift`
- `swift/Sources/PhroverKit/Config/RoverConfig.swift`

## 3. Behavior comparison

| Camera observation or condition | Central-region clearance | Motion-volume visibility |
| --- | --- | --- |
| Open room ahead, but near-wheel space is not visible | Can report generous clearance | Can reject because required nearby samples are outside the image |
| Wall close ahead | Reports short clearance if the wall occupies the sampled region | Can identify an intersecting obstacle and stop |
| Camera position is modeled incorrectly | May still report a plausible central distance | Can project required points into incorrect image locations |
| Collision model is larger than the actual assembly | Does not explicitly model that assembly | Can demand visibility of space the rover would never occupy |

## 4. Evidence from the physical rover

The operator reported a camera lens approximately **30 cm above the floor** and
**7–10 cm behind the midpoint between the front and rear axles**. Coybot was
updated to use a provisional **−8.5 cm forward offset**, the midpoint of that range.

### Device observations

| Check | Earlier run | Latest analyzed run |
| --- | ---: | ---: |
| Failure time, PDT | October 27, 2026, 21:09:49 | October 27, 2026, 22:22:07 |
| Visible clearance samples | 27 / 81 | 46 / 81 |
| Coverage | 33.3% | 56.8%, displayed as 57% |
| Outside camera view | 54 | 35 |
| Missing depth samples | 0 | 0 |
| Occluded samples | 0 | 0 |
| Depth age | 218 ms | 227 ms |
| Checking horizon | 30 cm | 30 cm |

The improvement is consistent with correcting the rearward camera offset, but
the runs used different captures and are not a controlled comparison.

In the latest run:

- Search and alignment completed.
- Map preflight reported `start_blocked`: the raw starting cell was clear, but
  obstacle inflation marked it blocked.
- The controller entered ready-path recovery.
- Fresh depth was available, but **35 of 81 clearance samples projected outside
  the image**.
- The visibility gate rejected the motion before a recovery probe pulse.
- Stop acknowledgement was confirmed.

At least **73 of 81 samples** must pass to meet 90%. The latest assessment was
therefore **27 passing samples short**.

The terminal reason was `blind_swept_volume`, not missing depth or the earlier
alignment-resolution error. The depth assessment did not report an intersecting
obstacle; that does not prove the unseen space was clear.

### Remaining uncertainty

The aggregate log used for this comparison does not identify which heights,
sides, or travel positions account for the 35 failures. It cannot distinguish
precisely among:

- Genuine camera blind areas.
- An overly broad collision envelope.
- Remaining mount-offset or orientation error.

The current box assumes a 30 × 25 cm footprint, an 8 cm lateral margin per side,
and a collision-height range of 4–50 cm. The photos suggest the raised mount and
phone are narrower than the chassis, but photographs alone do not establish the
dimensions needed to activate a narrower section model.

## 5. Conclusion and next steps

Motion-volume checking provides stronger geometric evidence **when its camera
calibration and collision model accurately describe the rover**. Incorrect or
oversized geometry can make it unnecessarily restrictive.

Central-region clearance is easier to satisfy, but a clear central view is weaker
evidence about the whole path. Switching back could remove the warning without
resolving the blind areas.

Recommended next steps:

1. Capture another failed assessment using the installed per-point diagnostics.
2. Identify the failing heights, sides, distances, and projected image edges.
3. Measure chassis height, upper mount/phone width and front-to-back extent, and
   overall assembly height.
4. Configure and validate the section-based collision model against those measurements.
5. Recheck physical visibility and obstacle stopping on the rover.

The rearward-offset correction is implemented. The 57% visibility failure is
**not yet confirmed resolved**.

## References

- [Follow-me device acceptance](follow-me-device-acceptance.md)
- [Astral stash comparison](superpowers/plans/2026-10-27-astral-stash-comparison.md)
- SDK implementation snapshot: `a656a60808c86db3f0dc1801e3904f8854e35ad7`.
- Latest analyzed local log:
  `/var/folders/g0/7vt60mh93ygg84tph_tyb6gw0000gn/T/opencode/coybot-ready-visibility-57-latest/phrover-runtime.log`.
- Earlier local log:
  `/var/folders/g0/7vt60mh93ygg84tph_tyb6gw0000gn/T/opencode/coybot-ready-visibility-latest/phrover-runtime.log`.

The log paths are local evidence locations, not files included in this repository.
