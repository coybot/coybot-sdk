# Remembered depth visibility

## Problem

The depth safety gate requires at least 90% of the motion-volume sample points
to be visible in the newest LiDAR frame. With the WAVE ROVER mount (lens about
30 cm above the floor, 8.5 cm behind the axle midpoint), many of the low and
lateral samples just ahead of the bumper lie outside the camera's field of view.
Device runs reported 35 of 81 samples `outside_view`, with no missing or occluded
depth. The camera does not see that space from where it sits, even though it
usually saw it a moment earlier from a different viewpoint.

## Rule

A coverage sample that is **outside the live camera's view** may be answered by
a recent depth frame from the same continuously tracked AR session:

1. The live frame is authoritative wherever it can see. A live `covered`,
   `occluded` or `missing_depth` result is never replaced by memory.
2. For an `outside_view` sample, retained frames are searched newest first. The
   first frame that sees the point decides it. `covered` becomes
   `remembered_covered`, and `occluded` stays occluded. Frames that also could
   not see the point are skipped.
3. Obstacles seen in retained frames also take part in the collision check
   unless the live camera can currently see that spot. A remembered obstacle the
   camera can no longer see still stops the rover, and one the live camera now
   sees through is dropped.

The 90% requirement, stopping distances, cluster rules and per-section and
boundary coverage are unchanged. Remembered samples count as covered for the
fraction.

## Bounds

| Bound | Value | Where |
| --- | --- | --- |
| Maximum memory age, relative to the live depth capture | 2.0 s | `DepthVisibilityMemory.maximumAge` |
| Minimum spacing between retained frames | 0.1 s | `DepthVisibilityMemory.minimumSpacing` |
| Maximum retained frames | 20 | `DepthVisibilityMemory.maximumFrames` |

Memory is cleared when:

- The depth processor is invalidated: AR reset, pause, interruption, or a frame
  without raw depth.
- A depth frame arrives with tracking quality other than normal. Relocalization
  can move the world origin, so earlier frames are no longer comparable.

Frames whose own depth evaluation failed are not retained.

## Limits

- **No viewpoint, no memory.** Space that no frame in the last 2.0 s saw stays
  `outside_view`. A rover that starts stationary has never seen the ground
  directly under the bottom edge of its view. An in-place search turn sweeps the
  camera around the rover and can reveal ground beside and ahead of the bumper
  that the final heading cannot see, but only if the turn finished within 2.0 s.
- **Moving objects.** Something can enter a region the live camera cannot see
  after the last frame saw it empty. The 2.0 s age bound limits this exposure.
  It is not zero.
- Remembered occupancy is binned into the live 4 cm grid. Its position error is
  comparable to live binning.

## Diagnostics

The coverage summary and each point report remembered decisions:

- `support: remembered_covered` and `remembered_age_s` on the point.
- `depth_remembered_points`, `depth_history_frames` and
  `depth_history_oldest_age_s` on the assessment.
- `depth_remembered_hazard_cells` on the observation.

A remaining `outside_view` point with `depth_history_frames > 0` was not seen
by any retained frame.
