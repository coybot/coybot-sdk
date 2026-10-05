# Low Camera Height Recalibration Design

## Context

The iPhone camera is rigidly mounted 30 cm above the floor. The current physical configuration declares a 55 cm camera height. Depth samples are transformed into rover-relative space using that calibration, so the 25 cm error can project floor returns into the rover collision-height band. The physical-device trace contains false near-clearance readings as low as 0.08–0.31 m while tracking remains normal.

## Goal

Recalibrate the fixed camera mount to 0.30 m so visible floor is excluded from collision geometry without weakening obstacle or depth safety.

## Non-goals

- Do not disable depth-based collision safety.
- Do not change the rover collision-height band.
- Do not change stopping margins, swept-volume coverage, connected-support thresholds, or obstacle-guard behavior.
- Do not add a runtime calibration UI or automatic camera-height estimation.
- Do not alter navigation, planning, tracking, or mission orchestration behavior.

## Design

Set `RoverConfig.cameraMountCalibration.cameraHeight` from `0.55` m to `0.30` m. Continue using the existing calibrated rover-origin transform and adaptive ground-offset correction. The corrected calibration places floor returns below `minimumCollisionHeight`, while obstacles extending into the rover collision band remain eligible hazards.

All existing fail-closed behavior remains active. Missing, stale, malformed, or insufficient depth remains unavailable rather than clear. Blind swept-volume checks, command-specific depth evaluation, obstacle stopping, communications checks, tipping checks, and watchdog behavior are unchanged.

## Testing

- Add a 30 cm mount regression in `DepthSafetyEvaluatorTests` proving a visible floor produces a clear forward observation.
- Add or adapt a 30 cm low-crossbar regression proving a supported obstacle cluster still produces caution or stop.
- Keep camera calibration validation accepting 0.30 m.
- Run focused depth-safety tests, the complete SDK test suite, and `git diff --check`.
- Build and install the signed app on the paired iPhone 15 Pro.

## Physical Acceptance

With the phone fixed 30 cm above the floor and the rover stationary:

- a clear floor view must not produce persistent false near-clearance readings;
- a real obstacle in the collision band must still reduce clearance or trigger a safety state;
- missing or blind depth must continue to fail closed.

Physical acceptance uses the runtime log from `bot.coy.phrover`; UI text alone is not sufficient evidence.
