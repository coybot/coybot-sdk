# Silent Search Calibration Camera Feedback Design

## Summary

Add a live camera preview and staged QR-calibration feedback to the Silent Search calibration screen. The operator must be able to distinguish QR decoding, LiDAR corner grounding, and accepted calibration samples without consulting device logs.

This addresses a device-observed failure where calibration remained at `0 of 3`. Runtime telemetry showed normal AR tracking but no calibration events, and replaying the representative camera image through both Apple Vision and Core Image produced zero QR detections. The current UI cannot reveal whether scanning or grounding failed.

## Goals

- Show the AR session camera feed during the calibration phase.
- Outline the currently decoded QR marker and display its marker ID.
- Present separate, latched states for QR decoding, four-corner LiDAR grounding, and accepted calibration samples.
- Surface actionable failures that are currently discarded silently.
- Preserve the existing single AR session and automatic transition after three accepted samples.

## Non-goals

- Showing the camera preview during setup or other Silent Search phases.
- Starting a second `AVCaptureSession`.
- Recording or persisting camera frames.
- Changing marker dimensions, calibration tolerances, sample count, or safety gates.
- Replacing Apple Vision/Core Image QR detection in this change.

## Architecture

### AR session and preview

`ARSessionManager` remains the sole camera and LiDAR owner. During calibration, `LiveSilentSearchViewModel` subscribes to `ARSessionManager.snapshots()` and converts the newest camera frame into a preview image. Preview conversion belongs to the app layer so PhroverKit does not expose UIKit types.

The preview pipeline will:

- render at no more than 10 FPS, with at least 100 milliseconds between preview conversions;
- reuse one `CIContext`;
- buffer only the newest frame;
- cancel its task and release the latest image when calibration ends.

Full-resolution snapshots continue through the existing calibration scanner independently of preview throttling.

### Calibration visual feedback

Introduce a UI-independent calibration feedback value containing:

- AR frame ID and monotonic timestamp;
- decoded marker ID;
- normalized, oriented QR corners;
- whether all four QR corners were grounded by LiDAR;
- accepted sample count.

`ARSharedMissionFrameCalibrator` emits feedback from the same frame used for scanning and grounding. `SilentSearchCoordinator` stores the latest visual feedback and three attempt-local latched stages:

1. QR decoded;
2. LiDAR corners grounded;
3. at least one sample accepted.

The coordinator resets all feedback when a calibration attempt starts, aborts, fails, or completes. Image data never enters coordinator state.

### UI components

Add a focused `CalibrationCameraPreview` component that receives:

- the current preview image;
- normalized QR corners, when recently detected;
- decoded marker ID;
- the three stage states;
- accepted sample count and current guidance.

`SilentSearchView` uses this component only for `.calibrating`. The existing `n of 3` progress remains visible.

## Data flow

1. `ARSessionManager` publishes an `ARFrameSnapshot`.
2. The view model may convert that snapshot into the newest throttled preview image.
3. `ARSharedMissionFrameCalibrator` scans the full-resolution image.
4. If the expected calibration payload is decoded, it emits QR-detected feedback with normalized corners and marker ID.
5. The calibrator attempts to ground each corner from the same snapshot.
6. If all corners ground, it emits grounded feedback and submits the observation to `SharedMissionCalibrator`.
7. If the sample passes calibration validation, the existing progress event increments the accepted count.
8. Three accepted samples preserve the existing automatic transition to optical exchange readiness.

Feedback carries frame identity so diagnostics cannot accidentally describe a different AR frame. The preview and polygon may be rendered asynchronously, but only the newest values are retained.

## Operator experience

The calibration screen shows the live camera preview with a polygon around the currently decoded marker. The marker ID appears next to the preview.

Each stage uses a stable state:

- gray: waiting;
- yellow: the previous stage succeeded and this stage is pending;
- green: succeeded during this calibration attempt.

Successful stages remain latched until the attempt ends. The polygon itself is live rather than latched: it updates with current detections and disappears after 500 milliseconds without a detection so it does not remain over an obsolete camera position.

Expected guidance includes:

- no QR: `Center the complete marker with its white border visible.`
- QR decoded but grounding failed: `Move the marker toward center or adjust the camera angle.`
- corners grounded: `Hold steady while samples are collected.`
- sample accepted: show the updated `n of 3` progress.

## Failure handling

The calibrator classifies every non-success path needed by the UI or telemetry as one of:

- scanner error;
- no expected QR payload (normal waiting state);
- wrong marker ID;
- stale frame identity or timestamp;
- tracking not normal;
- session-generation mismatch;
- missing depth data;
- failure to sample each named corner;
- calibration-model rejection.

No-QR frames are normal waiting state, not operator errors. Repeated identical failures are deduplicated or rate-limited. Generation mismatch and existing terminal calibration failures retain their current coordinator behavior.

## Runtime telemetry

Add low-volume transition events:

- `silent_search_qr_detected`, including marker and frame identity;
- `silent_search_qr_lost`;
- `silent_search_grounding_failed`, including failed corner or reason;
- `silent_search_corners_grounded`;
- existing calibration progress, rejection, and acceptance events.

Do not log every frame. Emit only state transitions or rate-limited recurring failures. Do not log camera images or QR payload bytes beyond the validated marker ID.

## Testing

### Unit tests

- Calibration feedback stages latch in order and reset between attempts.
- QR-detected feedback is emitted before grounding is attempted.
- Wrong marker IDs do not latch the expected-marker stage.
- Each corner-grounding failure produces the correct reason.
- Grounded feedback and accepted-sample progress preserve frame identity.
- Preview buffering retains only the newest frame and respects throttling.
- Preview work is cancelled when leaving calibration.

### UI tests

- Camera preview appears only during calibration.
- A decoded marker renders its polygon and marker ID.
- The three stage rows render gray, yellow, and green states correctly.
- Accepted sample progress remains `n of 3` and advances automatically.
- Actionable guidance appears for no QR and grounding failure states.

### Regression checks

- Existing Silent Search coordinator and calibration tests pass.
- Existing setup, optical exchange, search, return, and terminal UI behavior remains unchanged.
- Device testing confirms no second camera-session conflict and bounded preview memory usage.

## Acceptance criteria

- During calibration, the operator can see what the AR camera sees.
- A successfully decoded expected marker receives a visible four-corner outline and marker ID.
- The UI independently confirms QR decoding, LiDAR grounding, and accepted samples.
- Successful stages do not flicker back to waiting during the same attempt.
- A QR that cannot be decoded or grounded produces actionable guidance within one second.
- Preview rendering does not reduce full-resolution scanning or change calibration safety thresholds.
- Leaving calibration stops preview processing and clears attempt-local visual state.
- Runtime logs distinguish scanner failure from corner-grounding failure without per-frame spam.
