# Silent Search Calibration Camera Feedback Implementation Plan

> **For agentic workers:** Execute tasks in order with TDD on `codex/silent-search-v1`. Commit after each green task or cohesive pair. Do not stage the unrelated `.serena/project.yml` modification.

**Goal:** Show the live AR camera feed during Silent Search calibration and independently confirm expected-marker QR decoding, four-corner LiDAR grounding, and accepted calibration samples.

**Architecture:** Keep `ARSessionManager` as the sole camera owner. Add typed, image-free calibration feedback to PhroverKit; latch attempt-level state in `SilentSearchCoordinator`; and render a throttled latest-frame preview plus a transient QR polygon in the PhroverOperator app. Full-resolution scanning and calibration safety behavior remain unchanged.

**Tech stack:** Swift 6, XCTest, ARKit, Vision, Core Image, SwiftUI, Xcode iOS Simulator, and a LiDAR-capable iPhone for final verification.

**Approved design:** `docs/superpowers/specs/2026-08-14-silent-search-calibration-camera-feedback-design.md`

## Global constraints

- Use the existing `ARSessionManager.session`; never create an `AVCaptureSession` or second AR session.
- Keep RGB, depth, camera geometry, frame identity, and timestamps from one coherent `ARFrameSnapshot`.
- Scan full-resolution frames even though preview conversion is limited to at most 10 FPS.
- Buffer only the newest preview frame and reuse one `CIContext`.
- Keep UIKit image types in PhroverOperator; PhroverKit feedback contains only domain values and normalized coordinates.
- Latch successful stages until the attempt ends. Do not latch the polygon; clear it after 500 milliseconds without a detection.
- Do not change marker dimensions, acceptance thresholds, required sample count, generation checks, or safety behavior.
- Do not log images, pixel buffers, complete payloads, or per-frame state.
- Preserve all existing Silent Search phases and automatic advancement after three accepted samples.

---

### Task 1: Define typed calibration feedback and coordinator state

**Files:**
- Add: `swift/Sources/PhroverKit/SilentSearch/SilentSearchCalibrationFeedback.swift`
- Modify: `swift/Sources/PhroverKit/SilentSearch/SilentSearchDependencies.swift`
- Modify: `swift/Sources/PhroverKit/SilentSearch/SilentSearchCoordinator.swift`
- Modify: `swift/Tests/PhroverKitTests/SilentSearchCoordinatorTests.swift`

**Interfaces:**
- Add `SilentSearchCalibrationCorner` with `topLeft`, `topRight`, `bottomLeft`, and `bottomRight`.
- Add a typed `SilentSearchCalibrationFeedback` enum for expected QR detection, QR loss, scanner failure, grounding failure, and all-corners-grounded.
- Every frame-specific case carries `ARFrameID` and monotonic timestamp. Expected-marker detection also carries the validated marker ID and normalized `OrientedMarkerCorners`.
- Add a typed grounding-failure reason that distinguishes frame mismatch, timestamp mismatch, invalid payload, wrong marker ID, missing depth map, and the named corner that could not be sampled.
- Extend `SilentSearchCalibrationEvent` with a feedback case; do not create a second calibration event stream.
- Add `SilentSearchCalibrationVisualState` to the coordinator with latched `qrDecoded`, `cornersGrounded`, and `sampleAccepted` flags plus current marker ID, polygon, last detection timestamp, and current typed issue.

- [ ] Write coordinator tests first: expected-marker feedback latches QR state, grounding feedback cannot clear it, progress latches sample acceptance, and later no-QR frames do not flicker completed stages.
- [ ] Test wrong-marker and scanner-failure feedback without latching expected-marker success.
- [ ] Test QR loss clears only the current polygon/marker presentation, not the attempt-level stage flags.
- [ ] Test state reset on new calibration, abort, reset, terminal failure, and accepted completion.
- [ ] Implement the feedback types and one `receiveCalibration` branch that updates state without changing phase semantics.
- [ ] Keep existing progress/rejected/accepted behavior and telemetry assertions green.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SilentSearchCoordinatorTests
```

---

### Task 2: Classify scanner and LiDAR grounding outcomes

**Files:**
- Modify: `swift/Sources/PhroverKit/SilentSearch/Device/ARSharedMissionFrameCalibrator.swift`
- Modify: `swift/Tests/PhroverKitTests/ARSharedMissionFrameCalibratorTests.swift`

**Interfaces:**
- Replace the optional-only grounding seam with a typed result that returns either `SharedMissionCalibrationObservation` or one exact grounding-failure reason.
- Preserve a single ordered validation path so tests establish deterministic precedence: tracking, generation/frame identity, timestamp, payload, expected marker, depth availability, then top-left/top-right/bottom-left/bottom-right sampling.
- Emit expected-marker feedback immediately after decoding and before attempting LiDAR grounding.
- Emit all-corners-grounded only when every corner is sampled from that same snapshot.
- Emit one QR-lost transition after 500 milliseconds without detecting the expected marker; do not emit one event per empty frame.
- Catch scanner errors explicitly instead of discarding them with `try?`.

- [ ] Extend the existing grounding tests to assert every typed failure and its precedence.
- [ ] Write an async test proving QR-detected feedback arrives before grounding failure for the same frame.
- [ ] Write an async test proving all-corners-grounded precedes calibration progress.
- [ ] Test scanner exceptions, wrong marker IDs, and no-result frames.
- [ ] Test that repeated empty frames and repeated identical failures are deduplicated, while a later successful detection can emit again.
- [ ] Implement the classified pipeline without weakening any guard.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/ARSharedMissionFrameCalibratorTests
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SharedMissionCalibrationTests
```

---

### Task 3: Add transition-only calibration telemetry

**Files:**
- Modify: `swift/Sources/PhroverKit/SilentSearch/SilentSearchCoordinator.swift`
- Modify: `swift/Tests/PhroverKitTests/SilentSearchCoordinatorTests.swift`
- Modify: `swift/Tests/PhroverKitTests/RuntimeSilentSearchEventSinkTests.swift`

**Interfaces:**
- Record `silent_search_qr_detected` with marker, generation, and frame sequence.
- Record `silent_search_qr_lost` only after a previously detected marker becomes absent for 500 milliseconds.
- Record `silent_search_grounding_failed` with a stable typed reason and named corner when applicable.
- Record `silent_search_corners_grounded` with generation and frame sequence.
- Deduplicate consecutive identical scanner/grounding states. Existing progress, rejection, and acceptance events remain authoritative.

- [ ] Write event-sink tests for exact event names and allowed fields.
- [ ] Assert images, payload bytes, and unvalidated payload strings never enter logs.
- [ ] Assert repeated identical feedback does not produce per-frame telemetry spam.
- [ ] Implement transition logging in the coordinator, where attempt-local state already exists.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SilentSearchCoordinatorTests
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/RuntimeSilentSearchEventSinkTests
```

---

### Task 4: Build the bounded preview pipeline and overlay transform

**Files:**
- Add: `examples/PhroverOperator/PhroverOperator/App/CalibrationPreviewModel.swift`
- Add: `examples/PhroverOperator/PhroverOperatorTests/CalibrationPreviewModelTests.swift`
- Modify: `examples/PhroverOperator/PhroverOperator.xcodeproj/project.pbxproj`

**Interfaces:**
- Add a main-actor `CalibrationPreviewModel` that starts from an injected `AsyncStream<ARFrameSnapshot>`, exposes only the latest `UIImage`, and has explicit `start()`/`stop()` lifecycle.
- Inject frame rendering and timing seams for tests. Production rendering uses one retained `CIContext` and the same `.right` display orientation used by calibration scanning.
- Enforce at least 100 milliseconds between preview conversions and rely on newest-only stream buffering.
- Add a pure `CalibrationPreviewTransform` that maps normalized scanner coordinates into an aspect-fit preview rectangle, including the bottom-left-to-top-left coordinate conversion.

- [ ] Write failing throttle tests using snapshot timestamps: first frame renders, frames inside 100 milliseconds are skipped, and the newest eligible frame replaces the previous image.
- [ ] Test that `stop()` cancels consumption and releases the latest image.
- [ ] Test repeated start/stop cycles do not retain duplicate tasks.
- [ ] Write transform tests for portrait preview bounds, aspect-fit letterboxing, all four corners, and out-of-range coordinate clamping.
- [ ] Implement the preview model and transform without performing QR scanning in the app layer.
- [ ] Add both source and test files to the explicit Xcode project groups and build phases.
- [ ] Verify:

```bash
xcodebuild test   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "id=$(scripts/test-swift-sdk.sh --print-udid)"   -only-testing:PhroverOperatorTests/CalibrationPreviewModelTests
```

---

### Task 5: Wire preview lifecycle into the Silent Search view model

**Files:**
- Modify: `examples/PhroverOperator/PhroverOperator/App/SilentSearchViewModel.swift`
- Modify: `examples/PhroverOperator/PhroverOperatorTests/CalibrationPreviewModelTests.swift`

**Interfaces:**
- Extend the app-only `SilentSearchViewModel` protocol with preview image, calibration visual state, guidance text, and preview lifecycle methods.
- `LiveSilentSearchViewModel` owns one `CalibrationPreviewModel` composed from its existing `ARSessionManager`.
- `ScriptedSilentSearchViewModel` supplies deterministic preview and feedback fixtures without starting ARKit.
- Start preview consumption only when the calibration view appears; stop and clear it when that view disappears or the attempt ends.
- Guidance precedence is exact: tracking/session failure, scanner failure, wrong marker, grounding failure, grounded/holding steady, then default centering guidance.

- [ ] Test the mapping from each typed coordinator state to marker text, stage colors, and guidance.
- [ ] Test that latched stages survive transient no-QR state while the polygon expires.
- [ ] Test that leaving `.calibrating` stops the preview and clears presentation state.
- [ ] Implement view-model projection only; keep scanning and calibration decisions in PhroverKit.
- [ ] Verify the focused app unit tests from Task 4.

---

### Task 6: Render the calibration camera and staged feedback

**Files:**
- Add: `examples/PhroverOperator/PhroverOperator/Views/CalibrationCameraPreview.swift`
- Modify: `examples/PhroverOperator/PhroverOperator/Views/SilentSearchView.swift`
- Modify: `examples/PhroverOperator/PhroverOperator/App/SilentSearchViewModel.swift`
- Modify: `examples/PhroverOperator/PhroverOperatorUITests/SilentSearchUITests.swift`
- Modify: `examples/PhroverOperator/PhroverOperator.xcodeproj/project.pbxproj`

**Interfaces:**
- `CalibrationCameraPreview` renders the latest image aspect-fit, maps the current normalized corners through `CalibrationPreviewTransform`, and draws a visible four-edge polygon.
- Show the validated marker ID near the preview while current or latched detection state exists.
- Render three accessibility-addressable rows: `QR decoded`, `LiDAR corners grounded`, and `Sample accepted`.
- Use gray for waiting, yellow for the currently pending stage, and green for latched success.
- Preserve the existing `n of 3` progress and automatic phase advancement.
- Stable accessibility IDs cover the preview, polygon, marker ID, each stage, guidance, and progress.

- [ ] Extend the scripted `calibrating` scenario with a deterministic preview image and add scenarios for QR detected, grounding failed, and sample accepted.
- [ ] Write UI tests first for preview-only-during-calibration, polygon and marker ID, all stage states, guidance, and unchanged progress.
- [ ] Implement the focused component and replace the current icon-only calibration content.
- [ ] Add the new view to the explicit Xcode project source build phase.
- [ ] Verify:

```bash
xcodebuild test   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "id=$(scripts/test-swift-sdk.sh --print-udid)"   -only-testing:PhroverOperatorUITests/SilentSearchUITests
```

---

### Task 7: Run regressions and the focused device proof

**Files:**
- Update: `docs/silent-search-device-acceptance.md` with the focused calibration-feedback result and sanitized log path.

- [ ] Run the complete automated gate:

```bash
scripts/test-swift-sdk.sh
xcodebuild test   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "id=$(scripts/test-swift-sdk.sh --print-udid)"   -only-testing:PhroverOperatorTests
xcodebuild test   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "id=$(scripts/test-swift-sdk.sh --print-udid)"   -only-testing:PhroverOperatorUITests/SilentSearchUITests
xcodebuild build   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "generic/platform=iOS"
git diff --check
git status --short
```

- [ ] Install one commit on the iPhone 15 Pro Max and connect it to `UGV_1`.
- [ ] Confirm the setup command-link gate is ready before starting calibration.
- [ ] Reproduce the shallow-angle marker view: verify the preview matches the camera and remains at QR waiting state without falsely claiming LiDAR failure.
- [ ] Reframe the complete marker with its white border visible: verify the polygon and expected marker ID appear, then verify grounding and `1 of 3` progress independently.
- [ ] Hold for three distinct accepted frames and confirm automatic advancement.
- [ ] Move the marker out of view and confirm the polygon clears within 500 milliseconds while successful stage rows remain latched until the attempt ends.
- [ ] Pull `Documents/phrover-runtime.log` and confirm transition-only QR and grounding events distinguish the stages without image data or per-frame spam.
- [ ] Record the commit, device, marker placement, result, and sanitized log path. Do not describe the feature as device-verified until this row passes.
