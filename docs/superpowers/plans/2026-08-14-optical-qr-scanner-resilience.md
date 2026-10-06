# Optical QR Scanner Resilience Implementation Plan

> **For agentic workers:** Execute tasks in order with TDD on `codex/silent-search-v1`. Commit each green task or cohesive pair.

**Goal:** Prevent physical-device Vision barcode errors from aborting QR calibration when another Vision orientation or Core Image can still decode the marker.

**Architecture:** Add a detailed, backward-compatible scan outcome to `OpticalQRCodeScanner`; run each Vision orientation and Core Image independently; and route sanitized backend diagnostics through existing calibration feedback into transition-only telemetry without overwriting successful calibration state.

**Tech stack:** Swift 6, XCTest, Vision, Core Image, ARKit, PhroverKit, Xcode iOS Simulator, and iPhone 15 Pro Max device verification.

**Approved design:** `docs/superpowers/specs/2026-08-14-optical-qr-scanner-resilience-design.md`

## Global constraints

- Keep `OpticalQRCodeScanner.scan(_:) -> [OpticalObservation]` source-compatible.
- Only an invalid frame timestamp may throw from the detailed scan API.
- Remove deprecated `VNDetectBarcodesRequest.usesCPUOnly` configuration.
- One backend or orientation failure must never prevent later attempts.
- Preserve duplicate-frame suppression and canonical `.right` corner coordinates.
- Do not change QR payload validation, calibration geometry, LiDAR grounding, or acceptance thresholds.
- Do not log images, payload bytes, localized error descriptions, or stack traces.
- Backend diagnostics must not replace successful QR, grounding, or accepted-sample UI state.

---

### Task 1: Add a detailed best-effort scanner outcome

**Files:**
- Modify: `swift/Sources/PhroverKit/SilentSearch/OpticalExchangeService.swift`
- Modify: `swift/Tests/PhroverKitTests/OpticalExchangeServiceTests.swift`

**Interfaces:**
- Add `OpticalScannerBackend`, `OpticalScannerBackendDiagnostic`, and `OpticalScanOutcome` as data-only, `Equatable`, `Sendable` values.
- Diagnostics contain only backend, optional orientation, error domain, and numeric code.
- Add `scanDetailed(_:) -> OpticalScanOutcome`.
- Keep `scan(_:)` as a wrapper returning `scanDetailed(_).observations`.
- Add internal injected Vision/Core Image seams so tests can force deterministic empty, error, and success outcomes without mocking private implementation details.

- [ ] Write a failing test where `.right` Vision throws and `.up` Vision decodes; assert the observation and one sanitized diagnostic are returned.
- [ ] Write a failing test where all four Vision orientations throw and Core Image decodes.
- [ ] Test Vision errors plus empty Core Image returns empty observations with diagnostics rather than throwing.
- [ ] Test a clean backend cycle returns no diagnostics and duplicate frames remain suppressed.
- [ ] Test diagnostics expose domain/code but no localized description, image, or payload.
- [ ] Remove `usesCPUOnly`, catch each orientation independently, and always reach Core Image when Vision has no observation.
- [ ] Keep all orientation canonicalization tests green.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/OpticalExchangeServiceTests
```

---

### Task 2: Route backend diagnostics through calibration telemetry

**Files:**
- Modify: `swift/Sources/PhroverKit/SilentSearch/SilentSearchCalibrationFeedback.swift`
- Modify: `swift/Sources/PhroverKit/SilentSearch/Device/ARSharedMissionFrameCalibrator.swift`
- Modify: `swift/Sources/PhroverKit/SilentSearch/SilentSearchCoordinator.swift`
- Modify: `swift/Tests/PhroverKitTests/ARSharedMissionFrameCalibratorTests.swift`
- Modify: `swift/Tests/PhroverKitTests/SilentSearchCoordinatorTests.swift`
- Modify: `swift/Tests/PhroverKitTests/RuntimeSilentSearchEventSinkTests.swift`

**Interfaces:**
- Add calibration feedback carrying frame context plus one `OpticalScannerBackendDiagnostic`.
- Production calibration consumes `scanDetailed`; test scanners return deterministic `OpticalScanOutcome` values.
- Emit diagnostics independently before processing any returned observations.
- Record `silent_search_scanner_backend_failed` with backend, optional orientation, domain, code, and existing mission/frame context.
- Consecutive identical diagnostic tuples are logged once. A scan cycle without that diagnostic resets deduplication so recurrence logs again.
- Empty observations remain waiting-for-marker state. Observations continue through marker preflight and LiDAR grounding even when diagnostics are also present.

- [ ] Test fallback success emits a backend diagnostic and then QR-detected feedback without emitting `scanner_failure`.
- [ ] Test empty fallback output restores or retains waiting-for-marker guidance.
- [ ] Test a backend diagnostic does not clear latched QR, grounded, or accepted-sample state.
- [ ] Test identical diagnostic deduplication, clean-cycle reset, and changed orientation/domain/code transitions.
- [ ] Test telemetry field allow-list and absence of payload/image/error-description fields.
- [ ] Remove use of `silent_search_grounding_failed reason=scanner_failure` for recoverable backend errors.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/ARSharedMissionFrameCalibratorTests
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SilentSearchCoordinatorTests
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/RuntimeSilentSearchEventSinkTests
```

---

### Task 3: Run regressions and physical-device proof

- [ ] Run the automated gate:

```bash
scripts/test-swift-sdk.sh
xcodebuild build   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "generic/platform=iOS"
git diff --check
git status --short
```

- [ ] Build and install the same commit on the iPhone 15 Pro Max.
- [ ] Connect to `UGV_1`, confirm the command link is ready, and start calibration.
- [ ] Reproduce the marker framing that previously produced `scanner_failure`.
- [ ] Confirm the camera preview remains active and calibration shows waiting-for-marker or advances; it must not stop at a recoverable backend failure.
- [ ] Reframe the marker until one backend decodes it and verify QR, grounding, and accepted-sample stages advance independently.
- [ ] Pull `Documents/phrover-runtime.log` and confirm `silent_search_scanner_backend_failed` includes sanitized domain/code while no recoverable `scanner_failure` blocks calibration.
- [ ] Record the commit, device, result, and sanitized log path before calling the fix device-verified.
