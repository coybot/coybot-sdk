# Silent Search Implementation Plan

> **For agentic workers:** Execute tasks in order with TDD. Work only in `/Users/hungmai/Sites/Astral/astral-sdk-silent-search` on `codex/silent-search-v1`. Keep `NavigationController` as the sole wheel-motion authority. Commit after each green task or cohesive pair of tasks.

**Goal:** Implement the approved two-rover, QR-only Silent Search demonstration from shared-marker calibration through sector search, timed rendezvous, optical target reporting, and two-rover convergence.

**Architecture:** Add a deterministic `PhroverKit/SilentSearch` module with pure geometry, protocol, exploration, tracking, and coordinator cores. Add frame-coherent ARKit, Vision, navigation, and readiness adapters at the package edge. Add a dedicated SwiftUI tab only after the complete coordinator can be composed. No cloud or `RoverTeamRadio` participates.

**Tech stack:** Swift 6, XCTest, ARKit, Vision, Core Image, CryptoKit, SwiftUI, RoverNav, PhroverKit, Xcode iOS Simulator, two LiDAR-capable iPhones for final acceptance.

**Approved design:** `docs/superpowers/specs/2026-08-11-silent-search-design.md`

## Global constraints

- Base all implementation on `main` commit `2f2eedc`; do not copy code from the unrelated dirty primary worktree.
- Use one coherent AR frame for RGB, camera, depth, pose, frame identity, and tracking state.
- A shared frame is valid only for the AR session generation that produced it.
- Keep floating-point values out of optical wire payloads: millimeters, millidegrees, basis points, and epoch milliseconds are integers.
- `NavigationController` remains the only component that sends autonomous wheel commands.
- Apply sector policy to every search and return initial plan/replan. Release it only after the acknowledged convergence commit so a rover may reach a target found in its peer's sector. Never continue on an old path after a failed or rejected replan.
- Stop motion immediately on operator Stop, AR tracking loss, transport failure, reactive safety failure, or shared-frame invalidation.
- Do not expose unfinished Silent Search behavior in the app. Add the tab only in Task 16 after live composition exists.
- Do not log camera images, pixel buffers, or complete QR payloads.
- Device verification is a separate release gate. Code completion must not be described as device verification.

---

### Task 1: Establish the repeatable Swift SDK test gate

**Files:**
- Add: `scripts/test-swift-sdk.sh`
- Add: `scripts/tests/test-test-swift-sdk.sh`
- Modify: `.github/workflows/swift.yml`
- Modify: `README.md`

**Interfaces:**
- `scripts/test-swift-sdk.sh` selects an available iOS 26+ iPhone simulator, honors `SIM_UDID`, forwards all remaining `xcodebuild` arguments, and runs `RoverNavTests` plus `PhroverKitTests` through `coybot-sdk-Package`.

- [ ] Write `scripts/tests/test-test-swift-sdk.sh`; prepend a temporary directory containing stub `xcrun` and `xcodebuild` executables to `PATH`, feed a fixed simulator JSON fixture, and assert `--print-udid`, `SIM_UDID`, and forwarded arguments without depending on installed simulators.
- [ ] Implement the wrapper without hard-coding a simulator model.
- [ ] Change CI from RoverNav-only testing to the two regular test targets; keep live probes and simulation targets excluded.
- [ ] Document the wrapper as the normal package test command.
- [ ] Verify:

```bash
scripts/tests/test-test-swift-sdk.sh
scripts/test-swift-sdk.sh --print-udid
scripts/test-swift-sdk.sh -only-testing:RoverNavTests/FrontierFinderTests
swift build --target RoverNav
git diff --check
```

---

### Task 2: Define shared mission coordinates and fixed geometry

**Files:**
- Add: `swift/Sources/PhroverKit/SilentSearch/SharedMissionFrame.swift`
- Add: `swift/Tests/PhroverKitTests/SharedMissionFrameTests.swift`

**Interfaces:**
- Add `MissionPoint`, `MissionPose`, `RoverRole`, `SearchSector`, `SharedMissionFrame`, and `SilentSearchGeometry`.
- Mission `x` is east, `y` is north, heading zero is north, and positive heading is counterclockwise.
- `SharedMissionFrame` stores local origin, local north heading, and AR session generation; it converts points and poses in both directions.
- Fixed geometry includes the ±0.25 m center band, rendezvous points `(-0.60, -0.80)` and `(0.60, -0.80)`, 0.60 m target offsets, 0.20 m position tolerance, and 10-degree heading tolerance.

- [ ] Write failing identity, translated, rotated, wraparound, and round-trip tests.
- [ ] Test that mission heading semantics do not reuse `Pose2D.yaw` ambiguously.
- [ ] Test generation validity and every fixed geometry value.
- [ ] Implement finite-value validation and normalized headings.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SharedMissionFrameTests
```

---

### Task 3: Validate synthetic shared-marker calibration

**Files:**
- Add: `swift/Sources/PhroverKit/SilentSearch/SharedMissionCalibration.swift`
- Add: `swift/Tests/PhroverKitTests/SharedMissionCalibrationTests.swift`
- Add: `scripts/generate-silent-search-marker.swift`
- Add: `docs/assets/silent-search-calibration-marker.pdf`
- Add: `docs/silent-search-marker.md`

**Interfaces:**
- Add data-only `OrientedMarkerCorners`, `SharedMissionCalibrationObservation`, `SharedMissionCalibrationConfiguration`, `SharedMissionCalibrationDiagnostic`, and `SharedMissionCalibrator`.
- The marker QR payload grammar is exactly `PHROVER-CAL|1|<marker-id>`, where marker ID matches `[A-Z0-9_-]{1,24}`. The committed marker uses ID `SILENT_SEARCH_01`.
- `scripts/generate-silent-search-marker.swift` renders a vector PDF with a QR whose black-module square is exactly 0.20 m wide at 100% print scale and a north arrow centered beyond its logical top edge. `docs/silent-search-marker.md` requires actual-size printing and ruler verification.
- One observation carries marker ID, session generation, frame ID, monotonic frame timestamp, and four already-grounded, logically oriented corners. After Vision receives `.right` image orientation, its `topLeft`/`topRight` edge is the marker north edge aligned with the printed arrow; `bottomLeft`/`bottomRight` is south. Real-device confirmation of this convention is mandatory in Task 18.
- Derive origin from the four-corner mean, north from bottom-edge midpoint to top-edge midpoint, and width from top/bottom edge lengths.
- Accept three distinct frames within two seconds only when all origins are within 0.10 m of the component median, headings within 5 degrees of the circular mean, and widths within 15% of 0.20 m.

- [ ] Generate and commit the marker PDF, then test the generator's embedded payload, PDF page dimensions, 0.20 m QR extent, and arrow/top-edge alignment.
- [ ] Write failing derivation tests, including rotated markers and degenerate/non-finite corners.
- [ ] Write boundary tests for frame uniqueness, two seconds, origin, circular heading around ±π, width, marker ID, and generation.
- [ ] Define deterministic diagnostic precedence in tests.
- [ ] Implement the accumulator; repeated frame IDs replace or ignore evidence but never increase the count.
- [ ] Verify:

```bash
swift scripts/generate-silent-search-marker.swift --check docs/assets/silent-search-calibration-marker.pdf
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SharedMissionCalibrationTests
```

---

### Task 4: Freeze the canonical optical wire schema

**Files:**
- Add: `swift/Sources/PhroverKit/SilentSearch/OpticalMessage.swift`
- Add: `swift/Sources/PhroverKit/SilentSearch/OpticalMessageCodec.swift`
- Add: `swift/Tests/PhroverKitTests/OpticalMessageCodecTests.swift`

**Interfaces:**
- Protocol version is integer `1`. Envelope keys are exactly `b` body, `c` checksum, `i` lowercase mission UUID, `k` kind, `m` marker ID, `q` sequence, `r` role, `t` epoch milliseconds, and `v` protocol version.
- Kind raw values are `offer`, `accept`, `searchCommit`, `searchAck`, `status`, `decision`, `converge`, and `convergeAck`.
- Sequence starts at 1 and increases globally per sender; gaps are allowed.
- Coordinates/dimensions use signed `Int32` millimeters, headings signed `Int32` millidegrees, confidence `UInt16` basis points in `0...10000`, durations `UInt16` seconds, and time `Int64` epoch milliseconds. Strings are UTF-8, labels match `[a-z0-9_-]{1,40}`, marker IDs match `[A-Z0-9_-]{1,24}`, and hashes are 64 lowercase hex characters.
- Body schemas use these exact keys:
  - `offer`: `d` search seconds (`30...3600`), `e` center half-width mm, `n` target label, `w` marker width mm, and `ra`/`rb` rendezvous poses `{x,y,h}`.
  - `accept`: `h` full offer hash and `u` Rover B wall time in epoch ms.
  - `searchCommit`: `d` deadline epoch ms, `h` full acceptance hash, and `s` start epoch ms.
  - `searchAck`: `h` full search-commit hash.
  - `status`: `f` Boolean found; `p` previous A-status hash is omitted for A and required for B; when found, require `n` label, `x`/`y` coordinate mm, `c` rounded arithmetic mean confidence basis points, and `z` sample count `UInt16`; omit those keys when not found.
  - `decision`: `a` A-status hash, `b` B-status hash, `o` one of `found`, `notFound`, or `conflict`; require `x`/`y` only for `found`.
  - `converge`: `h` decision hash and `s` release epoch ms; require selected `x`/`y` mm only when the linked decision is `found`, forbid them for `notFound`, and forbid this message entirely after `conflict`.
  - `convergeAck`: `h` convergence-commit hash.
- Unknown, missing, forbidden, or `null` body keys fail decoding. Canonical JSON uses sorted keys.
- Checksum is lowercase hex of the first 16 SHA-256 bytes over canonical UTF-8 JSON with `c` omitted.
- A message-link hash is full lowercase SHA-256 over complete canonical message bytes including `c`.
- QR rendering uses correction level `M`; encoded canonical messages must be at most 1,200 UTF-8 bytes or encoding fails before rendering.
- Default validity accepts exactly 120 seconds old and exactly 5 seconds future; values beyond those boundaries fail.
- The first golden encoded offer is exactly:

```json
{"b":{"d":180,"e":250,"n":"chair","ra":{"h":0,"x":-600,"y":-800},"rb":{"h":0,"x":600,"y":-800},"w":200},"c":"9141f298cb2bbd51adbe56cb93861b7b","i":"00000000-0000-0000-0000-000000000001","k":"offer","m":"SILENT_SEARCH_01","q":1,"r":"a","t":1786406400000,"v":1}
```

- [ ] Write one golden offer fixture asserting these exact bytes and checksum before implementing generalized Codable support.
- [ ] Add golden fixtures for all eight kinds and their typed bodies, including found/not-found convergence bodies and rejection of convergence after conflict.
- [ ] Add tampering, unsupported version, malformed UTF-8/JSON, unknown key, noncanonical JSON, integer range, stale, and future tests.
- [ ] Implement encode/decode, canonical re-encoding validation, checksum, and message-link hash with CryptoKit.
- [ ] Render the largest status/decision fixtures and assert they remain under the chosen QR payload ceiling.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/OpticalMessageCodecTests
```

---

### Task 5: Implement transactional optical protocol sessions

**Files:**
- Add: `swift/Sources/PhroverKit/SilentSearch/OpticalProtocolSession.swift`
- Add: `swift/Tests/PhroverKitTests/OpticalProtocolSessionTests.swift`

**Interfaces:**
- Add `OpticalProtocolContext`, `OpticalProtocolPhase`, `OpticalProtocolSession`, `OpticalSequenceTracker`, `OutboundOpticalSequence`, and typed rejection reasons.
- Incoming validation checks syntax/checksum first, then phase, mission, marker, role, timestamp, sequence, schedule, and linked hashes. State changes only after all checks pass.
- Invalid scans do not consume sequence. Retry returns identical bytes and sequence.
- Handshake order is offer → acceptance → search commit at least 30 seconds ahead → hash acknowledgement accepted at least five seconds before start.
- Clock validation is exact: Rover B checks `abs(B wall at offer scan - offer.t) <= 2_000 ms`; acceptance requires `accept.u == accept.t`; Rover A checks `abs(A wall at acceptance scan - accept.u) <= 2_000 ms`. Accept exactly 2,000 ms and reject 2,001 ms. Schedule timestamps are interpreted only after these checks pass.
- Rendezvous order is A status → B status linked to A → A decision linked to both → convergence commit → hash acknowledgement.
- Sole-finder selection, dual-report median within 0.50 m, conflict, and neither-found are deterministic.

- [ ] Write failing transaction and replay tests before the phase machine.
- [ ] Implement offer/acceptance and clock disagreement boundaries.
- [ ] Implement search schedule/acknowledgement timing.
- [ ] Implement status chaining, decision selection, convergence commit, and terminal not-found.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/OpticalProtocolSessionTests
```

---

### Task 6: Render and scan QR payloads independently of mission state

**Files:**
- Add: `swift/Sources/PhroverKit/SilentSearch/OpticalExchangeService.swift`
- Add: `swift/Tests/PhroverKitTests/OpticalExchangeServiceTests.swift`

**Interfaces:**
- Add `OpticalFrame`, `OpticalObservation`, `OpticalQRCodeRenderer`, and `OpticalQRCodeScanner`.
- Renderer uses Core Image QR generation, integer scaling, high contrast, and a four-module white quiet zone.
- Scanner uses `VNDetectBarcodesRequest` restricted to QR and returns payload bytes, frame ID, timestamp, and normalized oriented quadrilateral.
- Scanner suppresses repeated processing of a frame ID but performs no mission/sequence validation.

- [ ] Write generated-image round-trip tests using all golden payloads and the largest payload.
- [ ] Test quiet zone, hard pixel edges, invalid images, non-QR barcodes, and repeated frame IDs.
- [ ] Follow `Detector`'s `.right` Vision orientation convention and preserve the observation corners for later device calibration.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/OpticalExchangeServiceTests
```

---

### Task 7: Add typed path admissibility to every navigation plan

**Files:**
- Add: `swift/Sources/PhroverKit/Nav/PathAdmissibilityPolicy.swift`
- Add: `swift/Sources/PhroverKit/SilentSearch/SectorPathPolicy.swift`
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift`
- Add: `swift/Tests/PhroverKitTests/NavigationPathPolicyTests.swift`
- Add: `swift/Tests/PhroverKitTests/SectorPathPolicyTests.swift`
- Modify: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift`

**Interfaces:**
- Add `PathPolicyViolation` and `PathAdmissibilityResult`, plus `PathAdmissibilityPolicy.evaluate(path:)`.
- Add `NavigationFailure` cases `noPose`, `noPath`, `pathRejected(PathPolicyViolation)`, `obstacle`, `commsLost`, `tipping`, `stalled`, `commandFailed`, `trackingLost`, and `cancelled`; add `NavigationResult` cases `arrived`, `failed(NavigationFailure)`, and `cancelled`.
- Preserve existing synchronous `navigate(to:)`, `navigate(to:stoppingAtForwardClearance:)`, `rotate(by:)`, `cancel()`, and string-facing `State` for current consumers. Add `navigateAndWait(to:stoppingAtForwardClearance:policy:) async -> NavigationResult`, `rotateAndWait(by:) async -> NavigationResult`, and `cancelAndWait() async`; existing methods delegate to the same typed internal operations.
- Add an optional per-operation policy; unrestricted behavior remains the default and policy state is cleared at completion/cancel.
- Evaluate exact start + planned waypoints + exact goal.
- Rover A allows only mission `x < -0.25`; Rover B only `x > 0.25`; boundaries, center band, opposite sector, and non-finite points fail.
- Initial and periodic plans return typed outcomes. Policy rejection is distinct from no-path and safety/transport failure.
- A rejected or missing replan awaits stop, clears the active path, completes with failure, and returns before pursuit or wheel send. Reset `replanCounter` for each goal.

- [ ] Write pure west/east, transformed-frame, intermediate crossing, boundary, and non-finite policy tests.
- [ ] Add a planner/command test seam without changing the public production initializer.
- [ ] Write failing initial rejection tests proving no path storage or command send.
- [ ] Write failing replan tests proving stop and no continued use of the old path.
- [ ] Implement typed outcomes and map them to the existing string-facing `State` without changing its cases.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/SectorPathPolicyTests \
  -only-testing:PhroverKitTests/NavigationPathPolicyTests \
  -only-testing:PhroverKitTests/NavigationSafetyTests
```

---

### Task 8: Select and remember sector frontiers deterministically

**Files:**
- Add: `swift/Sources/PhroverKit/SilentSearch/SectorExplorer.swift`
- Add: `swift/Tests/PhroverKitTests/SectorExplorerTests.swift`

**Interfaces:**
- Dedicated candidates have stable ID, local/mission centroid, width, status, rejection reason, safe path, and path length. Do not reuse `ExplorationCandidate`.
- Rebuilt observations strictly within 0.30 m retain identity. Build every eligible old/new pair, sort pairs by distance ascending, old numeric stable ID ascending, new mission x ascending, then y ascending, and greedily accept only pairs whose old and new members are both unmatched. Sort unmatched new observations by mission x, y, then width before assigning monotonic IDs `frontier_1`, `frontier_2`, and so on. Visited/rejected state lasts for the mission.
- Exclude visited, rejected, unreachable, out-of-sector destination, and policy-invalid path candidates.
- Rank by path length ascending, width descending, mission x ascending, mission y ascending, then stable ID.
- Mark visited only after arrival, settle, and scan. Exhaustion is explicit.

- [ ] Write identity/reordering/persistent-state tests.
- [ ] Write destination, full-path, unreachable, tie-break, and exhaustion tests using synthetic costmaps/frontiers.
- [ ] Compose existing `FrontierFinder`, `AStarPlanner`, and the same `SectorPathPolicy` sent to navigation.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh   -only-testing:RoverNavTests/FrontierFinderTests   -only-testing:PhroverKitTests/SectorExplorerTests
```

---

### Task 9: Confirm frame-distinct, depth-grounded targets

**Files:**
- Add: `swift/Sources/PhroverKit/SilentSearch/TargetTracker.swift`
- Add: `swift/Tests/PhroverKitTests/TargetTrackerTests.swift`

**Interfaces:**
- `TargetFrameObservation` carries frame ID, monotonic timestamp, and `[TargetDetectionObservation]`; each detection contains label, confidence, normalized center, and an optional local grounded point produced from that same frame. The pure tracker never calls ARKit or a grounding closure.
- Match the canonical label exactly and choose only the highest-confidence matching detection per frame.
- Require confidence ≥0.90 and valid shared-frame grounding. Do not fall through to a lower-confidence box when the chosen box cannot ground.
- Count each frame once, trim evidence older than two seconds, and confirm three samples whose points are each within 0.35 m of the component median.
- Confirmation latches label, coordinate, sample count, and confidence summary.

- [ ] Write label, confidence, best-box, ungrounded, duplicate-frame, transformed-point, time-window, cluster, outlier expiry, median, and latch tests.
- [ ] Emit structured rejection/confirmation events through an injected sink, never raw images.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/TargetTrackerTests
```

---

### Task 10: Model coordinator state and dependencies

**Files:**
- Add: `swift/Sources/PhroverKit/SilentSearch/SilentSearchMission.swift`
- Add: `swift/Sources/PhroverKit/SilentSearch/SilentSearchDependencies.swift`
- Add: `swift/Sources/PhroverKit/SilentSearch/SilentSearchCoordinator.swift`
- Add: `swift/Sources/PhroverKit/SilentSearch/RuntimeSilentSearchEventSink.swift`
- Add: `swift/Tests/PhroverKitTests/SilentSearchCoordinatorTests.swift`
- Add: `swift/Tests/PhroverKitTests/Support/SilentSearchTestDoubles.swift`

**Interfaces:**
- `@MainActor @Observable SilentSearchCoordinator` has explicit phases: setup, calibrating, handshake step, waiting for search, searching, returning, rendezvous step, waiting for convergence, converging, and terminal.
- `SilentSearchInstant` is monotonic nanoseconds in signed `Int64`. `SilentSearchClock` exposes `wallNowMilliseconds`, `monotonicNow`, and `sleep(until:) async throws`; production adapts `Date`/`ContinuousClock`, tests use manual advancement.
- `SilentSearchSafetyEvent` is `trackingNormal(generation)`, `trackingLimited(generation)`, `generationChanged`, `transportFailed`, `reactiveSafetyFailed`, or `operatorStop`. `SilentSearchSafetyMonitoring.events()` returns a multi-consumer `AsyncStream`.
- `SilentSearchOpticalExchanging` exposes cancellable `present(payload:) async throws` and `scan(until:) async throws -> Data`; Retry reuses the protocol session's current outgoing bytes.
- `SilentSearchExploring.nextCandidate()` returns selected candidate or exhausted; `markVisited` and `markRejected` are explicit. `SilentSearchTargetObserving.observeNextFrame(until:)` returns pending or confirmed target.
- `SilentSearchMotion` exposes `navigate(to:policy:) async -> SilentSearchMotionResult`, `rotate(to:tolerance:) async -> SilentSearchMotionResult`, `stop() async`, current mission pose, and current mission path. Policy is sector-constrained or unrestricted convergence.
- Readiness is a typed snapshot; calibration supplies progress/accepted-frame events; telemetry is `record(event:fields:)`.
- Coordinator owns one mission task and one safety-listener task; safety cancellation awaits motion stop before terminal or recovery transitions. Coordinator never parses failure strings.
- Only `transition(to:)` mutates phase and emits transition telemetry.

- [ ] Write failing legal/illegal transition, readiness, calibration, Stop, abort, reset, and stable-terminal tests.
- [ ] Implement model values and fakes with deterministic fake-clock waiters.
- [ ] Implement setup/calibration and central terminal cleanup only; leave later phases unreachable until their tasks.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SilentSearchCoordinatorTests
```

---

### Task 11: Drive handshake, timeout, and tracking safety through the coordinator

**Files:**
- Modify: `SilentSearchCoordinator.swift`
- Modify: `SilentSearchCoordinatorTests.swift`

- [ ] Write both-role handshake tests using the real codec/session and a fake optical relay.
- [ ] Test 30-second optical timeout, byte-identical Retry, Abort, clock mismatch, late acknowledgement, and acknowledged start.
- [ ] Test limited tracking cancels and awaits stop immediately, replans on recovery within five seconds, and invalidates calibration after five seconds.
- [ ] Test AR generation change always invalidates calibration and stops.
- [ ] Implement the phase logic without real sleeps or direct `Date()` calls.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/OpticalProtocolSessionTests \
  -only-testing:PhroverKitTests/SilentSearchCoordinatorTests
```

---

### Task 12: Run bounded search and timed return

**Files:**
- Modify: `SilentSearchCoordinator.swift`
- Modify: `SilentSearchCoordinatorTests.swift`

- [ ] Write initial settled scan, frontier selection, arrival/settle/scan, visited marking, target find, candidate no-path, terminal safety failure, exhaustion, and deadline tests.
- [ ] Require the same sector policy for explorer planning, search navigation, and return navigation to the role staging pose.
- [ ] Add a cross-sector target test proving search/return remain constrained while acknowledged convergence explicitly uses unrestricted planning with all normal safety gates.
- [ ] On deadline, cancel/await stop before navigating to the role staging pose.
- [ ] Distinguish intentional rendezvous waiting from stalled/failed motion in state.
- [ ] Fail rather than selecting an alternate unsafe staging pose.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/SectorExplorerTests \
  -only-testing:PhroverKitTests/TargetTrackerTests \
  -only-testing:PhroverKitTests/SilentSearchCoordinatorTests
```

---

### Task 13: Complete rendezvous, convergence, and two-rover simulation

**Files:**
- Modify: `SilentSearchCoordinator.swift`
- Modify: `SilentSearchCoordinatorTests.swift`
- Add: `swift/Tests/PhroverKitTests/TwoRoverSilentSearchSimulationTests.swift`

- [ ] Test role-ordered statuses, sole A/B finder, dual median, conflict, neither found, decision chaining, convergence commit/ack, and 60-second missing-partner timeout.
- [ ] Test fixed rendezvous scan headings and fixed west/east stand-off poses.
- [ ] Test that unsafe stand-off fails and no alternate is searched.
- [ ] Build a two-coordinator harness with shared fake clock and in-memory optical relay.
- [ ] Cover complete success, no target, deadline, exhaustion, duplicate/out-of-order messages, each missing protocol message, reset, and safety stop.
- [ ] Assert neither rover reports `FOUND` before its own position and heading tolerances pass.
- [ ] Verify:

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/SilentSearchCoordinatorTests \
  -only-testing:PhroverKitTests/TwoRoverSilentSearchSimulationTests
```

---

### Task 14: Publish coherent AR frame snapshots and device calibration/target adapters

**Files:**
- Modify: `swift/Sources/PhroverKit/Perception/ARSessionManager.swift`
- Modify: `swift/Sources/PhroverKit/Perception/Detector.swift`
- Add: `swift/Sources/PhroverKit/SilentSearch/Device/ARSharedMissionFrameCalibrator.swift`
- Add: `swift/Sources/PhroverKit/SilentSearch/Device/AROpticalExchangeService.swift`
- Add: `swift/Sources/PhroverKit/SilentSearch/Device/ARRoverTargetObservationSource.swift`
- Add: `swift/Tests/PhroverKitTests/ARSessionManagerTests.swift`
- Add: `swift/Tests/PhroverKitTests/ARSharedMissionFrameCalibratorTests.swift`
- Add: `swift/Tests/PhroverKitTests/AROpticalExchangeServiceTests.swift`
- Add: `swift/Tests/PhroverKitTests/ARRoverTargetObservationSourceTests.swift`
- Modify: `swift/Tests/PhroverKitTests/DetectorTests.swift`

**Interfaces:**
- `ARFrameID` is `(generation: UInt64, sequence: UInt64)`. Increment generation before each manager-owned reset, reset sequence to zero, then increment sequence exactly once per delegate frame.
- `ARFrameSnapshot` atomically carries ID, AR timestamp, image, camera transform/intrinsics/resolution, current depth, pose, and domain tracking quality.
- `ARSessionManager` provides independent streams per subscriber and removes continuations on termination. Snapshot streams use `.bufferingNewest(1)`; lifecycle event streams use unbounded buffering so reset/interruption/failure events are never coalesced.
- Preserve existing `latestPixelBuffer`, `latestCamera`, `latestDepthMap`, pose, and tracking properties for compatibility, but derive them from snapshot ingestion and clear depth when the current frame has none. Migrate new Silent Search consumers only.
- Frame-bound grounding accepts a snapshot; adapters never combine independent `latest*` values.
- Detector exposes supported canonical labels from `MLModel.modelDescription.classLabels` and a frame-preserving detection seam.

- [ ] Write primitive snapshot/reset/event tests without constructing `ARFrame`.
- [ ] Implement manager ingestion helpers, then delegate adaptation.
- [ ] Write synthetic oriented-corner + depth grounding tests and implement calibration adapter.
- [ ] Write QR frame-identity tests and implement live optical adapter.
- [ ] Write best-detection/same-frame grounding tests and implement target adapter.
- [ ] Treat real marker orientation and depth alignment as device gates, not simulator-proven claims.
- [ ] Verify all focused adapter tests.

---

### Task 15: Add awaited navigation and device readiness adapters

**Files:**
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift`
- Modify: `swift/Sources/PhroverKit/RoverSDK/RoverControl.swift`
- Add: `swift/Sources/PhroverKit/SilentSearch/Device/NavigationSilentSearchMotion.swift`
- Add: `swift/Sources/PhroverKit/SilentSearch/Device/SilentSearchDeviceEnvironment.swift`
- Add: `swift/Tests/PhroverKitTests/NavigationSilentSearchMotionTests.swift`
- Add: `swift/Tests/PhroverKitTests/SilentSearchDeviceEnvironmentTests.swift`

**Interfaces:**
- Consume Task 7's `navigateAndWait` and `cancelAndWait`; do not add a second navigation completion API.
- Add adapter-level final heading rotation and tolerance verification using Task 7's typed `rotateAndWait(by:)` plus current pose observations.
- Motion adapter converts mission poses to local poses, accepts an explicit `.sectorConstrained(SearchSector)` or `.unrestrictedConvergence` policy per request, awaits completion, performs heading correction, and verifies both tolerances.
- `RoverControl.probeLink() async throws` re-sends the existing non-motion feedback-flow-enable command (`T=131, cmd=1`), requires HTTP 2xx within the existing communication timeout/retry policy, and refreshes `lastAckAt`; it never sends a speed opcode.
- Environment readiness requires normal tracking, LiDAR depth/reconstruction support, valid generation, loaded detector with labels, and a successful probe no more than two seconds old.

- [ ] Write awaited-stop ordering and typed-result tests.
- [ ] Write final pose/heading success and unsafe/path/transport/tracking failures.
- [ ] Write readiness gating for every missing capability.
- [ ] Implement adapters without direct wheel sends.
- [ ] Verify focused and existing navigation/control tests.

---

### Task 16: Add the complete operator tab and functional map

**Files:**
- Add: `examples/PhroverOperator/PhroverOperator/App/SilentSearchViewModel.swift`
- Add: `examples/PhroverOperator/PhroverOperator/Views/SilentSearchView.swift`
- Add: `examples/PhroverOperator/PhroverOperator/Views/SilentSearchMapView.swift`
- Modify: `examples/PhroverOperator/PhroverOperator/App/PhroverOperatorApp.swift`
- Modify: `examples/PhroverOperator/PhroverOperator.xcodeproj/project.pbxproj`
- Add: `examples/PhroverOperator/PhroverOperatorUITests/SilentSearchUITests.swift`
- Modify: `examples/PhroverOperator/PhroverOperatorUITests/PhroverOperatorUITests.swift` to share launch helpers.

**Interfaces:**
- UI tests launch with arguments `-ui-testing -silent-search-scenario <name>`. Supported names are `setup-not-ready`, `calibrating`, `display-offer`, `scan-timeout`, `searching`, `returning`, `intentional-wait`, `converging`, `not-found`, `found`, and `navigation-failure`.
- `PhroverOperatorApp` parses those arguments before constructing production dependencies. In UI-test mode it builds a `ScriptedSilentSearchViewModel`, skips `CloudSession`, does not start ARKit or create a live `RoverControl`, and opens the tab container directly.
- Stable accessibility IDs cover the tab, Start, Stop, Retry, Abort, QR, scanner, map, terminal success/not-found, and exact failure reason.
- Map draws mission marker/north, sectors/center band, rover, frontiers by status, rendezvous points, transformed path, target, and local stand-off; it does not draw full meshes.

- [ ] Write UI tests first for readiness, calibration gating, QR/scanner timeout, Retry/Abort, persistent Stop in every moving phase, intentional wait, failures, not-found, and found.
- [ ] Implement pure map fitting/transform tests, then `Canvas` rendering.
- [ ] Implement phase screens and view model with no mission logic in views.
- [ ] Compose one live detector/adapters/coordinator in the app and add the fourth tab only now.
- [ ] Add all app files to the explicit Xcode project sources.
- [ ] Verify:

```bash
xcodebuild test   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "id=$(scripts/test-swift-sdk.sh --print-udid)"   -only-testing:PhroverOperatorUITests/SilentSearchUITests

xcodebuild build   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "id=$(scripts/test-swift-sdk.sh --print-udid)"
```

---

### Task 17: Complete telemetry, regressions, and device acceptance material

**Files:**
- Modify Silent Search components to use `SilentSearchEventSink`
- Add: `docs/silent-search-device-acceptance.md`
- Add: `scripts/verify-silent-search-logs.sh`
- Add: `scripts/tests/test-verify-silent-search-logs.sh`
- Add: `scripts/tests/fixtures/silent-search/{success-a.log,success-b.log,missing-ack.log,prohibited-payload.log}`
- Modify: `.github/workflows/swift.yml` to run the log-checker fixture test and one focused Silent Search UI scenario

- [ ] Assert structured events for calibration, protocol kind/sequence/rejection, frontier decisions, target evidence, deadline, rendezvous, convergence, tracking, and terminal result.
- [ ] Assert logs exclude complete payloads and image/frame contents.
- [ ] Add `scripts/verify-silent-search-logs.sh ROVER_A_LOG ROVER_B_LOG`. Exit 0 and print `silent-search logs valid` only for matching mission/marker, complementary roles, complete protocol order, target evidence, convergence, and terminal results. Exit 2 for usage/read errors and 1 for validation/prohibited cloud/team-radio/image/full-payload fields. Test every exit path with committed sanitized fixtures.
- [ ] Write the two-device matrix from the design: no target, deadline, each sole finder, matching/conflicting dual reports, withheld acknowledgements, missing partner, tracking recovery/failure, reset, unsafe poses, and Stop in every moving phase.
- [ ] Run the complete automated gate:

```bash
scripts/tests/test-test-swift-sdk.sh
scripts/tests/test-verify-silent-search-logs.sh
swift build --target RoverNav
scripts/test-swift-sdk.sh
xcodebuild test   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "id=$(scripts/test-swift-sdk.sh --print-udid)"   -only-testing:PhroverOperatorUITests
xcodebuild build   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination 'generic/platform=iOS'
git diff --check
git status --short
```

Expected: all regular package and UI tests pass, generic device build succeeds, and only intentional task files are changed.

---

### Task 18: Run the physical two-rover release gate

**Files:**
- Update: `docs/silent-search-device-acceptance.md` with results and sanitized log paths

- [ ] Build one commit and install the same app on both LiDAR-capable iPhones.
- [ ] Connect each phone only to its own rover AP; disable Bluetooth, VPN, hotspot, and cloud configuration; use airplane mode where rover Wi-Fi remains usable.
- [ ] Verify both phones calibrate to the same marker and agree on known shared points.
- [ ] Run the complete acceptance matrix and record each result.
- [ ] Pull `Documents/phrover-runtime.log` from both devices and run `scripts/verify-silent-search-logs.sh /path/to/rover-a.log /path/to/rover-b.log`.
- [ ] Confirm search and return paths never enter the center/opposite sector; any convergence crossing begins only after the acknowledged release. Confirm protocol order is optical-only, neither rover moves before acknowledged starts, and each displays `FOUND` only after its own tolerances pass.
- [ ] Record any hardware failures as not-device-verified; do not weaken safety or confidence thresholds to force a demo pass.
