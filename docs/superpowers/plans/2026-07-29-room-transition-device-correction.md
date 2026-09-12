# Room Transition Device Correction Implementation Plan

> **For agentic workers:** Execute task-by-task with TDD. Preserve the existing uncommitted room-topology implementation and unrelated worktree changes; never reset or broadly stage the working tree. Complete physical-device acceptance before committing the implementation.

**Goal:** Correct the physical scan heading convention and replace misleading raw object labels in the operator panel with room-transition navigation diagnostics.

**Architecture:** Convert Core Motion yaw once at the `ARSessionManager` sensor boundary; keep `NavigationController` in the existing rover-positive convention; publish diagnostic-only room-transition state from `MissionAgent`; reduce those state updates into a navigation summary consumed by the SwiftUI camera panel. Object detection remains available to object-target missions but is no longer run independently by the debug panel.

**Tech stack:** Swift 6, XCTest, ARKit, Core Motion, SwiftUI, RoverNav, PhroverKit, Xcode iOS Simulator, physical iPhone deployment.

**Approved design:** `docs/superpowers/specs/2026-07-29-room-transition-device-correction-design.md`

## Global constraints

- Preserve the existing bounded pulse count, one-time direction reversal, fresh-frame wait, AR pose rebasing, and stop-on-failure behavior.
- Convert coordinate conventions at the sensor boundary; do not scatter negations through navigation code.
- Do not use COCO object labels as doorway evidence or alter general object-target mission behavior.
- Diagnostic state must be read-only and must never send motion commands or mutate topology.
- Keep the deterministic room-transition limits: twelve total 30-degree scans and at most three candidate attempts.
- Keep physical crossing confirmation as the only operation that changes the room graph.
- Stage only task-owned files and hunks; the worktree contains pre-existing intended changes and unrelated files.

---

### Task 1: Normalize Core Motion Yaw at the Sensor Boundary

**Files:**
- Modify: `swift/Sources/PhroverKit/Perception/ARSessionManager.swift`
- Modify: `swift/Tests/PhroverKitTests/ARSessionManagerTests.swift`
- Modify: `swift/Tests/PhroverKitTests/RotationCommandTests.swift`

**Interfaces:**
- Adds an internal, pure `ARSessionManager.roverYaw(fromDeviceYaw:)` conversion seam.
- Keeps `ARSessionManager.inertialYaw` as the navigation-facing API, now expressed in rover-positive coordinates.
- Leaves `NavigationController` scan APIs and requested-angle semantics unchanged.

- [ ] **Step 1: Write failing yaw-conversion tests**

In `ARSessionManagerTests`, assert that:

- device yaw `+θ` becomes rover yaw `-θ`;
- device yaw `-θ` becomes rover yaw `+θ`;
- zero remains zero;
- values around positive and negative pi normalize into the same `[-π, π]` convention used by navigation.

In `RotationCommandTests`, feed converted physical yaw samples into `scanTurnMadeProgress` and `scanTurnReachedInertialTarget`. Prove that the device trace’s sign pattern is progress for a positive rover turn rather than opposite-direction motion.

- [ ] **Step 2: Run focused tests and observe failure**

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/ARSessionManagerTests \
  -only-testing:PhroverKitTests/RotationCommandTests
```

Expected: compile or assertion failure because the conversion seam does not exist and `inertialYaw` exposes raw device yaw.

- [ ] **Step 3: Implement the boundary conversion**

Implement one normalization helper in `ARSessionManager` and have `inertialYaw` return the converted `CMDeviceMotion.attitude.yaw`. Document the fixed iPhone mounting/camera convention next to the helper. Do not change `performScanRotation`, `scanTurnMadeProgress`, or `scanTurnReachedInertialTarget` unless a test exposes a separate bug.

- [ ] **Step 4: Verify focused safety behavior**

Run the command from Step 2. Expected: yaw conversion, scan completion, reversal, pulse limits, frame freshness, and stop-on-failure tests all pass.

---

### Task 2: Publish Explicit Room-Transition Diagnostic State

**Files:**
- Add: `swift/Sources/PhroverKit/Voice/RoomTransitionDebugState.swift`
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift`
- Modify: `swift/Tests/PhroverKitTests/RoomTransitionMissionTests.swift`
- Modify shared fakes only if necessary: `swift/Tests/PhroverKitTests/MissionAgentTests.swift`

**Interfaces:**
- Produces an `Equatable`, `Sendable` `RoomTransitionDebugState` with the approved idle, scanning, candidate, unreachable, approaching, confirming, completed, exhausted, and failed states.
- Adds an optional `roomTransitionStateDidChange` callback to `MissionAgent.init`, defaulting to no-op behavior for existing consumers.
- Retains terminal completion/exhaustion state until the next mission or stop so the operator can read it.

- [ ] **Step 1: Write failing state-sequence tests**

Extend `RoomTransitionMissionTests` to capture state callbacks and assert:

- no-candidate recovery emits scanning updates with monotonically advancing step numbers and current frontier/candidate counts, then `exhausted`;
- an unreachable candidate emits `candidateFound(...reachable: false)` followed by `unreachable` and is never approached;
- a reachable candidate emits candidate found, approaching, confirming crossing, and completed in order;
- tracking loss or motion failure emits a concise failed reason before bounded recovery or exhaustion;
- cancellation and emergency stop clear pending transition state and publish idle;
- object-target commands do not emit room-transition progress;
- callbacks do not change navigation goals, candidate ordering, or topology mutation timing.

- [ ] **Step 2: Run the focused tests and observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/RoomTransitionMissionTests
```

Expected: compile failure because the diagnostic state and callback do not exist.

- [ ] **Step 3: Add the isolated state model**

Keep the type in its own focused file. Use stable IDs already supplied by topology. Do not expose mutable topology or motion objects through the state.

- [ ] **Step 4: Emit state at existing mission boundaries**

Update `runRoomTransitionMission` and its terminal helpers without changing selection or safety authority:

1. publish scanning after each candidate refresh and before each bounded turn;
2. publish candidate reachability after ranking;
3. publish approaching only after `beginTransition` succeeds;
4. publish confirming immediately before awaited stop and graph confirmation;
5. publish completed only after `confirmTransition` succeeds;
6. publish failed/rejected outcomes before recovery;
7. publish exhausted when no safe route remains.

Add concise runtime events for exhausted and failed terminal outcomes so future device logs do not end silently.

- [ ] **Step 5: Verify mission behavior**

Run the focused command from Step 2. Expected: all existing room-transition behavior and new diagnostic sequences pass.

---

### Task 3: Replace Raw Object Output with a Navigation Summary

**Files:**
- Modify: `swift/Sources/PhroverKit/Perception/PerceptionDebugSummary.swift`
- Modify: `swift/Tests/PhroverKitTests/PerceptionDebugSummaryTests.swift`
- Modify: `examples/PhroverOperator/PhroverOperator/Views/ConversationView.swift`

**Interfaces:**
- Adds a pure navigation-debug presentation value/reducer that retains the latest opening count, candidate count, selected candidate, reachability, and transition label.
- Keeps `PerceptionDebugSummary.visibleObjects` source-compatible for external users, but the operator panel no longer calls it.
- Removes `Detector` from `LiveCameraDebugPanel`; `ARPerceptionSource` remains the detector consumer for object-target missions.

- [ ] **Step 1: Write failing summary tests**

In `PerceptionDebugSummaryTests`, assert that applying the diagnostic sequence produces stable rows for:

- tracking-independent navigation fields: openings, doorways, target, reachability, and transition;
- unknown counts before the first room scan;
- retained counts while approaching and confirming;
- unreachable, exhausted, failed, and completed terminal states;
- a new mission replacing stale terminal state.

Do not delete the existing object-summary compatibility tests.

- [ ] **Step 2: Run focused tests and observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/PerceptionDebugSummaryTests
```

Expected: compile failure because navigation summary formatting does not exist.

- [ ] **Step 3: Implement the pure presentation reducer**

Keep formatting deterministic and UI-framework independent. Represent unknown values as `—`, no selected target as `none`, and preserve the last scan counts through approach/confirmation/completion updates.

- [ ] **Step 4: Rewire `ConversationView`**

Create the detector once for the shared `ARPerceptionSource`, but do not pass it to `LiveCameraDebugPanel`. Capture `roomTransitionStateDidChange` on the main actor and reduce updates into the navigation summary. Render:

- `Tracking`
- `Clearance`
- `Openings`
- `Doorways`
- `Target`
- `Transition`

Remove the panel’s `visibleObjects` state, detector status, `Detector.detect` call, and detector-dependent task identity. Continue refreshing only the camera preview and live AR status.

- [ ] **Step 5: Verify tests and app build**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/PerceptionDebugSummaryTests
SIM_UDID="$(scripts/test-swift-sdk.sh --print-udid)"
xcodebuild build \
  -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -destination "platform=iOS Simulator,id=${SIM_UDID}"
```

Expected: summaries pass and the app builds without invoking object detection from the debug panel.

---

### Task 4: Run Regression Review and Physical-Device Acceptance

**Files:**
- Modify only where failing evidence requires it: files from Tasks 1–3
- Do not alter the approved design or broaden object-detection scope

- [ ] **Step 1: Run the complete automated gate**

```bash
scripts/test-swift-sdk.sh
swift build --target RoverNav
SIM_UDID="$(scripts/test-swift-sdk.sh --print-udid)"
xcodebuild build \
  -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -destination "platform=iOS Simulator,id=${SIM_UDID}"
git diff --check
```

Expected: all SDK suites pass, `RoverNav` builds on macOS, the app builds for an iOS simulator, and no whitespace errors remain.

- [ ] **Step 2: Review the complete correction**

Run standards and spec review against `docs/superpowers/specs/2026-07-29-room-transition-device-correction-design.md`. Confirm especially that the sign conversion exists only at the sensor boundary, diagnostics cannot control motion, and the debug panel performs no duplicate object inference.

- [ ] **Step 3: Deploy to the physical iPhone**

Build and install the app on device `C40C8EA8-E545-5B47-ADEA-CD8118AE844C` using the existing signed-device workflow, then launch bundle `bot.coy.phrover`. Do not assume simulator success proves the sensor convention.

- [ ] **Step 4: Exercise doorway traversal**

From a fresh AR session with the rover connected:

1. aim toward a doorway and say “Go to the other room”;
2. verify controlled scan steps without oscillation or repeated reversal;
3. verify the panel shows geometric openings, doorway candidates, selected target, and transition state—not `refrigerator`;
4. verify a reachable candidate is approached and the doorway plane is crossed;
5. verify the topology changes rooms only after crossing confirmation;
6. issue the command from the second room and verify reverse traversal through the known doorway.

- [ ] **Step 5: Pull and inspect the acceptance log**

```bash
DEVICE_ID="C40C8EA8-E545-5B47-ADEA-CD8118AE844C"
LOG_PATH="${TMPDIR:-/tmp}/phrover-runtime-room-transition-correction.log"

xcrun devicectl device copy from \
  --device "$DEVICE_ID" \
  --domain-type appDataContainer \
  --domain-identifier bot.coy.phrover \
  --source Documents/phrover-runtime.log \
  --destination "$LOG_PATH"

rg "nav_scan_(no_progress|direction_reversed|pulse_limit)|doorway_candidate_ranked|room_transition_(started|completed|failed|exhausted)|doorway_crossed" "$LOG_PATH"
```

Expected: scan yaw progresses in the requested rover direction; no pulse-limit event occurs in the acceptance window; start, crossing, and completion events occur in order; reverse traversal reuses the known doorway.

- [ ] **Step 6: Commit only after acceptance**

Inspect `git status`, the complete diff, staged diff, and recent log. Stage only the intended room-topology implementation plus this correction, excluding unrelated files and secrets. Commit with a concise message describing the physical root cause and corrected navigation diagnostics.
