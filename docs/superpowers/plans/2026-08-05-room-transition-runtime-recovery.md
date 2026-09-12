# Room Transition Runtime Recovery Implementation Plan

> **For agentic workers:** Execute task-by-task with TDD. Preserve all existing uncommitted work; never reset the worktree or broadly stage files. The current topology, depth-safety, mission, and UI files already contain intended uncommitted changes.

**Goal:** Correct current-pose doorway orientation, keep blind scan recovery fail-closed, and retain one operator-visible last-command result.

**Architecture:** Orient candidates in `SessionRoomTopology` from the current mission pose; return typed transition-start outcomes; make `MissionAgent` reassess any corrected goal and publish typed command status; keep depth retry and alternate-command authorization inside `NavigationController`; reduce command status into a persistent card in `ConversationView`.

**Tech stack:** Swift 6, XCTest, SwiftUI, ARKit, PhroverKit, RoverNav, Xcode iOS Simulator, physical iPhone.

**Approved design:** `docs/superpowers/specs/2026-08-05-room-transition-runtime-recovery-design.md`

## Global constraints

- Do not weaken or bypass `DepthSafetyEvaluator` or `ObstacleGuard`.
- Never send a goal calculated from a pre-correction doorway direction.
- Preserve twelve bounded scan steps, three candidate attempts, and crossing confirmation rules.
- Keep one last-command record only; do not add conversation persistence.
- Add specific telemetry before replacing the generic failure event.
- Stage only task-owned files and hunks because the worktree is heavily modified.

---

### Task 1: Make doorway orientation current-pose authoritative

**Files:**
- Modify: `swift/Sources/PhroverKit/Topology/RoomTopologyModels.swift`
- Modify: `swift/Sources/PhroverKit/Topology/SessionRoomTopology.swift`
- Modify: `swift/Tests/PhroverKitTests/SessionRoomTopologyTests.swift`

**Interfaces:**
- Change candidate refresh to accept `referencePose: Pose2D`.
- Add typed, `Equatable`, `Sendable` transition-start result and rejection reason values.
- Return a corrected candidate without opening pending-transition state when startup must flip orientation.

- [ ] **Step 1: Add failing device-geometry tests**

Create a fixture using the captured run: current pose near `(-0.01, 0.06)`, doorway plane near `(0.10, -1.66)`, and raw direction near `(1.00, 0.09)`. Make the room’s historical representative pose disagree with the current pose. Assert refresh orients from the passed current pose, flips the direction, and puts the beyond-plane goal on the opposite side.

Also assert telemetry includes raw direction, corrected direction, reference pose, signed distance, and `direction_flipped=true`.

- [ ] **Step 2: Add failing typed-start tests**

Cover started, corrected-orientation, already-active, missing-room, missing-candidate, and invalid-geometry/approach outcomes. A correction must update the stored candidate but leave `pendingTransition` nil.

- [ ] **Step 3: Run focused tests red**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SessionRoomTopologyTests
```

Expected: compile failures because current APIs accept no reference pose and return `Bool`.

- [ ] **Step 4: Implement the topology boundary**

Update protocol and concrete implementation. Extract one pure orientation helper used by refresh and transition startup. Calculate goals only after correction. Keep known-doorway reverse traversal behavior intact.

- [ ] **Step 5: Run focused tests green**

Run the command from Step 3 and confirm all existing topology crossing and reverse-doorway tests still pass.

---

### Task 2: Reassess corrected goals and publish command outcomes

**Files:**
- Add: `swift/Sources/PhroverKit/Voice/MissionCommandStatus.swift`
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift`
- Modify: `swift/Tests/PhroverKitTests/RoomTransitionMissionTests.swift`
- Modify as required: `swift/Tests/PhroverKitTests/MissionAgentTests.swift`

**Interfaces:**
- Add a typed command-status callback with recognized, working, succeeded, failed, and cancelled states.
- Add a per-command terminal-publication guard so each accepted command emits at most one terminal state.
- Consume typed transition-start outcomes rather than a generic boolean.

- [ ] **Step 1: Write failing mission geometry tests**

Assert the mission passes its current pose to candidate refresh. When startup returns a corrected candidate, assert the mission calls `assessGoal` again, starts transition only with the corrected candidate, and never sends the old goal to `navigate`.

- [ ] **Step 2: Write failing command-status tests**

Capture callback events and cover:

- successful room transition;
- candidate exhaustion;
- typed topology rejection;
- scan safety failure;
- emergency stop and task cancellation;
- missing pose and busy command;
- a representative successful and failed general mission path.

Assert recognized precedes working, exactly one terminal state is emitted, and returning phase to idle does not clear the terminal state.

- [ ] **Step 3: Run mission tests red**

```bash
scripts/test-swift-sdk.sh   -only-testing:PhroverKitTests/RoomTransitionMissionTests   -only-testing:PhroverKitTests/MissionAgentTests
```

- [ ] **Step 4: Implement typed mission flow**

Publish recognized synchronously for a nonblank utterance, then working when execution starts. Map topology outcomes explicitly. On corrected orientation, reassess before retrying startup. Replace `transition_could_not_start` with reason-specific telemetry. Route all accepted-command exits through one terminal helper.

Do not infer success from `phase == .idle`; publish at the actual completion/failure boundary.

- [ ] **Step 5: Run mission tests green**

Run the command from Step 3 and verify existing brain, cancellation, and room-transition behavior remains green.

---

### Task 3: Add bounded fresh-depth scan recovery

**Files:**
- Modify: `swift/Sources/PhroverKit/Perception/ARSessionManager.swift`
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift`
- Modify if needed: `swift/Sources/PhroverKit/Config/RoverConfig.swift`
- Modify: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift`
- Modify: `swift/Tests/PhroverKitTests/DepthSafetyEvaluatorTests.swift`

**Interfaces:**
- Expose a read-only depth-snapshot timestamp/version sufficient to await a newer raw-depth observation.
- Keep retry selection private to navigation.
- Use explicit bounded timeout/poll constants rather than unbounded waiting.

- [ ] **Step 1: Write failing safety tests**

Cover:

- initial `blind_swept_volume` stops before any movement command;
- retry waits for a strictly newer raw-depth snapshot;
- a newer clear observation authorizes the original rotation;
- if rotation remains blind, a low-speed same-direction arc is sent only when its exact observation is allowed;
- stale, missing, malformed, or blind arc depth sends no movement command;
- timeout leaves state failed with the actionable visibility message;
- obstacle, tipping, calibration, and command-link vetoes still win.

- [ ] **Step 2: Run focused safety tests red**

```bash
scripts/test-swift-sdk.sh   -only-testing:PhroverKitTests/NavigationSafetyTests   -only-testing:PhroverKitTests/DepthSafetyEvaluatorTests
```

- [ ] **Step 3: Implement bounded recovery**

On blind rotational coverage: stop, capture the current depth version, poll for a newer raw snapshot until the configured deadline, and re-evaluate the original command. If still blind, evaluate the existing depth-visible arc for the requested turn direction. Send only an `ObstacleGuard.allow` result. Otherwise remain stopped and fail with the approved operator message.

Add telemetry for retry start, fresh snapshot, original-command authorization, arc authorization/rejection, timeout, and final stop.

- [ ] **Step 4: Run focused safety tests green**

Run the Step 2 command. Confirm no existing test authorizes motion without raw depth.

---

### Task 4: Render a persistent last-command card

**Files:**
- Add: `swift/Sources/PhroverKit/Voice/LastCommandState.swift`
- Add: `swift/Tests/PhroverKitTests/LastCommandStateTests.swift`
- Modify: `examples/PhroverOperator/PhroverOperator/Views/ConversationView.swift`
- Modify only if source membership requires it: `examples/PhroverOperator/PhroverOperator.xcodeproj/project.pbxproj`

**Interfaces:**
- Add a pure `LastCommandState` reducer over `MissionCommandStatus`.
- Keep one optional current record with command text, display status, and concise message.
- `ConversationView` owns one reducer value and renders one accessible card.

- [ ] **Step 1: Write failing reducer tests**

Assert recognized → working → succeeded and recognized → working → failed. Verify idle phase changes do not affect the reducer, cancellation persists, speech partial changes cannot erase it, and a new recognized command replaces the old record.

- [ ] **Step 2: Run reducer tests red**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/LastCommandStateTests
```

- [ ] **Step 3: Implement the reducer and card**

Wire `MissionAgent` status callbacks on the main actor. Render command text, status, and message above the microphone controls. Keep existing transient Listening/Processing and navigation-debug output. Add stable accessibility identifiers for the card fields.

- [ ] **Step 4: Verify reducer and app build**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/LastCommandStateTests
SIM_UDID="$(scripts/test-swift-sdk.sh --print-udid)"
xcodebuild build   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "platform=iOS Simulator,id=${SIM_UDID}"
```

---

### Task 5: Regression review and physical-device acceptance

**Files:**
- Modify only where failing evidence requires it: files from Tasks 1–4
- Do not broaden scope into calibration, full history, or general UI redesign

- [ ] **Step 1: Run the complete automated gate**

```bash
scripts/test-swift-sdk.sh
swift build --target RoverNav
SIM_UDID="$(scripts/test-swift-sdk.sh --print-udid)"
xcodebuild build   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "platform=iOS Simulator,id=${SIM_UDID}"
git diff --check
```

- [ ] **Step 2: Review against the approved design**

Confirm current-pose authority, no stale goal dispatch, exact-command depth authorization, bounded recovery, one terminal status per command, and one persistent card. Verify no unrelated refactor or safety relaxation entered the diff.

- [ ] **Step 3: Deploy to the physical iPhone**

Build/install through the existing signed-device workflow and launch `bot.coy.phrover` on device `C40C8EA8-E545-5B47-ADEA-CD8118AE844C`.

- [ ] **Step 4: Run both acceptance paths**

1. With a safely visible doorway, say “Go to other room”; verify the card advances to Working, corrected candidate startup succeeds, crossing completes, and the card displays Succeeded.
2. With intentionally blind scan geometry, repeat; verify no motor command is sent and the card displays the repositioning instruction.

- [ ] **Step 5: Pull and inspect the device log**

```bash
DEVICE_ID="C40C8EA8-E545-5B47-ADEA-CD8118AE844C"
LOG_PATH="${TMPDIR:-/tmp}/phrover-runtime-recovery-acceptance.log"
xcrun devicectl device copy from   --device "$DEVICE_ID"   --domain-type appDataContainer   --domain-identifier bot.coy.phrover   --source Documents/phrover-runtime.log   --destination "$LOG_PATH"
rg "speech_capture_completed|voice_command_received|doorway_frontier_admitted|room_transition_(started|completed|failed|exhausted)|nav_scan_depth|nav_safety_stop" "$LOG_PATH"
```

Expected: corrected orientation telemetry and either a confirmed transition or a fail-closed visibility stop; no generic `transition_could_not_start`; no movement under unavailable depth.

- [ ] **Step 6: Commit only intended implementation after acceptance**

Inspect status, complete diff, staged diff, and recent history. Stage only task-owned hunks and files. The commit message must state the physical root cause: stale doorway-side orientation plus blind rotational depth coverage.
