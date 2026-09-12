# Room Transition Gravity-Heading Correction Implementation Plan

> **For agentic workers:** Execute task-by-task with TDD. Preserve the existing uncommitted room-topology implementation and all unrelated worktree changes; never reset or broadly stage the working tree. Do not commit the implementation until forward and reverse physical-device acceptance passes.

**Goal:** Replace unstable Euler-yaw scan completion with gravity-axis relative rotation, make commanded scans reliable, admit safely traversable hallway-width openings, and persist decision-complete doorway telemetry.

**Architecture:** Add a pure `RelativeHeadingTracker` that integrates Core Motion angular velocity projected onto gravity. `ARSessionManager` owns sensor sampling and exposes zero-based scan measurements. `NavigationController` performs conservative stop-pulse-stop-settle turns and validates settled AR movement against inertial rotation. `SessionRoomTopology` admits all openings above the rover-safe minimum and logs every admission or rejection. `MissionAgent` logs each scan and candidate-selection decision while retaining existing crossing authority.

**Tech stack:** Swift 6, XCTest, Core Motion, ARKit, RoverNav, PhroverKit, Xcode simulator and physical iPhone deployment.

**Approved design:** `docs/superpowers/specs/2026-07-29-room-transition-gravity-heading-correction-design.md`

## Global constraints

- Reliability takes priority over scan speed.
- Do not consume `CMAttitude.yaw` for scan completion and do not retain it as a fallback.
- Keep all scan motion bounded and stop motors before every terminal outcome.
- Remove automatic scan-direction reversal on contradictory heading data.
- Keep twelve total 30-degree room-transition scan steps and at most three candidate attempts.
- Keep the existing rover-safe minimum opening width and crossing-clearance rules.
- Remove only the fixed maximum opening width; do not weaken reachability or crossing confirmation.
- Keep cloud doorway evidence as an optional ranking boost, never an admission requirement.
- Emit telemetry at the component that makes each decision.
- Stage only task-owned files and hunks. The worktree contains intended uncommitted topology work and unrelated changes.

---

### Task 1: Build the Gravity-Axis Relative Heading Tracker

**Files:**
- Add: `swift/Sources/PhroverKit/Perception/RelativeHeadingTracker.swift`
- Add: `swift/Tests/PhroverKitTests/RelativeHeadingTrackerTests.swift`
- Modify if explicit constants belong centrally: `swift/Sources/PhroverKit/Config/RoverConfig.swift`

**Interface:**

Create an internal, pure tracker with value-type samples so tests require no live `CMDeviceMotion` instance. The sample contains timestamp, rotation-rate vector, and gravity vector. The tracker exposes reset, accumulated rotation, latest sample age/freshness, and a reliability result with explicit reasons.

Use `dot(rotationRate, normalizedGravity)` for signed rover-relative angular rate and trapezoidal integration between accepted samples. Reset establishes zero accumulated angle and clears prior integration state.

- [ ] **Step 1: Write failing tracker tests**

Cover:

- positive and negative rotation around gravity;
- equivalent results for upright, landscape, tilted, and near-vertical mount fixtures;
- reset producing independent zero-based scan steps;
- 1/60-second sample sequences and trapezoidal integration;
- non-monotonic timestamps;
- gaps above 0.10 seconds;
- gravity magnitude outside 0.8–1.2 g;
- non-finite input;
- freshness becoming stale after 0.15 seconds.

- [ ] **Step 2: Run the focused tests and observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/RelativeHeadingTrackerTests
```

Expected: compile failure because the tracker does not exist.

- [ ] **Step 3: Implement the smallest pure tracker**

Keep Core Motion framework types outside the tracker. Model reliability as data rather than booleans so navigation and telemetry receive a stable reason. Do not add global heading, magnetometer correction, or mount-specific transforms.

- [ ] **Step 4: Re-run focused tests**

Expected: all tracker fixtures pass deterministically without device hardware.

---

### Task 2: Integrate Relative Heading at the AR Sensor Boundary

**Files:**
- Modify: `swift/Sources/PhroverKit/Perception/ARSessionManager.swift`
- Modify: `swift/Tests/PhroverKitTests/ARSessionManagerTests.swift`
- Modify: `swift/Sources/PhroverKit/Config/RoverConfig.swift`

**Interface:**

Replace navigation-facing `inertialYaw` with explicit scan-measurement operations, such as reset/start measurement and read current measurement. `ARSessionManager` converts live `CMDeviceMotion` values into tracker samples on the main actor. Configure device-motion updates at 60 Hz and preserve sensor lifecycle across AR resets without carrying scan accumulation into a new generation.

- [ ] **Step 1: Write failing integration tests**

Assert that:

- injected samples reach the tracker in device coordinates;
- reset clears accumulated rotation;
- AR reset, interruption, and failure invalidate active scan measurement;
- stale and unreliable states propagate with reasons;
- the old `roverYaw(fromDeviceYaw:)` conversion is no longer needed by navigation.

Use a test seam that accepts tracker samples directly; do not attempt to construct private Core Motion objects.

- [ ] **Step 2: Run focused tests and observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/ARSessionManagerTests
```

- [ ] **Step 3: Wire live Core Motion updates**

Start updates with a 1/60-second interval, map timestamp/rotation rate/gravity into samples, and stop updates during pause/deinitialization as appropriate. Ensure AR session generation changes invalidate the current scan measurement.

- [ ] **Step 4: Remove raw Euler-yaw scan API usage**

Search the project for `inertialYaw`, `attitude.yaw`, and `roverYaw(fromDeviceYaw:)`. Keep no scan-completion dependency on those APIs. Delete obsolete tests only when replacement fixtures cover the same safety intent.

- [ ] **Step 5: Re-run Tasks 1–2 tests**

```bash
scripts/test-swift-sdk.sh   -only-testing:PhroverKitTests/RelativeHeadingTrackerTests   -only-testing:PhroverKitTests/ARSessionManagerTests
```

---

### Task 3: Rewrite Scan Rotation as Conservative Relative Control

**Files:**
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift`
- Modify: `swift/Sources/PhroverKit/Config/RoverConfig.swift`
- Modify: `swift/Tests/PhroverKitTests/RotationCommandTests.swift`
- Modify shared navigation fakes only if required: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift`

**Behavior:**

Each 30-degree scan begins by stopping, requiring a fresh normal AR pose, and resetting relative heading. Every iteration sends the minimum 0.02-second pulse, stops, waits 0.75 seconds, reads reliable relative rotation, and obtains a fresh AR frame. Correct-direction rotation at or beyond 23 degrees completes the step. Eight pulses remain the hard limit.

Do not escalate pulse duration and do not reverse automatically. Validate settled AR rotation against inertial rotation with a 15-degree disagreement limit. A commanded yaw delta alone is never a discontinuity; generation change, tracking loss, stale pose, unexpected translation at or above 0.10 meters, and excessive AR/inertial disagreement remain failures.

- [ ] **Step 1: Replace obsolete tests with failing approved-behavior tests**

Cover:

- target completion from accumulated relative rotation;
- correct-direction overshoot completing immediately;
- opposite-direction or unreliable samples stopping without reversal;
- every pulse retaining minimum duration;
- pulse limit stopping motors;
- expected 30–70-degree commanded AR yaw not treated as relocalization;
- translation jump, generation change, stale heading, tracking loss, and AR/inertial disagreement failing safely;
- each scan step resetting accumulation.

Remove or rewrite tests that require increasing pulse duration, one-time reversal, raw yaw conversion, or yaw-only pose rebasing.

- [ ] **Step 2: Run focused tests and observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/RotationCommandTests
```

- [ ] **Step 3: Implement conservative scan control**

Keep continuous navigation rotation unchanged. Isolate relative scan logic into focused helpers where pure tests are useful. Stop transport before publishing all failures. Log requested angle, accumulated angle, sample age/reliability, pulse number/duration, settled AR delta, disagreement, overshoot, and outcome.

- [ ] **Step 4: Delete obsolete reversal and rebasing paths**

Remove dead helpers and configuration only after references are gone. Keep fresh-frame waiting and bounded timeout behavior.

- [ ] **Step 5: Verify heading and navigation tests**

```bash
scripts/test-swift-sdk.sh   -only-testing:PhroverKitTests/RelativeHeadingTrackerTests   -only-testing:PhroverKitTests/ARSessionManagerTests   -only-testing:PhroverKitTests/RotationCommandTests   -only-testing:PhroverKitTests/NavigationSafetyTests
```

---

### Task 4: Admit Hallway Openings and Explain Candidate Filtering

**Files:**
- Modify: `swift/Sources/PhroverKit/Topology/SessionRoomTopology.swift`
- Modify: `swift/Sources/PhroverKit/Topology/RoomTopologyModels.swift` only if a typed rejection reason is needed
- Modify: `swift/Tests/PhroverKitTests/SessionRoomTopologyTests.swift`
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift`
- Modify: `swift/Tests/PhroverKitTests/RoomTransitionMissionTests.swift`

**Behavior:**

Remove the 2.0-meter maximum-width admission filter. Retain minimum safe width, finite centroid, normalizable direction, and finite beyond-plane goal requirements. Log every frontier with width, centroid, cell count, direction availability, admission result, and one explicit rejection reason: `below_safe_width`, `missing_direction`, `invalid_centroid`, `invalid_direction`, or `invalid_beyond_plane_goal`.

At every room-transition loop iteration, log mission ID, generation, scan step, opening count, candidate count, reachability assessments, selected candidate, or the reason scanning continues.

- [ ] **Step 1: Write failing topology tests**

Assert that:

- conventional doorway widths remain admitted;
- 2.1-meter and wider hallway frontiers are admitted;
- below-minimum widths remain rejected;
- non-finite centroid, missing/invalid direction, and invalid beyond-plane geometry are rejected with exact telemetry;
- wide openings participate in deterministic ranking without bypassing reachability.

Capture `SessionRoomTopology`’s telemetry sink directly.

- [ ] **Step 2: Run focused topology tests and observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/SessionRoomTopologyTests
```

- [ ] **Step 3: Implement candidate admission and telemetry**

Keep admission and rejection-reason derivation in a small pure helper so each frontier receives exactly one outcome. Do not silently drop malformed frontiers.

- [ ] **Step 4: Write failing mission telemetry tests**

Capture runtime telemetry through an injectable seam where possible. Assert complete no-opening, rejected-opening, unreachable-candidate, selected-candidate, scan-failure, exhausted, and completed sequences without changing mission decisions.

- [ ] **Step 5: Implement mission-level decision telemetry**

Add telemetry beside existing scan, assessment, selection, approach, crossing, and terminal boundaries. Do not let diagnostics control motion or topology.

- [ ] **Step 6: Verify topology and mission tests**

```bash
scripts/test-swift-sdk.sh   -only-testing:PhroverKitTests/SessionRoomTopologyTests   -only-testing:PhroverKitTests/RoomTransitionMissionTests
```

---

### Task 5: Run Automated Gates and Review the Complete Correction

**Files:**
- Modify only files from Tasks 1–4 where failing evidence requires it.

- [ ] **Step 1: Run all automated gates**

```bash
scripts/test-swift-sdk.sh
swift build --target RoverNav
SIM_UDID="$(scripts/test-swift-sdk.sh --print-udid)"
xcodebuild build   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "platform=iOS Simulator,id=${SIM_UDID}"
git diff --check
```

Expected: complete Swift tests pass, `RoverNav` builds, the simulator app builds, and no whitespace errors remain.

- [ ] **Step 2: Review against standards and the approved spec**

Confirm especially:

- no raw attitude-yaw scan dependency remains;
- the tracker is mount independent and independently testable;
- scan failure always stops motors;
- no reversal or pulse escalation remains;
- wide openings do not bypass minimum width, reachability, or crossing confirmation;
- telemetry explains every filtering and mission decision;
- unrelated worktree changes are untouched.

---

### Task 6: Physical Forward/Reverse Acceptance

**Device:** `C40C8EA8-E545-5B47-ADEA-CD8118AE844C`  
**Bundle:** `bot.coy.phrover`

- [ ] **Step 1: Build, sign, install, and launch**

Use the existing physical-device derived-data path and automatic provisioning workflow. Confirm the launched process remains alive.

- [ ] **Step 2: Run room A to room B**

From a fresh room-mapping session, place one safe doorway or hallway entrance within the bounded scan. Say “Go to the other room” and allow the mission to finish without intervention unless safety requires it.

Expected: no heading failure, reversal, or pulse-limit exhaustion; monotonic relative heading per step; explicit frontier/candidate outcomes; selected reachable candidate; confirmed doorway-plane crossing.

- [ ] **Step 3: Run room B to room A**

Say “Go to the other room” again from room B.

Expected: the known connection is selected in reverse and crossing confirmation records room A.

- [ ] **Step 4: Pull and inspect the runtime log**

```bash
DEVICE_ID="C40C8EA8-E545-5B47-ADEA-CD8118AE844C"
LOG_PATH="${TMPDIR:-/tmp}/phrover-runtime-gravity-heading-acceptance.log"

xcrun devicectl device copy from   --device "$DEVICE_ID"   --domain-type appDataContainer   --domain-identifier bot.coy.phrover   --source Documents/phrover-runtime.log   --destination "$LOG_PATH"

rg "relative_heading|nav_scan|frontier_|doorway_candidate|room_transition|doorway_crossed" "$LOG_PATH"
```

Verify two candidate selections and two confirmed crossings in order, with no heading reliability, AR disagreement, or pulse-limit failure in the acceptance window.

- [ ] **Step 5: Commit only after acceptance**

Inspect `git status`, complete diff, staged diff, and recent log. Stage only the intended room-topology implementation and this correction, excluding unrelated files, generated artifacts, memories, and secrets. Use a concise commit message that records the physical root cause: Euler yaw was unstable in the mounted orientation, and scan completion now integrates rotation around gravity.
