# Gravity-Aware Depth Safety Implementation Plan

> **For agentic workers:** Execute task-by-task with TDD. Preserve all existing uncommitted room-transition work and unrelated worktree changes; never reset or broadly stage the tree. Do not commit implementation changes until low-crossbar and bidirectional physical acceptance passes.

**Goal:** Replace the fixed image-space clearance percentile with gravity-aware rover-space collision geometry so the chassis stops before low crossbars, while making unavailable depth and failed replanning fail closed.

**Architecture:** Add a pure two-stage `DepthSafetyEvaluator`. Frame ingestion unprojects raw depth into a calibrated rover-space hazard snapshot; command evaluation intersects that snapshot with the intended differential-drive swept volume and computes clear, caution, stop, or unavailable. `ARSessionManager` owns ARKit conversion and the latest immutable snapshot. `NavigationController` computes a candidate command, asks the evaluator for a command-specific observation, and lets `ObstacleGuard` veto or limit it. Scene mesh remains secondary planning evidence, with corrected floor estimation and transactional replanning.

**Tech stack:** Swift 6, XCTest, CoreVideo, simd, ARKit, RoverNav, PhroverKit, iOS Simulator, and physical iPhone LiDAR deployment.

**Approved design:** `docs/superpowers/specs/2026-07-30-gravity-aware-depth-safety-design.md`

## Global constraints

- Safety geometry is defined in rover coordinates, never as a fixed image rectangle.
- Raw `sceneDepth` is required to authorize translational motion; smoothed depth cannot turn unavailable or stop into clear.
- The phone-to-rover transform is explicit and validated. Never guess missing calibration.
- A blind, stale, malformed, or unsupported swept volume fails closed.
- Connected low-profile obstacles must survive noise rejection; isolated pixels must not.
- Route planning cannot override the depth safety veto.
- Failed replanning clears the old path and stops.
- Preserve current doorway ranking, crossing confirmation, voice behavior, and unrelated navigation behavior.
- Stage only task-owned files and hunks.

---

### Task 1: Define calibrated safety inputs and outputs

**Files:**
- Add: `swift/Sources/PhroverKit/Perception/DepthSafetyModels.swift`
- Add: `swift/Tests/PhroverKitTests/DepthSafetyModelsTests.swift`
- Modify: `swift/Sources/PhroverKit/Config/RoverConfig.swift`

**Interfaces:**

Define small value types for:

- `CameraMountCalibration`: camera height, longitudinal/lateral offset, heading alignment, and validation result;
- `RoverCollisionGeometry`: chassis length, width, collision-height band, and margins;
- `DepthSafetySnapshot`: timestamp, raw-depth availability, camera frustum metadata, rover-space occupied cells, and valid-support coverage;
- `DepthSafetyObservation`: `clear`, `caution`, `stop`, or `unavailable`, with exact reason, clearance, support count, age, stopping distance, and motion class.

Keep ARKit classes out of these types. Invalid and non-finite dimensions must be represented as explicit validation failures.

- [ ] **Step 1: Write failing model and calibration tests**

Cover valid 0.40–0.70 m camera heights, non-finite values, impossible offsets, non-positive chassis dimensions, equality needed by tests, and stable diagnostic reason strings.

- [ ] **Step 2: Run the focused tests and observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/DepthSafetyModelsTests
```

Expected: compile failure because the models do not exist.

- [ ] **Step 3: Implement the minimal value types and configuration**

Put physical defaults and timing margins in `RoverConfig`; do not spread literals through sensor or navigation code. Keep validation pure and deterministic.

- [ ] **Step 4: Re-run the focused tests**

Expected: all model tests pass.

---

### Task 2: Build the pure gravity-aware depth evaluator

**Files:**
- Add: `swift/Sources/PhroverKit/Perception/DepthSafetyEvaluator.swift`
- Add: `swift/Tests/PhroverKitTests/DepthSafetyEvaluatorTests.swift`
- Reuse synthetic buffer helpers from or refactor: `swift/Tests/PhroverKitTests/UnprojectionTests.swift`

**Interface:**

Implement two pure operations:

1. Ingest a raw depth map with image size, intrinsics, camera transform, timestamp, calibration, and collision geometry into a rover-space `DepthSafetySnapshot`.
2. Evaluate a snapshot against an intended `WheelCommand`, current time, and braking parameters.

Unproject depth pixels with the frame intrinsics. Convert camera coordinates through the world/gravity frame and calibrated camera-to-rover transform. Voxelize points in the collision-height band. Use connected spatial support rather than a global percentile.

Construct straight, curved, reverse, and rotational swept footprints from differential-drive kinematics. Verify that the safety-critical swept volume is inside the raw-depth frustum and has valid support before returning clear.

- [ ] **Step 1: Write failing low-crossbar fixtures**

Generate synthetic depth maps for:

- a crossbar below camera height at 0.40, 0.55, and 0.70 m camera heights;
- pitch angles of -30, 0, and +30 degrees;
- centered, left-edge, and right-edge obstacles;
- crossbars occupying a narrow but connected set of samples;
- a clear floor and an overhead-only object.

Each visible crossbar must yield caution or stop. A crossbar outside observable coverage must yield unavailable, never clear.

- [ ] **Step 2: Write failing noise and motion-envelope tests**

Cover isolated near outliers, invalid depth, stale timestamps, straight motion, unequal-wheel arcs, reverse coverage, in-place rotation, speed-dependent stopping distance, and nearer raw evidence winning over any farther smoothed diagnostic value.

- [ ] **Step 3: Run the evaluator tests and observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/DepthSafetyEvaluatorTests
```

- [ ] **Step 4: Implement unprojection, voxel support, coverage, and swept-volume evaluation**

Keep frame ingestion and command evaluation separate so AR frame cadence does not depend on navigation cadence. Avoid ARKit object construction in tests.

- [ ] **Step 5: Re-run Tasks 1–2 tests**

```bash
scripts/test-swift-sdk.sh   -only-testing:PhroverKitTests/DepthSafetyModelsTests   -only-testing:PhroverKitTests/DepthSafetyEvaluatorTests   -only-testing:PhroverKitTests/UnprojectionTests
```

Expected: deterministic pass in seconds.

---

### Task 3: Integrate raw depth at the AR boundary

**Files:**
- Modify: `swift/Sources/PhroverKit/Perception/ARSessionManager.swift`
- Modify: `swift/Tests/PhroverKitTests/ARSessionManagerTests.swift`
- Modify if exposed in diagnostics: `swift/Sources/PhroverKit/Perception/PerceptionDebugSummary.swift`
- Modify corresponding tests: `swift/Tests/PhroverKitTests/PerceptionDebugSummaryTests.swift`

**Behavior:**

On each accepted AR frame, ingest `sceneDepth` into the evaluator with frame intrinsics, camera transform, and timestamp. Publish the latest immutable `DepthSafetySnapshot`. Keep smoothed depth only as secondary diagnostics. Reset, interruption, session failure, and generation changes synchronously invalidate the snapshot.

Expose a test seam accepting synthetic depth input and pure frame metadata; do not attempt to instantiate `ARDepthData` or `ARCamera` in tests.

- [ ] **Step 1: Write failing AR integration tests**

Assert that raw depth produces a generation-bound snapshot, smoothed-only depth cannot authorize translation, newer frames replace older ones, and all AR invalidation paths clear safety state.

- [ ] **Step 2: Run focused tests and observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/ARSessionManagerTests
```

- [ ] **Step 3: Wire frame ingestion and lifecycle invalidation**

Remove navigation's dependency on the old scalar `forwardClearance` only after all consumers migrate. Keep temporary compatibility private if needed during this task; no final motor path may use the fixed ROI calculation.

- [ ] **Step 4: Add bounded telemetry**

Log state changes and periodic summaries, not every depth pixel. Include raw availability, age, occupied cells, connected support, frustum coverage, calibration status, and nearest hazard.

- [ ] **Step 5: Re-run perception tests**

```bash
scripts/test-swift-sdk.sh   -only-testing:PhroverKitTests/ARSessionManagerTests   -only-testing:PhroverKitTests/DepthSafetyEvaluatorTests   -only-testing:PhroverKitTests/PerceptionDebugSummaryTests
```

---

### Task 4: Make command dispatch fail closed

**Files:**
- Modify: `swift/Sources/PhroverKit/Nav/ObstacleGuard.swift`
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift`
- Modify: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift`
- Modify: `swift/Sources/PhroverKit/Config/RoverConfig.swift`

**Behavior:**

Reorder each navigation tick to compute the candidate wheel command before the final safety gate. Evaluate that command against the current snapshot. Apply a caution speed cap, or send emergency stop for stop/unavailable. No translational command may be sent first and checked afterward.

Preserve communications, tipping, tracking, visual-target, and progress-watchdog protections. Visual-target approach cannot relax unavailable depth or a rover-space stop result.

- [ ] **Step 1: Write failing guard decision tests**

Cover clear, caution, stop, unavailable, stale, invalid calibration, and rotation decisions. Assert that caution returns an explicit capped command rather than a boolean.

- [ ] **Step 2: Write failing navigation integration tests**

Using `TestURLProtocol`, assert:

- unavailable or stale depth sends only emergency stop;
- low-crossbar stop sends no speed command;
- caution sends a capped speed command;
- clear sends the original command;
- visual-target approach never overrides stop/unavailable;
- safety observation is evaluated before every transport send.

- [ ] **Step 3: Run focused tests and observe failure**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/NavigationSafetyTests
```

- [ ] **Step 4: Implement command-aware guarding and loop reordering**

Keep the decision logic pure where possible. Every rejected command must stop transport before publishing terminal state.

- [ ] **Step 5: Add collision-trace incident regression**

Add a sanitized minimal event sequence derived from the physical trace: forward command with falsely distant legacy clearance, no progress, then near hazard. Assert diagnostics classify it as a probable sensing/contact miss rather than an ordinary route failure. Do not copy device identifiers, network data, or the full runtime log into the repository.

- [ ] **Step 6: Re-run navigation and visual-target tests**

```bash
scripts/test-swift-sdk.sh   -only-testing:PhroverKitTests/NavigationSafetyTests   -only-testing:PhroverKitTests/MissionAgentTests
```

---

### Task 5: Correct mesh floor classification and transactional replanning

**Files:**
- Modify: `swift/Sources/PhroverKit/Nav/CostmapBuilder.swift`
- Add: `swift/Tests/PhroverKitTests/CostmapBuilderTests.swift`
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift`
- Modify: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift`

**Behavior:**

Extract pure mesh-point classification from AR mesh iteration. Estimate floor from transformed vertex evidence or a validated horizontal floor plane; never from `anchor.transform.y - 2`. Classify furniture relative to that floor and preserve existing map inflation.

Make replanning transactional. A successful plan replaces `path`. A failed periodic plan clears `path`, stops motors, and reports planning failure.

- [ ] **Step 1: Write failing pure floor-classification tests**

Use transformed point fixtures representing floor, a low crossbar, tabletop, wall, and ceiling. Assert the crossbar and furniture remain obstacles while floor and ceiling are excluded.

- [ ] **Step 2: Write failing replanning tests**

Seed a valid path, introduce an impassable updated costmap, and assert failed replanning clears the path and stops rather than retaining old waypoints.

- [ ] **Step 3: Run focused tests and observe failure**

```bash
scripts/test-swift-sdk.sh   -only-testing:PhroverKitTests/CostmapBuilderTests   -only-testing:PhroverKitTests/NavigationSafetyTests
```

- [ ] **Step 4: Implement the smallest classifier and transactional plan update**

Do not redesign frontier extraction or room topology. Emit plan-success/failure and obstacle-evidence telemetry at the decision boundary.

- [ ] **Step 5: Re-run focused costmap, frontier, and navigation tests**

```bash
scripts/test-swift-sdk.sh   -only-testing:PhroverKitTests/CostmapBuilderTests   -only-testing:PhroverKitTests/NavigationSafetyTests   -only-testing:RoverNavTests/CostmapTests   -only-testing:RoverNavTests/FrontierFinderTests
```

---

### Task 6: Remove the legacy ROI path and run automated gates

**Files:**
- Modify only files from Tasks 1–5 where verification requires it.
- Remove obsolete tests from: `swift/Tests/PhroverKitTests/UnprojectionTests.swift`

- [ ] **Step 1: Remove legacy production usage**

Search for `forwardClearance`, `forwardClearance(fromDepthMap:)`, and fixed ROI bounds. Remove the scalar motor-safety path and tests that encode its tenth-percentile behavior. Retain generic unprojection helpers only if the new evaluator uses them.

- [ ] **Step 2: Run all automated gates**

```bash
scripts/test-swift-sdk.sh
swift build --target RoverNav
SIM_UDID="$(scripts/test-swift-sdk.sh --print-udid)"
xcodebuild build   -project examples/PhroverOperator/PhroverOperator.xcodeproj   -scheme PhroverOperator   -destination "platform=iOS Simulator,id=${SIM_UDID}"
git diff --check
```

Expected: all Swift tests pass, `RoverNav` builds, the simulator app builds, and no whitespace errors remain.

- [ ] **Step 3: Review against the approved spec**

Confirm:

- low crossbars are represented in rover space;
- fixed image ROI and global percentile no longer authorize motion;
- raw depth, calibration, freshness, and frustum coverage all fail closed;
- curved commands use swept chassis geometry;
- visual-target behavior cannot bypass safety;
- mesh floor estimation uses geometry;
- failed replanning never retains the old path;
- debug instrumentation is bounded and no temporary `[DEBUG-...]` logs remain;
- unrelated worktree changes are untouched.

---

### Task 7: Calibrate and complete physical acceptance

**Device:** `C40C8EA8-E545-5B47-ADEA-CD8118AE844C`  
**Bundle:** `bot.coy.phrover`

- [ ] **Step 1: Measure and record the rigid mount**

Measure camera height, longitudinal/lateral offset, and heading alignment. Put the approved values in app configuration. Confirm the mount does not shift under acceleration or turning.

- [ ] **Step 2: Build, sign, install, and launch**

Use the existing physical-device provisioning workflow. Confirm the launched process remains alive and reports valid calibration plus fresh raw-depth coverage before enabling translation.

- [ ] **Step 3: Run low-crossbar trials**

Place the same low crossbar ahead of the rover at center, left, and right lateral positions. Repeat with representative level, upward, and downward fixed pitches within the supported range. Begin at conservative speed.

Expected: every visible obstacle produces caution then stop without contact. Any blind swept volume produces unavailable and no translational command. Record obstacle distance, trigger distance, final stand-off, speed, depth age, and support count.

- [ ] **Step 4: Run clear-floor and depth-loss trials**

Verify normal clear-floor travel without persistent false stops. Then block or disable depth before requesting translation.

Expected: clear floor permits bounded motion; depth loss sends stop and prevents speed commands.

- [ ] **Step 5: Run room A to room B and room B to room A**

From a fresh room-mapping session, execute “Go to the other room” in both directions.

Expected: both confirmed room transitions complete without furniture contact, while all depth-safety and existing doorway telemetry remain healthy.

- [ ] **Step 6: Pull and inspect the runtime log**

```bash
DEVICE_ID="C40C8EA8-E545-5B47-ADEA-CD8118AE844C"
LOG_PATH="${TMPDIR:-/tmp}/phrover-runtime-depth-safety-acceptance.log"

xcrun devicectl device copy from   --device "$DEVICE_ID"   --domain-type appDataContainer   --domain-identifier bot.coy.phrover   --source Documents/phrover-runtime.log   --destination "$LOG_PATH"

rg "depth_safety|nav_safety_stop|nav_replan|room_transition|doorway_crossed" "$LOG_PATH"
```

Verify positive stop margins for every obstacle trial, explicit fail-closed depth loss, no probable-contact incident, and two confirmed crossings in order.

- [ ] **Step 7: Re-run the original feedback loop**

Repeat the low-crossbar scenario that produced the physical collision. The rover must stop because of depth safety before contact; it must not reach the no-progress watchdog first.

- [ ] **Step 8: Commit implementation only after acceptance**

Inspect `git status`, complete diff, staged diff, and recent commits. Stage only intended depth-safety implementation files and approved related room-transition changes. Exclude generated artifacts, memories, unrelated files, and secrets. The commit message should record the root cause: a fixed image-space ROI looked over the low crossbar, and safety now evaluates gravity-aware rover-space swept volume.
