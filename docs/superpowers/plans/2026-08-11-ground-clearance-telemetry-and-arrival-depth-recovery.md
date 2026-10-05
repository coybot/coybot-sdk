# Ground Clearance Telemetry and Arrival Depth Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the obsolete ground-sensitive forward-clearance signal and retry transient stale raw depth at arrival while retaining fail-closed geometry-aware obstacle safety.

**Architecture:** `DepthSafetyEvaluator` remains the depth-based motor-safety authority. `NavigationController` stops before an arrival-depth retry, waits for a newer `ARSessionManager` snapshot, then restarts the loop so every safety gate is re-evaluated. The fixed image-space percentile signal and its dead consumers are deleted.

**Tech Stack:** Swift 6, XCTest, ARKit, CoreVideo, Xcode/iOS device tooling.

## Global Constraints

- Never treat stale, missing, malformed, blind, or invalid depth as clear.
- Do not change collision geometry, swept-volume coverage, obstacle support, stopping distance, caution speed, communications, tipping, tracking, planning, or watchdog behavior.
- Retry only `stale_raw_depth` at arrival and stop transport before waiting.
- Preserve all unrelated dirty-worktree changes. Touched files already contain user work, so do not stage or commit them.
- Deploy only to iPhone 15 Pro CoreDevice `FC11C836-4978-5B20-9170-16EAD18568BE`, Xcode destination `00008130-00166C823C91001C`.

---

### Task 1: Recover transient stale depth at arrival

**Files:**
- Modify: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift:469-489`
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift:573-598`

**Interfaces:**
- Consumes: `depthSafetyObservation`, `ARSessionManager.depthSnapshotVersion`, and `waitForNewDepthSnapshot(after:deadline:operation:)`.
- Produces: `nav_arrival_depth_retry_started`, `nav_arrival_depth_fresh_snapshot`, and `nav_arrival_depth_retry_timeout` telemetry.

- [ ] **Step 1: Add the transient-stale regression**

```swift
func testReachedGoalWithStaleDepthWaitsForFreshSnapshotBeforeArrival() async {
    var depthState: DepthSafetyState = .unavailable(.staleRawDepth)
    let (navigation, ar) = makeNavigation(
        depthRecoveryTimeout: 0.5,
        depthSafetyState: { _ in depthState }
    )
    ingestTrackedPose(into: ar, yaw: 0, sequence: 1)
    ingestBlindDepth(into: ar)
    let initialDepthVersion = ar.depthSnapshotVersion

    navigation.navigate(to: Vec2(0.10, 0))
    await waitForRequestCount(2)
    try? await Task.sleep(for: .milliseconds(20))
    depthState = .clear
    ingestBlindDepth(into: ar)
    while navigation.state == .planning || navigation.state == .driving {
        try? await Task.sleep(for: .milliseconds(5))
    }

    XCTAssertGreaterThan(ar.depthSnapshotVersion, initialDepthVersion)
    XCTAssertEqual(navigation.state, .arrived)
    XCTAssertEqual(navigationCommandCount, 0)
}
```

This catches removal of the stale-only retry branch: the old implementation fails immediately.

- [ ] **Step 2: Verify RED**

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testReachedGoalWithStaleDepthWaitsForFreshSnapshotBeforeArrival
```

Expected: FAIL because navigation reports a depth-safety failure rather than `arrived`.

- [ ] **Step 3: Implement the minimal stale-only retry**

Evaluate the arrival probe through `depthSafetyObservation`. For `.unavailable(.staleRawDepth)`, capture `ar.depthSnapshotVersion`, stop transport, log retry start, wait for a newer snapshot, log recovery, and `continue`. If the wait expires, log timeout and execute the existing fail-closed arrival failure. Do not retry any other state.

Start fields:

```swift
"depth_age": Self.formatSeconds(observation.sampleAge)
"depth_version": String(depthVersion)
"timeout_seconds": Self.formatSeconds(scanDepthRecoveryTimeout)
```

Recovery fields:

```swift
"previous_version": String(depthVersion)
"depth_version": String(ar.depthSnapshotVersion)
"depth_timestamp": ar.depthSnapshotTimestamp.map(Self.formatSeconds) ?? "none"
```

- [ ] **Step 4: Verify GREEN with the Step 2 command**

Expected: PASS with zero speed-control commands.

- [ ] **Step 5: Add the persistent-stale timeout regression**

```swift
func testReachedGoalWithPersistentlyStaleDepthTimesOutFailClosed() async {
    let (navigation, ar) = makeNavigation(
        depthRecoveryTimeout: 0.03,
        depthSafetyState: { _ in .unavailable(.staleRawDepth) }
    )
    ingestTrackedPose(into: ar, yaw: 0, sequence: 1)
    ingestBlindDepth(into: ar)
    let logBefore = runtimeLog()

    navigation.navigate(to: Vec2(0.10, 0))
    while navigation.state == .planning || navigation.state == .driving {
        try? await Task.sleep(for: .milliseconds(5))
    }

    XCTAssertEqual(navigation.state, .failed("Depth safety stop at arrival: stale_raw_depth."))
    XCTAssertEqual(navigationCommandCount, 0)
    let logDelta = String(runtimeLog().dropFirst(logBefore.count))
    XCTAssertTrue(logDelta.contains("nav_arrival_depth_retry_timeout"))
    XCTAssertTrue(logDelta.contains("depth_state=stale_raw_depth"))
}
```

This catches an unbounded wait or unsafe arrival.

- [ ] **Step 6: Run the three arrival cases**

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testReachedGoalWithMissingDepthDoesNotReportArrival \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testReachedGoalWithStaleDepthWaitsForFreshSnapshotBeforeArrival \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testReachedGoalWithPersistentlyStaleDepthTimesOutFailClosed
```

Expected: PASS; missing depth fails immediately, transient stale depth recovers, and persistent stale depth times out fail-closed.

### Task 2: Delete the obsolete ground-sensitive signal

**Files:**
- Modify: `swift/Sources/PhroverKit/Perception/ARSessionManager.swift`
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift`
- Modify: `swift/Sources/PhroverCloud/Cloud/RoverTelemetryPublisher.swift`
- Modify: `examples/PhroverOperator/PhroverOperator/Views/DriveView.swift`
- Modify: `examples/PhroverOperator/PhroverOperator/Views/ConversationView.swift`
- Modify: `swift/Tests/PhroverKitTests/ARSessionManagerTests.swift`
- Modify: `swift/Tests/PhroverKitTests/UnprojectionTests.swift`
- Modify: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift`

**Interfaces:**
- Consumes: existing geometry-aware `depth_safety_*` telemetry.
- Produces: no `forwardClearance` state, percentile calculation, `forward_clearance` event, or drive field.

- [ ] **Step 1: Remove tests tied only to dead code**

Delete the two `testForwardClearance*` tests, all `VisualTargetApproachDecision` and `visualTargetApproachCommand` tests, and assertions that only check `ar.forwardClearance == .infinity`. Keep generic `ObstacleGuard` tests.

- [ ] **Step 2: Remove the AR signal**

Delete `forwardClearance`, `lastClearanceLogAt`, their resets, `logForwardClearanceIfNeeded`, and both static percentile functions. Preserve object-grounding depth storage:

```swift
if let depth = frame.smoothedSceneDepth ?? frame.sceneDepth {
    latestDepthMap = depth.depthMap
}
```

- [ ] **Step 3: Remove navigation consumers**

Delete `VisualTargetApproachDecision`, both uncalled helper functions, `telemetry["forward_clearance"]`, the obsolete cloud payload field `forwardClearanceM`, and both UI labels backed by `ar.forwardClearance`. In the two general safety calls where `checkForwardObstacle` is already `false`, replace `ar.forwardClearance` with `.infinity`; preserve communications and tipping arguments.

- [ ] **Step 4: Run focused suites**

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/NavigationSafetyTests \
  -only-testing:PhroverKitTests/ARSessionManagerTests \
  -only-testing:PhroverKitTests/UnprojectionTests \
  -only-testing:PhroverKitTests/DepthSafetyEvaluatorTests \
  -only-testing:PhroverKitTests/DepthSafetyModelsTests
```

Expected: PASS.

- [ ] **Step 5: Check cleanup**

```bash
rg -n 'ar\.forwardClearance|forwardClearanceM|var forwardClearance|func forwardClearance|forward_clearance|lastClearanceLogAt|VisualTargetApproachDecision|visualTargetApproachDecision|visualTargetApproachCommand' \
  swift/Sources swift/Tests
```

Expected: no matches. This is a cleanup check, not a behavior regression test.

### Task 3: Verify and deploy

**Files:**
- Verify modified files without staging them.
- Build: `/private/tmp/phrover-ground-recovery-derived/Build/Products/Debug-iphoneos/PhroverOperator.app`

**Interfaces:**
- Consumes: Tasks 1 and 2.
- Produces: signed app on the paired iPhone 15 Pro and a fresh physical trace.

- [ ] **Step 1: Run full verification**

```bash
scripts/test-swift-sdk.sh -quiet
git diff --check
```

Expected: both exit 0.

- [ ] **Step 2: Build**

```bash
xcodebuild -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -destination 'platform=iOS,id=00008130-00166C823C91001C' \
  -configuration Debug \
  -derivedDataPath /private/tmp/phrover-ground-recovery-derived build
```

Expected: `** BUILD SUCCEEDED **`.

- [ ] **Step 3: Install and launch**

```bash
xcrun devicectl device install app --device FC11C836-4978-5B20-9170-16EAD18568BE \
  /private/tmp/phrover-ground-recovery-derived/Build/Products/Debug-iphoneos/PhroverOperator.app
xcrun devicectl device process launch \
  --device FC11C836-4978-5B20-9170-16EAD18568BE bot.coy.phrover
```

Expected: both succeed for `bot.coy.phrover`.

- [ ] **Step 4: Pull a fresh physical trace**

After a stationary clear-floor arrival trial, copy app `Documents` from the iPhone 15 Pro into `/private/tmp/phrover-ground-recovery-log-*`. Require geometry-aware `depth_safety_*` evidence, no new `forward_clearance` events, and either fresh-snapshot recovery or a fail-closed persistent-stale timeout. Do not claim physical acceptance before analyzing this trace.
