# Fresh Depth Eligibility Retry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep rotation stopped across successive stale depth snapshots and authorize it only when the existing depth guard accepts a replacement observation within the bounded retry deadline.

**Architecture:** `NavigationController.rotationSafetyCommand` will own one deadline covering the complete recovery attempt. A focused helper will wait for successive snapshot-version advances and reevaluate each candidate with the original command; it returns only a guard-approved command or a typed final stop observation. Existing blind-volume arc fallback and general safety vetoes remain downstream.

**Tech Stack:** Swift 6, XCTest, async/await, `PhroverSDKTests` Xcode scheme, CoreDevice physical-device verification.

## Global Constraints

- Keep the existing maximum raw-depth age at `0.25s`.
- Never authorize motion merely because `depthSnapshotVersion` advanced.
- Keep the transport stopped between rejected replacement observations.
- Bound the entire retry by `scanDepthRecoveryTimeout` (`0.75s` in production).
- Preserve communication, tracking, tipping, obstacle, and blind swept-volume safety behavior.
- Do not stage or commit implementation/test files because they contain pre-existing worktree changes; commit only if the user later requests it.

---

### Task 1: Retry Until a Depth Observation Is Eligible

**Files:**
- Modify: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift:441-485`
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift:948-1100`

**Interfaces:**
- Consumes: `depthSafetyObservation(_:) -> DepthSafetyObservation`, `guardLayer.evaluate(command:depthSafety:) -> MotionGuardDecision`, `ar.depthSnapshotVersion`, `scanDepthRecoveryTimeout`.
- Produces: bounded rotation recovery that returns a guard-approved `WheelCommand` or retains the final `DepthSafetyObservation` for fail-closed reporting.

- [ ] **Step 1: Write the failing successive-snapshot test**

Add a test beside `testStaleRotationWaitsForFreshSnapshotBeforeSendingMotion` that keeps `depthState` stale for the first replacement snapshot, verifies that no navigation command is sent, then changes the state to clear and ingests a second replacement snapshot:

```swift
func testStaleRotationSkipsNewerStaleSnapshotUntilEligibleSnapshotArrives() async {
    var depthState: DepthSafetyState = .unavailable(.staleRawDepth)
    let (navigation, ar) = makeNavigation(
        depthRecoveryTimeout: 0.5,
        depthSafetyState: { _ in depthState }
    )
    ingestTrackedPose(into: ar, yaw: 0, sequence: 1)
    ingestBlindDepth(into: ar)

    let rotation = Task { await navigation.rotate(by: .pi / 2) }
    await waitForRequestCount(2)
    ingestBlindDepth(into: ar)
    try? await Task.sleep(for: .milliseconds(80))
    XCTAssertEqual(navigationCommandCount, 0)

    depthState = .clear
    ingestBlindDepth(into: ar)
    await waitForNavigationCommandCount(1)
    ingestTrackedPose(into: ar, yaw: .pi / 2, sequence: 2)
    await rotation.value

    XCTAssertEqual(navigation.state, .arrived)
    XCTAssertEqual(navigationCommandCount, 1)
}
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testStaleRotationSkipsNewerStaleSnapshotUntilEligibleSnapshotArrives
```

Expected: FAIL because the current controller exits after evaluating the first newer-but-stale snapshot, leaving `navigationCommandCount == 0` and navigation failed.

- [ ] **Step 3: Implement the bounded eligibility loop**

In `rotationSafetyCommand`, create one deadline immediately after stopping. Repeatedly wait for a version greater than the last evaluated version, evaluate the original rotation command, and:

```swift
case .allow(let safeCommand, let safeObservation):
    // Re-run rotationGeneralSafetyAllowsMotion and return safeCommand.
case .stopDepth(let candidate) where candidate.state == .unavailable(.staleRawDepth):
    // Record candidate age/version, advance last evaluated version, and continue.
case .stopDepth(let candidate):
    // Preserve candidate for the existing blind-volume arc fallback or final stop.
```

Change `waitForNewDepthSnapshot` to accept an explicit `deadline: Date` so every iteration shares one timeout instead of resetting it. On deadline exhaustion, emit `nav_scan_depth_retry_timeout` with the final `depth_state`, `depth_age`, and `depth_version`, then return no command.

- [ ] **Step 4: Run the new test and verify GREEN**

Run the Step 2 command again.

Expected: PASS; no command follows the first stale replacement, and exactly one rotation command follows the eligible replacement.

- [ ] **Step 5: Verify timeout and safety veto regressions**

Run:

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testStaleRotationWithoutFreshSnapshotTimesOutFailClosed \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testPostWaitCommsVetoPreventsRecoveredMotion \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testPostWaitTippingVetoPreventsRecoveredMotion \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testFreshDepthRetryAuthorizesOriginalRotation \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testBlindRotationWaitsForNewDepthThenTimesOutFailClosed
```

Expected: all PASS with zero unauthorized navigation commands.

- [ ] **Step 6: Run complete verification**

Run:

```bash
scripts/test-swift-sdk.sh -quiet
git diff --check
xcodebuild -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator -configuration Debug \
  -destination 'platform=iOS,id=00008130-00166C823C91001C' \
  -derivedDataPath /private/tmp/PhroverOperator-device-build build
```

Expected: full SDK suite PASS, no whitespace errors, and `** BUILD SUCCEEDED **`.

- [ ] **Step 7: Install and collect physical evidence**

Install and launch only on iPhone 15 Pro `FC11C836-4978-5B20-9170-16EAD18568BE`, issue `Go to refrigerator`, and pull `Documents` from `bot.coy.phrover`.

Acceptance requires either:

- `nav_scan_depth_retry_started` followed by one or more stale replacement events, then an accepted depth observation and a rotation command; or
- bounded timeout with `nav_scan_depth_retry_timeout` and no intervening rotation command.

Do not claim refrigerator mission recovery unless the trace also reaches a successful mission terminal.
