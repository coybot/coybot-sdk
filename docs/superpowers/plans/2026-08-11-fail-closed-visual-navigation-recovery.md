# Fail-Closed Visual Navigation Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Recover the iPhone 15 Pro refrigerator mission from transient stale rotation depth and unreachable direct target goals without authorizing motion from stale or planner-rejected data.

**Architecture:** `NavigationController` will generalize its existing bounded rotation depth-refresh path to both blind and stale raw depth, then re-run every safety gate against a newer snapshot. `MissionAgent` will reuse its existing staged visual-target approach when a successful brain decision resolves a visible target but cannot produce a reachable direct or stand-off goal.

**Tech Stack:** Swift 6, Swift Concurrency, XCTest, ARKit-derived depth snapshots, `NavigationController`, `MissionAgent`, `RoverNav`.

## Global Constraints

- Navigation remains fail-closed; stale depth never authorizes motion.
- A recovered rotation requires a strictly newer depth snapshot and a successful fresh re-evaluation of the original command.
- The depth-visible arc fallback remains exclusive to fresh `blind_swept_volume`; stale depth never chooses an alternate motion command.
- Visual staging uses only goals accepted by `RoverMotion.assessGoal`.
- Do not relax depth, obstacle, costmap, stand-off, or confidence thresholds.
- Preserve mission cancellation and room-mapping session generation checks.
- Preserve all unrelated dirty worktree changes. The production and test files in this plan already contain uncommitted work; do not stage or commit them wholesale.

---

### Task 1: Refresh Stale Rotation Depth Before Motion

**Files:**
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift:951-1078`
- Test: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift:393-535`

**Interfaces:**
- Consumes: `DepthSafetyState.unavailable(.staleRawDepth)`, `waitForNewDepthSnapshot(after:operation:)`, and the existing general/depth safety evaluators.
- Produces: a bounded retry of the exact requested rotation after a strictly newer depth snapshot.

- [ ] **Step 1: Write the failing fresh-snapshot regression**

Add next to `testFreshDepthRetryAuthorizesOriginalRotation`:

```swift
func testStaleRotationWaitsForFreshSnapshotBeforeSendingMotion() async {
    var depthState: DepthSafetyState = .unavailable(.staleRawDepth)
    let (navigation, ar) = makeNavigation(
        depthRecoveryTimeout: 0.5,
        depthSafetyState: { _ in depthState }
    )
    ingestTrackedPose(into: ar, yaw: 0, sequence: 1)
    ingestBlindDepth(into: ar)
    let initialVersion = ar.depthSnapshotVersion

    let rotation = Task { await navigation.rotate(by: .pi / 2) }
    await waitForRequestCount(2)
    try? await Task.sleep(for: .milliseconds(20))
    depthState = .clear
    ingestBlindDepth(into: ar)
    await waitForNavigationCommandCount(1)
    ingestTrackedPose(into: ar, yaw: .pi / 2, sequence: 2)
    await rotation.value

    XCTAssertGreaterThan(ar.depthSnapshotVersion, initialVersion)
    XCTAssertEqual(navigation.state, .arrived)
    XCTAssertEqual(navigationCommandCount, 1)
}
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testStaleRotationWaitsForFreshSnapshotBeforeSendingMotion
```

Expected: FAIL because `rotationSafetyCommand` currently enters the refresh path only for `blindSweptVolume`; stale depth immediately fails and sends zero motion commands.

- [ ] **Step 3: Write the timeout regression while production remains unchanged**

Add:

```swift
func testStaleRotationWithoutFreshSnapshotTimesOutFailClosed() async {
    let (navigation, ar) = makeNavigation(
        depthRecoveryTimeout: 0.03,
        depthSafetyState: { _ in .unavailable(.staleRawDepth) }
    )
    ingestTrackedPose(into: ar, yaw: 0, sequence: 1)
    ingestBlindDepth(into: ar)
    let logBefore = runtimeLog()

    await navigation.rotate(by: .pi / 2)

    XCTAssertEqual(
        navigation.state,
        .failed("Depth safety stop while rotating: stale_raw_depth.")
    )
    XCTAssertEqual(navigationCommandCount, 0)
    let logDelta = String(runtimeLog().dropFirst(logBefore.count))
    XCTAssertTrue(logDelta.contains("nav_scan_depth_retry_timeout"))
    XCTAssertTrue(logDelta.contains("depth_state=stale_raw_depth"))
}
```

Add the same private `runtimeLog()` helper used by `MissionAgentTests`, reading `RuntimeFileLog.logFileURL`. Run the test and verify RED because the current stale-depth path emits no bounded-retry timeout telemetry.

- [ ] **Step 4: Generalize the bounded refresh condition**

In `rotationSafetyCommand`, replace the blind-only entry condition with a private predicate that accepts only recoverable freshness states:

```swift
private static func shouldWaitForFreshRotationDepth(_ observation: DepthSafetyObservation) -> Bool {
    guard observation.motionClass == .rotating else { return false }
    switch observation.state {
    case .unavailable(.blindSweptVolume), .unavailable(.staleRawDepth):
        return true
    default:
        return false
    }
}
```

Use the existing stop, `waitForNewDepthSnapshot`, exact-command re-evaluation, fresh acknowledgement check, and cancellation handling. Keep the curved arc branch guarded by the refreshed state being `blindSweptVolume`. When the refresh times out, preserve the stale-specific failure text instead of using the blind-camera message.

- [ ] **Step 5: Run focused rotation safety tests and verify GREEN**

Run:

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testStaleRotationWaitsForFreshSnapshotBeforeSendingMotion \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testStaleRotationWithoutFreshSnapshotTimesOutFailClosed \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testBlindRotationWaitsForNewDepthThenTimesOutFailClosed \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testFreshDepthRetryAuthorizesOriginalRotation \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testPostWaitCommsVetoPreventsRecoveredMotion \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testPostWaitTippingVetoPreventsRecoveredMotion
```

Expected: PASS; fresh stale depth can recover, while timeout, transport veto, tipping veto, and blind-depth behavior remain fail-closed.

---

### Task 2: Stage Brain-Driven Visual Navigation Safely

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:1218-1370,1974-2160`
- Test: `swift/Tests/PhroverKitTests/MissionAgentTests.swift:1190-1800`

**Interfaces:**
- Consumes: `navigate(to:for:) -> Bool`, the existing bounded `approachOfflineVisualTarget`, `OfflineTargetApproachOutcome`, planner-ranked target staging candidates, and mission terminal publishing.
- Produces: staged recovery for a successful brain `.navigate(.visualQuery(...))` decision whose direct and stand-off goals are unreachable.

- [ ] **Step 1: Write the failing brain-driven staging regression**

Add:

```swift
func testBrainDrivenUnreachableVisualTargetStagesAndCompletes() async {
    let motion = FakeMotion()
    motion.navigateOutcomes = [.arrived, .arrived]
    let perception = FakePerception()
    perception.pose = Pose2D(position: .zero, yaw: 0)
    perception.frameSequence = 1
    perception.unprojectResult = Vec2(5, 0)
    perception.objects = [
        PerceivedObject(label: "refrigerator",
                        confidence: 0.99,
                        normalizedPoint: CGPoint(x: 0.5, y: 0.5))
    ]
    perception.frontiers = [
        Frontier(centroid: Vec2(3, 0), widthMeters: 1.0, cellCount: 5)
    ]
    motion.goalAssessment = { goal in
        if goal == Vec2(3, 0), motion.navigateCalls.isEmpty {
            return NavigationGoalAssessment(goal: goal, isReachable: true, pathDistance: 3)
        }
        let targetBecameReachable = !motion.navigateCalls.isEmpty
            && goal.x >= 4.69 && goal.x <= 4.71
        return NavigationGoalAssessment(
            goal: goal,
            isReachable: targetBecameReachable,
            pathDistance: targetBecameReachable ? 1.7 : .infinity
        )
    }
    motion.onNavigate = { call in
        guard call == 1 else { return }
        perception.pose = Pose2D(position: Vec2(3, 0), yaw: 0)
        perception.frameSequence = 2
    }
    var statuses: [MissionCommandStatus] = []
    let brain = FakeBrain(script: [
        .navigate(.visualQuery("refrigerator"))
    ])
    let agent = MissionAgent(
        motion: motion,
        perception: perception,
        voice: FakeVoice(),
        commandStatusDidChange: { statuses.append($0) },
        currentBrain: { brain }
    )

    await agent.handle("Go to the refrigerator")

    XCTAssertEqual(motion.navigateCalls.count, 2)
    XCTAssertEqual(motion.navigateCalls[0], Vec2(3, 0))
    XCTAssertEqual(motion.navigateCalls[1].x, 4.7, accuracy: 0.001)
    XCTAssertEqual(motion.navigateStopClearances, [RoverConfig.visualTargetStopDistance])
    guard case .succeeded = statuses.last else {
        return XCTFail("Expected brain-driven staged navigation to succeed")
    }
}
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/MissionAgentTests/testBrainDrivenUnreachableVisualTargetStagesAndCompletes
```

Expected: FAIL because the brain-driven branch ignores the `false` result from `navigate(to:for:)`, leaves motion idle, and never enters target staging.

- [ ] **Step 3: Write the no-route terminal regression**

Create `testBrainDrivenUnreachableVisualTargetFailsWithoutAskingBrainAgain`. Configure every `goalAssessment` as unreachable, provide no frontiers, and use one scripted visual navigation decision. Assert:

```swift
XCTAssertTrue(motion.navigateCalls.isEmpty)
XCTAssertEqual(brain.seenContexts.count, 1)
guard case .failed(_, _, let message) = statuses.last else {
    return XCTFail("Expected a terminal no-route failure")
}
XCTAssertEqual(message, "No safe route toward target.")
```

Run it and verify RED because the current loop continues to another brain tick rather than publishing the bounded navigation failure.

- [ ] **Step 4: Reuse the staged approach from the brain-driven branch**

Rename `approachOfflineVisualTarget` to `approachVisualTarget` because its implementation is planner- and mission-driven, not brain-failure-specific. Update the offline caller without changing behavior.

In both paths where the brain-driven branch resolves a visual target—direct resolution and post-scan resolution—check the boolean from `navigate(to:for:)`. When it is `false`, call:

```swift
await approachVisualTarget(
    effectiveTarget,
    initialGoal: goal,
    missionID: missionID,
    expectedSessionGeneration: roomTopology?.snapshot.sessionGeneration
)
```

Handle outcomes explicitly:

- `.arrived`: continue through existing optional-return and visual-target-success handling.
- `.failed(let reason)`: cancel motion, speak the reason, publish one failed mission terminal, set `.idle`, and return.
- `.cancelled`: return after existing cancellation telemetry.
- `.sessionGenerationChanged`: stop motion and return without stale success.

Extract a focused `failTargetNavigationMission(missionID:reason:)` helper so online failure telemetry does not incorrectly use offline-fallback or scan-rotation event names.

- [ ] **Step 5: Run focused target staging tests and verify GREEN**

Run:

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/MissionAgentTests/testBrainDrivenUnreachableVisualTargetStagesAndCompletes \
  -only-testing:PhroverKitTests/MissionAgentTests/testBrainDrivenUnreachableVisualTargetFailsWithoutAskingBrainAgain \
  -only-testing:PhroverKitTests/MissionAgentTests/testUnreachableVisibleTargetStagesThroughReachableOpeningAndResumes \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingFailsWithoutProgressCandidate \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingPreservesTransportFailure \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingStopsWhenSessionGenerationChanges \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingCancellationStopsFreshFrameWait
```

Expected: PASS; both brain-driven and offline visual-target paths share the same bounded, planner-confirmed staging behavior.

---

### Task 3: Verification and iPhone 15 Pro Evidence

**Files:**
- Verify only: all modified Swift sources and tests.
- Update only if the project already records physical-device verification there: `docs/phrover-fixes-2026-07-08.md`.

**Interfaces:**
- Consumes: Tasks 1 and 2 plus the `PhroverSDKTests` scheme and connected iPhone 15 Pro ending `68BE`.
- Produces: automated test evidence, a signed device build, an installed app, and a new runtime trace proving the failure sequence no longer reproduces.

- [ ] **Step 1: Run the complete touched suites**

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/NavigationSafetyTests \
  -only-testing:PhroverKitTests/MissionAgentTests
```

Expected: PASS.

- [ ] **Step 2: Run the full SDK gate**

```bash
scripts/test-swift-sdk.sh
```

Expected: PASS. If unrelated baseline failures exist, report them separately with exact test names and retain the focused green evidence.

- [ ] **Step 3: Check the complete dirty-worktree diff without staging it**

```bash
git diff --check
git status --short
```

Expected: no whitespace errors; pre-existing dirty files remain present and unstaged.

- [ ] **Step 4: Build and install the app on the iPhone 15 Pro**

Use the repository’s existing signed `xcodebuild` device route for `bot.coy.phrover`, targeting device `FC11C836-4978-5B20-9170-16EAD18568BE`. Install and launch only after the build succeeds.

- [ ] **Step 5: Run the physical mission and pull a new log**

Issue “Go to refrigerator” from the same physical setup, then copy `Documents` from `bot.coy.phrover` on the iPhone 15 Pro. Verify the trace shows one of these safe terminal paths:

- `stale_raw_depth` followed by a newer depth snapshot and an authorized exact rotation; or
- a timeout/unsafe refreshed observation with no movement command.

For an initially unreachable target, verify `mission_target_staging_selected` precedes movement and the selected goal was logged as reachable. The old sequence—two immediate stale-depth scan failures followed by repeated unreachable stand-offs and `No path to goal.`—must not recur before claiming runtime recovery.

## Worktree Handoff

Do not stage or commit `NavigationController.swift`, `MissionAgent.swift`, or their tests automatically because they contain substantial pre-existing uncommitted work. Report the exact new hunks separately so the user can decide how to package the combined work.
