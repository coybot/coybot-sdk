# Visible Target Staging Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a detected object mission reach the requested object through safe intermediate openings when no direct or collinear stand-off goal is plannable.

**Architecture:** `MissionAgent` keeps the visual query locked and owns a bounded approach loop. It first asks the existing planner for a direct or stand-off goal; when none is reachable, it ranks current unexplored frontiers by planner reachability and reduction in distance to the projected target, moves to the best staging point, waits for a fresh tracked frame, reacquires the same target, and retries. Existing LiDAR, cost-map, transport, cancellation, and room-topology components remain unchanged.

**Tech Stack:** Swift 6, Swift Concurrency, XCTest, ARKit-derived `Frontier`, `NavigationController`, `RoverNav`.

## Global Constraints

- Keep Apple Intelligence primary and retain the deterministic offline object fallback after a brain timeout.
- Keep the requested object query locked for the complete mission.
- Never bypass `NavigationController.assessGoal`, depth safety, tracking safety, session generation, or transport failure.
- Keep the preferred visual stop distance at `0.30` m and confidence threshold at `0.90`.
- Use at most three distinct target-staging candidates.
- Do not modify LiDAR projection, cost-map generation, frontier generation, or rover HTTP transport.
- Preserve unrelated dirty worktree changes and stage only files changed by each task.

---

### Task 1: Represent Reachable Visual Goals Explicitly

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:2619-2665`
- Test: `swift/Tests/PhroverKitTests/MissionAgentTests.swift:1162-1230`

**Interfaces:**
- Consumes: `RoverMotion.assessGoal(_:) -> NavigationGoalAssessment`, `RoverConfig.visualTargetStopDistance`, and `RoverConfig.visualTargetApproachDistance`.
- Produces: `reachableVisualNavigationGoal(for:) -> Vec2?`, where `nil` means every safe direct/stand-off candidate is unreachable.

- [ ] **Step 1: Change the regression test to require no unsafe fallback goal**

Add a test where every assessment is unreachable and assert that no navigation call is issued:

```swift
func testUnreachableVisualSurfaceDoesNotSubmitKnownUnreachableGoal() async {
    let motion = FakeMotion()
    let perception = FakePerception()
    perception.pose = Pose2D(position: .zero, yaw: 0)
    perception.unprojectResult = Vec2(2, 0)
    perception.objects = [
        PerceivedObject(label: "refrigerator",
                        confidence: 0.99,
                        normalizedPoint: CGPoint(x: 0.5, y: 0.5))
    ]
    motion.goalAssessment = { goal in
        NavigationGoalAssessment(goal: goal, isReachable: false, pathDistance: .infinity)
    }
    let agent = MissionAgent(
        motion: motion,
        perception: perception,
        voice: FakeVoice(),
        currentBrain: {
            ThrowingBrain(error: RoverBrainError.onDeviceUnavailable(.modelNotReady))
        }
    )

    await agent.handle("Go to the refrigerator")

    XCTAssertTrue(motion.navigateCalls.isEmpty)
}
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/MissionAgentTests/testUnreachableVisualSurfaceDoesNotSubmitKnownUnreachableGoal
```

Expected: FAIL because the current helper returns `objectGoal` after logging `mission_target_approach_goal_unreachable`.

- [ ] **Step 3: Return `nil` after exhausting safe stand-offs**

Change the helper signature and terminal return:

```swift
private func reachableVisualNavigationGoal(for objectGoal: Vec2) -> Vec2?
```

Keep the current direct assessment and incremental stand-off loop unchanged. Change its
final telemetry and return to:

```swift
RuntimeFileLog.append("mission_target_approach_goal_unreachable", fields: [
    "object_goal_x": String(format: "%.2f", objectGoal.x),
    "object_goal_y": String(format: "%.2f", objectGoal.y),
    "maximum_stand_off": String(format: "%.2f", maximumStandOff)
])
return nil
```

Make `navigate(to:for:)` return `Bool`; issue visual navigation only when the helper returns a goal:

```swift
@discardableResult
private func navigate(to goal: Vec2, for target: NavigationTarget) -> Bool {
    if case .visualQuery = target {
        guard let navigationGoal = reachableVisualNavigationGoal(for: goal) else { return false }
        motion.navigate(to: navigationGoal,
                        stoppingAtForwardClearance: RoverConfig.visualTargetStopDistance)
        return true
    }
    motion.navigate(to: goal)
    return true
}
```

- [ ] **Step 4: Run focused visual-goal tests and verify GREEN**

Run:

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/MissionAgentTests/testUnreachableVisualSurfaceDoesNotSubmitKnownUnreachableGoal \
  -only-testing:PhroverKitTests/MissionAgentTests/testUnreachableVisualSurfaceUsesReachableStandOffGoal \
  -only-testing:PhroverKitTests/MissionAgentTests/testUnreachableVisualSurfaceBacksOffUntilApproachGoalIsReachable
```

Expected: all three tests PASS.

- [ ] **Step 5: Commit the explicit unreachable result**

```bash
git add swift/Sources/PhroverKit/Voice/MissionAgent.swift \
  swift/Tests/PhroverKitTests/MissionAgentTests.swift
git commit -m "Avoid submitting unreachable visual goals"
```

---

### Task 2: Rank Safe Target-Staging Openings

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:237-265,2089-2125`
- Test: `swift/Tests/PhroverKitTests/MissionAgentTests.swift`

**Interfaces:**
- Consumes: `[ExplorationCandidate]`, current `Pose2D.position`, projected target `Vec2`, used candidate IDs, and `RoverMotion.assessGoal(_:)`.
- Produces: private `VisualTargetStagingCandidate` and `rankedVisualTargetStagingCandidates(toward:excluding:) -> [VisualTargetStagingCandidate]`.

- [ ] **Step 1: Write failing ranking tests**

Create mission-level tests with three frontiers:

```swift
perception.frontiers = [
    Frontier(centroid: Vec2(2, 1), widthMeters: 1.0, cellCount: 5),
    Frontier(centroid: Vec2(3, 0), widthMeters: 1.0, cellCount: 5),
    Frontier(centroid: Vec2(-1, 0), widthMeters: 1.0, cellCount: 5),
]
```

Configure `goalAssessment` so `(2,1)` and `(3,0)` are reachable, while all object/stand-off points are unreachable. Name the test `testVisibleTargetStagingPrefersCandidateWithMostTargetProgress` and assert `(3,0)` is selected because it reduces target distance most. In `testVisibleTargetStagingSkipsUnreachableCandidate`, make `(3,0)` unreachable and assert `(2,1)` is selected. In `testVisibleTargetStagingRejectsCandidateWithoutTargetProgress`, make only `(-1,0)` reachable and assert it is rejected because it increases target distance.

- [ ] **Step 2: Run the three tests and verify RED**

Run:

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingPrefersCandidateWithMostTargetProgress \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingSkipsUnreachableCandidate \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingRejectsCandidateWithoutTargetProgress
```

Expected: FAIL because visible-target missions currently do not rank or navigate to frontiers after stand-off exhaustion.

- [ ] **Step 3: Add the private ranking value and helper**

Add:

```swift
private struct VisualTargetStagingCandidate {
    let candidate: ExplorationCandidate
    let assessment: NavigationGoalAssessment
    let targetDistanceBefore: Double
    let targetDistanceAfter: Double

    var targetProgress: Double { targetDistanceBefore - targetDistanceAfter }
}
```

Implement `rankedVisualTargetStagingCandidates(toward:excluding:)` to:

1. Call `updateWorldModel()` before reading candidates.
2. Exclude visited and previously used IDs.
3. Assess every candidate with `motion.assessGoal`.
4. Require `isReachable` and at least `0.10` m target progress.
5. Sort by descending target progress, then ascending path distance.
6. Emit `mission_target_staging_candidate` for every assessed candidate.

- [ ] **Step 4: Run ranking tests and verify GREEN**

Expected: all ranking tests PASS, including the no-progress rejection case.

- [ ] **Step 5: Commit candidate ranking**

```bash
git add swift/Sources/PhroverKit/Voice/MissionAgent.swift \
  swift/Tests/PhroverKitTests/MissionAgentTests.swift
git commit -m "Rank safe visual target staging openings"
```

---

### Task 3: Stage, Reacquire, and Resume the Offline Mission

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:1470-1710`
- Test: `swift/Tests/PhroverKitTests/MissionAgentTests.swift:1120-1320`

**Interfaces:**
- Consumes: `navigate(to:for:) -> Bool`, ranked staging candidates, `waitForMotionToSettle`, `waitForFreshTrackedPerceptionFrame`, and `scanForUnresolvedVisualTarget`.
- Produces: private `approachOfflineVisualTarget(_:initialGoal:missionID:expectedSessionGeneration:) async -> OfflineTargetApproachOutcome`.

- [ ] **Step 1: Write the end-to-end failing mission test**

Reproduce the device behavior with a visible refrigerator at `Vec2(5,0)`, an opening at `Vec2(2,1)`, and assessments that make all target goals unreachable before the first staging move and reachable afterward. Configure `navigateOutcomes = [.arrived, .arrived]`, update the fake pose/frame in `onNavigate`, and assert:

```swift
XCTAssertEqual(motion.navigateCalls.count, 2)
XCTAssertEqual(motion.navigateCalls[0], Vec2(2, 1))
XCTAssertEqual(motion.navigateCalls[1].x, 4.7, accuracy: 0.001)
XCTAssertEqual(motion.navigateStopClearances, [RoverConfig.visualTargetStopDistance])
XCTAssertTrue(phases.contains(.acting))
guard case .succeeded = statuses.last else {
    return XCTFail("Expected staged refrigerator mission to succeed")
}
```

- [ ] **Step 2: Run the end-to-end test and verify RED**

Expected: FAIL because the current offline mission terminates after the unreachable direct goal.

- [ ] **Step 3: Add the bounded approach outcome and loop**

Add:

```swift
private enum OfflineTargetApproachOutcome {
    case arrived
    case failed(String)
    case cancelled
    case sessionGenerationChanged
}
```

Implement an approach loop with `maximumStagingAttempts = 3` and a `Set<String>` of used candidate IDs. For each iteration:

1. If `navigate(to: targetGoal, for: target)` returns true, wait for motion and return its concrete outcome.
2. Otherwise emit `mission_target_staging_started`.
3. Select the first ranked staging candidate; fail with `No safe route toward target.` if none exists.
4. Set `phase = .acting`, record `frameSequence`, navigate to the staging world point, and wait for motion.
5. Preserve cancellation, session-generation, and concrete motion failures.
6. Wait for a frame newer than the baseline with normal tracking.
7. Call `resolve(target, missionID:)`; if absent, use the existing bounded visual scan.
8. Emit `mission_target_staging_completed`, update `targetGoal`, and iterate.
9. Emit `mission_target_staging_exhausted` after three distinct candidates.

Replace the one-shot block at `runOfflineObjectMission` lines 1670-1706 with this outcome switch. Keep the existing arrival telemetry and optional return leg unchanged.

- [ ] **Step 4: Run staged mission and existing fallback tests**

Run:

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/MissionAgentTests/testUnreachableVisibleTargetStagesThroughReachableOpeningAndResumes \
  -only-testing:PhroverKitTests/MissionAgentTests/testBrainUnavailableFallsBackToVisibleRefrigeratorAndDoesNotReturn \
  -only-testing:PhroverKitTests/MissionAgentTests/testBrainUnavailableFallsBackToVisibleRefrigeratorAndReturnsWhenRequested
```

Expected: PASS.

- [ ] **Step 5: Commit staged approach execution**

```bash
git add swift/Sources/PhroverKit/Voice/MissionAgent.swift \
  swift/Tests/PhroverKitTests/MissionAgentTests.swift
git commit -m "Stage through openings toward visible targets"
```

---

### Task 4: Bound Recovery and Preserve Safety

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift`
- Test: `swift/Tests/PhroverKitTests/MissionAgentTests.swift`

**Interfaces:**
- Consumes: `OfflineTargetApproachOutcome` and the staging loop from Task 3.
- Produces: tested cancellation, session-reset, no-progress, repeated-candidate, attempt-budget, and motion-failure behavior.

- [ ] **Step 1: Add failing safety tests**

Add these focused tests:

- `testVisibleTargetStagingFailsWithoutProgressCandidate`: provide only a reachable
  frontier behind the start pose; assert no navigation and a failed terminal status.
- `testVisibleTargetStagingDoesNotReuseCandidateAfterFrontierRefresh`: return the same
  frontier after the first staging arrival while keeping target goals unreachable; assert
  its world point appears exactly once in `navigateCalls`.
- `testVisibleTargetStagingStopsAfterThreeDistinctCandidates`: provide four reachable,
  progress-making frontiers over successive frame sequences; keep target goals
  unreachable and assert only three staging navigation calls occur.
- `testVisibleTargetStagingPreservesTransportFailure`: settle the first staging navigation
  as `.failed("Rover command link lost.")`; assert that exact text in the terminal status.
- `testVisibleTargetStagingStopsWhenSessionGenerationChanges`: advance the topology
  generation from the staging `onNavigate` callback; assert `stopAndWaitCallCount == 1`
  and a failed terminal status.
- `testVisibleTargetStagingCancellationStopsFreshFrameWait`: start a staging mission with
  no newer observation, submit `Stop`, and assert active motion is cancelled and the first
  mission publishes no success.

- [ ] **Step 2: Run each safety test and verify RED**

Run:

```bash
scripts/test-swift-sdk.sh \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingFailsWithoutProgressCandidate \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingDoesNotReuseCandidateAfterFrontierRefresh \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingStopsAfterThreeDistinctCandidates \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingPreservesTransportFailure \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingStopsWhenSessionGenerationChanges \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisibleTargetStagingCancellationStopsFreshFrameWait
```

Expected: each test fails on the missing guard it specifies, not from fixture or compilation errors.

- [ ] **Step 3: Implement the minimal guards and telemetry**

Use the existing mission/session helper methods and emit:

```text
mission_target_staging_started
mission_target_staging_candidate
mission_target_staging_selected
mission_target_staging_completed
mission_target_staging_exhausted
```

Every terminal branch must call the existing `failOfflineObjectMission` path exactly once.

- [ ] **Step 4: Run all MissionAgent tests**

```bash
scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/MissionAgentTests
```

Expected: PASS.

- [ ] **Step 5: Commit safety coverage**

```bash
git add swift/Sources/PhroverKit/Voice/MissionAgent.swift \
  swift/Tests/PhroverKitTests/MissionAgentTests.swift
git commit -m "Bound visible target staging recovery"
```

---

### Task 5: Full Verification and Device Evidence

**Files:**
- Modify: `docs/phrover-fixes-2026-07-08.md`

**Interfaces:**
- Consumes: completed target-staging implementation and runtime telemetry.
- Produces: full automated verification and one physical-device evidence record.

- [ ] **Step 1: Run repository checks**

```bash
scripts/test-swift-sdk.sh
git diff --check
```

Expected: complete iOS SDK suite PASS and no whitespace errors.

- [ ] **Step 2: Build for the connected iPhone 15 Pro**

```bash
xcodebuild \
  -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -configuration Debug \
  -destination 'platform=iOS,id=00008130-00166C823C91001C' \
  -derivedDataPath /private/tmp/PhroverOperator-device-build \
  build
```

Expected: `** BUILD SUCCEEDED **`.

- [ ] **Step 3: Install and launch on the iPhone 15 Pro**

```bash
xcrun devicectl device install app \
  --device FC11C836-4978-5B20-9170-16EAD18568BE \
  /private/tmp/PhroverOperator-device-build/Build/Products/Debug-iphoneos/PhroverOperator.app
xcrun devicectl device process launch \
  --device FC11C836-4978-5B20-9170-16EAD18568BE \
  --terminate-existing bot.coy.phrover
```

- [ ] **Step 4: Run the physical acceptance command and pull logs**

With the phone connected to the rover Wi-Fi, issue `Go to refrigerator`. Pull the app `Documents` container and verify ordered events:

```text
speech_capture_completed
voice_command_received
mission_target_match
mission_target_approach_goal_unreachable
mission_target_staging_selected
nav_goal_start
mission_target_staging_completed
mission_target_match
nav_goal_start
mission_offline_fallback_target_arrived
```

Also verify rover request status `200`, no fourth staging attempt, and a terminal succeeded status.

- [ ] **Step 5: Document verified evidence**

Append the failure cause, implementation, automated test results, device/build identity, and exact runtime event sequence to `docs/phrover-fixes-2026-07-08.md`. Do not claim physical success if the acceptance command was not run.

- [ ] **Step 6: Commit verification documentation**

```bash
git add docs/phrover-fixes-2026-07-08.md
git commit -m "Document visible target staging verification"
```
