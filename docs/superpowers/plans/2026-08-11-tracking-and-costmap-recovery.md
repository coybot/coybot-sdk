# Tracking and Costmap Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Gate planning and motion on stable AR tracking, protect the costmap from limited-quality mesh updates, expose typed planner failures, and retry an all-unreachable visual target once after a bounded trusted-map refresh.

**Architecture:** A small readiness state machine in `ARSessionManager` owns the normal-pose streak and trusted-mesh revision. `NavigationController` consumes that state for bounded stop-and-wait recovery and maps typed `RoverNav` planner results into mission-facing assessments. `MissionAgent` performs one refreshed planning retry without returning to the brain loop.

**Tech Stack:** Swift 6, ARKit, XCTest, async/await, RoverNav A*, `PhroverSDKTests`, Xcode/CoreDevice physical-device verification.

## Global Constraints

- Require three consecutive fresh `.normal` observations before planning or motion.
- Use one shared 3.0-second planning-recovery deadline.
- Never accept mesh anchors while the normal streak is unsatisfied.
- Never clear/reset the AR session or weaken existing depth, obstacle, communication, tipping, or freshness limits.
- Permit at most one refreshed-map retry per visual-target approach.
- Do not stage or commit overlapping implementation/test files because the worktree contains pre-existing changes.

---

### Task 1: Typed A* and Goal Assessment Outcomes

**Files:**
- Modify: `swift/Sources/RoverNav/AStarPlanner.swift`
- Modify: `swift/Tests/RoverNavTests/AStarPlannerTests.swift`
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:1-45`
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift:90-110,659-667`
- Modify: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift:10-50`

**Interfaces:**
- Produces `PathPlanningFailure: String, Equatable, Sendable` with `startOutsideMap`, `goalOutsideMap`, `startBlocked`, `goalBlocked`, and `noConnectedPath`.
- Produces `PathPlanningResult: Equatable, Sendable` with `.path([Vec2])` and `.rejected(PathPlanningFailure)`.
- Produces `AStarPlanner.assess(from:to:in:) -> PathPlanningResult`; existing `plan(...) -> [Vec2]?` wraps it.
- Extends `NavigationGoalAssessment` with `rejectionReason: NavigationGoalRejection?`, defaulting to `nil` for existing reachable test fixtures.

- [ ] **Step 1: Add failing RoverNav typed-outcome tests**

Add focused tests asserting exact results for start outside, goal outside, start blocked, goal blocked, and a sealed-wall disconnected path:

```swift
XCTAssertEqual(
    AStarPlanner().assess(from: Vec2(-1, 0), to: Vec2(1, 1), in: emptyRoom()),
    .rejected(.startOutsideMap)
)
```

Repeat with in-bounds fixtures for the remaining four reasons.

- [ ] **Step 2: Run the RoverNav tests and verify RED**

```bash
swift test --package-path swift --filter AStarPlannerTests
```

Expected: compile failure because `assess`, `PathPlanningResult`, and `PathPlanningFailure` do not exist.

- [ ] **Step 3: Implement typed A* assessment**

Add the public enums and move the existing A* body into `assess`. Check start bounds, goal bounds, start blocked, and goal blocked in that order. Return `.rejected(.noConnectedPath)` only after the frontier is exhausted. Keep `plan` as:

```swift
public func plan(from start: Vec2, to goal: Vec2, in map: Costmap) -> [Vec2]? {
    guard case .path(let path) = assess(from: start, to: goal, in: map) else { return nil }
    return path
}
```

- [ ] **Step 4: Add failing PhroverKit assessment-reason tests**

Extend the current outside-map test to assert `.goalOutsideMap`; add a missing-pose assertion and a disconnected-map mapping assertion.

- [ ] **Step 5: Map planner results into `NavigationGoalAssessment`**

Add:

```swift
public enum NavigationGoalRejection: String, Equatable, Sendable {
    case missingPose, trackingUnstable
    case startOutsideMap, goalOutsideMap, startBlocked, goalBlocked, noConnectedPath
}
```

Add `rejectionReason` with an initializer default of `nil`. Update `NavigationController.assessGoal` and planning telemetry to translate every `PathPlanningFailure` by raw value or an exhaustive switch.

- [ ] **Step 6: Verify Task 1 GREEN**

```bash
swift test --package-path swift --filter AStarPlannerTests
scripts/test-swift-sdk.sh -quiet -only-testing:PhroverKitTests/NavigationSafetyTests
```

Expected: typed planner and navigation assessment tests PASS.

---

### Task 2: Stable Pose Streak and Trusted Mesh Revision

**Files:**
- Create: `swift/Sources/PhroverKit/Perception/PlanningReadiness.swift`
- Create: `swift/Tests/PhroverKitTests/PlanningReadinessTests.swift`
- Modify: `swift/Sources/PhroverKit/Perception/ARSessionManager.swift`
- Modify: `swift/Tests/PhroverKitTests/ARSessionManagerTests.swift`

**Interfaces:**
- Produces internal `PlanningReadinessSnapshot(sessionGeneration:normalObservationStreak:trustedMeshRevision:)` with `isPoseReady` computed as streak `>= 3`.
- Produces internal `PlanningReadinessTracker` methods `reset(sessionGeneration:)`, `ingest(_:)`, and `recordTrustedMeshUpdate()`.
- `ARSessionManager.planningReadiness` exposes the snapshot to `NavigationController`.

- [ ] **Step 1: Write failing readiness-state tests**

Create tests proving normal-limited-normal leaves streak 1, three normals produce pose readiness, generation changes reset state, and trusted mesh revision advances only when `recordTrustedMeshUpdate()` is invoked while pose-ready.

- [ ] **Step 2: Run and verify RED**

```bash
scripts/test-swift-sdk.sh -quiet -only-testing:PhroverKitTests/PlanningReadinessTests
```

Expected: compile failure because the tracker does not exist.

- [ ] **Step 3: Implement the pure readiness tracker**

Use a value type with no ARKit dependency:

```swift
struct PlanningReadinessTracker {
    private(set) var snapshot: PlanningReadinessSnapshot
    mutating func ingest(_ observation: PoseObservation) {
        guard observation.sessionGeneration == snapshot.sessionGeneration else { return }
        snapshot.normalObservationStreak = observation.trackingQuality == .normal
            ? snapshot.normalObservationStreak + 1 : 0
    }
    mutating func recordTrustedMeshUpdate() {
        guard snapshot.isPoseReady else { return }
        snapshot.trustedMeshRevision &+= 1
    }
}
```

- [ ] **Step 4: Integrate tracker into ARSessionManager**

Reset it from `resetTracking`, `suspendObservations`, and generation changes. Feed every accepted `PoseObservation` into it. In `collectMesh`, guard `planningReadiness.isPoseReady`; only then merge anchors and advance the trusted revision. Add one telemetry event when a mesh callback is rejected due to unstable tracking, rate-limited to avoid frame spam.

- [ ] **Step 5: Verify Task 2 GREEN**

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/PlanningReadinessTests \
  -only-testing:PhroverKitTests/ARSessionManagerTests
```

Expected: streak/reset/gating tests PASS.

---

### Task 3: Bounded Navigation Planning Recovery

**Files:**
- Modify: `swift/Sources/PhroverKit/Config/RoverConfig.swift`
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:20-55`
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift`
- Modify: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift`

**Interfaces:**
- Produces `PlanningRecoveryOutcome: Equatable, Sendable` with `.ready`, `.trackingTimeout`, `.meshTimeout`, `.cancelled`, `.sessionGenerationChanged`.
- Adds `RoverMotion.recoverPlanningContext() async -> PlanningRecoveryOutcome`; default implementation returns `.ready` for non-AR test doubles.
- Adds `RoverConfig.navigationTrackingRecoveryTimeout = 3.0` while preserving `navigationTrackingFreshness = 0.5` and poll interval `0.05`.

- [ ] **Step 1: Add failing tracking-flicker and late-recovery tests**

Write one test that feeds normal-limited-normal and asserts zero motion, then supplies three normals and a trusted mesh revision and asserts motion begins. Write another with a test timeout greater than 1.5 seconds that supplies readiness at 1.7 seconds and asserts recovery succeeds.

- [ ] **Step 2: Add failing tracking and mesh timeout tests**

Assert `.trackingTimeout` when no three-normal streak arrives and `.meshTimeout` when pose readiness arrives but trusted mesh revision does not advance. In both, assert `navigationCommandCount == 0`.

- [ ] **Step 3: Run and verify RED**

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testPlanningRecoveryIgnoresTrackingFlickerUntilStable \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testPlanningRecoveryAcceptsStableTrackingAfterPreviousDeadline \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testPlanningRecoveryTrackingTimeoutSendsNoMotion \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testPlanningRecoveryMeshTimeoutSendsNoMotion
```

Expected: compile failure because `recoverPlanningContext` and outcomes do not exist.

- [ ] **Step 4: Implement recovery with one deadline**

`recoverPlanningContext` must await `stopAndWait`, capture generation and mesh revision, emit `nav_planning_recovery_started`, then poll readiness until the shared deadline. Emit `nav_planning_pose_ready` once, require a greater trusted mesh revision, and return the typed result. Check task cancellation and session generation on every iteration.

- [ ] **Step 5: Defer initial planning behind readiness**

Move `planAndStore` from the synchronous portion of `startNavigation` into its owned async task. That task must stop, wait for stable pose readiness, require an existing trusted mesh snapshot in production, plan, and only then enter `drive`. Preserve operation-token handoff so cancellation and new commands cannot resume an older task.

For unit tests, extend `makeNavigation` with injected readiness closures so existing safety tests can start from a deterministic ready snapshot without manufacturing ARMeshAnchor instances. Dedicated readiness tests use the real AR snapshot provider.

- [ ] **Step 6: Use stable readiness in the drive loop**

Replace the single-observation predicate in `waitForUsableTracking` with `ar.planningReadiness.isPoseReady` plus the existing latest-observation age check. Log final quality, streak, observation age, and elapsed time on timeout.

- [ ] **Step 7: Verify Task 3 GREEN and regressions**

```bash
scripts/test-swift-sdk.sh -quiet -only-testing:PhroverKitTests/NavigationSafetyTests
```

Expected: all navigation safety tests PASS, including communication/tipping/depth veto tests.

---

### Task 4: One-shot Mission Costmap Refresh

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:2025-2230,2920-3100`
- Modify: `swift/Tests/PhroverKitTests/MissionAgentTests.swift`

**Interfaces:**
- Consumes `RoverMotion.recoverPlanningContext()` and typed `NavigationGoalAssessment.rejectionReason`.
- Produces one recovery attempt per `approachVisualTarget` invocation and telemetry `mission_target_planning_retry`.

- [ ] **Step 1: Write the failing refreshed-map success test**

Configure `FakeMotion.goalAssessment` so every first-pass goal is `.noConnectedPath`, make `planningRecovery` return `.ready`, then make the refreshed target and one staging goal reachable. Assert one recovery call, a second target resolution, navigation to the refreshed candidate, and mission arrival without another brain request.

- [ ] **Step 2: Write failing bounded-failure tests**

Add tests for a still-unreachable refreshed map and for `.trackingTimeout`, `.meshTimeout`, `.cancelled`, and `.sessionGenerationChanged`. Assert one recovery call, no unsafe navigation, and one terminal mission result.

- [ ] **Step 3: Run and verify RED**

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/MissionAgentTests/testAllUnreachableTargetRefreshesPlanningContextOnceAndContinues \
  -only-testing:PhroverKitTests/MissionAgentTests/testStillUnreachableRefreshedMapFailsWithoutSecondRecovery
```

Expected: FAIL because the mission currently exhausts immediately.

- [ ] **Step 4: Implement the one-shot recovery**

Track `hasRecoveredPlanningContext` inside `approachVisualTarget`. When no candidate is selected and the dominant rejection reason is a planner/map reason, call recovery once. On `.ready`, emit `mission_target_planning_retry`, reacquire the target with `resolve`, clear per-map candidate IDs, and continue. Map each non-ready outcome to the existing cancellation/session-change/failure terminal path.

- [ ] **Step 5: Add rejection reasons to target telemetry**

Include `rejection_reason` in direct stand-off rejection, staging-candidate, exhaustion, initial planning, and replanning events. Compute the dominant reason deterministically by count, breaking ties by the enum declaration order.

- [ ] **Step 6: Verify Task 4 GREEN**

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/MissionAgentTests \
  -only-testing:PhroverKitTests/NavigationSafetyTests
```

Expected: both complete suites PASS.

---

### Task 5: Complete and Physical Verification

**Files:**
- Verify all touched Swift sources/tests.
- Do not stage implementation files.

- [ ] **Step 1: Run complete automated verification**

```bash
scripts/test-swift-sdk.sh -quiet
git diff --check
```

Expected: complete SDK PASS and no whitespace errors.

- [ ] **Step 2: Build, install, and launch on iPhone 15 Pro**

```bash
xcodebuild -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator -configuration Debug \
  -destination 'platform=iOS,id=00008130-00166C823C91001C' \
  -derivedDataPath /private/tmp/PhroverOperator-device-build build
xcrun devicectl device install app \
  --device FC11C836-4978-5B20-9170-16EAD18568BE \
  /private/tmp/PhroverOperator-device-build/Build/Products/Debug-iphoneos/PhroverOperator.app
xcrun devicectl device process launch \
  --device FC11C836-4978-5B20-9170-16EAD18568BE \
  --terminate-existing bot.coy.phrover
```

Expected: build, install, and launch succeed on the iPhone 15 Pro only.

- [ ] **Step 3: Collect mission evidence**

Issue `Go to refrigerator`, pull the app `Documents` container, and verify ordered readiness, typed planning, and terminal events. Runtime success requires actual mission completion; a bounded typed failure proves safety and diagnostics only.
