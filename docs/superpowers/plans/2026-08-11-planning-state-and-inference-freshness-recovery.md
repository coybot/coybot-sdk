# Planning State and Inference Freshness Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent mission decisions from replacing navigation during asynchronous planning and ensure visual-target assessments run only after post-inference AR planning readiness is restored.

**Architecture:** `NavigationController.State` owns the canonical active-motion predicate. `NavigationController` exposes one requirement-driven planning-context interface that hides stopping, stable-pose polling, mesh revision checks, deadlines, cancellation, and telemetry. `MissionAgent` resolves a target before readiness, waits through both planning and driving, and performs no detector inference between final readiness and A* assessment.

**Tech Stack:** Swift 6, ARKit, Vision/CoreML, XCTest, async/await, RoverNav A*, `PhroverSDKTests`, Xcode/CoreDevice.

## Global Constraints

- Preserve the three-consecutive-normal-observation readiness gate.
- Preserve `navigationTrackingFreshness = 0.5` seconds.
- Preserve one shared `navigationTrackingRecoveryTimeout = 3.0` second deadline.
- Never accept limited/unavailable mesh callbacks or clear/reset the costmap for recovery.
- Permit at most one refreshed-map retry for a visual-target approach.
- Never issue a motor command from a non-ready planning context.
- Preserve cancellation and session-generation invalidation at every async wait.
- Do not move Vision/CoreML inference off the main actor in this change.
- Do not weaken depth, obstacle, communications, tipping, or watchdog behavior.
- Do not stage or commit overlapping implementation/test files in this dirty checkout; use the verification checkpoints below and leave changes unstaged unless the user separately authorizes a commit strategy.

---

### Task 1: Canonical Active Motion Semantics

**Files:**
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift:17-20`
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:3596-3637`
- Test: `swift/Tests/PhroverKitTests/MissionAgentTests.swift`

**Interfaces:**
- Produces: `NavigationController.State.isMotionActive: Bool`.
- Consumes: existing `.planning`, `.driving`, `.idle`, `.arrived`, and `.failed(String)` states.

- [ ] **Step 1: Add a failing state-classification test**

Add to `NavigationSafetyTests`:

```swift
func testOnlyPlanningAndDrivingAreActiveMotionStates() {
    XCTAssertTrue(NavigationController.State.planning.isMotionActive)
    XCTAssertTrue(NavigationController.State.driving.isMotionActive)
    XCTAssertFalse(NavigationController.State.idle.isMotionActive)
    XCTAssertFalse(NavigationController.State.arrived.isMotionActive)
    XCTAssertFalse(NavigationController.State.failed("stop").isMotionActive)
}
```

- [ ] **Step 2: Add a failing mission wait regression**

Extend `FakeMotion` with an opt-in planning transition:

```swift
var beginsNavigationInPlanning = false
var planningDelay: TimeInterval = 0.03

func navigate(to goal: Vec2) {
    lifecycleEvents.append("navigate")
    navigateCalls.append(goal)
    onNavigate?(navigateCalls.count)
    state = beginsNavigationInPlanning ? .planning : .driving
    let outcome = navigateOutcomes.isEmpty ? navigateOutcome : navigateOutcomes.removeFirst()
    Task { @MainActor in
        if beginsNavigationInPlanning {
            try? await Task.sleep(for: .seconds(planningDelay))
            guard state == .planning else { return }
            state = .driving
        }
        try? await Task.sleep(for: .milliseconds(20))
        guard state == .driving else { return }
        state = outcome
    }
}
```

Add `testMissionWaitsThroughPlanningBeforeRequestingAnotherDecision`, using a one-step brain-driven navigation and setting `beginsNavigationInPlanning = true`, then assert:

```swift
XCTAssertEqual(motion.navigateCalls.count, 1)
XCTAssertEqual(brain.seenContexts.count, 1)
XCTAssertEqual(motion.state, .arrived)
```

Add two async regressions using the same fake mode:

```swift
func testMissionCancellationDuringPlanningStopsMotion() async {
    motion.beginsNavigationInPlanning = true
    motion.planningDelay = 1
    let task = Task { await agent.handle("Go to the refrigerator") }
    await waitUntil { motion.state == .planning }
    task.cancel()
    await task.value
    XCTAssertEqual(motion.cancelCallCount, 1)
    XCTAssertTrue(motion.navigateCalls.count == 1)
}

func testSessionChangeDuringPlanningStopsMotion() async {
    motion.beginsNavigationInPlanning = true
    motion.planningDelay = 1
    let task = Task { await agent.handle("Go to the refrigerator") }
    await waitUntil { motion.state == .planning }
    topology.startSession(generation: 2, initialPose: .init(position: .zero, yaw: 0))
    await task.value
    XCTAssertGreaterThanOrEqual(motion.stopAndWaitCallCount, 1)
    XCTAssertEqual(motion.navigateCalls.count, 1)
}
```

Construct `agent`, `motion`, and `topology` with the existing mission-test helpers used by the neighboring cancellation and session-generation tests. Reuse their `waitUntil` helper rather than adding a second polling utility.

- [ ] **Step 3: Run the focused tests and verify RED**

Run:

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testOnlyPlanningAndDrivingAreActiveMotionStates \
  -only-testing:PhroverKitTests/MissionAgentTests/testMissionWaitsThroughPlanningBeforeRequestingAnotherDecision
```

Expected: compile failure for `isMotionActive`, and the mission regression shows premature settlement before implementation.

- [ ] **Step 4: Implement the canonical predicate**

Add beside `NavigationController.State`:

```swift
extension NavigationController.State {
    var isMotionActive: Bool {
        switch self {
        case .planning, .driving:
            true
        case .idle, .arrived, .failed:
            false
        }
    }
}
```

Keep it internal because only PhroverKit orchestration and tests consume it.

- [ ] **Step 5: Make mission waiting consume the predicate**

Change the loop to:

```swift
RuntimeFileLog.append("mission_motion_wait_started", fields: [
    "state": motion.state.description
])
while motion.state.isMotionActive {
    // Preserve the existing session-generation, mission-cancellation,
    // sleep, and CancellationError branches verbatim.
}
```

Keep `motion_settled` after the loop so it records only terminal states.

- [ ] **Step 6: Verify Task 1 GREEN**

Run the two focused tests from Step 3, then:

```bash
scripts/test-swift-sdk.sh -quiet -only-testing:PhroverKitTests/MissionAgentTests
```

Expected: all mission tests pass; the planning regression records one navigation call and one brain context.

- [ ] **Step 7: Verification checkpoint**

Run:

```bash
git diff --check -- \
  swift/Sources/PhroverKit/Nav/NavigationController.swift \
  swift/Sources/PhroverKit/Voice/MissionAgent.swift \
  swift/Tests/PhroverKitTests/NavigationSafetyTests.swift \
  swift/Tests/PhroverKitTests/MissionAgentTests.swift
```

Do not stage or commit these overlapping files.

---

### Task 2: Requirement-Driven Planning Context

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:16-72`
- Modify: `swift/Sources/PhroverKit/Nav/NavigationController.swift:207-247,786-796`
- Test: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift:119-180`
- Test: `swift/Tests/PhroverKitTests/MissionAgentTests.swift:3810-3920`

**Interfaces:**
- Produces: `PlanningContextRequirement` with `.stablePose` and `.refreshedTrustedMesh`.
- Produces: `RoverMotion.preparePlanningContext(requiring:) async -> PlanningRecoveryOutcome`.
- Preserves: `recoverPlanningContext()` as a compatibility wrapper for `.refreshedTrustedMesh`.
- Consumes: `PlanningReadinessSnapshot`, `navigationTrackingFreshness`, and `navigationTrackingRecoveryTimeout`.

- [ ] **Step 1: Add failing stable-pose requirement tests**

Add to `NavigationSafetyTests`:

```swift
func testStablePosePreparationDoesNotRequireNewMesh() async {
    var readiness = PlanningReadinessSnapshot(
        sessionGeneration: 1,
        normalObservationStreak: 3,
        trustedMeshRevision: 9
    )
    let (navigation, ar) = makeNavigation(
        trackingRecoveryTimeout: 0.1,
        planningReadinessSnapshot: { readiness }
    )
    ingestFreshNormalPose(into: ar, generation: 1, sequence: 1)

    let outcome = await navigation.preparePlanningContext(requiring: .stablePose)

    XCTAssertEqual(outcome, .ready)
    XCTAssertEqual(readiness.trustedMeshRevision, 9)
}
```

Add a flicker test that begins at streak 1, changes to 0, then changes to 3 while keeping the same mesh revision. Assert `.ready` only after streak 3.

- [ ] **Step 2: Add failing refreshed-mesh compatibility tests**

Update the existing recovery tests to call:

```swift
let outcome = await navigation.preparePlanningContext(requiring: .refreshedTrustedMesh)
XCTAssertEqual(outcome, .ready)
```

Keep assertions that revision `N` is insufficient and revision `N + 1` succeeds. Keep one existing call through `recoverPlanningContext()` and assert that it still requires the newer revision; this is the compatibility-wrapper proof.

- [ ] **Step 3: Run focused tests and verify RED**

Run:

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testStablePosePreparationDoesNotRequireNewMesh \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testPlanningPreparationIgnoresTrackingFlickerUntilStable \
  -only-testing:PhroverKitTests/NavigationSafetyTests/testPlanningRecoveryWaitsForStablePoseAndNewerMesh
```

Expected: compile failure because the requirement enum and method do not exist.

- [ ] **Step 4: Add the planning-context interface**

Add:

```swift
public enum PlanningContextRequirement: String, Equatable, Sendable {
    case stablePose = "stable_pose"
    case refreshedTrustedMesh = "refreshed_trusted_mesh"
}
```

Extend `RoverMotion`:

```swift
func preparePlanningContext(
    requiring requirement: PlanningContextRequirement
) async -> PlanningRecoveryOutcome
```

Provide the source-compatible defaults:

```swift
public func preparePlanningContext(
    requiring requirement: PlanningContextRequirement
) async -> PlanningRecoveryOutcome { .ready }

public func recoverPlanningContext() async -> PlanningRecoveryOutcome {
    await preparePlanningContext(requiring: .refreshedTrustedMesh)
}
```

- [ ] **Step 5: Generalize production recovery without duplicating polling**

Rename the production method and branch only on its success condition:

```swift
public func preparePlanningContext(
    requiring requirement: PlanningContextRequirement
) async -> PlanningRecoveryOutcome {
    await stopAndWait()
    guard !Task.isCancelled else { return .cancelled }
    let initial = planningReadinessSnapshot()
    let generation = ar.sessionGeneration
    let deadline = Date().addingTimeInterval(trackingRecoveryTimeout)
    var reportedPoseReady = false

    RuntimeFileLog.append("nav_planning_recovery_started", fields: [
        "requirement": requirement.rawValue,
        "session_generation": "\(generation)",
        "pose_streak": "\(initial.normalObservationStreak)",
        "mesh_revision": "\(initial.trustedMeshRevision)",
        "timeout": Self.formatSeconds(trackingRecoveryTimeout)
    ])

    while Date() < deadline {
        guard !Task.isCancelled else {
            return planningRecoveryFailed(.cancelled, requirement: requirement)
        }
        guard ar.sessionGeneration == generation else {
            return planningRecoveryFailed(.sessionGenerationChanged, requirement: requirement)
        }
        let snapshot = planningReadinessSnapshot()
        if snapshot.isPoseReady,
           Self.isNavigationObservationUsable(ar.latestObservation) {
            if requirement == .stablePose { return .ready }
            if snapshot.trustedMeshRevision > initial.trustedMeshRevision {
                RuntimeFileLog.append("nav_planning_mesh_refreshed", fields: [
                    "requirement": requirement.rawValue,
                    "previous_revision": "\(initial.trustedMeshRevision)",
                    "current_revision": "\(snapshot.trustedMeshRevision)"
                ])
                return .ready
            }
        }
        try? await Task.sleep(for: .seconds(RoverConfig.navigationTrackingPollInterval))
    }
    let outcome: PlanningRecoveryOutcome = requirement == .stablePose
        ? .trackingTimeout
        : (reportedPoseReady ? .meshTimeout : .trackingTimeout)
    return planningRecoveryFailed(outcome, requirement: requirement)
}
```

Retain the existing one-time `nav_planning_pose_ready` transition inside the loop. Update `planningRecoveryFailed` to accept the requirement and emit `requirement.rawValue`.

- [ ] **Step 6: Update the mission fake**

Replace `planningRecoveryOutcome` with:

```swift
var planningContextOutcomes: [PlanningContextRequirement: PlanningRecoveryOutcome] = [
    .stablePose: .ready,
    .refreshedTrustedMesh: .ready,
]
private(set) var planningContextRequirements: [PlanningContextRequirement] = []

func preparePlanningContext(
    requiring requirement: PlanningContextRequirement
) async -> PlanningRecoveryOutcome {
    planningContextRequirements.append(requirement)
    onPlanningRecovery?()
    return planningContextOutcomes[requirement] ?? .ready
}
```

Keep the compatibility wrapper only where an existing test explicitly exercises it.

- [ ] **Step 7: Verify Task 2 GREEN**

Run:

```bash
scripts/test-swift-sdk.sh -quiet -only-testing:PhroverKitTests/NavigationSafetyTests
```

Expected: all navigation safety tests pass, including tracking timeout, mesh timeout, cancellation, and no-motion assertions.

- [ ] **Step 8: Verification checkpoint**

Run `git diff --check` for the four files listed in this task. Do not stage or commit them.

---

### Task 3: Post-Inference Readiness for Initial Visual Planning

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:2030-2190,3110-3180`
- Test: `swift/Tests/PhroverKitTests/MissionAgentTests.swift:1180-1510`

**Interfaces:**
- Consumes: `preparePlanningContext(requiring: .stablePose)` from Task 2.
- Consumes: typed `NavigationGoalAssessment.rejectionReason`.
- Produces: one stable-pose retry for an initial target assessment rejected as `trackingUnstable`.

- [ ] **Step 1: Add a failing slow-inference freshness regression**

Use the mission fake seam to represent inference aging the current pose:

```swift
func testVisualTargetWaitsForStablePoseAfterInferenceBeforeAssessing() async {
    let motion = FakeMotion()
    let perception = FakePerception()
    perception.objects = [refrigeratorDetection]
    perception.unprojectResult = Vec2(4, 0)
    var planningReady = false
    motion.goalAssessment = { goal in
        NavigationGoalAssessment(
            goal: goal,
            isReachable: planningReady,
            pathDistance: planningReady ? goal.length : .infinity,
            rejectionReason: planningReady ? nil : .trackingUnstable
        )
    }
    motion.onPlanningRecovery = { planningReady = true }

    await agent.handle("Go to the refrigerator")

    XCTAssertEqual(motion.planningContextRequirements, [.stablePose])
    XCTAssertEqual(perception.unprojectFrameSequences.count, 1)
    XCTAssertEqual(motion.navigateCalls, [Vec2(4, 0)])
}
```

Use the existing refrigerator helper/fixture names in the test file rather than introducing a second detection factory.

- [ ] **Step 2: Add failing non-ready outcome tests**

Table-drive `.trackingTimeout`, `.cancelled`, and `.sessionGenerationChanged` for `.stablePose`. Assert no navigation calls and exactly one terminal mission result. Keep mesh timeout out of this table because `.stablePose` does not require a mesh refresh.

- [ ] **Step 3: Run focused tests and verify RED**

Run:

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/MissionAgentTests/testVisualTargetWaitsForStablePoseAfterInferenceBeforeAssessing \
  -only-testing:PhroverKitTests/MissionAgentTests/testInitialVisualPlanningFailsClosedWhenStablePoseTimesOut
```

Expected: the first test performs refreshed-mesh recovery or repeats target inference instead of requesting only `.stablePose`.

- [ ] **Step 4: Add a one-shot stable-pose branch to visual approach**

Inside `approachVisualTarget`, track:

```swift
var hasPreparedInitialVisualPose = false
```

When direct/stand-off assessment yields `trackingUnstable` before staging selection:

```swift
if !hasPreparedInitialVisualPose {
    hasPreparedInitialVisualPose = true
    switch await motion.preparePlanningContext(requiring: .stablePose) {
    case .ready:
        continue // reassess targetGoal; do not call resolve/detectObjects
    case .trackingTimeout:
        return .failed("AR tracking did not recover in time.")
    case .meshTimeout:
        preconditionFailure("stablePose cannot produce meshTimeout")
    case .cancelled:
        return .cancelled
    case .sessionGenerationChanged:
        return .sessionGenerationChanged
    }
}
```

Refactor `reachableVisualNavigationGoal` to return both the chosen goal and the dominant rejection when none is reachable:

```swift
private enum VisualNavigationGoalResult {
    case reachable(Vec2)
    case rejected(NavigationGoalRejection)
}
```

This avoids inferring `trackingUnstable` later from candidate counts and keeps the retry decision at the assessment seam.

- [ ] **Step 5: Verify Task 3 GREEN**

Run the focused tests from Step 3, then the complete `MissionAgentTests` suite.

Expected: one detector/unprojection pass, one `.stablePose` preparation, one navigation call, and no second brain decision.

- [ ] **Step 6: Verification checkpoint**

Run `git diff --check` on `MissionAgent.swift` and `MissionAgentTests.swift`. Do not stage or commit them.

---

### Task 4: Reorder the One-Shot Refreshed-Map Retry

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:2128-2175`
- Test: `swift/Tests/PhroverKitTests/MissionAgentTests.swift:1380-1470`

**Interfaces:**
- Consumes: `preparePlanningContext(requiring: .refreshedTrustedMesh)` from Task 2.
- Consumes: generation-bound target world goal from `resolve(_:missionID:)`.
- Produces: detector-before-readiness ordering and zero detector calls between readiness success and A* assessment.

- [ ] **Step 1: Replace the existing refreshed-map success test with an ordering test**

Record events in the fake adapters:

```swift
var events: [String] = []
perception.onDetectObjects = { events.append("detect") }
motion.onPlanningRecovery = { events.append("prepare") }
motion.goalAssessment = { goal in
    events.append("assess")
    return refreshedAssessment(for: goal)
}
```

Assert the retry suffix exactly:

```swift
XCTAssertEqual(Array(events.suffix(3)), ["detect", "prepare", "assess"])
XCTAssertEqual(
    motion.planningContextRequirements.filter { $0 == .refreshedTrustedMesh }.count,
    1
)
```

Add `onDetectObjects: (() -> Void)?` to `FakePerception` and invoke it at the start of `detectObjects()`.

- [ ] **Step 2: Add a failing no-inference-after-readiness test**

Make `onPlanningRecovery` set `didPrepare = true`; make `onDetectObjects` fail the test if `didPrepare` is already true:

```swift
perception.onDetectObjects = {
    if didPrepare { XCTFail("Detector ran after final planning readiness") }
}
```

Keep the final refreshed assessment reachable and assert successful navigation.

- [ ] **Step 3: Run focused tests and verify RED**

Run:

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/MissionAgentTests/testAllUnreachableTargetReacquiresBeforeRefreshedMapPreparation \
  -only-testing:PhroverKitTests/MissionAgentTests/testRefreshedMapRetryDoesNotInferAfterReadiness
```

Expected: current ordering is `prepare`, `detect`, `assess`, so both tests fail.

- [ ] **Step 4: Reorder the production retry**

Replace the `.ready`-then-`resolve` flow with:

```swift
guard let refreshedGoal = resolve(target, missionID: missionID) else {
    return .failed("I couldn't reacquire the target before refreshing the map.")
}
targetGoal = refreshedGoal

switch await motion.preparePlanningContext(requiring: .refreshedTrustedMesh) {
case .ready:
    usedCandidateIDs.removeAll()
    RuntimeFileLog.append("mission_target_planning_retry", fields: [
        "mission": "\(missionID)",
        "target": targetDescription(target),
        "original_rejection_reason": dominantRejection?.rawValue ?? "unknown",
        "inference_completed_before_readiness": "true"
    ])
    continue // reassess immediately; do not call resolve/detectObjects
case .trackingTimeout:
    return .failed("AR tracking did not recover in time.")
case .meshTimeout:
    return .failed("The navigation map did not refresh in time.")
case .cancelled:
    return .cancelled
case .sessionGenerationChanged:
    return .sessionGenerationChanged
}
```

Check mission cancellation and session generation immediately before and after `resolve`, and again after readiness returns.

- [ ] **Step 5: Keep retry bounded and typed**

Retain `hasRecoveredPlanningContext`. On the second all-unreachable result, emit `mission_target_staging_exhausted` with the dominant typed rejection and return `No safe route toward target.` without another call to `preparePlanningContext`.

- [ ] **Step 6: Verify Task 4 GREEN**

Run the two focused tests, then:

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/MissionAgentTests \
  -only-testing:PhroverKitTests/NavigationSafetyTests
```

Expected: all mission and navigation safety tests pass.

- [ ] **Step 7: Verification checkpoint**

Run `git diff --check` on the two task files. Do not stage or commit them.

---

### Task 5: Complete Automated and iPhone 15 Pro Verification

**Files:**
- Verify all touched Swift sources and tests.
- Use runtime artifact: app `Documents/phrover-runtime.log`.

**Interfaces:**
- Consumes: all behavior from Tasks 1-4.
- Produces: automated, signed-device-build, install, and physical-trace evidence.

- [ ] **Step 1: Run complete automated verification**

Run:

```bash
scripts/test-swift-sdk.sh -quiet
git diff --check
```

Expected: Xcode test exit code 0 and no whitespace errors.

- [ ] **Step 2: Verify the target device identity**

Run:

```bash
xcrun devicectl list devices
```

Require the available paired device to be exactly:

```text
iPhone 15 Pro (iPhone16,1)
CoreDevice identifier: FC11C836-4978-5B20-9170-16EAD18568BE
Xcode destination identifier: 00008130-00166C823C91001C
```

- [ ] **Step 3: Build and install the exact current workspace**

Run:

```bash
xcodebuild \
  -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -destination 'platform=iOS,id=00008130-00166C823C91001C' \
  -configuration Debug \
  -derivedDataPath /private/tmp/phrover-planning-recovery-derived \
  build

xcrun devicectl device install app \
  --device FC11C836-4978-5B20-9170-16EAD18568BE \
  /private/tmp/phrover-planning-recovery-derived/Build/Products/Debug-iphoneos/PhroverOperator.app
```

Expected: `BUILD SUCCEEDED` and installed bundle `bot.coy.phrover`.

- [ ] **Step 4: Run the physical mission**

With the rover stationary and connected, issue one refrigerator navigation command. Keep the iPhone camera unobstructed while detector inference and planning readiness complete. Do not infer success from UI text; wait for the mission terminal state.

- [ ] **Step 5: Pull the post-run runtime log**

Run:

```bash
TRACE_DIR=$(mktemp -d /private/tmp/phrover-planning-recovery-trace-XXXXXX)
xcrun devicectl device copy from \
  --device FC11C836-4978-5B20-9170-16EAD18568BE \
  --domain-type appDataContainer \
  --domain-identifier bot.coy.phrover \
  --source Documents \
  --destination "$TRACE_DIR"
```

- [ ] **Step 6: Run trace acceptance checks**

Run:

```bash
RUNTIME_LOG="$TRACE_DIR/phrover-runtime.log"
rg 'motion_settled state=(planning|driving)' "$RUNTIME_LOG"
rg 'mission_motion_wait_started|nav_planning_recovery_started|nav_planning_pose_ready|nav_planning_mesh_refreshed|mission_target_planning_retry|nav_drive_tick|nav_planning_failed|mission_target_staging_exhausted' "$RUNTIME_LOG" | tail -n 200
```

Expected:

- the first command prints no matches;
- `mission_motion_wait_started` covers `.planning` through the terminal state;
- detector/target reacquisition precedes final readiness;
- the final readiness is followed by either `nav_drive_tick` or a typed A* rejection;
- no refreshed candidate batch is rejected solely as `tracking_unstable` while forward tracking remains normal.

- [ ] **Step 7: Preserve the dirty-worktree boundary**

Run:

```bash
git status --short
git diff --check
```

Report touched files and verification evidence. Do not stage or commit overlapping implementation files without separate user authorization.
