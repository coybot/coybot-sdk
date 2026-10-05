# Visual Target Braking Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve a 0.30 m visual-target stand-off despite the 0.19 m post-stop approach measured in the device log.

**Architecture:** Retain the existing LiDAR-triggered stopping path and calibrate its configuration values to the observed WAVE ROVER behavior. Regression tests exercise `NavigationController` decisions directly so no hardware or network mock is required.

**Tech Stack:** Swift, XCTest, PhroverKit, RoverNav

## Global Constraints

- Do not change the 0.30 m desired stand-off.
- Do not change voice, perception, planning, or HTTP transport behavior.
- Do not add reverse-braking motor commands.

---

### Task 1: Calibrate Visual Target Braking

**Files:**
- Modify: `swift/Tests/PhroverKitTests/NavigationSafetyTests.swift`
- Modify: `swift/Sources/PhroverKit/Config/RoverConfig.swift`

**Interfaces:**
- Consumes: `NavigationController.visualTargetApproachDecision(distanceToGoal:forwardClearance:stopDistance:)` and `NavigationController.visualTargetApproachCommand(_:forwardClearance:stopDistance:)`.
- Produces: A 0.50 m brake trigger, 0.75 m slowdown threshold, and 0.10 m/s final wheel-speed cap.

- [x] **Step 1: Write the failing regression tests**

Add assertions that 0.50 m produces `.arrived`, 0.51 m produces `.approach`, and a 0.70 m forward command is capped at 0.10 m/s.

- [x] **Step 2: Verify the current calibration fails the new expectation**

Run:

```bash
rg -n 'visualTargetBrakeLeadDistance = 0\.20|visualTargetSlowdownDistance = 0\.75|visualTargetApproachMaxWheelSpeed = 0\.10' \
  swift/Sources/PhroverKit/Config/RoverConfig.swift
```

Expected before implementation: exit 1 because the safer calibration values are absent.

- [x] **Step 3: Apply the minimal configuration change**

Set:

```swift
public static let visualTargetBrakeLeadDistance = 0.20
public static let visualTargetSlowdownDistance = 0.75
public static let visualTargetApproachMaxWheelSpeed = 0.10
```

- [ ] **Step 4: Run the focused tests and verify they pass**

Run:

```bash
swift test --triple arm64-apple-macosx26.0 --filter NavigationSafetyTests
```

Expected: all `NavigationSafetyTests` pass.

The package test target is not exposed by the Xcode project, and SwiftPM's
macOS runner cannot compile this iOS-only ARKit target. The production iOS
scheme is built instead in Step 5; the test-runner limitation is unchanged by
this calibration.

- [x] **Step 5: Run iOS build verification**

Run:

```bash
xcodebuild build -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -destination 'platform=iOS Simulator,id=B99F477C-ACD3-4E31-A683-81048E5C7FDA'
git diff --check
```

Expected: the iOS application builds and `git diff --check` reports no whitespace errors.
