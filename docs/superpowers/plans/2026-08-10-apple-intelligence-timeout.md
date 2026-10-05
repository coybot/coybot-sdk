# Apple Intelligence Timeout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Allow available Apple Intelligence inference enough time to complete while preserving bounded online cloud fallback.

**Architecture:** `HybridBrain` will run the timeout race only when a cloud brain is configured and connectivity is available. Offline or local-only operation will call the on-device brain directly and rely on `MissionAgent`'s existing 12-second outer deadline. The online primary-stage default becomes eight seconds.

**Tech Stack:** Swift 6, Foundation concurrency, XCTest, Xcode iOS simulator tests.

## Global Constraints

- Apple Intelligence remains primary.
- No model-session state is shared between decisions.
- Cloud behavior changes only in the primary-stage timeout duration.
- Navigation, perception, and offline parser behavior remain unchanged.

---

### Task 1: Hybrid brain timeout policy

**Files:**
- Modify: `swift/Sources/PhroverCloud/Cloud/HybridBrain.swift:16-58`
- Test: `swift/Tests/PhroverCloudTests/HybridBrainTests.swift:69`

**Interfaces:**
- Consumes: `HybridBrain.init(cloud:onDevice:primaryTimeout:isOnline:missionTelemetry:)`
- Produces: `HybridBrain.nextAction(_:)` with local-only and offline timeout bypass.

- [ ] **Step 1: Write the failing local-only regression test**

Add a test with a delayed `RecordingBrain`, `cloud: nil`, and a 10 ms
`primaryTimeout`. Assert that the delayed on-device `.done` response is returned
instead of throwing `PrimaryStageTimeoutError`.

- [ ] **Step 2: Write the failing offline regression test**

Add a test with delayed on-device and configured cloud brains, `isOnline` false,
and a 10 ms primary timeout. Assert that on-device succeeds and cloud is never
called.

- [ ] **Step 3: Run the two tests and verify RED**

Run:

```bash
xcodebuild test -project examples/PhroverOperator/PhroverOperator.xcodeproj -scheme PhroverSDKTests -destination 'platform=iOS Simulator,id=EEA52712-371D-4FF6-B8EF-A2C78319D57F' -derivedDataPath /private/tmp/phrover-apple-intelligence-timeout -only-testing:PhroverCloudTests/HybridBrainTests
```

Expected: the new local-only/offline tests fail because the current 10 ms race
returns a timeout before the delayed on-device output.

- [ ] **Step 4: Implement the timeout policy**

Change the initializer default to `.seconds(8)`. In `nextAction`, when `cloud ==
nil` or `isOnline()` is false, call `onDevice.nextAction(context)` directly,
record `brain=on_device reason=primary`, and return its output. Retain the
existing race and cloud fallback when cloud exists and connectivity is online.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run the command from Step 3. Expected: all `HybridBrainTests` pass, including
the existing online timeout and cancellation tests.

- [ ] **Step 6: Verify integration and device build**

Run the focused PhroverCloud suite, `git diff --check`, then build, install, and
launch `bot.coy.phrover` on the connected iPhone 15 Pro Max.
