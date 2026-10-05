# Low Camera Height Recalibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Recalibrate the fixed iPhone camera mount from 0.55 m to 0.30 m so clear floor is filtered correctly while depth collision safety remains fail-closed.

**Architecture:** Keep the existing `CameraMountCalibration` and `DepthSafetyEvaluator` pipeline unchanged. Change only the production mount height, then prove through synthetic depth maps that a 30 cm clear floor is ignored and a supported low obstacle remains hazardous.

**Tech Stack:** Swift 6, CoreVideo depth buffers, simd transforms, XCTest, Xcode/CoreDevice.

## Global Constraints

- Set the fixed camera height to exactly `0.30` m.
- Do not disable or bypass depth-based collision safety.
- Do not change `minimumCollisionHeight`, `maximumCollisionHeight`, stopping margins, swept-volume coverage, connected-support thresholds, or obstacle-guard behavior.
- Missing, stale, malformed, invalid, or blind depth must remain fail-closed.
- Do not add runtime calibration UI or automatic camera-height estimation.
- Preserve all unrelated dirty-worktree changes; do not stage or commit overlapping configuration or test files unless the user separately approves a commit strategy.

---

### Task 1: Recalibrate the Fixed Camera Mount

**Files:**
- Modify: `swift/Sources/PhroverKit/Config/RoverConfig.swift:24-31`
- Test: `swift/Tests/PhroverKitTests/DepthSafetyEvaluatorTests.swift:1-115`

**Interfaces:**
- Consumes: `RoverConfig.cameraMountCalibration: CameraMountCalibration`.
- Consumes: `DepthSafetyEvaluator.ingest(rawDepthMap:intrinsics:intrinsicsImageSize:cameraTransform:timestamp:calibration:geometry:) -> DepthSafetySnapshot`.
- Produces: a production `CameraMountCalibration` whose `cameraHeight` is `0.30` m.

- [ ] **Step 1: Add the failing clear-floor regression**

Add this test to `DepthSafetyEvaluatorTests`:

```swift
func testThirtyCentimeterProductionMountKeepsVisibleFloorClear() {
    let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
    fillFloorPlane(cameraHeight: 0.30, in: map)

    let snapshot = DepthSafetyEvaluator.ingest(
        rawDepthMap: map,
        intrinsics: wideIntrinsics,
        cameraTransform: cameraTransform(height: 0.30),
        timestamp: 10,
        calibration: RoverConfig.cameraMountCalibration,
        geometry: geometry
    )
    let observation = DepthSafetyEvaluator.evaluate(
        snapshot,
        command: WheelCommand(left: 0.20, right: 0.20),
        now: 10.05
    )

    XCTAssertEqual(RoverConfig.cameraMountCalibration.cameraHeight, 0.30)
    XCTAssertEqual(observation.state, .clear)
}
```

- [ ] **Step 2: Add the low-obstacle safety regression at the same mount height**

Add this test beside the clear-floor regression:

```swift
func testThirtyCentimeterProductionMountStillDetectsLowCrossbar() {
    let map = makeDepthBuffer(width: 80, height: 60, constantDepth: 3.0)
    fill(depth: 0.70, x: 35...44, y: 31...32, in: map)

    let observation = DepthSafetyEvaluator.evaluate(
        DepthSafetyEvaluator.ingest(
            rawDepthMap: map,
            intrinsics: wideIntrinsics,
            cameraTransform: cameraTransform(height: 0.30),
            timestamp: 10,
            calibration: RoverConfig.cameraMountCalibration,
            geometry: geometry
        ),
        command: WheelCommand(left: 0.35, right: 0.35),
        now: 10.05
    )

    XCTAssertTrue(observation.state == .caution || observation.state == .stop)
    XCTAssertLessThan(observation.clearance, 0.80)
    XCTAssertGreaterThanOrEqual(observation.supportCount, 3)
}
```

- [ ] **Step 3: Run the focused tests and verify RED**

Run:

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/DepthSafetyEvaluatorTests/testThirtyCentimeterProductionMountKeepsVisibleFloorClear \
  -only-testing:PhroverKitTests/DepthSafetyEvaluatorTests/testThirtyCentimeterProductionMountStillDetectsLowCrossbar
```

Expected: the clear-floor test fails because the production calibration still reports `0.55` m. The crossbar test compiles and remains hazardous; rows 31–32 at 0.70 m project into the unchanged 0.04–0.50 m collision band at both the old and new calibration heights.

- [ ] **Step 4: Apply the minimal production recalibration**

In `RoverConfig.cameraMountCalibration`, change only the height:

```swift
static let cameraMountCalibration = CameraMountCalibration(
    cameraHeight: 0.30,
    forwardOffset: 0,
    lateralOffset: 0,
    headingAlignment: 0
)
```

- [ ] **Step 5: Run focused depth-safety verification**

Run:

```bash
scripts/test-swift-sdk.sh -quiet \
  -only-testing:PhroverKitTests/DepthSafetyEvaluatorTests \
  -only-testing:PhroverKitTests/DepthSafetyModelsTests
```

Expected: all selected tests pass. In particular, the 30 cm floor is `.clear`, the 30 cm low crossbar is `.caution` or `.stop`, and invalid calibration/depth cases remain unavailable.

- [ ] **Step 6: Check the touched-file diff**

Run:

```bash
git diff --check -- \
  swift/Sources/PhroverKit/Config/RoverConfig.swift \
  swift/Tests/PhroverKitTests/DepthSafetyEvaluatorTests.swift
```

Expected: no whitespace errors. Do not stage or commit these overlapping dirty files.

---

### Task 2: Automated, Signed-Build, and Physical Verification

**Files:**
- Verify: `swift/Sources/PhroverKit/Config/RoverConfig.swift`
- Verify: `swift/Tests/PhroverKitTests/DepthSafetyEvaluatorTests.swift`
- Runtime artifact: app container `Documents/phrover-runtime.log`

**Interfaces:**
- Consumes: production `cameraHeight = 0.30` m from Task 1.
- Produces: automated, signed-device-build, install, and physical clear-floor evidence.

- [ ] **Step 1: Run complete automated verification**

Run:

```bash
scripts/test-swift-sdk.sh -quiet
git diff --check
```

Expected: Xcode test exit code 0 and no whitespace errors.

- [ ] **Step 2: Verify the paired target device**

Run:

```bash
xcrun devicectl list devices
```

Require:

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
  -derivedDataPath /private/tmp/phrover-low-camera-derived \
  build

xcrun devicectl device install app \
  --device FC11C836-4978-5B20-9170-16EAD18568BE \
  /private/tmp/phrover-low-camera-derived/Build/Products/Debug-iphoneos/PhroverOperator.app
```

Expected: `BUILD SUCCEEDED` and installed bundle `bot.coy.phrover`.

- [ ] **Step 4: Capture a stationary clear-floor sample**

Launch the app with the phone fixed 30 cm above the floor. Point it toward unobstructed floor while the rover remains physically stationary. Wait at least 10 seconds so multiple depth samples are logged.

- [ ] **Step 5: Pull and inspect the new runtime log**

Run:

```bash
TRACE_DIR=$(mktemp -d /private/tmp/phrover-low-camera-trace-XXXXXX)
xcrun devicectl device copy from \
  --device FC11C836-4978-5B20-9170-16EAD18568BE \
  --domain-type appDataContainer \
  --domain-identifier bot.coy.phrover \
  --source Documents \
  --destination "$TRACE_DIR"

rg 'forward_clearance|depth_safety|blind_swept_volume|Obstacle ahead' \
  "$TRACE_DIR/phrover-runtime.log" | tail -n 100
```

Expected: the clear-floor interval has no persistent 0.08–0.31 m false-clearance cluster and no false obstacle stop. Missing or blind depth may still report its typed fail-closed state.

- [ ] **Step 6: Preserve the verification boundary**

Report the exact automated results, app bundle path, device identity, install result, and pulled log path. Keep implementation/test changes unstaged because both files overlap the pre-existing dirty checkout.
