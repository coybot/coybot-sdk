# Apple-Intelligence-Primary Offline Object Missions Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep Apple Intelligence as the primary mission brain while making visible-object navigation, color-qualified other-room search, and explicitly requested return trips work offline when the model is unavailable.

**Architecture:** Enrich RoverYOLO detections with local color evidence, expose the exact Apple Intelligence availability state, run Apple Intelligence before the optional cloud brain, and invoke a deterministic object-mission fallback only when the configured brains fail before producing a usable first decision. The fallback reuses `MissionAgent`'s existing slow scan, visual target lock, LiDAR goal, safety, room-transition, and return-to-start paths.

**Tech Stack:** Swift 6, Swift Concurrency, Foundation Models, ARKit/CoreVideo, Vision/Core ML, RoverNav, XCTest, XCUITest, Xcode iOS device runner

**Execution status (2026-08-10):** Tasks 1-8 are implemented and committed through
`4914a59`. Final independent review passed with no P0-P2 findings. The full SDK suite
passed 317/317 and the signed generic iOS build succeeded. Task 9 physical acceptance is
still pending because CoreDevice cannot currently locate the intended iPhone 15 Pro.

## Global Constraints

- Follow strict RED-GREEN-REFACTOR for every task: add one focused failing test, run it and inspect the expected failure, add only enough production code to pass, then rerun the focused suite.
- Apple Intelligence remains the primary planner. Cloud is optional and may run only after the on-device stage is unavailable, times out, or rejects the command.
- The deterministic fallback handles object-navigation missions only. It must reject unsupported requests without motion.
- Do not add an object-name allowlist. Parse a general object phrase and normalize only established aliases such as `fridge` to `refrigerator`.
- Return only when the command explicitly includes `come back`, `go back`, or `return`. `Go to the refrigerator` must stop at the refrigerator.
- Require object confidence `>= 0.90`; for color-qualified commands also require requested-color confidence `>= 0.70`.
- Stop approximately `0.30 m` in front of a confirmed visual target by using `RoverConfig.visualTargetStopDistance`.
- Emergency stop, depth/person/tracking safety, session generation, and rover transport failures remain authoritative.
- Preserve and reread the existing uncommitted changes in `ARSessionManager.swift`, `MissionAgent.swift`, `ARSessionManagerTests.swift`, and `MissionAgentTests.swift` before editing those files. Do not discard or overwrite them.
- Do not stage `.serena`, `AGENTS.md`, `docs/agents`, database MCP scripts, or the untracked Talk retry plan as part of this feature.
- Run SDK tests through `scripts/test-swift-sdk.sh`; a plain host `swift test` is not a valid PhroverKit verification route because PhroverKit imports iOS-only frameworks.
- Use `RuntimeFileLog` for all new runtime diagnostics. Do not log image bytes, transcripts beyond the existing command log, credentials, or cloud configuration values.

---

### Task 1: Parse the Bounded Offline Object Mission Contract

**Files:**
- Create: `swift/Sources/PhroverKit/Voice/OfflineObjectMissionIntent.swift`
- Create: `swift/Tests/PhroverKitTests/OfflineObjectMissionIntentParserTests.swift`

**Interfaces:**
- Produces: `OfflineObjectMissionIntent`, `LocalObjectColor`, and `OfflineObjectMissionIntentParser.parse(_:)`.
- Consumes later: `MissionAgent` fallback selection and color-aware visual grounding.

- [ ] **Step 1: Write failing parser tests**

Add tests covering the complete fallback boundary:

```swift
final class OfflineObjectMissionIntentParserTests: XCTestCase {
    func testParsesVisibleObjectWithoutImplicitReturn() throws {
        let intent = try XCTUnwrap(OfflineObjectMissionIntentParser.parse("Go to the fridge"))
        XCTAssertEqual(intent.objectQuery, "refrigerator")
        XCTAssertEqual(intent.targetLabel, "refrigerator")
        XCTAssertEqual(intent.requestedColors, [])
        XCTAssertFalse(intent.searchOtherRooms)
        XCTAssertFalse(intent.shouldReturn)
    }

    func testParsesColorOtherRoomAndExplicitReturn() throws {
        let intent = try XCTUnwrap(OfflineObjectMissionIntentParser.parse(
            "Go to the black chair in the other room and come back"
        ))
        XCTAssertEqual(intent.objectQuery, "black chair")
        XCTAssertEqual(intent.targetLabel, "chair")
        XCTAssertEqual(intent.requestedColors, [.black])
        XCTAssertTrue(intent.searchOtherRooms)
        XCTAssertTrue(intent.shouldReturn)
    }

    func testRejectsCommandsOutsideObjectNavigationContract() {
        XCTAssertNil(OfflineObjectMissionIntentParser.parse("Tell me a joke"))
        XCTAssertNil(OfflineObjectMissionIntentParser.parse("Go to the chair and bring me a book"))
        XCTAssertNil(OfflineObjectMissionIntentParser.parse("Go to"))
    }
}
```

- [ ] **Step 2: Run the focused parser tests and verify RED**

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-offline-parser-red.xcresult \
  -only-testing:PhroverKitTests/OfflineObjectMissionIntentParserTests
```

Expected: build/test failure because the parser and intent types do not exist.

- [ ] **Step 3: Implement the smallest parser that satisfies the contract**

Use value types that can be shared by perception and mission code:

```swift
public enum LocalObjectColor: String, CaseIterable, Hashable, Sendable {
    case black, white, gray, red, orange, yellow, green, blue, purple, brown
}

public struct OfflineObjectMissionIntent: Equatable, Sendable {
    public let objectQuery: String
    public let targetLabel: String
    public let requestedColors: Set<LocalObjectColor>
    public let searchOtherRooms: Bool
    public let shouldReturn: Bool
}

public enum OfflineObjectMissionIntentParser {
    public static func parse(_ utterance: String) -> OfflineObjectMissionIntent?
}
```

Implement `parse(_:)` by lowercasing and normalizing punctuation, matching one of `go to`, `navigate to`, `drive to`, `find`, or `look for` at the start, recording and removing supported return/location phrases, extracting every leading `LocalObjectColor`, and retaining the remaining noun phrase as `targetLabel`. Canonicalize `fridge` to `refrigerator` and `couch` to `sofa`. Strip articles without maintaining an object allowlist. Reject a blank target or residual conjunctions that indicate a second unsupported object/action.

- [ ] **Step 4: Run parser tests and verify GREEN**

Run the Step 2 command again. Expected: all parser tests pass.

- [ ] **Step 5: Commit the parser increment**

```bash
git add swift/Sources/PhroverKit/Voice/OfflineObjectMissionIntent.swift \
  swift/Tests/PhroverKitTests/OfflineObjectMissionIntentParserTests.swift
git diff --cached --check
git commit -m "Add offline object mission intent parser"
```

---

### Task 2: Expose Exact Apple Intelligence Availability

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/OnDeviceBrain.swift`
- Modify: `swift/Sources/PhroverKit/Voice/RoverBrain.swift`
- Modify: `swift/Tests/PhroverKitTests/OnDeviceBrainTests.swift`

**Interfaces:**
- Produces: `OnDeviceBrainAvailability`, `OnDeviceBrain.availability`, and a typed unavailable error.
- Preserves: `OnDeviceBrain.isAvailable` as a compatibility view over `availability == .available`.

- [ ] **Step 1: Add failing availability mapping tests**

Inject an app-level availability provider rather than constructing a real Foundation Model in tests:

```swift
func testReportsEveryUnavailableReasonWithoutCreatingResponder() async {
    for reason in [
        OnDeviceBrainAvailability.deviceNotEligible,
        .appleIntelligenceNotEnabled,
        .modelNotReady,
    ] {
        let brain = OnDeviceBrain(availability: { reason }, makeResponder: {
            XCTFail("Unavailable brain must not create a responder")
            return StubResponder(output: .init(decision: .done))
        })

        XCTAssertEqual(brain.availability, reason)
        do {
            _ = try await brain.nextAction(MissionContext())
            XCTFail("Expected unavailable brain to throw")
        } catch {
            XCTAssertEqual(error as? RoverBrainError, .onDeviceUnavailable(reason))
        }
    }
}
```

Also test the operator message for each state and that `.available` still creates one responder per decision.

- [ ] **Step 2: Run focused tests and verify RED**

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-availability-red.xcresult \
  -only-testing:PhroverKitTests/OnDeviceBrainTests
```

Expected: compile failure because structured availability and the typed error do not exist.

- [ ] **Step 3: Implement structured availability and error mapping**

Add the public app-level enum and preserve the old error case for other brain implementations:

```swift
public enum OnDeviceBrainAvailability: Equatable, Sendable {
    case available
    case deviceNotEligible
    case appleIntelligenceNotEnabled
    case modelNotReady

    public var operatorMessage: String? {
        switch self {
        case .available:
            nil
        case .deviceNotEligible:
            "Apple Intelligence is not supported on this iPhone. Supported object commands can still run offline."
        case .appleIntelligenceNotEnabled:
            "Turn on Apple Intelligence in Settings. Supported object commands can still run offline."
        case .modelNotReady:
            "Apple Intelligence is still preparing. Keep the iPhone on Wi-Fi and power. Supported object commands can still run offline."
        }
    }
}

public enum RoverBrainError: Error, Equatable, LocalizedError {
    case unavailable
    case onDeviceUnavailable(OnDeviceBrainAvailability)
    case unsupported(String)
}
```

Map `SystemLanguageModel.Availability` explicitly:

```swift
private static func mapAvailability(
    _ availability: SystemLanguageModel.Availability
) -> OnDeviceBrainAvailability {
    switch availability {
    case .available: .available
    case .unavailable(.deviceNotEligible): .deviceNotEligible
    case .unavailable(.appleIntelligenceNotEnabled): .appleIntelligenceNotEnabled
    case .unavailable(.modelNotReady): .modelNotReady
    @unknown default: .modelNotReady
    }
}
```

Log `on_device_brain_availability` whenever `nextAction` rejects an unavailable model.

- [ ] **Step 4: Run focused availability tests and verify GREEN**

Run the Step 2 command again. Expected: all `OnDeviceBrainTests` pass.

- [ ] **Step 5: Commit the availability increment**

```bash
git add swift/Sources/PhroverKit/Voice/OnDeviceBrain.swift \
  swift/Sources/PhroverKit/Voice/RoverBrain.swift \
  swift/Tests/PhroverKitTests/OnDeviceBrainTests.swift
git diff --cached --check
git commit -m "Expose Apple Intelligence availability reasons"
```

---

### Task 3: Make Apple Intelligence the Primary Brain

**Files:**
- Modify: `swift/Sources/PhroverCloud/Cloud/HybridBrain.swift`
- Create: `swift/Tests/PhroverCloudTests/HybridBrainTests.swift`

**Interfaces:**
- Produces: on-device-first `HybridBrain` with an injected primary-stage timeout for deterministic tests.
- Preserves: cloud configuration remains optional (`cloud: RoverBrain?`) and cloud output still returns through `RoverBrain`.

- [ ] **Step 1: Write failing brain-order tests**

```swift
@MainActor
final class HybridBrainTests: XCTestCase {
    func testUsesOnDeviceFirstWhileOnline() async throws {
        let onDevice = RecordingBrain(result: .success(.init(decision: .done)))
        let cloud = RecordingBrain(result: .success(.init(decision: .say("cloud"))))
        let brain = HybridBrain(cloud: cloud, onDevice: onDevice, isOnline: { true })

        let output = try await brain.nextAction(MissionContext(utterance: "go to the chair"))

        XCTAssertEqual(output.decision, .done)
        XCTAssertEqual(onDevice.callCount, 1)
        XCTAssertEqual(cloud.callCount, 0)
    }
}
```

Add four more explicit tests:

- `testUsesCloudOnlyAfterOnDeviceUnavailable`: make on-device throw `.onDeviceUnavailable(.modelNotReady)`, make cloud return `.done`, and assert both call counts are one and call order is `on_device`, then `cloud`.
- `testUsesCloudAfterPrimaryStageTimeout`: delay on-device for one second, inject a 10 ms primary timeout, return `.done` from cloud, and assert elapsed time is below 500 ms.
- `testRethrowsWhenOnDeviceAndCloudBothFail`: make both brains throw distinct errors and assert the final error includes the cloud failure while both call counts are one.
- `testDoesNotCallCloudWhileOffline`: make on-device fail, set `isOnline` false, assert the on-device error is rethrown and cloud call count remains zero.
- `testUnavailableOnDeviceWithoutConfiguredCloudRethrowsForMissionFallback`: initialize `HybridBrain(cloud: nil, onDevice: ...)`, assert the typed on-device error is rethrown, and verify selection telemetry records the failed on-device stage without inventing a cloud stage.

- [ ] **Step 2: Run HybridBrain tests and verify RED**

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-hybrid-red.xcresult \
  -only-testing:PhroverCloudTests/HybridBrainTests
```

Expected: order test fails because the current implementation is cloud-first.

- [ ] **Step 3: Reverse the selection order with a bounded primary stage**

Implement an internal timeout race local to `HybridBrain`:

```swift
public func nextAction(_ context: MissionContext) async throws -> BrainOutput {
    do {
        let output = try await onDeviceAction(context)
        logSelection("on_device", reason: "primary")
        return output
    } catch {
        guard let cloud, isOnline() else { throw error }
        logSelection("cloud", reason: "on_device_failed", error: error)
        return try await cloud.nextAction(context)
    }
}
```

The stage timeout must cancel its losing task. It must not alter `MissionAgent`'s outer mission-decision timeout, which remains the final guard around the complete configured brain chain.

- [ ] **Step 4: Run HybridBrain tests and verify GREEN**

Run the Step 2 command again. Expected: all HybridBrain order, offline, timeout, and error tests pass.

- [ ] **Step 5: Commit the brain-selection increment**

```bash
git add swift/Sources/PhroverCloud/Cloud/HybridBrain.swift \
  swift/Tests/PhroverCloudTests/HybridBrainTests.swift
git diff --cached --check
git commit -m "Prefer Apple Intelligence before cloud brain"
```

---

### Task 4: Enrich Local Detections with Color Evidence

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/RoverBrain.swift`
- Create: `swift/Sources/PhroverKit/Perception/LocalObjectColorAnalyzer.swift`
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:93-115`
- Create: `swift/Tests/PhroverKitTests/LocalObjectColorAnalyzerTests.swift`
- Modify: `swift/Tests/PhroverKitTests/OnDeviceBrainTests.swift`

**Interfaces:**
- Produces: `ObjectColorEvidence`, optional `PerceivedObject.normalizedBoundingBox`, and local color enrichment from the same pixel buffer RoverYOLO examined.
- Preserves: all existing three-argument `PerceivedObject` call sites via defaulted initializer arguments.

- [ ] **Step 1: Write failing pure color-classification tests**

Keep color math independently testable from camera buffers:

```swift
func testClassifiesBlackRegion() {
    let samples = Array(repeating: RGBSample(red: 0.04, green: 0.05, blue: 0.04), count: 64)
    XCTAssertGreaterThanOrEqual(
        LocalObjectColorAnalyzer.evidence(from: samples)[.black] ?? 0,
        0.70
    )
}
```

Add three more concrete tests:

- `testDoesNotCallDarkBlueBlack`: supply 64 samples with RGB `(0.02, 0.04, 0.16)` and assert `.blue` confidence is greater than `.black` confidence.
- `testClassifiesWhiteGrayAndBrownRegions`: use separate 64-sample arrays `(0.95, 0.95, 0.95)`, `(0.45, 0.46, 0.44)`, and `(0.35, 0.18, 0.07)`; assert the corresponding color is the highest-confidence evidence.
- `testAnalyzerUsesInsetRegionInsteadOfBoundingBoxEdge`: create a small BGRA pixel buffer with a white border and black center, analyze its full normalized box, and assert `.black` confidence is at least `0.70`.

Add an `OnDeviceBrainTests` assertion that a `black` color observation appears in the responder prompt with both confidences.

- [ ] **Step 2: Run focused tests and verify RED**

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-color-red.xcresult \
  -only-testing:PhroverKitTests/LocalObjectColorAnalyzerTests \
  -only-testing:PhroverKitTests/OnDeviceBrainTests
```

Expected: compile failure because color evidence and analyzer types do not exist.

- [ ] **Step 3: Add compatible perception data types**

```swift
public struct ObjectColorEvidence: Equatable, Sendable {
    public let color: LocalObjectColor
    public let confidence: Float
}

public init(
    label: String,
    confidence: Float,
    normalizedPoint: CGPoint,
    normalizedBoundingBox: CGRect? = nil,
    colorEvidence: [ObjectColorEvidence] = []
) {
    self.label = label
    self.confidence = confidence
    self.normalizedPoint = normalizedPoint
    self.normalizedBoundingBox = normalizedBoundingBox
    self.colorEvidence = colorEvidence
}
```

- [ ] **Step 4: Implement local pixel analysis**

`LocalObjectColorAnalyzer` must:

- lock the `CVPixelBuffer` read-only;
- directly support ARKit's bi-planar full-range/video-range YCbCr buffers and BGRA test/fallback buffers;
- convert sampled YCbCr values to RGB on the CPU without submitting Core Image or Metal work, so analysis cannot fail with background GPU permission errors;
- convert Vision's bottom-left normalized bounding box to pixel-buffer coordinates;
- inset the box before sampling to reduce background contamination;
- sample on a bounded grid so inference latency does not scale with object size;
- convert RGB to luminance/saturation/hue;
- return sorted evidence with confidence in `0...1`;
- always unlock the buffer with `defer`.

Expose the pure `evidence(from:)` helper as internal for `@testable import` tests.

- [ ] **Step 5: Wire detector bounding boxes and colors into `ARPerceptionSource`**

```swift
let detections = detector.detect(buffer)
return detections.map { detection in
    PerceivedObject(
        label: detection.label,
        confidence: detection.confidence,
        normalizedPoint: CGPoint(
            x: detection.boundingBox.midX,
            y: detection.boundingBox.midY
        ),
        normalizedBoundingBox: detection.boundingBox,
        colorEvidence: colorAnalyzer.analyze(
            pixelBuffer: buffer,
            normalizedBoundingBox: detection.boundingBox
        )
    )
}
```

Update `OnDeviceBrain.promptText` to emit `black chair (96% object confidence, 82% color confidence)` when evidence exists, and preserve the current format when it does not.

- [ ] **Step 6: Run color and prompt tests and verify GREEN**

Run the Step 2 command again. Expected: all synthetic classifier and prompt tests pass.

- [ ] **Step 7: Commit the perception increment**

Before staging, inspect `git diff -- swift/Sources/PhroverKit/Voice/MissionAgent.swift` and retain the pre-existing depth-heading change.

```bash
git add swift/Sources/PhroverKit/Voice/RoverBrain.swift \
  swift/Sources/PhroverKit/Perception/LocalObjectColorAnalyzer.swift \
  swift/Sources/PhroverKit/Voice/MissionAgent.swift \
  swift/Tests/PhroverKitTests/LocalObjectColorAnalyzerTests.swift \
  swift/Tests/PhroverKitTests/OnDeviceBrainTests.swift
git diff --cached --check
git commit -m "Add local color evidence to object detections"
```

---

### Task 5: Ground and Lock Color-Qualified Visual Targets

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:1338-1621`
- Modify: `swift/Tests/PhroverKitTests/MissionAgentTests.swift`

**Interfaces:**
- Consumes: parsed target label/colors and enriched `PerceivedObject` values.
- Produces: a match only when category and requested-color thresholds pass; the mission's original object query remains locked.

- [ ] **Step 1: Add failing matcher and target-lock regressions**

```swift
func testBlackChairRequiresCategoryAndBlackConfidenceThresholds() {
    let intent = OfflineObjectMissionIntent(
        objectQuery: "black chair",
        targetLabel: "chair",
        requestedColors: [.black],
        searchOtherRooms: false,
        shouldReturn: false
    )
    let grayChair = perceived("chair", object: 0.99, color: (.gray, 0.95))
    let weakBlackChair = perceived("chair", object: 0.99, color: (.black, 0.69))
    let blackChair = perceived("chair", object: 0.90, color: (.black, 0.70))

    XCTAssertEqual(
        MissionAgent.bestVisualTargetMatch(intent: intent, objects: [grayChair, weakBlackChair, blackChair]),
        blackChair
    )
}
```

Add `testUnqualifiedChairDoesNotRequireColorEvidence`, using a `chair` intent with an empty color set and a 0.90-confidence chair with no color evidence; assert it matches. Add `testOriginalBlackChairTargetDoesNotChangeWhenCameraShowsRefrigerator`, begin with no objects, reveal a refrigerator after the first scan and a black chair after the second, then assert two scan turns, one navigation call to the chair point, and no navigation call to the refrigerator point.

- [ ] **Step 2: Run focused mission tests and verify RED**

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-color-grounding-red.xcresult \
  -only-testing:PhroverKitTests/MissionAgentTests
```

Expected: compile failure for the intent-aware matcher or assertion failure because the current matcher ignores color.

- [ ] **Step 3: Add one canonical matching path**

Refactor the current query-token matcher so both brain decisions and fallback intents call one helper:

```swift
static func bestVisualTargetMatch(
    intent: OfflineObjectMissionIntent,
    objects: [PerceivedObject],
    minimumConfidence: Float = 0.90,
    minimumColorConfidence: Float = 0.70
) -> PerceivedObject? {
    objects
        .filter { canonicalLabel($0.label) == canonicalLabel(intent.targetLabel) }
        .filter { $0.confidence >= minimumConfidence }
        .filter { object in
            intent.requestedColors.allSatisfy { requested in
                object.colorEvidence.contains {
                    $0.color == requested && $0.confidence >= minimumColorConfidence
                }
            }
        }
        .max { $0.confidence < $1.confidence }
}
```

The existing `bestVisualTargetMatch(query:objects:minimumConfidence:)` should parse descriptors and delegate, retaining source compatibility for current tests and brain output.

Log `mission_attribute_match` and `mission_attribute_rejected` with target label, requested colors, object confidence, color confidence, and thresholds.

- [ ] **Step 4: Run mission tests and verify GREEN**

Run the Step 2 command again. Expected: the new color tests and every existing scan/lock/alias test pass.

- [ ] **Step 5: Commit the grounding increment**

```bash
git add swift/Sources/PhroverKit/Voice/MissionAgent.swift \
  swift/Tests/PhroverKitTests/MissionAgentTests.swift
git diff --cached --check
git commit -m "Ground visual targets by object and color"
```

---

### Task 6: Fall Back to a Visible Object Mission When Brains Fail

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:862-1215`
- Modify: `swift/Tests/PhroverKitTests/MissionAgentTests.swift`

**Interfaces:**
- Produces: fallback invocation only before the first usable brain decision.
- Reuses: `scanForUnresolvedVisualTarget`, `navigate(to:for:)`, visual target stop distance, mission cancellation, and terminal status publication.

- [ ] **Step 1: Add failing visible-target fallback mission tests**

Add three tests with a brain that throws `RoverBrainError.onDeviceUnavailable(.modelNotReady)`:

```swift
func testBrainUnavailableFallsBackToVisibleRefrigeratorAndDoesNotReturn() async {
    let fixture = makeMissionFixture(
        objects: [perceived("refrigerator", object: 0.99)],
        brainError: RoverBrainError.onDeviceUnavailable(.modelNotReady),
        navigationOutcomes: [.arrived]
    )

    await fixture.agent.handle("Go to the refrigerator")

    XCTAssertEqual(fixture.motion.navigateCalls.count, 1)
    XCTAssertEqual(fixture.motion.stopClearances, [RoverConfig.visualTargetStopDistance])
    XCTAssertFalse(fixture.motion.navigateCalls.contains(fixture.startPose.position))
    guard case .succeeded = fixture.terminalStatus else {
        return XCTFail("Expected a successful terminal status")
    }
}
```

Also test `Go to the refrigerator and come back` produces outbound then start-pose navigation, and `Tell me a joke` fails without rotate or navigate calls.

- [ ] **Step 2: Run focused mission tests and verify RED**

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-visible-fallback-red.xcresult \
  -only-testing:PhroverKitTests/MissionAgentTests
```

Expected: current code publishes the generic brain failure and never navigates.

- [ ] **Step 3: Introduce a first-decision fallback boundary**

Track whether a brain has produced any output. In the missing-brain, typed-unavailable, both-brains-failed, and outer-timeout paths, call the parser only when no usable output has been produced:

```swift
private func attemptOfflineObjectFallback(
    utterance: String,
    missionID: Int
) async -> OfflineFallbackOutcome {
    guard let intent = OfflineObjectMissionIntentParser.parse(utterance) else {
        RuntimeFileLog.append("mission_offline_fallback_rejected", fields: [
            "mission": "\(missionID)", "reason": "unsupported_command"
        ])
        return .unsupported
    }
    RuntimeFileLog.append("mission_offline_fallback_started", fields: [
        "mission": "\(missionID)", "target": intent.objectQuery
    ])
    return await runOfflineObjectMission(intent, missionID: missionID)
}
```

Do not fall back after a brain has already caused motion; that would mix planners mid-mission. Do not catch emergency-stop cancellation as a brain failure.

- [ ] **Step 4: Implement the visible-object leg by reusing existing motion helpers**

`runOfflineObjectMission` must lock the parsed intent once, check current detections, use the existing bounded 20-30 degree fresh-frame scan if needed, unproject the matched object, navigate with `visualTargetStopDistance`, and publish success only after `.arrived`.

If `shouldReturn` is true, call the existing return alignment/navigation helper using the command's `memory.missionStartPose`. If false, finish at the object.

- [ ] **Step 5: Run mission tests and verify GREEN**

Run the Step 2 command again. Expected: visible refrigerator, explicit return, unsupported command, emergency stop, and existing brain-driven tests all pass.

- [ ] **Step 6: Commit the visible fallback increment**

```bash
git add swift/Sources/PhroverKit/Voice/MissionAgent.swift \
  swift/Tests/PhroverKitTests/MissionAgentTests.swift
git diff --cached --check
git commit -m "Fallback to offline visible object missions"
```

---

### Task 7: Search Other Rooms and Record Doorway Routes

**Files:**
- Modify: `swift/Sources/PhroverKit/Topology/RoomTopologyModels.swift`
- Modify: `swift/Sources/PhroverKit/Topology/SessionRoomTopology.swift`
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift:462-858`
- Modify: `swift/Tests/PhroverKitTests/SessionRoomTopologyTests.swift`
- Modify: `swift/Tests/PhroverKitTests/MissionAgentTests.swift`

**Interfaces:**
- Produces: typed doorway route steps, shortest path lookup, and a reusable one-doorway traversal primitive.
- Reuses: existing candidate ranking, reachability, transition confirmation, slow scan, and safety handling.

- [ ] **Step 1: Add failing topology route tests**

Build a three-room graph through existing confirmed transitions, then assert:

```swift
func testShortestDoorwayPathReturnsForwardAndReverseStepsWithoutCycles() throws {
    let topology = makeThreeRoomTopology()

    let outward = topology.shortestDoorwayPath(from: room1, to: room3)
    XCTAssertEqual(outward?.map(\.fromRoomID), [room1, room2])
    XCTAssertEqual(outward?.map(\.toRoomID), [room2, room3])

    let returning = topology.shortestDoorwayPath(from: room3, to: room1)
    XCTAssertEqual(returning, outward?.reversed().map(\.reversed))
}
```

Also assert unknown/disconnected rooms return `nil` and a same-room path is empty.

- [ ] **Step 2: Run topology tests and verify RED**

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-topology-route-red.xcresult \
  -only-testing:PhroverKitTests/SessionRoomTopologyTests
```

Expected: compile failure because route types/query do not exist.

- [ ] **Step 3: Add typed route steps and breadth-first path lookup**

```swift
public struct DoorwayRouteStep: Equatable, Sendable {
    public let doorwayID: DoorwayID
    public let fromRoomID: RoomID
    public let toRoomID: RoomID

    public var reversed: DoorwayRouteStep {
        .init(doorwayID: doorwayID, fromRoomID: toRoomID, toRoomID: fromRoomID)
    }
}
```

Add `shortestDoorwayPath(from:to:) -> [DoorwayRouteStep]?` to `RoomTopologyManaging` and implement deterministic BFS ordered by `DoorwayID`. Never cross session generations.

Provide a protocol-extension default that returns `nil` so coordinator fakes that do not own a room graph remain source compatible; `SessionRoomTopology` supplies the real BFS implementation.

- [ ] **Step 4: Run topology tests and verify GREEN**

Run the Step 2 command again. Expected: route and all existing transition tests pass.

- [ ] **Step 5: Add failing other-room mission tests**

Test a scripted perception/topology sequence where the current-room scan has no chair, a doorway is crossed, a fresh frame reports a black chair at `>= 90%` category and `>= 70%` black confidence, and navigation arrives. Assert:

- each scan turn is `20-30` degrees and waits for a newer frame;
- the selected doorway is marked visited/searched;
- the locked target remains `black chair`;
- no cloud or second brain call is needed;
- no return occurs without return language;
- explicit return traverses the same doorway in reverse, then reaches the start pose;
- exhausted room/opening budgets fail and stop.

- [ ] **Step 6: Run mission tests and verify RED**

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-other-room-red.xcresult \
  -only-testing:PhroverKitTests/MissionAgentTests
```

Expected: current fallback stops after local scanning and cannot traverse/return through typed doorway routes.

- [ ] **Step 7: Extract one reusable room-transition attempt**

Refactor `runRoomTransitionMission` without changing its current user-facing behavior:

```swift
private enum RoomTraversalOutcome {
    case crossed(DoorwayRouteStep)
    case exhausted(String)
    case cancelled
}

private func traverseNextDoorway(
    preferredDoorwayID: DoorwayID? = nil,
    excluding: Set<DoorwayCandidateID>,
    missionID: Int
) async -> RoomTraversalOutcome
```

The standalone `go to another room` path becomes a thin wrapper that calls this helper and publishes its existing terminal status. The offline object search calls the same helper without prematurely completing the mission.

- [ ] **Step 8: Implement bounded room search and explicit reverse return**

For the fallback executor:

1. record start room, start pose, and session generation;
2. perform one bounded local scan;
3. traverse the highest-ranked safe unsearched doorway;
4. record each `DoorwayRouteStep` and rescan only after fresh tracked frames;
5. stop when the target is confirmed or the room/opening budget is exhausted;
6. when explicitly requested, obtain the current-to-start route and traverse each preferred doorway;
7. in the start room, reuse `navigateReturn(to:missionID:)` for the final pose.

Abort if the session generation changes. Log room searched, doorway crossed, return route start/step/completion, and the final failure reason.

- [ ] **Step 9: Run topology and mission tests and verify GREEN**

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-other-room-green.xcresult \
  -only-testing:PhroverKitTests/SessionRoomTopologyTests \
  -only-testing:PhroverKitTests/MissionAgentTests
```

Expected: all new other-room/return tests and existing transition recovery tests pass.

- [ ] **Step 10: Commit room search and route support**

```bash
git add swift/Sources/PhroverKit/Topology/RoomTopologyModels.swift \
  swift/Sources/PhroverKit/Topology/SessionRoomTopology.swift \
  swift/Sources/PhroverKit/Voice/MissionAgent.swift \
  swift/Tests/PhroverKitTests/SessionRoomTopologyTests.swift \
  swift/Tests/PhroverKitTests/MissionAgentTests.swift
git diff --cached --check
git commit -m "Search other rooms and return through doorway routes"
```

---

### Task 8: Surface Brain Diagnostics and Complete Runtime Telemetry

**Files:**
- Modify: `examples/PhroverOperator/PhroverOperator/Views/ConversationView.swift:21-119`
- Modify: `examples/PhroverOperator/PhroverOperatorUITests/PhroverOperatorUITests.swift`
- Modify: `swift/Sources/PhroverKit/Voice/RoverBrain.swift`
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift`
- Modify: `swift/Sources/PhroverCloud/Cloud/HybridBrain.swift`
- Modify: `swift/Tests/PhroverKitTests/MissionAgentTests.swift`
- Modify: `swift/Tests/PhroverCloudTests/HybridBrainTests.swift`

**Interfaces:**
- Produces: exact Talk-screen Apple Intelligence state and complete mission selection/search/return logs.
- Preserves: push-to-talk remains fixed and hittable above the tab bar.

- [ ] **Step 1: Add failing telemetry and UI diagnostic tests**

Add a public `MissionTelemetrySink` type alias beside `RoverBrain`, with a default closure that forwards to `RuntimeFileLog.append`. Inject the same sink into `HybridBrain` and `MissionAgent`. In tests, append events to an array and assert exact event ordering for unavailable-model fallback:

```swift
XCTAssertEqual(events.map(\.name), [
    "mission_brain_selected",
    "mission_offline_fallback_started",
    "mission_attribute_match",
])
```

Add Debug launch arguments `-ui-test-brain-model-not-ready` and `-ui-test-brain-available`, then assert the Talk screen shows a concise diagnostic only for the unavailable fixture while `push-to-talk-control` remains hittable.

- [ ] **Step 2: Run focused tests and verify RED**

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-telemetry-red.xcresult \
  -only-testing:PhroverKitTests/MissionAgentTests

xcodebuild test \
  -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -destination 'platform=iOS Simulator,name=iPhone 16 Pro' \
  -derivedDataPath /private/tmp/phrover-brain-ui-red \
  -only-testing:PhroverOperatorUITests/PhroverOperatorUITests/testTalkShowsModelNotReadyDiagnostic
```

Expected: missing events/fixture/diagnostic cause focused failures.

- [ ] **Step 3: Wire availability into Talk initialization**

Store the exact availability before constructing the agent:

```swift
@State private var brainAvailability: OnDeviceBrainAvailability = .modelNotReady

let onDevice = OnDeviceBrain()
brainAvailability = onDevice.availability
RuntimeFileLog.append("on_device_brain_availability", fields: [
    "state": brainAvailability.logValue
])
let brain: RoverBrain = HybridBrain(cloud: cloudBrain, onDevice: onDevice)
```

Render `brainAvailability.operatorMessage` in the scrollable status region, not over the mic. Availability must not disable push-to-talk because supported object missions can use the offline fallback.

- [ ] **Step 4: Complete structured telemetry**

Ensure the implementation emits the design events with mission ID and reason. Existing motion logging remains responsible for pose, goal, distance, wheel command, request URL, and response status; do not duplicate or remove those fields.

- [ ] **Step 5: Run focused telemetry/UI tests and verify GREEN**

Run the Step 2 commands again. Expected: focused SDK and UI tests pass.

- [ ] **Step 6: Commit diagnostics and telemetry**

```bash
git add examples/PhroverOperator/PhroverOperator/Views/ConversationView.swift \
  examples/PhroverOperator/PhroverOperatorUITests/PhroverOperatorUITests.swift \
  swift/Sources/PhroverKit/Voice/RoverBrain.swift \
  swift/Sources/PhroverKit/Voice/MissionAgent.swift \
  swift/Sources/PhroverCloud/Cloud/HybridBrain.swift \
  swift/Tests/PhroverKitTests/MissionAgentTests.swift \
  swift/Tests/PhroverCloudTests/HybridBrainTests.swift
git diff --cached --check
git commit -m "Expose object mission fallback diagnostics"
```

---

### Task 9: Full Regression, Device Installation, and Offline Acceptance

**Files:**
- Verify only: all files changed in Tasks 1-8
- Device logs: iPhone app container `Documents`
- Update after runtime proof: `docs/phrover-fixes-2026-07-08.md`

**Acceptance boundary:** Unit and simulator tests prove deterministic behavior; the feature is not complete until a physical iPhone run proves Foundation Models availability reporting, camera/LiDAR grounding, rover HTTP transport, stop distance, room traversal, and explicit return.

- [ ] **Step 1: Run focused suites together**

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-offline-object-focused.xcresult \
  -only-testing:PhroverKitTests/OfflineObjectMissionIntentParserTests \
  -only-testing:PhroverKitTests/OnDeviceBrainTests \
  -only-testing:PhroverKitTests/LocalObjectColorAnalyzerTests \
  -only-testing:PhroverKitTests/SessionRoomTopologyTests \
  -only-testing:PhroverKitTests/MissionAgentTests \
  -only-testing:PhroverCloudTests/HybridBrainTests
```

Expected: all focused suites pass with zero failures.

- [ ] **Step 2: Run the full iOS SDK suite**

```bash
scripts/test-swift-sdk.sh -quiet \
  -resultBundlePath /private/tmp/phrover-offline-object-full.xcresult
```

Expected: all RoverNav, PhroverKit, and PhroverCloud tests pass. Report unrelated baseline failures separately; do not hide them by rerunning only focused tests.

- [ ] **Step 3: Check the complete diff and build the app**

```bash
git diff --check
git status --short
xcodebuild build \
  -project examples/PhroverOperator/PhroverOperator.xcodeproj \
  -scheme PhroverOperator \
  -destination 'generic/platform=iOS' \
  -derivedDataPath /private/tmp/phrover-offline-object-build \
  -allowProvisioningUpdates
```

Expected: whitespace check exits 0 and Xcode reports `BUILD SUCCEEDED`.

- [ ] **Step 4: Install and launch on the connected iPhone 15 Pro**

First discover the current identifier rather than assuming it is unchanged:

```bash
xcrun devicectl list devices
xcrun devicectl device install app \
  --device FC11C836-4978-5B20-9170-16EAD18568BE \
  /private/tmp/phrover-offline-object-build/Build/Products/Debug-iphoneos/PhroverOperator.app
xcrun devicectl device process launch \
  --device FC11C836-4978-5B20-9170-16EAD18568BE \
  bot.coy.phrover
```

Expected: install and launch succeed. If the device identifier or bundle identifier differs, use the values returned by discovery/build output and record them.

- [ ] **Step 5: Run the four physical acceptance missions**

With rover Wi-Fi connected and adequate open space:

1. `Go to the refrigerator` -> stops about 0.30 m away and remains there.
2. `Go to the refrigerator and come back` -> reaches it, then returns to command start.
3. `Go to the black chair in the other room` -> searches with slow fresh-frame scans, crosses a safe doorway, and remains at the matching chair.
4. Repeat command 3 with `and come back` while cloud is unavailable -> reverses the doorway route and returns to command start.

Use the UI E-stop immediately if physical clearance, person safety, or steering behavior is incorrect.

- [ ] **Step 6: Pull and inspect device logs after each mission**

```bash
DEST=$(mktemp -d /private/tmp/phrover-offline-object-log-XXXXXX)
xcrun devicectl device copy from \
  --device FC11C836-4978-5B20-9170-16EAD18568BE \
  --domain-type appDataContainer \
  --domain-identifier bot.coy.phrover \
  --source Documents \
  --destination "$DEST"
rg -n "on_device_brain_availability|mission_brain_selected|mission_offline_fallback|mission_attribute|mission_room|mission_doorway|mission_return|mission_(completed|failed)|rover_request" "$DEST"
```

Expected evidence:

- exact Apple Intelligence availability is logged;
- Apple Intelligence is selected first when available;
- offline fallback starts only when configured brain stages fail before a first decision;
- target label/color remain locked across camera turns;
- scan turns are bounded and followed by fresh frames;
- terminal target distance is approximately 0.30 m;
- return events appear only for commands that explicitly requested return;
- every rover request has URL/result/status evidence and no motion follows a terminal failure.

- [ ] **Step 7: Record verified behavior in the fixes document**

Append a dated section to `docs/phrover-fixes-2026-07-08.md` containing only observed device results, exact log event names, test/build commands, and remaining limitations. Do not claim other-room or return success unless logs and physical behavior both prove it.

- [ ] **Step 8: Commit final verification documentation**

```bash
git add docs/phrover-fixes-2026-07-08.md
git diff --cached --check
git commit -m "Document offline object mission verification"
```

## Completion Criteria

- Apple Intelligence is demonstrably the first brain selected when available.
- Its exact unavailable reason is visible and logged when it cannot run.
- Optional cloud is called only after the on-device stage fails and never becomes an offline dependency.
- Visible refrigerator missions work with and without explicit return.
- `black chair in the other room` uses local category/color evidence, slow fresh-frame scans, safe doorway traversal, and bounded search.
- The target query cannot drift when the camera view changes.
- Return happens only when explicitly requested and follows the recorded topology route before final start-pose navigation.
- Existing speech retry, depth safety, target stopping, navigation recovery, room-transition, and rover communication tests remain green.
- Physical device logs corroborate brain selection, target lock, motion requests, stop distance, room traversal, and terminal state.
