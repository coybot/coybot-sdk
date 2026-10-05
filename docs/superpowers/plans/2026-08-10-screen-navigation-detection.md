# Screen Navigation Detection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an offline, navigation-grade screen detector whose localized result appears in Talk diagnostics and can be targeted by commands such as `Go to the monitor`.

**Architecture:** Keep `RoverYOLO` as the primary COCO detector. A screen-only Vision request using the same bundled model scans alternate orientations when primary results contain no screen-like object; a pure policy layer validates, canonicalizes, and deduplicates fallback results before `ARPerceptionSource` converts them into the existing `PerceivedObject` navigation path. Voice parsing and target matching normalize screen aliases to one `screen` category.

**Tech Stack:** Swift 6, Vision, Core ML, Create ML, XCTest, Python 3 standard library, Open Images V7 bounding-box annotations.

## Global Constraints

- Detection and navigation must work offline.
- Secondary detections require a localized bounding box and confidence of at least `0.90`.
- Secondary inference runs at no more than `2 Hz`, never overlaps itself, and is disabled while the app is inactive.
- Existing obstacle, stale-depth, and 30 cm stopping rules remain unchanged.
- Existing non-screen COCO detections must never be removed by fallback failure.
- The model acceptance gate is 90% recall, 95% precision, zero curated hard-negative detections, the supplied monitor fixture at 90% or above, and p95 inference at or below 300 ms on iPhone 15 Pro.

---

### Task 1: Pure Screen Detection Policy

**Files:**
- Create: `swift/Sources/PhroverKit/Perception/ScreenDetectionPolicy.swift`
- Test: `swift/Tests/PhroverKitTests/ScreenDetectionPolicyTests.swift`

**Interfaces:**
- Consumes: `[Detector.Detection]` from primary and fallback detectors.
- Produces: `ScreenDetectionPolicy.shouldRunFallback(for:)` and `ScreenDetectionPolicy.merge(primary:fallback:minimumConfidence:)`.

- [ ] **Step 1: Write failing policy tests**

Cover fallback suppression for `tv`, `laptop`, and `screen`; acceptance of a valid 0.90 `screen`; rejection of weak/wrong/invalid-box results; preservation of non-screen primary objects; and overlap deduplication by intersection-over-union.

- [ ] **Step 2: Verify RED**

Run: `scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/ScreenDetectionPolicyTests`

Expected: compilation fails because `ScreenDetectionPolicy` does not exist.

- [ ] **Step 3: Implement the minimal pure policy**

```swift
enum ScreenDetectionPolicy {
    static func shouldRunFallback(for primary: [Detector.Detection]) -> Bool
    static func merge(primary: [Detector.Detection],
                      fallback: [Detector.Detection],
                      minimumConfidence: Float = 0.90) -> [Detector.Detection]
}
```

Canonical fallback output is `screen`; reject non-finite, empty, out-of-range boxes and deduplicate screen-like boxes at IoU `>= 0.50`.

- [ ] **Step 4: Verify GREEN**

Run the focused test command, then `scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/DetectorTests`.

---

### Task 2: Vision Screen Localizer And Detector Composition

**Files:**
- Create: `swift/Sources/PhroverKit/Perception/ScreenLocalizer.swift`
- Modify: `swift/Sources/PhroverKit/Perception/Detector.swift`
- Modify: `Package.swift`
- Test: `swift/Tests/PhroverKitTests/DetectorTests.swift`

**Interfaces:**
- Consumes: `CVPixelBuffer`, `ScreenDetectionPolicy`, bundled `RoverYOLO` model.
- Produces: `ScreenLocalizing.detect(_:orientation:) -> [Detector.Detection]`, `Detector.setInferenceEnabled(_:)`, and composed `Detector.detect(_:)` results.

- [ ] **Step 1: Write failing composition tests**

Inject a deterministic `ScreenLocalizing` test implementation. Prove fallback runs only when primary lacks a screen-like result, failure preserves primary results, 500 ms rate limiting reuses primary-only output, and disabling inference prevents fallback calls.

- [ ] **Step 2: Verify RED**

Run: `scripts/test-swift-sdk.sh -only-testing:PhroverKitTests/DetectorTests`

Expected: compilation fails because the localizer injection and lifecycle API do not exist.

- [ ] **Step 3: Implement Vision localizer and composition**

`VisionScreenLocalizer` loads `RoverYOLO` with `.cpuAndNeuralEngine`, performs `VNCoreMLRequest` with `.scaleFill`, filters each orientation to screen-like labels, maps observations into `Detector.Detection`, logs accepted/rejected/failure events, and never changes primary detections on error. `Detector` uses a monotonic clock injection for deterministic rate-limit tests.

- [ ] **Step 4: Keep the accepted model resource**

Continue packaging only `Resources/RoverYOLO.mlpackage`. Do not add the rejected
`ScreenYOLO` candidate.

- [ ] **Step 5: Verify GREEN**

Run focused Detector and policy tests.

---

### Task 3: Screen Voice Aliases And Existing Navigation Path

**Files:**
- Modify: `swift/Sources/PhroverKit/Voice/OfflineObjectMissionIntent.swift`
- Modify: `swift/Sources/PhroverKit/Voice/MissionAgent.swift`
- Test: `swift/Tests/PhroverKitTests/OfflineObjectMissionIntentParserTests.swift`
- Test: `swift/Tests/PhroverKitTests/MissionAgentTests.swift`

**Interfaces:**
- Consumes: voice target labels and `PerceivedObject` labels.
- Produces: canonical target label `screen` for monitor/display/screen/television/tv aliases, while allowing primary `laptop` to satisfy a screen request.

- [ ] **Step 1: Write failing alias tests**

Prove `Go to the monitor`, `Go to the computer screen`, and `Go to the television` parse to `screen`; prove screen requests match detected `screen`, `tv`, and `laptop` objects at 0.90 or higher; prove unrelated objects do not match.

- [ ] **Step 2: Verify RED**

Run the two focused test classes and confirm alias assertions fail.

- [ ] **Step 3: Implement canonical screen-category matching**

Add phrase-level canonicalization before token comparison so multiword aliases collapse to `screen`. Keep exact category matching for every non-screen object.

- [ ] **Step 4: Verify GREEN**

Run the two focused test classes.

---

### Task 4: Reproducible Dataset And Model Evaluation

**Files:**
- Create: `scripts/screen-detector/prepare_open_images.py`
- Create: `scripts/screen-detector/train_screen_detector.swift`
- Create: `scripts/screen-detector/README.md`
- Create: `swift/Tests/PhroverKitTests/Fixtures/ScreenDetector/**`

**Interfaces:**
- Consumes: Open Images V7 class descriptions, bounding-box CSVs, image URLs, and local rover fixtures.
- Produces: Create ML JSON datasets, candidate model metrics, and deterministic acceptance fixtures. A model asset is produced only after acceptance.

- [ ] **Step 1: Write failing preparation tests**

Use Python `unittest` fixtures to prove class filtering, normalized-to-pixel box conversion, one-class relabeling, negative-image retention, image-ID partition isolation, and SHA-256 verification.

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest scripts/screen-detector/test_prepare_open_images.py`

Expected: import fails because the preparation module does not exist.

- [ ] **Step 3: Implement deterministic preparation**

The script accepts explicit metadata paths and manifest files, downloads only listed image IDs, verifies hashes, emits one Create ML `annotations.json` per split, and records source URL/license/attribution metadata. It never silently accepts an image without verified provenance.

- [ ] **Step 4: Implement Create ML training and evaluation**

The Swift script loads train/validation directories with `directoryWithImagesAndJsonAnnotation`, trains `MLObjectDetector` using explicit validation data, evaluates the held-out test directory, writes `ScreenYOLO.mlmodel`, upgrades it to the repository `.mlpackage`, and emits metrics plus SHA-256 metadata.

- [ ] **Step 5: Build and validate the model**

Prepare the pinned data, train, and run the held-out and hard-negative acceptance gates. The evaluated candidate failed with held-out mAP 0, so it is rejected and not added to the app. The supplied monitor fixture instead verifies the accepted `RoverYOLO` orientation fallback.

---

### Task 5: Lifecycle, Diagnostics, Full Verification, And Device Trial

**Files:**
- Modify: `examples/PhroverOperator/PhroverOperator/Views/ConversationView.swift`
- Modify: `docs/phrover-fixes-2026-07-08.md`
- Test: `examples/PhroverOperator/PhroverOperatorUITests/PhroverOperatorUITests.swift`

**Interfaces:**
- Consumes: app scene activity and composed detector output.
- Produces: no background fallback inference, `Visible: screen NN%`, and navigation through the existing target workflow.

- [ ] **Step 1: Write failing lifecycle/UI tests**

Prove inactive scene state disables detector inference and an accepted `screen` object appears in the existing visible-object summary without a second UI data path.

- [ ] **Step 2: Verify RED**

Run the focused UI/unit tests and confirm the lifecycle assertion fails.

- [ ] **Step 3: Wire scene lifecycle and document events**

Call `setInferenceEnabled(false)` before backgrounding and `true` after activation. Document model load, fallback start, accept/reject, stale/rate-limited, and failure log events.

- [ ] **Step 4: Run repository verification**

Run focused tests, `scripts/test-swift-sdk.sh`, `git diff --check`, and an iPhone Release/Debug build for the physical destination.

- [ ] **Step 5: Install and validate on iPhone 15 Pro**

Install with `xcrun devicectl`, keep the rover physically constrained, confirm the supplied angled monitor yields `Visible: screen 90%` or higher, then issue `Go to the screen` and verify direction adjustment plus stop at 30 cm before an unconstrained floor trial.
