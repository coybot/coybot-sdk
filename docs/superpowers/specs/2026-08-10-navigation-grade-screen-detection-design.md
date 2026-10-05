# Navigation-Grade Screen Detection Design

## Goal

Recognize computer monitors, displays, televisions, and screens in the live iPhone camera view and provide a localized bounding box that the rover can use as a navigation target. Detection and navigation must work offline.

## Root Cause

The bundled COCO YOLO model recognizes `tv` and `laptop`, but it does not contain `monitor`, `display`, or `screen` classes. The detector used to stop orientation fallback after the first nonempty result, even when that result was unrelated to the requested screen category. On the supplied screenshot, `.right` produced a false `stop sign`, while `.up` correctly localized the monitor as `laptop` at 95.9%. The early exit therefore hid a valid offline detection from both the Talk diagnostics and navigation.

## Rejected Built-In Vision Fallback

Apple Vision saliency plus image classification was evaluated against the supplied monitor frame. It classified the displayed content as `document`, `printed_page`, and `screenshot`, but did not identify a monitor. The bundled YOLO model also returned no `tv` box even when its confidence threshold was reduced from 25% to 5%.

Promoting those classifications to `screen` would create uncalibrated false targets for printed paper, windows, and posters. That approach is therefore rejected for rover navigation.

## Approach

Keep the existing COCO YOLO model as the primary detector. When its preferred orientation returns no screen-like object, run a screen-only fallback over all supported image orientations using the same bundled `RoverYOLO` model. Ignore unrelated labels during this fallback, continue until a `tv` or `laptop` candidate is found, then canonicalize an accepted result to `screen`.

This preserves the existing 80-class detector, works fully offline, and fixes the observed failure without shipping an unvalidated model.

## Components

### ScreenLocalizing

Add a small protocol representing the secondary detector:

- Input: a camera `CVPixelBuffer`.
- Output: zero or more `Detector.Detection` values.
- Production implementation: a Vision request backed by the bundled `RoverYOLO.mlpackage`, filtered to screen-like labels for each orientation.
- Tests can supply deterministic candidate regions and labels without invoking Vision models.

### Dedicated Model Experiment

A reproducible Open Images/Create ML training path was evaluated for a dedicated `ScreenYOLO` model. The repository tooling records:

- Training source and image/annotation licenses.
- Training and validation class counts.
- Export tool and version.
- Input dimensions and confidence/IoU defaults.
- Validation precision, recall, and false-positive results for hard-negative scenes.

The 30-iteration transfer-learning candidate failed acceptance with held-out mAP of 0 and was not packaged. A dedicated model may replace the current fallback only after it passes a held-out validation set containing both screens and visually similar non-screen rectangles.

### Dataset And Export Pipeline

Use the Open Images V7 bounding-box annotations as the reproducible starting dataset:

- Positive classes: `Computer monitor` (`/m/02522`) and `Television` (`/m/07c52`).
- Hard-negative classes: `Laptop` (`/m/01c648`), `Poster` (`/m/01n5jq`), `Whiteboard` (`/m/02d9qx`), `Door` (`/m/02dgv`), `Picture frame` (`/m/06z37_`), `Tablet computer` (`/m/0bh9flk`), `Book` (`/m/0bt_c3`), and `Window` (`/m/0d4v4`).
- Additional local hard negatives: printed pages, wall art, empty walls, and windows captured from the rover's operating environment.

Include angled, truncated, partially occluded, bright, dark, and content-heavy screens in the positive set. Collapse both positive source classes into the single `screen` training label. Keep negative images annotation-free unless they contain a positive-class screen.

The repository script downloads explicitly selected Open Images image IDs and annotations and converts normalized boxes into Create ML object-detector annotations. Split by source image ID so near-duplicate frames cannot cross partitions.

Train with Create ML's `MLObjectDetector` and evaluate before packaging. Source images are not committed to the SDK repository. Every included image must have its license and attribution recorded; images whose license cannot be verified are excluded.

### Detector

`Detector.detect` remains the public object-detection entry point.

- Run YOLO using the current orientation fallback.
- If YOLO returns `tv`, `laptop`, or `screen`, return those detections without running the secondary localizer.
- Otherwise run the screen-only orientation localizer and append accepted `screen` detections to the primary results.
- Suppress overlapping duplicate screen detections.
- Preserve all existing non-screen YOLO detections.

### Target Normalization

Normalize voice targets `monitor`, `display`, `computer monitor`, `computer screen`, `television`, and `tv` to the canonical target `screen`. Existing detected `tv` and `laptop` labels must remain matchable as screen aliases so commands do not depend on which detector produced the result.

### Navigation

Screen detections use the existing object-target workflow:

- Lock only after satisfying the mission confidence requirement.
- Use the localized bounding box for depth-aware unprojection.
- Retain obstacle and stale-depth safety checks.
- Stop at the existing object approach distance, currently 30 cm.
- Do not weaken safety checks when the screen classifier is uncertain.

## Confidence And Safety

The secondary detector must reject weak model outputs. A candidate is accepted only when:

- The model output has the canonical `screen` class.
- Its confidence meets a configurable screen-detection threshold.
- Its normalized bounding box is valid and nontrivial.

The confidence exposed to mission targeting is the detector confidence, clamped to `0...1`. The existing mission threshold remains authoritative. A classification without a localized bounding box must not become a navigable object.

Run the secondary detector at no more than 2 Hz while searching. A new frame must not start screen inference while the prior request is still running. Stop submitting work when the app is inactive, and discard a result if its frame timestamp is stale when navigation consumes it.

The initial screen acceptance threshold is 0.90. It may only be lowered after the held-out test report demonstrates that the replacement threshold still meets every acceptance gate below.

## Diagnostics

Add runtime events for:

- Screen-model inference started after primary YOLO lacked a screen-like object.
- Candidate accepted with label, confidence, and bounding box.
- Candidate rejected with a concise reason.
- Secondary localization failure.

The Talk screen's `Visible` line continues to summarize `RoverPerception.detectObjects()`, so an accepted candidate appears as `screen NN%` without a separate UI-only data path.

## Error Handling

- Model loading and Vision request errors are logged and produce no secondary detections.
- Missing or malformed model outputs produce no secondary detections.
- Secondary detection failure never removes valid YOLO detections.
- App backgrounding or cancellation must not submit new inference work.

## Testing

Use test-driven development with focused tests for:

1. Screen-only fallback results produce a canonical `screen` detection.
2. Unrelated or below-threshold labels are rejected.
3. Accepted detections preserve normalized bounding boxes and confidence.
4. Existing YOLO `tv`, `laptop`, or `screen` results skip secondary localization.
5. Secondary results append without removing non-screen YOLO objects.
6. Overlapping screen detections are deduplicated.
7. Voice aliases match `screen`, `tv`, and `laptop` detections.
8. Secondary model load or inference failure leaves existing YOLO detections intact.
9. The bundled model resource loads successfully on iOS.
10. The supplied monitor regression fixture is found after unrelated results from other orientations are filtered out.

The model acceptance report must demonstrate, at the configured 0.90 threshold:

- At least 90% recall on the held-out positive test set.
- At least 95% precision on the held-out positive and hard-negative test set combined.
- Zero accepted detections across the curated rover-environment hard-negative fixtures.
- A `screen` detection at 90% confidence or higher for the supplied angled-monitor fixture.
- Fallback inference p95 of 300 ms or less on the connected iPhone 15 Pro, measured while the app is active.

After automated tests pass, build and install on the connected iPhone 15 Pro. Verify the supplied monitor view displays a localized screen detection, then issue `Go to the screen` while the rover is raised or otherwise physically constrained before conducting a floor-navigation test.

## Out Of Scope

- Cloud vision as a requirement.
- General open-vocabulary detection for arbitrary objects.
- Replacing or retraining the existing 80-class YOLO model.
- Weakening obstacle, depth, or stopping-distance safety rules.
