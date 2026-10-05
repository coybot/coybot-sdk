# Phrover Screen Detector

This directory evaluates a possible dedicated offline `ScreenYOLO` fallback.
The current app does not ship this candidate: the evaluated 30-iteration model
had held-out mAP 0 and failed the acceptance gate. Production instead performs
a screen-only multi-orientation scan with the bundled `RoverYOLO` model.

## Prepare Data

Download the Open Images V7 validation/test bounding-box CSVs and corresponding
image metadata CSVs, then run:

```bash
SSL_CERT_FILE=/etc/ssl/cert.pem python3 scripts/screen-detector/prepare_open_images.py \
  --annotations /path/to/validation-annotations-bbox.csv \
  --annotations /path/to/test-annotations-bbox.csv \
  --metadata /path/to/validation-images-with-rotation.csv \
  --metadata /path/to/test-images-with-rotation.csv \
  --output /tmp/phrover-screen-dataset
```

The preparer emits deterministic split manifests and `provenance.json`. On a
repeat build, provide all three generated manifests with `--train-manifest`,
`--validation-manifest`, and `--test-manifest` to pin the exact image IDs.

## Train

```bash
SWIFT_MODULECACHE_PATH=/private/tmp/phrover-swift-cache \
CLANG_MODULE_CACHE_PATH=/private/tmp/phrover-clang-cache \
xcrun swift scripts/screen-detector/train_screen_detector.swift \
  --dataset /tmp/phrover-screen-dataset \
  --output /tmp/phrover-screen-model \
  --max-iterations 30

xcrun coremlcompiler upgrade \
  /tmp/phrover-screen-model/ScreenYOLO.mlmodel \
  /tmp/phrover-screen-model
```

Do not replace the bundled model unless held-out data, curated hard negatives,
the supplied monitor fixture, and physical-device latency meet the acceptance
thresholds in the screen-detection design document.
