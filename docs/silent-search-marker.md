# Silent Search Calibration Marker

The calibration marker payload is `PHROVER-CAL|1|SILENT_SEARCH_01`. Its printed north arrow defines mission north; with Vision image orientation `.right`, the QR `topLeft` to `topRight` edge must face that arrow.

Print `assets/silent-search-calibration-marker.pdf` at **100% / actual size** on A3 paper. Disable fit-to-page, scaling, and printer margin adjustment. After printing, use a ruler to verify that the QR's outer black-module square is exactly **200 mm by 200 mm** before using the marker.

Regenerate and validate the committed artifact from the repository root with:

```bash
swift scripts/generate-silent-search-marker.swift
swift scripts/generate-silent-search-marker.swift --check docs/assets/silent-search-calibration-marker.pdf
```
