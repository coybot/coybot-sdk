# Optical QR Scanner Resilience Design

## Summary

Make QR scanning resilient when one Apple Vision barcode request fails on a physical device. Vision orientations and Core Image become independent best-effort backends. A successful decode from any backend succeeds, while sanitized backend failures remain observable.

This addresses a device calibration attempt that entered `.calibrating` with normal AR tracking and a healthy rover command link, then emitted `scanner_failure` before QR detection or LiDAR grounding.

## Goals

- Prevent one Vision orientation error from aborting the remaining orientations or Core Image fallback.
- Allow calibration to proceed when any backend decodes the expected marker.
- Preserve diagnostic visibility when fallback succeeds.
- Distinguish normal no-marker results from backend failures.
- Remove deprecated forced CPU execution from Vision requests.

## Non-goals

- Replacing Vision or Core Image with another QR library.
- Changing calibration geometry, LiDAR grounding, marker payloads, or acceptance thresholds.
- Logging images, payload bytes, localized error descriptions, or stack traces.
- Changing optical message validation.

## Architecture

`OpticalQRCodeScanner` remains the single QR-scanning boundary. It gains a detailed scan outcome containing decoded observations and zero or more sanitized backend diagnostics.

The existing `scan(_:) -> [OpticalObservation]` interface remains available and delegates to the detailed scan path. Calibration uses the detailed outcome so it can publish diagnostics even when observations are returned successfully.

A diagnostic contains only:

- backend: Vision or Core Image;
- attempted image orientation when applicable;
- stable error domain;
- numeric error code.

## Scan flow

1. Validate the frame timestamp and suppress duplicate frame identity as today.
2. Try Vision with `.right`, `.up`, `.left`, and `.down` independently.
3. For each Vision attempt:
   - return immediately when decoded observations are found, including diagnostics accumulated from earlier failed attempts;
   - continue after an empty successful request;
   - catch an error, append a sanitized diagnostic, and continue.
4. If Vision produces no observation, run Core Image fallback.
5. Return Core Image observations plus accumulated diagnostics when it decodes a QR.
6. If Core Image completes with no QR, return an empty observation list plus diagnostics. This is a normal waiting state, not a calibration failure.
7. Only an invalid frame timestamp throws from the detailed API. Backend setup and execution failures return sanitized diagnostics and never throw.

`VNDetectBarcodesRequest.usesCPUOnly` is removed so Vision chooses a supported execution path.

## Calibration behavior

`ARSharedMissionFrameCalibrator` consumes the detailed outcome:

- any observation follows the existing marker preflight and LiDAR grounding flow;
- an empty outcome produces or retains the normal waiting-for-marker state;
- backend diagnostics are emitted independently and do not overwrite successful QR, grounding, or accepted-sample state;
- fallback success is never reported as `scanner_failure`.

Consecutive identical backend diagnostics are deduplicated by backend, orientation, domain, and code. A scan cycle without that diagnostic resets deduplication so a later recurrence emits a new transition.

## Telemetry

Add `silent_search_scanner_backend_failed` with these allowed fields:

- `backend`;
- `orientation` when applicable;
- `error_domain`;
- `error_code`;
- existing mission, role, marker, generation, and frame-sequence context where available.

Do not reuse `silent_search_grounding_failed` for backend diagnostics. Do not log payloads, images, localized descriptions, or stack traces.

## Testing

Use injected scanner backends at an internal test seam and verify:

- the first Vision orientation throws and a later Vision orientation decodes;
- all Vision orientations throw and Core Image decodes;
- Vision errors plus an empty Core Image result return normal no-marker output with diagnostics;
- fallback success carries diagnostics without becoming a scanner failure;
- duplicate frames remain suppressed;
- orientation-specific corner canonicalization remains correct;
- repeated diagnostics are deduplicated in calibration telemetry;
- diagnostic fields contain no payload, image, or localized error content;
- existing optical scanner and calibration tests remain green.

## Acceptance criteria

- A Vision error cannot prevent remaining Vision orientations or Core Image from running.
- A QR decoded by any backend advances through the existing calibration pipeline.
- No-marker output remains a waiting state even when one backend failed.
- Device logs expose stable backend domain/code information without sensitive content.
- The observed iPhone scanner exception no longer produces a terminal scanner failure when Core Image remains operational.
