# Visual Target Braking Calibration Design

## Problem

The July 13 device log records a successful emergency-stop request at 0.40 m,
followed by clearance readings down to 0.21 m. The fixed 0.10 m braking lead is
therefore too small to preserve the requested 0.30 m stand-off on the current
WAVE ROVER chassis.

## Design

- Keep the visual target stand-off at 0.30 m.
- Increase the braking lead to 0.20 m so the stop request is sent at 0.50 m.
- Begin final-approach speed limiting at 0.75 m.
- Cap final-approach wheel speed at 0.10 m/s.
- Leave voice recognition, target detection, path planning, and HTTP transport
  unchanged.

## Safety Behavior

The existing navigation controller remains responsible for issuing `{T:0}` and
marking the mission arrived. The earlier trigger compensates conservatively for
the measured chassis coast and command latency without introducing reverse
braking or relying on unavailable wheel-encoder feedback.

## Verification

Navigation safety tests must prove that 0.50 m is considered arrived, a value
just above 0.50 m remains in approach mode, and forward wheel commands in the
0.50-0.75 m slowdown band are capped at 0.10 m/s.
