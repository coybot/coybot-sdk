# Silent Search Device Acceptance

This runbook is the Task 18 physical-device release gate. Task 17 provides the procedure only; no row below is device-verified yet.

## Preconditions

- Use two LiDAR-capable iPhones and two WAVE ROVERs running the same commit.
- Connect each phone only to its own rover access point. Disable Bluetooth, VPN, hotspot, cloud configuration, and other radios; use airplane mode when rover Wi-Fi remains available.
- Print `docs/assets/silent-search-calibration-marker.pdf` at actual size and verify the QR module square is 0.20 m wide.
- Place the marker and safe test objects in a bounded indoor course with known shared-frame reference points.
- Clear `Documents/phrover-runtime.log` on both phones before each row and retain only sanitized logs.
- Do not weaken navigation safety, calibration, timing, confidence, or tolerance thresholds to obtain a pass.

## Per-Run Checks

1. Record commit, phones, rovers, target label, course, start time, and operator.
2. Confirm both phones report the same marker ID and generation-local calibration.
3. Measure at least two known points. Both shared-frame coordinates must agree within the documented calibration tolerance and marker north must match the printed arrow.
4. Confirm search and return paths remain strictly in the assigned sector. A center-band crossing is permitted only after the acknowledged convergence release.
5. Confirm all coordination is optical and ordered. Neither rover moves before the acknowledged search start or convergence release.
6. Confirm each rover displays `FOUND` only after its own position and heading tolerances pass.
7. Exercise Stop where applicable and confirm motion stops before a stable terminal result appears.
8. Pull both logs and run `scripts/verify-silent-search-logs.sh ROVER_A_LOG ROVER_B_LOG` for successful convergence runs.

## Acceptance Matrix

| Scenario | Required observation | Result | Sanitized logs |
| --- | --- | --- | --- |
| No target | Both return, exchange not-found statuses and acknowledged decision, remain stopped, display `NOT FOUND` | Not run | |
| Deadline during search | Active motion stops before sector-constrained return; rendezvous begins without old-path pursuit | Not run | |
| Rover A sole finder | A reports three-sample target evidence; both reach role-specific stand-offs after acknowledgement | Not run | |
| Rover B sole finder | B reports three-sample target evidence; both reach role-specific stand-offs after acknowledgement | Not run | |
| Matching dual reports | Reports within 0.50 m select the deterministic median and both converge | Not run | |
| Conflicting dual reports | Reports beyond 0.50 m terminate as conflict with no convergence commit or movement | Not run | |
| Withhold search acknowledgement | Both remain stopped; timeout exposes Retry/Abort; retry preserves bytes and sequence | Not run | |
| Withhold convergence acknowledgement | Both remain stopped at rendezvous; no sector release or convergence movement | Not run | |
| Missing partner | Waiting rover remains stopped and fails 60 seconds after the search deadline | Not run | |
| Tracking recovers within 5 seconds | Motion stops immediately, then replans from current pose under the same policy | Not run | |
| Tracking does not recover | Shared frame invalidates and mission terminates with motion stopped | Not run | |
| AR session reset | Shared frame invalidates immediately; old generation is never reused | Not run | |
| Unsafe rendezvous pose/path | Mission fails without searching for an alternate staging pose | Not run | |
| Unsafe convergence pose/path | Mission fails without searching for an alternate stand-off pose | Not run | |
| Stop while searching | Stop completes before terminal operator-stopped state | Not run | |
| Stop while returning | Stop completes before terminal operator-stopped state | Not run | |
| Stop while converging | Stop completes before terminal operator-stopped state | Not run | |

## Evidence Record

For every row, record pass/fail, observations, and sanitized log paths. A simulator/build pass is not device verification. Any hardware failure leaves the feature **not device verified** until the affected row is rerun successfully.
