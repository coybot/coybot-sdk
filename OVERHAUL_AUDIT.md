# Coybot Overhaul — Phase 0 Audit (coybot-sdk repo)

Scope: public API surface, mission schema, test coverage, licensing/SPDX, dependency
license table. Website/benchmark/dataset findings live in `../eco/OVERHAUL_AUDIT.md`.

## Public API surface

`src/coybot_sdk/__init__.py:8-73` re-exports a flat namespace from `drone.py`: connection
(`set_config_path`, `disconnect`), flight (`arm`, `disarm`, `takeoff`, `land`, `goto`,
`set_velocity`, `set_yaw`, `wait`), telemetry (`get_position`, `get_attitude`,
`get_battery`, `get_flight_mode`, `get_telemetry`, `is_armed`), setup (`motor_test`,
`configure_battery_monitoring`, `configure_failsafes`, `setup_drone`), and camera
(`capture_photo`, `release_camera`). Safety constants are clamped in code:
`MAX_VELOCITY=5.0 m/s`, `MAX_ALTITUDE=20.0 m`, `MIN_ALTITUDE=0.5 m`, `MAX_YAW_RATE=45.0
deg/s` (`drone.py:43-46`). It's a stateless-module wrapper over `pymavlink` with a global
connection object and a lock (`drone.py:61-62`). `camera/` auto-detects OAK-D Lite vs.
RealSense D435i. Quickstart shape: `takeoff()` (auto-arms) → `goto()`/`set_velocity()` →
`capture_photo()` → `land()` (auto-disarms).

## Mission schema — currently doesn't exist

No `MissionPlan` or `missions.create(waypoints=...)`-style API exists anywhere in `sdk/` —
grep for "mission"/"waypoint" across `sdk/**/*.py` returns zero hits. The SDK only exposes
single-shot imperative commands.

A mission concept does exist, but in the `eco` repo, not here:
`eco/drone/common/mission_runner.py:30-48` defines a `MissionRunner` that treats a mission
as an **opaque flat dict** persisted to `/var/lib/coybot/mission_active.json`, with its own
docstring admitting `"ROS2/MAVROS integration should be implemented inside start_mission()
and abort_mission()"` — it's a stub, not a working planner, and has no branching/tree
structure.

**This changes Phase 4's premise.** The master brief assumes MissionPlan v2 needs to be
"additive" and back-compat with an existing `missions.create(waypoints=...)` call — that
call doesn't exist yet in either repo. Phase 4 is a net-new build, which is actually
simpler (nothing to preserve), but it does mean someone should decide up front whether
`sdk/`'s new `MissionPlan` supersedes `eco/drone/common/mission_runner.py`'s stub or the two
are meant to stay separate (SDK-side plan object vs. on-device runner) — otherwise Phase 4
risks creating a third, competing mission concept.

## Test coverage

`sdk/tests/` has exactly two files:
- `test_imports.py` — 7 hardware-free smoke tests (import sanity, full public-symbol
  checklist mirroring `__init__.py.__all__`, camera modules importable without hardware
  SDKs). All collect and (per description) pass.
- `test_e2e_sitl.py` — a real, unmocked SITL integration test: spins up ArduPilot
  `sim_vehicle.py` for Copter/Rover/Plane, exercises arm → set_velocity → get_telemetry →
  disarm over live MAVLink TCP. It explicitly documents a gap: `takeoff()`/`land()` are
  **not** validated for Rover/Plane (lines 9-17), and the whole file skips if
  `sim_vehicle.py` isn't on `PATH`.

`pytest --collect-only` from `sdk/` in the ambient Python 3.9 collected the 7 import tests
fine but errored on `test_e2e_sitl.py` with `ModuleNotFoundError: No module named
'coybot_sdk'` — the package isn't installed in that interpreter. Not a code bug, just means
CI/local runs need `pip install -e .` first; worth confirming that's documented for
contributors if it isn't already.

There is no coverage anywhere (SDK or eco) of the benchmark scoring math — see
`eco/OVERHAUL_AUDIT.md`'s benchmark section for why (the harness code isn't in either repo).

## Licensing

- `sdk/LICENSE` (Apache 2.0, "Copyright 2026 Coybot AI, Inc.") and `sdk/NOTICE`
  ("coybot-sdk" / same copyright line) are both clean, standard, unmodified text.
- `sdk/pyproject.toml` declares `license = "Apache-2.0"` with correct SPDX ID.
- `Package.swift` has no license field (SwiftPM manifests don't support one) — licensing is
  conveyed via `sdk/LICENSE` only, which is fine.
- `ros2_ws/src/coybot_drone/package.xml:8` and `setup.py:24` both declare Apache-2.0.
- No vendored/copied GPL or AGPL source found anywhere. The MAVLink bridge
  (`ros2_ws/src/coybot_drone/coybot_drone/mavlink_bridge.py:78,83`) talks to the flight
  controller as a **separate process** over the MAVLink wire protocol via `pymavlink` — not
  linked code in the copyleft sense for that boundary. ROS 2 dependencies declared in
  `package.xml` (`isaac_ros_visual_slam`, `nav2_bringup`, etc.) are external `<depend>`
  packages, not vendored.

### One real gap: `pymavlink` is LGPLv3 and undisclosed

`pymavlink` (`pyproject.toml:30`, installed v2.4.49) is LGPLv3-licensed per its own PyPI
metadata, and it's used as a **direct Python import** inside `drone.py` (not a
separate-process boundary the way `eco/NOTICE` frames "ArduPilot MAVLink" for the eco repo).
This is very likely fine for a normal pip dependency under LGPL, but `sdk/NOTICE` doesn't
disclose it at all today, which breaks the parity `eco/NOTICE` otherwise maintains for its
own LGPL/AGPL third-party components. Recommend adding a `pymavlink (LGPLv3)` line to
`sdk/NOTICE` — cheap, and it's exactly the kind of thing an evaluator's legal team greps for.

### Dependency license table

| Source | Dependency | License | Flag |
|---|---|---|---|
| `pyproject.toml:30` | pymavlink ≥2.4 | LGPLv3 | ⚠️ undisclosed in NOTICE, see above |
| `pyproject.toml:31` | pyserial ≥3.5 | BSD | — |
| `pyproject.toml:32` | numpy ≥1.24 | BSD-3-Clause | — |
| `pyproject.toml:33` | PyYAML ≥6.0 | MIT | — |
| `pyproject.toml:34` | opencv-python ≥4.8 | Apache-2.0 | — |
| `pyproject.toml:38` | depthai ≥2.24 (optional) | MIT | — |
| `pyproject.toml:39` | pyrealsense2 ≥2.55 (optional) | Apache-2.0 | — |
| `pyproject.toml:42-44` | pytest, pytest-cov, ruff (dev) | MIT | — |
| build-system | hatchling | MIT | — |
| `Package.resolved:8` | aws-sdk-ios-spm 2.41.0 | Apache-2.0 | — |
| `ros2_ws/.../package.xml:12-26` | rclpy, std_msgs, sensor_msgs, geometry_msgs, nav_msgs, cv_bridge, image_transport, tf2_ros, isaac_ros_visual_slam, isaac_ros_nvblox, nav2_bringup, nav2_msgs | unknown — not independently verified per-package; typically Apache-2.0/BSD in the ROS 2 ecosystem | needs a `rosdep`/manual pass before Phase 1 closes |

## Ranked risk list

1. **MEDIUM** — `pymavlink` (LGPLv3) imported directly, undisclosed in `sdk/NOTICE`.
2. **LOW** — Two incompatible "mission" concepts exist across the two repos (none in `sdk/`,
   a dict-based stub in `eco/`); Phase 4 needs to pick one home rather than adding a third.
3. **LOW** — SITL suite has a documented, known gap (no Rover/Plane takeoff/land coverage) —
   not blocking, but shouldn't be described as "well-tested" without that caveat.
4. **LOW** — ROS 2 transitive dependency licenses not individually verified (item flagged
   "unknown" in the table above, not "clear").

## What this blocks in later phases

Phase 4 (MissionPlan v2) is genuinely simpler than the brief assumed — there's no existing
public API to preserve compatibility with. The one open design question before writing the
schema: does it live purely in `sdk/` and get consumed by `eco/drone/common/mission_runner.py`
as its executor, or does the runner get rewritten to match? Needs an eng-lead call before
Phase 4 starts, per the master prompt's own "Eng lead sign-off: MissionPlan v2 schema"
approval gate (tracked in `eco/OVERHAUL_AUDIT.md`).
