# coybot-sdk

**Drones that finish the mission when the network doesn't.**

`coybot-sdk` is the open interface for the [Coybot](https://coy.bot) stack — a Python
package for ArduPilot-based drones, and a set of Swift packages for Phrover, the
phone-brained WAVE ROVER. Everything that runs on the drone or the phone is here,
including the on-device reasoning loop; only the cloud backend is closed (see
["What's not in this repo"](#whats-not-in-this-repo)).

## Python (drones)

### Install

```bash
uv add coybot-sdk
```

or

```bash
pip install coybot-sdk
```

Camera support is optional. Pick the drivers you need:

```bash
pip install coybot-sdk[camera-oak]        # Luxonis OAK-D Lite
pip install coybot-sdk[camera-realsense]  # Intel RealSense D435i
pip install coybot-sdk[all]               # both
```

### Quickstart

Copy `src/coybot_sdk/config_example.yaml` to `config.yaml` in your working
directory and edit `serial_port` to match your flight controller.

```python
import time
import coybot_sdk as drone

# Arm, take off to 2 meters AGL, hover, and land.
if drone.takeoff(2.0):
    time.sleep(5)
    drone.land()

drone.disconnect()
```

All movement commands are clamped to safe limits (see
`MIN_ALTITUDE`, `MAX_ALTITUDE`, `MAX_VELOCITY`, `MAX_YAW_RATE` in
`coybot_sdk.drone`). Failsafes can be installed onto the flight controller
once with `coybot_sdk.configure_failsafes()`.

See the [`examples/`](./examples) directory for runnable scripts:

- `motor_test.py` — verify motor order and ESC connections.
- `arm_takeoff_land.py` — minimal flight cycle.
- `camera_capture.py` — capture a frame from the onboard camera.
- `sitl/` — run the SDK against simulated ArduPilot, no hardware needed. For the same
  calls exercised against all three supported frames (Copter, Rover, Plane), see
  [`tests/test_e2e_sitl.py`](./tests/test_e2e_sitl.py) — a real, runnable test that
  doubles as a per-vehicle-type usage reference.

### Hardware compatibility

- **Flight controller**: any ArduPilot-compatible board reachable over
  MAVLink. Tested on Cube Orange.
- **Companion computer**: tested on NVIDIA Jetson Orin Nano. macOS and
  generic Linux work for development against SITL.
- **Cameras**: Luxonis OAK-D Lite, Intel RealSense D435i.

## Swift (Phrover)

Phrover is the phone-brained WAVE ROVER: a cheap 4WD chassis whose brain is an iPhone
running Apple's on-device Foundation Model, ARKit, and CoreML — talking to the chassis'
ESP32 over WiFi. Three library products, layered so you only pull in what you need:

| Product | Contents | Depends on |
|---|---|---|
| `RoverNav` | Planning/control core — `Vec2`, `Pose2D`, `Costmap`, `AStarPlanner`, `PursuitController`, `WheelCommand`. Pure Foundation. | — |
| `PhroverKit` | The brain — perception (`ARSessionManager`, `Detector`), nav orchestration (`NavigationController`, `CostmapBuilder`, `ObstacleGuard`, `WorldMapStore`), voice (`DialogAgent`, `SpeechIn`/`SpeechOut`), and ESP32 comms (`RoverControl`, `RoverTelemetry`). | `RoverNav` |
| `PhroverCloud` | Reference cloud client — Cognito auth, AWS IoT MQTT telemetry, dialog escalation. Optional: bring your own backend instead, or skip it entirely and drive on-device only. | `PhroverKit`, `aws-sdk-ios-spm` |

### Install

Add the package in Xcode (File → Add Package Dependencies) or in `Package.swift`:

```swift
.package(url: "https://github.com/coybot/coybot-sdk", from: "0.1.0")
```

then depend on the products you need:

```swift
.target(name: "YourApp", dependencies: [
    .product(name: "PhroverKit", package: "coybot-sdk"),
])
```

### Quickstart

See [`examples/PhroverOperator`](./examples/PhroverOperator) for a complete, runnable
reference app — a thin SwiftUI wrapper over `PhroverKit`/`PhroverCloud` with manual
teleop, autonomous point-to-point navigation, and voice control. It runs fully on-device
with no cloud setup; copy `Config/PhroverCloud.example.plist` to `PhroverCloud.plist` and
fill in your own backend's endpoints to add sign-in and telemetry.

```swift
import PhroverKit

let control = RoverControl()       // ESP32 at 192.168.4.1 by default (AP mode)
try await control.send(WheelCommand(left: 0.2, right: 0.2))
try await control.stop()
```

### Hardware compatibility

- **Chassis**: Waveshare WAVE ROVER (or any base speaking the same Waveshare JSON
  protocol — `GET /js?json={"T":1,"L":<m/s>,"R":<m/s>}`).
- **Phone**: iPhone/iPad with LiDAR for full autonomy (`ARSessionManager` uses scene
  depth); manual teleop and voice work without LiDAR. iOS 26+ (Apple Foundation Model
  floor).

## What's included

- **Python SDK** — MAVLink wrappers for arming, takeoff, landing,
  velocity and position control, telemetry, parameters, and failsafe setup.
- **Camera drivers** — common abstraction (`Camera`, `CameraFrame`) plus
  implementations for OAK-D Lite (DepthAI) and Intel RealSense D435i.
- **CLI utilities** — `coybot-arm-disarm` and `coybot-motor-test`.
- **ROS 2 bridge** — an optional `coybot_drone` package under
  `ros2_ws/` that exposes the camera and MAVLink as ROS 2 topics for use
  with Isaac ROS Visual SLAM and Nav2.
- **Swift SDK** — `RoverNav`/`PhroverKit`/`PhroverCloud`, the full on-device brain
  (perception, navigation, voice, on-device reasoning) and ESP32 comms for Phrover,
  plus the `PhroverOperator` reference app.

## What's not in this repo

Everything that runs on a drone or on the phone is open, including the on-device
reasoning/planning loop — that's the whole point of this repo. What's intentionally
**not** here is the cloud backend:

- **Cloud orchestration.** Fleet provisioning, IoT messaging, video ingestion, and
  remote-pilot/dialog-escalation routing (the Lambda handlers behind `/rover/converse`
  and friends) live in our service. Nothing here requires them: drones fly and Phrover
  drives/navigates/talks fully on-device without any cloud step.

If you need fleet management, video pipelines, or hosted dialog escalation, talk to us
at [coy.bot](https://coy.bot).

## Documentation

Full docs live at [coy.bot/docs](https://coy.bot/docs).

## License

Apache License 2.0. See [LICENSE](./LICENSE) and [NOTICE](./NOTICE).

## Status

Early. APIs may change before 1.0. Pin a version in production.
