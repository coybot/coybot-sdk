# Coybot SDK Component and Sequence Diagrams

This document shows how the main components in `coybot-sdk` fit together and how the two primary runtime flows work.

## Component Diagram

```mermaid
flowchart LR
  subgraph "Python Drone SDK"
    PyApp["Python examples / user app"]
    SDK["coybot_sdk"]
    DroneAPI["drone.py MAVLink API"]
    CameraAPI["Camera abstraction"]
    OAK["OAK-D Lite driver"]
    RS["RealSense D435i driver"]
    CLI["CLI tools"]
  end

  subgraph "ROS 2 Optional Stack"
    Nav2["Nav2 / ROS nodes"]
    Bridge["coybot_drone MAVLink bridge"]
    Topics["ROS topics: cmd_vel, odom, battery, pose"]
  end

  subgraph "Drone Hardware / Simulation"
    MAVLink["pymavlink transport"]
    FC["ArduPilot flight controller"]
    SITL["ArduPilot SITL"]
    Drone["Drone frame / motors"]
    CamHW["Camera hardware"]
  end

  subgraph "Swift Phrover SDK"
    Operator["PhroverOperator iOS app"]
    Kit["PhroverKit"]
    Nav["RoverNav"]
    CloudClient["PhroverCloud optional client"]
    ARKit["ARKit / LiDAR / CoreML / Voice"]
    RoverControl["RoverControl HTTP client"]
  end

  subgraph "Rover Hardware / Optional Cloud"
    ESP32["WAVE ROVER ESP32"]
    Rover["4WD rover chassis"]
    Cloud["Coybot / user backend not in repo"]
    AWS["Cognito / AWS IoT MQTT"]
  end

  PyApp --> SDK
  CLI --> DroneAPI
  SDK --> DroneAPI
  SDK --> CameraAPI
  CameraAPI --> OAK
  CameraAPI --> RS
  OAK --> CamHW
  RS --> CamHW
  DroneAPI --> MAVLink
  MAVLink --> FC
  MAVLink --> SITL
  FC --> Drone

  Nav2 --> Topics
  Topics --> Bridge
  Bridge --> MAVLink
  Bridge --> Topics

  Operator --> Kit
  Kit --> Nav
  Kit --> ARKit
  Kit --> RoverControl
  RoverControl --> ESP32
  ESP32 --> Rover
  Operator --> CloudClient
  CloudClient --> AWS
  CloudClient --> Cloud
```

## Drone Command Sequence

```mermaid
sequenceDiagram
  actor User
  participant App as "Python script / CLI"
  participant SDK as "coybot_sdk.drone"
  participant MAV as "pymavlink"
  participant FC as "ArduPilot FC or SITL"
  participant Drone as "Drone motors / sensors"
  participant Camera as "Optional camera"

  User->>App: Run mission script
  App->>SDK: takeoff(altitude)
  SDK->>SDK: Load config.yaml / env port
  SDK->>MAV: Open serial or TCP MAVLink connection
  MAV->>FC: Wait for HEARTBEAT
  FC-->>MAV: HEARTBEAT
  SDK->>SDK: Clamp altitude / validate safety limits
  SDK->>MAV: ARM command
  MAV->>FC: COMMAND_LONG ARM
  FC-->>MAV: COMMAND_ACK
  SDK->>MAV: TAKEOFF command
  MAV->>FC: MAV_CMD_NAV_TAKEOFF
  FC->>Drone: Spin motors / climb
  FC-->>SDK: Position, attitude, battery telemetry

  App->>SDK: set_velocity(vx, vy, vz)
  SDK->>SDK: Clamp velocity
  SDK->>MAV: SET_POSITION_TARGET_LOCAL_NED
  MAV->>FC: Velocity target
  FC->>Drone: Move

  App->>SDK: capture_photo()
  SDK->>Camera: Auto-detect and get frame
  Camera-->>SDK: CameraFrame

  App->>SDK: land()
  SDK->>MAV: LAND command
  MAV->>FC: MAV_CMD_NAV_LAND
  FC->>Drone: Descend and disarm
  App->>SDK: disconnect()
```

## Phrover Rover Sequence

```mermaid
sequenceDiagram
  actor Operator
  participant App as "PhroverOperator iOS app"
  participant AR as "ARSessionManager"
  participant NavCtl as "NavigationController"
  participant Map as "CostmapBuilder"
  participant Planner as "RoverNav AStarPlanner"
  participant Guard as "ObstacleGuard"
  participant Pursuit as "PursuitController"
  participant Control as "RoverControl"
  participant ESP32 as "WAVE ROVER ESP32"
  participant Cloud as "Optional PhroverCloud"

  Operator->>App: Choose destination / voice command
  App->>AR: Start ARKit tracking
  AR-->>App: Pose, mesh anchors, forward clearance

  App->>NavCtl: navigate(to goal)
  NavCtl->>Map: Build costmap from AR mesh
  Map-->>NavCtl: Costmap
  NavCtl->>Planner: Plan path from pose to goal
  Planner-->>NavCtl: Waypoints

  loop Command interval
    NavCtl->>AR: Read current pose and clearance
    NavCtl->>Guard: Evaluate safety
    Guard-->>NavCtl: go / stop
    alt Safe
      NavCtl->>Pursuit: step(pose, path)
      Pursuit-->>NavCtl: WheelCommand
      NavCtl->>Control: send(WheelCommand)
      Control->>ESP32: GET /js?json={T,L,R}
      ESP32-->>Control: HTTP 2xx ACK
    else Unsafe
      NavCtl->>Control: stop()
      Control->>ESP32: Emergency stop command
    end
  end

  opt Cloud configured
    App->>Cloud: Auth, telemetry, dialog escalation
    Cloud-->>App: MQTT/dialog responses
  end
```
