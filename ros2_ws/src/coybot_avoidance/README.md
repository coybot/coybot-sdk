# coybot_avoidance

Map-free obstacle avoidance for multirotors: flies to a goal using only a forward depth camera, with no prior map.
ROS 2 (Humble), C++.

Each planning cycle (10 Hz) it takes a local goal 12 m ahead and searches a route to it through what the camera has
seen (OMPL). It builds a corridor of convex free space along that route (FIRI) and fits a trajectory inside it with
[GCOPTER](third_party/gcopter/README.md). The trajectory respects the speed cap and the vehicle's tilt, body-rate and
thrust limits, and ends at rest. The trajectory is then checked against the map. A plan that hits an obstacle, isn't
finite, is too short or exceeds the speed cap is rejected. When no plan passes, the planner keeps the current
trajectory while it is clear and switches to a straight stop once an obstacle is closer than the stopping distance.

The planner core (`include/coybot_avoidance/local_planner.hpp`) has no ROS dependency; the node is a thin wrapper.

## Interface

| | topic | type | notes |
|---|---|---|---|
| in | `odom` | `nav_msgs/Odometry` | pose in `world_frame` with the real attitude; twist in the body frame |
| in | `depth/image` | `sensor_msgs/Image` | `32FC1` (m) or `16UC1` (mm) |
| in | `depth/camera_info` | `sensor_msgs/CameraInfo` | |
| in | `goal` | `geometry_msgs/PoseStamped` | in `world_frame`; the goal's altitude is flown (within `slab_m`) |
| in | TF | | `base_frame` to the camera's optical frame (`camera_frame`, or the image's `frame_id`) |
| out | `trajectory` | `trajectory_msgs/MultiDOFJointTrajectory` | `header.stamp` = start time on the node's clock; points every `output_dt_s` with position, velocity, acceleration |
| out | `status` | `std_msgs/String` | one per cycle: `plan`, `keep: <why>`, `brake`, `stuck: <why>`, `arrived` |

`coybot_drone`'s `mavlink_bridge` follows `/avoidance/trajectory` with `SET_POSITION_TARGET_LOCAL_NED`
(position, velocity, acceleration and yaw). PX4 accepts that in OFFBOARD mode and ArduPilot in GUIDED. Arming,
taking off and switching mode are left to the caller. The bridge and the planner must run on the same clock
(the same host, or synchronised clocks).

```bash
ros2 launch coybot_avoidance avoidance.launch.py odom:=/mavlink/odom depth:=/camera/depth goal:=/goal_pose
ros2 run coybot_drone mavlink_bridge --ros-args -p state_publish_rate:=50.0
```

Run the bridge at 50 Hz with the planner, because the planner matches each depth image to odometry within 0.05 s.

## Parameters

`config/avoidance.yaml`, with units. The main ones:

- `max_vel`: speed cap (m/s).
- `inflate_m`: keep-out distance from any seen surface, vehicle radius included.
- `horizon_m`: how far ahead the local goal is.
- `slab_m`: how far plans may move from the goal altitude.
- `brake_acc`, `brake_latency_s`, `brake_margin_m`: the stopping-distance rule.
- `max_tilt_rad`, `max_bodyrate`, `mass_kg`, `thrust_min_n`, `thrust_max_n`: the vehicle model. The defaults
  describe a 2.0 kg x500-class quadrotor.

## Limits (read before flying)

- Nothing in the map is ever cleared, so a moving obstacle leaves a permanent keep-out zone. Use it in static scenes.
- The map is a fixed box around start and goal (15 m margin by default), sized when the first goal arrives.
  Anywhere outside it counts as blocked.
- Plans stay near the goal altitude (`slab_m`): the planner won't climb over obstacles.
- It does not back off when stuck: with no plan and nothing to keep, it reports `stuck` and waits.
- Tested in simulation only (Gazebo forests, PX4 1.17 and ArduCopter 4.7.1), not on hardware.

## Tests

```bash
colcon build --packages-select coybot_avoidance && colcon test --packages-select coybot_avoidance
```

The unit tests need no ROS graph:

- open-space arrival;
- synthetic forests at 4 and 8 m/s, which must reach the goal and stay outside the keep-out;
- a local goal must have free space around it;
- a wall appearing ahead must produce a stop outside the keep-out.

## Licence

Apache-2.0. `third_party/gcopter` is MIT (see its README). OMPL (BSD-3) is a build dependency.
