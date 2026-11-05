#!/usr/bin/env python3
"""
MAVLink Bridge Node — converts ROS 2 velocity commands to MAVLink.

Subscribes to Nav2 velocity commands (``/cmd_vel``) and converts them to
MAVLink ``SET_POSITION_TARGET_LOCAL_NED`` messages for the flight
controller. Also follows planned trajectories (``/avoidance/trajectory``, from
coybot_avoidance) with position, velocity, acceleration and yaw targets, which
PX4 (OFFBOARD) and ArduPilot (GUIDED) both accept; putting the vehicle in that
mode is left to the caller. Publishes drone state (position, velocity,
odometry with attitude, battery) to ROS 2 topics.

Frames: ``odom`` is x north, y west, z up (NED with y and z negated);
``base_link`` is x forward, y left, z up. Odometry twist is in ``base_link``.

See https://coy.bot/docs for details.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from trajectory_msgs.msg import MultiDOFJointTrajectory
import tf2_ros

from coybot_drone.trajectory_follow import (
    MAV_FRAME_LOCAL_NED, TYPE_MASK_PVA_YAW, Follower, to_ned)


class MAVLinkBridgeNode(Node):
    """
    ROS 2 node that bridges Nav2 commands to MAVLink.

    Subscribes:
    - ``/cmd_vel``: Velocity commands from Nav2 (a command clears any trajectory being followed).
    - ``trajectory_topic`` (``/avoidance/trajectory``): trajectories to follow.

    Publishes:
    - ``/mavlink/local_position/pose``
    - ``/mavlink/local_position/velocity``
    - ``/mavlink/odom``
    - ``/mavlink/battery``
    """

    # Safety limits (intentionally conservative for indoor navigation)
    MAX_LINEAR_VEL = 2.0   # m/s
    MAX_ANGULAR_VEL = 1.0  # rad/s
    MAX_VERTICAL_VEL = 1.0  # m/s

    def __init__(self):
        super().__init__("mavlink_bridge")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("state_publish_rate", 10.0)   # Hz; also the rate requested from the autopilot
        self.declare_parameter("trajectory_topic", "/avoidance/trajectory")
        self.declare_parameter("setpoint_rate", 20.0)        # Hz, while following a trajectory

        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = self.get_parameter("baud_rate").value
        self.state_rate = self.get_parameter("state_publish_rate").value

        self._mavlink = None
        self._att = (0.0, 0.0, 0.0)   # roll, pitch, yaw from ATTITUDE (NED / FRD), rad
        self.follower = Follower()

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.pose_pub = self.create_publisher(PoseStamped, "/mavlink/local_position/pose", 10)
        self.vel_pub = self.create_publisher(TwistStamped, "/mavlink/local_position/velocity", 10)
        self.odom_pub = self.create_publisher(Odometry, "/mavlink/odom", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/mavlink/battery", 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.traj_sub = self.create_subscription(
            MultiDOFJointTrajectory, self.get_parameter("trajectory_topic").value, self.trajectory_callback, 10)
        self.follow_timer = self.create_timer(1.0 / self.get_parameter("setpoint_rate").value, self.follow)

        state_period = 1.0 / self.state_rate
        self.state_timer = self.create_timer(state_period, self.publish_state)

        # Velocity command timeout (stop if no commands for this long)
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = rclpy.duration.Duration(seconds=0.5)
        self.timeout_timer = self.create_timer(0.1, self.check_cmd_timeout)

        self.get_logger().info("MAVLink bridge started")

    def _get_mavlink(self):
        """Lazy-load MAVLink connection."""
        if self._mavlink is None:
            try:
                from pymavlink import mavutil

                self.get_logger().info(
                    f"Connecting to {self.serial_port} at {self.baud_rate}..."
                )
                self._mavlink = mavutil.mavlink_connection(
                    self.serial_port,
                    baud=self.baud_rate,
                    source_system=255,
                )

                msg = self._mavlink.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
                if msg:
                    self._mavlink.target_system = msg.get_srcSystem()
                    self._mavlink.target_component = msg.get_srcComponent()
                    self.get_logger().info(
                        f"Connected to system {self._mavlink.target_system}"
                    )
                else:
                    self.get_logger().warn("No heartbeat received")

            except Exception as e:
                self.get_logger().error(f"MAVLink connection failed: {e}")
                self._mavlink = None

        return self._mavlink

    def cmd_vel_callback(self, msg: Twist):
        """
        Handle velocity commands from Nav2.

        ROS: x=forward, y=left,  z=up,   yaw=CCW positive
        NED: x=forward, y=right, z=down, yaw=CW positive
        """
        mav = self._get_mavlink()
        if mav is None:
            return

        self.last_cmd_time = self.get_clock().now()
        if self.follower.has_trajectory():
            self.get_logger().info("velocity command: trajectory dropped")
            self.follower = Follower()

        vx = self._clamp(msg.linear.x, -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL)
        vy = self._clamp(-msg.linear.y, -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL)
        vz = self._clamp(-msg.linear.z, -self.MAX_VERTICAL_VEL, self.MAX_VERTICAL_VEL)
        yaw_rate = self._clamp(-msg.angular.z, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)

        try:
            mav.mav.set_position_target_local_ned_send(
                0,
                mav.target_system,
                mav.target_component,
                9,  # MAV_FRAME_BODY_NED
                0b0000011111000111,  # type_mask: velocity only + yaw rate
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                0, yaw_rate,
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to send velocity command: {e}")

    def check_cmd_timeout(self):
        """Stop the drone if no velocity commands received recently."""
        mav = self._get_mavlink()
        if mav is None:
            return

        if self.follower.active(self._now_s()):
            return
        time_since_cmd = self.get_clock().now() - self.last_cmd_time
        if time_since_cmd > self.cmd_timeout:
            try:
                mav.mav.set_position_target_local_ned_send(
                    0,
                    mav.target_system,
                    mav.target_component,
                    9,
                    0b0000011111000111,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0,
                )
            except Exception:
                pass

    def publish_state(self):
        """Publish current drone state to ROS 2 topics."""
        mav = self._get_mavlink()
        if mav is None:
            return

        timestamp = self.get_clock().now().to_msg()

        try:
            mav.mav.request_data_stream_send(
                mav.target_system,
                mav.target_component,
                0,  # MAV_DATA_STREAM_ALL
                int(self.state_rate),
                1,
            )

            while True:
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break

                msg_type = msg.get_type()

                if msg_type == "LOCAL_POSITION_NED":
                    self._publish_pose(msg, timestamp)
                    self._publish_velocity(msg, timestamp)
                    self._publish_odom(msg, timestamp)

                elif msg_type == "ATTITUDE":
                    self._att = (msg.roll, msg.pitch, msg.yaw)

                elif msg_type == "SYS_STATUS":
                    self._publish_battery(msg, timestamp)

        except Exception as e:
            self.get_logger().warn(f"Error reading MAVLink: {e}")

    def _publish_pose(self, msg, timestamp):
        pose = PoseStamped()
        pose.header.stamp = timestamp
        pose.header.frame_id = "odom"

        # Convert NED to ROS frame (ENU)
        pose.pose.position.x = msg.x
        pose.pose.position.y = -msg.y
        pose.pose.position.z = -msg.z
        pose.pose.orientation = self._orientation()

        self.pose_pub.publish(pose)
        self._broadcast_tf(pose, timestamp)

    def _publish_velocity(self, msg, timestamp):
        vel = TwistStamped()
        vel.header.stamp = timestamp
        vel.header.frame_id = "base_link"

        vel.twist.linear.x = msg.vx
        vel.twist.linear.y = -msg.vy
        vel.twist.linear.z = -msg.vz

        self.vel_pub.publish(vel)

    def _publish_odom(self, msg, timestamp):
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = -msg.y
        odom.pose.pose.position.z = -msg.z
        odom.pose.pose.orientation = self._orientation()

        # twist in base_link (REP 105): rotate the odom-frame velocity into the body
        bx, by, bz = self._to_body(msg.vx, -msg.vy, -msg.vz)
        odom.twist.twist.linear.x = bx
        odom.twist.twist.linear.y = by
        odom.twist.twist.linear.z = bz

        self.odom_pub.publish(odom)

    def _publish_battery(self, msg, timestamp):
        battery = BatteryState()
        battery.header.stamp = timestamp
        battery.voltage = msg.voltage_battery / 1000.0
        battery.percentage = msg.battery_remaining / 100.0

        if msg.current_battery != -1:
            battery.current = msg.current_battery / 100.0

        self.battery_pub.publish(battery)

    def _broadcast_tf(self, pose: PoseStamped, timestamp):
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation = pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _rpy_odom(self):
        """Attitude in odom / base_link (x north, y west, z up; FLU): NED/FRD with y and z negated."""
        r, p, y = self._att
        return r, -p, -y

    def _orientation(self):
        from geometry_msgs.msg import Quaternion
        r, p, y = self._rpy_odom()
        cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                                  math.cos(y / 2), math.sin(y / 2))
        return Quaternion(w=cr * cp * cy + sr * sp * sy, x=sr * cp * cy - cr * sp * sy,
                          y=cr * sp * cy + sr * cp * sy, z=cr * cp * sy - sr * sp * cy)

    def _to_body(self, x, y, z):
        """odom-frame vector into base_link (transpose of the ZYX rotation)."""
        r, p, yw = self._rpy_odom()
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yw), math.sin(yw)
        R = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
             (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
             (-sp, cp * sr, cp * cr))
        return tuple(R[0][i] * x + R[1][i] * y + R[2][i] * z for i in range(3))

    def trajectory_callback(self, msg: MultiDOFJointTrajectory):
        """A new plan replaces the one being followed (it starts on the current one, so the change is smooth)."""
        try:
            pts = msg.points
            t = [q.time_from_start.sec + 1e-9 * q.time_from_start.nanosec for q in pts]
            p = [(q.transforms[0].translation.x, q.transforms[0].translation.y, q.transforms[0].translation.z)
                 for q in pts]
            v = [(q.velocities[0].linear.x, q.velocities[0].linear.y, q.velocities[0].linear.z) for q in pts]
            a = [(q.accelerations[0].linear.x, q.accelerations[0].linear.y, q.accelerations[0].linear.z)
                 for q in pts]
            self.follower.set_trajectory(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec, t, p, v, a)
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"trajectory rejected: {e}")

    def follow(self):
        """Send the trajectory's current target (holding the end point for a while after it)."""
        now = self._now_s()
        if not self.follower.active(now):
            return
        mav = self._get_mavlink()
        if mav is None:
            return
        sp = self.follower.setpoint(now)
        (x, y, z), (vx, vy, vz), (ax, ay, az), yaw = to_ned(sp)
        try:
            mav.mav.set_position_target_local_ned_send(
                0, mav.target_system, mav.target_component, MAV_FRAME_LOCAL_NED, TYPE_MASK_PVA_YAW,
                x, y, z, vx, vy, vz, ax, ay, az, yaw, 0)
        except Exception as e:
            self.get_logger().warn(f"Failed to send trajectory target: {e}")

    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))

    def destroy_node(self):
        if self._mavlink is not None:
            try:
                self._mavlink.mav.set_position_target_local_ned_send(
                    0, self._mavlink.target_system, self._mavlink.target_component,
                    9, 0b0000011111000111,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                )
                self._mavlink.close()
            except Exception:
                pass
            self.get_logger().info("MAVLink connection closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MAVLinkBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
