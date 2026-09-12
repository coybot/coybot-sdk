#!/usr/bin/env python3
"""
MAVLink Bridge Node — converts ROS 2 velocity commands to MAVLink.

Subscribes to Nav2 velocity commands (``/cmd_vel``) and converts them to
MAVLink ``SET_POSITION_TARGET_LOCAL_NED`` messages for the flight
controller. Also publishes drone state (position, velocity, odometry,
battery) to ROS 2 topics.

See https://coy.bot/docs for details.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
import tf2_ros


class MAVLinkBridgeNode(Node):
    """
    ROS 2 node that bridges Nav2 commands to MAVLink.

    Subscribes:
    - ``/cmd_vel``: Velocity commands from Nav2.

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
        self.declare_parameter("state_publish_rate", 10.0)

        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = self.get_parameter("baud_rate").value
        self.state_rate = self.get_parameter("state_publish_rate").value

        self._mavlink = None

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.pose_pub = self.create_publisher(PoseStamped, "/mavlink/local_position/pose", 10)
        self.vel_pub = self.create_publisher(TwistStamped, "/mavlink/local_position/velocity", 10)
        self.odom_pub = self.create_publisher(Odometry, "/mavlink/odom", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/mavlink/battery", 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

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
                10,
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
                    pass  # could publish attitude separately

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
        pose.pose.orientation.w = 1.0

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
        odom.pose.pose.orientation.w = 1.0

        odom.twist.twist.linear.x = msg.vx
        odom.twist.twist.linear.y = -msg.vy
        odom.twist.twist.linear.z = -msg.vz

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
