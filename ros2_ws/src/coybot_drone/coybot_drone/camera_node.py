#!/usr/bin/env python3
"""
Camera Bridge Node — publishes RGB and depth images to ROS 2 topics.

Bridges the coybot-sdk camera abstraction (OAK-D Lite / RealSense D435i)
to ROS 2 for use with Isaac ROS Visual SLAM and Nav2.

See https://coy.bot/docs for details.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge


class CameraBridgeNode(Node):
    """
    ROS 2 node that bridges the drone camera to ROS topics.

    Publishes:
    - ``/camera/rgb/image_raw``
    - ``/camera/rgb/camera_info``
    - ``/camera/depth/image_raw`` (if depth enabled)
    - ``/camera/depth/camera_info`` (if depth enabled)
    """

    def __init__(self):
        super().__init__("camera_bridge")

        self.declare_parameter("fps", 30)
        self.declare_parameter("enable_depth", True)
        self.declare_parameter("rgb_width", 1280)
        self.declare_parameter("rgb_height", 720)

        self.fps = self.get_parameter("fps").value
        self.enable_depth = self.get_parameter("enable_depth").value
        self.rgb_width = self.get_parameter("rgb_width").value
        self.rgb_height = self.get_parameter("rgb_height").value

        self.bridge = CvBridge()

        self.rgb_pub = self.create_publisher(Image, "/camera/rgb/image_raw", 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, "/camera/rgb/camera_info", 10)

        if self.enable_depth:
            self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", 10)
            self.depth_info_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", 10)

        self.camera = None
        self._init_camera()

        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)

        self.frame_count = 0
        self.get_logger().info(f"Camera bridge started at {self.fps} FPS")

    def _init_camera(self):
        """Initialize the camera using auto-detection."""
        try:
            from coybot_sdk.camera import get_camera

            self.camera = get_camera(
                rgb_fps=self.fps,
                enable_depth=self.enable_depth,
                rgb_resolution=(self.rgb_width, self.rgb_height),
            )

            if self.camera is None:
                self.get_logger().error("No camera detected")
                return

            self.camera.start()
            self.get_logger().info(f"Camera initialized: {self.camera.CAMERA_TYPE}")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize camera: {e}")
            self.camera = None

    def publish_frame(self):
        """Capture and publish a frame."""
        if self.camera is None:
            return

        try:
            frame = self.camera.get_frame(timeout_ms=100)
            if frame is None:
                return

            timestamp = self.get_clock().now().to_msg()

            if frame.rgb is not None:
                rgb_msg = self.bridge.cv2_to_imgmsg(frame.rgb, encoding="rgb8")
                rgb_msg.header = Header()
                rgb_msg.header.stamp = timestamp
                rgb_msg.header.frame_id = "camera_rgb_optical_frame"
                self.rgb_pub.publish(rgb_msg)

                rgb_info = self._create_camera_info(
                    frame.rgb.shape[1], frame.rgb.shape[0], timestamp, "camera_rgb_optical_frame"
                )
                self.rgb_info_pub.publish(rgb_info)

            if self.enable_depth and frame.depth is not None:
                depth_msg = self.bridge.cv2_to_imgmsg(frame.depth, encoding="16UC1")
                depth_msg.header = Header()
                depth_msg.header.stamp = timestamp
                depth_msg.header.frame_id = "camera_depth_optical_frame"
                self.depth_pub.publish(depth_msg)

                depth_info = self._create_camera_info(
                    frame.depth.shape[1], frame.depth.shape[0], timestamp, "camera_depth_optical_frame"
                )
                self.depth_info_pub.publish(depth_info)

            self.frame_count += 1

        except Exception as e:
            self.get_logger().warn(f"Error publishing frame: {e}")

    def _create_camera_info(self, width: int, height: int, timestamp, frame_id: str) -> CameraInfo:
        """Create CameraInfo message with approximate intrinsics.

        These should be calibrated per camera for best results; this is a
        placeholder for ~70 degree HFOV.
        """
        info = CameraInfo()
        info.header.stamp = timestamp
        info.header.frame_id = frame_id
        info.width = width
        info.height = height

        fx = width * 0.8
        fy = fx
        cx = width / 2.0
        cy = height / 2.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        info.distortion_model = "plumb_bob"

        return info

    def destroy_node(self):
        if self.camera is not None:
            self.camera.stop()
            self.get_logger().info("Camera stopped")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
