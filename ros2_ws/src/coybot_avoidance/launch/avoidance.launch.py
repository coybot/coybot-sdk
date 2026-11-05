"""Launch the map-free avoidance planner with the package defaults (config/avoidance.yaml).

Remap its inputs to your camera and odometry, for example:
  ros2 launch coybot_avoidance avoidance.launch.py odom:=/mavlink/odom depth:=/camera/depth
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("coybot_avoidance")
    args = [
        DeclareLaunchArgument("params", default_value=os.path.join(share, "config", "avoidance.yaml")),
        DeclareLaunchArgument("odom", default_value="/mavlink/odom"),
        DeclareLaunchArgument("depth", default_value="/camera/depth", description="prefix of image and camera_info"),
        DeclareLaunchArgument("goal", default_value="/goal_pose"),
    ]
    node = Node(
        package="coybot_avoidance",
        executable="avoidance_planner",
        name="avoidance_planner",
        parameters=[LaunchConfiguration("params")],
        remappings=[
            ("odom", LaunchConfiguration("odom")),
            ("depth/image", [LaunchConfiguration("depth"), "/image"]),
            ("depth/camera_info", [LaunchConfiguration("depth"), "/camera_info"]),
            ("goal", LaunchConfiguration("goal")),
            ("trajectory", "/avoidance/trajectory"),
            ("status", "/avoidance/status"),
        ],
        output="screen",
    )
    return LaunchDescription(args + [node])
