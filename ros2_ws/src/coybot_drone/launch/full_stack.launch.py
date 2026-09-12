#!/usr/bin/env python3
"""
Full Stack Launch File - Launches all components for autonomous drone navigation.

Components:
- Camera bridge (RGB-D to ROS2)
- MAVLink bridge (Nav2 commands to flight controller)
- Isaac ROS Visual SLAM
- Nav2 navigation stack
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('coybot_drone')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_slam = LaunchConfiguration('enable_slam', default='true')
    enable_nav2 = LaunchConfiguration('enable_nav2', default='true')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_enable_slam = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable Isaac ROS Visual SLAM'
    )
    
    declare_enable_nav2 = DeclareLaunchArgument(
        'enable_nav2',
        default_value='true',
        description='Enable Nav2 navigation stack'
    )
    
    # Camera bridge node
    camera_node = Node(
        package='coybot_drone',
        executable='camera_node',
        name='camera_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fps': 30,
            'enable_depth': True,
            'rgb_width': 1280,
            'rgb_height': 720,
        }],
        output='screen'
    )
    
    # MAVLink bridge node
    mavlink_node = Node(
        package='coybot_drone',
        executable='mavlink_bridge',
        name='mavlink_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'serial_port': '/dev/ttyACM0',
            'baud_rate': 115200,
            'state_publish_rate': 10.0,
        }],
        output='screen'
    )
    
    # Isaac ROS Visual SLAM (cuVSLAM)
    # Note: This requires Isaac ROS to be installed
    vslam_node = GroupAction(
        condition=IfCondition(enable_slam),
        actions=[
            Node(
                package='isaac_ros_visual_slam',
                executable='visual_slam_node',
                name='visual_slam',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'denoise_input_images': True,
                    'rectified_images': True,
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'input_left_camera_frame': 'camera_rgb_optical_frame',
                    'input_right_camera_frame': 'camera_depth_optical_frame',
                }],
                remappings=[
                    ('visual_slam/image_0', '/camera/rgb/image_raw'),
                    ('visual_slam/camera_info_0', '/camera/rgb/camera_info'),
                    ('visual_slam/image_1', '/camera/depth/image_raw'),
                    ('visual_slam/camera_info_1', '/camera/depth/camera_info'),
                ],
                output='screen'
            )
        ]
    )
    
    # Nav2 navigation stack
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    nav2_launch = GroupAction(
        condition=IfCondition(enable_nav2),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
                    'use_composition': 'True',
                    'autostart': 'true',
                }.items()
            )
        ]
    )
    
    # Static TF publisher for camera frames
    # These should ideally come from URDF, but this works for now
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=[
            '0', '0', '0.1',  # x, y, z offset from base_link
            '0', '0', '0',     # roll, pitch, yaw
            'base_link',
            'camera_rgb_optical_frame'
        ]
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_enable_slam,
        declare_enable_nav2,
        
        # Nodes
        camera_node,
        mavlink_node,
        static_tf_camera,
        
        # Optional components
        vslam_node,
        nav2_launch,
    ])
