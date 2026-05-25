# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Custom navigation stack for Jupiter.
# Includes EKF localisation (odometry fusion) + path planner.
#
# Usage:
#   # Terminal 1 — hardware + AI:
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_microros:=true
#
#   # Terminal 2 — navigation:
#   ros2 launch jupiter_nav jupiter_nav.launch.py
#
#   # Terminal 3 — send a goal (map coordinates):
#   ros2 topic pub --once /jupiter/goal geometry_msgs/msg/PoseStamped \
#     '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}'

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    default_map = os.path.join(os.path.expanduser('~'), 'maps', 'map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map',         default_value=default_map),
        DeclareLaunchArgument('initial_x',   default_value='0.0'),
        DeclareLaunchArgument('initial_y',   default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),

        # EKF (odometry fusion: wheel encoders + IMU → /odometry/filtered)
        # Pulls in imu_covariance_fixer, ekf_node, and base_footprint→imu_link TF.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'localization.launch.py')
            ),
        ),

        # Path planner — A* on saved map, publishes /jupiter/path
        Node(
            package='jupiter_nav',
            executable='jupiter_planner',
            name='jupiter_planner',
            output='screen',
            parameters=[{
                'map_yaml':         LaunchConfiguration('map'),
                'inflation_radius': 0.32,
                'initial_map_x':    LaunchConfiguration('initial_x'),
                'initial_map_y':    LaunchConfiguration('initial_y'),
                'initial_map_yaw':  LaunchConfiguration('initial_yaw'),
            }],
        ),
    ])
