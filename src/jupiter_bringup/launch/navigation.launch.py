# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Autonomous navigation — vision-only stack (no LiDAR).
#
# Localisation:  Isaac ROS cuVSLAM  — publishes map→odom→base_footprint TF
# Obstacle map:  Isaac ROS nvblox   — depth → GPU TSDF → 2D ESDF costmap slice
# Navigation:    Nav2 MPPI          — NvbloxCostmapLayer replaces LaserScan costmaps
#
# No pre-built map required. nvblox builds the 3D map incrementally as the robot
# moves; the Nav2 global planner plans through unknown space (allow_unknown: true)
# until the map fills in.
#
# Usage:
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py \
#     enable_microros:=true enable_nav:=true mode:=nav enable_voice:=false
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py \
#     enable_microros:=true enable_nav:=true mode:=nav   (full: nav + AI)

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    nav2_config = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        # cuVSLAM — visual odometry + localization (map→odom→base_footprint TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'visual_slam.launch.py')
            )
        ),

        # nvblox — GPU 3D reconstruction → 2D ESDF slice for Nav2 costmap layer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'nvblox.launch.py')
            )
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_config],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_config],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_config],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_config],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_config],
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_config],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_config],
            remappings=[
                ('cmd_vel',          'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
        ),
    ])
