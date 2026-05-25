# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Autonomous navigation launch — requires a saved map.
#
# Localisation: Isaac ROS cuVSLAM (Orbbec RGBD + IMU) replaces AMCL + EKF.
#   cuVSLAM publishes the full map → odom → base_footprint TF chain.
#   The camera must be started with depth + IMU enabled (nav mode in bringup).
#
# Usage:
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py \
#     enable_microros:=true enable_nav:=true mode:=nav enable_voice:=false
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py \
#     enable_microros:=true enable_nav:=true mode:=nav   (full: nav + AI)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    nav2_config = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    default_map = os.path.join(os.path.expanduser('~'), 'maps', 'map.yaml')

    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file',
        ),

        # Isaac ROS cuVSLAM — replaces AMCL + EKF.
        # Publishes map→odom TF (global localisation) and odom→base_footprint TF
        # (visual odometry). Place the robot at the map origin before starting.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'visual_slam.launch.py')
            )
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_config, {'yaml_filename': map_file}],
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
                ('cmd_vel',         'cmd_vel_nav'),
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
                    'map_server',
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
