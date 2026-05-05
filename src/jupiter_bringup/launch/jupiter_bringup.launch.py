# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Master bringup for Jupiter robot.
#
# Usage:
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py              # SLAM mapping mode
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py mode:=nav    # Autonomous navigation
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py mode:=nav map:=/path/to/map.yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    ldlidar_dir  = get_package_share_directory('ldlidar_stl_ros2')

    mode    = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')

    return LaunchDescription([

        DeclareLaunchArgument(
            'mode',
            default_value='slam',
            description='Operating mode: slam (build a map) or nav (navigate with saved map)',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(os.path.expanduser('~'), 'maps', 'map.yaml'),
            description='Full path to map yaml file (nav mode only)',
        ),

        # ── Hardware layer ────────────────────────────────────────────────────

        # micro-ROS agent — bridges ESP32 to ROS2 (IMU, odometry, cmd_vel)
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '--dev', '/dev/jupiter_esp32', '-b', '115200'],
            output='screen',
            name='micro_ros_agent',
        ),

        # ── Navigation layer (mutually exclusive modes) ───────────────────────
        # Start localization + SLAM first (t=3s) so the TF tree is populated
        # before the LiDAR starts sending scans.

        # SLAM mode — builds a new map (includes localization)
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'slam.launch.py')
                ),
                condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
            ),
        ]),

        # Nav mode — autonomous navigation with a saved map (includes localization)
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={'map': map_file}.items(),
                condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"])),
            ),
        ]),

        # LD19 LiDAR — delayed 6s so TF tree is ready before first scan arrives
        TimerAction(period=6.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ldlidar_dir, 'launch', 'ld19.launch.py')
                )
            ),
        ]),

        # ── Perception layer (TODO) ───────────────────────────────────────────
        # Vision node (face detection, object recognition via TensorRT)
        # Uncomment when jupiter_nodes vision is implemented:
        # Node(
        #     package='jupiter_nodes',
        #     executable='vision',
        #     name='vision',
        #     output='screen',
        # ),

        # ── Voice layer (TODO) ────────────────────────────────────────────────
        # Whisper ASR + Piper TTS + llama.cpp brain
        # Uncomment when voice pipeline is integrated into ROS2:
        # Node(
        #     package='jupiter_nodes',
        #     executable='voice',
        #     name='voice',
        #     output='screen',
        # ),

    ])
