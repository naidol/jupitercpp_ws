# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Master bringup for Jupiter robot.
#
# Usage:
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py                               # AI-only (voice/vision/brain/camera, no SLAM/LiDAR)
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_slam:=true             # SLAM mapping mode
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_slam:=true mode:=nav  # Autonomous navigation
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_slam:=true mode:=nav map:=/path/to/map.yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    ldlidar_dir  = get_package_share_directory('ldlidar_stl_ros2')

    mode        = LaunchConfiguration('mode')
    map_file    = LaunchConfiguration('map')
    enable_slam = LaunchConfiguration('enable_slam')

    return LaunchDescription([

        DeclareLaunchArgument(
            'enable_slam',
            default_value='false',
            description='Set true to enable SLAM/Nav2/EKF/LiDAR stack. Default false for AI-only operation.',
        ),
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

        # ── Navigation layer — only when enable_slam:=true ────────────────────

        # SLAM mode — builds a new map (includes localization + EKF)
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'slam.launch.py')
                ),
                condition=IfCondition(PythonExpression([
                    "'", enable_slam, "' == 'true' and '", mode, "' == 'slam'"
                ])),
            ),
        ]),

        # Nav mode — autonomous navigation with a saved map (includes localization)
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={'map': map_file}.items(),
                condition=IfCondition(PythonExpression([
                    "'", enable_slam, "' == 'true' and '", mode, "' == 'nav'"
                ])),
            ),
        ]),

        # LD19 LiDAR — only needed for SLAM/Nav
        TimerAction(period=6.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ldlidar_dir, 'launch', 'ld19.launch.py')
                ),
                condition=IfCondition(enable_slam),
            ),
        ]),

        # ── Perception layer ──────────────────────────────────────────────────

        # Orbbec Gemini 336 — AI-only mode: color-only MJPG 640x480 @ 15fps.
        # Depth disabled: tegra-xusb shares one DMA engine across all USB ports.
        # Depth streaming saturates it during Whisper GPU inference, causing MJPG
        # frame decode failures. Color-only keeps USB bandwidth well within budget.
        TimerAction(period=4.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('orbbec_camera'),
                        'launch', 'gemini_330_series.launch.py'
                    )
                ),
                launch_arguments={
                    'color_width':                '640',
                    'color_height':               '480',
                    'color_fps':                  '15',
                    'color_format':               'MJPG',
                    'enable_depth':               'false',
                    'enable_ir':                  'false',
                    'enable_point_cloud':         'false',
                    'enable_colored_point_cloud': 'false',
                }.items(),
                condition=IfCondition(PythonExpression(["'", enable_slam, "' == 'false'"])),
            ),
        ]),

        # Orbbec Gemini 336 — SLAM/Nav mode: color + depth enabled.
        # Depth is required for Nav2 costmaps and AprilTag docking distance.
        # LiDAR is also running in this mode, so Whisper is the primary USB
        # contender — acceptable since navigation reduces voice interaction.
        TimerAction(period=4.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('orbbec_camera'),
                        'launch', 'gemini_330_series.launch.py'
                    )
                ),
                launch_arguments={
                    'color_width':                '640',
                    'color_height':               '480',
                    'color_fps':                  '15',
                    'color_format':               'MJPG',
                    'enable_depth':               'true',
                    'enable_ir':                  'false',
                    'enable_point_cloud':         'false',
                    'enable_colored_point_cloud': 'false',
                }.items(),
                condition=IfCondition(enable_slam),
            ),
        ]),

        # Face recognition — delayed 6s so camera is publishing before we subscribe
        TimerAction(period=6.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_face_recognition',
                name='jupiter_face_recognition',
                output='screen',
                parameters=[{
                    'match_threshold': 0.55,  # SFace paper threshold — 0.40 caused false matches
                }],
            ),
        ]),

        # AprilTag + depth vision node
        TimerAction(period=6.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_vision',
                name='jupiter_vision',
                output='screen',
            ),
        ]),

        # ── Voice + Brain layer ───────────────────────────────────────────────

        # Whisper ASR + Piper TTS
        TimerAction(period=5.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_voice',
                name='jupiter_voice',
                output='screen',
                parameters=[{
                    'energy_threshold': 500.0,  # raise from 300 — filters ambient noise, speech RMS typically 2000+
                    'record_seconds':   8,       # 8s window → Whisper fires at most every 8s instead of 5s
                }],
            ),
        ]),

        # LLM brain (requires Ollama already running: ollama serve)
        TimerAction(period=5.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_brain',
                name='jupiter_brain',
                output='screen',
            ),
        ]),

    ])
