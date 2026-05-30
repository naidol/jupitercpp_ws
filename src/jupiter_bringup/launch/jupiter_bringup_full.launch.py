# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Full Jupiter bringup — single command starts everything.
# Includes camera.launch.py so no separate terminal is needed.
#
# Camera (color + IR stereo + IMU) is started first and left running
# persistently. All other subsystems start after camera is warm.
#
# Startup sequence:
#   t=0s  camera.launch.py  (color + left/right IR + IMU — all streams)
#   t=0s  Display + screensaver disabled
#   t=0s  micro-ROS agent
#   t=3s  Navigation stack (EKF + LiDAR + Nav2 + cuVSLAM)
#   t=4s  Voice/Whisper (camera warm by now, no DMA conflict)
#   t=5s  Face recognition + Vision + Brain
#
# Usage:
#   ros2 launch jupiter_bringup jupiter_bringup_full.launch.py
#   ros2 launch jupiter_bringup jupiter_bringup_full.launch.py enable_microros:=false

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    enable_microros = LaunchConfiguration('enable_microros')

    return LaunchDescription([

        DeclareLaunchArgument(
            'enable_microros',
            default_value='true',
            description='Set false to skip micro-ROS agent (ESP32 not connected).',
        ),

        # ── Camera ────────────────────────────────────────────────────────────
        # Started first and never killed — keeps Orbbec firmware warm.
        # All streams: color (face recog) + IR stereo (cuVSLAM) + IMU (EKF).
        TimerAction(period=0.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'camera.launch.py')
                ),
            ),
        ]),

        # ── Display + screensaver ─────────────────────────────────────────────
        TimerAction(period=0.0, actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'DISPLAY=:0 XAUTHORITY=/run/user/2002/gdm/Xauthority '
                     'xset s off s noblank dpms 0 0 0'],
                output='screen',
                name='disable_screensaver',
            ),
        ]),

        TimerAction(period=0.0, actions=[
            Node(
                package='jupiter_display',
                executable='jupiter_display',
                name='jupiter_display',
                output='screen',
                additional_env={'DISPLAY': ':0', 'XAUTHORITY': '/run/user/2002/gdm/Xauthority'},
            ),
        ]),

        # ── micro-ROS agent ───────────────────────────────────────────────────
        # Bus 001 (ESP32) — independent of camera Bus 002.
        TimerAction(period=0.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                     'serial', '--dev', '/dev/jupiter_esp32', '-b', '115200'],
                output='screen',
                name='micro_ros_agent',
                condition=IfCondition(enable_microros),
            ),
        ]),

        # ── Navigation stack ──────────────────────────────────────────────────
        # EKF + LD20 LiDAR + static TFs + cuVSLAM + Nav2 MPPI.
        # Started at t=3s — camera firmware warm by then, no DMA conflict.
        # Lifecycle manager fires 5s later (built-in delay in navigation.launch.py).
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'navigation.launch.py')
                ),
            ),
        ]),

        # ── Voice + Whisper ───────────────────────────────────────────────────
        # t=4s — camera init done (~2s from cold), Whisper loads 1.5GB to GPU.
        TimerAction(period=4.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_voice',
                name='jupiter_voice',
                output='screen',
                parameters=[{
                    'energy_threshold': 300.0,
                    'record_seconds':   4,
                    'vad_snr_ratio':    1.7,
                }],
            ),
        ]),

        # ── Brain + Face recognition + Vision ─────────────────────────────────
        # t=5s — after camera streams are publishing, Whisper nearly loaded.
        TimerAction(period=5.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_brain',
                name='jupiter_brain',
                output='screen',
            ),
        ]),

        TimerAction(period=5.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_face_recognition',
                name='jupiter_face_recognition',
                output='screen',
                parameters=[{
                    'match_threshold': 0.55,
                }],
            ),
        ]),

        TimerAction(period=5.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_vision',
                name='jupiter_vision',
                output='screen',
            ),
        ]),

    ])
