# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Full Jupiter bringup — Navigation + Voice + Vision + Brain in one launch.
#
# Single Orbbec camera instance serving all subsystems simultaneously:
#   - Left + Right IR stereo @ 15fps  → cuVSLAM localisation
#   - Color MJPG 640x480 @ 15fps      → Face recognition
#   - IMU accel + gyro @ 200Hz        → EKF odometry
#   - Laser projector OFF             → cuVSLAM passive IR tracking
#   - Depth disabled                  → saves USB DMA bandwidth
#
# Startup sequence (timed to avoid USB DMA contention):
#   t=0s  Display
#   t=2s  micro-ROS agent (ESP32 connects before camera DMA storm)
#   t=3s  Navigation stack (EKF + LiDAR + Nav2 + cuVSLAM)
#   t=5s  Camera (single instance, all streams)
#   t=7s  Voice + Brain (after camera is publishing)
#   t=8s  Face recognition + Vision (after camera is publishing)
#   t=8s  Nav2 lifecycle manager (5s delay inside navigation.launch.py)
#
# Usage:
#   ros2 launch jupiter_bringup jupiter_bringup_full.launch.py
#   ros2 launch jupiter_bringup jupiter_bringup_full.launch.py enable_microros:=false

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

        # ── Display ───────────────────────────────────────────────────────────
        # Started first so the HUD is visible during the rest of bringup.
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
        # Started at 2s — before the camera (5s). The Orbbec cold-boot DMA storm
        # would starve the ESP32 serial if they start together.
        TimerAction(period=2.0, actions=[
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
        # Nav2 lifecycle manager fires 5s after navigation.launch.py starts
        # (built-in TimerAction in that file) giving MPPI CUDA time to initialise.
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'navigation.launch.py')
                ),
            ),
        ]),

        # ── Orbbec Gemini 336 — full stream mode ──────────────────────────────
        # Single camera instance serving cuVSLAM (IR stereo) + face recognition
        # (color) + EKF (IMU) simultaneously.
        # Laser projector MUST stay OFF — its dot pattern moves with the camera
        # and causes cuVSLAM to track "moving features", producing massive drift.
        # Depth disabled — not needed by any current subsystem, saves DMA bandwidth.
        # Color at MJPG keeps USB bandwidth well below the 3.2 Gbps Bus 002 limit
        # even with dual IR + color + IMU all streaming together.
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('orbbec_camera'),
                        'launch', 'gemini_330_series.launch.py'
                    )
                ),
                launch_arguments={
                    'serial_number':                   'CP9KB53000HP',
                    'usb_port':                        '2-1',
                    # Color — face recognition
                    'enable_color':                    'true',
                    'color_width':                     '640',
                    'color_height':                    '480',
                    'color_fps':                       '15',
                    'color_format':                    'MJPG',
                    # IR stereo — cuVSLAM passive tracking
                    'enable_left_ir':                  'true',
                    'enable_right_ir':                 'true',
                    'left_ir_width':                   '640',
                    'left_ir_height':                  '480',
                    'left_ir_fps':                     '15',
                    'right_ir_width':                  '640',
                    'right_ir_height':                 '480',
                    'right_ir_fps':                    '15',
                    # Projector OFF — mandatory for cuVSLAM passive IR
                    'enable_laser':                    'false',
                    # Depth off — not needed, saves DMA bandwidth
                    'enable_depth':                    'false',
                    'enable_point_cloud':              'false',
                    'enable_colored_point_cloud':      'false',
                    # IMU — EKF yaw rate fusion
                    'enable_accel':                    'true',
                    'enable_gyro':                     'true',
                    'enable_sync_output_accel_gyro':   'true',
                    'accel_rate':                      '200hz',
                    'gyro_rate':                       '200hz',
                }.items(),
            ),
        ]),

        # ── Voice + Brain ─────────────────────────────────────────────────────
        # Started at 7s — camera needs ~2s to initialise after t=5s launch.
        # Whisper loads 1.5GB model to GPU; brain connects to Ollama (must be
        # running: ollama serve).
        TimerAction(period=7.0, actions=[
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

        TimerAction(period=7.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_brain',
                name='jupiter_brain',
                output='screen',
            ),
        ]),

        # ── Vision + Face recognition ─────────────────────────────────────────
        # Started at 8s — after camera is confirmed publishing color stream.
        TimerAction(period=8.0, actions=[
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

        TimerAction(period=8.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_vision',
                name='jupiter_vision',
                output='screen',
            ),
        ]),

    ])
