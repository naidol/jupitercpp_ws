# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Master bringup for Jupiter robot.
#
# Hybrid nav stack: cuVSLAM (global localisation) + LD20 LiDAR (obstacle costmap).
# cuVSLAM publishes map→odom TF; EKF owns odom→base_footprint via wheel odometry.
# LD20 /scan feeds Nav2 ObstacleLayer for local obstacle avoidance.
#
# Usage:
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py                                                                 # AI-only (voice/vision/brain/camera, no Nav, no micro-ROS)
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_microros:=true                                           # AI-only + ESP32 (IMU, odometry, cmd_vel)
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_microros:=true enable_nav:=true enable_voice:=false      # SLAM mapping (builds cuVSLAM visual map; voice off — Whisper GPU DMA stalls)
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_microros:=true enable_nav:=true mode:=nav enable_voice:=false   # Nav2 navigation only (no AI)
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_microros:=true enable_nav:=true mode:=nav               # Full robot: Nav2 + AI (voice/vision/brain)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')

    mode            = LaunchConfiguration('mode')
    enable_nav      = LaunchConfiguration('enable_nav')
    enable_microros = LaunchConfiguration('enable_microros')
    enable_voice    = LaunchConfiguration('enable_voice')

    return LaunchDescription([

        DeclareLaunchArgument(
            'enable_microros',
            default_value='false',
            description='Set true to start micro-ROS agent (requires ESP32 on /dev/jupiter_esp32).',
        ),
        DeclareLaunchArgument(
            'enable_nav',
            default_value='false',
            description='Set true to enable hybrid nav stack (cuVSLAM + LD20 LiDAR + Nav2). Mode selected by mode arg.',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='slam',
            description='Operating mode: slam (build cuVSLAM visual map) or nav (navigate with nvblox dynamic costmap)',
        ),
        DeclareLaunchArgument(
            'enable_voice',
            default_value='true',
            description='Set false to disable voice/brain nodes (recommended during SLAM mapping — Whisper GPU DMA stalls affect timing).',
        ),

        # ── Hardware layer ────────────────────────────────────────────────────

        # micro-ROS agent — started at 2s, BEFORE the camera (4s).
        # Orbbec cold-boot init takes ~55s and saturates the tegra-xusb DMA engine,
        # starving the ESP32 cp210x serial and causing micro-ROS to thrash if it tries
        # to connect during that window. Starting at 2s lets the connection fully
        # establish before the camera DMA storm begins.
        TimerAction(period=2.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                     'serial', '--dev', '/dev/jupiter_esp32', '-b', '115200'],
                output='screen',
                name='micro_ros_agent',
                condition=IfCondition(enable_microros),
            ),
        ]),

        # ── Navigation layer — only when enable_nav:=true ────────────────────

        # SLAM mode — builds a new map (includes localization + EKF)
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'slam.launch.py')
                ),
                condition=IfCondition(PythonExpression([
                    "'", enable_nav, "' == 'true' and '", mode, "' == 'slam'"
                ])),
            ),
        ]),

        # Nav mode — autonomous navigation with nvblox dynamic costmap (no pre-built map needed)
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'navigation.launch.py')
                ),
                condition=IfCondition(PythonExpression([
                    "'", enable_nav, "' == 'true' and '", mode, "' == 'nav'"
                ])),
            ),
        ]),

        # ── Perception layer ──────────────────────────────────────────────────

        # Orbbec Gemini 336 — AI-only mode: color-only MJPG 640x480 @ 15fps.
        # Depth disabled: tegra-xusb shares one DMA engine across all USB ports.
        # Depth streaming saturates it during Whisper GPU inference, causing MJPG
        # frame decode failures. Color-only keeps USB bandwidth well within budget.
        # Not started when enable_voice:=false (SLAM mapping mode) — camera feeds
        # face recognition which is also disabled; no point starting either.
        TimerAction(period=4.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('orbbec_camera'),
                        'launch', 'gemini_330_series.launch.py'
                    )
                ),
                launch_arguments={
                    'serial_number':              'CP9KB53000HP',
                    'usb_port':                   '2-1',
                    'color_width':                '640',
                    'color_height':               '480',
                    'color_fps':                  '15',
                    'color_format':               'MJPG',
                    'enable_depth':               'false',
                    'enable_ir':                  'false',
                    'enable_point_cloud':         'false',
                    'enable_colored_point_cloud': 'false',
                }.items(),
                condition=IfCondition(PythonExpression([
                    "'", enable_nav, "' == 'false' and '", enable_voice, "' == 'true'"
                ])),
            ),
        ]),

        # Orbbec Gemini 336 — SLAM mode: colour + depth + IMU @ 15fps for cuVSLAM.
        # 15fps keeps USB DMA budget well within limits with micro-ROS on the same bus.
        # cuVSLAM builds the visual keyframe map; save it after mapping with:
        #   ros2 service call /visual_slam/save_landmark_map ...
        TimerAction(period=4.0, actions=[
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
                    'color_width':                     '640',
                    'color_height':                    '480',
                    'color_fps':                       '15',
                    'color_format':                    'MJPG',
                    'enable_depth':                    'true',
                    'depth_width':                     '640',
                    'depth_height':                    '480',
                    'depth_fps':                       '6',
                    'enable_ir':                       'false',
                    'enable_point_cloud':              'false',
                    'enable_colored_point_cloud':      'false',
                    'align_mode':                      'SW',
                    'align_target_stream':             'COLOR',
                    'enable_accel':                    'true',
                    'enable_gyro':                     'true',
                    'enable_sync_output_accel_gyro':   'true',
                    'accel_rate':                      '200hz',
                    'gyro_rate':                       '200hz',
                }.items(),
                condition=IfCondition(PythonExpression([
                    "'", enable_nav, "' == 'true' and '", mode, "' == 'slam'"
                ])),
            ),
        ]),

        # Orbbec Gemini 336 — Nav mode: passive IR stereo + IMU (no depth, projector OFF).
        # cuVSLAM tracks ambient IR features (corners, doorframes, contrast edges).
        # The internal laser projector MUST be disabled — its dot pattern moves with the
        # camera and appears as "moving features" to cuVSLAM, causing tracking drift.
        # Passive IR in a lit room gives stable, room-fixed features on walls and floors.
        # IMU on /camera/gyro_accel/sample (frame: camera_accel_gyro_optical_frame).
        TimerAction(period=4.0, actions=[
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
                    'enable_color':                    'false',
                    'enable_depth':                    'false',
                    'enable_left_ir':                  'true',
                    'enable_right_ir':                 'true',
                    'left_ir_width':                   '640',
                    'left_ir_height':                  '480',
                    'left_ir_fps':                     '15',
                    'right_ir_width':                  '640',
                    'right_ir_height':                 '480',
                    'right_ir_fps':                    '15',
                    'enable_laser':                    'false',
                    'enable_point_cloud':              'false',
                    'enable_colored_point_cloud':      'false',
                    'enable_accel':                    'true',
                    'enable_gyro':                     'true',
                    'enable_sync_output_accel_gyro':   'true',
                    'accel_rate':                      '200hz',
                    'gyro_rate':                       '200hz',
                }.items(),
                condition=IfCondition(PythonExpression([
                    "'", enable_nav, "' == 'true' and '", mode, "' == 'nav'"
                ])),
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
                condition=IfCondition(enable_voice),
            ),
        ]),

        # AprilTag + depth vision node
        TimerAction(period=6.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_vision',
                name='jupiter_vision',
                output='screen',
                condition=IfCondition(enable_voice),
            ),
        ]),

        # ── Display ───────────────────────────────────────────────────────────

        TimerAction(period=2.0, actions=[
            Node(
                package='jupiter_display',
                executable='jupiter_display',
                name='jupiter_display',
                output='screen',
                additional_env={'DISPLAY': ':0', 'XAUTHORITY': '/run/user/2002/gdm/Xauthority'},
            ),
        ]),

        # ── Voice + Brain layer ───────────────────────────────────────────────

        # Whisper ASR + Piper TTS
        # Disabled during SLAM mapping (enable_voice:=false): Whisper GPU inference
        # causes tegra-xusb DMA stalls that can affect cuVSLAM image timing.
        TimerAction(period=5.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_voice',
                name='jupiter_voice',
                output='screen',
                parameters=[{
                    'energy_threshold': 300.0,   # coarse pre-filter; VAD catches steady noise above this
                    'record_seconds':   4,        # 4s window: speech dominates; 8s let startup zeros break VAD
                    'vad_snr_ratio':    1.7,      # peak/trough RMS ratio: HVAC≈1.1, speech≈2-10
                }],
                condition=IfCondition(enable_voice),
            ),
        ]),

        # LLM brain (requires Ollama already running: ollama serve)
        TimerAction(period=5.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='jupiter_brain',
                name='jupiter_brain',
                output='screen',
                condition=IfCondition(enable_voice),
            ),
        ]),

    ])
