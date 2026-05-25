# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Master bringup for Jupiter robot.
#
# Usage:
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py                                                                                    # AI-only (voice/vision/brain/camera, no SLAM/LiDAR, no micro-ROS)
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_microros:=true                                                              # AI-only + ESP32 (IMU, odometry, cmd_vel)
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_microros:=true enable_slam:=true enable_voice:=false                        # SLAM mapping (no voice — Whisper GPU causes USB DMA stalls that drop LiDAR frames)
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_microros:=true enable_slam:=true mode:=nav                                  # Autonomous navigation with AI

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

    mode            = LaunchConfiguration('mode')
    map_file        = LaunchConfiguration('map')
    enable_slam     = LaunchConfiguration('enable_slam')
    enable_microros = LaunchConfiguration('enable_microros')
    enable_voice    = LaunchConfiguration('enable_voice')

    return LaunchDescription([

        DeclareLaunchArgument(
            'enable_microros',
            default_value='false',
            description='Set true to start micro-ROS agent (requires ESP32 on /dev/jupiter_esp32).',
        ),
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
        DeclareLaunchArgument(
            'enable_voice',
            default_value='true',
            description='Set false to disable voice/brain nodes (required for SLAM mapping — Whisper GPU causes tegra-xusb DMA stalls that drop LiDAR frames).',
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

        # LD19 LiDAR — only needed for SLAM/Nav.
        # Delayed 28s: camera starts at 4s but takes up to 25s device-selection in
        # SLAM mode (depth disabled = ~38s total). Starting LiDAR before camera
        # settles causes serial FIFO starvation — SDK loses frame sync after first scan.
        #
        # Static TF (base_footprint → base_laser) — runs permanently, no restart needed.
        TimerAction(period=28.0, actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_base_laser_ld19',
                arguments=['0.06', '0', '0.22', '0', '0', '0', 'base_footprint', 'base_laser'],
                condition=IfCondition(enable_slam),
            ),
        ]),
        # LiDAR watchdog — starts ldlidar_stl_ros2_node and auto-restarts it when
        # /scan times out. Whisper GPU inference causes tegra-xusb DMA stalls that
        # drop cp210x serial bytes, breaking LD19 SDK frame sync (no self-recovery).
        # Delayed 30s: 2s after TF publisher to let it settle before first scan arrives.
        TimerAction(period=30.0, actions=[
            ExecuteProcess(
                cmd=[os.path.join(bringup_dir, 'scripts', 'lidar_watchdog.sh')],
                output='screen',
                name='lidar_watchdog',
                condition=IfCondition(enable_slam),
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
                    "'", enable_slam, "' == 'false' and '", enable_voice, "' == 'true'"
                ])),
            ),
        ]),

        # Orbbec Gemini 336 — SLAM mode with AI: color-only, same as AI-only mode.
        # Not started when enable_voice:=false (mapping mode) — face recognition is
        # disabled so there's no consumer; eliminating USB DMA frees up bandwidth
        # for the LiDAR serial path during scan processing.
        # Depth disabled: slam_toolbox only needs /scan (LiDAR), not depth frames.
        # Re-enable depth here only when Nav2 navigation with costmaps is needed.
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
                    "'", enable_slam, "' == 'true' and '", enable_voice, "' == 'true'"
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
        # causes tegra-xusb DMA stalls every 4s that starve the LiDAR serial FIFO
        # and block the slam_toolbox executor, preventing scans from being processed.
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
