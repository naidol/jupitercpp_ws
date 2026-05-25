# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Master bringup for Jupiter robot.
#
# Usage:
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py                                                                                    # AI-only (voice/vision/brain/camera, no LiDAR/Nav, no micro-ROS)
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_microros:=true                                                              # AI-only + ESP32 (IMU, odometry, cmd_vel)
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_microros:=true enable_nav:=true enable_voice:=false                         # SLAM mapping (no voice — Whisper GPU causes USB DMA stalls that drop LiDAR frames)
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_microros:=true enable_nav:=true mode:=nav enable_voice:=false               # Nav2 navigation only (no AI nodes)
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py enable_microros:=true enable_nav:=true mode:=nav                                   # Full robot: Nav2 + AI (voice/vision/brain)

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
            description='Set true to enable LiDAR/EKF/Nav stack (SLAM mapping or Nav2 navigation, selected by mode arg).',
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

        # Nav mode — autonomous navigation with a saved map (includes localization)
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={'map': map_file}.items(),
                condition=IfCondition(PythonExpression([
                    "'", enable_nav, "' == 'true' and '", mode, "' == 'nav'"
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
                condition=IfCondition(enable_nav),
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
                condition=IfCondition(enable_nav),
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

        # Orbbec Gemini 336 — SLAM mode: color-only.
        # Depth disabled: slam_toolbox only needs /scan (LiDAR), not depth frames.
        # Color MJPG at 15fps is low USB bandwidth and does not interfere with LiDAR serial.
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
                    "'", enable_nav, "' == 'true' and '", mode, "' == 'slam'"
                ])),
            ),
        ]),

        # Orbbec Gemini 336 — Nav mode: colour + depth + IMU for cuVSLAM.
        # enable_sync_output_accel_gyro publishes a combined sensor_msgs/Imu on /camera/imu.
        # align_mode=SW aligns depth to the colour frame → /camera/depth_to_color/image_raw.
        # depth 640×480 @ 30 fps is well within USB-C super-speed bandwidth.
        # Face recognition continues to use /camera/color/image_raw — no conflict.
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
                    'color_fps':                       '30',
                    'color_format':                    'MJPG',
                    'enable_depth':                    'true',
                    'depth_width':                     '640',
                    'depth_height':                    '480',
                    'depth_fps':                       '30',
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
