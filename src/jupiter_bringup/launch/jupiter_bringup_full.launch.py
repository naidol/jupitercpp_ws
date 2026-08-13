# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Full Jupiter bringup — single command starts everything.
#
# Startup sequence (enable_nav:=true, the default):
#   t=0s  Display + screensaver disabled
#   t=3s  navigation_s2e.launch.py — S2E lidar + AMCL + EKF + Nav2, AND the micro-ROS
#         agent, the Orbbec camera and every static TF (incl. tof_front_link)
#   t=4s  Voice/Whisper (camera warm by now, no DMA conflict)
#   t=5s  Face recognition + Vision + Brain
#
# With enable_nav:=false this file supplies the camera and the micro-ROS agent itself at
# t=0 instead. Exactly one of the two paths provides them — never both. Starting two
# Orbbec nodes on one serial number, or two agents on /dev/jupiter_esp32, breaks both.
#
# REPOINTED 2026-08-13 from navigation.launch.py to navigation_s2e.launch.py. The former
# is the retired stack (cuVSLAM, old c82_map_real map, base_laser TF 390 mm low and yaw
# 180 deg out for the LD20 removed on 2026-08-03) and enable_nav defaulted to false to
# avoid it. Nav is now ON by default.
#
# THIS IS THE ONE CANONICAL FULL-ROBOT LAUNCH. jupiter_full_s2e.launch.py was retired into
# it on 2026-08-13 (its dock_approach node came across; everything else here was newer).
# Do not fork a second full-stack launch — two of them drifting apart is exactly what left
# this file pointing at a retired nav stack for ten days without anyone noticing.
#
# Usage:
#   ros2 launch jupiter_bringup jupiter_bringup_full.launch.py
#   ros2 launch jupiter_bringup jupiter_bringup_full.launch.py enable_nav:=false
#   ros2 launch jupiter_bringup jupiter_bringup_full.launch.py enable_microros:=false

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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

        DeclareLaunchArgument(
            'enable_nav',
            default_value='true',
            description='Start the navigation stack (navigation_s2e.launch.py: S2E lidar + '
                        'AMCL + EKF + Nav2). ON by default since 2026-08-13, when this file '
                        'was repointed off the retired LD20+cuVSLAM navigation.launch.py.',
        ),

        # ── Camera ────────────────────────────────────────────────────────────
        # Started first and never killed — keeps Orbbec firmware warm.
        #
        # ⚠ navigation_s2e.launch.py STARTS THE CAMERA ITSELF (color-only MJPG 640x480@15).
        # Two Orbbec nodes on the same serial number fight over the USB device and neither
        # comes up reliably, so this include runs ONLY when nav is off and nothing else
        # is providing it.
        TimerAction(period=0.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'camera.launch.py')
                ),
                condition=UnlessCondition(LaunchConfiguration('enable_nav')),
            ),
        ]),

        # ── Display + screensaver ─────────────────────────────────────────────
        TimerAction(period=0.0, actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'DISPLAY=:0 XAUTHORITY=/run/user/2001/gdm/Xauthority '
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
                additional_env={'DISPLAY': ':0', 'XAUTHORITY': '/run/user/2001/gdm/Xauthority'},
            ),
        ]),

        # ── micro-ROS agent ───────────────────────────────────────────────────
        # Bus 001 (ESP32) — independent of camera Bus 002.
        #
        # ⚠ navigation_s2e.launch.py STARTS THE AGENT ITSELF. Two agents on
        # /dev/jupiter_esp32 both open the port and corrupt each other's session, so this
        # runs only when nav is off. Condition = enable_microros AND NOT enable_nav.
        TimerAction(period=0.0, actions=[
            ExecuteProcess(
                # micro_ros_agent lives in ~/microros_ws (separate overlay) — must be
                # sourced or "Package 'micro_ros_agent' not found". ttyUSB0 = /dev/jupiter_esp32.
                cmd=['bash', '-c',
                     'source "$HOME/microros_ws/install/local_setup.bash" && '
                     'exec ros2 run micro_ros_agent micro_ros_agent '
                     'serial --dev /dev/jupiter_esp32 -b 460800'],
                output='screen',
                name='micro_ros_agent',
                condition=IfCondition(PythonExpression([
                    "'", enable_microros, "' == 'true' and '",
                    LaunchConfiguration('enable_nav'), "' == 'false'"
                ])),
            ),
        ]),

        # ── Navigation stack ──────────────────────────────────────────────────
        # navigation_s2e.launch.py — S2E lidar + AMCL + EKF + Nav2, and it also brings up
        # the micro-ROS agent, the Orbbec camera and every static TF (including
        # base_footprint -> tof_front_link for the front VL53L0X).
        #
        # REPOINTED 2026-08-13 from navigation.launch.py, which is the retired stack:
        # cuVSLAM instead of AMCL, the old c82_map_real map, and a base_laser transform
        # 390 mm too low with the yaw 180 deg out — describing the LD20 that was physically
        # removed on 2026-08-03. Launching it put every scan in the wrong place.
        #
        # Started at t=3s — camera firmware warm by then, no DMA conflict.
        # color 1280x720 is REQUIRED here, not cosmetic: navigation_s2e defaults to 640x480 for
        # standalone nav, but jupiter_vision's AprilTag detector is parameterised for 1280x720
        # and its pose solution is wrong at any other resolution.
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'navigation_s2e.launch.py')
                ),
                launch_arguments={'color_width': '1280', 'color_height': '720'}.items(),
                condition=IfCondition(LaunchConfiguration('enable_nav')),
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
                additional_env={'XDG_RUNTIME_DIR': '/run/user/2001'},  # pw-cat -> PipeWire for TTS
                parameters=[{
                    'energy_threshold': 300.0,
                    'record_seconds':   4,
                    'vad_snr_ratio':    1.7,
                    # HDMI display-speaker sink on JetPack 7.2 (profile changed from hdmi-stereo)
                    'tts_sink':         'alsa_output.platform-88090b0000.hda.HiFi__hw_HDA_3__sink',
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
                    'match_threshold': 0.40,
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

        # ── Docking controller ────────────────────────────────────────────────
        # Engaged by the brain via /dock/engage ("go to the dock"). Drives the FINAL
        # approach only — Jupiter must already be near the dock with the tag in view.
        # Carried over from jupiter_full_s2e.launch.py when that file was retired.
        TimerAction(period=5.0, actions=[
            Node(
                package='jupiter_nodes',
                executable='dock_approach',
                name='dock_approach',
                output='screen',
                parameters=[{
                    'target_distance': 0.40,
                }],
            ),
        ]),

    ])
