# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# FULL-STACK bring-up (current S2E architecture) — the VOICE | VISION | AI | NAV coexistence test.
#
# Brings up, on the Thor, all at once:
#   - NAV: S2E lidar + AMCL + EKF + Nav2 + nvblox (depth costmap)   [via navigation_s2e.launch.py]
#   - CAMERA: one Orbbec serving depth@640 (nvblox) AND color@1280 (AprilTag detector)
#   - AI: voice/Whisper (ASR), brain, face recognition (TensorRT), vision (AprilTag)
#   - DOCKING: dock_approach (engaged by the brain via /dock/engage)
#   - Display HUD
#
# Startup is STAGGERED so the GPU consumers don't all spin up together (nvblox @8s inside
# navigation_s2e; Whisper loads ~1.5 GB to the GPU @12s; brain/face/vision/dock @14s).
#
# Voice docking: say "go to the dock" -> brain publishes /dock/engage -> dock_approach drives in.
#   (Triggers the FINAL approach only: Jupiter must already be near the dock with the tag in view.
#    Navigate-from-across-the-room-then-dock is a later integration.)
#
# *** EXTERNAL deps NOT started here — run these too for the full contention test: ***
#   - LLM server (vLLM / llama.cpp) — the brain queries it over HTTP; without it, brain replies fail.
#   - Pi5 audio node (ReSpeaker capture -> /voice/raw_text) — auto-starts on the Pi5.
#   - TTS (piper) — the spoken-response path.
#
# WATCH during the test (separate terminals):
#   tegrastats                                   # GPU/CPU/RAM/EMC — is anything pegged?
#   ros2 topic hz /scan                          # lidar still 10 Hz under load?
#   (controller_server log)                      # "Control loop missed its desired rate" = nav starved
#   tf2_monitor map base_footprint               # TF latency climbing?
#
# Usage:
#   ros2 launch jupiter_bringup jupiter_full_s2e.launch.py

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')

    return LaunchDescription([

        # ── NAV + camera (color 1280x720 for the tag detector, depth 640x480 for nvblox) ──
        # navigation_s2e has its own internal delays (Nav2 lifecycle @5s, nvblox @8s).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'navigation_s2e.launch.py')),
            launch_arguments={'color_width': '1280', 'color_height': '720'}.items(),
        ),

        # ── Display HUD + screensaver disable (DISPLAY/XAUTHORITY for the 7" panel) ──
        ExecuteProcess(
            cmd=['bash', '-c',
                 'DISPLAY=:0 XAUTHORITY=/run/user/2002/gdm/Xauthority xset s off s noblank dpms 0 0 0'],
            output='screen', name='disable_screensaver'),
        Node(package='jupiter_display', executable='jupiter_display', name='jupiter_display', output='screen',
             additional_env={'DISPLAY': ':0', 'XAUTHORITY': '/run/user/2002/gdm/Xauthority'}),

        # ── Voice + Whisper @12s — after nvblox is up, so the GPU loads stagger ──
        TimerAction(period=12.0, actions=[
            Node(package='jupiter_nodes', executable='jupiter_voice', name='jupiter_voice', output='screen',
                 parameters=[{'energy_threshold': 300.0, 'record_seconds': 4, 'vad_snr_ratio': 1.7}]),
        ]),

        # ── Brain + Face recognition + AprilTag vision + Docking @14s ──
        TimerAction(period=14.0, actions=[
            Node(package='jupiter_nodes', executable='jupiter_brain', name='jupiter_brain', output='screen'),
            Node(package='jupiter_nodes', executable='jupiter_face_recognition', name='jupiter_face_recognition',
                 output='screen', parameters=[{'match_threshold': 0.40}]),
            # AprilTag detector — default params already = color@1280x720, marker 0.149, tag id 1, optical frame.
            Node(package='jupiter_nodes', executable='jupiter_vision', name='jupiter_vision', output='screen'),
            # Docking controller — engaged by the brain via /dock/engage ("go to the dock").
            Node(package='jupiter_nodes', executable='dock_approach', name='dock_approach', output='screen',
                 parameters=[{'target_distance': 0.40}]),
        ]),
    ])
