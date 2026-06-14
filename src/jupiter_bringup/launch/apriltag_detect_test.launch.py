# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# AprilTag DETECTION test — Orbbec color @1280x720 + jupiter_vision (VPI tag36h11).
# Purpose: verify the tag-pose pipeline BEFORE building the docking server on top of it.
#
# IMPORTANT: color MUST be 1280x720 — the vision node's intrinsics (cx~637.8, cy~362.8)
# are calibrated for 1280x720. Running any other resolution scales the pose wrongly.
#
# Verify:
#   ros2 launch jupiter_bringup apriltag_detect_test.launch.py
#   # hold a printed tag (id matches docking_tag_id) a measured distance from the camera:
#   ros2 topic echo /vision/marker_pose
#   # position.z should ~= the tape-measured distance to the tag (camera optical axis).
#
# marker_size_m is passed here (0.149 = measured black-border edge) so it applies on the
# already-built binary without a rebuild; the source default was also set to 0.149.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    tag_id = LaunchConfiguration('docking_tag_id')

    return LaunchDescription([
        DeclareLaunchArgument('docking_tag_id', default_value='1',
            description='tag36h11 ID mounted on the docking station (printed set is id 1-6).'),

        # Orbbec Gemini 336 — COLOR ONLY @1280x720 (matches the vision node calibration).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('orbbec_camera'),
                'launch', 'gemini_330_series.launch.py')),
            launch_arguments={
                'serial_number': 'CP9KB53000HP', 'usb_port': '',
                'enable_color': 'true', 'color_width': '1280', 'color_height': '720',
                'color_fps': '15', 'color_format': 'MJPG',
                'enable_depth': 'false', 'enable_point_cloud': 'false', 'enable_laser': 'false',
                'enable_left_ir': 'false', 'enable_right_ir': 'false',
            }.items(),
        ),

        # AprilTag detector (VPI tag36h11) -> /vision/marker_pose (PoseStamped in camera_link)
        Node(
            package='jupiter_nodes', executable='jupiter_vision', name='jupiter_vision',
            output='screen',
            parameters=[{
                'camera_topic':   '/camera/color/image_raw',
                'image_width':    1280,
                'image_height':   720,
                'marker_size_m':  0.149,
                'docking_tag_id': tag_id,
            }],
        ),
    ])
