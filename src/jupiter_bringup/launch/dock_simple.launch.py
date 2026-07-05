# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Simple AprilTag docking — ESP32 + EKF + Orbbec color@1280x720 + jupiter_vision + dock_approach.
# No opennav, no lifecycle, no frame-convention machinery. Just: see tag -> drive -> stop at 0.40 m.
#
# Prereq: colcon build --packages-select jupiter_nodes jupiter_bringup
#
# Run:
#   ros2 launch jupiter_bringup dock_simple.launch.py
#   # place robot ~1.5 m in front of tag 1, then:
#   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: true}"    # start
#   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: false}"   # stop anytime

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    ekf_config  = os.path.join(bringup_dir, 'config', 'ekf_odom.yaml')
    target_distance = LaunchConfiguration('target_distance')

    return LaunchDescription([

        # base_footprint -> tag stop distance. 0.28 drives ~120mm closer than the old 0.40 standoff, to
        # seat the dock pogo pins. Fine-tune contact depth at launch, e.g. target_distance:=0.26 (firmer)
        # or 0.30 (lighter).
        DeclareLaunchArgument('target_distance', default_value='0.22'),

        # ESP32: receives /cmd_vel, publishes /odom/unfiltered
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '--dev', '/dev/jupiter_esp32', '-b', '921600'],
            output='screen', name='micro_ros_agent',
        ),

        # Odometry: IMU fix + EKF (odom -> base_footprint)
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_link_to_imu_link',
             arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'imu_link']),
        Node(package='jupiter_nodes', executable='imu_covariance_fixer', name='imu_covariance_fixer',
             output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_node',
             output='screen', parameters=[ekf_config]),

        # Camera mount TF (Orbbec driver adds camera_link -> camera_color_optical_frame).
        # Pitch -0.0995 rad = 5.7deg nose-UP (negative per REP-103): camera tilted up for face-rec.
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_camera_link',
             arguments=['--x', '0.100', '--y', '0.000', '--z', '0.475',
                        '--roll', '0', '--pitch', '-0.0995', '--yaw', '0',
                        '--frame-id', 'base_footprint', '--child-frame-id', 'camera_link']),

        # Orbbec 336 — COLOR ONLY @1280x720 (matches the vision node calibration)
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

        # AprilTag detector -> /vision/marker_pose (camera_color_optical_frame)
        Node(package='jupiter_nodes', executable='jupiter_vision', name='jupiter_vision',
             output='screen',
             parameters=[{
                 'camera_topic':    '/camera/color/image_raw',
                 'image_width':     1280,
                 'image_height':    720,
                 'marker_size_m':   0.100,
                 'docking_tag_id':  1,
                 'marker_frame_id': 'camera_color_optical_frame',
             }]),

        # The simple docking controller
        Node(package='jupiter_nodes', executable='dock_approach', name='dock_approach',
             output='screen',
             parameters=[{
                 'target_distance':   ParameterValue(target_distance, value_type=float),
                 'base_frame':        'base_footprint',
                 'max_linear':        0.12,
                 'max_angular':       0.5,
                 'detection_timeout': 0.5,
             }]),
    ])
