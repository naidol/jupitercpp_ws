# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Nav2 Docking Server bringup — the researched replacement for the bespoke dock_ir controller
# (which drove the approach open-loop; see project memory 2026-07-12).
#
# Runs: EKF odometry + rear webcam + AprilTag vision + opennav_docking server.
# NOT here: micro-ROS agent (run persistently, separately) and dock_ir/dock_range (RETIRED —
# never run them alongside this: they'd fight over cmd_vel).
#
#   # once (persistent):
#   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/jupiter_esp32 -b 460800
#   # then:
#   ros2 launch jupiter_bringup dock_opennav.launch.py
#   # bench test (dock from wherever it stands; no Nav2):
#   ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot \
#     "{use_dock_id: false, dock_pose: {header: {frame_id: odom}}, dock_type: jupiter_dock, \
#       navigate_to_staging_pose: false}"
#   # undock:
#   ros2 action send_goal /undock_robot opennav_docking_msgs/action/UndockRobot "{dock_type: jupiter_dock}"
#
# E-STOP during tests: a WARM teleop session ('k') — cold ROS CLI calls take seconds in discovery.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    ekf_config = os.path.join(bringup_dir, 'config', 'ekf_odom.yaml')
    dock_config = os.path.join(bringup_dir, 'config', 'docking_server.yaml')

    return LaunchDescription([

        # Odometry: IMU fix + EKF -> odom->base_footprint TF + /odometry/filtered
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_link_to_imu_link',
             arguments=['0', '0', '0.0675', '0', '0', '0', 'base_footprint', 'imu_link']),
        Node(package='jupiter_nodes', executable='imu_covariance_fixer', name='imu_covariance_fixer',
             output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_node',
             output='screen', parameters=[ekf_config]),

        # Rear docking webcam mount TF — REQUIRED by the docking server (it transforms the tag
        # pose out of the camera frame properly, unlike the retired camera-frame-hack controller).
        # Camera at X=-0.160 (rear), Z=0.4375 (100mm wheels), looking BACKWARD along -X:
        # optical frame = yaw pi (face rear) then optical convention (roll -pi/2, extra yaw -pi/2).
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_webcam_optical',
             arguments=['--x', '-0.160', '--y', '0', '--z', '0.4375',
                        '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                        '--frame-id', 'base_footprint', '--child-frame-id', 'webcam_optical_frame']),

        # Rear webcam -> /webcam/image_raw
        Node(package='jupiter_nodes', executable='webcam_publisher', name='webcam_publisher',
             output='screen',
             parameters=[{'device': 8, 'width': 1280, 'height': 720, 'fps': 30.0}]),

        # AprilTag detector -> /vision/marker_pose (80mm tag36h11, one-point fx calibration)
        Node(package='jupiter_nodes', executable='jupiter_vision', name='jupiter_vision',
             output='screen',
             parameters=[{
                 'camera_topic': '/webcam/image_raw',
                 'image_width': 1280, 'image_height': 720,
                 'marker_size_m': 0.080,
                 'cam_fx': 868.0, 'cam_fy': 868.0, 'cam_cx': 640.0, 'cam_cy': 360.0,
             }]),

        # Nav2 Docking Server (lifecycle) + its manager
        Node(package='opennav_docking', executable='opennav_docking', name='docking_server',
             output='screen', parameters=[dock_config],
             remappings=[('detected_dock_pose', '/vision/marker_pose')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_docking',
             output='screen',
             parameters=[{'autostart': True, 'node_names': ['docking_server']}]),
    ])
