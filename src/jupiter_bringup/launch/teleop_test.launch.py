# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Diagnostic bring-up: base + lidar + EKF ONLY — no Nav2, no AMCL/map, no camera/nvblox.
#
# Purpose: isolate base-level odometry/heading drift from Nav2/RPP behaviour.
# Drive pure forward with teleop and watch, in RViz with FIXED FRAME = odom:
#   (A) physical path — straight or curving right? (lay a tape line)
#   (B) /scan rigidity — stays locked to the walls, or rotates/smears?
#   (C) ros2 run tf2_ros tf2_echo odom base_footprint — does yaw climb on pure +vx?
#
# AMCL/map are deliberately EXCLUDED: AMCL corrects map->odom to mask odom drift,
# which is exactly what we're trying to measure. Watch the odom frame raw.
#
# Usage:
#   ros2 launch jupiter_bringup teleop_test.launch.py
#   # second terminal:
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    ekf_config = os.path.join(bringup_dir, 'config', 'ekf_odom.yaml')

    return LaunchDescription([

        # micro-ROS agent — ESP32: publishes /odom/unfiltered, subscribes /cmd_vel for the motors
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '--dev', '/dev/jupiter_esp32', '-b', '921600'],
            output='screen', name='micro_ros_agent',
        ),

        # RPLIDAR S2E — Ethernet/UDP, /scan in frame base_laser
        Node(
            package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'udp', 'udp_ip': '192.168.11.2', 'udp_port': 8089,
                'frame_id': 'base_laser', 'inverted': False,
                'angle_compensate': True, 'scan_mode': 'Sensitivity',
            }],
        ),

        # Static TF base_footprint -> base_laser (S2E riser mount; 0deg faces backward = yaw pi)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['0.035', '0.0', '0.515', '3.14159265', '0', '0', 'base_footprint', 'base_laser'],
        ),
        # Static TF base_footprint -> imu_link (BNO055)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'imu_link'],
        ),

        # IMU covariance fixer — BNO055 /imu/data -> /imu/data/corrected
        Node(
            package='jupiter_nodes', executable='imu_covariance_fixer',
            name='imu_covariance_fixer', output='screen',
        ),

        # EKF — wheel odom + BNO055 yaw -> odom->base_footprint TF
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_node',
            output='screen', parameters=[ekf_config],
        ),
    ])
