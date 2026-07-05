# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# LiDAR + wheel-odometry SLAM mapping (slam_toolbox) — the reliable, vision-free mapper.
#
# Replaces cuVSLAM for mapping. Uses only the rock-solid parts of Jupiter: ESP32
# mecanum wheel odometry + LD20 LiDAR. No camera, no light dependency, no visual
# features required. Produces a /map occupancy grid (the real 2D map).
#
# Pipeline:
#   ESP32 (micro-ROS) → /odom/unfiltered → EKF → odom→base_footprint TF
#   LD20 LiDAR        → /scan (base_laser frame)
#   slam_toolbox      → scan-matches /scan against odom → map→odom TF + /map grid
#
# IMU: BNO055 (on the ESP32, camera-free). The imu_covariance_fixer reads the raw
# BNO055 /imu/data, stamps valid covariances, and republishes /imu/data/corrected,
# which the EKF (ekf_odom.yaml, imu0) fuses for YAW-RATE only. This gives the odometry
# a heading reference independent of wheel slip and LiDAR geometry, so the map stays
# crisp through featureless corridors where scan-matching alone would drift/smear.
# Only the gyro yaw-rate is used; the BNO055 magnetometer/absolute-heading is ignored,
# so its calibration is irrelevant here.
#
# View the map: RViz, Add → Map, topic /map, Fixed Frame = map.
# Save the map after driving:
#   ros2 run nav2_map_server map_saver_cli -f /home/jupiter/jupitercpp_ws/maps/lab_map
#
# Usage (drive with teleop_twist_keyboard in another terminal → /cmd_vel):
#   ros2 launch jupiter_bringup slam_mapping.launch.py

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    ekf_config  = os.path.join(bringup_dir, 'config', 'ekf_odom.yaml')
    slam_config = os.path.join(bringup_dir, 'config', 'slam_params.yaml')

    return LaunchDescription([

        # micro-ROS agent — ESP32 mecanum wheel odometry → /odom/unfiltered
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '--dev', '/dev/jupiter_esp32', '-b', '921600'],
            output='screen',
            name='micro_ros_agent',
        ),

        # LD20 LiDAR — 360° scan on /scan, frame: base_laser
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD19',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD19'},
                {'topic_name': 'scan'},
                {'frame_id': 'base_laser'},
                {'port_name': '/dev/jupiter_lidar'},
                {'port_baudrate': 230400},
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False},
            ],
        ),

        # Static TF: base_footprint → base_laser
        # LD20 mounting: 6 cm forward, centred, 13 cm above ground, yaw 0
        # (LiDAR is in correct physical orientation — no software yaw compensation).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['0.06', '0', '0.13', '0', '0', '0', 'base_footprint', 'base_laser'],
        ),

        # Static TF: base_footprint → imu_link (BNO055 mount).
        # REQUIRED — without it robot_localization cannot transform the IMU's angular
        # velocity into the base frame and silently drops every IMU measurement.
        # Flat mount (z-up), ~5 cm above base; yaw 0 (chip X-axis points to robot front).
        # Only yaw-rate is fused, so this rotation just needs the IMU z-axis vertical.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'imu_link'],
        ),

        # IMU covariance fixer — BNO055 (/imu/data, no remap) → /imu/data/corrected.
        # Raw BNO055 ships zero covariances, which the EKF would reject; this stamps
        # valid ones so the EKF accepts the yaw-rate measurement.
        Node(
            package='jupiter_nodes',
            executable='imu_covariance_fixer',
            name='imu_covariance_fixer',
            output='screen',
        ),

        # EKF — fuses ESP32 wheel odometry + BNO055 yaw-rate into odom→base_footprint TF.
        # Wheel encoders give x/y/vyaw; the IMU yaw-rate anchors heading so it doesn't
        # drift on wheel slip or in featureless areas (keeps the slam_toolbox map crisp).
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config],
        ),

        # slam_toolbox — async mapping. Scan-matches /scan against odom to build the
        # map and publish map→odom. Delayed 5s so the ESP32 has connected, the EKF is
        # publishing odom→base_footprint, and /scan is flowing before it configures.
        TimerAction(period=5.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_config],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['slam_toolbox'],
                    'bond_timeout': 0.0,
                }],
            ),
        ]),
    ])
