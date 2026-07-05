# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# RECORD phase of the rosbag-based mapping workflow (for large maps, e.g. the
# 220 m² apartment). This brings up ONLY the cheap, real-time-safe producers and
# records their output to a bag. slam_toolbox (the expensive part that backs up
# and smears on long runs) is deliberately NOT launched here — it runs afterward,
# offline, against the recorded bag (see offline_mapping.launch.py).
#
# Why this fixes the "SLAM clogs on a big map" problem:
#   - Recording is pure disk I/O. The LiDAR + ESP32 odom + EKF are lightweight and
#     never fall behind, so there is no async queue to overflow while you drive.
#   - The bag captures the already-fused /tf (odom->base_footprint from the EKF)
#     plus /scan, so the offline pass only needs to run slam_toolbox.
#   - Raw /imu/data and /odom/unfiltered are also recorded so the EKF could be
#     re-run offline with different tuning if ever needed.
#
# Pipeline (identical to slam_mapping.launch.py, minus slam_toolbox):
#   ESP32 (micro-ROS) -> /odom/unfiltered -> EKF -> odom->base_footprint TF
#   LD20 LiDAR        -> /scan (base_laser frame)
#   ros2 bag record   -> /scan /tf /tf_static /odom/unfiltered /imu/data /imu/data/corrected
#
# Usage (drive with teleop_twist_keyboard in another terminal -> /cmd_vel):
#   ros2 launch jupiter_bringup record_mapping.launch.py
#   ros2 launch jupiter_bringup record_mapping.launch.py bag_path:=/path/to/bag
# Ctrl-C finalizes the bag. Then build the map with offline_mapping.launch.py.
# (Or just run ./record_mapping_run.sh from the workspace root.)

from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    ekf_config  = os.path.join(bringup_dir, 'config', 'ekf_odom.yaml')

    # Default bag path: timestamped under the workspace maps/bags/ directory.
    # ros2 bag record requires the target dir NOT to already exist.
    default_bag = os.path.join(
        os.path.expanduser('~/jupitercpp_ws/maps/bags'),
        'apt_run_' + datetime.now().strftime('%Y%m%d_%H%M%S'),
    )

    bag_path = LaunchConfiguration('bag_path')

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_path',
            default_value=default_bag,
            description='Output bag directory (must not already exist).',
        ),

        # micro-ROS agent — ESP32 mecanum wheel odometry → /odom/unfiltered
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '--dev', '/dev/jupiter_esp32', '-b', '115200'],
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

        # Static TF: base_footprint → base_laser (6 cm fwd, 13 cm up, yaw 0)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['0.06', '0', '0.13', '0', '0', '0', 'base_footprint', 'base_laser'],
        ),

        # Static TF: base_footprint → imu_link (BNO055 mount, ~5 cm up, yaw 0)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'imu_link'],
        ),

        # IMU covariance fixer — /imu/data → /imu/data/corrected (valid covariances)
        Node(
            package='jupiter_nodes',
            executable='imu_covariance_fixer',
            name='imu_covariance_fixer',
            output='screen',
        ),

        # EKF — fuses wheel odom + BNO055 yaw-rate → odom->base_footprint TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config],
        ),

        # Record the bag. /scan + /tf + /tf_static are all the offline pass needs;
        # the raw /imu and /odom topics are extra insurance for re-tuning the EKF.
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_path,
                 '/scan', '/tf', '/tf_static',
                 '/odom/unfiltered', '/imu/data', '/imu/data/corrected'],
            output='screen',
            name='mapping_bag_record',
        ),
    ])
