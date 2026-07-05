# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# SLAM mapping with the RPLIDAR S2E (Ethernet) in place of the LD20 (USB).
# Identical odom/EKF/slam_toolbox chain to slam_mapping.launch.py — only the lidar
# driver changes: sllidar_ros2 (UDP) instead of ldlidar_stl_ros2 (serial).
#
# The LD20 can stay physically plugged in; just don't run BOTH drivers (both publish
# /scan). This launch runs ONLY the S2E driver.
#
# PREREQS:
#   - S2E powered (12V) + on the switch, reachable at its factory default 192.168.11.2.
#   - Thor has an alias IP on that subnet (TEMPORARY, gone on reboot):
#       sudo ip addr add 192.168.11.100/24 dev enP2p1s0
#     (Permanent later: reconfigure lidar to 10.0.0.3 via RoboStudio, or persist the alias.)
#
# >>> TODO: the base_footprint->base_laser TF below is a PLACEHOLDER for the temporary
#     ~0.45 m top placement. MEASURE the real x (forward), y (lateral), z (height) of the
#     S2E's optical centre from base_footprint and update before trusting the map. <<<
#
# Usage:
#   ros2 launch jupiter_bringup slam_mapping_s2e.launch.py
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard      # drive
#   ros2 run nav2_map_server map_saver_cli -f <path>          # save

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
            output='screen', name='micro_ros_agent',
        ),

        # RPLIDAR S2E — Ethernet/UDP, publishes /scan in frame base_laser
        Node(
            package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'udp',
                'udp_ip': '192.168.11.2',
                'udp_port': 8089,
                'frame_id': 'base_laser',       # must match the static TF below + slam base_frame chain
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
        ),

        # Static TF: base_footprint → base_laser
        # MEASURED 2026-06-09: S2E on its riser, x=3.5cm fwd, centred, scan plane 0.515 m up, level.
        # YAW = pi (180 deg): verified in RViz that the lidar's 0deg is mounted facing BACKWARD, so the
        # base_laser frame is rotated 180 deg about z relative to base_footprint. args = x y z yaw pitch roll.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['0.035', '0.0', '0.515', '3.14159265', '0', '0', 'base_footprint', 'base_laser'],
        ),

        # Static TF: base_footprint → imu_link (BNO055 mount, ~5 cm up, yaw 0)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'imu_link'],
        ),

        # IMU covariance fixer — /imu/data → /imu/data/corrected
        Node(
            package='jupiter_nodes', executable='imu_covariance_fixer',
            name='imu_covariance_fixer', output='screen',
        ),

        # EKF — wheel odom + BNO055 yaw-rate → odom→base_footprint TF
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_node',
            output='screen', parameters=[ekf_config],
        ),

        # slam_toolbox — delayed 5 s so ESP32 + EKF + /scan are flowing first
        TimerAction(period=5.0, actions=[
            Node(
                package='slam_toolbox', executable='async_slam_toolbox_node',
                name='slam_toolbox', output='screen', parameters=[slam_config],
            ),
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_slam', output='screen',
                parameters=[{'use_sim_time': False, 'autostart': True,
                             'node_names': ['slam_toolbox'], 'bond_timeout': 0.0}],
            ),
        ]),
    ])
