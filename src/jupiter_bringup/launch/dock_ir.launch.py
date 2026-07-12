# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Reverse IR + LiDAR docking.
#   LATERAL (steer) -> IR heartbeat balance   /dock/ir_rate   (dock_ir)
#   DISTANCE (stop) -> S2E wall range          /dock/range     (dock_range)
#   SQUARE          -> the guide rails, mechanically
# Place the robot roughly facing the dock (RECEIVERS toward the beacon), then engage.
#
#   ros2 launch jupiter_bringup dock_ir.launch.py reverse:=true dock_depth:=<m> contact_dist:=<m>
#   # if the lateral steer DIVERGES, flip steer_sign:=-1.0
#
#   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: true}"   # start
#   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: false}"  # stop
#
# NOTE: launch from a shell that has sourced ~/microros_ws/install/setup.bash (the agent
# lives there). Bring-up helpers: dock_range_monitor.py (dock_depth), ir_rate_monitor.py
# (confirm the balance signal).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    ekf_config  = os.path.join(bringup_dir, 'config', 'ekf_odom.yaml')

    reverse        = LaunchConfiguration('reverse')
    steer_sign     = LaunchConfiguration('steer_sign')
    approach_speed = LaunchConfiguration('approach_speed')
    Ky_balance     = LaunchConfiguration('Ky_balance')
    dock_depth     = LaunchConfiguration('dock_depth')
    contact_dist   = LaunchConfiguration('contact_dist')

    return LaunchDescription([

        DeclareLaunchArgument('reverse', default_value='false',
                              description='true = back into the dock caster-first'),
        DeclareLaunchArgument('steer_sign', default_value='1.0',
                              description='flip to -1.0 if the IR balance steer diverges'),
        DeclareLaunchArgument('approach_speed', default_value='0.13',
                              description='m/s CONSTANT drive speed (base needs ~0.13-0.15 to translate)'),
        DeclareLaunchArgument('Ky_balance', default_value='0.40',
                              description='rad/s per unit IR balance (lateral steer gain)'),
        DeclareLaunchArgument('dock_depth', default_value='0.0',
                              description='m — wall -> pogo-contact plane (MEASURE with dock_range_monitor)'),
        DeclareLaunchArgument('contact_dist', default_value='0.0',
                              description='m — stop when dist_to_pogo <= this'),

        # ESP32: receives /cmd_vel, publishes /odom/unfiltered + /dock/ir
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '--dev', '/dev/jupiter_esp32', '-b', '460800'],
            output='screen', name='micro_ros_agent',
        ),

        # Odometry: IMU fix + EKF -> /odometry/filtered (yaw-rate damping + stall detection)
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_link_to_imu_link',
             arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'imu_link']),
        Node(package='jupiter_nodes', executable='imu_covariance_fixer', name='imu_covariance_fixer',
             output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_node',
             output='screen', parameters=[ekf_config]),

        # RPLIDAR S2E — Ethernet/UDP, /scan in frame base_laser (the dock ranging sensor)
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
             output='screen',
             parameters=[{
                 'channel_type': 'udp', 'udp_ip': '192.168.11.2', 'udp_port': 8089,
                 'frame_id': 'base_laser', 'inverted': False,
                 'angle_compensate': True, 'scan_mode': 'Sensitivity',
             }]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_link_to_base_laser',
             arguments=['0.035', '0.0', '0.5325', '3.14159265', '0', '0', 'base_footprint', 'base_laser']),  # +17.5mm: 100mm AGV wheels

        # LiDAR wall ranging: /scan rear sector -> /dock/range (dist_to_pogo, wall_angle)
        Node(package='jupiter_nodes', executable='dock_range', name='dock_range',
             output='screen',
             parameters=[{
                 'scan_topic':       '/scan',
                 'rear_bearing_deg': 0.0,     # S2E rear = bearing 0 (frame yaw pi)
                 'sector_half_deg':  10.0,    # ±10° validated (fit ±2mm @0.8m); ±20° overshot the box edges
                 'dock_depth':       ParameterValue(dock_depth, value_type=float),
             }]),

        # Dock controller: IR balance -> lateral, LiDAR range -> distance/stop, rails -> square
        Node(package='jupiter_nodes', executable='dock_ir', name='dock_ir',
             output='screen',
             parameters=[{
                 'approach_speed': ParameterValue(approach_speed, value_type=float),
                 'reverse':        ParameterValue(reverse, value_type=bool),
                 'steer_sign':     ParameterValue(steer_sign, value_type=float),
                 'Ky_balance':     ParameterValue(Ky_balance, value_type=float),
                 'contact_dist':   ParameterValue(contact_dist, value_type=float),
                 'control_rate':   20.0,
             }]),
    ])
