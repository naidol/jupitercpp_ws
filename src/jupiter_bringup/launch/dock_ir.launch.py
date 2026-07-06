# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# IR beacon docking launch — ESP32 + EKF + dock_ir controller.
# No camera required. Place robot so the RECEIVERS face the dock, then engage.
#
# Forward dock (default):
#   ros2 launch jupiter_bringup dock_ir.launch.py
# Reverse dock (back in caster-first — receivers on the robot rear):
#   ros2 launch jupiter_bringup dock_ir.launch.py reverse:=true
#   # if it steers the WRONG way (diverges), relaunch with: steer_sign:=-1.0
#
#   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: true}"   # start
#   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: false}"  # stop

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
    steer_wz       = LaunchConfiguration('steer_wz')

    return LaunchDescription([

        DeclareLaunchArgument('reverse', default_value='false',
                              description='true = back into the dock caster-first'),
        DeclareLaunchArgument('steer_sign', default_value='1.0',
                              description='flip to -1.0 if reverse steering diverges'),
        DeclareLaunchArgument('approach_speed', default_value='0.08',
                              description='m/s drive speed (lower = gentler, less overshoot)'),
        DeclareLaunchArgument('steer_wz', default_value='0.20',
                              description='rad/s correction rate (lower = gentler, less overshoot)'),

        # ESP32: receives /cmd_vel, publishes /odom/unfiltered + /dock/ir
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '--dev', '/dev/jupiter_esp32', '-b', '460800'],
            output='screen', name='micro_ros_agent',
        ),

        # Odometry: IMU fix + EKF (provides /odometry/filtered for stuck detection)
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_link_to_imu_link',
             arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'imu_link']),
        Node(package='jupiter_nodes', executable='imu_covariance_fixer', name='imu_covariance_fixer',
             output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_node',
             output='screen', parameters=[ekf_config]),

        # IR dock controller
        Node(package='jupiter_nodes', executable='dock_ir', name='dock_ir',
             output='screen',
             parameters=[{
                 'approach_speed':       ParameterValue(approach_speed, value_type=float),
                 'steer_wz':             ParameterValue(steer_wz, value_type=float),
                 'ir_timeout':           1.0,    # s — stop if no IR signal
                 'stuck_vel_threshold':  0.01,   # m/s — below = physically stopped
                 'stuck_timeout':        3.0,    # s — stopped this long = docked
                 'control_rate':         20.0,   # Hz
                 'reverse':    ParameterValue(reverse, value_type=bool),
                 'steer_sign': ParameterValue(steer_sign, value_type=float),
             }]),
    ])
