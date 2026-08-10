# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Docking v2 NODES only (clean-slate aligner):
#   dock_reflector + dock_aligner_v2
#
# This launch intentionally does NOT start:
# - micro_ros_agent (keep one long-lived agent in microros_ws)
# - sllidar_node
# - static TF publisher for base_footprint -> base_laser
#
# Reason: restarting those shared runtime processes during docking iteration can churn
# the ESP32 connection and add unrelated variables.
#
# Usage:
#   ros2 launch jupiter_bringup dock_reflector_aligner_v2_nodes.launch.py
#   ros2 service call /dock/v2/align_start std_srvs/srv/Trigger "{}"
#   ros2 topic echo /dock/v2/aligner_state std_msgs/msg/String

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Reflector topics.
    launch_reflector = LaunchConfiguration('launch_reflector')
    scan_topic = LaunchConfiguration('scan_topic')
    reflector_topic = LaunchConfiguration('reflector_topic')
    confidence_topic = LaunchConfiguration('confidence_topic')

    # v2 control topics/services.
    v2_state_topic = LaunchConfiguration('v2_state_topic')
    v2_start_service = LaunchConfiguration('v2_start_service')
    v2_cancel_service = LaunchConfiguration('v2_cancel_service')

    # Shared robot I/O.
    imu_topic = LaunchConfiguration('imu_topic')
    contact_topic = LaunchConfiguration('contact_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_reflector', default_value='true',
            description='Set false to reuse an already-running dock_reflector node.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='Input LaserScan topic for dock_reflector.'),
        DeclareLaunchArgument('reflector_topic', default_value='/dock/reflector',
                              description='Reflector debug topic consumed by dock_aligner_v2.'),
        DeclareLaunchArgument('confidence_topic', default_value='/dock/reflector_confidence',
                              description='Reflector confidence topic consumed by dock_aligner_v2.'),

        DeclareLaunchArgument('v2_state_topic', default_value='/dock/v2/aligner_state',
                              description='State output topic for dock_aligner_v2.'),
        DeclareLaunchArgument('v2_start_service', default_value='/dock/v2/align_start',
                              description='Trigger service to start one v2 docking run.'),
        DeclareLaunchArgument('v2_cancel_service', default_value='/dock/v2/align_cancel',
                              description='Cancel service for v2 docking.'),

        DeclareLaunchArgument('imu_topic', default_value='/imu/data',
                              description='IMU topic for heading hold.'),
        DeclareLaunchArgument('contact_topic', default_value='/dock/contact',
                              description='Dock contact bitmask topic from ESP32.'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='Command velocity topic for motion output.'),

        # Existing 3-strip reflector detector.
        Node(
            package='jupiter_nodes', executable='dock_reflector', name='dock_reflector',
            output='screen',
            condition=IfCondition(launch_reflector),
            parameters=[{
                'scan_topic': scan_topic,
                'debug_topic': reflector_topic,
                'confidence_topic': confidence_topic,
                'pose_topic': '/dock/reflector_pose',
                'target_frame': 'base_footprint',
            }],
        ),

        # Clean-slate aligner (v2). Uses isolated services/state topic by default.
        Node(
            package='jupiter_nodes', executable='dock_aligner_v2', name='dock_aligner_v2',
            output='screen',
            parameters=[{
                'reflector_topic': reflector_topic,
                'confidence_topic': confidence_topic,
                'imu_topic': imu_topic,
                'contact_topic': contact_topic,
                'cmd_vel_topic': cmd_vel_topic,
                'state_topic': v2_state_topic,
                'start_service': v2_start_service,
                'cancel_service': v2_cancel_service,
            }],
        ),
    ])
