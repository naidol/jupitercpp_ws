# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('jupiter_bringup'),
        'config', 'ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='jupiter_nodes',
            executable='imu_covariance_fixer',
            name='imu_covariance_fixer',
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
        # Static transform: base_footprint -> imu_link
        # Adjust x/y/z if the BNO055 is offset from the robot centre.
        # Adjust yaw if the chip X-axis does not point toward the robot's front.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_link_broadcaster',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.05',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'imu_link',
            ],
        ),
    ])
