# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# OFFLINE A/B test harness for the scan_deskew_node. Same as offline_mapping.launch.py, but
# inserts scan_deskew_node between the replayed /scan and slam_toolbox:
#
#   bag /scan ──► scan_deskew_node ──► /scan/deskewed ──► slam_toolbox
#                      ▲
#   bag /odom/unfiltered (twist: vx,vy,omega) ──┘
#
# Lets us prove (free, no robot) whether motion-compensating the LD20's 166 ms sweep removes the
# rotational haze, by comparing the resulting map against the raw-scan map from offline_mapping.
#
# Usage:
#   ros2 launch jupiter_bringup offline_mapping_deskew.launch.py bag_path:=/path/to/bag
#   # turn deskew off for a matched-plumbing baseline:
#   ros2 launch jupiter_bringup offline_mapping_deskew.launch.py bag_path:=/path/to/bag deskew:=false
# Save while alive:
#   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '<path>'}}"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    slam_config = os.path.join(bringup_dir, 'config', 'slam_params.yaml')

    bag_path = LaunchConfiguration('bag_path')
    rate     = LaunchConfiguration('rate')
    deskew   = LaunchConfiguration('deskew')
    stamp_position = LaunchConfiguration('stamp_position')

    return LaunchDescription([
        DeclareLaunchArgument('bag_path',
            description='Bag recorded by record_mapping.launch.py (needs /scan, /tf, /odom/unfiltered).'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('deskew', default_value='true',
            description='true = deskew before SLAM; false = passthrough baseline.'),
        DeclareLaunchArgument('stamp_position', default_value='end',
            description='Where header.stamp sits in the sweep: start|mid|end.'),

        # base_footprint -> base_laser (deskew works in laser frame; SLAM needs this for the chain)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['0.06', '0', '0.13', '0', '0', '0', 'base_footprint', 'base_laser'],
        ),

        # The node under test: raw /scan + bag twist -> /scan/deskewed
        Node(
            package='jupiter_nodes', executable='scan_deskew_node', name='scan_deskew_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'input_scan_topic': '/scan',
                'output_scan_topic': '/scan/deskewed',
                'odom_topic': '/odom/unfiltered',
                'enable_deskew': deskew,
                'stamp_position': stamp_position,
            }],
        ),

        # slam_toolbox consuming the DESKEWED scan
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {'use_sim_time': True, 'scan_topic': '/scan/deskewed'}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_slam', output='screen',
            parameters=[{'use_sim_time': True, 'autostart': True,
                         'node_names': ['slam_toolbox'], 'bond_timeout': 0.0}],
        ),

        # Replay scan + dynamic tf + wheel odom (twist source for the deskew node)
        TimerAction(period=4.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag_path, '--clock', '--rate', rate,
                     '--topics', '/scan', '/tf', '/odom/unfiltered'],
                output='screen', name='mapping_bag_play',
            ),
        ]),
    ])
