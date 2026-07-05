# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# OFFLINE phase of the rosbag-based mapping workflow. Replays a bag recorded by
# record_mapping.launch.py into a SOLO slam_toolbox. Because nothing else is
# running (no driving, no LiDAR driver, no EKF competing for CPU) and playback
# rate is under your control, slam_toolbox can take all the wall-clock time it
# needs per scan — the async queue never overflows, so a 220 m² map builds cleanly
# regardless of how long the drive took. Re-runnable with different params without
# re-driving the apartment.
#
# What is replayed vs. published live:
#   - From the bag (--topics): /scan and /tf (odom->base_footprint from the EKF).
#   - Published live here: the base_footprint->base_laser static TF. (Replaying
#     transient_local /tf_static across a late-joining subscriber is fragile, so we
#     publish it deterministically instead and exclude /tf_static from playback.)
#   - slam_toolbox itself publishes map->odom + /map.
#   TF chain: map->odom (slam) + odom->base_footprint (bag) + base_footprint->base_laser (live).
#
# Usage:
#   ros2 launch jupiter_bringup offline_mapping.launch.py bag_path:=/path/to/bag
#   # if slam still can't keep up on a huge bag, slow playback down:
#   ros2 launch jupiter_bringup offline_mapping.launch.py bag_path:=/path/to/bag rate:=0.5
#
# When playback finishes slam_toolbox stays up — save the map, then Ctrl-C:
#   ros2 run nav2_map_server map_saver_cli -f /home/jupiter/jupitercpp_ws/maps/apartment

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

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_path',
            description='Path to the bag recorded by record_mapping.launch.py (required).',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
            description='Playback rate. Lower it (e.g. 0.5) if slam_toolbox falls '
                        'behind on a very large bag.',
        ),

        # Static TF: base_footprint → base_laser (must match the record-time mount).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['0.06', '0', '0.13', '0', '0', '0', 'base_footprint', 'base_laser'],
        ),

        # slam_toolbox — solo, sim time driven by the bag's /clock.
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {'use_sim_time': True}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['slam_toolbox'],
                'bond_timeout': 0.0,
            }],
        ),

        # Play the bag after slam_toolbox has configured. --clock drives sim time;
        # we replay only the dynamic topics and let the static TF above stand in.
        TimerAction(period=4.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag_path,
                     '--clock', '--rate', rate,
                     '--topics', '/scan', '/tf'],
                output='screen',
                name='mapping_bag_play',
            ),
        ]),
    ])
