# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Autonomous navigation — hybrid stack (cuVSLAM localisation + LD20 LiDAR obstacles).
#
# Localisation:  Isaac ROS cuVSLAM  — publishes map→odom→base_footprint TF
# Static map:    map_server          — pre-built LiDAR occupancy grid (StaticLayer)
# Obstacle map:  LD20 LiDAR         — /scan → Nav2 ObstacleLayer costmap
# Navigation:    Nav2 MPPI          — StaticLayer + ObstacleLayer fused in costmaps
#
# Jupiter must start at the map origin (floor marker) so cuVSLAM map frame aligns
# with the static occupancy grid coordinate frame.
#
# Usage:
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py \
#     enable_microros:=true enable_nav:=true mode:=nav enable_voice:=false

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    nav2_config = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    ekf_config  = os.path.join(bringup_dir, 'config', 'ekf_odom.yaml')
    map_yaml    = os.path.join(bringup_dir, 'maps', 'c82_map_real.yaml')

    return LaunchDescription([

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

        # Static TF: map → odom (identity)
        # cuVSLAM map→odom is disabled — its VIO diverges in the passage (poor IR features
        # near featureless cupboard), producing a tilted/Z-offset transform that lifts the
        # costmaps off the floor. Identity here keeps map=odom so local odometry drives Nav2
        # correctly until cuVSLAM can be properly initialised in a feature-rich space.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_identity',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),

        # Static TF: base_footprint → base_laser
        # LD20 physical mounting: 6 cm forward, centred, 13 cm above ground.
        # Yaw=pi: LiDAR mounted 180° reversed (cable exits toward front of robot).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['0.06', '0', '0.13', '0', '0', '0', 'base_footprint', 'base_laser'],
        ),

        # IMU Covariance Fixer — adds valid variances to the Orbbec IMU data
        # so the EKF node doesn't ignore the measurements.
        Node(
            package='jupiter_nodes',
            executable='imu_covariance_fixer',
            name='imu_covariance_fixer',
            output='screen',
            remappings=[('/imu/data', '/camera/gyro_accel/sample')],
        ),

        # EKF — fuses ESP32 wheel odometry into odom→base_footprint TF.
        # Wheel encoders handle mecanum lateral (vy) motion correctly; VIO cannot.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config],
        ),

        # cuVSLAM — visual odometry + localization (map→odom TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'visual_slam.launch.py')
            )
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_config],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_config],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_config],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_config],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_config],
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_config],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_config],
            remappings=[
                ('cmd_vel',          'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),

        # Lifecycle manager delayed 5s — MPPI CUDA kernel init causes the controller_server
        # to miss the service response deadline if the lifecycle_manager starts immediately.
        # 5s gives all Nav2 nodes time to fully initialise their RMW layer first.
        TimerAction(period=5.0, actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'bond_timeout': 0.0,
                    'node_names': [
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother',
                    ],
                }],
            ),
        ]),
    ])
