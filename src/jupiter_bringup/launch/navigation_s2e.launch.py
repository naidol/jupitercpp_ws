# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Autonomous navigation: RPLIDAR S2E + Orbbec 336 (nvblox) + static map + AMCL (no cuVSLAM, no LD20).
#
# Localisation:  AMCL              — scan-matches S2E /scan vs the static map -> map->odom TF
# Static map:    map_server        — the pre-built apartment occupancy grid
# Odometry:      ESP32 + EKF       — odom->base_footprint (wheel + BNO055 yaw)
# Obstacles 2D:  S2E /scan         — Nav2 ObstacleLayer (local + global costmaps), 360 deg @ 0.515m
# Obstacles 3D:  Orbbec depth      — nvblox GPU ESDF -> NvbloxCostmapLayer; catches LOW furniture
#                                    (chairs, table legs, feet) the lidar at 0.515m is blind to
# Navigation:    Nav2 MPPI (DiffDrive — vx+wz, no strafe; strafe kept only for docking)
#
# Camera mount static TF base_footprint->camera_link = 0.100, 0.000, 0.475 (measured 2026-06-10;
#   ~1.3deg nose-down pitch known + DEFERRED as minor). nvblox needs map->base_footprint (AMCL+EKF).
#
# TF chain: map->odom (AMCL) -> odom->base_footprint (EKF) -> base_footprint->{base_laser, camera_link}.
#
# Usage:
#   ros2 launch jupiter_bringup navigation_s2e.launch.py            # uses maps/apartment_s2e_v2.yaml
#   ros2 launch jupiter_bringup navigation_s2e.launch.py map:=/path/to/other_map.yaml
# Then in RViz: set "2D Pose Estimate" where the robot physically is, then "2D Goal Pose".
# Prereq: lidar reachable (persistent alias 192.168.11.100/24 on enP2p1s0 -> S2E 192.168.11.2).

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, TimerAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    nav2_config = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    ekf_config  = os.path.join(bringup_dir, 'config', 'ekf_odom.yaml')
    # Maps saved by map_saver live in the workspace-root maps/ dir (not the package share).
    default_map = os.path.expanduser('~/jupitercpp_ws/maps/apartment_s2e_v2.yaml')

    map_arg = LaunchConfiguration('map')

    return LaunchDescription([

        DeclareLaunchArgument('map', default_value=default_map,
            description='Static map yaml for map_server (default apartment_s2e_v2.yaml).'),

        # ── Sensors / odometry ────────────────────────────────────────────────
        # micro-ROS agent — ESP32: /odom/unfiltered + receives /cmd_vel for the motors
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '--dev', '/dev/jupiter_esp32', '-b', '115200'],
            output='screen', name='micro_ros_agent',
        ),

        # RPLIDAR S2E — Ethernet/UDP, /scan in frame base_laser
        Node(
            package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'udp', 'udp_ip': '192.168.11.2', 'udp_port': 8089,
                'frame_id': 'base_laser', 'inverted': False,
                'angle_compensate': True, 'scan_mode': 'Sensitivity',
            }],
        ),

        # Static TF base_footprint -> base_laser (measured S2E riser mount; 0deg faces backward = yaw pi)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['0.035', '0.0', '0.515', '3.14159265', '0', '0', 'base_footprint', 'base_laser'],
        ),
        # Static TF base_footprint -> imu_link (BNO055)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'imu_link'],
        ),

        # ── Orbbec 336 camera (DEPTH-ON for nvblox) ───────────────────────────
        # Depth + point cloud + laser projector ON — the OPPOSITE of the cuVSLAM-era
        # camera.launch.py (which has them off). IR streams off (nvblox uses depth only).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('orbbec_camera'),
                'launch', 'gemini_330_series.launch.py')),
            launch_arguments={
                'serial_number': 'CP9KB53000HP', 'usb_port': '2-1',
                'enable_color': 'true', 'color_width': '640', 'color_height': '480',
                'color_fps': '15', 'color_format': 'MJPG',
                'enable_depth': 'true', 'depth_width': '640', 'depth_height': '480',
                'depth_fps': '15', 'enable_point_cloud': 'true', 'enable_laser': 'true',
                'enable_left_ir': 'false', 'enable_right_ir': 'false',
            }.items(),
        ),
        # Static TF base_footprint -> camera_link (measured 2026-06-10: 0.100/0/0.475).
        # ~1.3deg nose-down pitch known + deferred as minor; add a --pitch term here when fixed.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_camera_link',
            arguments=['--x', '0.100', '--y', '0.000', '--z', '0.475',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_footprint', '--child-frame-id', 'camera_link'],
        ),

        # IMU covariance fixer — BNO055 /imu/data -> /imu/data/corrected (no remap; not the camera IMU)
        Node(
            package='jupiter_nodes', executable='imu_covariance_fixer',
            name='imu_covariance_fixer', output='screen',
        ),

        # EKF — wheel odom + BNO055 yaw -> odom->base_footprint TF
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_node',
            output='screen', parameters=[ekf_config],
        ),

        # ── Localisation: static map + AMCL ───────────────────────────────────
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[nav2_config, {'yaml_filename': map_arg}],
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            output='screen', parameters=[nav2_config],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'use_sim_time': False, 'autostart': True, 'bond_timeout': 0.0,
                         'node_names': ['map_server', 'amcl']}],
        ),

        # ── Nav2 navigation stack ─────────────────────────────────────────────
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[nav2_config], remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
             output='screen', parameters=[nav2_config]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav2_config]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[nav2_config]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[nav2_config]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
             output='screen', parameters=[nav2_config]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=[nav2_config],
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),

        # Navigation lifecycle manager — delayed 5s so the MPPI CUDA kernel finishes init
        # before the manager pings the controller_server (else it misses the service deadline).
        TimerAction(period=5.0, actions=[
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_navigation', output='screen',
                parameters=[{'use_sim_time': False, 'autostart': True, 'bond_timeout': 0.0,
                             'node_names': ['controller_server', 'smoother_server', 'planner_server',
                                            'behavior_server', 'bt_navigator', 'waypoint_follower',
                                            'velocity_smoother']}],
            ),
        ]),

        # ── nvblox (Orbbec depth -> ESDF slice for NvbloxCostmapLayer) ─────────
        # Delayed 8s: needs the full map->odom->base_footprint TF (AMCL + EKF) to place the
        # reconstruction, plus the camera depth stream flowing. Reuses nvblox.launch.py so the
        # nvblox config lives in exactly one place.
        TimerAction(period=8.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    bringup_dir, 'launch', 'nvblox.launch.py'))),
        ]),
    ])
