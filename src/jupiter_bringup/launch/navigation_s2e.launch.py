# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Autonomous navigation: RPLIDAR S2E (high) + LD20 (low) dual-lidar + static map + AMCL (no cuVSLAM).
#
# Localisation:  AMCL              — scan-matches S2E /scan vs the static map -> map->odom TF
# Static map:    map_server        — the pre-built apartment occupancy grid
# Odometry:      ESP32 + EKF       — odom->base_footprint (wheel + BNO055 yaw)
# Obstacles hi:  S2E /scan         — Nav2 ObstacleLayer (local + global), 360 deg @ 0.515m
# Obstacles lo:  LD20 /scan_low    — Nav2 ObstacleLayer 2nd source @ ~0.13m; catches LOW furniture
#                                    (chair legs, feet, kick-boards) the S2E plane is blind to.
#                                    REPLACES nvblox (camera now tilted up for face-rec -> bad depth).
# Camera:        Orbbec COLOR ONLY — freed for face-rec + AprilTag docking (full stack); no depth.
# Navigation:    Nav2 MPPI (DiffDrive — vx+wz, no strafe; strafe kept only for docking)
#
# Camera mount static TF base_footprint->camera_link = 0.100, 0.000, 0.475, pitch -0.0995 (5.7deg up).
#
# TF chain: map->odom (AMCL) -> odom->base_footprint (EKF)
#           -> base_footprint->{base_laser, ld20_laser, camera_link, imu_link}.
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
    color_w = LaunchConfiguration('color_width')
    color_h = LaunchConfiguration('color_height')

    return LaunchDescription([

        DeclareLaunchArgument('map', default_value=default_map,
            description='Static map yaml for map_server (default apartment_s2e_v2.yaml).'),
        # Color resolution is overridable so the full stack can run the camera at 1280x720 for the
        # AprilTag docking detector while nvblox still uses depth@640. Default 640x480 keeps standalone
        # nav unchanged.
        DeclareLaunchArgument('color_width',  default_value='640',
            description='Orbbec color width  (set 1280 in the full stack for AprilTag docking).'),
        DeclareLaunchArgument('color_height', default_value='480',
            description='Orbbec color height (set 720 to pair with color_width 1280).'),

        # ── Sensors / odometry ────────────────────────────────────────────────
        # micro-ROS agent — ESP32: /odom/unfiltered + receives /cmd_vel for the motors
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '--dev', '/dev/jupiter_esp32', '-b', '460800'],
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

        # LD20 LOW lidar (LD19 profile) — /scan_low in frame ld20_laser, ~0.13 m above ground.
        # Second obstacle source for the costmap: catches LOW furniture (chair legs, feet) under the
        # S2E's 0.515 m plane. Orientation verified in RViz (yaw 0). The costmap masks its own near
        # structure via 0.30 m min ranges, so no angular crop is needed here. Replaces nvblox.
        Node(
            package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node', name='LD19_low',
            output='screen',
            parameters=[
                {'product_name':           'LDLiDAR_LD19'},
                {'topic_name':             'scan_low'},
                {'frame_id':               'ld20_laser'},
                {'port_name':              '/dev/jupiter_lidar'},
                {'port_baudrate':          230400},
                {'laser_scan_dir':         True},
                {'enable_angle_crop_func': False},
            ],
        ),
        # Static TF base_footprint -> ld20_laser: 6 cm forward, centred, 13 cm up, yaw 0.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_ld20_laser',
            arguments=['0.06', '0', '0.13', '0', '0', '0', 'base_footprint', 'ld20_laser'],
        ),
        # Static TF base_footprint -> imu_link (BNO055)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'imu_link'],
        ),

        # ── Orbbec 336 camera (COLOR ONLY — nvblox retired 2026-06-20) ─────────
        # Depth/point-cloud/laser OFF: the LD20 low lidar now owns low-obstacle sensing, so the camera
        # is freed for face-rec + AprilTag docking (color stream, consumed by the full-stack vision
        # nodes). Dropping depth also reclaims USB bandwidth on the shared xHCI controller + GPU.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('orbbec_camera'),
                'launch', 'gemini_330_series.launch.py')),
            launch_arguments={
                'serial_number': 'CP9KB53000HP', 'usb_port': '',
                'enable_color': 'true', 'color_width': color_w, 'color_height': color_h,
                'color_fps': '15', 'color_format': 'MJPG',
                'enable_depth': 'false', 'enable_point_cloud': 'false', 'enable_laser': 'false',
                'enable_left_ir': 'false', 'enable_right_ir': 'false',
                'enable_gyro': 'true', 'enable_accel': 'true',  # IMU for EKF imu1 high-rate yaw
            }.items(),
        ),
        # Static TF base_footprint -> camera_link (measured 2026-06-10: 0.100/0/0.475).
        # Pitch -0.0995 rad = 5.7deg nose-UP (negative = up per REP-103): camera was physically tilted
        # up so face-rec can see a standing user. This supersedes the old deferred ~1.3deg nose-down.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_camera_link',
            arguments=['--x', '0.100', '--y', '0.000', '--z', '0.475',
                       '--roll', '0', '--pitch', '-0.0995', '--yaw', '0',
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
        # nvblox RETIRED 2026-06-20: low-obstacle sensing moved to the LD20 (scan_low, above) because the
        # camera is now tilted 5.7deg nose-UP for face-rec, which makes its floor-grazing depth unreliable.
        # Re-enable by restoring this TimerAction + nvblox_layer in nav2_params + depth in the camera block.
    ])
