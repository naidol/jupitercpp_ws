# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Docking NODES only — NO micro_ros_agent (run the agent PERSISTENTLY & separately).
# Restarting the agent churns the ESP32 micro-ROS client into a reboot/reconnect loop,
# and starting the whole stack at once can disrupt the ESP32 handshake. So: keep ONE
# long-lived agent, and start/stop just these nodes as you iterate on docking.
#
#   # once (persistent, in its own terminal):
#   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/jupiter_esp32 -b 460800
#   # then (re-runnable):
#   ros2 launch jupiter_bringup dock_ir_nodes.launch.py reverse:=true dock_depth:=0.307 contact_dist:=0.03 steer_sign:=-1.0
#   ros2 service call /dock_engage std_srvs/srv/SetBool "{data: true}"
#
# Same nodes as dock_ir.launch.py minus the agent. See that file for the architecture.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')
    ekf_config  = os.path.join(bringup_dir, 'config', 'ekf_odom.yaml')

    reverse           = LaunchConfiguration('reverse')
    steer_sign        = LaunchConfiguration('steer_sign')
    approach_speed    = LaunchConfiguration('approach_speed')
    near_speed        = LaunchConfiguration('near_speed')
    Ky_balance        = LaunchConfiguration('Ky_balance')
    dock_depth        = LaunchConfiguration('dock_depth')
    contact_dist      = LaunchConfiguration('contact_dist')
    apriltag_steer_sign = LaunchConfiguration('apriltag_steer_sign')
    k_bearing         = LaunchConfiguration('k_bearing')
    k_bearing_d       = LaunchConfiguration('k_bearing_d')
    handoff_angle_max = LaunchConfiguration('handoff_angle_max')
    retry_dist        = LaunchConfiguration('retry_dist')
    bearing_deadband  = LaunchConfiguration('bearing_deadband')
    handoff_dist      = LaunchConfiguration('handoff_dist')
    cam_device        = LaunchConfiguration('cam_device')

    return LaunchDescription([

        # Defaults below = the WORKING recipe from the first clean AprilTag-guided dock (2026-07-08).
        DeclareLaunchArgument('reverse', default_value='true',
                              description='true = back into the dock caster-first'),
        DeclareLaunchArgument('steer_sign', default_value='1.0',
                              description='IR-near steer sign (unused while Ky_balance=0)'),
        DeclareLaunchArgument('approach_speed', default_value='0.10',
                              description='m/s TRUE (100mm wheels, honest radius) — same motor regime as the old golden "0.06" (=0.092 true); reverse breakaway needs ~3-4s PID windup'),
        DeclareLaunchArgument('near_speed', default_value='0.08',
                              description='m/s TRUE — rail-thread + seat; already rolling at handoff so no breakaway needed (old "0.05" = 0.077 true)'),
        DeclareLaunchArgument('Ky_balance', default_value='0.0',
                              description='0 = NEAR drives STRAIGHT, rails square it (IR balance saturates+crabs at the mouth)'),
        DeclareLaunchArgument('dock_depth', default_value='0.212',
                              description='m — FRAME-plane -> seat (re-measured 2026-07-12 vs the new tag-frame stand: frame=0.242 at physical seat, minus 0.03 contact_dist; re-zero when pogo bracket mounts)'),
        DeclareLaunchArgument('contact_dist', default_value='0.03',
                              description='m — stop when dist_to_pogo <= this'),
        DeclareLaunchArgument('apriltag_steer_sign', default_value='-1.0',
                              description='FAR steer sign — PINNED -1.0 (+1.0 diverges/veers left)'),
        DeclareLaunchArgument('k_bearing', default_value='0.3',
                              description='rad/s per rad of tag bearing (0.6 rings/overshoots — keep corrections lazy)'),
        DeclareLaunchArgument('k_bearing_d', default_value='0.15',
                              description='rad/s per rad/s bearing rate (D damping — kills the offset-recovery wiggle)'),
        DeclareLaunchArgument('handoff_angle_max', default_value='0.21',
                              description='rad (~12deg) — decision-point heading gate; rails funnel is 28deg/side so this keeps 2x margin'),
        DeclareLaunchArgument('retry_dist', default_value='0.70',
                              description='m — go-around retreat depth (couch blocks at z~0.82: 0.85 was UNREACHABLE)'),
        DeclareLaunchArgument('bearing_deadband', default_value='0.04',
                              description='rad — FAR steer deadband (wide: glide straight, do not chase mm)'),
        DeclareLaunchArgument('handoff_dist', default_value='0.35',
                              description='m — tag z at/below this -> hand off FAR->NEAR'),
        DeclareLaunchArgument('cam_device', default_value='8',
                              description='/dev/videoN of the rear docking webcam'),

        # Odometry: IMU fix + EKF -> /odometry/filtered (stall detection)
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_link_to_imu_link',
             arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'imu_link']),
        Node(package='jupiter_nodes', executable='imu_covariance_fixer', name='imu_covariance_fixer',
             output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_node',
             output='screen', parameters=[ekf_config]),

        # RPLIDAR S2E — /scan in frame base_laser (the dock ranging sensor)
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
             output='screen',
             parameters=[{
                 'channel_type': 'udp', 'udp_ip': '192.168.11.2', 'udp_port': 8089,
                 'frame_id': 'base_laser', 'inverted': False,
                 'angle_compensate': True, 'scan_mode': 'Sensitivity',
             }]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_link_to_base_laser',
             arguments=['0.035', '0.0', '0.5325', '3.14159265', '0', '0', 'base_footprint', 'base_laser']),  # +17.5mm: 100mm AGV wheels

        # LiDAR wall ranging: /scan rear sector -> /dock/range
        Node(package='jupiter_nodes', executable='dock_range', name='dock_range',
             output='screen',
             parameters=[{
                 'scan_topic':       '/scan',
                 'rear_bearing_deg': 0.0,
                 'sector_half_deg':  5.5,   # narrowed 2026-07-12: range against the dock's 280mm-wide TAG-FRAME plane (dock-fixed, 105mm in front of wall) — ±10° clipped wall at the edges -> INVALID
                 'dock_depth':       ParameterValue(dock_depth, value_type=float),
             }]),

        # Rear docking webcam -> /webcam/image_raw (the AprilTag far-align camera)
        Node(package='jupiter_nodes', executable='webcam_publisher', name='webcam_publisher',
             output='screen',
             parameters=[{
                 'device': ParameterValue(cam_device, value_type=int),
                 'width':  1280,
                 'height': 720,
                 'fps':    30.0,
             }]),

        # AprilTag detector on the webcam -> /vision/marker_pose (camera frame: x lateral, z dist)
        Node(package='jupiter_nodes', executable='jupiter_vision', name='jupiter_vision',
             output='screen',
             parameters=[{
                 'camera_topic': '/webcam/image_raw',
                 'image_width':  1280,
                 'image_height': 720,
                 'marker_size_m': 0.080,   # 80mm tag (user-confirmed 2026-07-09) centred at camera height (0.42m) — in frame from ~1.4m down to DOCKED (~0.15m)
                 'cam_fx': 868.0, 'cam_fy': 868.0,
                 'cam_cx': 640.0, 'cam_cy': 360.0,
             }]),

        # Dock controller: AprilTag bearing -> FAR lateral, IR balance -> NEAR lateral,
        #                  LiDAR range -> distance/stop, rails -> square
        Node(package='jupiter_nodes', executable='dock_ir', name='dock_ir',
             output='screen',
             parameters=[{
                 'approach_speed':      ParameterValue(approach_speed, value_type=float),
                 'near_speed':          ParameterValue(near_speed, value_type=float),
                 'reverse':             ParameterValue(reverse, value_type=bool),
                 'steer_sign':          ParameterValue(steer_sign, value_type=float),
                 'Ky_balance':          ParameterValue(Ky_balance, value_type=float),
                 'contact_dist':        ParameterValue(contact_dist, value_type=float),
                 'apriltag_steer_sign': ParameterValue(apriltag_steer_sign, value_type=float),
                 'k_bearing':           ParameterValue(k_bearing, value_type=float),
                 'k_bearing_d':         ParameterValue(k_bearing_d, value_type=float),
                 'handoff_angle_max':   ParameterValue(handoff_angle_max, value_type=float),
                 'retry_dist':          ParameterValue(retry_dist, value_type=float),
                 'bearing_deadband':    ParameterValue(bearing_deadband, value_type=float),
                 'handoff_dist':        ParameterValue(handoff_dist, value_type=float),
                 'control_rate':        20.0,
             }]),
    ])
