# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# SLAM mapping — builds a cuVSLAM visual keyframe map for nav-mode re-localisation.
#
# cuVSLAM runs in full localisation+mapping mode, publishing map→odom→base_footprint TF.
# Drive the robot around the environment to accumulate visual keyframes, then save:
#
#   ros2 service call /visual_slam/save_landmark_map \
#     isaac_ros_visual_slam_interfaces/srv/SaveMap \
#     "{map_url: '/home/jupiter/jupitercpp_ws/maps/visual_map'}"
#
# In nav mode cuVSLAM loads that map and re-localises at startup; nvblox builds the
# obstacle costmap dynamically from depth images — no occupancy grid needed.
#
# Usage (voice MUST be disabled — Whisper GPU DMA stalls affect cuVSLAM timing):
#   ros2 launch jupiter_bringup jupiter_bringup.launch.py \
#     enable_microros:=true enable_nav:=true mode:=slam enable_voice:=false

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('jupiter_bringup')

    # Static TF: base_footprint → camera_link (needed by cuVSLAM)
    camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link',
        arguments=[
            '--x',     '0.10',
            '--y',     '0.0',
            '--z',     '0.49',
            '--roll',  '0',
            '--pitch', '0',
            '--yaw',   '0',
            '--frame-id',       'base_footprint',
            '--child-frame-id', 'camera_link',
        ],
    )

    # cuVSLAM in full localisation+mapping mode.
    # Publishes map→odom TF and odom→base_footprint TF.
    # Accumulates visual keyframes into a map that can be saved and re-loaded in nav mode.
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'tracking_mode':                        1,
            'depth_scale_factor':                   1000.0,
            'enable_image_denoising':               False,
            'rectified_images':                     False,
            'image_jitter_threshold_ms':            500.0,
            'sync_matching_threshold_ms':           200.0,
            'base_frame':                           'base_footprint',
            'map_frame':                            'map',
            'odom_frame':                           'odom',
            'imu_frame':                            'camera_accel_gyro_optical_frame',
            'enable_localization_n_mapping':        True,
            'num_cameras':                          1,
            'min_num_images':                       1,
            'camera_optical_frames':                ['camera_color_optical_frame'],
            'enable_slam_visualization':            True,
            'enable_landmarks_view':                True,
            'enable_observations_view':             True,
            'gyro_noise_density':                   0.000244,
            'gyro_random_walk':                     0.000019393,
            'accel_noise_density':                  0.001862,
            'accel_random_walk':                    0.003,
            'calibration_frequency':                200.0,
            'enable_ground_constraint_in_odometry': True,
            'enable_ground_constraint_in_slam':     True,
        }],
        remappings=[
            ('visual_slam/image_0',       '/camera/color/image_raw'),
            ('visual_slam/camera_info_0', '/camera/color/camera_info'),
            ('visual_slam/depth_0',       '/camera/depth/image_raw'),
            ('visual_slam/imu',           '/camera/gyro_accel/sample'),
        ],
    )

    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return LaunchDescription([camera_link_tf, visual_slam_container])
