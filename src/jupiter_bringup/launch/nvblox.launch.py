# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# nvblox — GPU 3D reconstruction → 2D ESDF costmap for Nav2
#
# Takes Orbbec depth + colour images and the cuVSLAM pose (via TF) and builds a
# TSDF voxel map on the GPU. Slices it at robot-body height to produce a 2D ESDF
# DistanceMapSlice that the NvbloxCostmapLayer in Nav2 reads directly.
#
# Replaces the LD19 LiDAR entirely — no /scan topic needed.

import launch
from launch_ros.actions import Node


def generate_launch_description():

    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[{
            # Coordinate frames — must match cuVSLAM output
            'global_frame':              'map',
            'pose_frame':                'base_footprint',

            # Voxel map resolution — 5 cm matches Nav2 costmap resolution
            'voxel_size':                0.05,

            # Single RGB-D camera
            'num_cameras':               1,
            'use_depth':                 True,
            'use_color':                 True,
            'use_lidar':                 False,

            # Workspace Bounds — Explicitly set to Unbounded (0) to prevent the
            # Y-axis 2.0m restriction seen in the logs.
            'workspace_bounds_type':     0,
            'workspace_bounds_min_height_m': 0.0,
            'workspace_bounds_max_height_m': 2.0,

            # Static TSDF mapping — no dynamic object segmentation
            'mapping_type':              'static_tsdf',

            # 2D ESDF slice for Nav2 costmap
            'esdf_mode':                 '2d',
            'esdf_slice_height':         0.5,    # slice at 50 cm — mid-body obstacle height
            'esdf_2d_min_height':        0.4,    # ignore floor/baseboard returns below knee height
            'esdf_2d_max_height':        1.8,    # ignore ceiling (correct 4.4 param name)

            # Processing rates — tuned for 6 fps depth stream (minimum valid Orbbec 336 profile)
            'integrate_depth_rate_hz':   6.0,
            'integrate_color_rate_hz':   5.0,
            'update_esdf_rate_hz':       5.0,
            'update_mesh_rate_hz':       2.0,
            'publish_layer_rate_hz':     5.0,

            # Map memory management — clear beyond 5 m from robot
            'map_clearing_radius_m':     5.0,
            'map_clearing_frame_id':     'base_footprint',

            # Publish ESDF distance slice for Nav2 costmap layer
            'publish_esdf_distance_slice': True,

            # Drop stale images — process only the latest frame, never a backlog.
            # Without this, nvblox queues depth frames during cuVSLAM tracking gaps
            # and processes them 5+ seconds late at wrong robot positions, creating
            # phantom obstacles from the robot's own spinning motion.
            'maximum_sensor_message_queue_length': 1,
        }],
        remappings=[
            ('camera_0/depth/image',       '/camera/depth/image_raw'),
            ('camera_0/depth/camera_info', '/camera/depth/camera_info'),
            ('camera_0/color/image',       '/camera/color/image_raw'),
            ('camera_0/color/camera_info', '/camera/color/camera_info'),
        ],
    )

    return launch.LaunchDescription([nvblox_node])
