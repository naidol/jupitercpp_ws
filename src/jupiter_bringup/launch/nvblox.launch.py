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

            # Static TSDF mapping — no dynamic object segmentation
            'mapping_type':              'static_tsdf',

            # 2D ESDF slice for Nav2 costmap
            'esdf_mode':                 '2d',
            'esdf_slice_height':         0.5,    # slice at 50 cm — mid-body obstacle height
            'esdf_slice_min_height':     0.05,   # ignore floor returns
            'esdf_slice_max_height':     1.5,    # ignore ceiling

            # Processing rates — tuned for 15 fps depth stream
            'integrate_depth_rate_hz':   10.0,
            'integrate_color_rate_hz':   5.0,
            'update_esdf_rate_hz':       5.0,
            'update_mesh_rate_hz':       2.0,
            'publish_layer_rate_hz':     5.0,

            # Map memory management — clear beyond 5 m from robot
            'map_clearing_radius_m':     5.0,
            'map_clearing_frame_id':     'base_footprint',

            # Publish ESDF distance slice for Nav2 costmap layer
            'publish_esdf_distance_slice': True,
        }],
        remappings=[
            ('depth/image',       '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
            ('color/image',       '/camera/color/image_raw'),
            ('color/camera_info', '/camera/color/camera_info'),
        ],
    )

    return launch.LaunchDescription([nvblox_node])
