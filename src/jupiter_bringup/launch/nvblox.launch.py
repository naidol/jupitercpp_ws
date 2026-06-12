# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# nvblox — GPU 3D reconstruction (Orbbec depth) -> 2D ESDF costmap slice for Nav2.
#
# Builds a TSDF voxel map on the Thor GPU from the Orbbec depth image, slices it at
# body height, and publishes a DistanceMapSlice that Nav2's NvbloxCostmapLayer reads.
# This is the 3D obstacle source that catches what the S2E lidar (at 0.515 m) is blind
# to — chairs, table legs, low furniture below the scan plane.
#
# POSE SOURCE: nvblox only needs the robot pose via TF (global_frame -> pose_frame).
# It does NOT need cuVSLAM — AMCL (map->odom) + EKF (odom->base_footprint) provide it.
# (The old cuVSLAM-era nvblox_slice_relay delay node is NOT needed here — it only
#  existed to paper over cuVSLAM's laggy TF; AMCL+EKF TF timing is fine.)
#
# Camera bring-up (depth + camera_info) is separate — see the nav launch / camera.launch.py.

import launch
from launch_ros.actions import Node


def generate_launch_description():

    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[{
            # Coordinate frames — pose comes from AMCL+EKF via TF
            'global_frame':              'map',
            'pose_frame':                'base_footprint',

            # Voxel resolution — 5 cm matches the Nav2 costmap resolution
            'voxel_size':                0.05,

            # Single RGB-D camera (Orbbec Gemini 336)
            'num_cameras':               1,
            'use_depth':                 True,
            'use_color':                 True,
            'use_lidar':                 False,

            # Workspace bounds unbounded in plane; clamp height to the room
            'workspace_bounds_type':         0,
            'workspace_bounds_min_height_m': 0.0,
            'workspace_bounds_max_height_m': 2.0,

            'mapping_type':              'static_tsdf',

            # 2D ESDF slice for the Nav2 costmap.
            # *** 2026-06-12 BUGFIX: the height-band params were named esdf_2d_min_height /
            # esdf_2d_max_height -- those names DO NOT EXIST in nvblox, so they were SILENTLY
            # IGNORED and nvblox used the defaults esdf_slice_min_height=0.0 / max=1.0. That
            # sliced FROM THE FLOOR (0.0 m) upward, marking the floor itself (worsened by the
            # ~1.3deg camera pitch) as an obstacle carpet -> robot boxed in with "collision
            # ahead" the instant it tried to move in any cluttered room. Correct names verified
            # in /opt/ros/jazzy/include/nvblox/integrators/esdf_integrator_params.h. ***
            # min 0.10 m clears the floor; raise toward 0.15 if the floor still bleeds in,
            # lower toward 0.05 if low obstacles get missed. max 1.80 catches tall furniture.
            'esdf_mode':                 '2d',
            'esdf_slice_height':         0.30,   # output slice Z (metadata; correct name, was already applied)
            'esdf_slice_min_height':     0.10,   # was esdf_2d_min_height (WRONG NAME -> ignored -> floor marked)
            'esdf_slice_max_height':     1.80,   # was esdf_2d_max_height (WRONG NAME -> ignored)

            # Rates — tuned for the ~5–15 fps Orbbec depth stream
            'integrate_depth_rate_hz':   10.0,
            'integrate_color_rate_hz':   5.0,
            'update_esdf_rate_hz':       5.0,
            'update_mesh_rate_hz':       2.0,
            'publish_layer_rate_hz':     5.0,

            # Memory — clear the map beyond this radius of the robot. 2026-06-10: 5.0 -> 2.5.
            # static_tsdf accumulates+keeps everything within this radius and never forgets it,
            # so a large radius bakes in phantom obstacles (bootstrap-pose integration + drift)
            # that boxed the robot in. 2.5 m covers the 4x4 m local costmap and constantly
            # refreshes near the robot, wiping stale phantoms as it moves.
            'map_clearing_radius_m':     2.5,
            'map_clearing_frame_id':     'base_footprint',

            # Publish the ESDF distance slice consumed by NvbloxCostmapLayer
            'publish_esdf_distance_slice': True,

            # Only ever process the latest depth frame — never a stale backlog
            # (stale frames get integrated at the wrong robot pose = phantom obstacles).
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
