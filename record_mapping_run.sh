#!/usr/bin/env bash
# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# One-command RECORD step of the rosbag mapping workflow (large maps, e.g. the
# 220 m² apartment). Brings up the cheap live pipeline (micro-ROS odom + LD20
# LiDAR + EKF) and records a bag — NO slam_toolbox, so nothing clogs while you
# drive. Build the actual map afterward, offline:
#
#   ros2 launch jupiter_bringup offline_mapping.launch.py bag_path:=<the bag printed below>
#
# Usage:
#   ./record_mapping_run.sh            # bag name "apt_run_<timestamp>"
#   ./record_mapping_run.sh lounge     # bag name "lounge_<timestamp>"
#
# Drive in a SECOND terminal:
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Cold-start rule still applies: drive the route promptly, then Ctrl-C here to
# finalize the bag. (Recording doesn't degrade like live SLAM — but a focused
# drive still makes the best map.)

set -euo pipefail

WS="$HOME/jupitercpp_ws"
NAME="${1:-apt_run}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="$WS/maps/bags/${NAME}_${STAMP}"

# Source ROS + the workspace overlay. Disable nounset around sourcing — the ROS
# setup scripts reference internal vars (AMENT_TRACE_SETUP_FILES, etc.) that are
# unset on a clean shell and would trip `set -u`.
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

# Parent dir must exist; the bag dir itself must NOT (ros2 bag creates it).
mkdir -p "$WS/maps/bags"

echo "──────────────────────────────────────────────────────────────"
echo " Recording mapping bag to:"
echo "   $BAG"
echo ""
echo " Drive in another terminal:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo " When done, Ctrl-C here, then build the map offline:"
echo "   ros2 launch jupiter_bringup offline_mapping.launch.py bag_path:=$BAG"
echo "──────────────────────────────────────────────────────────────"

exec ros2 launch jupiter_bringup record_mapping.launch.py "bag_path:=$BAG"
