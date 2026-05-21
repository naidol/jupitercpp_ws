#!/bin/bash
# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# LiDAR watchdog — auto-restarts ldlidar_stl_ros2_node when /scan times out.
# Whisper GPU inference causes tegra-xusb DMA stalls that overflow the LD19
# cp210x serial FIFO, breaking SDK frame sync (the SDK has no self-recovery).

set -u

LOG_TAG="[lidar_watchdog]"
SCAN_TIMEOUT=5   # seconds without /scan before triggering restart
STARTUP_GRACE=8  # seconds to wait for first scan after node start
RESTART_DELAY=2  # seconds between kill and restart

log() { echo "$LOG_TAG $(date '+%H:%M:%S') $*"; }

start_ldlidar() {
    ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node \
        --ros-args \
        -p product_name:=LDLiDAR_LD19 \
        -p topic_name:=scan \
        -p frame_id:=base_laser \
        -p port_name:=/dev/jupiter_lidar \
        -p port_baudrate:=230400 \
        -p laser_scan_dir:=true \
        -p enable_angle_crop_func:=false \
        -p angle_crop_min:=135.0 \
        -p angle_crop_max:=225.0 &
    echo $!
}

# Reduce serial interrupt coalescing — helps absorb brief USB DMA stalls
setserial /dev/jupiter_lidar low_latency 2>/dev/null || true

LIDAR_PID=""
cleanup() {
    log "Shutting down"
    [ -n "$LIDAR_PID" ] && kill "$LIDAR_PID" 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

while true; do
    LIDAR_PID=$(start_ldlidar)
    log "Started ldlidar PID $LIDAR_PID"

    log "Startup grace ${STARTUP_GRACE}s..."
    sleep "$STARTUP_GRACE"

    while kill -0 "$LIDAR_PID" 2>/dev/null; do
        if timeout "$SCAN_TIMEOUT" ros2 topic echo --once /scan > /dev/null 2>&1; then
            sleep 3  # healthy — pause before next check to avoid spinning DDS participants
        else
            log "/scan silent for >${SCAN_TIMEOUT}s — restarting ldlidar"
            kill "$LIDAR_PID" 2>/dev/null
            wait "$LIDAR_PID" 2>/dev/null
            break
        fi
    done

    log "Restart in ${RESTART_DELAY}s..."
    sleep "$RESTART_DELAY"
done
