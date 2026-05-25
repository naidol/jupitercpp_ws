# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(jupiter_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/lidar_watchdog.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
