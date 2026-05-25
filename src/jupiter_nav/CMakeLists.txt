# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(jupiter_nav)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# ── Planner node ──────────────────────────────────────────────────────────────
add_executable(jupiter_planner
  src/occupancy_map.cpp
  src/jupiter_planner_node.cpp
)
ament_target_dependencies(jupiter_planner
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(jupiter_planner ${OpenCV_LIBS})

install(TARGETS jupiter_planner
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch   DESTINATION share/${PROJECT_NAME})

ament_package()
