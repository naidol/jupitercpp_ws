# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(jupiter_nodes LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/nvidia/vpi4")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vpi REQUIRED)
find_package(OpenCV REQUIRED)

# Add all includes explicitly to ensure the compiler sees cv_bridge and VPI
include_directories(
  include
  ${VPI_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(jupiter_vision src/vision.cpp)
add_executable(imu_covariance_fixer src/imu_covariance_fixer.cpp)

# Link ROS2 dependencies
ament_target_dependencies(jupiter_vision
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
)

ament_target_dependencies(imu_covariance_fixer
  rclcpp
  sensor_msgs
)

# Link Hardware libraries
target_link_libraries(jupiter_vision
  vpi
  cuda
  ${OpenCV_LIBS}
)

install(TARGETS
  jupiter_vision
  imu_covariance_fixer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()