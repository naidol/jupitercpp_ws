# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(jupiter_audio_capture LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ALSA REQUIRED)

add_executable(mic_capture_node src/mic_capture_node.cpp)
ament_target_dependencies(mic_capture_node rclcpp std_msgs)
target_include_directories(mic_capture_node PRIVATE ${ALSA_INCLUDE_DIRS})
target_link_libraries(mic_capture_node ${ALSA_LIBRARIES})

install(TARGETS mic_capture_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
