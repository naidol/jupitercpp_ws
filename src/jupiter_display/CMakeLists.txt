# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(jupiter_display LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Quick Qml)

add_executable(jupiter_display src/display_node.cpp)

ament_target_dependencies(jupiter_display
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(jupiter_display
  Qt6::Core
  Qt6::Gui
  Qt6::Quick
  Qt6::Qml
)

install(TARGETS jupiter_display DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY qml DESTINATION share/${PROJECT_NAME})

ament_package()
