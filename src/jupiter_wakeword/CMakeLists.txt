# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(jupiter_wakeword LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# sherpa-onnx — pre-built aarch64 shared library in workspace root
set(SHERPA_ONNX_DIR "$ENV{HOME}/jupitercpp_ws/sherpa-onnx")

add_executable(jupiter_wakeword src/wake_word_node.cpp)

ament_target_dependencies(jupiter_wakeword rclcpp std_msgs)

target_include_directories(jupiter_wakeword PRIVATE
  "${SHERPA_ONNX_DIR}/include"
)

target_link_libraries(jupiter_wakeword
  "${SHERPA_ONNX_DIR}/lib/libsherpa-onnx-c-api.so"
)

# Embed RPATH so the binary finds libsherpa-onnx-c-api.so at runtime
set_target_properties(jupiter_wakeword PROPERTIES
  BUILD_RPATH   "${SHERPA_ONNX_DIR}/lib"
  INSTALL_RPATH "${SHERPA_ONNX_DIR}/lib"
)

install(TARGETS jupiter_wakeword DESTINATION lib/${PROJECT_NAME})

ament_package()
