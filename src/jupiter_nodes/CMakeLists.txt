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
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

# whisper.cpp paths (standalone build in workspace root)
set(WHISPER_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../whisper.cpp/include")
set(WHISPER_GGML_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../whisper.cpp/ggml/include")
set(WHISPER_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../../whisper.cpp/build/src/libwhisper.so")

# Add all includes explicitly to ensure the compiler sees cv_bridge and VPI
include_directories(
  include
  ${VPI_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(jupiter_vision src/vision.cpp)
add_executable(imu_covariance_fixer src/imu_covariance_fixer.cpp)
add_executable(jupiter_voice src/voice.cpp)
add_executable(jupiter_brain src/brain.cpp)
add_executable(jupiter_face_recognition src/face_recognition.cpp)

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

ament_target_dependencies(jupiter_voice
  rclcpp
  std_msgs
)

ament_target_dependencies(jupiter_brain
  rclcpp
  std_msgs
)

ament_target_dependencies(jupiter_face_recognition
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

# Link Hardware libraries
target_link_libraries(jupiter_vision
  vpi
  cuda
  ${OpenCV_LIBS}
)

target_include_directories(jupiter_voice PRIVATE
  ${WHISPER_INCLUDE_DIR}
  ${WHISPER_GGML_INCLUDE_DIR}
)

target_link_libraries(jupiter_voice
  ${WHISPER_LIB}
  asound
)

# Embed RPATH so jupiter_voice finds libwhisper.so at runtime without LD_LIBRARY_PATH
set_target_properties(jupiter_voice PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../whisper.cpp/build/src"
  INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../whisper.cpp/build/src"
)

target_link_libraries(jupiter_brain
  CURL::libcurl
  nlohmann_json::nlohmann_json
)

target_include_directories(jupiter_face_recognition PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  /usr/include/aarch64-linux-gnu                              # NvInfer.h, NvOnnxParser.h
  /usr/local/cuda-13.0/targets/sbsa-linux/include            # cuda_runtime_api.h
)

target_link_libraries(jupiter_face_recognition
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
  nvinfer
  nvonnxparser
  /usr/local/cuda-13.0/targets/sbsa-linux/lib/libcudart.so
)

install(TARGETS
  jupiter_vision
  imu_covariance_fixer
  jupiter_voice
  jupiter_brain
  jupiter_face_recognition
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()