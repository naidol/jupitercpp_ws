# Jupiter Robot Project — CLAUDE.md

## Main Objectives
- Build a 4-wheeled autonomous robot that includes vision, voice & brain functions
- The robot runs on mecanum wheels and is to navigate using the ROS2 Navigation stack
- The vision system must be able to perform face recognition and use April tags for docking and also recognise surrounding environment
- The voice system should perform ASR direct voice commands to the Brain and respond via TTS
- The Brain should interpret and respond to voice commands, using VLM or LLM
- The Brain should identify know users and keep a record of past conversations with registered users.  Unregistered users should be recorded as a guest
- If the robot uncounters unknown users, it should make a polite introduction and offer the 'guest' to register as a user
- The robot is powered by a 16.8V (4S5P) Li-Ion battery pack and should be able to navigate to its docking station to recharge when needed.
- The robot's name is Jupiter

## Hardware Platform
- **Compute:** Jetson AGX Thor — JetPack 7.1, Ubuntu 24.04
- **GPU:** Blackwell architecture — CUDA 13.0+, TensorRT 10.x
- **Microcontroller:** ESP32 via micro-ROS (C/C++ only)
- **Camera:** Orbbec 336 — OrbbecSDK C++ native
- **Microphone:** ReSpeaker 3800 — ALSA interface
- **LiDAR:** LD20 — UART serial protocol
- **Development:** Direct on Jetson desktop, no cross-compilation

## Software Stack
- **OS:** Ubuntu 24.04 Jetpack 7.1 bare metal — NO Docker
- **ROS 2:** Jazzy
- **Language:** Pure C++ throughout — NO Python
- **Build:** Colcon + CMake, Release build type
- **Audio:** whisper.cpp with CUDA — GGUF models
- **Vision AI:** TensorRT C++ API — ONNX models
- **LLM:** llama.cpp with CUDA — GGUF models

## Critical Constraints
- NO Python in any ROS 2 nodes — C++ only
- NO Docker containers — bare metal only
- NO sudo pip install — never touch system Python
- All CUDA code must target Blackwell architecture
- TensorRT engines compiled on this device only
- CMakeLists must use ament_cmake, not plain cmake

## Code Style Preferences
- Modern C++17 throughout
- RAII for all resource management
- Smart pointers — no raw new/delete
- Explicit error handling on all SDK calls
- Meaningful variable names — no single letter variables
- Comments on all CUDA kernel launches explaining dims

## ROS 2 Conventions
- Node names: snake_case
- Topic names: /robot/subsystem/data
- All nodes must handle SIGINT cleanly
- Use rclcpp::shutdown() in signal handlers
- Parameter-driven configuration — no hardcoded values

## Current Package Structure
- jupiter_bringup/ — launch files, config (EKF, SLAM, Nav2)
- jupiter_nodes/ — C++ nodes: imu_covariance_fixer, vision (VPI/AprilTags stub), diagnostics
- ldlidar_stl_ros2/ — LD20 LiDAR driver (LD19 compatible, 6Hz, 8m indoor range)

## Standalone AI Tools (workspace root, not yet ROS2 nodes)
- whisper.cpp — CUDA ASR (speech-to-text)
- piper_tts/ — TTS (text-to-speech)
- llama.cpp — CUDA LLM brain
- jupiter_talk.sh — voice pipeline orchestration script
- Python reference implementations in ~/jupiter_ws (original project pre-C++ rebuild)

## Completed Subsystems
- Localization: EKF (robot_localization) + BNO055 IMU via micro-ROS, yaw covariance ~0.016 rad²
- SLAM: slam_toolbox async mapping, lifecycle_manager auto-activates on launch
- Nav2: MPPI controller (Omni motion model for mecanum), full Nav2 stack configured
- Master launch: ros2 launch jupiter_bringup jupiter_bringup.launch.py (mode:=slam or mode:=nav)
- ESP32 auto-reconnect: 4-state micro-ROS state machine (WAITING→AVAILABLE→CONNECTED→DISCONNECTED), no physical reset needed
- ESP32 battery monitoring: publishes sensor_msgs/BatteryState on /battery/state at 1Hz (R1=100kΩ/R2=22kΩ divider on GPIO34 fitted and working — reads true pack voltage)

## Known Issues / Workarounds
- systemd services jupiter-microros and jupiter-lidar are DISABLED during development

## Goals for This Session
- [Update this each session with what you want to achieve]