# Jupiter Robot Project — CLAUDE.md

## Main Objectives
- Build a 4-wheeled autonomous robot that includes vision, voice & brain functions
- The robot is 2-wheel DIFFERENTIAL drive with ONE rear caster 180mm behind the drive axle, and
  navigates using the ROS2 Navigation stack. (It is NOT mecanum/omni — that was an early plan.
  The single caster matters: it must never be commanded to pivot in place, see Docking below.)
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
- **LiDAR:** Slamtec S2E — Ethernet/UDP 192.168.11.2:8089, scan plane 0.518m, mounted yaw π
  (the LD20 that used to sit at 0.13m was physically REMOVED 2026-08-03)
- **Front ToF:** VL53L0X on the ESP32 I2C bus — low near-field obstacles under the S2E plane
- **Dock sensing:** 2× LJ18A3-8-Z/BX inductive prox (12V NPN) + TSAL6400 IR charge-enable beacon
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
- jupiter_bringup/ — launch files, config (EKF, SLAM, Nav2, AMCL)
- jupiter_nodes/ — C++ nodes: dock_aligner_v3 (current docker), dock_approach, dock_reflector,
  dock_range, dock_ir, imu_covariance_fixer, jupiter_brain, jupiter_voice,
  jupiter_face_recognition, jupiter_vision, scan_deskew_node
  (dock_aligner + dock_aligner_v2 are SUPERSEDED, kept in-tree as record only)
- sllidar_ros2/ — Slamtec S2E driver (the live lidar)
- ldlidar_stl_ros2/ — LD20 driver, hardware removed 2026-08-03

## Standalone AI Tools (workspace root, not yet ROS2 nodes)
- whisper.cpp — CUDA ASR (speech-to-text)
- piper_tts/ — TTS (text-to-speech)
- llama.cpp — CUDA LLM brain
- jupiter_talk.sh — voice pipeline orchestration script
- Python reference implementations in ~/jupiter_ws (original project pre-C++ rebuild)

## Completed Subsystems
- Localization: AMCL (DifferentialMotionModel) + EKF (robot_localization) + BNO055 IMU via
  micro-ROS, yaw covariance ~0.016 rad². cuVSLAM and nvblox are RETIRED — do not reintroduce.
- SLAM: slam_toolbox async mapping, lifecycle_manager auto-activates on launch
- Nav2: RegulatedPurePursuit controller + SmacPlanner2D + SimpleSmoother.
  Local costmap = obstacle_layer (S2E /scan) + range_layer (ToF /tof/front) + inflation.
  Global costmap = static + obstacle + inflation (ToF deliberately excluded — one narrow cone
  carries no useful global information and would stamp phantom obstacles).
- Docking: SOLVED 2026-08-11 by dock_aligner_v3 — POSITION control (/wheel_move count segments,
  not /cmd_vel), ARC segments never in-place pivots (the single rear caster stalls on a pivot),
  and POSE control (converges heading as well as position). Success = /dock/contact == 3 (both
  prox), which gates the IR beacon -> dock SSR -> charging. V1/V2 used velocity control and
  failed; do not resume them.
- Chassis calibration (do NOT "tidy" these): WHEEL_SEPARATION 0.3586 m, WHEEL_RADIUS 0.050 m
  (100mm AGV wheels), COUNTS_PER_REV 1290, caster 180mm behind the drive axle.
- Master launch: ros2 launch jupiter_bringup jupiter_bringup_full.launch.py
  - This is the ONE full-stack launch. Do not fork a second one — jupiter_bringup.launch.py and
    jupiter_full_s2e.launch.py were deleted 2026-08-13 after drifting onto a retired stack.
  - Nav comes from navigation_s2e.launch.py, which also starts the camera, the micro-ROS agent
    and every static TF. enable_nav:=false makes bringup_full supply camera+agent instead.
  - Mapping is separate: ros2 launch jupiter_bringup slam_mapping_s2e.launch.py
- Front ToF: VL53L0X on the ESP32 I2C bus, published as sensor_msgs/Range on /tof/front (~7Hz),
  frame tof_front_link. Verified 2.6mm accurate at 1m. NOT yet wired into any costmap layer.
- ESP32 auto-reconnect: 4-state micro-ROS state machine (WAITING→AVAILABLE→CONNECTED→DISCONNECTED), no physical reset needed
- ESP32 battery monitoring: publishes sensor_msgs/BatteryState on /battery/state at 1Hz (R1=100kΩ/R2=22kΩ divider on GPIO34 fitted and working — reads true pack voltage)

## Known Issues / Workarounds
- systemd services jupiter-microros and jupiter-lidar are DISABLED during development
- ⚠️ DO NOT run autonomous nav in clutter. The low-obstacle layer is ONE 25° ToF cone dead ahead
  to 1.2 m — it sees what you drive straight at and misses a chair leg 300mm off the centreline.
  It is not the ToF ring the plan assumed (the TCA9548A mux is dead; only one sensor is fitted).
- ⚠️ VL53L0X MOUNTING IS CRITICAL. In a light-grey printed casing the sensor returned ~4% valid
  readings — the bore reflected its own laser into the receiver. Black matte, aperture ≥6-7mm,
  module flush or proud, NEVER recessed. Full detail: firmware/i2c_scan/src/main.cpp.
- ⚠️ voltageScale() in firmware compensates motor duty against MEASURED PACK voltage, but VM is
  a regulated 12V rail — the motors never see the pack. Suspected to swing duty ~29% across a
  discharge for no physical reason. See docs/NEW_ESP32_MOTION_CONTROL_BOARD_2026.md §9.1.
- Thor's git had no github.com host key after the JetPack 7.2 rebuild, so it silently could not
  fetch. Fixed 2026-08-13, but Thor's working tree still carries uncommitted drift from that gap.

## Goals for This Session
- [Update this each session with what you want to achieve]