# Jupiter Robot Design Document

**Author:** Logan Naidoo <naidoo.logan@gmail.com>  
**Started:** Early 2026  
**Status:** Active Development  
**License:** Apache-2.0

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware](#2-hardware)
   - 2.1 [Compute Platform](#21-compute-platform)
   - 2.2 [Drive System](#22-drive-system)
   - 2.3 [Power System](#23-power-system)
   - 2.4 [Sensors](#24-sensors)
   - 2.5 [Microcontroller](#25-microcontroller)
   - 2.6 [Peripherals](#26-peripherals)
3. [Software Stack](#3-software-stack)
   - 3.1 [Operating System & Middleware](#31-operating-system--middleware)
   - 3.2 [Localisation & Odometry](#32-localisation--odometry)
   - 3.3 [Mapping & Navigation](#33-mapping--navigation)
   - 3.4 [Obstacle Sensing](#34-obstacle-sensing)
   - 3.5 [Vision System](#35-vision-system)
   - 3.6 [Docking System](#36-docking-system)
   - 3.7 [Voice & Brain (AI)](#37-voice--brain-ai)
   - 3.8 [Audio Sub-node (Pi5)](#38-audio-sub-node-pi5)
   - 3.9 [Display HUD](#39-display-hud)
   - 3.10 [Power Management](#310-power-management)
4. [System Architecture](#4-system-architecture)
5. [Key Engineering Decisions](#5-key-engineering-decisions)
6. [Known Issues & Workarounds](#6-known-issues--workarounds)
7. [Roadmap](#7-roadmap)

---

## 1. Project Overview

Jupiter is a 4-wheeled autonomous home-assistant robot built on a Jetson AGX Thor compute platform. It is designed to navigate a domestic apartment autonomously, recognise and interact with registered users by name, respond to voice commands, and return to its charging dock when battery is low.

**Core capabilities (target):**
- Autonomous room-to-room navigation with obstacle avoidance
- Face recognition — identifies registered users, greets guests and offers registration
- Natural language voice interaction via wake-word, ASR, LLM brain, and TTS
- AprilTag-guided precision docking for autonomous recharging
- Real-time sensor fusion from dual LiDAR planes, depth camera, and IMU

**Design philosophy:**  
Pure C++ throughout (no Python in ROS 2 nodes), bare-metal Ubuntu (no Docker), CUDA-accelerated AI inference on the Blackwell GPU. The emphasis is on a production-quality embedded system, not a research prototype.

**The name:** Jupiter — the largest planet in the solar system. Appropriately ambitious.

---

## 2. Hardware

### 2.1 Compute Platform

| Item | Choice | Reason |
|---|---|---|
| SBC | NVIDIA Jetson AGX Thor | Blackwell GPU (sm_110), 64GB unified memory, PCIe NVMe, sufficient for simultaneous Nav2 + LLM + vision inference |
| JetPack | 7.1 (L4T R38.4.0) | Latest stable at project start; CUDA 13.0+, TensorRT 10.x |
| OS | Ubuntu 24.04 bare metal | No Docker overhead; direct hardware access for CUDA, USB, serial |
| Storage | NVMe SSD (internal) | Fast I/O for map data, model weights, conversation history |

**Challenges:**
- **Cold-boot hang:** Thor intermittently freezes at UEFI "Exiting boot services" on true cold start (battery fully disconnected overnight). No journal entry (pre-userspace). Workaround: double power cycle via circuit breaker. Root cause: UEFI PCIe/xHCI enumeration timing on Blackwell; no L4T firmware fix available in R38.4.0.
- **No 40-pin GPIO:** Unlike older Jetson modules, the AGX Thor developer kit has no 40-pin expansion header — only USB-A, USB-C, and PCIe. This ruled out direct GPIO-based sensor connections and forced all peripherals onto USB or Ethernet.
- **Single xHCI USB controller:** All USB ports share one DMA controller and IRQ. High-bandwidth camera (USB3) and micro-ROS serial (USB2) on the same bus caused DMA stalls that blocked the slam_toolbox executor every ~4 seconds during Whisper GPU inference. Resolved by USB topology management and isolating the Orbbec camera on a dedicated hub.

### 2.2 Drive System

| Item | Initial Choice | Final Choice | Reason for Change |
|---|---|---|---|
| Wheels | Mecanum (omni-directional) | **Rubber skid-steer (65mm)** | Mecanum wheels could not climb door threshold lips (~8mm). Rubber wheels handle thresholds; lose lateral strafe but gain threshold clearance. Strafe retained only for AprilTag docking approach. |
| Motor controller | ESP32-based custom | ESP32-based custom (unchanged) | micro-ROS on ESP32 publishes `/odom/unfiltered` and subscribes `/cmd_vel`. Mecanum kinematics computed on ESP32. |
| Chassis | Multi-level aluminium extrusion (20×20mm) | Unchanged | Four vertical uprights at corners support upper levels. These uprights are within the LD20 LiDAR scan radius and required costmap min-range masking. |

**Note on mecanum vs rubber:** The decision to switch to rubber wheels was made after repeated navigation failures at door thresholds. Mecanum wheels are smooth-tyred and stall on the 8mm lip. The Nav2 stack and docking approach were redesigned around the rubber wheel (skid-steer) kinematics.

### 2.3 Power System

| Rail | Source | Consumers |
|---|---|---|
| 16.8V (4S5P Li-Ion) | Battery pack | Jetson AGX Thor, motor controller, 12V buck input |
| 12V (300W/20A buck) | 16.8V rail | ESP32, motor controller, 7" HDMI display |
| 5V | WaveShare USB hub | Peripheral USB devices |

**Protection:** Easton 40A MCB (miniature circuit breaker) on the main battery rail. This breaker serves as both the primary power switch and an emergency stop. It is also the mechanism used to force a full hardware reset when Thor hangs during cold boot.

**Charging:** Battery is disconnected and charged on a bench supply overnight. The Pi5 shutdown service (see §3.10) ensures the Pi5 is cleanly powered off before the battery is disconnected.

### 2.4 Sensors

#### RPLIDAR S2E (Primary LiDAR — navigation)
- **Interface:** Ethernet UDP (static IP 192.168.11.2, Thor at 192.168.11.100/24 on `enP2p1s0`)
- **Mount height:** 0.515m above ground
- **Frame:** `base_laser`, yaw π (faces backward, corrected in TF)
- **Topic:** `/scan`
- **Role:** Primary obstacle detection for Nav2 ObstacleLayer; AMCL scan-matching for localisation

#### LD20 LiDAR (Secondary LiDAR — low obstacle layer)
- **Interface:** USB serial (cp210x), `/dev/jupiter_lidar`, 230400 baud
- **Mount height:** 0.13m above ground, 0.06m forward of centre
- **Frame:** `ld20_laser`, yaw 0
- **Topic:** `/scan_low`
- **Role:** Catches low obstacles (chair legs, feet, kick-boards) invisible to the S2E at 0.515m
- **Initial choice:** Was the primary LiDAR before the S2E arrived. Parked, then reinstated as the low obstacle layer.
- **Self-structure masking:** The robot's own aluminium uprights return at <0.20m. Costmap `obstacle_min_range` and `raytrace_min_range` set to 0.30m — no angular crop needed.

#### Orbbec Gemini 336 (Depth Camera)
- **Interface:** USB3 (identified by serial number `CP9KB53000HP`, not port)
- **Mount:** 0.100m forward, 0.475m height, pitch −0.0995 rad (5.7° nose-up for face recognition)
- **Topics:** `/camera/color/image_raw`, `/camera/depth/...`
- **Role (current):** Color stream for face recognition and AprilTag docking. Depth stream currently disabled — camera tilt makes floor-grazing depth unreliable for nvblox.
- **Initial role:** Depth + nvblox GPU ESDF for low obstacle detection. **Retired** when camera was tilted up for face recognition, making the depth data unreliable at the floor grazing angle. Replaced by LD20.
- **Cold-boot quirk:** Orbbec missing on cold boot due to Tegra USB3 xHCI quirk. Fixed by `jupiter-usb-camera-fix.service` which rebinds the xHCI controller at boot. Camera moved to USB-A 10G hub at port 2-3.1.

#### BNO055 IMU
- **Interface:** I²C via ESP32, published to `/imu/data`
- **Corrected topic:** `/imu/data/corrected` (via `imu_covariance_fixer` node)
- **Role:** Absolute yaw orientation (magnetometer, NDOF mode) + yaw rate fusion in EKF. Calibration persists across reboots.
- **EKF fusion:** Absolute yaw orientation enabled (critical for heading stability — without this, heading integrates from rate only and drifts between AMCL corrections, causing wavy straight-line drives).

### 2.5 Microcontroller

**ESP32 via micro-ROS (C/C++ only)**

- Publishes: `/odom/unfiltered` (mecanum wheel kinematics), `/battery/state` (1Hz, `sensor_msgs/BatteryState`)
- Subscribes: `/cmd_vel` (motor commands)
- Connected via USB serial `/dev/jupiter_esp32`
- **micro-ROS agent:** Runs on Thor as an `ExecuteProcess` in all launch files

**Challenge — reconnection:** Early firmware required a physical reset if the micro-ROS agent restarted. Resolved with a 4-state reconnection state machine (`WAITING → AVAILABLE → CONNECTED → DISCONNECTED`) that re-initialises automatically without physical intervention.

**Challenge — cmd_vel watchdog:** The ESP32 has no internal cmd_vel watchdog. If a node goes silent, the last velocity command is latched and the robot keeps moving. The `dock_approach` node was specifically written to publish zero velocity every tick when not engaged, regardless of state.

**Battery monitoring:** Hardware voltage divider (R1=100kΩ / R2=22kΩ) on GPIO34 reads the battery voltage. Readings are garbage until the divider is physically wired; software is complete.

### 2.6 Peripherals

| Device | Interface | Role |
|---|---|---|
| ReSpeaker 3800 | USB (via Pi5) | Microphone array for voice capture |
| 7" HDMI display | HDMI + USB touch | Status HUD (mode, WiFi, sensors, temps, battery, face display) |
| WaveShare USB hub | USB-A | ESP32 + display + LD20 isolation from camera bus |
| Raspberry Pi 5 | Ethernet (10.0.0.2) | Dedicated audio sub-node (ASR/TTS pipeline) |

---

## 3. Software Stack

### 3.1 Operating System & Middleware

- **OS:** Ubuntu 24.04 (JetPack 7.1), bare metal
- **ROS 2:** Jazzy
- **Build:** Colcon + CMake, Release build type, `ament_cmake`
- **Language:** Pure C++17 throughout all ROS 2 nodes. No Python in the ROS graph.
- **DDS:** FastDDS (default). CycloneDDS evaluated as fallback for Ethernet interface-pinning issues — not yet needed.
- **ROS_DOMAIN_ID:** 0 (reverted from a trial of 82; must match on Thor and Pi5 everywhere)

### 3.2 Localisation & Odometry

**EKF (`robot_localization`)** fuses:
| Source | Fused States | Notes |
|---|---|---|
| `/odom/unfiltered` (ESP32) | vx, vy, vyaw | Mecanum wheel kinematics |
| `/imu/data/corrected` (BNO055) | yaw orientation, vyaw | Absolute yaw anchor + rate |

- Publishes `odom → base_footprint` TF at 50Hz, `two_d_mode: true`
- Process noise for yaw reduced (0.06 → 0.01) to prevent heading drift between AMCL corrections

**AMCL** (Adaptive Monte Carlo Localisation):
- Scan-matches `/scan` (S2E) against the static apartment map
- Publishes `map → odom` TF (global correction)
- Update triggers: 0.05m translation or 0.1 rad rotation
- Particle range: 500–3000

**TF chain:** `map → odom` (AMCL) → `odom → base_footprint` (EKF) → `base_footprint → {base_laser, ld20_laser, camera_link, imu_link}`

**Challenge — heading drift:** Without absolute yaw in the EKF, the heading integrated from rate only, drifting between AMCL corrections. This manifested as wavy straight-line drives even on clear floor. Fixed by enabling BNO055 absolute yaw orientation fusion and reducing yaw process noise.

### 3.3 Mapping & Navigation

**SLAM:** `slam_toolbox` async mode — used to build the apartment occupancy grid map. Saved as `maps/apartment_s2e_v2.yaml`. Not run during navigation (map is pre-built).

**Navigation stack:** Nav2 (Jazzy)

| Component | Choice | Notes |
|---|---|---|
| Global planner | SmacPlanner2D | Cost-aware A* with MOORE-8 connectivity; routes through door/passage centres via `cost_travel_multiplier: 4.0`. Replaced NavFn which cut corners. |
| Local controller | RPP (Regulated Pure Pursuit) | Deterministic path tracker; replaced MPPI which freelanced in open space. |
| Path smoother | SimpleSmoother | `w_smooth: 0.6 / w_data: 0.1` — aggressively smooths A* lattice jags before RPP sees them |
| Costmap | ObstacleLayer (dual source) | `/scan` (S2E, high) + `/scan_low` (LD20, low) |

**RPP tuning history:**
- `rotate_to_heading_min_angle`: 0.785 → 1.0 → 1.4 rad (reduce oscillation near obstacles)
- `rotate_to_heading_angular_vel`: 1.0 → 0.5 rad/s (prevent overshoot)
- `max_angular_accel`: 3.2 → 1.6 rad/s² (gentle ramp, no snap)
- `min_lookahead_dist`: 0.30 → 0.40 → 0.70m (average out path lattice jags)

**Motion model:** DiffDrive (vx + wz only) for navigation. Strafe (vy) reserved for AprilTag docking approach only.

**Challenge — nvblox floor-marking bug:** nvblox was marking the floor as an obstacle because the config used non-existent parameter names (`esdf_2d_min_height` / `esdf_2d_max_height`). These were silently ignored, leaving defaults that treated everything as an obstacle. Fixed by verifying parameter names against `/opt/ros/jazzy/include/nvblox/` installed headers (`esdf_slice_min_height: 0.10` / `esdf_slice_max_height: 1.80`). This was the root cause of the robot being frozen and unable to move to any goal.

**Challenge — AMCL vs NavFn corner cutting:** NavFn planned diagonal shortcuts through walls and furniture because the global costmap lacked a static layer. Fixed by adding `StaticLayer` to the global costmap so Smac sees the pre-built map.

### 3.4 Obstacle Sensing

**Two-layer approach (post nvblox retirement):**

| Layer | Source | Height | Catches |
|---|---|---|---|
| High | S2E `/scan` | 0.515m | Walls, furniture, people above shin height |
| Low | LD20 `/scan_low` | 0.13m | Chair legs, feet, kick-boards, floor clutter |

Both feed Nav2's `ObstacleLayer` as `observation_sources`. Self-structure masking via `obstacle_min_range: 0.30` and `raytrace_min_range: 0.30` — the robot's own aluminium uprights return at <0.20m and are ignored.

**nvblox retirement (2026-06-20):** Camera tilted 5.7° up for face recognition makes depth data unreliable at the floor grazing angle. The LD20 takes over the low-obstacle role at a fraction of the GPU cost.

### 3.5 Vision System

**Face Recognition (`jupiter_vision` node, C++ / TensorRT)**
- SFace model, TensorRT engine compiled on Thor (Blackwell sm_110)
- Preprocessing: RGB, normalised to [−1, 1]
- Threshold: 0.40 cosine similarity
- **Bug fixed:** Early version used raw BGR/[0,255] → false matches between different people. Fixed 2026-06-04 (commit 805121e). All face profiles must be re-registered after this fix.
- Registered users: Indrani (wife), Logan — stored with face embeddings and conversation history
- Unregistered users greeted as "Guest" with registration offer

**AprilTag Detection (`jupiter_vision` node, C++ / VPI)**
- Tag family: tag36h11, tag ID 1 = docking station
- Physical tag size: 149×149mm (printed)
- Pose estimation: solvePnP → 6DOF pose in `camera_color_optical_frame`
- Published: `/vision/marker_pose` (geometry_msgs/PoseStamped)
- Tag mounting height: ~0.50m centre from floor (matches camera FOV centre at docking distance with 5.7° up-tilt)

### 3.6 Docking System

**Initial approach:** `opennav_docking` package. **Abandoned** — opaque `external_detection_rotation` convention caused consistent misalignment; backward runaway occurred when ESP32 latched last cmd_vel on node silence.

**Final approach:** Bespoke `dock_approach` node (~150 lines C++), written from scratch.

**Algorithm:**
1. Subscribe `/vision/marker_pose`
2. TF-transform tag pose to `base_footprint`
3. Compute approach point on tag normal at `target_distance`
4. Servo: `vx = k_lin × ρ`, `vy = k_lin × lateral_error`, `wz = k_ang × yaw_error`
5. Stop when `ρ < position_tol` and `|yaw| < yaw_tol` — or tag lost within `docked_on_loss_rho`
6. Publish zero velocity every tick when not engaged (prevents ESP32 cmd_vel latch)

**Key parameters:**
- `target_distance: 0.28m` (drives ~120mm closer than the 0.40m visual standoff to seat pogo pins)
- `k_lin: 0.8`, `k_ang: 1.0`, `max_linear: 0.12 m/s`, `max_angular: 0.5 rad/s`
- `position_tol: 0.04m`, `yaw_tol: 0.05 rad`

**Engagement:** `/dock/engage` topic (Bool) or `/dock_engage` service (SetBool) or voice command "go to the dock".

**Square-up result:** Left side 0.33m / right side 0.35m from dock wall — 2cm / ~2° alignment, acceptable for pogo pin contact.

**Challenge — polar controller singularity:** First holonomic controller used a polar (ρ, α, β) formulation that went singular as ρ→0, causing spinning and tag loss. Replaced with direct Cartesian servo (current approach) which is well-behaved at close range.

### 3.7 Voice & Brain (AI)

**ASR:** `whisper.cpp` with CUDA (GGUF model), running on Thor GPU  
**TTS:** `piper_tts` (C++), fast neural TTS  
**LLM:** `llama.cpp` → migrating to `vLLM 0.21.0 + Qwen3.6-35B-A3B-FP8` (MoE, sm_110 kernels, port 8000)  
**VLM:** `llava:7b` via ollama HTTP (for environment description)

**Brain node (`brain.cpp`)** — the orchestration core:
- Wake word detection: "hey Jupiter" / "wake up"
- Sleep command: "go to sleep"
- Dock command: "go to the dock" / "dock yourself" / "go charge" → publishes `/dock/engage`
- User identification via face recognition → personalised responses
- Conversation history per registered user
- Guest flow: polite introduction + registration offer
- **Registration cancel:** Any of "cancel", "stop", "skip", "guest", "never mind", "forget it", "leave it", "don't register", "not interested" → cancels registration mid-flow, continues as Guest

**VAD (Voice Activity Detection):** Startup-skip + 25% trim prevents false triggers from pw-record startup audio ramp and HVAC/fan noise.

### 3.8 Audio Sub-node (Pi5)

The ReSpeaker microphone moved to a Raspberry Pi 5 (10.0.0.2) to isolate the USB audio device from Thor's congested xHCI bus.

- Full voice pipeline runs on Pi5: VAD → Whisper ASR → publish to Thor via ROS 2 (ROS_DOMAIN_ID=0, FastDDS over Ethernet)
- jupiter-audio.service: systemd auto-start on Pi5 boot
- Pi5 has NVMe SSD (not microSD)

**Shutdown protection (2026-06-23):**
- `jupiter-shutdown-pi5.service` on Thor: SSHes Pi5 and powers it off when Thor shuts down
- `jupiter-thor-watchdog.service` on Pi5: if Thor (10.0.0.1) doesn't respond within 90s of Pi5 boot (Thor hung at UEFI), Pi5 powers itself off cleanly before the user hard-cuts the circuit breaker

### 3.9 Display HUD

7" HDMI display shows a real-time status overlay:
- **Top bar:** Mode (INTERACT/NAVIGATE/DOCK), WiFi SSID
- **Bottom bar:** Sensor health (LiDAR, camera, IMU, ESP32), CPU/GPU temps, battery percentage bar
- TTS-done signal triggers face display update (eliminates word-count lag)
- Screensaver disabled at launch: `xset s off s noblank dpms 0 0 0`

### 3.10 Power Management

**Battery monitoring:** ESP32 reads the battery voltage via hardware divider (R1=100kΩ / R2=22kΩ on GPIO34), publishes `sensor_msgs/BatteryState` on `/battery/state` at 1Hz. 16.0V / 83.4% confirmed on battery power.

**Low-battery dock return:** Logic planned — brain monitors `/battery/state`; below threshold, interrupts current task and navigates to dock. Not yet implemented.

**USB camera fix:** `jupiter-usb-camera-fix.service` rebinds the xHCI controller at boot to recover the Orbbec camera from Tegra USB3 cold-init quirk. Runs in 8 seconds.

---

## 4. System Architecture

```
                    ┌─────────────────────────────────────────────────────┐
                    │                  JETSON AGX THOR                    │
                    │                                                     │
  Battery 16.8V ───┤  ┌──────────┐  ┌──────────┐  ┌──────────────────┐ │
  (4S5P Li-Ion)    │  │  Nav2    │  │  Brain   │  │  Vision          │ │
                    │  │  Stack   │  │  (LLM/   │  │  (Face Rec +     │ │
  12V Buck ─────── │  │  AMCL    │  │   VLM)   │  │   AprilTag)      │ │
  ESP32/Display    │  │  RPP     │  │  Whisper │  │  TensorRT C++    │ │
                    │  │  Smac2D  │  │  Piper   │  │                  │ │
                    │  └────┬─────┘  └────┬─────┘  └────────┬─────────┘ │
                    │       │              │                  │           │
                    │  ┌────▼─────────────▼──────────────────▼─────────┐ │
                    │  │              ROS 2 Jazzy (DDS)                 │ │
                    │  └──┬──────────┬──────────┬──────────┬───────────┘ │
                    │     │          │           │          │             │
                    │  /scan      /scan_low  /cmd_vel  /dock/engage      │
                    │     │          │           │          │             │
                    └─────┼──────────┼───────────┼──────────┼────────────┘
                          │          │           │          │
                     S2E LiDAR   LD20 LiDAR   ESP32     dock_approach
                    (Ethernet)   (USB serial) (micro-ROS)   node
                                               │
                                          Mecanum Motors
                                          + BNO055 IMU

         ┌──────────────────┐
         │  Raspberry Pi 5  │   Ethernet (10.0.0.0/24)
         │  ReSpeaker USB   │ ─────────────────────────► Thor 10.0.0.1
         │  ASR + TTS       │   ROS 2 topics (audio)
         │  10.0.0.2        │
         └──────────────────┘
```

---

## 5. Key Engineering Decisions

### 5.1 ROS 2 vs Custom Navigation
At a point of repeated Nav2 frustration (opennav_docking failures, nvblox bugs, AMCL corner-cutting), the decision was made to evaluate MRPT, OMPL, and custom P2P navigation as alternatives. After analysis, **ROS 2 / Nav2 was retained** because:
- The bugs encountered were configuration errors, not fundamental Nav2 limitations
- The ecosystem (AMCL, SmacPlanner2D, RPP) is mature and well-understood once properly configured
- Custom navigation would require reimplementing path planning, costmaps, and localisation from scratch
- Commercial robots (Boston Dynamics, Clearpath) do use ROS internally

### 5.2 Mecanum → Rubber Wheels
The original mecanum wheels were chosen for omnidirectional capability (strafe for docking). Rubber skid-steer wheels were adopted when mecanum proved unable to cross 8mm door threshold lips. The strafe capability needed for docking alignment is retained by keeping the ESP32 firmware's vy command path active — it is simply not used during normal navigation.

### 5.3 nvblox → LD20 for Low Obstacle Sensing
nvblox (GPU ESDF from depth camera) was the original low-obstacle layer. It was retired when the camera was tilted 5.7° up for face recognition, making floor-grazing depth unreliable. The LD20 LiDAR (already mounted on the chassis) was reinstated as a costmap observation source at 0.13m height — simpler, more reliable, and zero GPU cost.

### 5.4 Bespoke Docking vs opennav_docking
`opennav_docking` was trialled and abandoned after persistent misalignment caused by opaque `external_detection_rotation` conventions. The bespoke `dock_approach` node (150 lines) solved the problem cleanly in one session by measuring the tag → driving to it → stopping at the target distance. Direct, testable, no black-box conventions.

### 5.5 Pi5 Audio Sub-node
The ReSpeaker microphone was originally on Thor's USB bus. USB DMA contention between Whisper GPU inference and the xHCI controller caused SLAM executor stalls. Moving audio capture to a dedicated Pi5 connected via Ethernet eliminated the DMA conflict entirely and improved camera init time from 50s to 2.5s.

---

## 6. Known Issues & Workarounds

| Issue | Status | Workaround |
|---|---|---|
| Thor UEFI cold-boot hang | Open — firmware bug | Double power cycle via circuit breaker; Pi5 watchdog protects against NVMe corruption |
| Wavy straight-line drives | In progress | EKF absolute yaw + reduced process noise (needs hardware confirmation) |
| Door threshold lips | Resolved | Rubber wheels; doorway nav working |
| Orbbec missing on cold boot | Resolved | `jupiter-usb-camera-fix.service` |
| ESP32 cmd_vel latch | Resolved | dock_approach publishes zero every tick when idle |
| BNO055 yaw drift | Resolved | Absolute yaw orientation fusion enabled in EKF |
| SLAM + Whisper DMA conflict | Resolved | Audio moved to Pi5 sub-node |
| Face recognition false matches | Resolved | SFace preprocessing bug fixed (commit 805121e); profiles re-registered |
| nvblox floor-marking | Resolved | Correct param names verified against installed headers |
| Pi5 hard-kill on Thor shutdown | Resolved | `jupiter-shutdown-pi5.service` + `jupiter-thor-watchdog.service` |

---

## 7. Roadmap

### Near Term
- [ ] Confirm BNO055 absolute yaw EKF fusion on hardware (stationary angular.z noise test)
- [ ] Dual IMU fusion: add Orbbec built-in IMU as `imu1` (yaw rate, high frequency)
- [ ] Improve docking alignment precision
- [ ] Low-battery dock-return logic in brain.cpp

### Medium Term
- [ ] Pan-tilt camera servo (face tracking without fixed up-tilt compromise)
- [ ] Voice capture wedge fix (continuous pw-record stream to prevent mic silence after first capture)
- [ ] Operational modes (INTERACT / NAVIGATE / DOCK) to manage GPU resource contention
- [ ] Registration: ASR name confirmation before committing to profile

### Long Term
- [ ] Same-name user collision handling (unique ID as key, name as label)
- [ ] Resource governor (GPU arbitration between Nav2, Whisper, VLM, face recognition)
- [ ] ESP32 power-on watchdog relay (auto-retry Thor cold boot without manual intervention)
- [ ] Full room-to-room navigation under complete AI stack load

---

*Document maintained alongside the codebase at `docs/JUPITER_ROBOT_DESIGN.md`.*  
*Last updated: 2026-06-23*
