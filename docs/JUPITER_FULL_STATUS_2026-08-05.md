# Jupiter Robot — Full Development Status Report

**Author:** Logan Naidoo <naidoo.logan@gmail.com>
**Compiled:** 2026-08-05
**Scope:** every subsystem (Voice · Vision · AI · ROS 2 · Docking · Firmware), every file in `~/jupitercpp_ws`, and the complete ROS 2 underlay install list.

---

## 0. Executive summary

| Subsystem | State | Note |
|---|---|---|
| **Voice (ASR/TTS)** | ✅ Working | whisper.cpp CUDA + piper; mic offloaded to Pi 5 over Ethernet |
| **Vision — face rec** | ✅ Working | YuNet detect (OpenCV) + SFace embed (TensorRT); per-user profiles |
| **Vision — AprilTag** | ⛔ Retired | VPI tag pipeline works but the docking path it served was abandoned |
| **AI / Brain** | ✅ Working | ollama `gemma4:e2b` (text) + `llava:7b` (vision), HTTP on localhost:11434 |
| **Localisation (EKF)** | ✅ Working | wheel odom + BNO055 yaw + camera gyro → `odom→base_footprint` |
| **SLAM mapping** | ✅ Working | slam_toolbox + S2E; record→offline-replay workflow for large maps |
| **Nav2 autonomous** | ✅ Validated 2026-08-03 | AMCL + RPP + SmacPlanner2D; drove Lab↔lounge, no collisions |
| **Docking** | ❌ **Not achieved** | See §5. Detector solved; controller not. Best result `contact=2` |
| **Display / face** | ✅ Working | Qt6 QML face on 7″ HDMI |
| **Battery / charging** | ⚠️ Works, unreliable | Charges when seated; drained flat twice under load; no hysteresis |
| **ToF near-field ring** | 🔜 Parts on hand | 8× VL53L0X + 2× TCA9548A; one 8° mount printed, not wired |

**The one blocking gap for autonomy:** the robot cannot reliably return to its charger. Everything else in the companion loop (see, hear, think, speak, navigate) functions.

---

## 1. Hardware platform (as-built, 2026-08-05)

| Component | Interface | Role |
|---|---|---|
| Jetson AGX Thor | — | Main compute. JetPack 7.2 / L4T R39.2, Ubuntu 24.04, kernel 6.8.12-tegra |
| Raspberry Pi 5 | Ethernet `10.0.0.2` | Audio sub-node (ReSpeaker mic capture) |
| ESP32 | USB serial `/dev/jupiter_esp32` @ 460800 | Motors, encoders, BNO055 IMU, prox, battery ADC, dock IR |
| RPLIDAR S2E | **Ethernet/UDP** `192.168.11.2:8089` | Nav scan + dock reflector detection. Scan plane **0.518 m** |
| Orbbec Gemini 336 | USB3 | Colour (face rec). Tilted **5.7° nose-up** → near floor blind |
| 7″ HDMI display | HDMI | Jupiter face (Qt6 QML) |
| Drive | 2× front driven (DRV8870) + 2× rear casters | **Differential drive**, 100 mm AGV rubber wheels |
| Dock | Arduino Nano + SSR + TSOP + 2× inductive prox | IR-keyed charge gate |

> ⚠️ **`CLAUDE.md` is stale on three points** — it still says *mecanum wheels* (now diff-drive + casters), *LD20 LiDAR* (now S2E, LD20 physically removed), and *llama.cpp for LLM* (now ollama). Worth correcting; it is the file that steers every AI session.

---

## 2. Subsystem detail

### 2.1 Voice
```
ReSpeaker → [Pi 5] mic_capture_node → /audio/mic_raw (16 kHz mono S16)
          → [Thor] jupiter_voice → whisper.cpp (CUDA) → /voice/raw_text
          → jupiter_brain → /voice/response_text → piper TTS → HDMI speaker
```
Audio capture was moved to the Pi 5 to end the ReSpeaker/USB-DMA contention with the camera on Thor's single xHCI controller. `/voice/tts_done` signals speech completion (drives the display HUD). `/jupiter/expecting_name` gates name-capture during registration.

### 2.2 Vision
- **Face recognition** (`face_recognition.cpp`) — YuNet detector on OpenCV CPU, SFace embeddings via **TensorRT CUDA**. Publishes `/current_user`; the Brain uses it to greet by name and to route conversation history. Profiles + history live in `memory/` (gitignored, personal data).
- **AprilTag** (`vision.cpp`, VPI tag36h11) — functional, calibrated for 1280×720, but **retired**: it existed to serve AprilTag docking, which was abandoned in favour of the LiDAR-reflector approach.

### 2.3 AI / Brain
`brain.cpp` is the orchestrator: intent detection routes to **ollama** over HTTP (`localhost:11434/v1/chat/completions`) — `gemma4:e2b` for text, `llava:7b` for visual questions (grabs a camera snapshot via `/vision/trigger`). Maintains per-user conversation history on disk, handles sleep/wake, guest registration, and can command `/dock/engage`.

### 2.4 Navigation
```
TF: map →(AMCL)→ odom →(EKF)→ base_footprint → {base_laser, camera_link, imu_link}
```
- **EKF** fuses wheel odometry + BNO055 absolute yaw + Orbbec gyro (yaw-rate only).
- **AMCL** scan-matches S2E `/scan` against the saved map (`apartment_s2e_v2`).
- **Nav2**: RegulatedPurePursuit controller, SmacPlanner2D global planner, SimpleSmoother.
- **Validated 2026-08-03**: autonomous goals in the lounge/lab, round-trip, no collisions.

**Known nav issues (open):**
1. **Snaking** — reduced by wiring `SmoothPath` into the BT; residual likely wheel trim (`MOTOR1/2_TRIM` both 1.00, untuned for the 100 mm wheels).
2. **Floor threshold** (lab→lounge) — the robot **cannot climb it** at nav speed; clears every *other* threshold with momentum. Mechanical fix required (TPU ramp).
3. ⚠️ **Low-obstacle layer is blind.** `nav2_params.yaml` still declares `scan_low` (LD20, removed) and an `nvblox` costmap layer (retired). Neither publishes. **Do not run autonomous nav in clutter until the ToF ring exists.**

---

## 3. Complete file inventory

Legend: **[LIVE]** in active use · **[RETIRED]** kept as record, not built/run · **[DEAD]** not referenced by any build.

### 3.1 `src/jupiter_nodes/` — the custom C++ nodes

| File | Lines | Purpose | Depends on |
|---|---|---|---|
| `brain.cpp` **[LIVE]** | 898 | Intent → LLM/VLM; user tracking; conversation history; sleep/wake; dock trigger | `libcurl`, `nlohmann_json`, **ollama**; subs `/voice/raw_text`, `/current_user`, `/battery/state` |
| `voice.cpp` **[LIVE]** | 590 | ASR window assembly → whisper.cpp → text; TTS via piper | links `libwhisper.so`; subs `/audio/mic_raw` (Pi 5), `/voice/response_text` |
| `face_recognition.cpp` **[LIVE]** | 585 | YuNet detect + SFace embed → `/current_user`; face registration | TensorRT (`nvinfer`,`nvonnxparser`), OpenCV, CUDA; subs `/camera/color/image_raw` |
| `dock_reflector.cpp` **[LIVE]** | 278 | **Two-strip** retro-reflector dock detection → centre ● + face angle | `/scan`, tf2; feeds `dock_aligner` |
| `dock_aligner.cpp` **[LIVE, UNTESTED]** | 457 | Stop-and-re-aim docking state machine → `/cmd_vel` | `/dock/reflector`, `/imu/data`, `/dock/contact` |
| `imu_covariance_fixer.cpp` **[LIVE]** | 65 | Injects sane covariances into BNO055 `/imu/data` | feeds EKF as `imu0` |
| `scan_deskew_node.cpp` **[LIVE, mapping only]** | 196 | Motion-compensates intra-scan skew before SLAM | `/scan` + `/odometry/filtered` |
| `webcam_publisher.cpp` **[RETIRED]** | 74 | Rear USB webcam → `/webcam/image_raw` | served AprilTag docking |
| `vision.cpp` **[RETIRED]** | 320 | VPI AprilTag detect → `/vision/marker_pose` | VPI4, CUDA; served AprilTag docking |
| `dock_approach.cpp` **[RETIRED]** | 308 | AprilTag docking controller (2-phase + go-around) | `/vision/marker_pose` |
| `dock_ir.cpp` **[RETIRED]** | 640 | IR-heartbeat + AprilTag hybrid docking controller | `/dock/ir_rate`, `/dock/range` |
| `dock_range.cpp` **[RETIRED]** | 157 | LiDAR wall-ranging for the IR docking path | `/scan` |
| `diagnostics.cpp` **[DEAD]** | 28 | Thermal-zone read stub — **not in `CMakeLists.txt`, never built** | — |

**Build config:** `CMakeLists.txt` (12 executables), `package.xml`.

### 3.2 `src/jupiter_bringup/` — launch + config

**Launch files (`launch/`):**

| File | State | Purpose |
|---|---|---|
| `jupiter_bringup_full.launch.py` | **LIVE — the daily launch** | Everything: camera, voice, brain, face, display, nav |
| `jupiter_full_s2e.launch.py` | LIVE | Full-stack S2E coexistence test (voice+vision+AI+nav together) |
| `navigation_s2e.launch.py` | **LIVE** | S2E + AMCL + EKF + Nav2 (current nav stack) |
| `slam_mapping_s2e.launch.py` | LIVE | Map building with S2E |
| `record_mapping.launch.py` + `offline_mapping.launch.py` | LIVE | Record→replay workflow for large (220 m²) maps |
| `offline_mapping_deskew.launch.py` | LIVE | A/B harness for `scan_deskew_node` |
| `camera.launch.py` / `camera_ai.launch.py` | LIVE | Orbbec persistent launch (all streams / colour-only) |
| `teleop_test.launch.py` | LIVE | Base+lidar+EKF only — isolates drivetrain from Nav2 |
| `ld20_low.launch.py` | RETIRED | LD20 low-obstacle layer (hardware removed) |
| `navigation.launch.py`, `jupiter_bringup.launch.py`, `slam.launch.py`, `visual_slam.launch.py`, `nvblox.launch.py`, `slam_mapping.launch.py` | RETIRED | cuVSLAM / nvblox / LD20 era |
| `dock_ir.launch.py`, `dock_ir_nodes.launch.py`, `dock_opennav.launch.py`, `dock_simple.launch.py`, `apriltag_detect_test.launch.py` | RETIRED | Superseded docking generations |

> **Note:** the current docking pair (`dock_reflector` + `dock_aligner`) has **no launch file** — deliberately run via `ros2 run` against one long-lived micro-ROS agent, because restarting the agent churns the ESP32 into a reconnect loop.

**Config (`config/`):**

| File | Purpose |
|---|---|
| `nav2_params.yaml` (386 ln) | Full Nav2 config. ⚠️ still declares dead `scan_low` + `nvblox` layers |
| `ekf_odom.yaml` (99 ln) | robot_localization: odom0 + imu0 (BNO055 abs yaw) + imu1 (camera gyro rate) |
| `slam_params.yaml` (62 ln) | slam_toolbox, Ceres solver, 18 m max range for S2E |
| `navigate_to_pose_omni.xml` | Behaviour tree — Wait-only recovery (no blind Spin/BackUp); `SmoothPath` added 2026-08-03 |
| `navigate_through_poses_omni.xml` | Multi-waypoint BT variant |
| `docking_server.yaml` | RETIRED — opennav_docking config |
| `orbbec_gemini336_profiles.md`, `startup_strategy.md` | Reference notes |

**Other:** `rviz/jupiter_nav.rviz`, `maps/c82_map_real.*`, `scripts/jupiter-usb-camera-fix.sh` (Orbbec cold-boot xHCI rebind), `scripts/lidar_watchdog.sh`, `systemd/jupiter-usb-camera-fix.service`.

### 3.3 Other packages

| Package | State | Notes |
|---|---|---|
| `jupiter_display/` | **LIVE** | Qt6 QML animated face (`qml/JupiterFace.qml`, `src/display_node.cpp`) + HUD (mode, WiFi, sensor health, temps, battery) |
| `jupiter_audio_capture/` | **LIVE on Pi 5** | `mic_capture_node.cpp` — ALSA → `/audio/mic_raw`. Has `COLCON_IGNORE` so Thor skips it |
| `sllidar_ros2/` | **LIVE** | Vendor RPLIDAR driver (S2E over UDP). ~100 SDK files |
| `ldlidar_stl_ros2/` | **RETIRED** | Vendor LD20/LD19 driver — hardware removed |

### 3.4 `firmware/` — ESP32 + dock microcontrollers

| File | Purpose |
|---|---|
| `esp32/src/firmware.ino` (638 ln) | Main firmware: micro-ROS client, motor PID, odometry, IMU, prox seat logic, battery, dock IR emitter, `cmd_vel` watchdog (400 ms) |
| `esp32/include/jupiter_config.h` (128 ln) | **All pins + tuning constants.** PID `K_P 5.0 / K_I 5.0`, trims 1.00 (untuned), battery divider, IR burst timing, prox debounce |
| `esp32/{motor,encoder,pid,kinematics,odometry,imu_bno055,oled_1306}.cpp/.h` | Hardware modules (PID + odometry derived from Juan Miguel Jimeno's linorobot, Apache-2.0) |
| `esp32/platformio.ini` | PlatformIO: esp32dev, micro-ROS serial transport, Adafruit BNO055/SSD1306 libs |
| `dock_charger/src/dock_charger.ino` (78 ln) | **Dock-side Nano**: TSOP receives robot's 38 kHz beacon → gates SSR. Power-on state = OFF (dead dock) |
| `dock_beacon/src/jupiter_dock_beacon.ino` (67 ln) | RETIRED — old dock-side IR beacon (direction reversed; the robot now emits) |
| `esp32/archive/` | Old firmware/kinematics copies |

**ESP32 micro-ROS interface:** publishes `/odom/unfiltered`, `/imu/data`, `/battery/state`, `/dock/contact`, `wheel_encoders`, `wheel_speeds`; subscribes `cmd_vel`, `/save_imu`.

### 3.5 Supporting directories

| Path | Contents |
|---|---|
| `docs/` | 7 documents + this one (see §7) |
| `maps/` | 12 saved maps. **Current: `apartment_s2e_v2.{yaml,pgm}`** (0.05 m/px) |
| `hardware/dock_funnel/` | Parametric funnel generator (`funnel_rail_gen.py`), STLs, DXF, README |
| `hardware/ir_receiver_shroud.scad` | IR receiver shroud (IR-era) |
| `apriltags/` | Printable tag PDFs/PNGs (retired path) |
| `scripts/` | Python bench tools: `dock_range_monitor`, `ir_rate_monitor`, `ir_monitor`, `drive_pulse`, `scan_rear_peek` |
| root `*.py`, `*.sh` | `floor_pitch_check.py` (camera pitch cal), `scan_sectors.py`, `record_mapping_run.sh` |
| **gitignored** | `whisper.cpp/` (1.8 G), `piper_tts/` (185 M), `models/` (76 M), `memory/` (260 K — **personal data, never push**), `llama.cpp/` (now empty — retired) |

---

## 4. ROS 2 underlay — required installs

### 4.1 Base
| Item | Why |
|---|---|
| **Ubuntu 24.04 + JetPack 7.2 (L4T R39.2)** | Platform. Provides CUDA, cuDNN, **TensorRT 10.16** |
| **ROS 2 Jazzy** (`ros-jazzy-desktop`) | Middleware. RMW = default FastDDS. `ROS_DOMAIN_ID=0` **must match the Pi 5** |
| `colcon`, `rosdep` | Build tooling |

### 4.2 ROS packages (all confirmed present at `/opt/ros/jazzy`)
| Package | Needed by | Why |
|---|---|---|
| `robot_localization` | `ekf_odom.yaml` | EKF fusing wheel odom + IMU → `odom→base_footprint` |
| `nav2_amcl` | navigation_s2e | Localisation against the saved map |
| `nav2_map_server` | navigation_s2e | Serves the static occupancy grid |
| `nav2_controller` | navigation_s2e | Hosts RegulatedPurePursuit |
| `nav2_planner` | navigation_s2e | Hosts SmacPlanner2D |
| `nav2_smoother` | navigation_s2e | SimpleSmoother — irons out grid-planner jags (anti-snaking) |
| `nav2_bt_navigator` | navigation_s2e | Runs the custom behaviour trees |
| `nav2_behaviors` | navigation_s2e | Recovery behaviours (Wait/costmap clear only) |
| `nav2_waypoint_follower` | navigation_s2e | Multi-waypoint missions ("go to known spots" plan) |
| `nav2_velocity_smoother` | navigation_s2e | Smooths `cmd_vel_nav` → `cmd_vel` |
| `nav2_lifecycle_manager` | navigation_s2e | Auto-activates the lifecycle nodes |
| `nav2_costmap_2d` | nav2_params | Costmap layers |
| `slam_toolbox` | slam_mapping_s2e | Map building |
| `tf2_ros` | everywhere | Transform tree + static publishers |
| `cv_bridge` | face_rec, vision, webcam | ROS Image ↔ OpenCV |
| `image_transport` | camera | Image pipeline |
| `vision_msgs` | vision | Detection message types |

**Present but unused:** `opennav_docking` (retired Gen-2 docking).
**Absent on Thor (by design):** `rviz2`, `teleop_twist_keyboard` — run from the hub/desktop over the LAN. `apriltag_ros` — Jupiter uses its own VPI implementation.

### 4.3 Separate overlay
| Item | Why |
|---|---|
| **`~/microros_ws` → `micro_ros_agent`** | ESP32 link. **Lives outside the main workspace** — every launch must `source ~/microros_ws/install/local_setup.bash` first. Serial `/dev/jupiter_esp32` @ **460800** (not 115200) |

### 4.4 Non-ROS libraries (confirmed versions)
| Library | Version | Needed by | Why |
|---|---|---|---|
| **TensorRT** | 10.16.2.10+cuda13.2 | `face_recognition` | SFace embedding inference on GPU |
| **CUDA** | via JetPack 13.x | face_rec, whisper, VPI | GPU compute |
| **OpenCV** | 4.8.0 | face_rec, vision, webcam | YuNet detection, image ops |
| **NVIDIA VPI** | vpi4 (`/opt/nvidia/vpi4`) | `vision.cpp` | Hardware AprilTag detector *(retired path — but `jupiter_nodes` will not configure without it)* |
| **libcurl** | 8.5.0 | `brain` | HTTP to ollama |
| **nlohmann_json** | 3.11.3 | brain, face_rec | JSON payloads + profile store |
| **Qt6** (Core/Gui/Quick/Qml) | 6.4.2 | `jupiter_display` | QML face UI |
| **libasound2-dev** | 1.2.11 | `mic_capture_node` (Pi 5) | ALSA capture |
| **Orbbec SDK** | — | camera | Gemini 336 driver (`orbbec_camera` ROS pkg) |

### 4.5 AI runtimes (workspace-local, gitignored)
| Runtime | Size | Why |
|---|---|---|
| **whisper.cpp** (CUDA build) | 1.8 G | ASR. `jupiter_voice` links `libwhisper.so` directly and hard-codes the relative path in `CMakeLists.txt` — **the workspace layout matters** |
| **piper** | 185 M | TTS |
| **ollama** 0.32.5 | ~12 G models | LLM+VLM server. **`gemma4:e2b`** + **`llava:7b`** only |
| **ONNX models** | 76 M | YuNet + SFace. TensorRT engines **rebuild on-device on first run** — never copy `.trt` between machines |

**Explicitly NOT installed** (previously trialled, all removed): Isaac ROS / cuVSLAM / nvblox, Docker, vLLM + Qwen3, llama.cpp, sherpa-onnx, DeepStream.

---

## 5. Docking — the honest assessment

Docking is the **only subsystem that does not work**. Four generations have been attempted.

### 5.1 What did NOT work

| Gen | Approach | Why it failed |
|---|---|---|
| **1** | IR beacon (dock→robot) balance | IR balance is **flat far out** (both receivers saturate) — cannot resolve lateral until the mouth, too late to correct |
| **2** | **AprilTag** (`dock_approach`, `dock_ir`) | Good (x,z), **noisy yaw** → robot arrived angled. Later `opennav_docking` added an opaque `external_detection_rotation` frame convention causing consistent misalignment, plus a **backward runaway** when the ESP32 latched the last `cmd_vel` on node silence (→ drove the watchdog fix) |
| **3** | **Single horizontal retro strip** + square-then-reverse | Centroid was good; the **PCA line-fit ANGLE went to mush below ~0.35 m** (the 250 mm strip subtends ~65°, fits an arc not a line) → forced a blind IMU-heading freeze for the final 0.5 m |
| **4** | Single-strip + **frozen IMU heading hold** | ⭐ **The architectural error.** It controlled *squareness to the face* and then held a **memorised compass heading** — it never targeted the **dock centre**. "Square ≠ centred": the robot can be perfectly square and still 6 cm off the centreline. Any offset was locked in and integrated into a lateral slide → banked left or right → missed the throat |

**Control laws tried inside Gen 3/4 — all failed, all for the same reason:**

| Attempt | Result |
|---|---|
| P-loop `kp_heading 1.2` | ±10° **limit cycle** (loop delay ~150–250 ms on an integrator plant) |
| Detuned P `kp 0.5` | Oscillation gone, but **steady ~6° droop** → 19 cm lateral drift |
| **Gyro-rate damping** (`k_damp`, added to the code) | Stable, sign correct, but did **not** fix the drift — the drift is slow, damping only fights fast rates |
| Higher gain + damping (`kp 0.9`) | Best heading yet (±5°, recovered) — but still **6.5 cm off-centre** at the throat |
| Bolted-on **cross-track** trim | **Unstable** — overshot, banked left (this had also failed and been removed in an earlier era) |
| Raising speed 0.08 → 0.14 m/s | Escaped wheel stiction, but did not fix centring |

### 5.2 ⭐ The decisive finding

> **This drivetrain has essentially no steering authority while reversing at crawl speed.**

Measured live: a **saturated** correction (`angular.z = 0.20` held for 2+ seconds at `v = 0.14`) produced **no rotation**, while the lateral drift kept growing. The wheel-speed differential (±3 cm/s) cannot overpower the side-force of the loaded **leading** casters. (Reversing turns the rear trailing casters into *leading* casters — classically unstable; the drift direction was random left/right depending on how the casters happened to sit.)

This single fact invalidates **every** in-motion steering scheme, which is why all of them failed regardless of gains.

**What the robot *does* do reliably (proven every run):**
1. **Rotate in place** to ±1° (min-speed 0.15 + deadband + 0.3 s settle).
2. **Reverse dead straight open-loop** — teleop at 0.21 m/s with `angular = 0` gave a flat gyro (±0.002 rad/s).

### 5.3 What IS working

| Component | Status |
|---|---|
| **Two-strip reflector detector** (`dock_reflector.cpp`) | ✅ **Solved and proven.** Two vertical retro strips (20 mm × 125 mm, 250 mm apart, midpoint = pogo centre). Two-cluster centroids → dock centre ●; baseline-perpendicular → angle. Measured baseline 256 mm vs 250 expected. **Skew rock-steady** where the single strip flickered −13°…+21°. Angle stays valid to the throat and *improves* closer in. Free false-positive rejection via the rigid 250 mm separation check |
| **Charging chain** | ✅ ESP32 fires 38 kHz beacon only when both prox seated AND battery < 16.70 V → dock Nano closes SSR. Pogo pins dead until asked (correct safety posture) |
| **Seat sensing** | ✅ Two inductive prox → `/dock/contact` bitmask (1=L, 2=R, 3=both), 5-cycle debounce, 1.5 s seat grace |
| **Funnel rails** | ✅ Reprinted with 100 mm throat + shorter funnel. Improved behaviour (robot reaches the throat instead of wedging at the mouth) but cannot square a 6–8° cocked arrival |

### 5.4 What is built but INCOMPLETE

**`dock_aligner.cpp` — "stop-and-re-aim" controller. Compiled on Thor. NEVER TESTED.**

Design (uses *only* the two proven primitives, no in-motion steering):
```
ACQUIRE → AIM (rotate in place, point the REAR at a carrot ON the centreline)
        → CRUISE (angular = EXACTLY 0, straight, IMU-monitored)
        → drift > 3° and range > 0.45 m? → stop, re-AIM, cruise again
        → inside 0.45 m: committed, straight into the throat
        → THROAT (blind firm push, rails absorb residual angle)
        → PUSH (pivot the open corner if only one prox latched) → SEATED
```
Priority is **centre first, square second** — the throat exists to absorb residual *angle*; it can never fix arriving *off-centre*. Each stop-and-re-aim also settles the casters (the "pre-flip" for free).

### 5.5 Recommended paths if docking is resumed

1. **One clean test** of the stop-and-re-aim controller (it is built and waiting).
2. **Mechanical capture** — a genuinely wide funnel mouth that swallows ±5 cm / ±10° arrivals. The vacuum-robot philosophy: *make the dock forgiving rather than the robot precise.* Print in halves if build-plate-limited. **This is the highest-confidence path.**
3. **Caster change** — damped or rigid rear casters to kill the reverse instability at source.
4. *(Doctrine-level)* Nose-first docking would put the casters trailing and make the control problem far easier — but it conflicts with the deliberate choice to reverse-dock so Jupiter keeps facing the room while charging.

### 5.6 Related open safety item

⚠️ **SSR stays latched after undocking mid-charge.** The directional IR still reaches the dock TSOP at ≥1.1 m, so pulling the robot off a live dock can leave the SSR closed → **pogo pins live at 16.8 V while undocked**. *Interim rule: power the dock off before undocking mid-charge.* Needs a strict/fast emitter cutoff on any prox release.

---

## 6. Cross-cutting open items

| Item | Impact |
|---|---|
| **ToF ring not built** (8× VL53L0X + 2× TCA9548A on ESP32 I²C mux) | Nav is **blind below 0.518 m** — cables, feet, the incoming puppy. Blocks autonomous nav in clutter. One 8° mount printed; nothing wired |
| **Wheel trims untuned** (`MOTOR1/2_TRIM = 1.00`) | Residual snaking; noted as "re-tune on new wheels" since the 100 mm swap |
| **Battery return-to-dock logic** | Not implemented — and a flat pack **cannot be dock-recovered** (ESP32 needs pack power to fire the IR handshake → chicken-and-egg). "Never go flat" is the only mitigation |
| **Charging under load** | Robot drained flat *while docked* twice — full bringup draw exceeds the ~75 W dock. Needs the "docked companion" low-power mode |
| **Internal LAN switch on Pi 5 USB power** | Single point of failure: Pi 5 hang drops the whole internal net (Thor↔Pi5↔S2E) |
| **`nav2_params.yaml` references dead sources** | `scan_low` + `nvblox` layers declared but nothing publishes |
| **Uncommitted work** | 6 modified files including both docking nodes (see §8) |

---

## 7. Documentation index

| Doc | Contents |
|---|---|
| `JUPITER_ROBOT_DESIGN.md` (413 ln) | Master design document |
| `JUPITER_PROJECT_STATUS.md` (338 ln) | Earlier status review |
| `JUPITER_DOCKING_ISSUES.md` (287 ln) | Docking saga + Gen-3 issue log (⚠️ predates the two-strip work) |
| `NAV2_STATUS.md` (234 ln) | Nav2 configuration status |
| `JUPITER_CLEAN_REBUILD.md` (119 ln) | **Reproducible rebuild manifest** post-reflash — the install source of truth |
| `THOR_USB_DMA.md` (96 ln) | USB/DMA topology (authoritative — don't re-derive with `lsusb`) |
| `DOCK_FUSION_BRINGUP.md` (101 ln) | Fused docking runbook |

---

## 8. Git state

**Branch head:** `03d11e3` — *JetPack 7.2 rebuild + docking: full-bringup fixes, both-prox seat*

**Uncommitted (6 files):**
```
M .gitignore
M src/jupiter_bringup/config/nav2_params.yaml          # yaw_goal_tolerance 3.14 → 0.10
M src/jupiter_bringup/config/navigate_to_pose_omni.xml # SmoothPath added to BT
M src/jupiter_bringup/launch/navigation_s2e.launch.py  # micro-ROS source fix; LD20 node removed
M src/jupiter_nodes/src/dock_aligner.cpp               # stop-and-re-aim rewrite (UNTESTED)
M src/jupiter_nodes/src/dock_reflector.cpp             # two-strip detector (PROVEN)
```
The nav fixes and the two-strip detector are tested and worth committing; the aligner rewrite is untested.

---

*End of report.*
