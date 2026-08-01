# Jupiter Robot — Project Status Review

**Author:** Logan Naidoo <naidoo.logan@gmail.com>
**License:** Apache-2.0
**Status snapshot date:** 2026-07-27
**Scope:** Complete current state of ALL hardware and software — what is in use now, and what it replaced.

> This document is the current-state overview. The older narrative design doc lives at
> [`docs/JUPITER_ROBOT_DESIGN.md`](JUPITER_ROBOT_DESIGN.md) (dated 2026-06-23, describes the
> 4-wheel/mecanum + AprilTag era) and is now **partly superseded** by this file — see
> §12 "What Was Replaced".

---

## 0. What Jupiter Actually Is (read this first)

Jupiter is **not** a navigation trolley that happens to talk. It is an **always-on vision / voice / AI companion** that lives in the home:

- The **Orbbec camera is the primary, always-on sensor** — face recognition and presence run continuously.
- **Voice + AI brain** (ASR → LLM/VLM → TTS) is the main interaction loop.
- **`jupiter_bringup_full` is the single daily launch** — it runs continuously, *including while the robot is docked and charging*.
- **Docking is in service of that job**, not the job itself: the robot **reverse-docks (caster-first)** *on purpose* so it keeps **facing and serving the user while it charges**. Autonomous charging is "how Jupiter stays alive to keep doing its real work."

Every engineering decision is judged against that: keep the companion alive and interactive.

---

## 1. Current Status at a Glance

| Subsystem | State | Notes |
|---|---|---|
| Compute (Jetson AGX Thor) | ✅ Operational | Occasional UEFI cold-boot hang (documented workaround) |
| Drive train (2× wheel + caster) | ✅ Operational | Rebuilt this cycle: brake-mode reverse, feed-forward, PID anti-windup |
| Odometry | ✅ Fixed & honest (~6%) | Landmark `getRPM()` 2.6× bug fixed; CPR recalibrated 1372→1290 |
| Battery / power | ✅ Operational | Live `/battery/state`; ~16.2 V / 87 % at last check |
| Charging chain (dock SSR + IR gate) | ✅ Bench-proven | See §10; two safety items parked (SSR latch, hysteresis) |
| S2E LiDAR (primary) | ✅ Operational | Ethernet UDP; navigation + dock reflector detection |
| Orbbec Gemini 336 camera | ✅ Operational | Primary always-on sensor; cold-boot USB fix in place |
| IMU (heading) | ⚠️ In transition | BNO055 switched NDOF→IMUPLUS (magnetometer was motor-corrupted); Orbbec IMU under evaluation as alternative |
| Face recognition | ✅ Working | SFace/TensorRT; false-match bug fixed |
| Voice (ASR/TTS) | ✅ Working | whisper.cpp + piper on Pi5 sub-node |
| AI brain (LLM/VLM) | ✅ Working / migrating | llama.cpp → vLLM + Qwen3 MoE; llava VLM |
| Navigation (Nav2 + AMCL, 2D LiDAR) | ✅ Configured/working | cuVSLAM + nvblox **abandoned** (feature-poor home) |
| **Docking (reverse, LiDAR-reflector)** | 🚧 **In progress** | Detection + square + gentle-hold reverse **proven**; full both-prox seat **not yet proven** — see §9 |
| Display HUD | ✅ Working | 7" HDMI status overlay |
| Low obstacle layer (LD20) | ⏸️ Parked | LiDAR mounted but unplugged; planned 2nd costmap layer |

Legend: ✅ working · ⚠️ decision open · 🚧 active dev · ⏸️ parked

---

## 2. Hardware — Current Configuration

### 2.1 Compute

| Item | Choice | Notes |
|---|---|---|
| SBC | NVIDIA Jetson AGX Thor | Blackwell GPU (sm_110), 64 GB unified, internal NVMe |
| JetPack | 7.1 (L4T R38.4.0) | CUDA 13.x, TensorRT 10.x |
| OS | Ubuntu 24.04 bare metal | No Docker |
| Hostname / net | `thor` @ 192.168.0.8 (LAN); 192.168.11.100 (LiDAR net) | dual-homed |

**Known constraints:** No 40-pin GPIO (USB/PCIe only); single xHCI USB controller shared across all USB (drove the USB-topology work and the Pi5 audio offload); intermittent UEFI cold-boot hang at "Exiting boot services" (workaround: power-cycle; Pi5 watchdog protects the filesystem). See [`docs/THOR_USB_DMA.md`](THOR_USB_DMA.md).

### 2.2 Drive System — **evolved 4 → 2 wheels + caster**

| Generation | Configuration | Why replaced |
|---|---|---|
| v1 | **4× mecanum** (omni, can strafe) | Could not climb ~8 mm door-threshold lips; smooth tyres stalled |
| v2 | **4× rubber skid-steer, 65 mm** | Threshold clearance gained; lost strafe. Interim. |
| **v3 (current)** | **2× drive wheels (100 mm AGV) + 1 rear caster** | Simpler, robust differential drive; caster gives thresholds + a stable 3-point stance. Wheels installed 2026-07-11. |

**Current drive facts** (see [`firmware/esp32/include/jupiter_config.h`](../firmware/esp32/include/jupiter_config.h)):
- **2 driven wheels:** `motor1` = LEFT, `motor2` = RIGHT (100 mm AGV, radius 0.050 m). `motor3/4` defined but **unused** (rear caster).
- **Rear caster:** centre-rear, free-swivelling (office-chair double caster). **Leads during reverse docking.**
- Wheel separation `0.355 m`; `COUNTS_PER_REV = 1290` (re-calibrated 2026-07-27, was 1372).
- Kinematics = **differential drive** (`kinematics.cpp` fixed from a mecanum `(L+W)` term to pure `W` rotation).
- Motor driver: dual-input H-bridge, **brake-mode** in both directions (reverse was previously slow-decay/weak → fixed this cycle).

### 2.3 Power & Charging

| Rail | Source | Consumers |
|---|---|---|
| 16.8 V (4S5P Li-Ion) | Battery pack | Thor, motor controller, 12 V buck input |
| 12 V (300 W / 20 A buck) | 16.8 V | ESP32, motor controller, 7" display |
| 5 V | WaveShare USB hub | peripheral USB |

- **Protection / E-stop:** Easton 40 A MCB on the main rail (also the cold-boot hard-reset switch).
- **Battery telemetry:** ESP32 ADC via divider on GPIO34 → `/battery/state` at 1 Hz. Calibrated to multimeter.
- **Charging:** contact charging on the dock via pogo pins, gated by an IR handshake + SSR (see §10). Bench charging used during development.
- ⚠️ **Charging-reliability caveat:** the robot has twice gone flat *while sitting on the dock running full bringup* — charge-in did not sustain the heavy always-on load. Parked; mitigations = return-to-dock at a comfortable threshold + load-shed when docked. A truly flat pack **cannot** be dock-recovered (pins go live only after the ESP32 fires the IR handshake, and the ESP32 needs pack power → chicken-and-egg) — bench charge required.

### 2.4 Sensors

| Sensor | Interface | Mount | Role (current) |
|---|---|---|---|
| **RPLIDAR S2E** (primary) | Ethernet UDP `192.168.11.2:8089` | **0.518 m** height, `base_laser`, yaw π (rear = bearing 0) | Nav2 obstacle/AMCL scan-match **and** dock retro-reflector detection |
| **Orbbec Gemini 336** (primary always-on) | USB3 (by serial) | ~0.475 m, ~5.7° nose-up | Color → face recognition; depth available. **The always-on companion sensor.** |
| **BNO055 IMU** | I²C via ESP32 → `/imu/data` | on-chassis | Heading. **NDOF→IMUPLUS** (gyro+accel, magnet-immune) after motor magnetic fields corrupted NDOF yaw |
| **Orbbec 336 built-in IMU** | USB (camera) | in camera | **Under evaluation** as heading source vs BNO055 |
| **2× inductive proximity** (LJ18A3-8-Z/BX) | ESP32 GPIO (rear-L 13, rear-R 15) | rear corners | Dock **seat** confirmation: LOW = metal/contact. `/dock/contact` bitmask (1=L, 2=R, 3=both) |
| **LD20 LiDAR** | USB serial | 0.13 m | ⏸️ Low-obstacle layer — **mounted but unplugged**, parked |

### 2.5 Microcontroller — ESP32 (micro-ROS)

- Firmware: C/C++ (PlatformIO), micro-ROS over **serial XRCE-DDS, 460800 baud**, device `/dev/jupiter_esp32`.
- Agent runs on Thor from `~/microros_ws` (must be sourced — full bringup currently launches it but does not source that overlay: known gap).
- **Publishes:** `/odom` (diff-drive), `/imu/data`, `/battery/state`, `/dock/contact`.
- **Subscribes:** `/cmd_vel`.
- **cmd_vel watchdog:** 400 ms silence → motors brake (prevents last-command latch runaway).
- **Charge-enable IR emitter** (TSAL6400, GPIO4): 38 kHz burst packets, fires only while **both prox seated AND battery < 16.70 V** — the dock's keep-alive dead-man.
- 4-state auto-reconnect state machine (no physical reset needed).

### 2.6 Peripherals

| Device | Interface | Role |
|---|---|---|
| ReSpeaker 3800 mic array | USB via **Raspberry Pi 5** | Voice capture (offloaded off Thor's USB bus) |
| Raspberry Pi 5 | Ethernet | Audio sub-node: capture → (ASR feed). NVMe-booted. Clean-shutdown + watchdog services |
| 7" HDMI display + touch | HDMI + USB | Status HUD |
| WaveShare USB hub | USB-A | ESP32 + display isolation from camera bus |

---

## 3. Software — Workspace Map

Workspace root: `~/jupitercpp_ws`  ·  Language: **pure C++17** in all ROS 2 nodes  ·  ROS 2 **Jazzy**  ·  `ament_cmake`, Release builds.

### 3.1 Packages (`src/`)

| Package | Purpose |
|---|---|
| [`jupiter_nodes`](../src/jupiter_nodes) | All the C++ application nodes (brain, voice, vision, docking, diagnostics) |
| [`jupiter_bringup`](../src/jupiter_bringup) | Launch files, Nav2/EKF/SLAM config, maps, systemd units |
| [`jupiter_audio_capture`](../src/jupiter_audio_capture) | Pi5 mic capture node (`COLCON_IGNORE` on Thor build) |
| [`jupiter_display`](../src/jupiter_display) | 7" HUD (Qt/QML) |
| [`sllidar_ros2`](../src/sllidar_ros2) | RPLIDAR S2E driver (Ethernet) |
| [`ldlidar_stl_ros2`](../src/ldlidar_stl_ros2) | LD20 driver (parked low-obstacle layer) |

### 3.2 Nodes (`src/jupiter_nodes/src/`) — purpose & status

| Node | File | Purpose | Status |
|---|---|---|---|
| Brain | [`brain.cpp`](../src/jupiter_nodes/src/brain.cpp) | Intent → LLM/VLM, user tracking, wake/sleep, dock command, conversation history | ✅ |
| Voice | [`voice.cpp`](../src/jupiter_nodes/src/voice.cpp) | Subscribes `/audio/mic_raw` (from Pi5) → whisper ASR → `/voice/raw_text`; `/voice/response_text` → piper TTS | ✅ |
| Face recognition | [`face_recognition.cpp`](../src/jupiter_nodes/src/face_recognition.cpp) | YuNet detect + SFace embed (TensorRT) → `/current_user`; registration | ✅ |
| Vision | [`vision.cpp`](../src/jupiter_nodes/src/vision.cpp) | VPI AprilTag (tag36h11) — **legacy docking path** | 🗄️ legacy |
| **Dock reflector** | [`dock_reflector.cpp`](../src/jupiter_nodes/src/dock_reflector.cpp) | **CURRENT docking sensing** — S2E intensity threshold → bright cluster → PCA line-fit → `/dock/reflector_pose` + `/dock/reflector` | ✅ |
| **Dock aligner** | [`dock_aligner.cpp`](../src/jupiter_nodes/src/dock_aligner.cpp) | **CURRENT docking control** — ACQUIRE → SQUARE → REVERSE_IN → PUSH → SEATED | 🚧 |
| Dock approach | [`dock_approach.cpp`](../src/jupiter_nodes/src/dock_approach.cpp) | AprilTag docking controller | 🗄️ superseded |
| Dock IR / range | [`dock_ir.cpp`](../src/jupiter_nodes/src/dock_ir.cpp), [`dock_range.cpp`](../src/jupiter_nodes/src/dock_range.cpp) | IR-beacon docking era | 🗄️ superseded |
| Scan deskew | [`scan_deskew_node.cpp`](../src/jupiter_nodes/src/scan_deskew_node.cpp) | Motion-compensate LaserScan during turns | ✅ (SLAM aid) |
| IMU covariance fixer | [`imu_covariance_fixer.cpp`](../src/jupiter_nodes/src/imu_covariance_fixer.cpp) | Rewrites BNO055 covariances for EKF | ✅ |
| Webcam publisher | [`webcam_publisher.cpp`](../src/jupiter_nodes/src/webcam_publisher.cpp) | Tail UVC webcam for AprilTag | 🗄️ legacy |
| Diagnostics | [`diagnostics.cpp`](../src/jupiter_nodes/src/diagnostics.cpp) | Sensor-health aggregation for HUD | ✅ |

🗄️ = kept in-tree as record but not in the current daily path.

### 3.3 Daily launch

**The one daily command:** `ros2 launch jupiter_bringup jupiter_bringup_full.launch.py`
([`jupiter_bringup_full.launch.py`](../src/jupiter_bringup/launch/jupiter_bringup_full.launch.py)) — starts, in order: Orbbec camera (kept warm) → display → micro-ROS agent → navigation stack (EKF + LiDAR + Nav2) → voice/whisper → brain + face-rec + vision. Runs continuously, even while docked.

---

## 4. Motion Control & Odometry

Firmware modules in [`firmware/esp32/`](../firmware/esp32):

| Module | File | Role |
|---|---|---|
| Main loop | `src/firmware.ino` | micro-ROS pub/sub, prox/seat logic, IR emitter gate, watchdog |
| Kinematics | `src/kinematics.cpp` | Diff-drive cmd_vel ↔ wheel RPM (mecanum `(L+W)` term removed) |
| PID | `src/pid.cpp` | Per-wheel closed-loop + **anti-windup** (conditional integration) |
| Motor | `src/motor.cpp` | H-bridge PWM; **brake-mode both directions** + static-friction feed-forward |
| Encoder | `src/encoder.cpp` | Quadrature counts + RPM |
| Odometry | `src/odometry.cpp` | Wheel odom integration → `/odom` |
| IMU | `src/imu_bno055.cpp` | BNO055 (**IMUPLUS mode**) |
| Config | `include/jupiter_config.h` | All pins & tuning constants |

### Landmark fix this cycle — the `getRPM()` 2.6× bug
`Encoder::getRPM()` returned **0.0** when called sooner than its 100 ms recompute window. The control loop runs at ~20–26 Hz (far faster than 10 Hz), so most calls got 0.0 → the PID read the wheel as *stalled* and **over-drove ~2.6×**, while odometry **under-reported by the same factor**. Net effect for months: "the robot drives too fast" and every odom-based distance was wrong (drove ~2.5 m on a 1 m command). **Fixed** by holding the last computed RPM between updates (`last_rpm_`). This also retroactively corrected an earlier wrong conclusion that "the BNO055 was the liar" — the *odom* was the liar; the BNO was closer to truth.

**Feed-forward** is **breakaway-only** (applied only while a wheel is stalled and commanded to move) so it breaks stiction without slamming fine heading corrections during docking.

### Localisation (EKF)
`robot_localization` EKF fuses `/odom` (ESP32) + `/imu/data/corrected` (BNO055), publishes `odom→base_footprint`, `two_d_mode: true`. Config: [`config/ekf_odom.yaml`](../src/jupiter_bringup/config/ekf_odom.yaml).

---

## 5. Navigation & Mapping

**Current stack (2D LiDAR):**
- **SLAM:** `slam_toolbox` async → apartment occupancy grid (`maps/apartment_s2e_v2.yaml`).
- **Localisation:** AMCL scan-matching `/scan` (S2E) against the static map.
- **Nav2:** SmacPlanner2D (global) + Regulated Pure Pursuit (local) + SimpleSmoother; **DiffDrive** motion model. Config: [`config/nav2_params.yaml`](../src/jupiter_bringup/config/nav2_params.yaml).
- **Deskew:** `scan_deskew_node` compensates the sweep-during-turn distortion.

**Abandoned vision-nav (important history):** an Isaac-ROS **cuVSLAM + nvblox** stack was built and briefly "worked," then **abandoned** — the home is **feature-poor for vision** (white walls, white floor tiles, narrow passages starve visual odometry; low light drops frame rate → 0 landmarks). nvblox was also retired when the camera was tilted up for face recognition (floor-grazing depth unreliable). **2D LiDAR SLAM + AMCL is the committed approach.** (Legacy launch/config remain in-tree; see the memory notes flagged for archiving.)

---

## 6. Vision System

- **Face recognition** ([`face_recognition.cpp`](../src/jupiter_nodes/src/face_recognition.cpp)): YuNet detect (OpenCV CPU) + SFace embed (TensorRT/CUDA), 0.40 cosine threshold, `/current_user`. Registered users have face embeddings + conversation history; unknown → "Guest" + registration offer. (Early false-match bug from BGR/[0,255] preprocessing fixed 2026-06-04 — profiles must be re-registered after that fix.)
- **AprilTag** (`vision.cpp`, VPI tag36h11): part of the **legacy** docking path, retained but not in the current dock flow.

---

## 7. Voice & AI Brain

| Function | Tool | Notes |
|---|---|---|
| ASR | `whisper.cpp` (CUDA, GGUF) | Runs against Pi5-captured audio |
| TTS | `piper` (neural, C++) | `/voice/response_text` → speech |
| LLM (text) | `llama.cpp` → **migrating to vLLM 0.21 + Qwen3 MoE FP8** | port 8000, sm_110 kernels |
| VLM | `llava:7b` (ollama HTTP) | environment description |
| Orchestration | [`brain.cpp`](../src/jupiter_nodes/src/brain.cpp) | wake/sleep, intent routing, user context, dock command |

Audio capture is on the **Pi5 sub-node** ([`mic_capture_node.cpp`](../src/jupiter_audio_capture/src/mic_capture_node.cpp)) to keep the ReSpeaker off Thor's congested xHCI bus (which was stalling the SLAM executor during Whisper inference). Pi5 has clean-shutdown + Thor-hang watchdog systemd services.

---

## 8. Docking & Charging Overview

Jupiter **reverse-docks (caster-first)** so it keeps facing the user while charging. The docking *sensing/control* (§9) and the *charging chain* (§10) are separate concerns.

### Docking approach — evolution
| Generation | Method | Why replaced |
|---|---|---|
| v1 | **AprilTag** (camera, `dock_approach`) | Good (x,z) but noisy **yaw** → robot arrived angled; single-tag pose jittery at close range |
| v2 | **IR beacon + LiDAR wall-range** (`dock_ir`, `dock_range`) | IR gave side but no range; brittle; superseded |
| v2.5 | **opennav_docking** (Nav2 Docking Server) | Opaque `external_detection_rotation` convention → misalignment; abandoned |
| **v3 (current)** | **S2E LiDAR + retro-reflective strip**, reverse-in | mm-accurate, robust; the LiDAR is already always-on for nav |

---

## 9. Current Docking System (v3) — Detailed Status

### Concept
A **250 × 50 mm retro-reflective strip** on the dock, at LiDAR height (0.518 m). On the S2E's 6-bit intensity scale (0–63) the strip **saturates at 63** while room clutter reads ≤ ~33 → an intensity threshold (~40) isolates it cleanly.

### Detector — [`dock_reflector.cpp`](../src/jupiter_nodes/src/dock_reflector.cpp)
Intensity threshold → largest contiguous bright cluster → PCA line-fit → TF to `base_footprint`. Publishes:
- `/dock/reflector_pose` (PoseStamped)
- `/dock/reflector` (Float32MultiArray): `[valid, along, lateral, range, bearing, skew, strip_len, fit_rms, n]`

Key derived metric: **`nyaw = wrap(skew + bearing + π)`** = the dock-face heading error (0 = square). This — not raw skew — is what the controller nulls.

### Controller — [`dock_aligner.cpp`](../src/jupiter_nodes/src/dock_aligner.cpp)
State machine: **IDLE → ACQUIRE → SQUARE → REVERSE_IN → PUSH → SEATED / ABORT** (service-triggered via `/dock/align_start`).

- **ACQUIRE:** wait for a stable reflector lock.
- **SQUARE:** rotate in place at the pre-dock pose to null `nyaw` to **±1°**. Uses a **min-rotational-speed floor (0.15 rad/s) + hard ±1° deadband + 300 ms settle** — this recipe **converges cleanly with no oscillation** (the previous versions hunted ±14°). **This is proven and is the best square achieved to date.**
- **REVERSE_IN:** gentle reverse creep. Far band uses reflector `nyaw`; close-in (range < `reflector_trust_range`) switches to **IMU gyro heading-hold** (reflector angle degrades at close range). Proven to deliver the robot **square and centred** from a good staging.
- **PUSH** *(new, built 2026-07-27, UNTESTED)*: when one prox latches but not both, pivot the un-seated rear corner toward the dock (derived sign: left-only → CW, right-only → CCW) until both confirm. **Self-terminating** — firmware zeros angular the instant `dock_seated` (both prox) is true.

### Where it stands — honest
- ✅ Reflector detection across the full approach.
- ✅ SQUARE to ±1°, no oscillation.
- ✅ Gentle-hold reverse arrives square + centred **from a good (on-axis) staging** → seats **one** prox (`contact=1`) reliably. Best approach achieved to date.
- ❌ **Full both-prox seat (`contact=3`) not yet proven.** A ~4° mechanical throat-entry deflection engages one prox, not both.
- ⚠️ **Funnel entry from an *off-axis* staging is unsolved.** An 8–10 cm lateral staging offset overshoots the funnel. An **active cross-track correction during the reverse was attempted and removed** — it is an unstable/coupled control problem in the wrong layer. **Correct architecture: the lateral offset belongs to Nav2 staging** (drive the robot onto the dock's normal axis first), *then* square + reverse **straight**. `dock_aligner` retains an (off-by-default) `lateral_gain`/`lat_sign` hook but it should not be relied on.
- 🚧 The **PUSH** seat-closer is built but **not yet tested** (the one test run failed earlier at SQUARE — the wheels did not rotate and the reflector latched a spurious object; cause needs eyes-on-robot: drivetrain-not-commanding vs reflector-going-spurious).

### Two candidate finishers for the last cm (either/both)
1. **Motor PUSH** (software, built, untested) — pivot the open corner in.
2. **3D-printed throat-extension guide rails** (mechanical) — mechanically square the last cm.

### Uncommitted work
`git status` shows **uncommitted** changes to [`dock_aligner.cpp`](../src/jupiter_nodes/src/dock_aligner.cpp) (SQUARE recipe + PUSH state), plus `docking_server.yaml` and `dock_opennav.launch.py` (legacy). Build is clean; not committed pending a proven test.

---

## 10. Charging Chain (bench-proven)

- **Dock side:** an Arduino **Nano** ([`firmware/dock_charger/`](../firmware/dock_charger)) gates a **Solid-State Relay** to the pogo pins. It closes the SSR **only** while it receives the robot's 38 kHz IR keep-alive packets ("dead-man's" gate) — so the pins are **dead until the robot asks**, which is the correct safety posture (no exposed live 16.8 V).
- **Robot side:** ESP32 fires the IR emitter (GPIO4) **only** when both prox are seated AND battery < 16.70 V.
- **Seat sensing:** two inductive prox sensors at the rear corners; firmware debounces `both-LOW` → `dock_seated`, with a bounded 1.5 s grace after first contact so the rails can square the robot.
- ⚠️ **Parked safety items:** (7) SSR can stay latched if the robot is pulled out mid-charge (IR still reaches the dock TSOP at range) — pins may stay live when undocked; interim = power the dock off before undocking. (5) full-charge cutoff has no hysteresis → SSR can chatter near 16.70 V under load. Both in the parking lot.

---

## 11. Firmware Detail (ESP32)

See §4 table. Key constants ([`jupiter_config.h`](../firmware/esp32/include/jupiter_config.h)): `K_P 5.0 / K_I 5.0 / K_D 0`, `MOTOR_FF_STATIC 200` (breakaway-only, gated by `MOTOR_FF_RELEASE_RPM 4.0` / `MOTOR_FF_CMD_MIN 3.0`), `COUNTS_PER_REV 1290`, `CMD_VEL_TIMEOUT_MS 400`, `CONTACT_SEAT_GRACE_MS 1500`, `BATTERY_FULL_STOP 16.70`. Flash via `~/.platformio/penv/bin/pio run -t upload`; agent from `~/microros_ws`.

Other firmware trees: [`firmware/dock_charger`](../firmware/dock_charger) (Nano SSR gate, current), [`firmware/dock_beacon`](../firmware/dock_beacon) (old IR beacon, superseded).

---

## 12. What Was Replaced / Abandoned (consolidated)

| Area | Was | Now | Reason |
|---|---|---|---|
| Wheels | 4× mecanum → 4× rubber 65 mm | **2× 100 mm AGV + rear caster** | Door thresholds; simpler diff-drive |
| Kinematics | Mecanum (holonomic) | **Differential drive** | Follows the wheel change |
| Docking sense | AprilTag → IR beacon → opennav | **S2E LiDAR retro-reflector, reverse-in** | Accuracy + robustness; reuse always-on LiDAR |
| Nav odometry/localisation | cuVSLAM (visual) | **2D LiDAR SLAM + AMCL** | Home too feature-poor for vision |
| 3D obstacle | nvblox (GPU ESDF) | (LD20 low layer, parked) | Camera tilted up for faces |
| IMU heading mode | BNO055 **NDOF** (magnetometer) | BNO055 **IMUPLUS** (gyro+accel) | Motor magnetic fields corrupted the magnetometer |
| LLM runtime | llama.cpp | **vLLM + Qwen3 MoE** (migrating) | VLM latency, MoE efficiency |
| Mic location | Thor USB | **Raspberry Pi 5** sub-node | Thor xHCI DMA stalled SLAM during Whisper |
| Odom scaling | `getRPM` returned 0.0 between updates | **holds `last_rpm_`** | 2.6× over-drive / under-report bug |

---

## 13. Open Issues & Parking Lot

The single registry of deferred work is the memory file `PARKING_LOT.md` (not in the repo — in the assistant's memory store). Headline items:

- **Docking finish:** prove `contact=3` via PUSH and/or printed throat rails; solve funnel-entry via **Nav2 on-axis staging** (not a reverse-time cross-track servo).
- **Charging reliability:** don't run heavy load on the dock expecting net charge; add load-shed-when-docked + full-charge hysteresis (#5) + SSR-latch safety audit (#7).
- **Return-to-dock logic:** brain subscribes `/battery/state`, fires a nav goal at a comfortable threshold (never let it go flat — a flat pack can't be dock-recovered).
- **IMU decision:** finalise BNO055-IMUPLUS vs Orbbec-336 IMU as primary heading, by objective test.
- **Drive polish:** ~8 % L/R steady-state imbalance (right weaker) → small `MOTOR2_TRIM`; PID throttle retune (sluggish ramp, was masked by the getRPM bug); low-speed startup yaw transient.
- **Full-bringup docking integration** + rogue `/cmd_vel` publisher audit + agent overlay sourcing.

---

## 14. Quick File Index

| Want to look at… | Path |
|---|---|
| Original objectives & constraints | [`CLAUDE.md`](../CLAUDE.md) |
| Older narrative design doc | [`docs/JUPITER_ROBOT_DESIGN.md`](JUPITER_ROBOT_DESIGN.md) |
| USB / DMA topology (authoritative) | [`docs/THOR_USB_DMA.md`](THOR_USB_DMA.md) |
| Current docking sensing | [`src/jupiter_nodes/src/dock_reflector.cpp`](../src/jupiter_nodes/src/dock_reflector.cpp) |
| Current docking control | [`src/jupiter_nodes/src/dock_aligner.cpp`](../src/jupiter_nodes/src/dock_aligner.cpp) |
| Firmware config / tuning | [`firmware/esp32/include/jupiter_config.h`](../firmware/esp32/include/jupiter_config.h) |
| Nav2 params | [`src/jupiter_bringup/config/nav2_params.yaml`](../src/jupiter_bringup/config/nav2_params.yaml) |
| EKF params | [`src/jupiter_bringup/config/ekf_odom.yaml`](../src/jupiter_bringup/config/ekf_odom.yaml) |
| Daily launch | [`src/jupiter_bringup/launch/jupiter_bringup_full.launch.py`](../src/jupiter_bringup/launch/jupiter_bringup_full.launch.py) |
| Nav-only (S2E) launch | [`src/jupiter_bringup/launch/navigation_s2e.launch.py`](../src/jupiter_bringup/launch/navigation_s2e.launch.py) |

---

*Snapshot compiled 2026-07-27 from the live `~/jupitercpp_ws` tree, git history, and prior design notes. Subsystem states reflect that date; verify against the code before relying on any single line.*
