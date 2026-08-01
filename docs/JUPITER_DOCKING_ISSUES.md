# Jupiter Robot — Docking System & Issues Log

**Author:** Logan Naidoo <naidoo.logan@gmail.com>
**License:** Apache-2.0
**Snapshot date:** 2026-07-27
**Purpose:** A dedicated record of Jupiter's docking system — **exactly which code is in use and where it lives in `jupitercpp_ws`** — and the full history of docking problems encountered to date, what caused them, and how each was resolved (or where it still stands).

> Companion doc: [`docs/JUPITER_PROJECT_STATUS.md`](JUPITER_PROJECT_STATUS.md) (whole-project overview).

---

## 1. TL;DR — Current Docking System

Jupiter **reverse-docks (caster-first)** onto a contact-charging station, guided by the **S2E LiDAR detecting a retro-reflective strip** on the dock. It does this so the robot keeps **facing the user while charging** (Jupiter is an always-on companion, not a park-and-die trolley).

**Current pipeline:**
```
 S2E LiDAR /scan
      │
      ▼
 dock_reflector  ──►  /dock/reflector (+ /dock/reflector_pose)     [SENSING]
      │
      ▼
 dock_aligner    ──►  /cmd_vel   (state: /dock/aligner_state)      [CONTROL]
   ACQUIRE → SQUARE → REVERSE_IN → PUSH → SEATED / ABORT
      │
      ▼
 ESP32 firmware  ──►  motors + /dock/contact (prox seat) + IR emitter   [ACTUATION + SEAT]
      │  (38 kHz IR "charge-enable" beacon, only when seated + battery not full)
      ▼
 dock_charger (Arduino Nano on the DOCK)  ──►  SSR ──► pogo-pin charge power   [CHARGING GATE]
```

**Current status:** detection ✅, square-to-±1° ✅, gentle-hold reverse arrives square+centred ✅ → seats **one** proximity sensor reliably. **Full both-prox seat (`contact=3`) is NOT yet proven.** See §6.

---

## 2. WHERE THE CODE LIVES (authoritative file map)

All paths are relative to the workspace root `~/jupitercpp_ws`.

### 2.1 In-use (current pipeline)

| Component | File | Role |
|---|---|---|
| **Reflector detector** (sensing) | [`src/jupiter_nodes/src/dock_reflector.cpp`](../src/jupiter_nodes/src/dock_reflector.cpp) | S2E intensity threshold → largest bright cluster → PCA line-fit → dock pose |
| **Aligner** (control) | [`src/jupiter_nodes/src/dock_aligner.cpp`](../src/jupiter_nodes/src/dock_aligner.cpp) | Closed-loop SQUARE → REVERSE_IN → PUSH state machine → `/cmd_vel` |
| **Robot firmware** | [`firmware/esp32/src/firmware.ino`](../firmware/esp32/src/firmware.ino) | Prox seat logic, seat-grace reflex, IR charge-enable emitter, cmd_vel watchdog |
| **Firmware config** | [`firmware/esp32/include/jupiter_config.h`](../firmware/esp32/include/jupiter_config.h) | Prox pins, seat grace, IR emitter, motor/PID/FF constants |
| **Motor driver** | [`firmware/esp32/src/motor.cpp`](../firmware/esp32/src/motor.cpp) | Brake-mode drive (both directions) + static-friction feed-forward |
| **Dock-side charger** | [`firmware/dock_charger/src/dock_charger.ino`](../firmware/dock_charger/src/dock_charger.ino) | Arduino Nano: IR-keyed SSR charge gate ("dead-man") |

> **Note:** there is **no launch file** that starts the current `dock_reflector` + `dock_aligner` pair — they are run with `ros2 run` and triggered by the `/dock/align_start` service (see §7). This is deliberate: docking is iterated against **one long-lived micro-ROS agent** so restarting the agent doesn't churn the ESP32 into a reconnect loop.

### 2.2 Legacy / retired docking code (kept in-tree as record, NOT in the current path)

| File | Era | Why retired |
|---|---|---|
| [`src/jupiter_nodes/src/dock_approach.cpp`](../src/jupiter_nodes/src/dock_approach.cpp) | AprilTag visual approach | Noisy tag yaw → arrived angled |
| [`src/jupiter_nodes/src/dock_ir.cpp`](../src/jupiter_nodes/src/dock_ir.cpp) | IR-beacon lateral steering | IR gave side but no range; brittle |
| [`src/jupiter_nodes/src/dock_range.cpp`](../src/jupiter_nodes/src/dock_range.cpp) | LiDAR wall-range for IR era | Can't measure lateral (featureless wall) |
| [`src/jupiter_nodes/src/vision.cpp`](../src/jupiter_nodes/src/vision.cpp), [`webcam_publisher.cpp`](../src/jupiter_nodes/src/webcam_publisher.cpp) | AprilTag detect + tail webcam | Camera-based docking abandoned |
| [`src/jupiter_bringup/config/docking_server.yaml`](../src/jupiter_bringup/config/docking_server.yaml), [`launch/dock_opennav.launch.py`](../src/jupiter_bringup/launch/dock_opennav.launch.py) | Nav2 `opennav_docking` | Opaque `external_detection_rotation` → misalignment |
| [`launch/dock_simple.launch.py`](../src/jupiter_bringup/launch/dock_simple.launch.py), [`dock_ir.launch.py`](../src/jupiter_bringup/launch/dock_ir.launch.py), [`dock_ir_nodes.launch.py`](../src/jupiter_bringup/launch/dock_ir_nodes.launch.py) | AprilTag / IR launches | Superseded |
| [`firmware/dock_beacon/`](../firmware/dock_beacon) | Old Nano IR beacon | Superseded by `dock_charger` |

---

## 3. Interfaces (current pipeline)

### 3.1 `dock_reflector` — [`dock_reflector.cpp`](../src/jupiter_nodes/src/dock_reflector.cpp)

| Direction | Topic / Type | Notes |
|---|---|---|
| Sub | `/scan` `sensor_msgs/LaserScan` | S2E; **6-bit intensity 0–63** |
| Pub | `/dock/reflector_pose` `geometry_msgs/PoseStamped` | dock face in `base_footprint`; orientation yaw = outward normal (points at robot) |
| Pub | `/dock/reflector` `std_msgs/Float32MultiArray` | self-describing debug vector (below) |

**`/dock/reflector` array:** `[0] valid  [1] along_x(m)  [2] lateral_y(m,+left)  [3] range(m)  [4] bearing(rad)  [5] skew(rad, 0=squared)  [6] strip_len(m)  [7] fit_rms(m)  [8] n_points`

**Key params** (defaults): `intensity_min 40` (strip saturates 63, room ≤33), `range_min 0.10`, `range_max 4.0`, `max_gap_deg 2.0`, `min_points 6`, `max_fit_rms 0.03`.

> **The metric that matters:** the aligner computes **`nyaw = wrap(skew + bearing + π)`** = the dock-face heading error (0 = square). It nulls **`nyaw`**, *not* raw `skew` (skew is rotation-invariant position-on-normal, not heading).

### 3.2 `dock_aligner` — [`dock_aligner.cpp`](../src/jupiter_nodes/src/dock_aligner.cpp)

| Direction | Interface / Type | Notes |
|---|---|---|
| Sub | `/dock/reflector` `Float32MultiArray` | dock pose from detector |
| Sub | `/imu/data` `sensor_msgs/Imu` | gyro heading for the close-in hold |
| Sub | `/dock/contact` `std_msgs/UInt8` | prox bitmask: 1=left, 2=right, 3=both |
| Pub | `/cmd_vel` `geometry_msgs/Twist` | motion command |
| Pub | `/dock/aligner_state` `std_msgs/String` (transient-local) | current state name |
| Service | `/dock/align_start` `std_srvs/Trigger` | begin docking |
| Service | `/dock/align_cancel` `std_srvs/Trigger` | stop / abort |

**State machine:** `IDLE → ACQUIRE → SQUARE → REVERSE_IN → PUSH → SEATED / ABORT`.

**Key params** (defaults, all overridable — house rule = no hardcoded config):

| Param | Default | Meaning |
|---|---|---|
| `control_hz` | 20.0 | loop rate |
| `v_reverse` | 0.05 | reverse creep speed (m/s) |
| `max_angular` | 0.25 | angular cap (rad/s) |
| `kp_heading` | 1.2 | reflector heading gain (SQUARE + far-band) |
| `heading_sign` | 1.0 | rotation direction (verified) |
| `square_tol_deg` | 1.0 | squared when |nyaw| ≤ this |
| `square_omega_min` / `_max` | 0.15 / 0.30 | **min-speed floor** (breaks floor scrub) + cap — the anti-oscillation recipe |
| `square_settle_s` | 0.3 | hold in-band before locking heading |
| `square_timeout_s` | 25.0 | abort if it never squares |
| `square_only` | false | SQUARE then stop (sign-check mode) |
| `reflector_trust_range` | 0.35 | below this range → IMU gyro-hold heading (reflector angle unreliable close-in) |
| `kp_yaw` | 0.8 | IMU yaw-hold gain |
| `lateral_gain` / `lat_sign` / `max_lateral_corr` | **0.0** / 1.0 / 0.05 | cross-track hook — **OFF by default** (see §5, the failed detour) |
| `seat_contact_mask` | 3 | both prox = seated |
| `seat_zone` | 0.30 | only seat-terminate this close (m) |
| `seat_settle_s` | 2.0 | no-progress + contact this long → enter PUSH |
| `stuck_abort_s` | 4.0 | no-progress + NO contact near seat → abort (arrived skewed) |
| `stall_eps` | 0.008 | progress smaller than this = stalled (m) |
| `seat_range_floor` | 0.15 | hard stop even with no contact (m) |
| `push_omega` / `push_v` / `push_timeout_s` | 0.22 / 0.03 / 4.0 | **PUSH** seat-closer (built 2026-07-27, **untested**) |
| `overall_timeout_s` | 90.0 | global reverse timeout |

---

## 4. Docking Evolution — the four generations (issues that killed each)

Jupiter has been through **four** docking architectures. Each was abandoned for a concrete reason.

### Gen 1 — AprilTag visual approach  *(retired)*
- **Code:** `dock_approach.cpp`, `vision.cpp` (VPI tag36h11), `webcam_publisher.cpp`; launch `dock_simple.launch.py`.
- **How:** camera sees tag → solvePnP 6DOF → drive to a point on the tag normal → stop at standoff.
- **Milestones reached** (git): dead-centre approach confirmed, pogo-pin contact confirmed with guide fins (`target_distance` 0.40→0.22 m).
- **Issues that killed it:**
  - Single-tag **yaw was noisy** → the robot arrived **angled** even when position was good.
  - solvePnP **flip ambiguity** at range required rejection logic.
  - A first **polar (ρ,α,β) controller went singular** as ρ→0 (spinning, tag loss) → replaced by a Cartesian servo.
  - Close-range pose too jittery to steer → had to lock heading and drive blind the last stretch.

### Gen 2 — IR beacon + LiDAR wall-range  *(retired)*
- **Code:** `dock_ir.cpp` (lateral from IR heartbeat balance), `dock_range.cpp` (distance from S2E wall), `firmware/dock_beacon/` (Nano IR beacon); launch `dock_ir*.launch.py`.
- **How:** IR gives **side** (left/right of centre), LiDAR wall-fit gives **range**; guide rails do the squaring.
- **Issues that killed it:**
  - IR gives side but **no distance and no lateral offset** — `dock_range` itself documents it **cannot** measure lateral against a featureless wall.
  - Brittle: IR silence during ESP32 reconnects; baud instability (921600→460800 to stop micro-ROS session drops).
  - Fundamentally **under-sensed** — no clean landmark for both lateral *and* angle.

### Gen 2.5 — Nav2 `opennav_docking`  *(retired)*
- **Code:** `docking_server.yaml`, `dock_opennav.launch.py`.
- **Issue that killed it:** the opaque **`external_detection_rotation`** frame convention caused **consistent misalignment**; also a **backward runaway** when the ESP32 latched the last `cmd_vel` on node silence (→ drove the cmd_vel watchdog fix). Black-box conventions weren't debuggable in reasonable time.

### Gen 3 — S2E LiDAR retro-reflective strip, reverse-in  *(CURRENT)*
- **Code:** `dock_reflector.cpp` + `dock_aligner.cpp` + firmware prox/seat/IR + `dock_charger.ino`.
- **Why this design wins:** the LiDAR is **already always-on** for nav; the retro-reflective strip **saturates intensity (63)** while the room stays ≤33, so it's isolated by a simple threshold and **re-found every cycle even if the dock moves**. The strip's centroid gives **lateral**, its line-fit gives **face angle** — the two things the IR/wall era couldn't measure. See §5–6 for its own issue log.

---

## 5. Gen-3 Issue Log (the current system)

Chronological problems on the current LiDAR-reflector reverse-dock, with resolution status.

| # | Issue | Root cause | Resolution | Status |
|---|---|---|---|---|
| 1 | **"Jacket decoy" false positive** — detector locked a bright 63-saturated cluster 1.29 m front-right and I declared success | A hi-viz **safety jacket** on a couch is also retro-reflective; it was in the front sector, not the dock (rear) | Look in the **rear** sector; move stray retro-reflective objects; the strip is re-found by geometry each cycle | ✅ resolved |
| 2 | **Strip not detected / read matte** | LiDAR **scan plane height** was wrong in TF (0.5325 m) vs the real **0.518 m** → the beam missed the 50 mm-tall strip | Re-measured lidar at **518 mm**, matched strip centre, fixed TF | ✅ resolved (commit `a3e2aef`) |
| 3 | **13 s reverse breakaway stall** — reverse crept for ~13 s before moving | `motor.cpp` drove reverse in **slow-decay** mode (electrically weak) | **Brake-mode reverse** (PWM the direction pin) | ✅ 13 s→1.3 s (commit `bd2e4f2`) |
| 4 | **Continuous feed-forward over-steer** — fine reverse over-rotated off the dock | A **continuous** static-FF kick slammed tiny heading corrections | **Breakaway-only FF** (apply only while a wheel is stalled + commanded to move) | ✅ resolved (commit `ee426ee`) |
| 5 | **⭐ Robot drove ~2.5 m on a 1 m command; all odom-nav poisoned** | **`getRPM()` returned 0.0** between its 100 ms recompute calls; loop runs ~20–26 Hz → PID **over-drove ~2.6×**, odom **under-reported 2.6×** | Hold `last_rpm_` between updates | ✅ **landmark fix** (commit `49e2fe5`) |
| 6 | **Wrong conclusion: "the BNO055 is lying"** (read 2.5× the odom yaw) | It was the **odom** that was 2.6× low (issue #5), not the BNO | Corrected after odom calibration — the BNO was closer to truth | ✅ owned/corrected |
| 7 | **Erratic heading while driving** | **BNO055 NDOF** mode uses the **magnetometer**, corrupted by motor magnetic fields | Switched to **IMUPLUS** (gyro+accel, magnet-immune relative yaw) | ✅ flashed (commit `49e2fe5`); verify on HW |
| 8 | **Odometry ~scale error after wheel swap** | `COUNTS_PER_REV` still 1372 (old 65 mm wheels) after 100 mm AGV wheels fitted | Recalibrated to **1290** (tape-measured) | ✅ resolved (commit `49e2fe5`) |
| 9 | **SQUARE oscillation / hunting (±14°)** — in-place squaring never settled | Fixed FF kick over-drove near target; no deadband → overshoot/decay | **Min-rotational-speed floor (0.15) + hard ±1° deadband + 300 ms settle** | ✅ **SOLVED** — converges cleanly, best square to date |
| 10 | **Cross-track (funnel-from-offset) control unstable** | A proportional cross-track + heading servo **during the reverse** is a coupled, under-damped problem → growing oscillation (lat +0.08→−0.09→+0.15) | **Removed** it. Correct layer = **Nav2 on-axis staging**, then square + reverse straight. `lateral_gain` left OFF by default | ⚠️ approach abandoned (see §6) |
| 11 | **Funnel entry from an off-axis staging** — ~8–10 cm lateral offset overshoots the funnel (lands "half in, half out" on one side) | Gentle-hold reverse drives **straight** (by design) and does not correct lateral | Belongs to **staging accuracy** (Nav2) + funnel width, NOT the reverse controller | ❌ **open** (architectural: needs Nav2 staging) |
| 12 | **Last-cm throat skew — one prox seats, not both (`contact=1`)** | ~4° mechanical deflection at throat entry engages one corner | Two candidate finishers → PUSH (#13) and/or printed throat rails | ❌ **open** |
| 13 | **PUSH seat-closer** — pivot the un-seated rear corner in until both latch | (fix for #12) derived sign: left-only→CW, right-only→CCW; self-terminates when firmware sees both prox | Built 2026-07-27 in `dock_aligner.cpp` | 🚧 **built, UNTESTED** |
| 14 | **Test run failed at SQUARE** — wheels didn't rotate for 12 s, then reflector latched a spurious object 1 m to the side | Undetermined from remote: drivetrain-not-commanding **or** reflector-going-spurious. Battery healthy (16.16 V), agent up | Needs **eyes on the robot** to see if wheels physically turn during SQUARE | ❓ **needs on-site diagnosis** |

⭐ Issue #5 is the single most consequential bug of the project — it silently poisoned every odom-based distance for months and produced the misleading "drives too fast" symptom.

---

## 6. Where Gen-3 Docking Stands (honest)

**Proven:**
- ✅ Reflector detected continuously across the whole approach.
- ✅ **SQUARE to ±1°** with no oscillation (the min-speed + deadband + settle recipe).
- ✅ **Gentle-hold reverse** delivers the robot **square and centred** from a good (on-axis) staging → seats **one** proximity sensor reliably. This is the best docking achieved to date.

**Not solved:**
- ❌ **Both-prox seat (`contact=3`)** — a ~4° throat-entry deflection catches one corner. Fix path: the **PUSH** (built, untested) and/or **3D-printed throat-extension guide rails**.
- ❌ **Funnel entry from an off-axis staging** — the reverse creep does not (and, per the architecture, should not) correct lateral offset. This belongs to **Nav2 staging** placing the robot on the dock's normal axis first. The attempted reverse-time cross-track servo (#10) was unstable and was removed.

**Correct target architecture (for whoever continues this):**
```
Nav2  →  drive to STAGING pose ON the dock's normal axis  (this handles LATERAL offset — stable drive-to-pose)
dock_aligner SQUARE  →  null heading to ±1°               (proven)
dock_aligner REVERSE_IN  →  straight gentle-hold creep    (proven, arrives square+centred)
dock_aligner PUSH / throat rails  →  seat the 2nd prox    (the remaining gap)
firmware/dock_charger  →  IR handshake → SSR → charge
```
The lesson: **do not try to correct lateral offset during the reverse.** Correct it at the staging/navigation layer, then reverse straight.

**Uncommitted:** the SQUARE recipe + PUSH state in `dock_aligner.cpp` are **built clean but not committed** (pending a proven `contact=3` run). `git status` also shows legacy `docking_server.yaml` / `dock_opennav.launch.py` modified.

---

## 7. Firmware Seat & Charge Logic

### 7.1 Proximity seat sensing — [`firmware.ino`](../firmware/esp32/src/firmware.ino) + [`jupiter_config.h`](../firmware/esp32/include/jupiter_config.h)
- Two inductive prox sensors (LJ18A3-8-Z/BX, NPN): **rear-left GPIO13, rear-right GPIO15**. `INPUT_PULLUP`: **LOW = metal/contact**.
- Publishes **`/dock/contact`** bitmask: `1`=left, `2`=right, `3`=both.
- **Seat debounce:** `both-LOW` for `PROX_DEBOUNCE_CYCLES (5 × 20 ms)` → `dock_seated`.
- **Seat-grace reflex** (the important interaction the aligner relies on):
  - After **first** contact, keep easing in for `CONTACT_SEAT_GRACE_MS (1500)` so the rails can square the robot, then stop.
  - **Only `target_linear_velocity < 0` (reverse) is zeroed** on `both_contact || grace_expired`.
  - **`target_angular_velocity` is zeroed only when `dock_seated` (BOTH prox).** → the aligner's PUSH rotation is free to act with one prox, and is auto-killed the instant both confirm. **This is by design.**

### 7.2 Charge-enable IR emitter — robot side ([`firmware.ino`](../firmware/esp32/src/firmware.ino))
- TSAL6400 on **GPIO4**, 38 kHz burst packets.
- Fires **only** when `dock_seated (both prox)` AND `battery < 16.70 V`. → the dock's charge is a **dead-man**: no seat, no beacon, no power.

### 7.3 Dock-side charge gate — [`dock_charger.ino`](../firmware/dock_charger/src/dock_charger.ino) (Arduino Nano)
- TSOP/VS1838B receiver on **D2 (INT0)**; SSR-25DD control on **D5**.
- **Power-on state: SSR OFF (dead dock, always).**
- SSR **ON** only after **`ON_CONFIRM_MS (500 ms)`** of sustained beacon activity; SSR **OFF** fast on **`OFF_TIMEOUT_MS (300 ms)`** of silence. A stray remote press can't sustain long enough.
- Net: pogo pins are **dead until the seated robot asks** — the correct safety posture (no exposed live 16.8 V).

### 7.4 Charging-chain issues (parked)
- ⚠️ **SSR-latch on undock-while-charging:** the directional IR still reaches the dock TSOP at ≥1.1 m, so pulling the robot out mid-charge may leave the SSR closed → pins stay live when undocked. **Interim: power the dock off before undocking mid-charge.** Needs a strict/fast emitter cutoff on any prox release.
- ⚠️ **Full-charge cutoff has no hysteresis** (hard 16.70 V) → SSR can chatter near full under load.
- ⚠️ **Flat-on-dock:** the robot has twice drained flat *while docked running full bringup* — charge-in didn't sustain the always-on load. And a truly **flat pack can't be dock-recovered** (ESP32 needs pack power to fire the IR handshake that enables the pins → chicken-and-egg → bench charge required). Mitigation = **return-to-dock at a comfortable threshold** + shed heavy load when docked.

---

## 8. How to Run / Test the Current Docking

**Prereq:** one long-lived micro-ROS agent (do **not** restart it between iterations), S2E LiDAR up, TF `base_footprint→base_laser` publishing (z = 0.518, yaw = π).

```bash
# 1. LiDAR (Ethernet UDP)
ros2 run sllidar_ros2 sllidar_node --ros-args \
  -p channel_type:=udp -p udp_ip:=192.168.11.2 -p udp_port:=8089 \
  -p frame_id:=base_laser -p scan_mode:=Sensitivity

# 2. base_laser TF (rear-facing mount)
ros2 run tf2_ros static_transform_publisher \
  --x 0.035 --y 0 --z 0.518 --yaw 3.14159265 \
  --frame-id base_footprint --child-frame-id base_laser

# 3. Detector + controller
ros2 run jupiter_nodes dock_reflector
ros2 run jupiter_nodes dock_aligner --ros-args -p kp_yaw:=0.5 -p v_reverse:=0.08

# 4. Trigger
ros2 service call /dock/align_start std_srvs/srv/Trigger

# Watch:  /dock/aligner_state   /dock/reflector   /dock/contact
#   square_only:=true  → SQUARE then stop (heading-sign check, no reverse)
#   cancel:            ros2 service call /dock/align_cancel std_srvs/srv/Trigger
```

**Staging:** place the robot **~1 m out, rear toward the dock, roughly on the dock's normal axis, a few degrees off square.** The lateral offset must be small (funnel-width-limited) until Nav2 on-axis staging exists — this is the open item (§5 #11).

---

## 9. Commit Trail (docking, most recent first)

```
49e2fe5  firmware: fix getRPM 0.0-return (2.6x speed/odom error) + BNO IMUPLUS + CPR recal   ⭐
ee426ee  Breakaway-only feed-forward + aligner stuck-abort; gentle-hold reverse proven
ac0b7d2  firmware: static-friction feed-forward + diff-drive rotation fix
bd2e4f2  firmware: reverse brake-mode + PID anti-windup (fix 13s reverse stall)
bfae499  Add dock_aligner (Stage A: reverse-in controller)
af9f1e0  Add dock_reflector node: retro-reflective LiDAR dock detection
0918b9d  esp32: cmd_vel watchdog + dock proximity sensors + charge-enable IR emitter
f9a5a8a  firmware: dock_charger (Nano) — IR-keyed SSR charge gate, bench-proven
d4b2410  docking: adopt Nav2 Docking Server (opennav_docking)   [Gen 2.5, retired]
0624f74  docking: AprilTag-guided two-phase controller           [Gen 1]
...      (dock_ir / IR-beacon era, Gen 2)  963af94, 756a32d, ce7b915, etc.
```
Uncommitted (2026-07-27): `dock_aligner.cpp` SQUARE recipe + PUSH state (built clean, awaiting a proven `contact=3` test).

---

*Compiled 2026-07-27 from the live `~/jupitercpp_ws` tree and git history. Status lines reflect that date — verify against the code before relying on any single statement.*
