# ACTIVE TASK — Reverse Docking (handover brief)

**Date:** 2026-08-05 · **Status:** ❌ not working · **Attempts:** 4 generations, ~10 live runs over 2 sessions
**Full project context:** [`JUPITER_FULL_STATUS_2026-08-05.md`](JUPITER_FULL_STATUS_2026-08-05.md)

> **Read §1 before proposing anything.** The obvious first move on this problem — tune the controller
> gains — has already been done exhaustively and is measurably a dead end. Starting there will waste
> days of real hardware time.

---

## 1. ⭐ The decisive measurement — read this first

**This drivetrain has essentially no steering authority while reversing at crawl speed.**

Measured live, 2026-08-05: a **saturated** steering command (`angular.z = 0.20 rad/s`, the controller
cap) held for **2+ seconds** at `linear.x = -0.14 m/s` produced **no observable rotation**, while the
robot's lateral drift continued to grow.

**Why:** reversing turns the rear *trailing* casters into *leading* casters — classically unstable. The
available wheel-speed differential (±~3 cm/s at docking speed) cannot overcome the side-force of the
loaded casters. Drift direction is **random left or right**, depending on how the casters happen to be
sitting when motion starts — which is why it is not a trim or calibration issue.

**Consequence:** *every* control law that steers while moving is invalid here, regardless of gains.
That covers everything in §3.

**What the robot DOES do reliably** (proven in every single run):
1. **Rotate in place** to **±1°** — min-speed 0.15 rad/s + deadband + 0.3 s settle.
2. **Reverse dead straight open-loop** — teleop at 0.21 m/s with `angular.z = 0` gives a flat gyro
   (±0.002 rad/s). Confirmed both by IMU and by eye.

Any viable controller must be built from **only these two primitives**, or must stop steering
altogether and solve the problem mechanically (§5).

---

## 2. The task

Jupiter must **reverse** onto a charging dock and seat **both** rear proximity sensors
(`/dock/contact == 3`), which is what triggers the IR handshake that closes the dock's SSR and
energises the pogo pins.

Reverse (not nose-first) is **deliberate doctrine**: Jupiter faces the room while charging so it can
keep serving users. Nose-first would put the casters back in their stable trailing configuration and
make this far easier — but it changes what the robot *is*. Do not switch without asking Logan.

**Best result achieved:** `contact=2` (one prox seated). **Most runs:** `contact=0` — jammed off-centre
at the funnel throat, aborted.

---

## 3. ❌ DISPROVEN — do not re-attempt

### Architectural approaches
| Gen | Approach | Why it failed |
|---|---|---|
| 1 | IR beacon balance (dock→robot) | IR balance is **flat far out** (both receivers saturate); cannot resolve lateral until the mouth — too late to correct |
| 2 | **AprilTag** (`dock_approach`, `dock_ir`, later `opennav_docking`) | Good (x,z) but **noisy yaw** → arrived angled. `opennav_docking`'s opaque `external_detection_rotation` convention caused consistent misalignment |
| 3 | **Single horizontal retro strip** + square-then-reverse | Centroid fine, but the PCA line-fit **angle balloons below ~0.35 m** (a 250 mm strip subtends ~65° up close — fits an arc, not a line) |
| 4 | Single strip + **frozen IMU heading hold** for the last 0.5 m | ⭐ **The architectural error.** It controlled *squareness to the dock face*, then froze a *compass* heading. It never targeted the **dock centre**. "Square ≠ centred" — the robot can be perfectly square and still 6 cm off the centreline; the frozen heading then locks that error in and integrates it into a lateral slide |

### Control laws (all inside Gen 3/4, all failed for the §1 reason)
| Attempt | Result |
|---|---|
| P-loop, `kp_heading 1.2` | ±10° **limit cycle** (~150–250 ms loop delay on an integrator plant) |
| Detuned P, `kp 0.5` | Oscillation gone → replaced by a steady ~6° droop → **19 cm** lateral drift |
| **Gyro-rate damping** (`k_damp`, implemented, sign verified) | Stable and correct, but does **not** fix the drift — the drift is slow, damping only opposes fast rates |
| `kp 0.9` + damping | Best heading achieved (±5°, self-recovering) — still arrived **6.5 cm off-centre** |
| Bolted-on **cross-track** trim | **Unstable** — overshot and banked left. (This had already failed and been removed in an earlier era; re-adding it repeated the failure) |
| Continuous **pure pursuit** | Same failure mode |
| Speed raise 0.08 → 0.14 m/s | Correctly escaped wheel stiction (below ~0.08 the commands don't execute at all) but did not fix centring |

**If a proposal reduces to "adjust a gain, deadband, lookahead or speed and retry" — it is in this table.**

---

## 4. ✅ PROVEN — keep, do not rewrite

### The two-strip reflector detector — `src/jupiter_nodes/src/dock_reflector.cpp`
This part is **solved**. Two narrow **vertical** retro-reflective strips (20 mm × 125 mm) mounted
**250 mm apart**, centred on the dock's pogo centreline.

- **Dock centre ●** = midpoint of the two cluster centroids → ~mm precision
- **Dock angle** = perpendicular to the known 250 mm baseline → **σ ≈ 0.1–0.25°**, and it *improves*
  as the robot closes in (more beams per strip), which is the exact inverse of the old single-strip
  failure
- **Vertical** strips make the lateral reading **height-invariant** — immune to scan-plane pitch/vibration
- **Free false-positive rejection**: any cluster pair whose separation ≠ ~250 mm is rejected

**Live verification:** measured baseline **256 mm** (vs 250 expected); `skew` **rock-steady at +5.5°**
where the old single-strip detector flickered between −13° and +21°. Valid continuously from ~0.9 m to
the throat.

Interface (unchanged from the old detector, so controllers are drop-in):
```
/dock/reflector  std_msgs/Float32MultiArray
  [0] valid  [1] along_x m  [2] lateral_y m (+left)  [3] range m
  [4] bearing rad  [5] skew rad (0 = square)  [6] measured baseline m
  [7] baseline error m  [8] n_points
/dock/reflector_pose  geometry_msgs/PoseStamped   (dock face in base_footprint)
```

### Other working pieces
- **Charging chain** — ESP32 emits a 38 kHz beacon only when both prox are seated **and** battery
  < 16.70 V; the dock Nano closes the SSR after 500 ms of sustained beacon. Pogo pins are dead until
  asked (correct safety posture).
- **Seat sensing** — two inductive prox → `/dock/contact` bitmask (1 = left, 2 = right, 3 = both),
  5-cycle debounce, 1.5 s seat grace during which the robot keeps easing in so the rails can square it.
- **Funnel rails** — reprinted with a 100 mm throat + shorter funnel. Measurably better (the robot now
  reaches the throat instead of wedging at the mouth) but cannot square a 6–8° cocked arrival.

---

## 5. 🔨 BUILT BUT NEVER TESTED — the immediate next step

**`src/jupiter_nodes/src/dock_aligner.cpp`** — "stop-and-re-aim" controller. **Compiles clean and is
deployed on Thor. It has never been run.** Treat its behaviour as unknown.

Design — uses *only* the two proven primitives from §1, like a driver backing a trailer:

```
ACQUIRE  → stable two-strip lock
AIM      → rotate IN PLACE until the REAR points at a carrot ON the dock centreline
           (not "square to the face" — centre first, square second)
CRUISE   → reverse with angular.z EXACTLY 0, monitored on the IMU
           (navigator: each reflector sample refreshes yaw_des = yaw + aim)
           drift > 3° and range > 0.45 m?  → STOP, re-AIM, cruise again
COMMIT   → inside 0.45 m no more stops: straight into the throat
THROAT   → blind firm push (steering off); the rails absorb the residual angle
PUSH     → if only one prox latched, pivot the open corner in about the seated one
SEATED   → both prox
```

Two design points worth preserving:
- **Centre first, square second.** The throat exists to absorb residual *angle*; it can never fix a
  robot that arrives *off-centre*. Every earlier generation had this backwards.
- Each stop-and-re-aim also **settles the casters** — the long-wanted "caster pre-flip", for free.

Proven values are baked in as parameter **defaults** (so a plain `ros2 run` needs no arguments):
`v_reverse 0.14` · `throat_zone 0.28` · `seat_zone 0.34` · `square_omega_min 0.15` ·
`drift_deadband 3°` · `commit_range 0.45`

### How to run it
```bash
# 1. micro-ROS agent (separate overlay, ONE long-lived instance)
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/jupiter_esp32 -b 460800

# 2. S2E LiDAR (Ethernet UDP)
ros2 run sllidar_ros2 sllidar_node --ros-args -p channel_type:=udp \
  -p udp_ip:=192.168.11.2 -p udp_port:=8089 -p frame_id:=base_laser -p scan_mode:=Sensitivity

# 3. rear-facing lidar TF  (z = 0.518 tape-measured, yaw = pi)
ros2 run tf2_ros static_transform_publisher --x 0.035 --y 0 --z 0.518 --yaw 3.14159265 \
  --frame-id base_footprint --child-frame-id base_laser

# 4. detector + controller
ros2 run jupiter_nodes dock_reflector
ros2 run jupiter_nodes dock_aligner

# 5. trigger (robot ~1 m out, dock BEHIND it — the S2E is rear-facing)
ros2 service call /dock/align_start std_srvs/srv/Trigger

# watch: /dock/aligner_state  /dock/reflector  /dock/contact
```

**Stopping it:** `/dock/align_cancel` sets state to ABORT but has been observed to keep re-issuing the
push when the robot is physically wedged. **The reliable stop is killing the `dock_aligner` process** —
that removes the only `/cmd_vel` publisher and the firmware's 400 ms watchdog halts the wheels.
*(Known defect worth fixing: cancel/ABORT should latch a hard zero rather than continue ticking.)*

---

## 6. Candidate paths — ranked by confidence

1. **Widen the funnel mouth (mechanical).** ⭐ *Highest confidence.* Make the dock forgiving rather
   than the robot precise — the vacuum-robot philosophy. A capture mouth that swallows ±5 cm / ±10°
   sidesteps the §1 limitation entirely, and the robot can already arrive within that envelope. Print
   in halves and join if the build plate is the constraint. **Note the current trend is the wrong way:**
   the funnel was recently made *shorter*, which reduces correction runway.
2. **Test the stop-and-re-aim controller** (§5). Free — it is already built and deployed.
3. **Change the casters** — damped, or rigid/steerable — to remove the reverse instability at source.
4. **Nose-first docking** — fixes the physics outright, but conflicts with the face-the-room doctrine.
   Requires Logan's agreement, not an engineering decision alone.

### Open safety item
⚠️ **SSR latch on undock-while-charging.** The directional IR still reaches the dock's TSOP at ≥1.1 m,
so pulling the robot off a live dock can leave the SSR **closed** → pogo pins live at 16.8 V while
undocked. **Interim rule: power the dock off before undocking mid-charge.** The real fix is a strict,
fast emitter cutoff on any prox release (`ir_emit_active` gate in `firmware/esp32/src/firmware.ino`).

---

## 7. Hardware reference

| Item | Value |
|---|---|
| Robot | `jupiter@192.168.0.8` (Thor). Pi 5 audio node at `10.0.0.2`, reachable **via Thor only** |
| S2E LiDAR | Ethernet UDP `192.168.11.2:8089`, scan plane **0.518 m**, rear-facing (yaw = π) |
| ESP32 | `/dev/jupiter_esp32` @ **460800** baud |
| Reflector strips | 2 × vertical, **20 mm × 125 mm**, **250 mm apart**, midpoint = pogo centre |
| Funnel throat | 100 mm (recently reprinted, shorter funnel) |
| Prox sensors | Inductive NPN, rear-left GPIO13 / rear-right GPIO15, `INPUT_PULLUP` (LOW = metal) |
| Battery | 4S Li-ion, 16.8 V full, charge cutoff 16.70 V (**no hysteresis** — known issue) |

**Before any motion test:** have a warm teleop session open as an e-stop, and confirm with Logan that
he is beside the robot.
