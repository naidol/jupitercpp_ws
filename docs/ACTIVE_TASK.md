# ACTIVE TASK — Reverse Docking (handover brief)

**Date:** 2026-08-05, updated 2026-08-10
**Full project context:** [`JUPITER_FULL_STATUS_2026-08-05.md`](JUPITER_FULL_STATUS_2026-08-05.md)

> ## ✅ SOLVED 2026-08-10 — `dock_aligner_v3` docked autonomously (`contact=3`)
>
> ```
> seg 1  ARC 0.250 m  +2.8 deg  DONE
> seg 2  ARC 0.250 m  +6.7 deg  DONE
> seg 3  ARC 0.146 m  -0.6 deg  DONE   (offset corrected to -0.000)
> seg 4  COMMIT 0.181 m @40rpm  -> both prox seated MID-SEGMENT
> final: range 0.196 (seated 0.1962), lateral -0.005, skew -1.4 deg
> ```
>
> Four segments, zero stalls, both proximity sensors. **`contact=3` gates the ESP32 IR beacon,
> which closes the dock SSR — so the robot can now put itself on charge.**
>
> **What made it work, in order of contribution:**
> 1. **POSITION control instead of velocity** (Logan's proposal). Steering as a differential
>    DISTANCE is geometry; it does not care that the velocity loop takes 4-9 s to settle — which
>    is exactly what killed V1/V2, whose corrections expired before the firmware executed them.
> 2. **ARC segments instead of in-place pivots** — from Logan's correction that this chassis has
>    ONE rear caster, not two. A pivot demands a 90 deg caster swivel from rest and stalled in
>    100 % of attempts; an arc asks for ~2-9 deg and completed 10 of 11.
> 3. Damped arc loop (`arc_gain 0.6`, `look_min 0.35`) so the aim error stops amplifying close in.
> 4. Calibrated `WHEEL_SEPARATION` (0.3586, measured) so commanded angles are real.
>
> **NOT established — this is ONE dock from a good staging pose (~1.0 m, roughly centred):**
> - **Repeatability** across varied staging (off-axis, skewed, caster wherever)
> - **`cmd_vel` override** (the e-stop path) — still unverified
> - The wheels-down `±90°` IMU acceptance test in §7 — never run
> - `SEAT_NUDGE` — written but never executed; the approach was clean enough not to need it
>
> §3's "disproven" list remains valid **for velocity control** and is kept as the record of why
> that path was abandoned. **§1 is still the most useful part of this document** — but note its
> mechanism was superseded: see the correction below.

---

## 1. ⭐ The decisive measurement — read this first

**The ESP32 velocity PID takes 4–9 seconds to reach a commanded wheel speed. A docking approach
lasts ~7 seconds. Every steering correction was abandoned before the firmware could execute it.**

Measured 2026-08-08 (`bench_wheel_tracking_long.py`, straight reverse, 10 s dwell, data in
[`bench_wheel_tracking_long.csv`](bench_wheel_tracking_long.csv)):

| Speed | 2–3 s | 8–10 s |
|---|---|---|
| 0.04 m/s | 51 % | **88 %** |
| 0.06 m/s | 44 % | **91 %** |
| 0.08 m/s | 43 % | **108 %** |
| 0.12 m/s | 66 % | **98 %** |

The loop **does** converge — it is **under-gained in the low-RPM docking regime**, not broken.
`K_P = 5` is scaled for the full 0–214 RPM range (`PWM_MAX = 1023`), so a 5 RPM error produces
~2.4 % duty against a wheel needing ~15–20 % to break friction; the slow integral then does all the
work. A steering correction *is* a change in commanded wheel speeds, so it inherits that 4–9 s lag.
This is why saturated commands produced no rotation, why the measured dead time was 2–3 s, and why
both wheels sat at the *mean* of their two commanded speeds — the differential never had time to
develop.

> **⚠️ CORRECTION — two earlier explanations here were wrong. Do not act on them.**
>
> 1. **"The drivetrain has no steering authority; leading casters overpower the differential"**
>    (Claude, 2026-08-05). The *observation* was right — saturated command, no rotation — but the
>    cause was not caster physics. It was the firmware still ramping. This wrongly framed the
>    problem as a mechanical wall requiring a wider funnel or different casters.
> 2. **"The PID is structurally broken / cannot deliver differential"** (Copilot,
>    [`DOCKING_HANDOVER_2026-08-08.md`](DOCKING_HANDOVER_2026-08-08.md) §1, §4). Right layer,
>    wrong verdict: that report measured t = 2–3 s of a 3 s step and read a transient as steady
>    state. Its numbers reproduce exactly; only the interpretation was wrong. Its §5.2 option 2
>    (coupled velocity + yaw-rate rewrite) is **not warranted**. Note also `PWM_MAX = 1023`, not
>    255 — every duty figure in its §5 is 4× off.
>
> **The common failure:** three sessions diagnosed the symptom at the wrong layer — controller
> gains, then caster mechanics, then firmware structure — before anyone held a test long enough to
> watch the loop converge. When a plant "won't respond", measure how long it takes to respond
> before concluding it can't.
>
> 3. **⭐ 2026-08-10 — "leading caster instability / shimmy" was ALSO wrong** (Claude, repeatedly).
>    This chassis has **ONE rear caster, not two** (Logan's correction; confirmed in
>    `firmware.ino`, "rear caster config"). It sits **180 mm behind the drive axle**, so an
>    **in-place rotation requires it to swivel 90° from rest under load** — it behaves as a locked
>    skid, and the segment stalls. Turning **while moving** only asks for `atan(0.180/R)`:
>
>    | manoeuvre | caster swivel | outcome |
>    |---|---|---|
>    | straight drive | 0° | ✅ always completed |
>    | arc, R = 3.6 m | ~3° | ✅ |
>    | arc, R = 0.8 m | ~13° | ✅ |
>    | **in-place pivot** | **90°** | ❌ stalled every attempt |
>
>    This retires a long-running misdiagnosis: the "caster drift" blamed for months, the ±2°
>    rotation scatter, and the pivot stalls are **one phenomenon** — a single trailing caster being
>    asked to swivel. Not instability, not shimmy. **Design rule: never command an in-place pivot
>    on this chassis. Turn while moving.**

**Consequence:** the blocker is a **firmware gain fix** (§6), not a control-law redesign and not
mechanical. The control laws in §3 failed because they were all issuing corrections shorter than
the plant's response time — they are not invalidated on their own merits, but none should be
re-tested until the firmware converges in ≲1.5 s.

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

> ⚠️ **It is NOT on `main`.** Because it is untested, it lives only on the branch
> **`transition/claude-to-copilot`** (commit `7f08dd3`). `main` still carries the *previous*
> SQUARE-then-REVERSE aligner — the known-failing Gen-4 version described in §3. To work on the new
> one: `git checkout transition/claude-to-copilot`. Merge it to `main` only once a live run proves it.
>
> ⚠️ **Thor's working copy is ahead of `main`.** The robot already has the stop-and-re-aim source
> deployed and built. So `ros2 run jupiter_nodes dock_aligner` on the robot **right now runs the new
> untested controller**, regardless of which branch is checked out on the hub. Before any test,
> confirm which version is actually deployed:
> `ssh jupiter@192.168.0.8 'grep -c STOP-AND-RE-AIM ~/jupitercpp_ws/src/jupiter_nodes/src/dock_aligner.cpp'`
> (`3` = new controller, `0` = old). Rebuild after any rsync — see §5 "How to run it".

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

*(Re-ranked 2026-08-08 after the §1 correction. The mechanical options were ranked first when the
problem was believed to be a caster/physics wall; it is a firmware gain problem, so they drop.)*

1. **Fix the ESP32 velocity loop.** ⭐ *Now the only thing blocking progress.* Instrument
   `moveBase()` (log `req_rpm, current_rpm, pid_out, integral, duty`), then apply **continuous
   Coulomb friction feed-forward** — `MOTOR_FF_STATIC` already is this, but is gated to fire only
   below `MOTOR_FF_RELEASE_RPM = 4`, so it assists breakaway then abandons the wheel — **plus a
   higher `K_P`** for low-RPM authority. One change at a time; re-run
   `bench_wheel_tracking_long.py` after each. Requires a firmware flash → Logan's explicit approval.
   **Acceptance:** ≥ 90 % of commanded within **1.5 s**, and delivered wheel ratio ≥ 1.6 when 1.96
   commanded (re-run Phase B of `bench_wheel_breakaway.py` at 3 s dwell).
2. **Then re-test a controller.** Once the loop is fast, the §3 laws deserve a fresh look — they
   failed against a plant that could not answer them, which is not the same as failing on merit.
   The stop-and-re-aim controller (§5) is built, deployed and free to try; `dock_aligner_v2`'s
   pursuit law is the more capable option if the plant can now steer.
3. **Widen the funnel mouth (mechanical).** Still worthwhile as tolerance — making the dock forgiving
   rather than the robot precise is sound regardless. But it is no longer the primary fix, and the
   recent reprint went the *wrong* way (shorter funnel = less correction runway).
4. **Caster / nose-first changes.** Deprioritised — the reverse instability they target is not the
   root cause. Nose-first also conflicts with the face-the-room doctrine and is Logan's call, not an
   engineering decision alone.

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
