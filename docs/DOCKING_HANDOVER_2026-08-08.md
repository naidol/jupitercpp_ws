# Docking Challenge — Session Handover Report

**Date:** 2026-08-08
**Author:** GitHub Copilot (agent session on hub, `/home/logan/jupitercpp_ws`)
**Audience:** Claude Code (next agent session) and Logan
**Status:** Docking is **BLOCKED on ESP32 motor-control firmware**. No further
controller tuning on the Thor side will fix it. Evidence below.

> ## ⚠️ SUPERSEDED IN PART — read [§8 Addendum](#8-addendum-2026-08-08-later--the-pid-converges-it-is-slow-not-broken) FIRST
>
> A follow-up bench run (same day, longer dwell) **reproduced this report's numbers exactly**
> but showed they were a **transient, not a steady state**. The ESP32 PID **does** converge to
> ~90–108 % of commanded wheel speed — it just needs **4–9 s** to get there.
>
> - ✅ **Still valid:** the blockage is in the ESP32 motor-control layer, not perception or the
>   docking controller. That core finding stands and is confirmed.
> - ❌ **Superseded:** "the plant cannot steer" / the implication that the loop is structurally
>   broken. It is **under-gained in the low-RPM regime**, not broken.
> - ❌ **Do not action §5.2 option 2** (coupled velocity + yaw-rate rewrite) — not warranted.
> - ⚠️ **Arithmetic error throughout §5:** `PWM_MAX` is **1023** (`PWM_BITS = 10`), not 255.
>   Every duty percentage in §5 is 4× off (`MOTOR_FF_STATIC = 200` is **19.5 %** duty, not 78 %).
>
> Everything else below — the trials, the watchdogs, the bench harness, the instrumentation
> advice — remains accurate and useful.

---

## 1. Executive summary

The reflector-guided reverse-docking pipeline (S2E LiDAR reflector detection →
`dock_aligner` / `dock_aligner_v2` → `/cmd_vel` → ESP32 diff-drive firmware) fails
not in perception and not in the docking controller, but in the **lowest layer**:
the ESP32 per-wheel velocity PID does not deliver commanded wheel speeds, and at
docking speeds it **erases the commanded left/right differential almost completely**.

Two floor tests (no dock, open floor) measured this directly:

- **Open-loop bench** (`scripts/bench_wheel_breakaway.py`): actual wheel RPM is
  50–70 % of commanded at *every* speed, steady-state, despite K_I = 5. A commanded
  1.83:1 wheel ratio (v = 0.06, az = ±0.10) was delivered as **1.06:1** — zero
  steering authority at original docking speed.
- **Closed-loop steering test** (`scripts/bench_steer_closedloop.py`) at the best
  operating point the plant has (v = 0.12 m/s, az = ±0.22, the exact steering law
  deployed in `dock_aligner_v2`): commanded wheel ratio 1.96:1, delivered
  **1.01–1.04**; 2–3 s of *zero yaw response* under fully-saturated command;
  measured yaw-rate gain ≈ **0.01** of commanded. The robot could not hold a
  heading against a 1.27 °/s caster drift with near-full authority. **OVERALL: FAIL.**

Every "mystery" of the four dock trials (yaw diverging opposite to command,
inversions, no-progress stalls) is explained by this: the controller was steering,
the plant was not answering, and passive caster drift did whatever it wanted.

---

## 2. What was changed in V1 (`dock_aligner.cpp`)

V1 is a stage-machine aligner (SQUARE → REVERSE_IN → PUSH). Improvements made this
session, in order:

1. **`reflector_trust_range`** — below this range the reflector angle geometry
   degenerates; the controller holds IMU yaw instead of chasing noisy bearing.
2. **`center_tolerance_m`** tightening and envelope checks before committing to
   reverse.
3. **No steering while reversing in commit** — `angular.z == 0` inside the funnel
   throat; steering there fights the funnel rails and side-loads the chassis.
4. **Adaptive PUSH boost** — if still single-prox after `push_boost_after_s = 1.5`,
   boost pivot + reverse pressure (`push_omega_boost = 0.28`, `push_v_boost = 0.05`),
   `push_timeout_s = 5.0`.
5. **Reflector confidence score** concept (separation error + frame-to-frame
   continuity) published for gating.

V1 outcomes improved from wild side-banking to repeatable near-misses
(single-contact, contact = 1), but it could not close the last few mm reliably.
That motivated V2.

---

## 3. What V2 tried to achieve (`dock_aligner_v2.cpp`)

Full rewrite as a **reverse pure-pursuit controller** with layered safety
watchdogs. Key design:

- **Pursuit law:** `mu = atan2(-(lat + sin(nyaw)·s − lat_cal), −(along + cos(nyaw)·s))`
  with look-ahead `s = min(0.5·range, max(0, range − close_aim_hold_m))`.
- **Steering:** `az = clamp(kp·mu − kd·gyro_z, ±az_max)`, gyro low-pass 0.8/0.2.
  Angular clamp also limited so the inner wheel never drops below
  `min_wheel_speed` (stalled inner wheel inverts the turn).
- **Compliant throat/seat behavior** (from trial 2 evidence): zero steering inside
  the throat, timed straight push in the seat zone, envelope aborts only active
  outside the throat. The funnel mechanically squares the robot — proven in
  trial 2 (arrived range 0.285 m, nyaw −0.1°, lat +36 mm; funnel rotated the
  chassis ~6° against full opposite command).
- **Adaptive stall boost** (from trial 3): steps `v` up while range is not closing
  (cocked casters), decays when moving.
- **Watchdogs, all proven live on hardware:** capture envelope at ACQUIRE,
  reverse envelope aborts, no-progress abort, μ-divergence tripwire (later
  replaced), single-contact abort, hard-stop range, seat-push timeout, pose
  invalid/stale abort, overall timeout. Four authorized trials, four safe aborts,
  zero collisions.

### The four dock trials (all V2)

| Trial | Outcome | Diagnosis |
|---|---|---|
| 1 | μ tripwire at 0.713 m | lateral converged, yaw response lagged (fit τ≈7.5 s) → added gyro damping + close-aim hold |
| 2 | single-contact abort at 0.209 m (contact 2) | free approach excellent; steering fought funnel rails in throat → compliant seating |
| 3 | no-progress (8 mm/2 s) | casters cocked from manual staging; no breakaway at v = 0.06 → stall boost |
| 4 | μ tripwire at 0.471 m | rotation *opposite* to command in both saturated phases → triggered the floor tests |

### Post-bench retune (deployed, now moot until firmware is fixed)

After the open-loop bench, two grid searches against the measured plant found **zero**
tunes meeting the seat-plane criterion. Re-scoping success to *clean throat entry*
(|lat| < 35 mm, |head| < 8° at 0.30 m — funnel does the rest) gave 154/162 clean,
6 safe aborts, 2 marginal (8.0–8.6° heading, within what the funnel squared in
trial 2). That tune was deployed: v = 0.12 constant, az cap 0.22, kp = 2.0,
kd = 2.0, `half_track_m` corrected 0.15 → **0.1775** (firmware `WHEEL_SEPARATION`
is 0.355 m), new **mid-course gate at 0.45 m** (abort unless |lat err| < 45 mm and
|head err| < 8°), ACQUIRE requires **≥ 0.85 m runway**, μ tripwire removed
(it fired on drift the controller had no authority to fight).

The closed-loop floor test then showed the sim's plant assumption (yaw gain ≈ 0.4)
was wrong for rolling wheels (actual ≈ 0.01), so **do not attempt trial 5 with this
tune** — it is well-designed against a plant that does not exist yet.

---

## 4. Floor-test findings (the measured plant)

### 4.1 Open-loop bench (data: `docs/bench_wheel_breakaway.csv`)

- Straight reverse, cmd → actual RPM (both wheels): 3.8→2.9, 5.7→3.1, 7.6→4.6,
  9.5→6.35, 11.5→6.9, 15.3→7.85, 19.1→9.8, 22.9→16.6. **50–70 % tracking at all
  speeds, steady-state, with K_I = 5 active — should be impossible.**
- First motion always < 0.42 s from rest → wheels break away fine; trial 3's stall
  was a caster/floor outlier, not systemic.
- Differential (cmd L/R → actual): 14.8/8.1 → 6.8/6.4 (ratio 1.83 → **1.06**);
  20.4/10.2 → 7.1/6.2 (**1.15**); 25.9/12.3 → 17.0/13.5 (1.26); 30.4/15.5 →
  23.5/17.5 (~1.3). Differential authority is crushed, worst at low speed.

### 4.2 Closed-loop steering test (data: `docs/bench_steer_closedloop.csv`)

Reversing at 0.12 m/s, exact deployed steering law (kp 2.0, kd 2.0, clamp ±0.22),
gyro-bias-corrected yaw integration:

| Phase | Verdict | Detail |
|---|---|---|
| HOLD 3 s | FAIL | drifted −3.8° while commanding az to −0.196; robot kept turning +0.022 rad/s |
| STEP +10° | FAIL (4.28 s) | reached target mostly via ambient +1.27 °/s drift |
| STEP −10° | FAIL (4.76 s) | error grew 10° → 14.1° under full saturated command before any response |

- Mean gyro during saturated az = ±0.22: **±0.002 rad/s → yaw gain ≈ 0.01**.
- Delivered wheel ratio during saturation: **1.01–1.04** vs 1.96 commanded.
- Dead time under full command: **2.1–3.1 s**.

**Conclusion:** rolling-wheel differential response is far worse than from-rest
steps suggested. The plant cannot steer at docking speeds. Firmware fix required.

---

## 5. Firmware analysis recommendations

Files (read-only inspected this session, **not modified**):
`firmware/esp32/src/firmware.ino` (`moveBase()`, ~line 399 dt calc),
`firmware/esp32/src/pid.cpp`, `firmware/esp32/include/pid.h`,
`firmware/esp32/src/motor.cpp` (`setSpeed()`, inverted-PWM DRV8870 drive),
`firmware/esp32/include/jupiter_config.h` (K_P=5, K_I=5, K_D=0, PWM_MAX=255,
`MOTOR_FF_STATIC=200`, `MOTOR_FF_RELEASE_RPM=4`, `MOTOR_FF_CMD_MIN=3`,
WHEEL_SEPARATION=0.355, COUNTS_PER_REV=1290, MOTOR_MAX_RPM=330, per-motor trims).

### 5.1 Analyse first — instrument before changing anything

Add a temporary debug publisher (or serial CSV) in `moveBase()` emitting per wheel:
`req_rpm, current_rpm, pid_out, integral, ff_applied, duty_written` at ~20 Hz.
Re-run the two bench scripts. This pins the mechanism instead of guessing.
Specific questions the instrumentation must answer:

1. **Unit/scale mismatch (prime suspect):** PID error is in RPM; output is consumed
   by `setSpeed()` as PWM duty (0–255). K_P=5 → a 6 RPM error = 12 % duty. With
   K_I=5 the integral *should* remove steady-state error within seconds — it
   measurably does not. Watch the integral term directly: is it growing, clamped,
   reset, or is dt wrong in practice?
2. **Anti-windup interaction (pid.cpp, conditional integration, 2026-07-25 change):**
   verify the "unwinding only" branches are not blocking accumulation in normal
   (non-saturated) operation, e.g. via `setpoint==0 && error==0` resets or the
   in-range branch never being hit due to the FF term pushing past limits.
3. **`MOTOR_FF_STATIC = 200` (78 % duty) breakaway kick:** it is applied per wheel
   whenever |req_rpm| > 3 and |current_rpm| < 4. During slow differential moves the
   *inner* wheel repeatedly dips under 4 RPM → gets a 200-count kick → both wheels
   end up equal. This alone plausibly explains delivered ratio ≈ 1.0. Check whether
   FF fires continuously (not just at breakaway) during the differential phases.
4. **DRV8870 inverted-PWM drive (`motor.cpp`)**: duty near-saturation behavior and
   `PWM_MAX` off-by-one (`pow(2,PWM_BITS)-1` as a macro — beware double evaluation
   and float `pow` in integer context).
5. **Encoder feedback rate/quantization:** current_rpm update at ~20.8 Hz from
   COUNTS_PER_REV=1290 is fine on paper; confirm no aliasing at 2–10 RPM.

### 5.2 Fix options, in preferred order

1. **Tune (cheap, try first):** rescale PID into PWM domain — output ≈
   `duty = K·(rpm_error)` needs K in the region of `PWM_MAX / MAX_RPM_USED`
   (≈ 255/60 ≈ 4 for docking speeds — so K_P alone is *not* obviously wrong, which
   is why instrumentation must confirm where the output actually goes). Gate
   `MOTOR_FF_STATIC` so it can never fire on a wheel that is *commanded slower than
   its pair* (differential protection), or drop FF to ~120 and only for the first
   150 ms after rest.
2. **Structural (if tuning can't hold a 2:1 ratio):** replace per-wheel independent
   PID with **coupled velocity + yaw-rate control** on the ESP32: an outer loop
   trims the left/right *difference* using the same encoders, so differential is a
   controlled quantity, not an emergent one. This is the robust fix and also cures
   nav creep robot-wide.
3. **Acceptance test (must pass before another dock attempt):** re-run
   `bench_steer_closedloop.py` — all three phases PASS (±3°, reach ≤ 4 s), and
   delivered wheel ratio ≥ 1.6 when 1.96 commanded.

Firmware build is PlatformIO (`firmware/esp32/`); flashing requires Logan's
explicit approval. Keep one long-lived micro-ROS agent (see repo instructions).

---

## 6. Source files modified or to-modify

### Modified this session (hub + deployed/built on Thor)

| File | Status |
|---|---|
| `src/jupiter_nodes/src/dock_aligner_v2.cpp` | **new + iterated** — pursuit controller, watchdogs, compliant seating, stall boost, measured-plant retune, mid-course gate. Built clean both machines. |
| `src/jupiter_nodes/src/dock_aligner.cpp` | V1 improvements (§2) |
| `src/jupiter_nodes/CMakeLists.txt` | added `dock_aligner_v2` target + deps + install |
| `scripts/bench_wheel_breakaway.py` | new — open-loop plant characterization |
| `scripts/bench_steer_closedloop.py` | new — closed-loop steering acceptance test |

### Intended to modify (pending Logan's approval — firmware)

| File | Intended change |
|---|---|
| `firmware/esp32/src/firmware.ino` | instrument `moveBase()`; possibly coupled yaw-rate loop |
| `firmware/esp32/src/pid.cpp` / `include/pid.h` | verify/fix integral path; PWM-domain scaling |
| `firmware/esp32/include/jupiter_config.h` | K_P/K_I retune; `MOTOR_FF_STATIC` gating/reduction |
| `firmware/esp32/src/motor.cpp` | only if inverted-PWM issues surface in instrumentation |

### Small leftover on the Thor side

- `dock_aligner_v2` `min_confidence`, seat/contact params were validated; once the
  firmware passes the acceptance test, re-verify the sim tune against *newly
  measured* gain/τ before trial 5. Do not reuse the g=0.25–0.55 envelope blindly.

---

## 7. Operational state & rules (do not skip)

- Robot: on open floor near the last floor-test end pose; contact = 0; **no**
  `dock_aligner_v2` process running (pkilled before the steer test). Dock present
  but robot not staged.
- Bench CSVs archived alongside this report: `docs/bench_wheel_breakaway.csv`
  (open-loop plant characterization) and `docs/bench_steer_closedloop.csv`
  (closed-loop steering test trace: t, phase, target_deg, yaw_deg, gyro, az_cmd,
  rpm1, rpm2). Originals were in `/tmp` on Thor (volatile).
- Standing rules with Logan: **one motion per explicit "GO"**; contact 3 is the only
  dock success; never hard-cut Thor power; warm teleop ready before any motion;
  evidence beats theory — when his observation contradicts reasoning, run the test;
  after 2–3 failed parameter attempts stop tuning and re-examine architecture
  (this session's core lesson: the architecture problem was one layer down).

---

## 8. Addendum 2026-08-08 (later) — the PID converges; it is slow, not broken

**Author:** Claude Code · **Data:** [`bench_wheel_tracking_long.csv`](bench_wheel_tracking_long.csv)
· **Script:** [`scripts/bench_wheel_tracking_long.py`](../scripts/bench_wheel_tracking_long.py)

### 8.1 Why the re-run

`bench_wheel_breakaway.py` uses `STEP_S = 3.0` and averages only the **last 1.0 s** of it
(`analyze()`, line 79) — so it samples t = 2–3 s. With `K_I = 5`, an error of ~4 RPM, a 20.8 Hz
loop and **`PWM_MAX = 1023`**, the integral accumulates only ~20 PWM/s. It needs roughly **7 s**
to build the ~150 PWM a wheel wants to overcome low-speed friction. §4.1's figures are therefore
a **transient**, and cannot distinguish "converges too slowly" from "never converges".

### 8.2 Result — straight reverse, 10 s dwell, tracking by time window

| Speed | cmd RPM | 2–3 s *(what §4.1 measured)* | 8–10 s | |
|---|---|---|---|---|
| 0.04 m/s | 7.6 | 51 % | **88 %** | rising |
| 0.06 m/s | 11.5 | 44 % | **91 %** | rising |
| 0.08 m/s | 15.3 | 43 % | **108 %** | rising |
| 0.12 m/s | 22.9 | 66 % | **98 %** | rising |

Run at 14.41 → 14.38 V. Phase B (differential) deliberately omitted: if authority is restored,
`az = 0.22` held 10 s would sweep ~126° of arc.

**The 2–3 s column reproduces §4.1's 50–70 % exactly.** This is not a contradictory result — it is
the same result with the observation window extended. The original measurement was right; only the
"steady-state" reading of it was wrong. Convergence takes **~9 s at 0.04 m/s, ~5 s at 0.12 m/s**,
matching the ~7 s predicted from the gains. Overshoot to 108–110 % is the signature of an integral
carrying the loop with far too little proportional help.

### 8.3 What this explains

A docking approach from 0.9 m at 0.12 m/s lasts **~7 s**; the loop needs **~5 s** to deliver a
commanded speed. Every steering correction — which *is* a change in commanded wheel speeds — was
issued and then abandoned long before the firmware could execute it. That is exactly the **2–3 s
dead time under saturated command** measured in §4.2, and why both wheels sat at the *mean* of the
two commanded speeds: the differential never had time to develop. The robot was not refusing to
steer; it was **always still ramping**.

### 8.4 Revised firmware recommendation

The blockage is real and is in the ESP32 layer (§1 stands), but the fix is **gain structure, not a
rewrite**:

- **`K_P = 5` is scaled for the full 0–214 RPM range** (`1023/214 ≈ 4.8`). At docking speeds a
  5 RPM error yields ~25 PWM = **2.4 % duty**, against a wheel needing ~15–20 % to break friction.
  The proportional path is effectively absent exactly where docking operates, leaving the slow
  integral to do everything.
- **`MOTOR_FF_STATIC` is the right idea, mis-gated.** It is Coulomb friction compensation, but
  fires only below `MOTOR_FF_RELEASE_RPM = 4` — it assists breakaway then abandons the wheel
  precisely when sustained help is needed. Note it is **19.5 %** duty, not 78 %.

Suggested order (one change at a time, re-run this bench after each):
1. **Instrument `moveBase()`** per §5.1 — log `req_rpm, current_rpm, pid_out, integral, duty` and
   confirm the integral trajectory matches the ~20 PWM/s predicted here.
2. **Continuous Coulomb feed-forward** (a sustained friction offset, sign of commanded direction)
   so the PID only handles the residual, plus a **higher `K_P`** for low-RPM authority.
3. **Acceptance test:** §5.2's criterion still applies — delivered wheel ratio ≥ 1.6 when 1.96
   commanded — *plus* convergence to ≥ 90 % within **1.5 s**, since a docking approach is ~7 s.

**Do NOT do §5.2 option 2** (coupled velocity + yaw-rate control). It was the correct response to a
plant that never converges; this plant converges.

### 8.5 Caveats

- Run at **14.4 V** (50 % pack). Convergence is proven regardless — a low pack cannot manufacture
  tracking — but re-tune gains at a representative voltage.
- `v = 0.04` showed an asymmetry (m1 6.0 vs m2 1.2 RPM in the 4–6 s window), likely stick-slip.
  Worth attention only if sub-0.05 m/s operation matters.
- Differential authority after a gain fix is **not yet measured** — re-run Phase B of
  `bench_wheel_breakaway.py` (at 3 s dwell, for safety) once the loop is fast.
