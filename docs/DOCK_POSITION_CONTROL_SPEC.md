# Position-Control Docking — Design Spec (for review, nothing built)

**Date:** 2026-08-08 · **Status:** 📋 **PROPOSAL — not implemented, not flashed**
**Origin:** Logan's proposal, after the velocity loop was measured at 4–9 s to reach a commanded
wheel speed ([`DOCKING_HANDOVER_2026-08-08.md`](DOCKING_HANDOVER_2026-08-08.md) §8).

---

## 1. Why

Docking failed because **steering corrections expired before the firmware executed them**. A
docking approach lasts ~7 s; the ESP32 velocity loop needs 4–9 s to reach a commanded wheel speed,
so the commanded left/right differential never developed.

**The insight that removes the problem:** steering is a differential **distance**, not a differential
**speed**.

```
to rotate θ:   Δs = θ × W        (W = wheel separation)
```

That is pure geometry — it does not care how fast it happens. So if the wheels are commanded in
**encoder counts** and eventually deliver them, **the turn is exact even if the velocity profile is
slow and ugly**. The 4–9 s lag stops being fatal: we wait for completion instead of racing a window.

**Scope note:** Nav2 is **not** in the docking path (it only drives to a staging pose, then stops).
This adds a *second, separate* command path used only by docking. **The `/cmd_vel` velocity path
Nav2 uses is untouched** — which makes this lower-risk than retuning the shared velocity loop.

---

## 2. The maths

**Verified live constants** (`jupiter_config.h`; `Kinematics kinematics(WHEEL_RADIUS, WHEEL_SEPARATION, WHEEL_BASE)`):

| | |
|---|---|
| `COUNTS_PER_REV` | 1290 per wheel revolution |
| `WHEEL_RADIUS` | 0.050 m → circumference **0.31416 m** |
| `WHEEL_SEPARATION` W | 0.355 m |
| **counts per metre** | 1290 / 0.31416 = **4106** |
| **1 count** | **0.2435 mm** |

**Differential drive in distance form — the whole conversion:**

```
forward   s = (s_L + s_R) / 2            centre travel
          θ = (s_R - s_L) / W            rotation (rad)

inverse   s_L = s - θ·W/2
          s_R = s + θ·W/2
```

| Move | s_L | s_R | counts |
|---|---|---|---|
| rotate 1° in place | −3.10 mm | +3.10 mm | ∓12.7 |
| rotate 10° in place | −31.0 mm | +31.0 mm | ∓127 |
| straight 0.5 m | +0.5 m | +0.5 m | +2053 each |
| reverse 1.2 m | −1.2 m | −1.2 m | −4927 each |

**Angular resolution ≈ 0.079°/count** (~13 counts per degree) — quantisation is nowhere near a
limit. Segment targets are computed by the aligner and sent as counts; the ESP32 does not need the
geometry.

---

## 3. ⚠️ Calibrate `W` FIRST — before anything else

`θ = (s_R − s_L)/W`, so **a 2 % error in W is a 2 % error in every turn**. The current
`WHEEL_SEPARATION = 0.355` was **derived geometrically** ("0.385 out-out minus one 30 mm wheel
width") and has **never been calibrated against actual rotation**.

**Procedure** (no firmware needed — do it with the existing velocity path):
1. Mark the robot's heading precisely (tape line / laser).
2. Command a full 360° in-place rotation; record encoder counts for both wheels.
3. Measure the **actual** angle turned (BNO055 yaw is fine, floor marks better).
4. Correct: `W_true = W_assumed × (θ_commanded / θ_actual)`.
5. Repeat 3× in each direction; take the mean. Asymmetry between CW and CCW indicates scrub, not W.

**Do not trust commanded angles until this is done.** It is the single highest-leverage calibration
for the whole scheme.

---

## 4. ESP32 firmware — new command mode

### 4.1 Interface

New micro-ROS subscription, **additive**; the existing `cmd_vel` path is unchanged and still the
default.

| Topic | Type | Payload |
|---|---|---|
| `/wheel_move` | `std_msgs/Int32MultiArray` | `[counts_left, counts_right, max_rpm, flags]` |
| `/wheel_move_state` | `std_msgs/UInt8` | 0 = IDLE, 1 = RUNNING, 2 = DONE, 3 = ABORT_STALL, 4 = ABORT_TIMEOUT |
| `/wheel_move_remaining` | `std_msgs/Int32MultiArray` | `[remaining_left, remaining_right]` |

- **Relative** counts (a delta from wherever the wheel is now), not absolute — no shared origin to
  drift.
- `max_rpm` caps the speed of this segment.
- A new `/wheel_move` **supersedes** any in-flight move (last command wins).
- Any `cmd_vel` message with non-zero velocity **cancels** an in-flight move and returns to velocity
  mode — so teleop always wins as an e-stop.

### 4.2 Control law per wheel

```
error_counts = target_counts - counts_travelled_since_command
rpm_cmd      = clamp(K_POS * error_counts, -max_rpm, +max_rpm)     [trapezoidal-limited]
```
then `rpm_cmd` feeds the **existing** velocity PID unchanged. This is a position loop wrapped around
the current velocity loop — no rewrite of what already works.

- **Trapezoidal profile:** ramp `rpm_cmd` up/down at `MAX_ACCEL_RPM_S` so segments don't jerk.
- **Both wheels finish together:** scale both wheels' `max_rpm` by the ratio of their remaining
  counts, so a rotation stays a rotation instead of one wheel finishing first and the robot arcing.
  *This matters more than it looks — it is the count-domain equivalent of the differential collapse.*
- **Completion:** `|error| <= DONE_TOLERANCE_COUNTS` (≈ 8 counts ≈ 2 mm) on **both** wheels, held for
  100 ms, then brake and publish DONE.

### 4.3 Safety — mandatory

| Guard | Behaviour |
|---|---|
| **Stall** | no count progress on a commanded wheel for `STALL_MS` (≈ 700 ms) → stop, publish ABORT_STALL. **Critical:** a position loop pushes forever against a blocked wheel; without this it will cook a motor. |
| **Timeout** | segment exceeds `expected_time × 3` → ABORT_TIMEOUT |
| **Segment cap** | reject any move > `MAX_SEGMENT_COUNTS` (≈ 1.5 m) — guards against a bad computation driving the robot across the room |
| **cmd_vel override** | any non-zero `cmd_vel` cancels the move (teleop e-stop) |
| **Existing reflexes** | prox seat-reflex and the `cmd_vel` watchdog remain in force |
| **Deadband stall near target** | the last few counts are hardest; allow `DONE_TOLERANCE_COUNTS` rather than chasing zero, and let the outer loop correct the residual |

---

## 5. Aligner side

Replace the continuous Twist stream with **discrete segments**, re-measuring between each:

```
loop:
    read reflector -> (along, lateral, nyaw)
    e = along·sin(nyaw) - lateral·cos(nyaw)      offset from the dock centreline
    d = -along·cos(nyaw) - lateral·sin(nyaw)     distance back along the axis

    if d < COMMIT_RANGE:  final straight segment -> throat -> rails -> prox
    θ = rear_aim_error()                          already implemented and tested
    if |θ| > AIM_TOL:   send rotate segment  (s_L,s_R = ∓θW/2)
    else:               send drive segment   (s_L,s_R = -min(SEG_LEN, d))
    wait for DONE / abort on ABORT_*
```

**Nested structure — this is the point:**
- **inner, open loop:** execute a segment in counts — precise, no dependency on velocity tracking
- **outer, closed loop:** re-measure with the reflector after every segment

Slip and residual error are absorbed by the outer loop rather than accumulating. Each stop also
settles the casters — the long-wanted pre-flip, for free.

---

## 6. Cross-check rotations against the IMU

Encoders measure **wheels**, not the **robot**. Slip and caster scrub during in-place rotation can
deliver counts without turning the body — the fundamental limit of odometric steering.

So for every rotation segment, record BNO055 yaw before and after:
```
slip_ratio = θ_imu / θ_commanded
```
- `≈ 1.0` → clean, proceed
- `< 0.8` → real slip. Correct the residual on the next segment, and log it.
- Persistently `< 1.0` by a constant factor → **`W` is miscalibrated**, not slip → redo §3.

This is cheap, and it converts a silent error into a measured one.

---

## 7. Acceptance tests — in order, before any dock attempt

| # | Test | Pass |
|---|---|---|
| 1 | `W` calibration (§3) | CW and CCW agree within 2 % |
| 2 | Rotate ±90°, wheels down, open floor | IMU within **±2°** of commanded, repeatable 5× |
| 3 | Drive 1.0 m straight | tape-measured within **±20 mm**; heading drift < 2° |
| 4 | Stall guard | block a wheel by hand → ABORT_STALL within 700 ms, motors stop |
| 5 | cmd_vel override | teleop during a move cancels it immediately |
| 6 | Segment sequence | 10°, 0.3 m, −10°, 0.3 m — ends within 30 mm / 3° of prediction |

Only after all six pass does a dock attempt make sense.

---

## 8. What this does NOT fix

- **The velocity loop is still slow** (4–9 s). Nav2 still drives through it. This spec removes it
  from docking's critical path but does not repair it — that remains open, and the `K_P` scaling
  analysis in the handover addendum §8.4 still applies when it is addressed.
- **Friction deadband still exists** — it just becomes a *precision* problem (stall a few counts
  short, correct next segment) instead of a *timing* problem that makes docking impossible.
- **Nothing here has been built or tested.** Every number above is derived, not measured.

---

## 9. Implementation order

1. **Calibrate `W`** (§3) — no code, do it first
2. ESP32: position mode + safety guards (§4) — **requires a flash, needs Logan's approval**
3. Acceptance tests 1–5 (§7) — wheels-up first where possible
4. Aligner: segment logic (§5) + IMU cross-check (§6)
5. Acceptance test 6, then a dock attempt

**Open question for Logan:** the existing `dock_aligner` (stop-and-re-aim, built but never run)
already has the right *shape* — AIM in place, then straight segments. It could be adapted to emit
count segments rather than rewritten. Worth deciding before step 4 whether to adapt it or start
clean.
