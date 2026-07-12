# Fused Docking Bring-up Runbook (LiDAR + IMU + IR)

Robot-side checklist for the sensor-fused reverse dock. Do the steps **in order** —
step 3 produces the two parameters steps 4–5 need. Nothing here can be done off the
robot (the hub has no VPI/CUDA to link `jupiter_nodes`).

## The architecture in one line
Three DOF, each measured by the sensor best suited to it:

| DOF | meaning | sensor | topic |
|-----|---------|--------|-------|
| **x** range  | distance to pogo pins | S2E LiDAR, wall line-fit | `/dock/range` (from `dock_range`) |
| **θ** square | angle to the dock wall | LiDAR wall-angle, IMU-damped | `/dock/range` + `/odometry/filtered` |
| **y** lateral| left/right of dock centre | IR beam | `/dock/ir` |

A flat wall is featureless sideways, so the LiDAR **cannot** give lateral — that is why
IR is kept. IR no longer does heading; the IMU + LiDAR own that.

---

## 1. Build — prove the link
```bash
cd ~/jupitercpp_ws
colcon build --packages-select jupiter_nodes
source install/setup.bash
```
Both `dock_range` (new) and the rewritten `dock_ir` compile clean vs Jazzy headers;
this step proves they **link** (unverified on the hub).

## 2. Bring up idle and watch the sensing
```bash
ros2 launch jupiter_bringup dock_ir.launch.py reverse:=true
```
This starts the S2E driver + `dock_range` + `dock_ir`, but `dock_ir` does **nothing**
until you engage. In another terminal, watch the range stream:
```bash
python3 scripts/dock_range_monitor.py          # or: ros2 topic echo /dock/range
```
Confirm, by rolling the robot by hand:
- wall distance **tracks reality** as it moves toward/away from the dock,
- **`fit ±mm` stays small** (the rear sector is clean wall — bearing 0 really points at the dock wall),
- `wall_angle ≈ 0` when you set it square, sign as expected when you skew it,
- (bonus) IMU yaw-rate and wall-angle agree when you rotate it.

If the rear sector is NOT pointing at the wall, the LiDAR mount/frame differs from
assumption — re-check `rear_bearing_deg` (S2E=0 because `base_laser` yaw=π; LD20
`/scan_low`=180). Do **not** proceed until `/dock/range` reads sane.

## 3. Calibrate the two constants (EMPIRICAL — not a tape measure)
Manually back the robot (reversed) into the dock to **physical pogo contact**. Read
`wall_dist` at that moment from the monitor.

- **`dock_depth` = that `wall_dist` value.** It bakes in both the LiDAR→robot-tail
  distance and the dock→wall distance in a single measurement.
- **`contact_dist ≈ 0.005 m`** — stop ~5 mm shy so the magnetic pogos snap it home.

## 4. First engaged run — pin the sign conventions
```bash
ros2 launch jupiter_bringup dock_ir.launch.py \
    reverse:=true dock_depth:=<measured> contact_dist:=0.005
# start / stop:
ros2 service call /dock_engage std_srvs/srv/SetBool "{data: true}"
ros2 service call /dock_engage std_srvs/srv/SetBool "{data: false}"
```
Place the robot ~0.5 m out, reversed, roughly facing the dock, then engage. Watch it
go **ALIGN → APPROACH**. Two signs are found empirically (same drill as before):
- squaring rotation **diverges** → add `wall_angle_sign:=-1.0`
- IR lateral nudge **diverges** → add `steer_sign:=-1.0`

## 5. Tune
Repeat runs, adjusting live via launch args:
- `approach_speed` (max drive speed, default 0.08)
- `Kp_theta` / `Kd_theta` (squaring stiffness / IMU damping) — raise Kd if it hunts
- others (`Kx`, `align_tol`, `lateral_nudge`, `creep_min`) are node params if needed.

---

## Reference — `/dock/range` message layout
`std_msgs/Float32MultiArray`, index convention (must match `dock_range.cpp` /
`dock_ir.cpp`):

| idx | field | units |
|-----|-------|-------|
| 0 | valid (1/0) | — |
| 1 | dist_to_pogo | m |
| 2 | wall_dist | m |
| 3 | wall_angle | rad (0 = square) |
| 4 | fit_rms | m |
| 5 | n_points | — |

## Stop logic
- **Primary:** `dist_to_pogo ≤ contact_dist` (stop on a distance, before contact force builds).
- **Backstop:** stall detector — if commanding drive but `|odom velocity| < stuck_vel_threshold`
  for `stuck_timeout` (3 s), declare pogo contact. Catches the case where range is
  momentarily off or the dock sits closer than expected. (Wheel slip can mask it — hence
  it's the backstop, not the primary.)

## Held in reserve (do NOT add pre-test)
**VL53L0X ToF** — redundant with the LiDAR for the approach; only complementary for the
final cm (direct pogo-gap + close-range squareness), which the stall detector + magnetic
snap already cover. Add only if live testing shows the commit zone stops short or rams.
