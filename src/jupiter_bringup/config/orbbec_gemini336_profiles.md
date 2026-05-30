# Orbbec Gemini 336 — Stream Profiles Reference

Serial: CP9KB53000HP  
Firmware: 1.4.60  
ROS Wrapper: 2.7.6  
USB: 3.2 Gen1 (5Gbps) on Bus 002 Port 1  
usb_port param: '2-1'

---

## CRITICAL: SDK Init Behaviour

- **Color-only mode** (depth disabled): `Select device cost ~22s` — SDK hangs waiting
  for depth HW that never initialises. DO NOT use color-only.
- **Color + Depth**: `Select device cost ~267ms` — fast path. Always enable depth
  alongside color even if depth data is not consumed.
- **enable_laser must always be false** when cuVSLAM is running — projector dot
  pattern causes massive VIO drift.

---

## Depth Profiles (confirmed from SDK error output)

| Width | Height | FPS options |
|-------|--------|-------------|
| 848   | 480    | 6, 10, 15, 30, 60, 100 |
| 640   | 480    | 6, 10, 15, 30, 60, 90  |
| 640   | 400    | 6, 10, 15, 30          |
| 640   | 360    | 6, 10, 15, 30, 60, 90  |
| 480   | 270    | 6, 10, 15, 30, 60, 90  |
| 424   | 266    | 6, 10, 15, 30          |
| 424   | 240    | 6, 10, 15, 30, 60, 90  |
| 1280  | 800    | 6, 10, 15, 30          |
| 1280  | 720    | 6, 10, 15, 30          |

**Minimum depth profile: 424x240 @ 6fps**  
320x240 does NOT exist — do not use it.

---

## Color Profiles (confirmed working)

| Width | Height | FPS | Format |
|-------|--------|-----|--------|
| 640   | 480    | 15  | MJPG   |

---

## IR Stereo Profiles (for cuVSLAM nav mode)

| Width | Height | FPS |
|-------|--------|-----|
| 640   | 480    | 15  |

---

## IMU

- Accel: up to 200Hz
- Gyro: up to 200Hz
- Frame: camera_accel_gyro_optical_frame

---

## Confirmed Working Configurations

### AI-only (voice/vision/brain — no nav)
```
color:  640x480 @ 15fps MJPG
depth:  424x240 @ 6fps  (dummy — forces fast SDK init, nobody subscribes)
accel:  100Hz
gyro:   100Hz
enable_laser: false
```

### Nav mode (cuVSLAM localisation)
```
left_ir:  640x480 @ 15fps
right_ir: 640x480 @ 15fps
accel:    200Hz
gyro:     200Hz
enable_laser: false
enable_color: false
enable_depth: false
```

### SLAM mapping
```
color:  640x480 @ 15fps MJPG
depth:  640x480 @ 6fps
accel:  200Hz
gyro:   200Hz
enable_laser: false (was true in old config — now always false)
```

### Full stack (nav + vision — jupiter_bringup_full.launch.py)
```
color:      640x480 @ 6fps MJPG (reduced to avoid LiDAR DMA starvation)
left_ir:    640x480 @ 15fps
right_ir:   640x480 @ 15fps
accel:      200Hz
gyro:       200Hz
enable_laser: false
enable_depth: false
```
