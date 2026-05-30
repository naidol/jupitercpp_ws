# Jupiter Startup Strategy

## Subsystem Startup Constraints

### Orbbec Gemini 336 Camera
- **Cold start** (after power-on, reboot, or clean shutdown): ~43s total init
  - Select device cost: ~20s (camera firmware cold boot)
  - Settings + stream enable: ~23s additional
- **Warm start** (camera firmware left running after crash/no clean exit): ~1.5s
- **Root cause of slow cold start**: Orbbec SDK's `Close device` call tells the
  camera firmware to reset. Next launch must wait for firmware to re-boot from flash.
- **Fix (future)**: Run camera as a persistent systemd service that is never cleanly
  stopped — camera stays warm across bringup restarts. Not yet implemented.
- **Profile required for fast SDK init**: depth MUST be enabled alongside color.
  Color-only mode triggers a 20s SDK timeout. See orbbec_gemini336_profiles.md.
- **Minimum depth profile**: 424x240 @ 6fps (320x240 does NOT exist)

### Whisper ASR
- Loads 1.5GB model to GPU: ~1.5s
- Must NOT overlap with camera USB select — causes memory bus saturation
- Camera select must complete BEFORE Whisper starts (camera starts at t=1s,
  Whisper at t=2s; camera select done in 235ms if warm, 20s if cold)

### TensorRT Face Recognition (SFace)
- Loads from cache: ~0.5s when GPU is idle, ~5s when competing with Whisper GPU
- Must be subscribed to /camera/color/image_raw before first greeting fires

### Brain (Ollama)
- Connects to Ollama HTTP server: ~0.2s (fast, no GPU load)
- Publishes greeting to /tts/input on first face match → Voice MUST be subscribed
  before first face match triggers, otherwise greeting is lost and display is stuck
  on speaking icon forever

### Voice (Piper TTS)
- Subscribes to /tts/input for TTS playback
- Must be running BEFORE first face match triggers the greeting
- If voice misses the greeting message, display shows speaking icon forever
  with no audio and never transitions to listening

### micro-ROS Agent
- Connects to ESP32 on Bus 001 (/dev/jupiter_esp32)
- Bus 001 is completely independent of camera Bus 002
- Fast connection (~345ms) — start early, get out of the way
- Must be running before Nav2 needs wheel odometry

---

## Current Startup Sequence (AI-only mode: enable_nav:=false enable_voice:=true)

```
t=0s   disable_screensaver  xset s off dpms 0 0 0 — prevents display blanking
t=0s   jupiter_display      Robot face on 7-inch panel immediately
t=0s   micro_ros_agent      ESP32 connection (Bus 001, fast, independent)
t=1s   camera               color 640x480@15fps + depth 424x240@6fps + IMU 100Hz
t=2s   jupiter_voice        Whisper loads 1.5GB GPU (~1.5s → ready by t=3.5s)
t=3s   jupiter_face_recognition  TensorRT from cache (~0.5s → ready by t=3.5s)
t=3s   jupiter_vision       AprilTags (fast start)
t=3s   jupiter_brain        Connects to Ollama (~0.2s → ready by t=3.2s)
```

**Expected greeting time (warm camera):**  ~8-10s after launch
**Expected greeting time (cold camera):**  ~65s after launch (camera firmware boot)

---

## Current Startup Sequence (Nav mode: enable_nav:=true mode:=nav)

```
t=0s   disable_screensaver
t=0s   micro_ros_agent      ESP32 must connect before camera DMA storm
t=3s   navigation.launch.py  EKF + LiDAR + static TFs + cuVSLAM + Nav2 nodes
t=4s   camera (nav mode)    left_ir + right_ir 640x480@15fps + IMU 200Hz
                             (nav mode: NO color, NO depth — IR only for cuVSLAM)
t=8s   lifecycle_manager    5s delay inside navigation.launch.py (MPPI CUDA init)
```

---

## Current Startup Sequence (Full: enable_nav:=true mode:=nav enable_voice:=true)
Use jupiter_bringup_full.launch.py for this mode.

```
t=0s   micro_ros_agent
t=0s   jupiter_display
t=3s   navigation.launch.py
t=5s   camera (full mode)   color 640x480@6fps + left_ir + right_ir@15fps + IMU 200Hz
                             (6fps color to avoid LiDAR USB DMA starvation)
t=7s   jupiter_voice        After camera stable
t=8s   jupiter_face_recognition
t=8s   jupiter_vision
t=8s   jupiter_brain
```

---

## Known Issues / Constraints

| Issue | Cause | Status |
|---|---|---|
| Camera cold start 60s | Orbbec firmware reboots on clean SDK exit | Accepted — future fix: persistent camera service |
| LiDAR timeout in full stack | LD20 USB serial starved by camera DMA | Accepted — fixed by RPLIDAR S2E (ETA June 10) |
| No full stack (Nav+Voice) | LiDAR DMA starvation + Whisper GPU causes system freeze | Accepted — fixed by RPLIDAR S2E + XIAO ESP32S3 |

---

## Startup Commands

```bash
# AI only (voice/vision/brain, no navigation)
ros2 launch jupiter_bringup jupiter_bringup.launch.py \
  enable_microros:=true enable_nav:=false enable_voice:=true

# Navigation only (no voice/AI)
ros2 launch jupiter_bringup jupiter_bringup.launch.py \
  enable_microros:=true enable_nav:=true mode:=nav enable_voice:=false

# SLAM mapping (no voice)
ros2 launch jupiter_bringup jupiter_bringup.launch.py \
  enable_microros:=true enable_nav:=true mode:=slam enable_voice:=false

# Full stack (Nav + Voice) — limited by USB DMA until June 10 hardware
ros2 launch jupiter_bringup jupiter_bringup_full.launch.py \
  enable_microros:=true
```
