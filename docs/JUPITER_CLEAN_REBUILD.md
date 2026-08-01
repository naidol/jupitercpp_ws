# Jupiter — Clean Rebuild Manifest (JetPack 7.2 / L4T R39.2)

**Purpose:** reproducible "how Jupiter is built" checklist after the 2026-07-29 reflash. Philosophy:
**install the bare minimum; add back only what proves missing.** No cuVSLAM, no Docker, no trial cruft.
**All recovered data is at `~/thor_recovery/` on the hub** (`full/` = whole home+etc+boot+root; `memory/` = profiles+conversations).

---

## 1. Hardware — final inventory

| Keep | Drop |
|---|---|
| **Orbbec Gemini 336** (USB3) — primary always-on sensor | **LD20 LiDAR** — unused; pillars + cable runs cripple its scan |
| **ESP32 / micro-ROS** (USB serial) — motors, IMU, prox, battery, dock IR | **Rear webcam** (USB) — only fed the abandoned AprilTag path |
| **RPLIDAR S2E** (Ethernet) — nav + dock reflector | **WaveShare USB hub** — brownout-prone (boot-hang suspect) |
| **7" HDMI display** (HDMI + USB touch*) | *(touch line too, if touch input unused)* |
| **ReSpeaker** — on the Pi5 (Ethernet), not Thor USB | |
| **Dock**: Nano + SSR + IR TSOP + 2× prox | |

**7" display:** keep for the face, **remove the touch USB line**, power it from a **separate 12 V barrel-jack** feed. The
face (video) AND the TTS audio to its speaker both ride **HDMI** — so the display comes **off the USB bus entirely.**
*(Verify the speaker is HDMI-audio, not USB-audio — the touch is the USB part; audio almost certainly rides HDMI.)*

**USB topology (the boot-reliability fix):** with LD20, webcam, ReSpeaker (Pi5) and display-touch all gone, Thor is left with
just **Orbbec + ESP32** on USB — plug **both directly into Thor's native USB ports** (clean sequenced power from the Jetson),
**no external hub.** Only add a **bus-powered** USB-C→USB-A adapter if short on ports; never an externally-buck-powered hub
(that recreates the brownout). This eliminates the WaveShare/VIA hub that was wedging cold boots.

---

## 2. Lean software stack to INSTALL

**Base (from JetPack 7.2 flash):** Ubuntu 24.04, CUDA, cuDNN, TensorRT, Multimedia. *(Host-side x86 CUDA NOT installed — build is on-device.)*

**Then, on-device, in order:**
1. **ROS 2 Jazzy** (ros-jazzy-desktop or -ros-base) + `colcon`, `rosdep`.
2. **micro-ROS agent** (`~/microros_ws`) — for the ESP32 link.
3. **Nav/localisation ROS pkgs:** `robot_localization` (EKF), Nav2, `slam_toolbox`.
4. **Orbbec SDK** (OrbbecSDK C++) — the camera.
5. **AI stack — CONFIRMED from `brain.cpp` (2026-07-29): the Brain calls ollama at `localhost:11434` `/v1/chat/completions`.**
   - ASR → **whisper.cpp** (CUDA build)
   - TTS → **piper**
   - LLM + VLM → **ollama** only. Models: **`gemma4:e2b`** (text) + **`llava:7b`** (vision).
   - **NO vLLM, NO Qwen, NO llama.cpp** — the Brain uses none of them (they were model-swap trials).
6. **jupiter workspace:** deploy `jupitercpp_ws` source from the hub (`dev-deploy` / git), `colcon build --packages-select jupiter_nodes jupiter_bringup sllidar_ros2` (Release).
7. **Face-rec models:** SFace + YuNet ONNX → **TensorRT engines recompile on-device on first run** (don't copy `.trt` from backup).

---

## 3. Explicitly EXCLUDE (the sediment we're not reinstalling)

- **Isaac ROS / cuVSLAM / nvblox** — abandoned; home too feature-poor for visual odometry.
- **Docker / nvidia-container-toolkit** — house rule: no Docker.
- **`ldlidar_stl_ros2`** — LD20 driver; LiDAR dropped.
- **AprilTag docking path** — `vision.cpp` (VPI), `webcam_publisher`, `dock_approach`, `dock_ir`, `dock_range`. Keep the source in-tree as record; don't build/run. (Superseded by `dock_reflector` + `dock_aligner`.)
- **sherpa-onnx** — second ASR stack; using whisper.cpp.
- **vLLM + Qwen3 (35 GB safetensors) + the torch/triton/cudnn Python env** — CONFIRMED unused (Brain uses ollama+gemma4:e2b). This was the biggest model-swap trial; skipping it removes the worst part of a Jetson rebuild.
- **llama.cpp + `gemma-2b.gguf`** — trial; Brain calls ollama, not llama.cpp directly.
- **Unused ollama blobs** — e.g. the ~19.9 GB blob (a dead trial); keep only `gemma4:e2b` + `llava:7b`.
- **DeepStream, Holoscan, Nsight** — not installed.

---

## 4. Restore from `~/thor_recovery/` (cherry-pick, never wholesale)

**Models** → copy to the fresh `/home/jupiter/` (only what the Brain actually uses — ~8 GB, not 70):
- **ollama:** restore `full/jupiter/.ollama/`, keep **`gemma4:e2b` + `llava:7b`**, then `ollama rm` the unused big blobs (e.g. ~19.9 GB trial).
- **whisper.cpp:** `full/jupiter/jupitercpp_ws/whisper.cpp/models/`.
- **SKIP:** `full/jupiter/models/` (35 GB Qwen — unused), the vLLM/torch Python env, `llama.cpp/models/`.

**Precious data** → `memory/` (both `~/thor_recovery/memory/` and `full/jupiter/jupitercpp_ws/memory/`):
- Restore to `/home/jupiter/jupitercpp_ws/memory/{profiles,conversations}/` — Logan, Indrani, Jevan.

**Config** → cherry-pick from `full/etc` + `full/boot` (do NOT overwrite the fresh `/etc` wholesale):
- **udev:** `full/etc/udev/rules.d/99-jupiter-serial.rules` — **edit down to just the ESP32 symlink** (`/dev/jupiter_esp32`); LD20 rule gone.
- **systemd:** re-create `jupiter-microros.service`, `jupiter-shutdown-pi5.service`. **Skip `jupiter-lidar.service`** (LD20 gone). **`jupiter-usb-camera-fix.service` — try WITHOUT it first** (7.2 fixes USB enum); re-add only if the Orbbec cold-boot quirk returns.
- **WiFi:** SSID + PSK from `full/etc/netplan/90-NM-d14cca7b-…yaml` (netplan/NM renderer).
- **env:** `ROS_DOMAIN_ID=0` in `.bashrc` (must match Pi5), ROS source lines.
- **display: ALWAYS-ON face, NEVER blank, NEVER locked out (the real requirement).** The face must stay up permanently — there's **no touchscreen to wake it** if it blanks. But `efifb:off`/`fbcon=map:0` (the old blunt fix) killed the console fallback → lockout. Correct recipe — disable **all three** blank sources at the *software* level while KEEPING the console:
  - Kernel cmdline: **`consoleblank=0`** (never blank the fbcon, but keep it → `Ctrl+Alt+F2` still logs in). Use this INSTEAD of `efifb:off`.
  - Session: `xset s off -dpms s noblank` (X screensaver + DPMS off — already in bringup).
  - GNOME (if used): `gsettings set org.gnome.desktop.session idle-delay 0` + screensaver lock off.
  - Result: face never blanks from any source, console always available. Verify: leave it idle 15+ min → face still up; `Ctrl+Alt+F2` → login.
- **display face autostart (RE-CREATE — old files were LOST):** old GDM session `/usr/share/xsessions/jupiter.desktop` → `/usr/local/bin/jupiter-session.sh` were in `/usr` (not backed up, not in repo) → gone. Recreate to auto-launch the face **fullscreen over a normal desktop**. Face app (`jupiter_display` QML) + its `DISPLAY=:0` launch live in `jupiter_bringup_full.launch.py` (repo) — reuse. **Keep the new session files IN the repo** (`jupiter_bringup/scripts/`) so a reflash never loses them again.
- **⚠️ BACKUP-SCOPE LESSON (2026-07-29):** our SSD backup grabbed `/home`+`/etc`+`/boot`+`/root` but **excluded `/usr`** — which cost us the two custom `/usr/local` + `/usr/share/xsessions` display files. Next full backup: also include **`/usr/local/`** and any custom `/usr/share/*` (custom scripts/binaries/sessions live there).

---

## 5. Ordered rebuild

1. Flash **JetPack 7.2** (Direct Flash) → first boot → **username = `jupiter`** (critical: keeps every `/home/jupiter/...` path + config aligned).
2. Restore **WiFi** (from the netplan file) → network up → `sudo apt update && upgrade`.
3. Install **ROS 2 Jazzy** + colcon/rosdep + the nav/localisation packages.
4. Install **Orbbec SDK**, build **whisper.cpp** (CUDA), **piper**, the chosen **LLM**, **ollama**.
5. Restore **models** + **`memory/`** from `~/thor_recovery/`.
6. Deploy `jupitercpp_ws` source (dev-deploy from hub) → `colcon build` (Release).
7. Cherry-pick **config** (udev, systemd, env) per §4.
8. Plug **Orbbec + ESP32** into native USB ports; flash ESP32 firmware if needed; start micro-ROS agent.
9. First run builds the **TensorRT face-rec engine** (slow once, cached after).
10. Bring up `jupiter_bringup_full`, verify voice/face/nav; re-verify the profiles loaded (Logan/Indrani/Jevan).

---

## 6. Gotchas / decisions

- **⭐ SDK Manager flashing — download the IMAGE ONLY (relearned every reflash):** do **not** let SDK Manager download all components. It parallelizes aggressively, saturates the link, and "`12 range(s) failed permanently`" thrashes the whole download to a crawl (~96 KB/s). In **Step 02, deselect everything except "Jetson Linux"** (Image + Flash) — uncheck all "Jetson Runtime Components" + "Jetson SDK Components". With the single ~3 GB image alone, it downloads clean and fast. Flash the OS, then install CUDA/cuDNN/TensorRT **on-device** with `sudo apt install nvidia-jetpack` (pulls the latest anyway — so the component download was always redundant). Use **"Download now, install later"** + **Direct Flash**. This is the reliable, lean path.


- **Username `jupiter`** — non-negotiable for path alignment.
- **⭐ HARD RULE — Thor must NEVER have SSH as its only way in.** The old cmdline (`video=efifb:off console=tty0 … fbcon=map:0`) suppressed the framebuffer console so the 7" stayed blank for the face — which meant **no local login on a monitor**. When the fs corrupted and sshd died (2026-07-29), that config turned a recoverable problem into a **total lockout** (blank display + serial-not-routed + dead SSH = no door). This cost hours and forced the SSD-pull recovery. On the rebuild, keep the face AND a safety door — two options:
  - **(a) RECOMMENDED — don't suppress the console; run the Jupiter face as a *fullscreen app* over the desktop.** Face shows in normal use; monitor+keyboard always yields a login. Zero lockout risk.
  - **(b) If you want pure `efifb:off` boot-to-face**, it is MANDATORY to first set up AND **test** a working **UART serial console + getty** on the debug port (the USB-C debug enumerates as ttyACM0-3) as the guaranteed back door.
  - **Non-negotiable: verify the second access path WORKS before enabling any console-suppression config.** Never configure the lockout and *then* find there's no way in. Don't blindly restore the old cmdline.
- **Display isn't just a passive HDMI sink** — its console/cmdline config has system-wide access consequences (see the HARD RULE above).
- **TensorRT engines / colcon build / Python env** — rebuild fresh, never restore (OS/version-tied).
- **micro-ROS agent** must source `~/microros_ws` (full bringup previously didn't — fix in the launch).
- **BNO055 vs Orbbec-336 IMU** — still an open "which is primary heading" decision (parked); doesn't affect the flash.

*Compiled 2026-07-29 during the 7.2 reflash prep. Add rows as we discover anything the bare-minimum install missed.*
