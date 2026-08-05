# Repository Instructions — Jupiter Robot

Jupiter is an **always-on vision/voice/AI companion robot** on a Jetson AGX Thor. It is not a
navigation platform that happens to talk — the camera, voice and brain are the product; navigation
and docking exist to keep it alive and mobile. Judge design decisions against that.

---

## 1. Environment (verified 2026-08-05 — trust this over `CLAUDE.md`)

| | |
|---|---|
| Compute | Jetson AGX Thor — **JetPack 7.2 / L4T R39.2**, Ubuntu 24.04, kernel 6.8.12-tegra |
| ROS | **ROS 2 Jazzy**, RMW = FastDDS (default), `ROS_DOMAIN_ID=0` |
| Language | **C++17** only. `ament_cmake` + colcon, `CMAKE_BUILD_TYPE=Release` |
| GPU stack | CUDA 13.x, **TensorRT 10.16**, OpenCV 4.8.0, VPI4 |
| AI runtimes | **ollama** (`gemma4:e2b` text, `llava:7b` vision) · **whisper.cpp** (CUDA ASR) · **piper** (TTS) |
| Drive | **Differential drive** — 2 driven front wheels (DRV8870) + 2 passive rear casters, 100 mm AGV rubber |
| LiDAR | **RPLIDAR S2E over Ethernet/UDP** (192.168.11.2:8089), scan plane at **0.518 m** |
| MCU | ESP32 via micro-ROS, `/dev/jupiter_esp32` @ **460800** baud |

> ⚠️ **`CLAUDE.md` in the repo root is STALE.** It says mecanum wheels (now diff-drive + casters),
> LD20 LiDAR (removed — now S2E), and llama.cpp for the LLM (now ollama). Ignore those three claims.
> Everything else in it still holds.

---

## 2. Hard constraints — do not violate

- **NO Python in ROS 2 nodes.** C++ only. (Python is fine for standalone bench scripts in `scripts/`.)
- **NO Docker.** Bare metal only.
- **NO `sudo pip install`** — never touch system Python.
- **NO new external dependencies** without explicit approval from Logan.
- TensorRT engines are built **on-device**; never copy `.trt` files between machines.
- `CMakeLists.txt` must use `ament_cmake`, not plain cmake.

---

## 3. Build & deploy loop — **this is not a local project**

Code is edited on the **hub** (`/home/logan/jupitercpp_ws`) but **builds and runs on the robot**
(`jupiter@192.168.0.8:/home/jupiter/jupitercpp_ws`). Editing files locally changes nothing on the
robot until they are deployed.

```bash
# 1. deploy the changed file(s)
rsync -a src/jupiter_nodes/src/<file>.cpp \
      jupiter@192.168.0.8:/home/jupiter/jupitercpp_ws/src/jupiter_nodes/src/

# 2. build ON the robot
ssh jupiter@192.168.0.8 'cd ~/jupitercpp_ws && source /opt/ros/jazzy/setup.bash && \
  colcon build --packages-select jupiter_nodes --cmake-args -DCMAKE_BUILD_TYPE=Release'

# 3. run as a transient systemd unit (survives the SSH session)
ssh jupiter@192.168.0.8 'sudo systemd-run --unit=<name> --uid=jupiter --gid=jupiter \
  --setenv=HOME=/home/jupiter --setenv=XDG_RUNTIME_DIR=/run/user/2001 \
  bash -lc "source /opt/ros/jazzy/setup.bash; source \$HOME/jupitercpp_ws/install/setup.bash; \
  exec ros2 run <pkg> <exe>"'
```

**Config files** (`config/*.yaml`, `*.xml`) must be rsync'd to **both** the `src/` tree **and**
`install/jupiter_bringup/share/jupiter_bringup/config/` — launches read the installed copy.

**micro-ROS agent lives in a separate overlay** (`~/microros_ws`) and must be sourced explicitly:
```bash
source $HOME/microros_ws/install/local_setup.bash && \
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/jupiter_esp32 -b 460800
```
Keep **one long-lived agent**. Restarting it churns the ESP32 into a reconnect loop.

---

## 4. Safety rules — non-negotiable

- **NEVER hard-cut Thor's power.** Always `sudo poweroff`. Yanking power has corrupted the SSD before
  and cost a full reflash.
- **Never take an action that changes robot state** — power, motors, running services, launching nodes
  — without Logan explicitly asking. Propose and wait.
- **Before commanding motion:** have a warm teleop session ready as an e-stop. Cold ROS CLI calls take
  seconds to discover and are useless in an emergency.
- **The Pi 5 must be shut down before Thor** — it runs off the same rail, and Thor is the only network
  route to it (`ssh jupiter@10.0.0.2` via Thor).
- **Power the dock off before undocking mid-charge** — a known latch bug can leave the pogo pins live
  at 16.8 V (see `docs/ACTIVE_TASK.md` §6).
- ⚠️ **Do not run autonomous navigation in clutter.** The low-obstacle layer is blind: the S2E sees
  nothing below 0.518 m, the camera is tilted up, and the ToF ring is not built yet.

---

## 5. Never commit or push

`.gitignore` covers these — do not defeat it:

- **`memory/`** — face profiles and conversation history. **Personal data for real people.** Never push.
- `models/`, `whisper.cpp/`, `piper_tts/`, `llama.cpp/` — large external/build artifacts
- `build/`, `install/`, `log/`, `firmware/**/.pio/`

---

## 6. Code style

- Modern C++17; RAII; smart pointers, no raw `new`/`delete`
- Explicit error handling on every SDK call
- Meaningful names — no single-letter variables
- Node names `snake_case`; topics `/robot/subsystem/data`
- Handle SIGINT cleanly (`rclcpp::shutdown()`) — this matters: the S2E LiDAR motor only spins down on a
  clean shutdown, so an unclean kill leaves it spinning
- **Parameter-driven config — no hardcoded values.** Proven values belong as parameter *defaults* in
  the node, not as launch-time overrides that get lost
- Update `.cpp`, `.hpp`, `CMakeLists.txt` and `package.xml` together in one change

---

## 7. Working agreements with Logan

- **Logan is a seasoned engineer** (retired, mechatronics-literate household). Treat him as a peer.
  Evidence beats theory; his hardware intuition has repeatedly been right where model reasoning was wrong.
- **When his empirical observation contradicts your reasoning, run the test — don't defend the theory.**
- **Don't default to blaming hardware.** Rank hypotheses by cost-to-test; try free config/software
  checks before betting on a hardware cause.
- **State confidence honestly.** Say what is proven, what is inferred, and what is untested. Do not
  present an untested change as working.
- **Avoid long tune-and-retest loops on hardware.** Each iteration costs his real time. If two or three
  parameter attempts fail, stop and re-examine the architecture instead of continuing to tune.
