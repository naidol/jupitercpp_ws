<!--
Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
SPDX-License-Identifier: Apache-2.0
-->

# Jetson AGX Thor — USB / DMA Topology (Hardware Reference)

Authoritative reference so we stop re-deriving this with `lsusb` every time.
Verified on hardware 2026-06-08; **Orbbec re-cabled 2026-06-11 (see CURRENT STATE below).**

## ⚠️ CURRENT STATE — 2026-06-11 re-cable (read this first; the map further down is the older 2026-06-08 layout)
- **Orbbec moved OFF the USB-C direct port (was `2-1`, 5 Gbps) → now on the USB-A side, behind the Realtek
  USB-3 hub at `2-3.1`** (unique id `2-3.1-4`). Verified enumerating @ USB3.2, uvcvideo bound, serial CP9KB53000HP.
- **WHY:** the cold-boot enumeration miss was tied to the USB-C `2-1` direct port. On 2026-06-11 the controller
  re-bind fix (auto at boot AND manual) FAILED to recover it, so the camera was physically moved to the USB-A
  10 G hub — the arrangement that never had the cold-boot problem. **VALIDATED 2026-06-11: a true power-cycle cold
  start enumerated the camera @ 5000M USB3 on `2-3.1` with NO intervention (fix service no-op'd, exit 0). Cold-boot
  miss RESOLVED by the USB-A-hub move.**
- **"Where's the 10 Gbps?"** The `2-3` Realtek hub uplink negotiates **10000M (USB 3.2 Gen2)** with the host. The
  **Orbbec runs 5000M because the Gemini 336 is a USB 3.0 / 5 Gbps device** — that's its hardware ceiling; it shows
  5 Gbps on ANY port, even a direct 10 G one. Nothing is lost by sitting behind the 10 G hub.
- **WaveShare hub (VIA 2109; ESP32 + 7" display + 5 V to the Ethernet switch) → USB-C** (per the re-cable plan;
  the VIA hub now shows on Bus 001 with the touch + CP2102 — physical port not lsusb-verifiable, confirm visually).
  LiDAR is off-USB entirely (RPLIDAR S2E on Ethernet).
- **Launches now match the camera by SERIAL ONLY** (`usb_port` → `''` in camera.launch.py, camera_ai.launch.py,
  navigation_s2e.launch.py, jupiter_bringup.launch.py) so future re-cabling never breaks camera bring-up again.
  Validated 2026-06-11: camera.launch.py connected purely by serial on `2-3.1`.

## TL;DR (the facts that keep getting forgotten)
- There is **ONE xHCI host controller**: `a80aa10000.usb` (under `/sys/devices/platform/bus@0/`).
  It exposes **two logical buses** that are just its USB-2 and USB-3 root hubs:
  - **Bus 001** = USB-2 root hub (480M)
  - **Bus 002** = USB-3 root hub (advertises 20000M = USB 3.2 Gen2x2 *capability*)
- **Bus number ≠ separate controller.** Bus 001 and Bus 002 are the **same** controller. USB-2 and
  USB-3 devices therefore **share its DMA engine + IRQ** → they contend at the controller level.
- **No physical 20 Gbps port exists.** The 20000M is the controller's *capability*. Real links:
  - **USB-C port = 5 Gbps** (Orbbec camera)
  - **USB-A ports = 10 Gbps**, but **drop to 480M** when only USB-2 devices are connected.
  - CP2102 serial devices negotiate **12M** (USB 1.1 full-speed) on the **480M** USB-2 bus.
- **`tegra-xusb` is DMA-stall sensitive:** heavy GPU DMA (Whisper / TensorRT) stalls it (~every 4 s),
  which blocks the slam_toolbox executor and can drop USB device data. See the SLAM+Whisper note.

## Verified device map (2026-06-08)
**Bus 001 — USB-2 side of `a80aa10000.usb` (480M root):**
```
1-3        12M   Bluetooth (IMC Networks 13d3:3586)
1-4       480M   Realtek 4-port USB-2 hub (0bda:5489)
 └ 1-4.1  480M   VIA Labs USB-2 hub (2109:2817)   [the WaveShare hub chain]
    ├ 1-4.1.1  12M  WCH USB2IIC_CTP_CONTROL — 7" display touch controller (8440:2004)
    ├ 1-4.1.2  12M  CP2102 UART (10c4:ea60)
    └ 1-4.1.3  12M  CP2102 UART (10c4:ea60)
```
**Bus 002 — USB-3 side of `a80aa10000.usb` (20000M-cap root):**
```
2-1         5000M  Orbbec Gemini 336 — DIRECT, no hub (USB-C "5b" port, 5 Gbps)
                   [serial CP9KB53000HP; usb_port:'2-1' in jupiter_bringup.launch.py]
                   [2026-06-08: NOT enumerating — absent from lsusb; check power/USB-C link]
2-3        10000M  Realtek 4-port USB-3 hub (0bda:0489)  [USB-A side, 10 Gbps; SPARE/empty]
 └ 2-3.1    5000M  VIA Labs USB-3 hub (2109:0817)        [5 Gbps; SPARE/empty]
```

## Device → mount mapping
- `/dev/jupiter_lidar` → `ttyUSB1` (CP2102 10c4:ea60, Bus 001)
- `/dev/jupiter_esp32` → `ttyUSB0` (CP2102 10c4:ea60, Bus 001)
- Orbbec Gemini 336 → Bus 002 Port `2-1` (USB-C "5b", DIRECT, no hub, 5 Gbps; serial CP9KB53000HP)
- ReSpeaker XVF3800 → **moved off USB to the Raspberry Pi 5** (Ethernet) to kill USB contention.

## Contention implications (the "why this matters")
- **LiDAR (CP2102, Bus 001) and Orbbec (USB-3, Bus 002) share the SAME controller** `a80aa10000.usb`.
  So moving the LiDAR to Ethernet (RPLIDAR S2E) **does** free shared xHCI DMA/IRQ the camera also uses.
  Magnitude = **modest** (LiDAR is 12M, so it's interrupt/scheduling relief, not bandwidth) but **real**,
  given how DMA-stall-sensitive `tegra-xusb` is. It also decongests the USB-2 hub for the **ESP32 + 7"
  display**, and makes the LiDAR itself **immune** to the tegra-xusb stalls.
- The **camera's biggest contention source is GPU DMA** (Whisper/TensorRT), NOT the LiDAR — address that
  via GPU scheduling / operational modes (INTERACT/NAVIGATE/DOCK), separately from the lidar transport.

## How to re-verify (if hardware changes)
```
lsusb -t                                            # tree + speeds
readlink -f /sys/bus/usb/devices/usb1               # controller of Bus 001
readlink -f /sys/bus/usb/devices/usb2               # controller of Bus 002 (same path = same controller)
for d in /sys/bus/usb/devices/*/; do [ -f "$d/speed" ] && echo "$(basename $d) $(cat $d/speed) $(cat $d/product 2>/dev/null)"; done
```

## Gotchas
- **COLD-BOOT CAMERA MISS (confirmed fault + fix, 2026-06-08):** the Thor does NOT enumerate the Orbbec
  on cold boot — at boot the USB-3 hubs come up but port `2-1` fires no connect event, so no camera and
  no `/dev/video*`. Camera/cable/port are healthy: a physical re-plug (or a `tegra-xusb` controller
  re-bind) enumerates it instantly @ 5 Gbps with all 8 `/dev/video0-7`. **Auto-fix:**
  `scripts/jupiter-usb-camera-fix.sh` + `systemd/jupiter-usb-camera-fix.service` — boot-time oneshot that
  re-binds the controller IF the camera is missing (conditional; no-op on a good boot). Install to
  /usr/local/bin + /etc/systemd/system, enable, validate on next cold boot.
  REFINEMENT (Logan, 2026-06-08): the miss happens on a true COLD START (power-cycle) only — a warm
  `sudo reboot` enumerates the camera fine (device stays powered/link-ready across the reset). So the
  fix is INSTALLED + ENABLED but UNVALIDATED until the next real power-off→power-on; check the service
  log then for "enumerated after re-bind" (= fix fired) vs "already enumerated" (= that boot succeeded).
