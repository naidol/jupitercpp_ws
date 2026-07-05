#!/usr/bin/env bash
# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Recover the Orbbec Gemini 336 when the Jetson Thor misses it on COLD BOOT.
#
# Verified fault (2026-06-08): at cold boot the single tegra-xusb controller
# (a80aa10000.usb) enumerates the USB hubs but NOT the camera on USB-C port 2-1
# (no connect event fires). A physical unplug/replug fixes it. This script does the
# software equivalent: re-bind the xHCI controller, forcing a fresh enumeration of
# everything on it, which brings the camera up — no physical access needed.
#
# CONDITIONAL: it only re-binds if the camera is actually missing. A good boot = no-op.
# Side effect when it DOES re-bind: the LiDAR + ESP32 (same controller, Bus 001)
# re-enumerate too. Harmless at early boot; their /dev/jupiter_* symlinks are keyed by
# serial number (udev), so they stay correct. See docs/THOR_USB_DMA.md.

set -u

DEV_ID="2bc5:0803"                      # Orbbec Gemini 336 (vendor:product)
CONTROLLER="a80aa10000.usb"             # the Thor's single xHCI controller (verified)
DRV="/sys/bus/platform/drivers/tegra-xusb"

present() { lsusb -d "$DEV_ID" >/dev/null 2>&1; }

if present; then
  echo "Orbbec ($DEV_ID) already enumerated — nothing to do."
  exit 0
fi

if [ ! -w "$DRV/unbind" ] || [ ! -w "$DRV/bind" ]; then
  echo "ERROR: $DRV/{bind,unbind} not writable (need root, or driver path changed)."
  exit 2
fi

for attempt in 1 2 3; do
  echo "Orbbec missing (attempt $attempt/3) — re-binding $CONTROLLER ..."
  echo "$CONTROLLER" > "$DRV/unbind" 2>/dev/null || echo "  unbind write failed"
  sleep 2
  echo "$CONTROLLER" > "$DRV/bind"   2>/dev/null || echo "  bind write failed"
  sleep 4
  if present; then
    echo "Orbbec enumerated after re-bind (attempt $attempt). /dev/video* should now exist."
    exit 0
  fi
done

echo "Orbbec STILL missing after 3 re-binds — a physical re-plug may be required."
exit 1
