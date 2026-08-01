#!/bin/bash
# Safely copy the funnel files to the Windows Downloads folder (dual-boot, same disk).
# Run:  sudo bash /home/logan/jupitercpp_ws/hardware/dock_funnel/copy_to_windows.sh
#
# SAFETY: mounts with plain ntfs-3g and NEVER forces. If Windows is hibernated (Fast Startup),
# the mount fails or comes up read-only and this script REFUSES to write — no corruption risk.
set -u
SRC="/home/logan/jupitercpp_ws/hardware/dock_funnel"
DEV="/dev/nvme0n1p3"        # the 292GB NTFS Windows partition
MP="/mnt/wintmp"

mkdir -p "$MP"
echo ">> mounting $DEV (safe, no force) ..."
if ! mount -t ntfs-3g "$DEV" "$MP" 2>/tmp/ntfsmount.err; then
  echo "!! mount FAILED — Windows is almost certainly hibernated (Fast Startup ON)."
  sed 's/^/   /' /tmp/ntfsmount.err
  echo "   Fix: boot Windows -> disable Fast Startup -> full Shutdown (not restart). Or use OneDrive."
  exit 1
fi
if mount | grep -qE " $MP .*[(,]ro[,)]"; then
  echo "!! mounted READ-ONLY (hibernated) — refusing to write. Unmounting."
  umount "$MP"; exit 1
fi
echo ">> mounted READ-WRITE (Windows cleanly shut down). Copying ..."
copied=0
for d in "$MP"/Users/*/Downloads; do
  case "$d" in *"/Default/"*|*"/Public/"*|*"/All Users/"*|*"/Default User/"*) continue;; esac
  [ -d "$d" ] || continue
  cp -v "$SRC"/funnel_rail_right.stl "$SRC"/funnel_rail_left.stl "$SRC"/funnel_rail_profile.dxf "$d"/ \
    && echo "   -> $d" && copied=1
done
sync
umount "$MP"
if [ "$copied" = 1 ]; then
  echo ">> DONE. Reboot into Windows; the STLs + DXF are in your Downloads. Open in OrcaSlicer."
else
  echo "!! No user Downloads folder found under $MP/Users/*/ — check the path manually."
fi
