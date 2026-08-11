#!/usr/bin/env bash
# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# One repeatability trial of dock_aligner_v3. Records the START pose, runs the dock, records the
# OUTCOME, and appends a CSV row so trials are comparable instead of anecdotal.
#
# Success is contact==3 (BOTH prox). contact==2 or 1 is a PARTIAL seat: the pogo pins do not
# energise, so it is a failure for charging purposes even though it looks close.
#
#   ./dock_test_run.sh "label"        # label describes the staging, e.g. "left-offset"
#
# CSV: ~/dock_trials.csv
#   label,start_range,start_lat,start_skew,start_conf,
#   segments,result,contact,end_range,end_lat,end_skew,seconds

# NOTE: deliberately NOT 'set -u'. ROS 2 setup.bash references unset variables
# (AMENT_TRACE_SETUP_FILES, COLCON_TRACE, ...), so 'set -u' makes the script exit
# silently at the first source — which it did, with no output and no trial run.
LABEL="${1:-trial}"
EXTRA="${2:-}"          # optional --ros-args overrides, e.g. "-p seg_rpm:=16"
CSV="$HOME/dock_trials.csv"
TIMEOUT=140

source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$HOME/jupitercpp_ws/install/setup.bash" 2>/dev/null

refl () {   # "range lateral skew conf"
  local r c
  r=$(timeout 4 ros2 topic echo --once /dock/reflector 2>/dev/null \
      | grep -E '^-' | head -6 | sed 's/^- *//' | tr '\n' ' ')
  c=$(timeout 4 ros2 topic echo --once /dock/reflector_confidence 2>/dev/null \
      | grep -oE '[0-9.]+' | head -1)
  echo "$(echo "$r" | cut -d' ' -f4) $(echo "$r" | cut -d' ' -f3) $(echo "$r" | cut -d' ' -f6) ${c:-0}"
}
contact () { timeout 4 ros2 topic echo --once /dock/contact 2>/dev/null | grep -oE '[0-9]+$' | head -1; }
v3state () { timeout 4 ros2 topic echo --once /dock/v3/aligner_state 2>/dev/null | grep -oE '[A-Z_]+' | head -1; }

[ -f "$CSV" ] || echo "label,start_range,start_lat,start_skew,start_conf,segments,result,contact,end_range,end_lat,end_skew,seconds,params" > "$CSV"

# fresh aligner each trial so counters/latches start clean
sudo systemctl stop jup-v3 2>/dev/null; sudo systemctl reset-failed jup-v3 2>/dev/null; sleep 1
sudo systemd-run --unit=jup-v3 --uid=jupiter --gid=jupiter --setenv=HOME=/home/jupiter \
  --setenv=XDG_RUNTIME_DIR=/run/user/2001 \
  bash -lc "source /opt/ros/jazzy/setup.bash; source \$HOME/jupitercpp_ws/install/setup.bash; exec ros2 run jupiter_nodes dock_aligner_v3 ${EXTRA:+--ros-args $EXTRA}" >/dev/null 2>&1
sleep 6

read SR SL SK SC <<< "$(refl)"
printf "  START  range %-7s lateral %-8s skew %-7s conf %s\n" "$SR" "$SL" "$SK" "$SC"

T0=$(date +%s)
timeout 6 ros2 service call /dock/v3/align_start std_srvs/srv/Trigger >/dev/null 2>&1

RESULT="TIMEOUT"
while [ $(( $(date +%s) - T0 )) -lt $TIMEOUT ]; do
  S=$(v3state)
  case "$S" in
    SEATED) RESULT="SEATED"; break ;;
    ABORT)  RESULT="ABORT";  break ;;
  esac
  sleep 2
done
SECS=$(( $(date +%s) - T0 ))

sleep 2
read ER EL EK EC <<< "$(refl)"
C=$(contact)
SEGS=$(journalctl -u jup-v3 --since "${TIMEOUT} seconds ago" --no-pager 2>/dev/null | grep -c "^.*seg [0-9]*:")

VERDICT="FAIL"; [ "${C:-0}" = "3" ] && VERDICT="PASS"
printf "  END    range %-7s lateral %-8s skew %-7s contact %s  [%s in %ss, %s segs] -> %s\n" \
  "$ER" "$EL" "$EK" "${C:-?}" "$RESULT" "$SECS" "$SEGS" "$VERDICT"

echo "$LABEL,$SR,$SL,$SK,$SC,$SEGS,$RESULT,${C:-0},$ER,$EL,$EK,$SECS,$EXTRA" >> "$CSV"
echo "  (appended to $CSV)"
