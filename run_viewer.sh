#!/usr/bin/env bash
# Show the LEFT Ditto leader live in the viser viewer (encoders only, torque off).
#   ./run_viewer.sh /dev/cu.usbserial-XXXX      # macOS
#   ./run_viewer.sh /dev/ttyUSB0                # Linux
#   ./run_viewer.sh                             # no hardware: sliders only
set -euo pipefail
cd "$(dirname "$0")/ditto_sharpa"
if [[ $# -ge 1 ]]; then
  exec python sharpa_teleop/viz/view_left_leader.py --left --ditto --leader-only "u2d2.usb_port=$1" hand_config=ditto_left_leader_only
else
  exec python sharpa_teleop/viz/view_left_leader.py --left --leader-only hand_config=ditto_left_leader_only
fi
