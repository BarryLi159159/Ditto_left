# Left-hand Ditto leader — live viewer

Shows the left Ditto leader (index + 3-motor thumb) moving in a 3D viewer
(viser) while you move the glove. Encoders are read only; motor torque stays off.

## What is in here
- `ditto_sharpa/sharpa_teleop/` — viewer (`viz/view_left_leader.py`), hardware
  session, configs, and the mirrored left-hand model
  `retargeting/assets/ditto_left_leader/` (see its README for how it was made).
- `ditto_sharpa/sharpa_teleop/retargeting/assets/sharpa_right/` — Sharpa hand model; the viewer requires it to exist even in leader-only mode (not loaded).
- `ditto_sharpa/ditto/` — Dynamixel leader-hand interface used by the session.
- `dynamixel_u2d2/` — U2D2 serial interface package (installed with pip below).
- `requirements.txt` — pinned Python deps. `run_viewer.sh` — one-line launcher.
- `left-hand-branch.patch` — the same files as a git patch, for anyone who
  already has the ditto-x repo (`git am left-hand-branch.patch` on top of
  origin/mac-remote).

## Setup (Python 3.11)
    uv venv --python 3.11 .venv && source .venv/bin/activate   # or python3.11 -m venv .venv
    pip install -r requirements.txt
    pip install -e dynamixel_u2d2

## Run
    ./run_viewer.sh /dev/cu.usbserial-XXXX     # macOS: ls /dev/cu.usbserial-*
    ./run_viewer.sh /dev/ttyUSB0               # Linux (add yourself to the dialout group)
    ./run_viewer.sh                            # no glove connected: sliders only

Then open http://127.0.0.1:8080 in a browser ON THE SAME MACHINE (the server
binds to localhost only; from another machine use an SSH tunnel:
`ssh -L 8080:127.0.0.1:8080 user@host`).

## Hardware notes
- Motors: XC330, Protocol 2.0, **4 Mbps**, IDs thumb 111/112/114, index 121/122/123.
- Power the motor chain from **12 V** (5 V is below the XC330 minimum: motors
  stay silent on the bus while the U2D2 still shows up on USB).
- Cable goes in the U2D2's 3-pin TTL connector.

## Calibration
`conf/hand_config/ditto_left_leader_only.yaml` holds per-motor
`zero_position` (encoder counts at the model's zero pose) and `direction`.
These numbers are specific to the glove they were measured on. For a different
left glove: run the viewer, open the "Zero calibration" panel, drag the sliders
until the model matches your hand, click "Print zero_position for yaml", and
copy the printed values into the yaml. If a joint moves the wrong way, flip its
`direction`.

The model's thumb matches the 3-motor hardware: three segments driven by
`thumb_joint_0/1/3` (motors 111/112/114), mirrored from the latest right-hand
CAD export. Its kinematics equal the earlier 4-segment model, so existing
calibrations carry over.

## Checking the model against the printed parts
The viewer's "Index-thumb gap (model)" panel shows, live, the closest gap
between the printed distal parts of the index and thumb in the model (plus the
pad- and tip-frame distances). Calibrate the zeros first, then hold a pose (for
example a pinch), measure the real gap between the same printed parts with a
caliper, type it in "Caliper gap", and click "Log this pose". Rows go to
`ditto_sharpa/pad_gap_log.csv` (joint angles, model gap, caliper gap) for
deciding which link lengths in the URDF to adjust. Gaps above 10 mm are shown
coarse (~1 mm); overlapping parts read as 0.
