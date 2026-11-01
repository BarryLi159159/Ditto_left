"""Run sharpa_teleop's view_teleop.py with a partial leader chain.

poll_joint_angles() returns one angle per *connected* motor, but the viewer
indexes it as the full URDF q. Scatter the subset into the full joint order,
leaving missing joints at 0, so the viewer can render.

``--left`` swaps in the mirrored left-hand model
(retargeting/assets/ditto_left_leader, index + thumb) and adds a
"zero calibration" GUI folder: per-joint offset sliders (deg) applied live,
and a button that prints the equivalent zero_position counts for the
hand_config yaml. Run it without ``--3f``.

The viser server is pinned to 127.0.0.1.

Usage (from ditto_sharpa/):
    python sharpa_teleop/viz/view_left_leader.py --left --ditto --leader-only \
        u2d2.usb_port=/dev/cu.usbserial-XXXX hand_config=ditto_left_leader_only
    python sharpa_teleop/viz/view_left_leader.py --left --leader-only      # sliders only
    python sharpa_teleop/viz/view_left_leader.py --3f --ditto --leader-only u2d2.usb_port=...   # right hand
"""
import re
import runpy
import sys
from pathlib import Path

import numpy as np
import viser
import yaml

pkg = Path(__file__).resolve().parent.parent
repo = pkg.parent
for p in (pkg, repo):
    sys.path.insert(0, str(p))

left = "--left" in sys.argv
sys.argv = [a for a in sys.argv if a != "--left"]

from hardware_interfaces.ditto_leader import session as S  # noqa: E402
from retargeting.paths import ASSETS_DIR, DITTO_3F_LEADER_JOINT_NAMES, DITTO_3F_THUMB_JOINT_NAMES, DITTO_INDEX_JOINT_NAMES  # noqa: E402

FULL_JOINT_NAMES = (*DITTO_INDEX_JOINT_NAMES, *DITTO_3F_THUMB_JOINT_NAMES) if left else DITTO_3F_LEADER_JOINT_NAMES
MOTOR_FOR_JOINT = dict(zip(DITTO_3F_LEADER_JOINT_NAMES, (121, 122, 123, 131, 132, 133, 111, 112, 114)))

if left:
    import viz.view_assets as VA  # noqa: E402

    LEFT_URDF = ASSETS_DIR / "ditto_left_leader" / "ditto_left_leader.urdf"
    VA.ditto_leader_urdf = lambda *args, **kwargs: LEFT_URDF

# Which hand_config is in use (for printing zero_position with the right direction).
hc_name = next((a.split("=", 1)[1] for a in sys.argv if a.startswith("hand_config=")), None)
hc = yaml.safe_load((pkg / "conf" / "hand_config" / f"{hc_name}.yaml").read_text()) if hc_name else None
motor_cfg = {j["motor_id"]: j["joint_settings"] for j in (hc or {}).get("leader", {}).get("joint_settings", [])}

_server = {"obj": None}
_orig_init = viser.ViserServer.__init__


def _localhost_init(self, host="127.0.0.1", port=8080, *args, **kwargs):
    _orig_init(self, "127.0.0.1", port, *args, **kwargs)
    _server["obj"] = self


viser.ViserServer.__init__ = _localhost_init

_gui = {"sliders": None}
COUNTS_PER_DEG = 4096 / 360.0


def _ensure_gui(joint_names):
    if _gui["sliders"] is not None or _server["obj"] is None:
        return
    server = _server["obj"]
    with server.gui.add_folder("Zero calibration (deg offset, live)"):
        sliders = {n: server.gui.add_slider(f"{n} (motor {MOTOR_FOR_JOINT[n]})", min=-150.0, max=150.0, step=1.0, initial_value=0.0) for n in joint_names}
        btn = server.gui.add_button("Print zero_position for yaml")
        live = server.gui.add_button("Print current joint angles")
    _gui["sliders"] = sliders; print("zero-calibration GUI ready:", list(sliders), flush=True)
    _gui["last"] = {}

    @btn.on_click
    def _(_):
        print("\n=== zero_position for ditto_left_leader_only.yaml ===", flush=True)
        for n, s in sliders.items():
            m = MOTOR_FOR_JOINT[n]; cfg = motor_cfg.get(m, {})
            zero, direction = cfg.get("zero_position", 2048), cfg.get("direction", 1)
            new_zero = int(round(zero - direction * s.value * COUNTS_PER_DEG))
            print(f"  motor {m} ({n}): offset {s.value:+.0f} deg -> zero_position: {new_zero}  (was {zero}, direction {direction})", flush=True)
        print("=== end ===\n", flush=True)

    @live.on_click
    def _(_):
        print("current joint angles (deg, after offsets):", {n: round(float(np.rad2deg(v)), 1) for n, v in _gui["last"].items()}, flush=True)


_orig_poll = S.LeaderHardwareSession.poll_joint_angles


def _poll_full(self):
    angles = _orig_poll(self)
    if angles is None:
        return None
    _ensure_gui(list(self.leader_joint_names))
    sliders = _gui["sliders"] or {}
    adjusted = [float(a) + np.deg2rad(sliders[n].value) if n in sliders else float(a) for n, a in zip(self.leader_joint_names, angles)]
    _gui["last"] = dict(zip(self.leader_joint_names, adjusted))
    if len(adjusted) == len(FULL_JOINT_NAMES):
        return np.array(adjusted)
    full = np.zeros(len(FULL_JOINT_NAMES), dtype=float)
    for name, value in zip(self.leader_joint_names, adjusted):
        full[FULL_JOINT_NAMES.index(name)] = value
    return full


S.LeaderHardwareSession.poll_joint_angles = _poll_full

script = pkg / "viz" / "view_teleop.py"
sys.argv = [str(script)] + sys.argv[1:]
runpy.run_path(str(script), run_name="__main__")
