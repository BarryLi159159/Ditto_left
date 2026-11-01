#!/usr/bin/env python3
"""
Deadband-focused live plot for Ditto–Sharpa force rendering.

Per leader joint (``plot_joint: true``) we plot:
  Left:  filtered torque signal expressed as the virtual follower current
         (mA), with the force-rendering deadband thresholds as horizontal lines.
  Right: base force-rendering damping current (mA) and the net leader current
         command (mA).

Axes are intentionally small and fixed per joint:
  y-limits are ``±(force_rendering_max_current + 20)`` (from the joint config).
This avoids autoscale jitter and keeps Matplotlib responsive.
"""

from __future__ import annotations

import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

_PKG = Path(__file__).resolve().parent.parent
_REPO = _PKG.parent
for _path in (_PKG, _REPO):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import matplotlib.animation as animation  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from hydra import compose, initialize_config_dir  # noqa: E402

from _paths import CONF_DIR, DITTO_CONF_DIR  # noqa: E402
from retargeting.paths import (  # noqa: E402
    DITTO_3F_LEADER_JOINT_NAMES,
    DITTO_3F_LEADER_MOTOR_IDS,
)
from teleop.ditto_sharpa_teleop import DittoSharpaTeleop, finger_modes_from_config  # noqa: E402

_MOTOR_TO_JOINT = dict(zip(DITTO_3F_LEADER_MOTOR_IDS, DITTO_3F_LEADER_JOINT_NAMES))

DEFAULT_HAND_CONFIG = "ditto_2f_tactile"


@dataclass
class RunFlags:
    retarget_hz: float = 40.0


def _strip_run_flags() -> RunFlags:
    flags = RunFlags()
    remaining = [sys.argv[0]]
    args = iter(sys.argv[1:])
    for arg in args:
        if arg == "--retarget-rate":
            flags.retarget_hz = float(next(args))
        elif arg.startswith("--retarget-rate="):
            flags.retarget_hz = float(arg.split("=", 1)[1])
        else:
            remaining.append(arg)
    sys.argv = remaining
    return flags


class DeadbandPlotter:
    def __init__(self, config, flags: RunFlags):
        self.config = config
        self.flags = flags

        # Downsample traces so Matplotlib stays responsive.
        self.plot_window_seconds = 6.0
        self._plot_dt = 0.04  # ~25 Hz traces
        self.max_history = int(self.plot_window_seconds / self._plot_dt)

        self.finger_modes = finger_modes_from_config(config)

        self.leader_chain = list(config.hand_config.leader.motor_ids)
        self.num_joints = len(self.leader_chain)
        self.labels = [
            f"M{m} ({_MOTOR_TO_JOINT.get(m, '?')})" for m in self.leader_chain
        ]

        self.plot_indices = self._plot_indices()
        self.num_rows = len(self.plot_indices)
        self.t_axis = np.linspace(
            -self.plot_window_seconds,
            0.0,
            self.max_history,
            dtype=np.float32,
        )

        # Per-motor current deadband thresholds and y-scale.
        self.thresholds_pos: list[float] = []
        self.thresholds_neg: list[float] = []
        self.left_y_lims: list[tuple[float, float]] = []
        self.right_y_lims: list[tuple[float, float]] = []
        self._read_per_joint_current_params()

        n = self.num_joints
        # Data deques store per-*leader chain index* values.
        self.synth_filt_mA = [
            deque([0.0] * self.max_history, maxlen=self.max_history)
            for _ in range(n)
        ]
        self.damping_mA = [
            deque([0.0] * self.max_history, maxlen=self.max_history)
            for _ in range(n)
        ]
        self.net_cmd_mA = [
            deque([0.0] * self.max_history, maxlen=self.max_history)
            for _ in range(n)
        ]

        self.state_queue: queue.Queue = queue.Queue(maxsize=2)
        self.running = False
        self.controller: DittoSharpaTeleop | None = None

        self._build_controller()
        self._setup_plots()

    def _plot_indices(self) -> list[int]:
        flags: dict[int, bool] = {}
        leader_cfg = self.config.hand_config.leader
        if leader_cfg.get("joint_settings"):
            for jc in leader_cfg.joint_settings:
                mid = jc.get("motor_id")
                if mid is not None:
                    flags[mid] = bool(jc.get("plot_joint", True))
        idxs = [i for i, m in enumerate(self.leader_chain) if flags.get(m, True)]
        if not idxs:
            print("  [plot] no joints have plot_joint=true; plotting all")
            return list(range(self.num_joints))
        skipped = [m for i, m in enumerate(self.leader_chain) if i not in idxs]
        if skipped:
            print(f"  [plot] skipping joints (plot_joint=false): {skipped}")
        return idxs

    def _read_per_joint_current_params(self) -> None:
        """
        Extract per-motor current deadband thresholds and max current.

        These config keys are in *mA* because the current controller thresholds
        compare against the follower current command.
        """
        leader_cfg = self.config.hand_config.leader
        # Defaults if something is missing.
        max_curr_default = 50.0
        pos_default = 50.0
        neg_default = 50.0

        pos_map: dict[int, float] = {}
        neg_map: dict[int, float] = {}
        max_map: dict[int, float] = {}

        if leader_cfg.get("joint_settings"):
            for jc in leader_cfg.joint_settings:
                mid = jc.get("motor_id")
                cc = jc.get("current_control") if jc is not None else None
                if mid is None or cc is None:
                    continue
                base = float(cc.get("force_rendering_threshold", pos_default))
                pos_map[int(mid)] = float(cc.get("force_rendering_threshold_positive", base))
                neg_map[int(mid)] = float(cc.get("force_rendering_threshold_negative", base))
                max_map[int(mid)] = float(cc.get("force_rendering_max_current", max_curr_default))

        self.thresholds_pos = []
        self.thresholds_neg = []
        self.left_y_lims = []
        self.right_y_lims = []
        for row, j in enumerate(self.plot_indices):
            motor_id = int(self.leader_chain[j])
            tpos = float(pos_map.get(motor_id, pos_default))
            tneg = float(neg_map.get(motor_id, neg_default))
            max_curr = float(max_map.get(motor_id, max_curr_default))
            # Left axis: deadband-centric scale.
            left_y = max(tpos, tneg) + 100.0
            # Right axis: render-centric scale.
            right_y = max_curr + 20.0
            self.thresholds_pos.append(tpos)
            self.thresholds_neg.append(tneg)
            self.left_y_lims.append((-left_y, left_y))
            self.right_y_lims.append((-right_y, right_y))

    def _build_controller(self) -> None:
        # Match the behavior of run_teleop.py / the earlier light plot:
        # always construct the Sharpa follower session so we can fetch
        # measured torques / tactile frames as needed.
        from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession

        self.controller = DittoSharpaTeleop(
            self.config,
            sharpa_follower=SharpaFollowerSession(self.config, verbose=True),
            retarget_hz=self.flags.retarget_hz,
            state_queue=self.state_queue,
        )

    def _setup_plots(self) -> None:
        self.fig, axes = plt.subplots(
            self.num_rows, 2, figsize=(12, 2.8 * self.num_rows), squeeze=False
        )
        self.axes = axes

        self.left_lines = []
        self.right_damp_lines = []
        self.right_net_lines = []

        for row, j in enumerate(self.plot_indices):
            left_y_lo, left_y_hi = self.left_y_lims[row]
            right_y_lo, right_y_hi = self.right_y_lims[row]

            # Left: synthetic follower current from filtered torque.
            ax_l = self.axes[row, 0]
            ax_l.set_title(f"{self.labels[j]} — filtered torque→I (mA)")
            ax_l.set_ylabel("mA")
            ax_l.grid(True, alpha=0.3)
            ax_l.axhline(y=0, color="k", linestyle="-", alpha=0.3)
            ax_l.axhline(y=self.thresholds_pos[row], color="r", linestyle="--", alpha=0.6)
            ax_l.axhline(y=-self.thresholds_neg[row], color="r", linestyle="--", alpha=0.6)
            ax_l.set_ylim(left_y_lo, left_y_hi)

            (l_line,) = ax_l.plot(
                self.t_axis,
                np.zeros_like(self.t_axis),
                color="orange",
                linewidth=1.6,
                label="filtered (from torque)",
            )
            self.left_lines.append(l_line)

            # Right: base damping + net command.
            ax_r = self.axes[row, 1]
            ax_r.set_title(f"{self.labels[j]} — damping + net (mA)")
            ax_r.grid(True, alpha=0.3)
            ax_r.axhline(y=0, color="k", linestyle="-", alpha=0.3)
            ax_r.set_ylim(right_y_lo, right_y_hi)

            (d_line,) = ax_r.plot(
                self.t_axis,
                np.zeros_like(self.t_axis),
                color="purple",
                linestyle=":",
                linewidth=1.8,
                label="base damping",
            )
            (n_line,) = ax_r.plot(
                self.t_axis,
                np.zeros_like(self.t_axis),
                color="c",
                linestyle="-",
                linewidth=1.6,
                label="net leader cmd",
            )
            self.right_damp_lines.append(d_line)
            self.right_net_lines.append(n_line)

            if row == self.num_rows - 1:
                ax_l.set_xlabel("s")
                ax_r.set_xlabel("s")

        # One legend for the right column (left can be noisy per-row).
        self.axes[0, 1].legend(loc="upper right", fontsize=8)
        plt.tight_layout()

    def _data_thread(self) -> None:
        last_plot_t = -self._plot_dt
        while self.running:
            try:
                state = self.state_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            # Drain to keep freshest sample.
            while True:
                try:
                    state = self.state_queue.get_nowait()
                except queue.Empty:
                    break

            now = time.monotonic()
            if now - last_plot_t < self._plot_dt:
                continue
            last_plot_t = now

            ld = state.get("leader", {})
            fo = state.get("follower", {})
            for j in self.plot_indices:
                self.synth_filt_mA[j].append(
                    fo.get("currents", [0.0])[j] if j < len(fo.get("currents", [])) else 0.0
                )
                self.damping_mA[j].append(
                    ld.get("force_rendering_damping_currents", [0.0])[j]
                    if j < len(ld.get("force_rendering_damping_currents", []))
                    else 0.0
                )
                self.net_cmd_mA[j].append(
                    ld.get("command_currents", [0.0])[j]
                    if j < len(ld.get("command_currents", []))
                    else 0.0
                )

    def _update(self, _frame):
        if not self.running:
            return []
        artists = []
        for row, j in enumerate(self.plot_indices):
            synth = np.fromiter(self.synth_filt_mA[j], dtype=np.float32, count=self.max_history)
            damp = np.fromiter(self.damping_mA[j], dtype=np.float32, count=self.max_history)
            net = np.fromiter(self.net_cmd_mA[j], dtype=np.float32, count=self.max_history)

            self.left_lines[row].set_ydata(synth)
            self.right_damp_lines[row].set_ydata(damp)
            self.right_net_lines[row].set_ydata(net)
            artists.extend(
                (self.left_lines[row], self.right_damp_lines[row], self.right_net_lines[row])
            )
        return artists

    def run(self) -> None:
        assert self.controller is not None
        print("Starting deadband-focused live plot")
        print(f"  Leader motors: {self.leader_chain}")
        print(f"  Plot joints: {len(self.plot_indices)} row(s)")
        modes_str = ", ".join(
            f"{f} (pos:{m['position']}, force:{m['force']})" for f, m in self.finger_modes.items()
        )
        print(f"  Per-finger mode: {modes_str}")
        print("  Close the plot window to stop.\n")

        self.controller.connect()
        self.controller.setup_motors()
        self.running = True

        control_thread = threading.Thread(
            target=self.controller.start_control_loop, daemon=True
        )
        control_thread.start()

        data_thread = threading.Thread(target=self._data_thread, daemon=True)
        data_thread.start()

        time.sleep(0.5)
        _ani = animation.FuncAnimation(
            self.fig,
            self._update,
            interval=50,
            blit=True,
            cache_frame_data=False,
        )
        plt.show()

        print("\nStopping deadband plot...")
        self.running = False
        self.controller.stop()
        control_thread.join(timeout=2.0)
        data_thread.join(timeout=1.0)
        self.controller.disconnect()
        print("Stopped.")


def main() -> None:
    flags = _strip_run_flags()
    searchpath = f"hydra.searchpath=[file://{DITTO_CONF_DIR}]"
    overrides = [searchpath]
    if not any(arg.startswith("hand_config=") for arg in sys.argv[1:]):
        overrides.append(f"hand_config={DEFAULT_HAND_CONFIG}")
    overrides += sys.argv[1:]

    with initialize_config_dir(version_base=None, config_dir=str(CONF_DIR)):
        cfg = compose(config_name="config", overrides=overrides)

    DeadbandPlotter(cfg, flags).run()


if __name__ == "__main__":
    main()

