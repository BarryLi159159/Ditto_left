#!/usr/bin/env python3
"""Live plot for Ditto–Sharpa force rendering.

Compares, per leader joint (``plot_joint: true``):
  torque        : filtered estimated joint torque (Nm)
  force render  : force-rendering current and damping (mA)

Keep this plot light: two columns, few traces, short window. The old 3-column
view (raw+filt+measured+cmd) was too heavy for a live teleop loop.

Usage (from sharpa_teleop repo root):
    python sharpa_teleop/viz/force_plot.py
    python sharpa_teleop/viz/force_plot.py hand_config=ditto_3f_tactile
    python sharpa_teleop/viz/force_plot.py u2d2.usb_port=/dev/ttyUSB0

Control mode is config-only (``hand_config.control.fingers``). The Sharpa
follower is always connected (same as ``run_teleop.py``).

Raise gains at runtime, e.g.:
    ... hand_config.leader.joint_settings.1.current_control.force_rendering_gain=0.05
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
    DITTO_LEADER_JOINT_NAMES,
    DITTO_LEADER_MOTOR_IDS,
)
from teleop.ditto_sharpa_teleop import (  # noqa: E402
    DittoSharpaTeleop,
    finger_modes_from_config,
)

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


class ForceRenderPlotter:
    """Light live plot: filtered torque + rendered current, one row per joint."""

    def __init__(self, config, flags: RunFlags):
        self.config = config
        self.flags = flags
        self.plot_window_seconds = 4.0
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
            -self.plot_window_seconds, 0.0, self.max_history, dtype=np.float32
        )

        n = self.num_joints
        self.tau_filt = [deque([0.0] * self.max_history, maxlen=self.max_history) for _ in range(n)]
        self.force_render = [deque([0.0] * self.max_history, maxlen=self.max_history) for _ in range(n)]
        self.damping = [deque([0.0] * self.max_history, maxlen=self.max_history) for _ in range(n)]

        self.state_queue: queue.Queue = queue.Queue(maxsize=2)
        self.running = False
        self.controller: DittoSharpaTeleop | None = None

        self._build_controller()
        self._setup_plots()

    def _plot_indices(self) -> list[int]:
        """Chain indices to draw: leader joints with plot_joint != false (default true)."""
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

    def _build_controller(self) -> None:
        from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession

        self.controller = DittoSharpaTeleop(
            self.config,
            sharpa_follower=SharpaFollowerSession(self.config, verbose=True),
            retarget_hz=self.flags.retarget_hz,
            state_queue=self.state_queue,
        )

    def _setup_plots(self) -> None:
        self.fig, axes = plt.subplots(
            self.num_rows, 2, figsize=(10, 2.6 * self.num_rows), squeeze=False
        )
        self.axes = axes
        self.tau_lines, self.fr_lines, self.damp_lines = [], [], []

        for row, j in enumerate(self.plot_indices):
            ax_t = self.axes[row, 0]
            ax_t.set_title(f"{self.labels[j]} — torque (Nm)")
            ax_t.set_ylabel("Nm")
            ax_t.set_xlim(-self.plot_window_seconds, 0.0)
            ax_t.set_ylim(-1.0, 1.0)
            ax_t.grid(True, alpha=0.3)
            ax_t.axhline(y=0, color="k", linestyle="-", alpha=0.3)
            (lt,) = ax_t.plot(self.t_axis, np.zeros(self.max_history), color="orange", linewidth=1.5)
            self.tau_lines.append(lt)

            ax_f = self.axes[row, 1]
            ax_f.set_title(f"{self.labels[j]} — rendered (mA)")
            ax_f.set_ylabel("mA")
            ax_f.set_xlim(-self.plot_window_seconds, 0.0)
            ax_f.set_ylim(-80.0, 80.0)
            ax_f.grid(True, alpha=0.3)
            ax_f.axhline(y=0, color="k", linestyle="-", alpha=0.3)
            (lf,) = ax_f.plot(
                self.t_axis, np.zeros(self.max_history), color="crimson", label="force", linewidth=1.5
            )
            (ld,) = ax_f.plot(
                self.t_axis, np.zeros(self.max_history), color="purple", linestyle=":", label="damp", linewidth=1.3
            )
            self.fr_lines.append(lf)
            self.damp_lines.append(ld)

            if row == self.num_rows - 1:
                ax_t.set_xlabel("s")
                ax_f.set_xlabel("s")

        self.axes[0, 1].legend(loc="upper right", fontsize=8)
        plt.tight_layout()

    def _data_thread(self) -> None:
        last_plot_t = -self._plot_dt
        while self.running:
            try:
                state = self.state_queue.get(timeout=0.1)
            except queue.Empty:
                continue
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
                self._push(self.tau_filt[j], fo.get("torques_nm_filtered", []), j)
                self._push(self.force_render[j], ld.get("force_rendering_currents", []), j)
                self._push(self.damping[j], ld.get("force_rendering_damping_currents", []), j)

    @staticmethod
    def _push(dq: deque, values: list, idx: int) -> None:
        dq.append(values[idx] if idx < len(values) else 0.0)

    def _update(self, _frame):
        if not self.running:
            return []
        artists = []
        for row, j in enumerate(self.plot_indices):
            tau = np.fromiter(self.tau_filt[j], dtype=np.float32, count=self.max_history)
            fr = np.fromiter(self.force_render[j], dtype=np.float32, count=self.max_history)
            damp = np.fromiter(self.damping[j], dtype=np.float32, count=self.max_history)
            self.tau_lines[row].set_ydata(tau)
            self.fr_lines[row].set_ydata(fr)
            self.damp_lines[row].set_ydata(damp)
            artists.extend((self.tau_lines[row], self.fr_lines[row], self.damp_lines[row]))
        return artists

    def run(self) -> None:
        assert self.controller is not None
        print("Starting force-render live plot")
        print(f"  Leader motors: {self.leader_chain}")
        print(f"  Sharpa: on")
        print(f"  Plot: {self.num_rows} joint(s), 2 columns, {self.plot_window_seconds:.0f}s window")
        modes_str = ", ".join(
            f"{f} (pos:{m['position']}, force:{m['force']})"
            for f, m in self.finger_modes.items()
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

        print("\nStopping force-render plot...")
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

    ForceRenderPlotter(cfg, flags).run()


if __name__ == "__main__":
    main()
