#!/usr/bin/env python3
"""
Calibration GUI for leader/follower motor pairs.

Features:
- Hydra config loading (same stack as teleop_controller.py)
- Click-to-enable independent motor pairs
- One-active-pair-at-a-time teleop
- Keyboard controls:
    - a: start/stop teleop loop with selected pairs
    - f: capture follower encoder values (raw counts)
    - l: capture leader encoder values (raw counts)
    - o: apply suggested offsets to offset entries
    - r: reset all offsets to zero
    - p: print current offsets to terminal
    - q: quit GUI
- Per-pair offset (counts) applied as effective zero-position shift
"""

import math
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import hydra
import tkinter as tk
from tkinter import ttk, messagebox
from omegaconf import DictConfig, OmegaConf

sys.path.append(str(Path(__file__).parent.parent))

from controllers.teleop_controller import TeleopHand


class CalibrationGUIController:
    """GUI calibration utility for teleoperated pair alignment."""

    def __init__(self, config: DictConfig):
        # Mutable copy so we can safely tweak behavior if needed later
        self.config = OmegaConf.create(OmegaConf.to_container(config, resolve=True))
        self.verbose = self.config.hand_config.get("verbose", self.config.get("verbose", True))

        self.teleop_hand: Optional[TeleopHand] = None
        self.root: Optional[tk.Tk] = None

        self.running = False
        self.control_thread: Optional[threading.Thread] = None
        self.io_lock = threading.Lock()
        self.control_frequency = float(self.config.hand_config.get("control_frequency", self.config.get("control_frequency", 50)))
        self.dt = 1.0 / max(1.0, self.control_frequency)

        self.leader_ids: List[int] = []
        self.follower_ids: List[int] = []
        self.num_pairs = 0

        self.enabled_flags: List[bool] = []
        self.offset_counts: List[int] = []

        self.enabled_vars: List[tk.BooleanVar] = []
        self.offset_vars: List[tk.IntVar] = []
        self.active_flags: List[bool] = []
        self.active_vars: List[tk.BooleanVar] = []

        self.base_zero_positions: List[int] = []
        self.leader_base_zero_positions: List[int] = []
        self.effective_zero_vars: List[tk.StringVar] = []
        self.suggested_offset_vars: List[tk.StringVar] = []
        self.follower_capture_vars: List[tk.StringVar] = []
        self.leader_capture_vars: List[tk.StringVar] = []

        self.status_var: Optional[tk.StringVar] = None

        # Tracks the hold position for non-active joints
        self.hold_follower_targets: List[float] = []
        self.last_leader_states = []
        self.last_follower_states = []

    def setup_interfaces(self) -> bool:
        """Connect to hardware and initialize motor metadata."""
        try:
            self.teleop_hand = TeleopHand(self.config)
        except Exception as exc:
            messagebox.showerror("Connection error", f"Failed to initialize TeleopHand:\n{exc}")
            return False

        self.leader_ids = list(self.teleop_hand.leader_chain)
        self.follower_ids = list(self.teleop_hand.follower_chain)
        self.num_pairs = min(len(self.leader_ids), len(self.follower_ids))
        self.leader_ids = self.leader_ids[: self.num_pairs]
        self.follower_ids = self.follower_ids[: self.num_pairs]

        if self.num_pairs == 0:
            messagebox.showerror("Invalid config", "No leader/follower pair found in configuration.")
            return False

        follower_converters = self.teleop_hand.follower_hand.joint_converters
        self.base_zero_positions = [int(follower_converters[i].zero_position) for i in range(self.num_pairs)]
        leader_converters = self.teleop_hand.leader_hand.joint_converters
        self.leader_base_zero_positions = [int(leader_converters[i].zero_position) for i in range(self.num_pairs)]

        self.enabled_flags = [False for _ in range(self.num_pairs)]
        self.offset_counts = [0 for _ in range(self.num_pairs)]
        self.active_flags = [False for _ in range(self.num_pairs)]

        return True

    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("Hand Calibration GUI")
        self.root.geometry("980x560")
        self.status_var = tk.StringVar(value="Status: Not connected")
        self.enabled_vars = [tk.BooleanVar(value=val) for val in self.enabled_flags]
        self.active_vars = [tk.BooleanVar(value=val) for val in self.active_flags]
        self.offset_vars = [tk.IntVar(value=val) for val in self.offset_counts]
        self.effective_zero_vars = [tk.StringVar(value=str(base + off)) for base, off in zip(self.base_zero_positions, self.offset_counts)]
        self.suggested_offset_vars = [tk.StringVar(value="-") for _ in range(self.num_pairs)]
        self.follower_capture_vars = [tk.StringVar(value="-") for _ in range(self.num_pairs)]
        self.leader_capture_vars = [tk.StringVar(value="-") for _ in range(self.num_pairs)]

        main = ttk.Frame(self.root, padding="10")
        main.pack(fill=tk.BOTH, expand=True)

        title = ttk.Label(main, text="Hand Pair Calibration", font=("Arial", 15, "bold"))
        title.pack(anchor=tk.W, pady=(0, 8))

        controls = ttk.Frame(main)
        controls.pack(fill=tk.X, pady=(0, 8))

        controls_row1 = ttk.Frame(controls)
        controls_row1.pack(fill=tk.X, anchor=tk.W, pady=(0, 4))
        controls_row2 = ttk.Frame(controls)
        controls_row2.pack(fill=tk.X, anchor=tk.W)

        ttk.Button(controls_row1, text="Start/Stop (a)", command=self.toggle_control).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(controls_row1, text="Capture Follower (f)", command=self.capture_follower).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(controls_row1, text="Capture Leader (l)", command=self.capture_leader).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(controls_row1, text="Apply Offsets (o)", command=self.apply_suggested_offsets).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(controls_row1, text="Reset Offsets (r)", command=self.reset_offsets).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(controls_row1, text="Print Offsets (p)", command=self.print_offsets).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(controls_row1, text="Quit (q)", command=self.on_close).pack(side=tk.LEFT, padx=(0, 8))

        ttk.Button(controls_row2, text="Enable all", command=self.enable_all_pairs).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(controls_row2, text="Disable all", command=self.disable_all_pairs).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(controls_row2, text="Activate all", command=self.activate_all_pairs).pack(side=tk.LEFT, padx=(12, 8))
        ttk.Button(controls_row2, text="Deactivate all", command=self.deactivate_all_pairs).pack(side=tk.LEFT, padx=(0, 8))

        ttk.Label(
            controls,
            text="Keys: a=start/stop, f=save follower encoders, l=save leader encoders, o=apply suggested offsets, r=reset offsets, p=print offsets, q=quit",
        ).pack(anchor=tk.W, pady=(4, 0))

        status_label = ttk.Label(main, textvariable=self.status_var)
        status_label.pack(anchor=tk.W, pady=(0, 8))

        table = ttk.LabelFrame(main, text="Motor Pairs", padding="8")
        table.pack(fill=tk.BOTH, expand=True)

        headers = ["Enable", "Active", "Leader ID", "Follower ID", "Base zero", "Offset", "Suggested offset", "Effective zero", "Follower enc", "Leader enc"]
        for c, header in enumerate(headers):
            ttk.Label(table, text=header, font=("Arial", 10, "bold")).grid(row=0, column=c, sticky=tk.W, padx=6, pady=4)

        for i in range(self.num_pairs):
            row = i + 1
            ttk.Checkbutton(table, variable=self.enabled_vars[i]).grid(row=row, column=0, padx=6, pady=3)
            ttk.Checkbutton(table, variable=self.active_vars[i], command=lambda idx=i: self._ensure_active_enabled(idx)).grid(
                row=row, column=1, padx=6, pady=3
            )
            ttk.Label(table, text=str(self.leader_ids[i])).grid(row=row, column=2, padx=6, pady=3, sticky=tk.W)
            ttk.Label(table, text=str(self.follower_ids[i])).grid(row=row, column=3, padx=6, pady=3, sticky=tk.W)
            ttk.Label(table, text=str(self.base_zero_positions[i])).grid(row=row, column=4, padx=6, pady=3, sticky=tk.W)

            spin = ttk.Spinbox(table, from_=-2000, to=2000, increment=1, width=8, textvariable=self.offset_vars[i])
            spin.grid(row=row, column=5, padx=6, pady=3, sticky=tk.W)
            self.offset_vars[i].trace_add("write", lambda *_args, idx=i: self._update_effective_zero(idx))

            ttk.Label(table, textvariable=self.suggested_offset_vars[i]).grid(row=row, column=6, padx=6, pady=3, sticky=tk.W)
            ttk.Label(table, textvariable=self.effective_zero_vars[i]).grid(row=row, column=7, padx=6, pady=3, sticky=tk.W)
            ttk.Label(table, textvariable=self.follower_capture_vars[i]).grid(row=row, column=8, padx=6, pady=3, sticky=tk.W)
            ttk.Label(table, textvariable=self.leader_capture_vars[i]).grid(row=row, column=9, padx=6, pady=3, sticky=tk.W)

        ttk.Label(
            main,
            text="Behavior: only the active enabled pair teleoperates; all other follower joints hold last value.",
        ).pack(anchor=tk.W, pady=(6, 0))

        self.root.bind("<KeyPress-a>", lambda _e: self.toggle_control())
        self.root.bind("<KeyPress-f>", lambda _e: self.capture_follower())
        self.root.bind("<KeyPress-l>", lambda _e: self.capture_leader())
        self.root.bind("<KeyPress-o>", lambda _e: self.apply_suggested_offsets())
        self.root.bind("<KeyPress-r>", lambda _e: self.reset_offsets())
        self.root.bind("<KeyPress-p>", lambda _e: self.print_offsets())
        self.root.bind("<KeyPress-q>", lambda _e: self.on_close())
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def enable_all_pairs(self):
        for v in self.enabled_vars:
            v.set(True)
        self.status_var.set("Status: Enabled all pairs")

    def disable_all_pairs(self):
        for v in self.enabled_vars:
            v.set(False)
        self.status_var.set("Status: Disabled all pairs")

    def activate_all_pairs(self):
        for v in self.active_vars:
            v.set(True)
        for v in self.enabled_vars:
            v.set(True)
        self.status_var.set("Status: Activated all pairs")

    def deactivate_all_pairs(self):
        for v in self.active_vars:
            v.set(False)
        self.status_var.set("Status: Deactivated all pairs")

    @staticmethod
    def _parse_int_or_none(value: str) -> Optional[int]:
        try:
            v = value.strip()
            if v == "" or v == "-":
                return None
            return int(v)
        except Exception:
            return None

    def _suggest_offset_for_pair(self, pair_index: int) -> Optional[int]:
        """
        Compute the follower zero-position offset (counts) that would make leader and follower match.

        Let:
          L = leader encoder raw position
          F = follower encoder raw position
          ZL = leader converter zero_position
          ZF = follower converter zero_position (base_zero_positions)

        We match relative counts:
          (L - ZL) == (F - (ZF + offset))

        => offset = F - ZF - (L - ZL)
        """
        if not self.follower_capture_vars or not self.leader_capture_vars:
            return None

        follower_enc = self._parse_int_or_none(self.follower_capture_vars[pair_index].get())
        leader_enc = self._parse_int_or_none(self.leader_capture_vars[pair_index].get())
        if follower_enc is None or leader_enc is None:
            return None

        follower_zero = self.base_zero_positions[pair_index]
        leader_zero = self.leader_base_zero_positions[pair_index]
        return int(follower_enc - follower_zero - (leader_enc - leader_zero))

    def _update_suggested_offsets(self):
        """Update per-row suggested offsets from the saved follower/leader encoders."""
        if not self.suggested_offset_vars:
            return
        for i in range(self.num_pairs):
            suggested = self._suggest_offset_for_pair(i)
            self.suggested_offset_vars[i].set(str(suggested) if suggested is not None else "-")

    def apply_suggested_offsets(self):
        """
        Copy per-row suggested offsets into the editable offset fields.
        The control loop consumes self.offset_vars each iteration, so this is
        applied live even while teleop is running.
        """
        applied_count = 0
        for i in range(self.num_pairs):
            suggested = self._suggest_offset_for_pair(i)
            if suggested is None:
                continue
            self.offset_vars[i].set(int(suggested))
            applied_count += 1

        if applied_count == 0:
            self.status_var.set("Status: No suggested offsets to apply")
        else:
            self.status_var.set(f"Status: Applied suggested offsets to {applied_count} pair(s)")

    def reset_offsets(self):
        """Reset all per-pair offsets to zero."""
        for i in range(self.num_pairs):
            self.offset_vars[i].set(0)
        self.status_var.set("Status: Reset all offsets to zero")

    def print_offsets(self):
        """Print current per-pair offsets (counts) to terminal."""
        offsets_by_follower_id = {self.follower_ids[i]: int(self.offset_vars[i].get()) for i in range(self.num_pairs)}
        print(f"Current offsets (follower_id -> counts): {offsets_by_follower_id}")
        self.status_var.set("Status: Printed offsets to terminal")

    def _ensure_active_enabled(self, idx: int):
        """If a pair is marked active, ensure it is enabled."""
        if 0 <= idx < self.num_pairs and self.active_vars[idx].get():
            self.enabled_vars[idx].set(True)

    def _update_effective_zero(self, idx: int):
        base = self.base_zero_positions[idx]
        offset = int(self.offset_vars[idx].get())
        self.effective_zero_vars[idx].set(str(base + offset))

    def _get_enabled_indices(self) -> List[int]:
        return [i for i in range(self.num_pairs) if self.enabled_vars[i].get()]

    def _get_active_enabled_indices(self) -> List[int]:
        enabled = set(self._get_enabled_indices())
        return [i for i in range(self.num_pairs) if (i in enabled and self.active_vars[i].get())]

    def _offset_counts_to_joint_radians(self, pair_index: int) -> float:
        """
        Convert offset in encoder counts to joint-space radians for the follower pair.
        This models changing zero_position by `offset`.
        """
        converter = self.teleop_hand.follower_hand.joint_converters[pair_index]
        counts = float(self.offset_vars[pair_index].get())
        return counts * converter.radians_per_count / (converter.transmission_ratio * converter.direction)

    def _initialize_hold_targets(self):
        with self.io_lock:
            follower_states = self.teleop_hand.follower_hand.read_states()
        self.hold_follower_targets = [state.joint_angle for state in follower_states[: self.num_pairs]]

    def toggle_control(self):
        if self.running:
            self.running = False
            self.status_var.set("Status: Stopped")
            return

        enabled = self._get_enabled_indices()
        active_enabled = self._get_active_enabled_indices()
        if not active_enabled:
            messagebox.showwarning("No active pairs", "Mark at least one pair Active (and Enabled) before pressing 'a'.")
            return

        try:
            self._initialize_hold_targets()
        except Exception as exc:
            messagebox.showerror("State read error", f"Could not read follower states:\n{exc}")
            return

        self.running = True
        self.status_var.set(f"Status: Running ({len(active_enabled)} active)")
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

    def _control_loop(self):
        while self.running:
            loop_start = time.time()
            try:
                with self.io_lock:
                    leader_states = self.teleop_hand.leader_hand.read_states()
                    follower_states = self.teleop_hand.follower_hand.read_states()

                if len(leader_states) < self.num_pairs or len(follower_states) < self.num_pairs:
                    continue

                self.last_leader_states = leader_states
                self.last_follower_states = follower_states

                active_indices = self._get_active_enabled_indices()
                if not active_indices:
                    continue

                # Update hold targets from real follower state to avoid jumps/drift
                for i in range(self.num_pairs):
                    self.hold_follower_targets[i] = follower_states[i].joint_angle

                # Multi-pair teleop:
                # command all followers, but only active pairs track leader (+ offset)
                command = list(self.hold_follower_targets)
                for idx in active_indices:
                    offset_rad = self._offset_counts_to_joint_radians(idx)
                    command[idx] = leader_states[idx].joint_angle + offset_rad
                with self.io_lock:
                    self.teleop_hand.follower_hand.send_position_commands(command)

                # Preserve current-based leader behavior from configured teleop mode
                if self.teleop_hand.leader_mode == "current":
                    current_cmds = self.teleop_hand.current_controller.compute_bulk_current_commands(
                        leader_states,
                        self.teleop_hand.leader_control_params,
                        time.time(),
                        follower_states,
                        self.teleop_hand.show_current_breakdown,
                    )
                    with self.io_lock:
                        self.teleop_hand.leader_hand.send_current_commands(current_cmds)

            except Exception as exc:
                if self.verbose and self.running:
                    print(f"[Calibration loop] {exc}")

            elapsed = time.time() - loop_start
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def capture_follower(self):
        """Capture current follower raw encoder values and store in GUI."""
        if not self.teleop_hand:
            return
        try:
            enabled = set(self._get_enabled_indices())
            if self.running and self.last_follower_states:
                follower_states = self.last_follower_states
            else:
                with self.io_lock:
                    follower_states = self.teleop_hand.follower_hand.read_states()
            for i in range(min(self.num_pairs, len(follower_states))):
                if i not in enabled:
                    continue
                enc = int(follower_states[i].raw_position)
                self.follower_capture_vars[i].set(str(enc))

            values = {
                self.follower_ids[i]: int(follower_states[i].raw_position)
                for i in range(min(self.num_pairs, len(follower_states)))
                if i in enabled
            }
            print(f"Follower encoder values: {values}")
            self.status_var.set("Status: Follower encoders captured")
            self._update_suggested_offsets()
        except Exception as exc:
            messagebox.showerror("Capture error", f"Failed to capture follower encoders:\n{exc}")

    def capture_leader(self):
        """Capture current leader raw encoder values and store in GUI."""
        if not self.teleop_hand:
            return
        try:
            enabled = set(self._get_enabled_indices())
            if self.running and self.last_leader_states:
                leader_states = self.last_leader_states
            else:
                with self.io_lock:
                    leader_states = self.teleop_hand.leader_hand.read_states()
            for i in range(min(self.num_pairs, len(leader_states))):
                if i not in enabled:
                    continue
                enc = int(leader_states[i].raw_position)
                self.leader_capture_vars[i].set(str(enc))

            values = {
                self.leader_ids[i]: int(leader_states[i].raw_position)
                for i in range(min(self.num_pairs, len(leader_states)))
                if i in enabled
            }
            print(f"Leader encoder values: {values}")
            self.status_var.set("Status: Leader encoders captured")
            self._update_suggested_offsets()
        except Exception as exc:
            messagebox.showerror("Capture error", f"Failed to capture leader encoders:\n{exc}")

    def on_close(self):
        self.running = False
        time.sleep(0.1)
        if self.teleop_hand is not None:
            self.teleop_hand.disconnect()
            self.teleop_hand = None
        if self.root is not None:
            self.root.destroy()

    def run(self):
        if not self.setup_interfaces():
            return
        self.create_gui()
        self.status_var.set("Status: Ready (enable pairs, choose active, press 'a')")
        self.root.mainloop()


def main():
    config_path = Path(__file__).parent.parent / "hand_interfaces" / "conf"

    @hydra.main(version_base=None, config_path=str(config_path), config_name="config")
    def run_calibration(cfg: DictConfig) -> None:
        if TeleopHand.is_idle_hand_config(cfg):
            print(
                "Calibration needs leader/follower pairs. "
                "Default Hydra stack uses hand_config=none.\n"
                "Example: python controllers/calibration_gui_controller.py hand_config=7dof/7dof",
                file=sys.stderr,
            )
            sys.exit(2)
        print(f"Calibration GUI - {cfg.config_name}")
        print(f"Description: {cfg.description}")
        print("------------------------------------------------------------")
        controller = CalibrationGUIController(cfg)
        controller.run()

    run_calibration()


if __name__ == "__main__":
    main()
