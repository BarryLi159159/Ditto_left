#!/usr/bin/env python3
"""
Live Plot Tool

Live plotting of leader and follower motor states to diagnose oscillations.
Plots current, velocity, and position for each joint in real-time.

Usage:
    python live_plot.py hand_config=2dof/2dof_parallel_xc330_to_xm430

Optional flags to show only specific plots (can combine):
    python live_plot.py --position hand_config=...
    python live_plot.py --current --velocity hand_config=...
"""

import time
import queue
import threading
import hydra
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path
from omegaconf import DictConfig
from collections import deque

# Add the hand_interfaces module to the path
import sys
sys.path.append(str(Path(__file__).parent.parent))

from controllers.teleop_controller import TeleopHand


class LivePlotter:
    """Live plotter for visualizing motor states in real-time."""

    # Supported plot types and their column labels
    PLOT_TYPES = ["current", "velocity", "position"]

    def __init__(self, config: DictConfig, max_history: int = 2000, plot_window_seconds: float = 10.0,
                 plot_types: list = None):
        """
        Initialize the live plotter.

        Args:
            config: Hydra configuration object
            max_history: Maximum number of data points to keep in history (reduced for performance)
            plot_window_seconds: Number of seconds of data to display in plots
            plot_types: List of plot types to show ("current", "velocity", "position").
                        If None or empty, all three are shown.
        """
        self.config = config
        self.max_history = max_history
        self.plot_window_seconds = plot_window_seconds

        # Determine which plot types to show
        if plot_types:
            self.plot_types = [p for p in plot_types if p in self.PLOT_TYPES]
        else:
            self.plot_types = list(self.PLOT_TYPES)
        self.num_plot_types = len(self.plot_types)

        # Get motor IDs and configuration
        self.leader_chain = list(config.hand_config.leader.motor_ids)
        self.follower_chain = list(config.hand_config.follower.motor_ids)
        self.num_joints = len(self.follower_chain)

        leader_config = config.hand_config.leader

        # Get force rendering thresholds from config, matching by motor_id to leader_chain order
        self.thresholds = []
        threshold_map = {}
        if hasattr(leader_config, "joint_settings") and leader_config.joint_settings:
            for joint_config in leader_config.joint_settings:
                motor_id = joint_config.get("motor_id")
                if motor_id is not None and "current_control" in joint_config:
                    cc = joint_config["current_control"]
                    threshold = cc.get("force_rendering_threshold", 50.0)
                    threshold_map[motor_id] = threshold

        for leader_motor_id in self.leader_chain:
            threshold = threshold_map.get(leader_motor_id, 50.0)
            self.thresholds.append(threshold)

        while len(self.thresholds) < self.num_joints:
            self.thresholds.append(50.0)

        # Data storage (using deque for efficient appending/popping)
        self.time_data = deque(maxlen=max_history)
        self.leader_currents = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.follower_currents = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.leader_velocities = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.follower_velocities = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.leader_positions = [deque(maxlen=max_history) for _ in range(self.num_joints)]
        self.follower_positions = [deque(maxlen=max_history) for _ in range(self.num_joints)]

        # Thread-safe queue for state data (smaller size to prevent backlog)
        self.state_queue = queue.Queue(maxsize=20)

        # Control flags
        self.running = False
        self.teleop_hand = None

        # Performance optimization: only auto-scale every N frames
        self.scale_counter = 0
        self.scale_interval = 10  # Auto-scale every 10 frames (~0.5s at 20Hz)

        # Setup matplotlib
        self.setup_plots()

    def setup_plots(self):
        """Setup matplotlib figure and subplots."""
        num_cols = self.num_plot_types
        fig_width = 5 * num_cols
        self.fig, axes = plt.subplots(
            self.num_joints, num_cols, figsize=(fig_width, 4 * self.num_joints), squeeze=False
        )
        # axes is always 2D thanks to squeeze=False
        # Build a mapping from plot type -> column index
        self.col_map = {ptype: col for col, ptype in enumerate(self.plot_types)}
        self.axes = axes

        # Set titles and labels
        for joint_idx in range(self.num_joints):
            leader_id = self.leader_chain[joint_idx] if joint_idx < len(self.leader_chain) else None
            follower_id = self.follower_chain[joint_idx]

            if "current" in self.col_map:
                col = self.col_map["current"]
                ax = self.axes[joint_idx, col]
                ax.set_title(f"Joint {joint_idx} - Current (mA)\nLeader: {leader_id}, Follower: {follower_id}")
                ax.set_ylabel("Current (mA)")
                ax.grid(True, alpha=0.3)
                threshold_val = self.thresholds[joint_idx]
                ax.axhline(y=threshold_val, color="r", linestyle="--",
                           label=f"Threshold ({threshold_val:.1f} mA)", alpha=0.7)
                ax.axhline(y=-threshold_val, color="r", linestyle="--", alpha=0.7)
                y_max = 300
                ax.set_ylim(-y_max, y_max)

            if "velocity" in self.col_map:
                col = self.col_map["velocity"]
                ax = self.axes[joint_idx, col]
                ax.set_title(f"Joint {joint_idx} - Velocity (rad/s)")
                ax.set_ylabel("Velocity (rad/s)")
                ax.grid(True, alpha=0.3)
                ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
                ax.set_ylim(-2.0, 2.0)

            if "position" in self.col_map:
                col = self.col_map["position"]
                ax = self.axes[joint_idx, col]
                ax.set_title(f"Joint {joint_idx} - Position (rad)")
                if joint_idx == self.num_joints - 1:
                    ax.set_xlabel("Time (s)")
                ax.set_ylabel("Position (rad)")
                ax.grid(True, alpha=0.3)
                ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
                ax.set_ylim(-1.5, 1.5)

        plt.tight_layout()

        # Initialize line objects for plotting
        self.leader_current_lines = []
        self.follower_current_lines = []
        self.leader_velocity_lines = []
        self.follower_velocity_lines = []
        self.leader_position_lines = []
        self.follower_position_lines = []

        for joint_idx in range(self.num_joints):
            if "current" in self.col_map:
                col = self.col_map["current"]
                line_l, = self.axes[joint_idx, col].plot([], [], "b-", label="Leader", linewidth=1.5, alpha=0.8)
                line_f, = self.axes[joint_idx, col].plot([], [], "g-", label="Follower", linewidth=1.5, alpha=0.8)
                self.leader_current_lines.append(line_l)
                self.follower_current_lines.append(line_f)

            if "velocity" in self.col_map:
                col = self.col_map["velocity"]
                line_l, = self.axes[joint_idx, col].plot([], [], "b-", label="Leader", linewidth=1.5, alpha=0.8)
                line_f, = self.axes[joint_idx, col].plot([], [], "g-", label="Follower", linewidth=1.5, alpha=0.8)
                self.leader_velocity_lines.append(line_l)
                self.follower_velocity_lines.append(line_f)

            if "position" in self.col_map:
                col = self.col_map["position"]
                line_l, = self.axes[joint_idx, col].plot([], [], "b-", label="Leader", linewidth=1.5, alpha=0.8)
                line_f, = self.axes[joint_idx, col].plot([], [], "g-", label="Follower", linewidth=1.5, alpha=0.8)
                self.leader_position_lines.append(line_l)
                self.follower_position_lines.append(line_f)

        # Add legend to first column of first row(s)
        self.axes[0, 0].legend(loc="upper right", fontsize=8)
        if self.num_joints > 1:
            self.axes[1, 0].legend(loc="upper right", fontsize=8)

    def data_collection_thread(self):
        """Thread function to collect data from state queue."""
        start_time = time.time()

        while self.running:
            try:
                # Get state data from queue (non-blocking with timeout)
                # Use most recent data if queue has backlog
                try:
                    states_data = None
                    # Get the most recent data (drain queue and keep last)
                    while True:
                        try:
                            states_data = self.state_queue.get_nowait()
                        except queue.Empty:
                            break
                    # If we got data, use it; otherwise wait briefly
                    if states_data is None:
                        states_data = self.state_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                # Calculate time since start
                current_time = time.time() - start_time
                self.time_data.append(current_time)

                # Extract data for each joint
                leader_states = states_data.get("leader", {})
                follower_states = states_data.get("follower", {})

                leader_currents = leader_states.get("currents", [])
                follower_currents = follower_states.get("currents", [])
                leader_velocities = leader_states.get("joint_velocities", [])
                follower_velocities = follower_states.get("joint_velocities", [])
                leader_positions = leader_states.get("joint_angles", [])
                follower_positions = follower_states.get("joint_angles", [])

                # Store data for each joint
                for joint_idx in range(self.num_joints):
                    if joint_idx < len(leader_currents):
                        self.leader_currents[joint_idx].append(leader_currents[joint_idx])
                        self.leader_velocities[joint_idx].append(leader_velocities[joint_idx])
                        self.leader_positions[joint_idx].append(leader_positions[joint_idx])
                    else:
                        self.leader_currents[joint_idx].append(0.0)
                        self.leader_velocities[joint_idx].append(0.0)
                        self.leader_positions[joint_idx].append(0.0)

                    if joint_idx < len(follower_currents):
                        self.follower_currents[joint_idx].append(follower_currents[joint_idx])
                        self.follower_velocities[joint_idx].append(follower_velocities[joint_idx])
                        self.follower_positions[joint_idx].append(follower_positions[joint_idx])
                    else:
                        self.follower_currents[joint_idx].append(0.0)
                        self.follower_velocities[joint_idx].append(0.0)
                        self.follower_positions[joint_idx].append(0.0)

            except Exception as e:
                print(f"Error in data collection thread: {e}")
                time.sleep(0.01)

    def update_plots(self, frame):
        """Animation update function - optimized for low resource usage."""
        if not self.running or len(self.time_data) == 0:
            return []

        # Get control frequency to calculate plot window size
        control_freq = self.config.hand_config.get("control_frequency", 200)
        plot_window_size = int(self.plot_window_seconds * control_freq)

        # Only use recent data for plotting (reduce processing)
        time_array = np.array(list(self.time_data))
        if len(time_array) > plot_window_size:
            time_array = time_array[-plot_window_size:]
            plot_length = plot_window_size
        else:
            plot_length = len(time_array)

        if plot_length == 0:
            return []

        all_lines = []
        should_scale = (self.scale_counter % self.scale_interval == 0)
        self.scale_counter += 1

        for joint_idx in range(self.num_joints):
            # Efficiently extract recent data (only convert what we need)
            leader_current_list = list(self.leader_currents[joint_idx])
            follower_current_list = list(self.follower_currents[joint_idx])
            leader_velocity_list = list(self.leader_velocities[joint_idx])
            follower_velocity_list = list(self.follower_velocities[joint_idx])
            leader_position_list = list(self.leader_positions[joint_idx])
            follower_position_list = list(self.follower_positions[joint_idx])

            # Trim to plot window (only use recent data)
            if len(leader_current_list) > plot_length:
                leader_current_array = np.array(leader_current_list[-plot_length:], dtype=np.float32)
                follower_current_array = np.array(follower_current_list[-plot_length:], dtype=np.float32)
                leader_velocity_array = np.array(leader_velocity_list[-plot_length:], dtype=np.float32)
                follower_velocity_array = np.array(follower_velocity_list[-plot_length:], dtype=np.float32)
                leader_position_array = np.array(leader_position_list[-plot_length:], dtype=np.float32)
                follower_position_array = np.array(follower_position_list[-plot_length:], dtype=np.float32)
                time_array_plot = time_array[-plot_length:]
            else:
                # Use all available data (no padding needed)
                leader_current_array = np.array(leader_current_list, dtype=np.float32)
                follower_current_array = np.array(follower_current_list, dtype=np.float32)
                leader_velocity_array = np.array(leader_velocity_list, dtype=np.float32)
                follower_velocity_array = np.array(follower_velocity_list, dtype=np.float32)
                leader_position_array = np.array(leader_position_list, dtype=np.float32)
                follower_position_array = np.array(follower_position_list, dtype=np.float32)
                time_array_plot = time_array

            # Ensure arrays are same length
            min_len = min(len(time_array_plot), len(leader_current_array))
            time_array_plot = time_array_plot[-min_len:]
            leader_current_array = leader_current_array[-min_len:]
            follower_current_array = follower_current_array[-min_len:]
            leader_velocity_array = leader_velocity_array[-min_len:]
            follower_velocity_array = follower_velocity_array[-min_len:]
            leader_position_array = leader_position_array[-min_len:]
            follower_position_array = follower_position_array[-min_len:]

            # Update lines and collect them for blitting
            if "current" in self.col_map:
                self.leader_current_lines[joint_idx].set_data(time_array_plot, leader_current_array)
                self.follower_current_lines[joint_idx].set_data(time_array_plot, follower_current_array)
                all_lines.extend([self.leader_current_lines[joint_idx], self.follower_current_lines[joint_idx]])

            if "velocity" in self.col_map:
                self.leader_velocity_lines[joint_idx].set_data(time_array_plot, leader_velocity_array)
                self.follower_velocity_lines[joint_idx].set_data(time_array_plot, follower_velocity_array)
                all_lines.extend([self.leader_velocity_lines[joint_idx], self.follower_velocity_lines[joint_idx]])

            if "position" in self.col_map:
                self.leader_position_lines[joint_idx].set_data(time_array_plot, leader_position_array)
                self.follower_position_lines[joint_idx].set_data(time_array_plot, follower_position_array)
                all_lines.extend([self.leader_position_lines[joint_idx], self.follower_position_lines[joint_idx]])

            # Auto-scale x-axis (only every N frames to reduce CPU usage)
            if should_scale and len(time_array_plot) > 0:
                xlim = (max(0, time_array_plot[-1] - 10), time_array_plot[-1] + 1)
                for ptype in self.plot_types:
                    self.axes[joint_idx, self.col_map[ptype]].set_xlim(*xlim)

        return all_lines

    def run(self):
        """Run the live plot tool."""
        print("🔍 Starting Live Plot Tool")
        print(f"   Monitoring {self.num_joints} joint(s)")
        print(f"   Plotting: {', '.join(self.plot_types)}")
        print(f"   Leader motors: {self.leader_chain}")
        print(f"   Follower motors: {self.follower_chain}")
        print(f"   Force rendering thresholds: {self.thresholds}")
        print("\n   Close the plot window to stop the live plot tool.\n")

        # Create TeleopHand instance
        self.teleop_hand = TeleopHand(self.config, state_queue=self.state_queue)

        # Connect and setup
        self.teleop_hand.connect()
        self.teleop_hand.setup_motors()

        # Start control loop in separate thread
        self.running = True
        control_thread = threading.Thread(target=self.teleop_hand.start_control_loop, daemon=True)
        control_thread.start()

        # Start data collection thread
        data_thread = threading.Thread(target=self.data_collection_thread, daemon=True)
        data_thread.start()

        # Wait a bit for data to start flowing
        time.sleep(0.5)

        # Start animation (reduced frequency for better performance)
        ani = animation.FuncAnimation(
            self.fig,
            self.update_plots,
            interval=100,  # Update every 100ms (10 Hz refresh rate - reduced for performance)
            blit=True,
            cache_frame_data=False,
        )

        # Show plot (blocking)
        plt.show()

        # Cleanup
        print("\n🛑 Stopping live plot tool...")
        self.running = False
        self.teleop_hand.running = False

        # Wait for threads to finish
        control_thread.join(timeout=2.0)
        data_thread.join(timeout=1.0)

        # Disconnect
        self.teleop_hand.disconnect()
        print("✅ Live plot tool stopped.")


def _parse_plot_flags():
    """Parse --position, --velocity, --current flags from sys.argv.

    These flags are stripped from sys.argv so Hydra doesn't see them.
    If none are specified, all plot types are returned (default behavior).
    """
    plot_flags = {"--current": "current", "--velocity": "velocity", "--position": "position"}
    selected = []
    remaining_argv = []
    for arg in sys.argv:
        if arg in plot_flags:
            selected.append(plot_flags[arg])
        else:
            remaining_argv.append(arg)
    sys.argv = remaining_argv
    return selected if selected else None  # None means "show all"


def main():
    """Main function to run live plot tool."""
    # Parse plot type flags before Hydra processes argv
    plot_types = _parse_plot_flags()

    # Get the config directory
    config_path = Path(__file__).parent.parent / "hand_interfaces" / "conf"

    @hydra.main(version_base=None, config_path=str(config_path), config_name="config")
    def run_live_plot(cfg: DictConfig) -> None:
        """Run live plot tool with the specified configuration."""
        if TeleopHand.is_idle_hand_config(cfg):
            print(
                "Live plot needs a teleop hand configuration. "
                "Default Hydra stack uses hand_config=none.\n"
                "Example: python controllers/live_plot.py hand_config=7dof/7dof",
                file=sys.stderr,
            )
            sys.exit(2)
        print(f"🔍 Live Plot Tool - {cfg.config_name}")
        print(f"Description: {cfg.description}\n")

        # Create and run live plot tool
        live_plot = LivePlotter(cfg, plot_types=plot_types)
        live_plot.run()

    run_live_plot()


if __name__ == "__main__":
    main()

