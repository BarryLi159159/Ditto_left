#!/usr/bin/env python3
"""
GUI Hand Control Script

A real-time GUI for controlling hand joints with sliders.
Demonstrates the flexibility of the new HandInterface architecture.
"""

import sys
import time
import threading
from pathlib import Path
import math

import tkinter as tk
from tkinter import ttk, messagebox
import hydra
from omegaconf import DictConfig, OmegaConf

sys.path.append(str(Path(__file__).parent.parent))

from controllers.teleop_controller import TeleopHand


class GUIHandController:
    """GUI-based hand controller using the new HandInterface architecture."""
    
    def __init__(self, config: DictConfig):
        """Initialize the GUI hand controller."""
        # Create a mutable copy of the config to allow modifications
        self.config = OmegaConf.create(OmegaConf.to_container(config, resolve=True))
        self.verbose = self.config.hand_config.get('verbose', self.config.get('verbose', True))
        
        # Override leader mode to 'agent' for GUI control (follower mode stays the same)
        # This allows reusing existing configs by just changing leader mode
        if self.config.hand_config.leader.mode != 'agent':
            # Set leader mode to 'agent'
            self.config.hand_config.leader.mode = 'agent'
        
        # Create TeleopHand with agent mode (leader='agent', follower keeps its original mode)
        self.teleop_hand = None
        
        # GUI variables
        self.root = None
        self.joint_vars = {}  # Tkinter variables for joint angles
        self.joint_sliders = {}  # Slider widgets
        self.joint_limits = {}  # Joint limits for each motor
        self.running = False
        self.control_thread = None
        self.control_loop_thread = None  # Thread for TeleopHand control loop
        
        # Control settings
        self.gui_control_frequency = 20
        self.command_duration = 1.0 / self.gui_control_frequency
        
    def create_hand_interfaces(self):
        """Create and connect hand interface using TeleopHand."""
        try:
            # Create TeleopHand with agent mode
            self.teleop_hand = TeleopHand(self.config)
            
            # Get joint limits from the hand interface
            self._get_joint_limits()
            
            return True
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect to hand interface: {e}")
            return False
    
    def _get_joint_limits(self):
        """Get joint limits from the hand interface."""
        # Get follower joint limits
        follower_hand = self.teleop_hand.follower_hand
        for i, motor_id in enumerate(follower_hand.motor_ids):
            converter = follower_hand.joint_converters[i]
            min_angle, max_angle = converter.get_joint_limits()
            self.joint_limits[motor_id] = (min_angle, max_angle)
    
    def create_gui(self):
        """Create the GUI interface."""
        self.root = tk.Tk()
        self.root.title("GUI Motor Control")
        self.root.geometry("600x400")
        
        # Create main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        # Title
        title_label = ttk.Label(main_frame, text="🤖 GUI Motor Control", font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # Control buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=1, column=0, columnspan=3, pady=(0, 20))
        
        self.start_button = ttk.Button(button_frame, text="▶️ Start", command=self.start_control)
        self.start_button.pack(side=tk.LEFT, padx=(0, 10))
        
        self.stop_button = ttk.Button(button_frame, text="⏹️ Stop", command=self.stop_control, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=(0, 10))
        
        # Status label
        self.status_label = ttk.Label(button_frame, text="Status: Ready", foreground="orange")
        self.status_label.pack(side=tk.LEFT, padx=(20, 0))
        
        # Joint control frame
        self.joint_frame = ttk.LabelFrame(main_frame, text="Joint Control", padding="10")
        self.joint_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        self.joint_frame.columnconfigure(1, weight=1)
        
        # Create joint sliders (will be empty until connected)
        self._create_joint_sliders(self.joint_frame)
        
        # Note: Position display removed to avoid port conflicts
        # The slider values show the target positions, which is sufficient
        
        # Configure main frame grid weights
        main_frame.rowconfigure(2, weight=1)
        
        # Handle window closing
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def _create_joint_sliders(self, parent):
        """Create joint control sliders."""
        if not self.teleop_hand or not self.teleop_hand.follower_hand:
            # Show placeholder message
            placeholder_label = ttk.Label(parent, text="Connect to motor to see controls", font=("Arial", 12, "italic"))
            placeholder_label.grid(row=0, column=0, columnspan=3, pady=20)
            return
        
        follower_hand = self.teleop_hand.follower_hand
        for i, motor_id in enumerate(follower_hand.motor_ids):
            # Motor label
            motor_label = ttk.Label(parent, text=f"Motor {motor_id}:")
            motor_label.grid(row=i, column=0, sticky=tk.W, padx=(0, 10), pady=5)
            
            # Joint angle variable
            joint_var = tk.DoubleVar(value=0.0)
            self.joint_vars[motor_id] = joint_var
            
            # Get joint limits
            min_angle, max_angle = self.joint_limits.get(motor_id, (-math.pi/2, math.pi/2))
            min_deg = math.degrees(min_angle)
            max_deg = math.degrees(max_angle)
            
            # Slider
            slider = ttk.Scale(
                parent, 
                from_=min_deg, 
                to=max_deg, 
                variable=joint_var,
                orient=tk.HORIZONTAL,
                length=400,
                command=lambda val, mid=motor_id: self._on_slider_change(mid, val)
            )
            slider.grid(row=i, column=1, sticky=(tk.W, tk.E), padx=(0, 10), pady=5)
            self.joint_sliders[motor_id] = slider
            
            # Value label
            value_label = ttk.Label(parent, text="0.0°")
            value_label.grid(row=i, column=2, sticky=tk.W, pady=5)
            
            # Store reference to value label for updates
            slider.value_label = value_label
    
    def _recreate_sliders(self):
        """Recreate sliders after connection."""
        # Clear existing widgets
        for widget in self.joint_frame.winfo_children():
            widget.destroy()
        
        # Recreate sliders
        self._create_joint_sliders(self.joint_frame)
    
    def _on_slider_change(self, motor_id, value):
        """Handle slider value changes."""
        if motor_id in self.joint_sliders:
            # Update the value label
            self.joint_sliders[motor_id].value_label.config(text=f"{float(value):.1f}°")
    
    def _update_sliders_from_current_positions(self):
        """Update sliders with current joint positions (read once before control loop starts)."""
        if not self.teleop_hand or not self.teleop_hand.follower_hand:
            return
        
        try:
            # Read states directly once during initialization (before control loop starts)
            # This is safe because the control loop hasn't started yet
            follower_hand = self.teleop_hand.follower_hand
            follower_states = follower_hand.read_states()
            for i, state in enumerate(follower_states):
                motor_id = state.motor_id
                joint_angle_deg = math.degrees(state.joint_angle)
                
                if motor_id in self.joint_vars:
                    self.joint_vars[motor_id].set(joint_angle_deg)
                    self.joint_sliders[motor_id].value_label.config(text=f"{joint_angle_deg:.1f}°")
        except Exception as e:
            print(f"Error reading current positions: {e}")
    
    def start_control(self):
        """Connect to hand interface and start the control loop."""
        # Disable start button immediately to prevent double-clicking
        self.start_button.config(state=tk.DISABLED)
        self.status_label.config(text="Status: Connecting...", foreground="orange")
        self.root.update()  # Update GUI to show status change
        
        # Step 1: Connect and setup
        if not self.teleop_hand:
            if not self.create_hand_interfaces():
                self.status_label.config(text="Status: Connection Failed", foreground="red")
                self.start_button.config(state=tk.NORMAL)
                messagebox.showerror("Connection Error", "Failed to connect to hand interface")
                return
            
            # Recreate sliders now that we have motor info
            self._recreate_sliders()
            
            # Update joint sliders with current positions
            self._update_sliders_from_current_positions()
        
        # Step 2: Initialize external command with current positions before starting control loop
        self.status_label.config(text="Status: Initializing...", foreground="orange")
        self.root.update()
        
        try:
            # Read states directly once during initialization (before control loop starts)
            # This is safe because the control loop hasn't started yet
            follower_hand = self.teleop_hand.follower_hand
            follower_states = follower_hand.read_states()
            current_joint_angles = [state.joint_angle for state in follower_states]
            self.teleop_hand.schedule_command(current_joint_angles, self.command_duration)
        except Exception as e:
            print(f"Warning: Could not initialize commands from current positions: {e}")
            # Initialize with zeros as fallback
            follower_hand = self.teleop_hand.follower_hand
            self.teleop_hand.schedule_command([0.0] * len(follower_hand.motor_ids), self.command_duration)
        
        # Step 3: Start control loop
        self.status_label.config(text="Status: Starting...", foreground="orange")
        self.root.update()
        
        self.running = True
        self.stop_button.config(state=tk.NORMAL)
        self.status_label.config(text="Status: Controlling", foreground="blue")
        
        # Start TeleopHand control loop in background thread
        self.teleop_hand.running = True
        self.control_loop_thread = threading.Thread(target=self.teleop_hand.start_control_loop, daemon=True)
        self.control_loop_thread.start()
        
        # Small delay to ensure control loop thread starts and initializes before GUI thread
        time.sleep(0.2)
        
        # Start GUI control thread (updates commands and displays)
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
    
    def stop_control(self):
        """Stop the control loop and disconnect."""
        self.running = False
        if self.teleop_hand:
            self.teleop_hand.running = False
            # Wait a bit for threads to stop
            time.sleep(0.1)
            # Disconnect
            self.teleop_hand.disconnect()
            self.teleop_hand = None
        
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.status_label.config(text="Status: Stopped", foreground="orange")
        
        # Clear sliders
        for widget in self.joint_frame.winfo_children():
            widget.destroy()
        self.joint_vars = {}
        self.joint_sliders = {}
    
    def _control_loop(self):
        """Main control loop running in separate thread."""
        target_period = 1.0 / self.gui_control_frequency
        
        while self.running:
            loop_start = time.time()
            
            try:
                # Check if teleop_hand exists and is still valid
                if not self.teleop_hand:
                    continue
                
                if not self.teleop_hand.follower_hand:
                    continue
                
                # Get current slider values
                follower_hand = self.teleop_hand.follower_hand
                joint_angles = []
                for motor_id in follower_hand.motor_ids:
                    if motor_id in self.joint_vars:
                        angle_deg = self.joint_vars[motor_id].get()
                        angle_rad = math.radians(angle_deg)
                        joint_angles.append(angle_rad)
                    else:
                        joint_angles.append(0.0)
                
                # Update commands via TeleopHand API
                # This is the only operation - no state reading to avoid port conflicts
                # Double-check teleop_hand is still valid before calling (race condition protection)
                if self.teleop_hand:
                    self.teleop_hand.schedule_command(joint_angles, self.command_duration)
                
                # Note: Position display updates removed to avoid port conflicts
                # The TeleopHand control loop handles all state reading
                
            except Exception as e:
                # Only print error if we're still running (avoid spam during shutdown)
                if self.running:
                    print(f"Control loop error: {e}")
            
            # Maintain control frequency
            elapsed = time.time() - loop_start
            sleep_time = target_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def on_closing(self):
        """Handle window closing."""
        self.running = False
        if self.teleop_hand:
            self.teleop_hand.running = False
            self.teleop_hand.disconnect()
        self.root.destroy()
    
    def run(self):
        """Run the GUI application."""
        self.create_gui()
        self.root.mainloop()


def main():
    """Main function to run the GUI hand control."""
    # Load configuration
    config_path = Path(__file__).parent.parent / "hand_interfaces" / "conf"
    
    @hydra.main(version_base=None, config_path=str(config_path), config_name="config")
    def run_gui(cfg: DictConfig) -> None:
        """Run the GUI with the specified configuration."""
        if TeleopHand.is_idle_hand_config(cfg):
            print(
                "GUI controller needs a hand with follower motors. "
                "Default Hydra stack uses hand_config=none.\n"
                "Example: python controllers/gui_controller.py hand_config=7dof/7dof",
                file=sys.stderr,
            )
            sys.exit(2)
        print(f"🤖 GUI Hand Control - {cfg.config_name}")
        print(f"Description: {cfg.description}")
        print("=" * 50)
        
        # Create and run GUI controller
        gui_controller = GUIHandController(cfg)
        gui_controller.run()
    
    run_gui()


if __name__ == "__main__":
    main()
