#!/usr/bin/env python3
"""
GUI Hand Control Script (Vectorized Multi-Hand Interface)

A real-time GUI for controlling hand joints with sliders.
Demonstrates the flexibility of the new vectorized multi-hand interface.
"""

import sys
import time
import threading
import numpy as np
from pathlib import Path
import math

import tkinter as tk
from tkinter import ttk, messagebox
import hydra
from omegaconf import DictConfig

sys.path.append(str(Path(__file__).parent.parent))

from hand_interfaces.src.hand_interface_multi import create_hand_interface_multi
from hand_interfaces.src.hand_data import HandData


class GUIHandController:
    """GUI-based hand controller using the vectorized multi-hand interface."""
    
    def __init__(self, config: DictConfig):
        """Initialize the GUI hand controller."""
        self.config = config
        self.verbose = config.hand_config.get('verbose', config.get('verbose', True))
        
        # Create hand interface
        self.follower_hand = None
        
        # GUI variables
        self.root = None
        self.joint_vars = {}  # Tkinter variables for joint angles
        self.joint_sliders = {}  # Slider widgets
        self.joint_limits = {}  # Joint limits for each motor
        self.running = False
        self.control_thread = None
        
        # Control settings
        self.control_frequency = config.hand_config.get('control_frequency', 50)
        
    def create_hand_interfaces(self):
        """Create and connect hand interface."""
        try:
            # Get U2D2 configuration
            u2d2_config = self.config.u2d2
            use_fake_u2d2 = u2d2_config.get('fake_u2d2', False)
            use_dual_u2d2 = u2d2_config.get('use_dual_u2d2', False)
            
            if use_dual_u2d2:
                # Dual U2D2 configuration - follower always on USB0
                follower_config = u2d2_config.follower
                follower_port = follower_config.get('usb_port', '/dev/ttyUSB0')
                follower_baudrate = follower_config.get('baudrate', 4000000)
            else:
                # Single U2D2 configuration
                usb_port = u2d2_config.get('usb_port', '/dev/ttyUSB0')
                baudrate = u2d2_config.get('baudrate', 4000000)
                follower_port = usb_port
                follower_baudrate = baudrate
            
            # Create follower hand interface
            self.follower_hand = create_hand_interface_multi(
                self.config, "follower", "real",
                port=follower_port, baudrate=follower_baudrate, use_fake=use_fake_u2d2
            )
            
            # Connect and setup
            self.follower_hand.connect()
            self.follower_hand.setup_motors()
            
            # Get joint limits from the hand interface
            self._get_joint_limits()
            
            return True
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect to hand interface: {e}")
            return False
    
    def _get_joint_limits(self):
        """Get joint limits from the hand interface."""
        # Get follower joint limits from the single multi-joint converter
        min_angles, max_angles = self.follower_hand.joint_converter.get_joint_limits()
        
        for i, motor_id in enumerate(self.follower_hand.motor_ids):
            self.joint_limits[motor_id] = (min_angles[i], max_angles[i])
    
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
        
        self.connect_button = ttk.Button(button_frame, text="🔌 Connect", command=self.connect_hands)
        self.connect_button.pack(side=tk.LEFT, padx=(0, 10))
        
        self.start_button = ttk.Button(button_frame, text="▶️ Start Control", command=self.start_control, state=tk.DISABLED)
        self.start_button.pack(side=tk.LEFT, padx=(0, 10))
        
        self.stop_button = ttk.Button(button_frame, text="⏹️ Stop Control", command=self.stop_control, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=(0, 10))
        
        # Status label
        self.status_label = ttk.Label(button_frame, text="Status: Disconnected", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=(20, 0))
        
        # Joint control frame
        self.joint_frame = ttk.LabelFrame(main_frame, text="Joint Control", padding="10")
        self.joint_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        self.joint_frame.columnconfigure(1, weight=1)
        
        # Create joint sliders (will be empty until connected)
        self._create_joint_sliders(self.joint_frame)
        
        # Current position display
        self.position_frame = ttk.LabelFrame(main_frame, text="Current Positions", padding="10")
        self.position_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.position_labels = {}
        # Will be populated when connected
        
        # Configure main frame grid weights
        main_frame.rowconfigure(2, weight=1)
        
        # Handle window closing
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def _create_joint_sliders(self, parent):
        """Create joint control sliders."""
        if not self.follower_hand:
            # Show placeholder message
            placeholder_label = ttk.Label(parent, text="Connect to motor to see controls", font=("Arial", 12, "italic"))
            placeholder_label.grid(row=0, column=0, columnspan=3, pady=20)
            return
        
        for i, motor_id in enumerate(self.follower_hand.motor_ids):
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
        
        # Recreate position labels
        self._recreate_position_labels()
    
    def _recreate_position_labels(self):
        """Recreate position labels after connection."""
        # Clear existing widgets
        for widget in self.position_frame.winfo_children():
            widget.destroy()
        
        # Recreate position labels
        self.position_labels = {}
        for i, motor_id in enumerate(self.follower_hand.motor_ids):
            label = ttk.Label(self.position_frame, text=f"Motor {motor_id}: 0.0°")
            label.grid(row=i, column=0, sticky=tk.W, padx=(0, 20))
            self.position_labels[motor_id] = label
    
    def _on_slider_change(self, motor_id, value):
        """Handle slider value changes."""
        if motor_id in self.joint_sliders:
            # Update the value label
            self.joint_sliders[motor_id].value_label.config(text=f"{float(value):.1f}°")
    
    def connect_hands(self):
        """Connect to hand interface."""
        if self.create_hand_interfaces():
            self.status_label.config(text="Status: Connected", foreground="green")
            self.connect_button.config(state=tk.DISABLED)
            self.start_button.config(state=tk.NORMAL)
            
            # Recreate sliders now that we have motor info
            self._recreate_sliders()
            
            # Update joint sliders with current positions
            self._update_sliders_from_current_positions()
        else:
            self.status_label.config(text="Status: Connection Failed", foreground="red")
    
    def _update_sliders_from_current_positions(self):
        """Update sliders with current joint positions."""
        if not self.follower_hand:
            return
        
        try:
            hand_data = self.follower_hand.read_states()
            for i, motor_id in enumerate(self.follower_hand.motor_ids):
                joint_angle_deg = math.degrees(hand_data.joint_angles[i])
                
                if motor_id in self.joint_vars:
                    self.joint_vars[motor_id].set(joint_angle_deg)
                    self.joint_sliders[motor_id].value_label.config(text=f"{joint_angle_deg:.1f}°")
        except Exception as e:
            print(f"Error reading current positions: {e}")
    
    def start_control(self):
        """Start the control loop."""
        if not self.follower_hand:
            messagebox.showerror("Error", "Hand interface not connected")
            return
        
        self.running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.status_label.config(text="Status: Controlling", foreground="blue")
        
        # Start control thread
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
    
    def stop_control(self):
        """Stop the control loop."""
        self.running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.status_label.config(text="Status: Stopped", foreground="orange")
    
    def _control_loop(self):
        """Main control loop running in separate thread."""
        target_period = 1.0 / self.control_frequency
        
        while self.running:
            loop_start = time.time()
            
            try:
                # Get current slider values as numpy array
                joint_angles = np.array([
                    math.radians(self.joint_vars[motor_id].get())
                    if motor_id in self.joint_vars
                    else 0.0
                    for motor_id in self.follower_hand.motor_ids
                ])
                
                # Send commands to follower hand (expects numpy array)
                self.follower_hand.send_position_commands(joint_angles)
                
                # Update position display
                self._update_position_display()
                
            except Exception as e:
                print(f"Control loop error: {e}")
            
            # Maintain control frequency
            elapsed = time.time() - loop_start
            sleep_time = target_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def _update_position_display(self):
        """Update the current position display."""
        if not self.follower_hand or not hasattr(self, 'position_labels'):
            return
        
        try:
            hand_data = self.follower_hand.read_states()
            for i, motor_id in enumerate(self.follower_hand.motor_ids):
                joint_angle_deg = math.degrees(hand_data.joint_angles[i])
                
                if motor_id in self.position_labels:
                    self.position_labels[motor_id].config(text=f"Motor {motor_id}: {joint_angle_deg:.1f}°")
        except Exception as e:
            print(f"Error updating position display: {e}")
    
    def on_closing(self):
        """Handle window closing."""
        self.running = False
        if self.follower_hand:
            self.follower_hand.disconnect()
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
        print(f"🤖 GUI Hand Control - {cfg.config_name}")
        print(f"Description: {cfg.description}")
        print("=" * 50)
        
        # Create and run GUI controller
        gui_controller = GUIHandController(cfg)
        gui_controller.run()
    
    run_gui()


if __name__ == "__main__":
    main()

