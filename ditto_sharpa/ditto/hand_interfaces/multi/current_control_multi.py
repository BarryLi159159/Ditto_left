#!/usr/bin/env python3
"""
Multi-Motor Current Control Laws for Teleop Hand

This module provides current control laws that naturally handle multiple motors
with vectorized operations, eliminating the need for for loops.
"""

import numpy as np
import math
from typing import List, Optional, Dict
from dataclasses import dataclass
from .hand_data import HandData


@dataclass
class ControlParams:
    """Control parameters for multiple motors."""
    motor_ids: List[int]
    
    # Friction compensation parameters (numpy arrays)
    mu_coulomb_cw: np.ndarray = None      # Clockwise coulomb friction coefficients
    mu_coulomb_ccw: np.ndarray = None     # Counterclockwise coulomb friction coefficients
    mu_damping: np.ndarray = None         # Damping coefficients
    mu_static_amplitude: np.ndarray = None  # Static friction amplitude coefficients
    static_frequency: np.ndarray = None   # Static friction dithering frequencies (Hz)
    static_threshold_cw: np.ndarray = None   # Clockwise velocity thresholds (rad/s)
    static_threshold_ccw: np.ndarray = None  # Counterclockwise velocity thresholds (rad/s)
    
    # Force rendering parameters (numpy arrays)
    force_rendering_gain: np.ndarray = None     # Gains for force rendering
    force_rendering_alpha: np.ndarray = None    # Low-pass filter alphas
    force_rendering_threshold: np.ndarray = None  # Minimum follower currents (mA)
    force_rendering_max_current: np.ndarray = None  # Maximum force rendering currents (mA)
    
    # Control law selection (numpy arrays of booleans)
    enable_feedback: np.ndarray = None
    enable_coulomb_comp: np.ndarray = None
    enable_damping_comp: np.ndarray = None
    enable_static_comp: np.ndarray = None
    enable_force_rendering: np.ndarray = None
    
    def __post_init__(self):
        """Initialize default values after dataclass creation."""
        num_motors = len(self.motor_ids)
        
        # Initialize arrays with default values
        if self.mu_coulomb_cw is None:
            self.mu_coulomb_cw = np.zeros(num_motors)
        if self.mu_coulomb_ccw is None:
            self.mu_coulomb_ccw = np.zeros(num_motors)
        if self.mu_damping is None:
            self.mu_damping = np.zeros(num_motors)
        if self.mu_static_amplitude is None:
            self.mu_static_amplitude = np.zeros(num_motors)
        if self.static_frequency is None:
            self.static_frequency = np.full(num_motors, 10.0)
        if self.static_threshold_cw is None:
            self.static_threshold_cw = np.ones(num_motors)
        if self.static_threshold_ccw is None:
            self.static_threshold_ccw = np.ones(num_motors)
        
        if self.force_rendering_gain is None:
            self.force_rendering_gain = np.zeros(num_motors)
        if self.force_rendering_alpha is None:
            self.force_rendering_alpha = np.full(num_motors, 0.95)
        if self.force_rendering_threshold is None:
            self.force_rendering_threshold = np.full(num_motors, 50.0)
        if self.force_rendering_max_current is None:
            self.force_rendering_max_current = np.full(num_motors, 2300.0)
        
        if self.enable_feedback is None:
            self.enable_feedback = np.zeros(num_motors, dtype=bool)
        if self.enable_coulomb_comp is None:
            self.enable_coulomb_comp = np.zeros(num_motors, dtype=bool)
        if self.enable_damping_comp is None:
            self.enable_damping_comp = np.zeros(num_motors, dtype=bool)
        if self.enable_static_comp is None:
            self.enable_static_comp = np.zeros(num_motors, dtype=bool)
        if self.enable_force_rendering is None:
            self.enable_force_rendering = np.zeros(num_motors, dtype=bool)


class CurrentControlLaws:
    """Current control laws that naturally handle multiple motors."""
    
    def __init__(self, control_frequency: float = 50.0):
        """
        Initialize current control laws.
        
        Args:
            control_frequency: Control frequency in Hz
        """
        self.control_frequency = control_frequency
        self.dt = 1.0 / control_frequency
        
        # History for force rendering
        self._prev_force_cmds_array = None
    
    def zero_current(self, hand_data: HandData, params: ControlParams, 
                    time: float = 0.0) -> np.ndarray:
        """
        Zero current control law - returns zeros.
        
        Args:
            hand_data: HandData object with current states
            params: ControlParams object
            time: Current time
            
        Returns:
            Array of zero current commands
        """
        return np.zeros(len(hand_data.motor_ids))
    
    def coulomb_friction(self, hand_data: HandData, params: ControlParams, 
                        time: float = 0.0) -> np.ndarray:
        """
        Coulomb friction compensation.
        
        Args:
            hand_data: HandData object with current states
            params: ControlParams object
            time: Current time
            
        Returns:
            Array of coulomb friction compensation commands
        """
        # Get joint velocities
        joint_velocities = hand_data.joint_velocities
        
        # Vectorized coulomb friction calculation
        # Positive velocities: use cw coefficient with positive sign
        # Negative velocities: use ccw coefficient with negative sign
        coulomb_cmds = np.where(
            joint_velocities > 0,
            params.mu_coulomb_cw * np.sign(joint_velocities),  # +mu_cw
            np.where(
                joint_velocities < 0,
                params.mu_coulomb_ccw * np.sign(joint_velocities),  # -mu_ccw
                0  # zero velocity
            )
        )
        
        # Apply only where enabled
        coulomb_cmds = np.where(params.enable_coulomb_comp, coulomb_cmds, 0)
        
        return coulomb_cmds
    
    def damping_compensation(self, hand_data: HandData, params: ControlParams, 
                            time: float = 0.0) -> np.ndarray:
        """
        Damping compensation.
        
        Args:
            hand_data: HandData object with current states
            params: ControlParams object
            time: Current time
            
        Returns:
            Array of damping compensation commands
        """
        # Get joint velocities
        joint_velocities = hand_data.joint_velocities
        
        # Vectorized damping calculation
        damping_cmds = -params.mu_damping * joint_velocities
        
        # Apply only where enabled
        damping_cmds = np.where(params.enable_damping_comp, damping_cmds, 0)
        
        return damping_cmds
    
    def static_friction(self, hand_data: HandData, params: ControlParams, 
                       time: float = 0.0) -> np.ndarray:
        """
        Static friction dithering.
        
        Args:
            hand_data: HandData object with current states
            params: ControlParams object
            time: Current time
            
        Returns:
            Array of static friction commands
        """
        # Get joint velocities
        joint_velocities = hand_data.joint_velocities
        
        # For positive velocities: check if 0 <= velocity < threshold_cw
        # For negative velocities: check if -threshold_ccw < velocity <= 0
        below_threshold_cw = (joint_velocities >= 0) & (joint_velocities < params.static_threshold_cw)
        below_threshold_ccw = (joint_velocities <= 0) & (joint_velocities > -params.static_threshold_ccw)
        
        # Generate dithering signal (using cosine like original)
        dither_signal = np.cos(np.pi * params.static_frequency * time)
        
        # Apply static friction only where enabled and below threshold
        static_cmds = np.where(
            params.enable_static_comp & (below_threshold_cw | below_threshold_ccw),
            params.mu_static_amplitude * dither_signal,
            0
        )
        
        return static_cmds
    
    def force_rendering(self, hand_data: HandData, params: ControlParams, 
                        time: float = 0.0, follower_currents: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Force rendering based on follower current feedback.
        
        Args:
            hand_data: HandData object with current states
            params: ControlParams object
            time: Current time
            follower_currents: Array of follower currents (optional)
            
        Returns:
            Array of force rendering commands
        """
        if follower_currents is None:
            return np.zeros(len(hand_data.motor_ids))
        
        # Get previous commands as array (initialize if needed)
        if (self._prev_force_cmds_array is None or len(self._prev_force_cmds_array) != len(hand_data.motor_ids)):
            self._prev_force_cmds_array = np.zeros(len(hand_data.motor_ids))
        
        # Step 1: Apply gain
        k = params.force_rendering_gain
        raw_cmds = -k * follower_currents
        
        # Step 2: Low-pass filter
        alpha = params.force_rendering_alpha
        force_cmds = alpha * raw_cmds + (1 - alpha) * self._prev_force_cmds_array
        
        # Step 3: Clamp to max
        force_cmds = np.clip(force_cmds, -params.force_rendering_max_current, 
                                params.force_rendering_max_current)
        
        # Step 4: Apply masking based on enable flags and thresholds
        # Only apply force rendering where:
        #   - Force rendering is enabled
        #   - Follower current exceeds threshold (absolute value)
        above_threshold = np.abs(follower_currents) >= params.force_rendering_threshold
        should_apply = params.enable_force_rendering & above_threshold
        
        # Zero out commands where conditions not met
        force_cmds = np.where(should_apply, force_cmds, 0.0)
        
        # Store for next iteration
        self._prev_force_cmds_array = force_cmds
        
        return force_cmds
    
    def compute_current_command(self, hand_data: HandData, params: ControlParams, 
                              time: float = 0.0, follower_currents: Optional[np.ndarray] = None,
                              verbose: bool = False) -> np.ndarray:
        """
        Compute current commands for all motors.
        
        Args:
            hand_data: HandData object with current states
            params: ControlParams object
            time: Current time
            follower_currents: Array of follower currents (optional)
            verbose: Whether to print debug information
            
        Returns:
            Array of current commands
        """
        # Check if any control laws are enabled (early return optimization)
        # For small arrays, checking Python bools is faster than np.any()
        any_enabled = (params.enable_feedback.any() if params.enable_feedback is not None else False) or \
                     (params.enable_coulomb_comp.any() if params.enable_coulomb_comp is not None else False) or \
                     (params.enable_damping_comp.any() if params.enable_damping_comp is not None else False) or \
                     (params.enable_static_comp.any() if params.enable_static_comp is not None else False) or \
                     (params.enable_force_rendering.any() if params.enable_force_rendering is not None else False)
        
        if not any_enabled:
            return np.zeros(len(hand_data.motor_ids))
        
        # Initialize total command
        total_cmds = np.zeros(len(hand_data.motor_ids))
        
        # Initialize debug variables (only used if verbose)
        coulomb_cmds = np.zeros(len(hand_data.motor_ids))
        damping_cmds = np.zeros(len(hand_data.motor_ids))
        static_cmds = np.zeros(len(hand_data.motor_ids))
        force_cmds = np.zeros(len(hand_data.motor_ids))
        
        # Only compute enabled control laws (optimization - use .any() method for efficiency)
        if params.enable_coulomb_comp is not None and params.enable_coulomb_comp.any():
            coulomb_cmds = self.coulomb_friction(hand_data, params, time)
            total_cmds += coulomb_cmds
        
        if params.enable_damping_comp is not None and params.enable_damping_comp.any():
            damping_cmds = self.damping_compensation(hand_data, params, time)
            total_cmds += damping_cmds
        
        if params.enable_static_comp is not None and params.enable_static_comp.any():
            static_cmds = self.static_friction(hand_data, params, time)
            total_cmds += static_cmds
        
        if params.enable_force_rendering is not None and params.enable_force_rendering.any():
            force_cmds = self.force_rendering(hand_data, params, time, follower_currents)
            total_cmds += force_cmds
        
        # Clamp to motor limits (assuming XM430 with 2.3A max current = 2300 mA)
        max_current = 2300  # mA
        total_cmds = np.clip(total_cmds, -max_current, max_current)
        
        # Debug output
        if verbose:
            for i, motor_id in enumerate(hand_data.motor_ids):
                print(f"Motor {motor_id} computed torques: "
                      f"{0.0:6.3f} (cmd) + {coulomb_cmds[i]:6.3f} (coulomb) + "
                      f"{damping_cmds[i]:6.3f} (damping) + {static_cmds[i]:6.3f} (static) + "
                      f"{force_cmds[i]:6.3f} (force)")
                if follower_currents is not None and i < len(follower_currents):
                    print(f"Follower current: {follower_currents[i]:6.3f}")
                print(f"Commanded torque: {total_cmds[i]:6.3f}")
        
        return total_cmds
    
    def reset_force_rendering_history(self):
        """Reset the force rendering command history."""
        self._prev_force_cmds_array = None


def create_control_params_from_config(motor_ids: List[int], joint_settings: List[Dict]) -> ControlParams:
    """
    Create ControlParams from configuration.
    
    Args:
        motor_ids: List of motor IDs
        joint_settings: List of joint setting dictionaries
        
    Returns:
        ControlParams object
    """
    num_motors = len(motor_ids)
    
    # Initialize arrays
    mu_coulomb_cw = np.zeros(num_motors)
    mu_coulomb_ccw = np.zeros(num_motors)
    mu_damping = np.zeros(num_motors)
    mu_static_amplitude = np.zeros(num_motors)
    static_frequency = np.full(num_motors, 10.0)
    static_threshold_cw = np.ones(num_motors)
    static_threshold_ccw = np.ones(num_motors)
    
    force_rendering_gain = np.zeros(num_motors)
    force_rendering_alpha = np.full(num_motors, 0.95)
    force_rendering_threshold = np.full(num_motors, 50.0)
    force_rendering_max_current = np.full(num_motors, 2300.0)
    
    enable_feedback = np.zeros(num_motors, dtype=bool)
    enable_coulomb_comp = np.zeros(num_motors, dtype=bool)
    enable_damping_comp = np.zeros(num_motors, dtype=bool)
    enable_static_comp = np.zeros(num_motors, dtype=bool)
    enable_force_rendering = np.zeros(num_motors, dtype=bool)
    
    # Fill arrays from configuration
    for i, motor_id in enumerate(motor_ids):
        # Find settings for this motor
        motor_settings = None
        for settings in joint_settings:
            if settings.get('motor_id') == motor_id:
                motor_settings = settings
                break
        
        if motor_settings and 'current_control' in motor_settings:
            cc = motor_settings['current_control']
            
            # Friction compensation
            mu_coulomb_cw[i] = cc.get('mu_coulomb_cw', 0.0)
            mu_coulomb_ccw[i] = cc.get('mu_coulomb_ccw', 0.0)
            mu_damping[i] = cc.get('mu_damping', 0.0)
            mu_static_amplitude[i] = cc.get('mu_static_amplitude', 0.0)
            static_frequency[i] = cc.get('static_frequency', 10.0)
            static_threshold_cw[i] = cc.get('static_threshold_cw', 1.0)
            static_threshold_ccw[i] = cc.get('static_threshold_ccw', 1.0)
            
            # Force rendering
            force_rendering_gain[i] = cc.get('force_rendering_gain', 0.0)
            force_rendering_alpha[i] = cc.get('force_rendering_alpha', 0.95)
            force_rendering_threshold[i] = cc.get('force_rendering_threshold', 50.0)
            force_rendering_max_current[i] = cc.get('force_rendering_max_current', 2300.0)
            
            # Enable flags
            enable_feedback[i] = cc.get('enable_feedback', False)
            enable_coulomb_comp[i] = cc.get('enable_coulomb_comp', False)
            enable_damping_comp[i] = cc.get('enable_damping_comp', False)
            enable_static_comp[i] = cc.get('enable_static_comp', False)
            enable_force_rendering[i] = cc.get('enable_force_rendering', False)
    
    return ControlParams(
        motor_ids=motor_ids,
        mu_coulomb_cw=mu_coulomb_cw,
        mu_coulomb_ccw=mu_coulomb_ccw,
        mu_damping=mu_damping,
        mu_static_amplitude=mu_static_amplitude,
        static_frequency=static_frequency,
        static_threshold_cw=static_threshold_cw,
        static_threshold_ccw=static_threshold_ccw,
        force_rendering_gain=force_rendering_gain,
        force_rendering_alpha=force_rendering_alpha,
        force_rendering_threshold=force_rendering_threshold,
        force_rendering_max_current=force_rendering_max_current,
        enable_feedback=enable_feedback,
        enable_coulomb_comp=enable_coulomb_comp,
        enable_damping_comp=enable_damping_comp,
        enable_static_comp=enable_static_comp,
        enable_force_rendering=enable_force_rendering
    )

