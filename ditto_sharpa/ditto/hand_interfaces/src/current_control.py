#!/usr/bin/env python3
"""
Current Control Laws for Teleop Hand

This module provides various control laws for current control of Dynamixel motors,
including friction compensation, force rendering, and zero current control.
"""

import numpy as np
from typing import List, Optional
from dataclasses import dataclass
from .motor_data import MotorData

@dataclass
class ControlParams:
    """Control parameters for a motor."""
    motor_id: int

    # ++ Force rendering ++

    # Force rendering parameters
    enable_force_rendering: bool = False
    force_rendering_gain: float = 0.0  # Gain for force rendering
    force_rendering_alpha: float = 0.95  # Symmetric output LPF alpha (fallback)
    force_rendering_threshold: float = 0.0  # Fallback when pos/neg not set (mA)
    force_rendering_threshold_positive: float = 0.0
    force_rendering_threshold_negative: float = 0.0
    gradual_rendering: bool = False # Start rendering from threshold (0 force) instead of current (jump)
    force_rendering_max_current: float = 200.0  # Maximum force rendering current (mA)

    # Velocity-dependent gain: K_eff = K0 / (1 + c * |qdot|). Softens impacts; full
    # stiffness when stationary.
    enable_force_rendering_adaptive_gain: bool = False
    force_rendering_adaptive_velocity_coeff: float = 0.0

    # Cap rising |I| on net leader current (force + damping); mA/s (0 = off).
    command_rate_limit_mA_per_s: float = 0.0

    # Unidirectional gate: render force only when follower current has the
    # configured sign (force_rendering_flexion_direction_sign * I > 0).
    force_rendering_flexion_only: bool = False
    force_rendering_flexion_direction_sign: float = 1.0

    # Force rendering damping (virtual leader damping gated on follower force + velocity)
    enable_force_rendering_damping: bool = False
    force_rendering_damping_gain: float = 0.0
    force_rendering_damping_velocity_threshold: float = 0.0
    force_rendering_damping_max_current: float = 200.0

    # ++ Friction compensation ++
    
    # Coulomb friction compensation parameters
    enable_coulomb_comp: bool = False
    mu_coulomb_cw: float = 0.0   # Clockwise coulomb friction coefficient
    mu_coulomb_ccw: float = 0.0  # Counterclockwise coulomb friction coefficient
    
    # Damping compensation parameters
    enable_damping_comp: bool = False
    mu_damping: float = 0.0      # Damping coefficient
    
    # Static friction compensation parameters
    enable_static_comp: bool = False
    mu_static_amplitude: float = 0.0  # Static friction amplitude coefficient
    static_frequency: float = 10.0  # Static friction dithering frequency (Hz)
    static_threshold_cw: float = 1.0   # Clockwise velocity threshold for static friction (rad/s)
    static_threshold_ccw: float = 1.0  # Counterclockwise velocity threshold for static friction (rad/s)


class CurrentControlLaws:
    """Collection of current control laws for teleop control."""
    
    @staticmethod
    def zero_current(motor_data: MotorData, params: ControlParams, time: float = 0.0) -> float:
        """
        Zero current control law - returns 0 current command.
        
        Args:
            motor_data: Current motor state data
            params: Control parameters (unused for zero current)
            time: Current time (unused for zero current)
            
        Returns:
            Current command in mA
        """
        return 0.0
    
    @staticmethod
    def _force_rendering_threshold(params: ControlParams, follower_current: float) -> float:
        """Signed threshold value for force rendering."""
        if follower_current >= 0.0:
            return params.force_rendering_threshold_positive
        return params.force_rendering_threshold_negative

    @staticmethod
    def _follower_above_force_threshold(
        params: ControlParams, follower_current: float
    ) -> bool:
        if follower_current >= 0.0:
            return follower_current >= params.force_rendering_threshold_positive
        return follower_current <= -params.force_rendering_threshold_negative

    @staticmethod
    def _force_sign_allowed(
        params: ControlParams, follower_current: float
    ) -> bool:
        """True when follower signal has configured sign for unidirectional mode."""
        return (params.force_rendering_flexion_direction_sign * follower_current) > 0.0

    @staticmethod
    def force_rendering(motor_data: MotorData, params: ControlParams, time: float = 0.0,
                       follower_data: Optional[MotorData] = None,
                       prev_command: Optional[float] = None) -> float:
        """
        Force rendering control law based on follower current feedback.
        
        Args:
            motor_data: Current motor state data
            params: Control parameters containing force rendering parameters
            time: Current time (unused)
            follower_data: Follower motor state data
            prev_command: Previous force rendering command for output low-pass filtering
        Returns:
            Force rendering current in mA
        """
        if not params.enable_force_rendering or follower_data is None:
            return 0.0

        follower_current = follower_data.current
        if follower_current is None:
            return 0.0
        
        current_mag = abs(follower_current)
        threshold = CurrentControlLaws._force_rendering_threshold(
            params, follower_current
        )
        
        if current_mag < threshold:
            return 0.0
            
        k = params.force_rendering_gain
        if (
            params.enable_force_rendering_adaptive_gain
            and params.force_rendering_adaptive_velocity_coeff > 0.0
        ):
            k = k / (
                1.0
                + params.force_rendering_adaptive_velocity_coeff
                * abs(motor_data.joint_velocity)
            )
        
        # Calculate force command
        if params.gradual_rendering:
            # Gradual rendering: start from 0 at threshold
            # Force magnitude = max(0, |current| - threshold)
            force_magnitude = max(0.0, current_mag - threshold)
            
            # Direction opposes current (negative feedback)
            # If current > 0, feedback should be negative.
            # If current < 0, feedback should be positive.
            if follower_current >= 0:
                raw_feedback = -k * force_magnitude
            else:
                raw_feedback = k * force_magnitude
        else:
            # Standard rendering: proportional to total current
            raw_feedback = -k * follower_current
        
        # Unidirectional mode: zero the force term when follower sign is wrong.
        if params.force_rendering_flexion_only and not CurrentControlLaws._force_sign_allowed(
            params, follower_current
        ):
            raw_feedback = 0.0

        # Apply output low-pass filter
        alpha = params.force_rendering_alpha
        if prev_command is not None:
            filtered_command = alpha * raw_feedback + (1 - alpha) * prev_command
        else:
            filtered_command = raw_feedback
        
        # Clamp to max force rendering limit
        max_curr = params.force_rendering_max_current
        filtered_command = np.clip(filtered_command, -max_curr, max_curr)
            
        return filtered_command

    @staticmethod
    def force_rendering_damping(
        motor_data: MotorData,
        params: ControlParams,
        time: float = 0.0,
        follower_data: Optional[MotorData] = None,
    ) -> float:
        """Virtual leader damping when force feedback is active and |velocity| exceeds threshold."""
        del time
        if not params.enable_force_rendering_damping or follower_data is None:
            return 0.0

        follower_current = follower_data.current
        if follower_current is None:
            return 0.0
        if not CurrentControlLaws._follower_above_force_threshold(
            params, follower_current
        ):
            return 0.0

        velocity = motor_data.joint_velocity
        excess_speed = max(
            0.0, abs(velocity) - params.force_rendering_damping_velocity_threshold
        )
        if excess_speed <= 0.0:
            return 0.0

        sign = 1.0 if velocity >= 0.0 else -1.0
        cmd = -sign * params.force_rendering_damping_gain * excess_speed
        max_mA = params.force_rendering_damping_max_current
        return max(-max_mA, min(max_mA, cmd))
    

    @staticmethod
    def coulomb_friction_compensation(motor_data: MotorData, params: ControlParams, time: float = 0.0) -> float:
        """
        Coulomb friction compensation control law.
        
        Args:
            motor_data: Current motor state data
            params: Control parameters containing coulomb friction coefficients
            time: Current time (unused)
            
        Returns:
            Coulomb friction compensation current in mA
        """
        # TODO Make coulomb friction compensation based on motor velocity
        
        if not params.enable_coulomb_comp:
            return 0.0
            
        velocity = motor_data.joint_velocity
        
        if velocity > 0:
            return params.mu_coulomb_cw * np.sign(velocity)
        else:
            return params.mu_coulomb_ccw * np.sign(velocity)
    
    @staticmethod
    def damping_compensation(motor_data: MotorData, params: ControlParams, time: float = 0.0) -> float:
        """
        Damping compensation control law.
        
        Args:
            motor_data: Current motor state data
            params: Control parameters containing damping coefficient
            time: Current time (unused)
            
        Returns:
            Damping compensation current in mA
        """
        if not params.enable_damping_comp:
            return 0.0
            
        velocity = motor_data.joint_velocity
        return -params.mu_damping * velocity
    
    @staticmethod
    def static_friction_compensation(motor_data: MotorData, params: ControlParams, time: float = 0.0) -> float:
        """
        Static friction compensation control law.
        
        Args:
            motor_data: Current motor state data
            params: Control parameters containing static friction parameters
            time: Current time for cosine wave generation
            
        Returns:
            Static friction compensation current in mA
        """
        if not params.enable_static_comp:
            return 0.0
            
        velocity = motor_data.joint_velocity
        
        # Apply static friction compensation when velocity is below thresholds (nearly static)
        # Use different thresholds for clockwise and counterclockwise directions
        if velocity >= 0 and velocity < params.static_threshold_cw:
            # Clockwise direction - use cw threshold (including zero velocity)
            return params.mu_static_amplitude * np.cos(np.pi * params.static_frequency * time)
        elif velocity <= 0 and velocity > -params.static_threshold_ccw:
            # Counterclockwise direction - use ccw threshold (including zero velocity)
            return params.mu_static_amplitude * np.cos(np.pi * params.static_frequency * time)
        else:
            return 0.0


class CurrentController:
    """
    Main current controller that combines multiple control laws.
    """
    
    def __init__(self, control_frequency: float = 50.0):
        """
        Initialize the current controller.
        
        Args:
            control_frequency: Control loop frequency in Hz
        """
        self.control_frequency = control_frequency
        self.prev_force_commands = {}  # Store previous force rendering commands for filtering
        
        # Used for plotting only: 
        self._last_force_rendering_damping_cmds: list[float] = []
        self._current_force_rendering_damping_cmd = 0.0
        self._last_force_rendering_cmds: list[float] = []
        self._current_force_rendering_cmd = 0.0

    @property
    def last_force_rendering_damping_cmds(self) -> list[float]:
        """Per-motor force rendering damping from the most recent bulk compute."""
        return list(self._last_force_rendering_damping_cmds)

    @property
    def last_force_rendering_cmds(self) -> list[float]:
        """Per-motor force-rendering-only current from the most recent bulk compute."""
        return list(self._last_force_rendering_cmds)
        
    def compute_current_command(self, motor_data: MotorData, params: ControlParams,
                              time: float = 0.0, follower_data: Optional[MotorData] = None,
                              verbose: bool = False) -> float:
        """
        Compute the total current command by combining all enabled control laws.
        
        Args:
            motor_data: Current motor state data
            params: Control parameters for this motor
            time: Current time
            follower_data: Follower motor state data for force rendering
            verbose: Whether to print detailed command breakdown
            
        Returns:
            Total current command in mA
        """
        total_command = 0.0
        
        # Zero current (always applied if no other laws are enabled)
        if not any([params.enable_force_rendering,
                   params.enable_force_rendering_damping,
                   params.enable_coulomb_comp,
                   params.enable_damping_comp,
                   params.enable_static_comp]):
            return CurrentControlLaws.zero_current(motor_data, params, time)
        
        # Calculate individual contributions
        force_cmd = 0.0
        force_rendering_damping_cmd = 0.0
        coulomb_cmd = 0.0
        damping_cmd = 0.0
        static_cmd = 0.0
        
        # Force rendering
        if params.enable_force_rendering:
            prev_command = self.prev_force_commands.get(motor_data.motor_id, 0.0)
            force_cmd = CurrentControlLaws.force_rendering(
                motor_data, params, time, follower_data, prev_command
            )
            total_command += force_cmd
            self.prev_force_commands[motor_data.motor_id] = force_cmd
        self._current_force_rendering_cmd = force_cmd

        # Force rendering damping
        if params.enable_force_rendering_damping:
            force_rendering_damping_cmd = CurrentControlLaws.force_rendering_damping(
                motor_data, params, time, follower_data
            )
            total_command += force_rendering_damping_cmd

        self._current_force_rendering_damping_cmd = force_rendering_damping_cmd
        
        # Coulomb friction compensation
        if params.enable_coulomb_comp:
            coulomb_cmd = CurrentControlLaws.coulomb_friction_compensation(motor_data, params, time)
            total_command += coulomb_cmd
        
        # Damping compensation
        if params.enable_damping_comp:
            damping_cmd = CurrentControlLaws.damping_compensation(motor_data, params, time)
            total_command += damping_cmd
        
        # Static friction compensation
        if params.enable_static_comp:
            static_cmd = CurrentControlLaws.static_friction_compensation(motor_data, params, time)
            total_command += static_cmd
        
        # Clamp to motor limits (assuming XM430 with 2.3A max current = 2300 mA)
        max_current = 2300  # mA
        total_command = np.clip(total_command, -max_current, max_current)
        
        # Print detailed breakdown if verbose
        if verbose:
            print(f"Motor {motor_data.motor_id} computed torques: "
                  f"{0.0:6.3f} (cmd) + {force_cmd:6.3f} (force) + "
                  f"{force_rendering_damping_cmd:6.3f} (force_rendering_damping) + "
                  f"{coulomb_cmd:6.3f} (coulomb) + {damping_cmd:6.3f} (damping) + {static_cmd:6.3f} (static)")
            follower_current = follower_data.current if follower_data is not None else 0.0
            print(f"Follower current: {follower_current:6.3f}")
            print(f"Commanded torque: {total_command:6.3f}")
        
        return total_command
    
    def compute_bulk_current_commands(self, motor_data_list: List[MotorData],
                                    params_list: List[ControlParams],
                                    time: float = 0.0,
                                    follower_data_list: Optional[List[MotorData]] = None,
                                    verbose: bool = False) -> List[float]:
        """
        Compute current commands for multiple motors.
        
        Args:
            motor_data_list: List of motor state data
            params_list: List of control parameters for each motor
            time: Current time
            follower_data_list: List of follower motor state data for force rendering
            verbose: Whether to print detailed command breakdown
            
        Returns:
            List of current commands in mA
        """
        if follower_data_list is None:
            follower_data_list = [None] * len(motor_data_list)

        commands = []
        force_rendering_damping_cmds: list[float] = []
        force_rendering_cmds: list[float] = []
        for i, (motor_data, params) in enumerate(zip(motor_data_list, params_list)):
            follower_data = follower_data_list[i] if i < len(follower_data_list) else None
            command = self.compute_current_command(
                motor_data, params, time, follower_data, verbose
            )
            commands.append(command)
            force_rendering_damping_cmds.append(self._current_force_rendering_damping_cmd)
            force_rendering_cmds.append(self._current_force_rendering_cmd)

        # Used for plotting only: 
        self._last_force_rendering_damping_cmds = force_rendering_damping_cmds
        self._last_force_rendering_cmds = force_rendering_cmds
        
        return commands
    
    def reset_force_rendering_history(self):
        """Reset the force rendering command history."""
        self.prev_force_commands.clear()


def create_control_params_from_config(motor_id: int, config: dict) -> ControlParams:
    """
    Create ControlParams from configuration dictionary.
    
    Args:
        motor_id: Motor ID
        config: Configuration dictionary containing control parameters
        
    Returns:
        ControlParams object
    """

    # Handle conflicts between force rendering damping and damping compensation
    enable_force_rendering_damping = bool(
        config.get("enable_force_rendering_damping", False)
    )
    enable_damping_comp = bool(config.get("enable_damping_comp", False))
    if enable_force_rendering_damping:
        enable_damping_comp = False

    return ControlParams(
        motor_id=motor_id,
        # Force rendering
        enable_force_rendering=config.get('enable_force_rendering', False),
        force_rendering_gain=config.get('force_rendering_gain', 0.0),
        force_rendering_alpha=config.get('force_rendering_alpha', 0.95),
        force_rendering_threshold=config.get('force_rendering_threshold', 0.0),
        force_rendering_threshold_positive=config.get(
            'force_rendering_threshold_positive',
            config.get('force_rendering_threshold', 0.0),
        ),
        force_rendering_threshold_negative=config.get(
            'force_rendering_threshold_negative',
            config.get('force_rendering_threshold', 0.0),
        ),
        gradual_rendering=config.get('gradual_rendering', False),
        force_rendering_max_current=config.get('force_rendering_max_current', 200.0),
        enable_force_rendering_adaptive_gain=bool(
            config.get('enable_force_rendering_adaptive_gain', False)
        ),
        force_rendering_adaptive_velocity_coeff=float(
            config.get('force_rendering_adaptive_velocity_coeff', 0.0)
        ),
        command_rate_limit_mA_per_s=float(
            config.get('command_rate_limit_mA_per_s', 0.0)
        ),
        force_rendering_flexion_only=bool(
            config.get('force_rendering_flexion_only', False)
        ),
        force_rendering_flexion_direction_sign=float(
            config.get('force_rendering_flexion_direction_sign', 1.0)
        ),
        
        # Force rendering damping
        enable_force_rendering_damping=enable_force_rendering_damping,
        force_rendering_damping_gain=float(
            config.get('force_rendering_damping_gain', 0.0)
        ),
        force_rendering_damping_velocity_threshold=float(
            config.get('force_rendering_damping_velocity_threshold', 0.0)
        ),
        force_rendering_damping_max_current=float(
            config.get('force_rendering_damping_max_current', 200.0)
        ),
        
        # Friction compensation
        enable_coulomb_comp=config.get('enable_coulomb_comp', False),
        mu_coulomb_cw=config.get('mu_coulomb_cw', 0.0),
        mu_coulomb_ccw=config.get('mu_coulomb_ccw', 0.0),
        enable_damping_comp=enable_damping_comp,
        mu_damping=config.get('mu_damping', 0.0),
        enable_static_comp=config.get('enable_static_comp', False),
        mu_static_amplitude=config.get('mu_static_amplitude', 0.0),
        static_frequency=config.get('static_frequency', 10.0),
        static_threshold_cw=config.get('static_threshold_cw', 1.0),
        static_threshold_ccw=config.get('static_threshold_ccw', 1.0),
    )