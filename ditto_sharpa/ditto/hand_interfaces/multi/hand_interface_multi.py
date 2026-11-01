#!/usr/bin/env python3
"""
Multi-Motor Hand Interface Module

This module provides a hand interface that naturally handles multiple motors
with vectorized operations.
"""

import time
import math
import numpy as np
from typing import List, Dict, Optional, Tuple
from abc import ABC, abstractmethod

from dynamixel_u2d2 import U2D2Interface, FakeU2D2Interface
from .joint_converter_multi import JointConverter
from .hand_data import HandData
from .current_control_multi import CurrentControlLaws, ControlParams, create_control_params_from_config


class BaseHandInterface(ABC):
    """Base class for multi-motor hand hardware interfaces."""
    
    def __init__(self, config, chain_type: str):
        """
        Initialize the hand interface.
        
        Args:
            config: Hydra configuration object
            chain_type: Type of chain ('leader' or 'follower')
        """
        self.config = config
        self.chain_type = chain_type
        self.chain_config = config.hand_config[chain_type]
        self.motor_ids = self.chain_config.motor_ids
        self.mode = self.chain_config.mode
        self.verbose = config.get('verbose', False)
        
        # Create joint converter for all motors
        self.joint_converter = JointConverter(
            self.motor_ids, 
            self.config.motor_models, 
            self.config.joint_configs
        )
        
        # Create current control laws
        self.current_control = CurrentControlLaws(
            config.hand_config.get('control_frequency', config.get('control_frequency', 50))
        )
        
        # Create control parameters from config
        joint_settings = getattr(self.chain_config, 'joint_settings', [])
        self.control_params = create_control_params_from_config(self.motor_ids, joint_settings)
        
        # Control settings
        self.control_frequency = config.hand_config.get('control_frequency', config.get('control_frequency', 50))
        
    @abstractmethod
    def connect(self) -> None:
        """Connect to the hardware interface."""
        pass
    
    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the hardware interface."""
        pass
    
    @abstractmethod
    def setup_motors(self) -> None:
        """Set up motors according to their configurations."""
        pass
    
    @abstractmethod
    def read_states(self) -> HandData:
        """Read current state of all motors."""
        pass
    
    @abstractmethod
    def send_position_commands(self, joint_angles: np.ndarray) -> None:
        """Send joint angle targets to motors."""
        pass
    
    @abstractmethod
    def send_current_commands(self, currents: np.ndarray) -> None:
        """Send current commands to motors."""
        pass
    
    def get_joint_angles(self) -> np.ndarray:
        """Get current joint angles in radians."""
        hand_data = self.read_states()
        return hand_data.joint_angles
    
    def get_joint_velocities(self) -> np.ndarray:
        """Get current joint velocities in rad/s."""
        hand_data = self.read_states()
        return hand_data.joint_velocities


class RealHandInterface(BaseHandInterface):
    """Real hand interface using U2D2 hardware."""
    
    def __init__(self, config, chain_type: str, port: str = None, baudrate: int = 4000000, use_fake: bool = False):
        super().__init__(config, chain_type)
        self.u2d2 = None
        self.port = port
        self.baudrate = baudrate
        self.use_fake = use_fake
        
        # Pre-allocate numpy arrays for performance (reused each cycle)
        n_motors = len(self.motor_ids)
        self._positions_array = np.empty(n_motors, dtype=np.int32)
        self._velocities_array = np.empty(n_motors, dtype=np.int32)
        self._currents_array = np.empty(n_motors, dtype=np.int32)
        
    def connect(self) -> None:
        """Connect to U2D2 interface."""
        if self.u2d2 is not None:
            return
            
        if self.port is None:
            raise ValueError(f"Port not specified for {self.chain_type} hand interface")
            
        if self.verbose:
            print(f"🔌 Connecting {self.chain_type} hand to U2D2 on {self.port}...")
        
        # Create U2D2 interface
        if self.use_fake:
            self.u2d2 = FakeU2D2Interface(usb_port=self.port, baudrate=self.baudrate, motor_ids=self.motor_ids, verbose=self.verbose)
        else:
            self.u2d2 = U2D2Interface(usb_port=self.port, baudrate=self.baudrate, motor_ids=self.motor_ids)
        
        if self.verbose:
            if self.use_fake:
                print(f"✅ {self.chain_type.capitalize()} Fake U2D2 Interface Initialized! (Testing Mode)")
            else:
                print(f"✅ {self.chain_type.capitalize()} U2D2 Interface Initialized!")
    
    def disconnect(self) -> None:
        """Disconnect from U2D2 interface."""
        if self.verbose:
            print(f"🧹 Disconnecting {self.chain_type} hand...")
        
        if self.u2d2 is not None:
            # Disable torque on all motors
            for motor_id in self.motor_ids:
                try:
                    self.u2d2.disable_torque(motor_id)
                except:
                    pass  # Ignore errors during disconnect
            
            if not self.use_fake and hasattr(self.u2d2, 'close'):
                self.u2d2.close()
            self.u2d2 = None
        
        if self.verbose:
            print(f"✅ {self.chain_type.capitalize()} Interface Disconnected!")
    
    def setup_motors(self) -> None:
        """Set up motors according to their configurations."""
        if self.u2d2 is None:
            raise RuntimeError("U2D2 interface not connected. Call connect() first.")
        
        if self.verbose:
            print(f"🔧 Setting up {self.chain_type} motors...")
        
        for motor_id in self.motor_ids:
            self._setup_motor(motor_id)
        
        if self.verbose:
            print(f"✅ {self.chain_type.capitalize()} motors ready!")
    
    def _setup_motor(self, motor_id: int) -> None:
        """Set up a single motor based on its configuration."""
        # Get motor model for this specific motor
        motor_model = self._get_motor_model(motor_id)
        
        # Build settings hierarchy: defaults → chain → motor-specific
        settings = self._build_motor_settings(motor_id, motor_model)
        
        if self.verbose:
            print(f"  {self.chain_type.capitalize()} Motor {motor_id}: {self.mode} mode...")
        
        # Apply mode-specific setup
        self._setup_motor_mode(motor_id, self.mode, settings)
    
    def _get_motor_model(self, motor_id: int) -> str:
        """Get the motor model for a specific motor."""
        # Start with default from Hydra config
        motor_model = self.config.joint_configs.default.motor_model
        
        # Check for motor-specific override in joint settings
        if hasattr(self.chain_config, 'joint_settings') and self.chain_config.joint_settings:
            for joint_config in self.chain_config.joint_settings:
                if joint_config['motor_id'] == motor_id and 'joint_settings' in joint_config:
                    motor_model = joint_config['joint_settings'].get('motor_model', motor_model)
                    break
        
        return motor_model
    
    def _build_motor_settings(self, motor_id: int, motor_model: str) -> dict:
        """Build motor settings hierarchy: defaults → chain → motor-specific."""
        # Start with motor model defaults
        motor_model_config = self.config.motor_models[motor_model]
        settings = {
            'p_gain': motor_model_config.get('default_p_gain', 500),
            'i_gain': motor_model_config.get('default_i_gain', 0),
            'd_gain': motor_model_config.get('default_d_gain', 0),
            'velocity_limit': motor_model_config.get('default_velocity_limit', 150)
        }
        
        # Apply chain-level settings
        if hasattr(self.chain_config, 'general_settings'):
            chain_settings = dict(self.chain_config.general_settings)
            settings.update(chain_settings)
        
        # Apply motor-specific settings from joint_settings if they exist
        if hasattr(self.chain_config, 'joint_settings') and self.chain_config.joint_settings:
            for joint_config in self.chain_config.joint_settings:
                if joint_config['motor_id'] == motor_id and 'motor_settings' in joint_config:
                    motor_settings = dict(joint_config['motor_settings'])
                    settings.update(motor_settings)
                    if self.verbose:
                        print(f"    Applying custom motor settings for motor {motor_id}: {motor_settings}")
                    break
        
        # Validate final settings against motor model limits
        self._validate_motor_settings(motor_id, motor_model, settings)
        return settings
    
    def _setup_motor_mode(self, motor_id: int, chain_mode: str, settings: dict) -> None:
        """Set up motor for the specified control mode."""
        # Disable torque to change operating mode
        self.u2d2.disable_torque(motor_id)

        if chain_mode == "torque_off":
            # Just disable torque, no other setup needed
            pass
            
        elif chain_mode == "current":
            # Set up current control mode
            self.u2d2.set_motor_mode(motor_id, 'current')
            
            # Apply current control settings
            if settings.get('current_limit') is not None:
                self.u2d2.set_current_limit(motor_id, settings.get('current_limit'))
            
            # Enable torque
            self.u2d2.enable_torque(motor_id)
            
            # Send current command
            current_cmd = settings.get('current_command', 0)
            self.u2d2.set_goal_current(motor_id, current_cmd)
            
        elif chain_mode == "position":
            # Set up position control mode
            self.u2d2.set_motor_mode(motor_id, 'position')
            
            # Apply position control settings
            if settings.get('velocity_limit') is not None:
                self.u2d2.set_velocity_limit(motor_id, settings.get('velocity_limit'))
            if settings.get('p_gain') is not None:
                self.u2d2.set_position_p_gain(motor_id, settings.get('p_gain'))
            if settings.get('i_gain') is not None:
                self.u2d2.set_position_i_gain(motor_id, settings.get('i_gain'))
            if settings.get('d_gain') is not None:
                self.u2d2.set_position_d_gain(motor_id, settings.get('d_gain'))
            
            # Enable torque
            self.u2d2.enable_torque(motor_id)
            
        else:
            raise ValueError(f"Unsupported control mode: {chain_mode}")
    
    def _validate_motor_settings(self, motor_id: int, motor_model: str, settings: dict) -> None:
        """Validate motor settings against motor model specifications."""
        motor_model_config = self.config.motor_models[motor_model]
        
        # Validate velocity limit
        if 'velocity_limit' in settings:
            velocity_limit = settings['velocity_limit']
            max_velocity_units = motor_model_config.get('max_velocity_units', 201)
            
            if velocity_limit > max_velocity_units:
                max_velocity_rpm = motor_model_config.get('max_velocity_rpm', 46)
                print(f"⚠️  Motor {motor_id}: Velocity limit {velocity_limit} exceeds max {max_velocity_units} units ({max_velocity_rpm} RPM)")
                print(f"    Clamping velocity limit to {max_velocity_units} units")
                settings['velocity_limit'] = max_velocity_units
        
        # Validate current limit
        if 'current_limit' in settings:
            current_limit = settings['current_limit']
            max_current_units = motor_model_config.get('max_current_units', 2300)
            
            if current_limit > max_current_units:
                max_current_amps = motor_model_config.get('max_current_amps', 2.3)
                print(f"⚠️  Motor {motor_id}: Current limit {current_limit} mA exceeds max {max_current_units} mA ({max_current_amps} A)")
                print(f"    Clamping current limit to {max_current_units} mA")
                settings['current_limit'] = max_current_units
        
        # Validate current command
        if 'current_command' in settings:
            current_command = settings['current_command']
            max_current_units = motor_model_config.get('max_current_units', 2300)
            
            if abs(current_command) > max_current_units:
                max_current_amps = motor_model_config.get('max_current_amps', 2.3)
                print(f"⚠️  Motor {motor_id}: Current command {current_command} mA exceeds max {max_current_units} mA ({max_current_amps} A)")
                print(f"    Clamping current command to {max_current_units} mA")
                settings['current_command'] = max_current_units if current_command > 0 else -max_current_units
    
    def read_states(self) -> HandData:
        """
        Read current state of all motors.
        
        Returns:
            HandData object with all motor states
        """
        if self.u2d2 is None:
            raise RuntimeError("U2D2 interface not connected. Call connect() first.")
        
        try:
            # Use sync read for better performance
            positions, velocities, currents = self.u2d2.sync_read_state()
            
            # Copy into pre-allocated arrays (faster than np.array() for small arrays)
            self._positions_array[:] = positions
            self._velocities_array[:] = velocities
            self._currents_array[:] = currents
            
            # VECTORIZED joint space conversion (no for loops!)
            joint_angles = self.joint_converter.motor_to_joint(self._positions_array)
            joint_velocities = self.joint_converter.motor_to_joint_velocity(self._velocities_array)
            
            # Create HandData object (use pre-allocated arrays)
            hand_data = HandData(
                motor_ids=self.motor_ids,
                positions=self._positions_array,
                velocities=self._velocities_array,
                currents=self._currents_array,
                joint_angles=joint_angles,
                joint_velocities=joint_velocities
            )
            
            return hand_data
                
        except Exception as e:
            if self.verbose:
                print(f"⚠️ Error reading {self.chain_type} motor states: {e}")
            # Return empty HandData
            return HandData(
                motor_ids=self.motor_ids,
                positions=np.zeros(len(self.motor_ids), dtype=np.int32),
                velocities=np.zeros(len(self.motor_ids), dtype=np.int32),
                currents=np.zeros(len(self.motor_ids), dtype=np.int32),
                joint_angles=np.zeros(len(self.motor_ids), dtype=np.float64),
                joint_velocities=np.zeros(len(self.motor_ids), dtype=np.float64)
            )
    
    def send_position_commands(self, joint_angles: np.ndarray) -> None:
        """
        Send joint angle targets to motors.
        
        Args:
            joint_angles: Array of joint angles in radians
        """
        if self.u2d2 is None:
            raise RuntimeError("U2D2 interface not connected. Call connect() first.")
        
        if len(joint_angles) != len(self.motor_ids):
            raise ValueError(f"Expected {len(self.motor_ids)} joint angles, got {len(joint_angles)}")
        
        # VECTORIZED joint angle clamping and conversion (no for loops!)
        clamped_angles = self.joint_converter.clamp_joint_angles(joint_angles)
        motor_positions = self.joint_converter.joint_to_motor(clamped_angles)
        
        # Debug output for joint limits
        if self.verbose:
            for i, (original, clamped) in enumerate(zip(joint_angles, clamped_angles)):
                if abs(original - clamped) > 0.01:  # Only show if clamped
                    joint_deg = math.degrees(original)
                    clamped_deg = math.degrees(clamped)
                    min_deg, max_deg = self.joint_converter.get_joint_limits_degrees()
                    print(f"    🔒 Motor {self.motor_ids[i]}: Joint angle {joint_deg:.1f}° clamped to {clamped_deg:.1f}° (limits: {min_deg[i]:.1f}° to {max_deg[i]:.1f}°)")
        
        # Send to motors using sync write
        self.u2d2.sync_write_positions(motor_positions.tolist())
    
    def send_current_commands(self, currents: np.ndarray) -> None:
        """
        Send current commands to motors.
        
        Args:
            currents: Array of current commands
        """
        if self.u2d2 is None:
            raise RuntimeError("U2D2 interface not connected. Call connect() first.")
        
        if len(currents) != len(self.motor_ids):
            raise ValueError(f"Expected {len(self.motor_ids)} current commands, got {len(currents)}")
        
        # Use sync write for better performance
        self.u2d2.sync_write_currents(currents.tolist())


class SimulatedHandInterface(BaseHandInterface):
    """Simulated hand interface for testing without hardware."""
    
    def __init__(self, config, chain_type: str):
        super().__init__(config, chain_type)
        # Initialize with default values
        self.simulated_states = HandData(
            motor_ids=self.motor_ids,
            positions=np.full(len(self.motor_ids), 2048, dtype=np.int32),  # Center position
            velocities=np.zeros(len(self.motor_ids), dtype=np.int32),
            currents=np.zeros(len(self.motor_ids), dtype=np.int32),
            joint_angles=np.zeros(len(self.motor_ids), dtype=np.float64),
            joint_velocities=np.zeros(len(self.motor_ids), dtype=np.float64)
        )
    
    def connect(self) -> None:
        """Connect to simulated interface."""
        if self.verbose:
            print(f"🔌 Connecting to simulated {self.chain_type} hand...")
            print(f"✅ Simulated {self.chain_type.capitalize()} Interface Initialized!")
    
    def disconnect(self) -> None:
        """Disconnect from simulated interface."""
        if self.verbose:
            print(f"🧹 Disconnecting simulated {self.chain_type} hand...")
    
    def setup_motors(self) -> None:
        """Set up simulated motors."""
        if self.verbose:
            print(f"🔧 Setting up simulated {self.chain_type} motors...")
            print(f"✅ Simulated {self.chain_type.capitalize()} motors ready!")
    
    def read_states(self) -> HandData:
        """Read simulated motor states."""
        return HandData(
            motor_ids=self.simulated_states.motor_ids,
            positions=self.simulated_states.positions.copy(),
            velocities=self.simulated_states.velocities.copy(),
            currents=self.simulated_states.currents.copy(),
            joint_angles=self.simulated_states.joint_angles.copy(),
            joint_velocities=self.simulated_states.joint_velocities.copy()
        )
    
    def send_position_commands(self, joint_angles: np.ndarray) -> None:
        """Send joint angle targets to simulated motors."""
        if len(joint_angles) != len(self.motor_ids):
            raise ValueError(f"Expected {len(self.motor_ids)} joint angles, got {len(joint_angles)}")
        
        # VECTORIZED conversion (no for loops!)
        clamped_angles = self.joint_converter.clamp_joint_angles(joint_angles)
        motor_positions = self.joint_converter.joint_to_motor(clamped_angles)
        
        # Update simulated states
        self.simulated_states.positions = motor_positions
        self.simulated_states.joint_angles = clamped_angles
    
    def send_current_commands(self, currents: np.ndarray) -> None:
        """Send current commands to simulated motors."""
        if len(currents) != len(self.motor_ids):
            raise ValueError(f"Expected {len(self.motor_ids)} current commands, got {len(currents)}")
        
        # Update simulated states
        self.simulated_states.currents = currents.astype(np.int32)


def create_hand_interface_multi(config, chain_type: str, interface_type: str = "real", 
                               port: str = None, baudrate: int = 4000000, use_fake: bool = False) -> BaseHandInterface:
    """
    Factory function to create hand interfaces.
    
    Args:
        config: Hydra configuration object
        chain_type: Type of chain ('leader' or 'follower')
        interface_type: Type of interface ('real' or 'simulated')
        port: USB port for real interface (e.g., '/dev/ttyUSB0')
        baudrate: Baudrate for real interface
        use_fake: Whether to use fake U2D2 interface for testing
    
    Returns:
        Hand interface instance
    """
    if interface_type == "real":
        return RealHandInterface(config, chain_type, port, baudrate, use_fake)
    elif interface_type == "simulated":
        return SimulatedHandInterface(config, chain_type)
    else:
        raise ValueError(f"Unknown interface type: {interface_type}")
