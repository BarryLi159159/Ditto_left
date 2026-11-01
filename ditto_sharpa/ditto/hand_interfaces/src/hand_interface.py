#!/usr/bin/env python3
"""
Hand Interface Module

Provides hardware abstraction for hand control with support for different
control modes and motor configurations.
"""

import time
import math
from typing import List, Dict, Optional, Tuple
from abc import ABC, abstractmethod

from dynamixel_u2d2 import U2D2Interface, FakeU2D2Interface
from .joint_converter import JointConverter
from .motor_data import MotorData


class BaseHandInterface(ABC):
    """Base class for hand hardware interfaces."""
    
    def __init__(self, config, chain_type: str):
        """
        Initialize the hand interface.
        
        Args:
            config: Hydra configuration object
            chain_type: Type of chain ('leader' or 'follower')
        """
        self.config = config
        self.chain_type = chain_type
        self.verbose = config.hand_config.get('verbose', config.get('verbose', True))
        
        # Get chain configuration
        self.chain_config = config.hand_config[chain_type]
        self.motor_ids = list(self.chain_config.motor_ids)
        self.mode = self.chain_config.mode
        self._sync_read_mode = self._resolve_sync_read_mode()

        # Resolve motor models once (single source of truth)
        # List is aligned with self.motor_ids order.
        self.motor_models = self._resolve_motor_models()
        # Public mapping for debugging/telemetry (used by config summary, etc.)
        self.motor_model_by_id = {mid: mm for mid, mm in zip(self.motor_ids, self.motor_models)}
        
        # Create joint converters
        self.joint_converters = self._create_joint_converters()
        self._converter_by_motor_id = {c.motor_id: c for c in self.joint_converters}
        
        # Control settings
        self.control_frequency = config.hand_config.get('control_frequency', config.get('control_frequency', 50))

    def _resolve_sync_read_mode(self) -> str:
        """Which registers to sync-read: ``position``, ``pos_vel``, or ``all`` states."""
        mode = getattr(self.chain_config, "sync_read_mode", None)
        if mode is None:
            # Encoder-only leaders (Ditto → Sharpa teleop) need joint angles only.
            return "position" if self.mode == "torque_off" else "all"
        mode = str(mode).lower()
        if mode not in ("position", "pos_vel", "all"):
            raise ValueError(
                f"sync_read_mode must be 'position', 'pos_vel', or 'all', got {mode!r}"
            )
        return mode
        
    def _get_motor_model_config(self, motor_model: str) -> Dict:
        """Get motor model configuration, raising error if not found."""
        motor_models_config = self.config.motor_models
        if motor_model not in motor_models_config:
            raise ValueError(f"Unsupported motor model: {motor_model}. Available: {list(motor_models_config.keys())}")
        return motor_models_config[motor_model]

    def _resolve_motor_models(self) -> List[str]:
        """
        Resolve motor_model per motor_id using:
          per-motor override in chain_config.joint_settings[].joint_settings.motor_model
          -> config.joint_configs.default.motor_model

        Returns list aligned with self.motor_ids.
        """
        default_model = str(self.config.joint_configs.default.motor_model)
        override_by_id: Dict[int, str] = {}

        if hasattr(self.chain_config, 'joint_settings') and self.chain_config.joint_settings:
            for entry in self.chain_config.joint_settings:
                motor_id = entry.get('motor_id')
                js = entry.get('joint_settings', {}) or {}
                motor_model = js.get('motor_model')
                if motor_id is not None and motor_model:
                    override_by_id[int(motor_id)] = str(motor_model)

        return [override_by_id.get(int(mid), default_model) for mid in self.motor_ids]
    
    def _get_joint_config(self, motor_model: str, custom_settings: Optional[Dict] = None) -> Dict:
        """
        Get joint configuration for a motor model.
        
        Config hierarchy (highest to lowest priority):
        1. custom_settings (if provided) - motor-specific overrides
        2. joint_configs[motor_model] - motor model-specific config
        3. joint_configs['default'] - default config
        
        Args:
            motor_model: Motor model name
            custom_settings: Optional dict of custom joint settings that override defaults
            
        Returns:
            Joint configuration dictionary with all settings merged
        """
        joint_configs = self.config.joint_configs
        
        if custom_settings is not None:
            # Merge: default -> custom_settings (custom overrides default)
            joint_config = dict(joint_configs.get('default', {}))
            joint_config.update(custom_settings)
            return joint_config
        else:
            # Use motor-specific if available, else default
            if motor_model in joint_configs:
                return joint_configs[motor_model]
            else:
                return joint_configs.get('default', {})
    
    def _create_joint_converters(self) -> List[JointConverter]:
        """
        Create joint converters for all motors in this chain.
        
        Per-motor custom settings from chain_config.joint_settings override defaults.
        Returns List of JointConverter instances, one per motor in chain order.
        """
        # Build mapping of motor_id -> custom joint settings if available
        joint_config_map = {}
        if hasattr(self.chain_config, 'joint_settings') and self.chain_config.joint_settings:
            for joint_config_dict in self.chain_config.joint_settings:
                motor_id = joint_config_dict['motor_id']
                joint_config_map[motor_id] = joint_config_dict.get('joint_settings', {})
        
        converters = []
        for motor_id, motor_model in zip(self.motor_ids, self.motor_models):
            # Determine motor model and joint settings
            if motor_id in joint_config_map:
                # Custom settings for this motor
                custom_settings = joint_config_map[motor_id]
                joint_config = self._get_joint_config(motor_model, custom_settings)
                if self.verbose:
                    print(f"    Using custom joint config for motor {motor_id}: {joint_config}")
            else:
                # Default settings
                joint_config = self._get_joint_config(motor_model)
                if self.verbose:
                    print(f"    Using default joint config for motor {motor_id}")
            
            # Get motor model config and create converter
            motor_model_config = self._get_motor_model_config(motor_model)
            converter = JointConverter(motor_id, motor_model_config, joint_config, motor_model=motor_model)
            converters.append(converter)
        
        return converters
    
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
    def read_states(self) -> List[MotorData]:
        """Read current state of all motors."""
        pass
    
    @abstractmethod
    def send_position_commands(self, joint_angles: List[float]) -> None:
        """Send joint angle targets to motors."""
        pass
    
    @abstractmethod
    def send_current_commands(self, currents: List[int]) -> None:
        """Send current commands to motors."""
        pass
    
    def get_joint_angles(self) -> List[float]:
        """Get current joint angles in radians."""
        motor_data = self.read_states()
        return [data.joint_angle for data in motor_data]
    
    def get_joint_velocities(self) -> List[float]:
        """Get current joint velocities in rad/s."""
        motor_data = self.read_states()
        return [data.joint_velocity for data in motor_data]


class RealHandInterface(BaseHandInterface):
    """Real hand interface using U2D2 hardware."""
    
    def __init__(self, config, chain_type: str, port: str = None, baudrate: int = 4000000, use_fake: bool = False):
        super().__init__(config, chain_type)
        self.u2d2 = None
        self.port = port
        self.baudrate = baudrate
        self.use_fake = use_fake
        
    def connect(self) -> None:
        """Connect to the U2D2 interface."""
        if self.u2d2 is not None:
            return
        
        if self.port is None:
            raise ValueError(f"Port not specified for {self.chain_type} hand interface")
        
        if self.verbose:
            print(f"🔌 Connecting {self.chain_type} hand to U2D2 on {self.port}...")
        
        if self.use_fake:
            self.u2d2 = FakeU2D2Interface(self.port, self.baudrate, motor_ids=self.motor_ids, verbose=self.verbose)
        else:
            self.u2d2 = U2D2Interface(self.port, self.baudrate, motor_ids=self.motor_ids)

        if self._sync_read_mode == "position":
            self.u2d2.init_specific_group_sync_read("position")
        elif self._sync_read_mode == "pos_vel":
            self.u2d2.init_group_sync_read_pos_vel(self.motor_ids)
        
        if self.verbose:
            if self.use_fake:
                print(f"✅ {self.chain_type.capitalize()} Fake U2D2 Interface Initialized! (Testing Mode)")
            else:
                print(f"✅ {self.chain_type.capitalize()} U2D2 Interface Initialized!")
    
    def disconnect(self) -> None:
        """Disconnect from the U2D2 interface."""
        if self.verbose:
            print(f"🧹 Disconnecting {self.chain_type} hand...")
        
        if self.u2d2 is not None:
            for motor_id in self.motor_ids:
                try:
                    self.u2d2.disable_torque(motor_id)
                except:
                    print(f"Error disabling torque for motor {motor_id}")
            self.u2d2.close()
            self.u2d2 = None
    
    def setup_motors(self) -> None:
        """Set up all motors according to their configurations."""
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
        # Single source of truth: resolved at init time
        return self.motor_model_by_id.get(int(motor_id), str(self.config.joint_configs.default.motor_model))
    
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
                # settings current_limit is in mA; convert to raw motor units
                converter = self._converter_by_motor_id.get(motor_id)
                if converter is None:
                    raise RuntimeError(f"No JointConverter found for motor {motor_id}")
                self.u2d2.set_current_limit(
                    motor_id,
                    converter.mA_to_motor_current_raw(settings.get('current_limit'))
                )
            
            # Enable torque
            self.u2d2.enable_torque(motor_id)
            
            # Send current command
            current_cmd = settings.get('current_command', 0)
            converter = self._converter_by_motor_id.get(motor_id)
            if converter is None:
                raise RuntimeError(f"No JointConverter found for motor {motor_id}")
            self.u2d2.set_goal_current(motor_id, converter.mA_to_motor_current_raw(current_cmd))
            
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
    
    def read_states(self) -> List[MotorData]:
        """Read current state of all motors."""
        if self.u2d2 is None:
            raise RuntimeError("U2D2 interface not connected. Call connect() first.")

        try:
            if self._sync_read_mode == "position":
                positions = self.u2d2.sync_read_specific("position")
                velocities = [0] * len(self.motor_ids)
                currents = [0] * len(self.motor_ids)
            elif self._sync_read_mode == "pos_vel":
                positions, velocities = self.u2d2.sync_read_pos_vel()
                currents = [0] * len(self.motor_ids)
            else:
                positions, velocities, currents = self.u2d2.sync_read_state()

            motor_data_list = []
            for i, motor_id in enumerate(self.motor_ids):
                raw_position = int(positions[i])
                raw_velocity = int(velocities[i])
                raw_current = int(currents[i])

                motor_angle = self.joint_converters[i].motor_pos_raw_to_motor_angle(raw_position)
                motor_velocity = self.joint_converters[i].motor_vel_raw_to_motor_angular_vel(raw_velocity)
                motor_current = self.joint_converters[i].motor_current_raw_to_mA(raw_current)

                joint_angle = self.joint_converters[i].motor_angle_to_joint_angle(motor_angle)
                joint_velocity = self.joint_converters[i].motor_angular_velocity_to_joint_velocity(
                    motor_velocity
                )

                motor_data_list.append(
                    MotorData(
                        motor_id=motor_id,
                        raw_position=raw_position,
                        raw_velocity=raw_velocity,
                        raw_current=raw_current,
                        position=motor_angle,
                        velocity=motor_velocity,
                        current=motor_current,
                        joint_angle=joint_angle,
                        joint_velocity=joint_velocity,
                    )
                )

            return motor_data_list

        except Exception as e:
            if self.verbose:
                print(f"⚠️ Error reading {self.chain_type} motor states: {e}")
            return []

    def send_position_commands(self, joint_angles: List[float]) -> None:
        """Send joint angle targets to motors."""
        if self.u2d2 is None:
            raise RuntimeError("U2D2 interface not connected. Call connect() first.")
        
        if len(joint_angles) != len(self.motor_ids):
            raise ValueError(f"Expected {len(self.motor_ids)} joint angles, got {len(joint_angles)}")
        
        # Convert joint angles to motor positions
        motor_positions = []
        for i, joint_angle in enumerate(joint_angles):
            # Clamp to limits
            clamped_angle = self.joint_converters[i].clamp_joint_angle(joint_angle)
            # Convert to motor position
            motor_position = self.joint_converters[i].joint_to_motor(clamped_angle)
            motor_positions.append(motor_position)
            
            # Debug output for joint limits
            if self.verbose and abs(joint_angle - clamped_angle) > 0.01:  # Only show if clamped
                joint_deg = math.degrees(joint_angle)
                clamped_deg = math.degrees(clamped_angle)
                min_deg, max_deg = [math.degrees(lim) for lim in self.joint_converters[i].get_joint_limits()]
                print(f"    🔒 Motor {self.motor_ids[i]}: Joint angle {joint_deg:.1f}° clamped to {clamped_deg:.1f}° (limits: {min_deg:.1f}° to {max_deg:.1f}°)")
        
        # Send to motors using sync write
        self.u2d2.sync_write_positions(motor_positions)
    
    def send_current_commands(self, currents: List[int]) -> None:
        """
        Send current commands to motors (mA).
        
        Args:
            currents: List of current commands in mA
        """
        if self.u2d2 is None:
            raise RuntimeError("U2D2 interface not connected. Call connect() first.")
        
        if len(currents) != len(self.motor_ids):
            raise ValueError(f"Expected {len(self.motor_ids)} current commands, got {len(currents)}")
        
        # Convert mA -> motor current raw units (model-dependent) then sync write
        raw_currents = [
            self.joint_converters[i].mA_to_motor_current_raw(currents[i])
            for i in range(len(self.motor_ids))
        ]
        self.u2d2.sync_write_currents(raw_currents)


class SimulatedHandInterface(BaseHandInterface):
    """Simulated hand interface for testing without hardware."""
    
    def __init__(self, config, chain_type: str):
        super().__init__(config, chain_type)
        self.simulated_states = [MotorData(
            motor_id=motor_id,
            position=0.0,  # rad (relative to zero)
            velocity=0.0,  # rad/s
            current=0.0,   # mA
            raw_position=2048,
            raw_velocity=0,
            raw_current=0,
            joint_angle=0.0,
            joint_velocity=0.0
        ) for motor_id in self.motor_ids]
    
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
    
    def read_states(self) -> List[MotorData]:
        """Read simulated motor states."""
        return self.simulated_states.copy()
    
    def send_position_commands(self, joint_angles: List[float]) -> None:
        """Send joint angle targets to simulated motors."""
        if len(joint_angles) != len(self.motor_ids):
            raise ValueError(f"Expected {len(self.motor_ids)} joint angles, got {len(joint_angles)}")
        
        # Update simulated states
        for i, joint_angle in enumerate(joint_angles):
            clamped_angle = self.joint_converters[i].clamp_joint_angle(joint_angle)
            motor_position = self.joint_converters[i].joint_to_motor(clamped_angle)
            
            self.simulated_states[i].joint_angle = clamped_angle
            self.simulated_states[i].raw_position = motor_position
            self.simulated_states[i].position = self.joint_converters[i].motor_position_to_motor_angle(motor_position)
    
    def send_current_commands(self, currents: List[int]) -> None:
        """Send current commands to simulated motors."""
        if len(currents) != len(self.motor_ids):
            raise ValueError(f"Expected {len(self.motor_ids)} current commands, got {len(currents)}")
        
        # Update simulated states
        for i, current in enumerate(currents):
            # currents passed in mA (for consistency with real interface)
            self.simulated_states[i].current = float(current)
            self.simulated_states[i].raw_current = self.joint_converters[i].mA_to_motor_current_raw(float(current))


def create_hand_interface(config, chain_type: str, interface_type: str = "real", 
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
