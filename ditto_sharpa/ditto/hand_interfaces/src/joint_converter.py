#!/usr/bin/env python3
"""
Joint Abstraction Layer for Teleop Control

This module provides abstraction between joint space (radians, limits) and motor space (encoder counts).
It handles conversion between radians and encoder counts, joint limits, and motor model configurations.
Uses Hydra configurations.
"""

import math
from typing import Dict, List, Optional, Tuple


class JointConverter:
    """Converts between joint space (radians) and motor space (encoder counts)."""
    
    def __init__(self, motor_id: int, motor_model_config: Dict, joint_config: Dict, motor_model: str = "UNKNOWN"):
        """
        Initialize joint converter.
        
        Args:
            motor_id: Motor ID
            motor_model_config: Motor model configuration from Hydra
            joint_config: Joint configuration from Hydra
        """
        self.motor_id = motor_id
        self.motor_model = str(motor_model)
        self.motor_model_config = motor_model_config
        self.joint_config = joint_config
        
        # Extract joint configuration values
        self.zero_position = joint_config.get('zero_position', 2048)
        self.min_angle = math.radians(joint_config.get('min_angle_deg', -90))
        self.max_angle = math.radians(joint_config.get('max_angle_deg', 90))
        self.transmission_ratio = joint_config.get('transmission_ratio', 1.0)
        self.direction = joint_config.get('direction', 1)
        
        # Validate configuration
        self._validate_config()
        
        # Calculate conversion factors
        resolution = motor_model_config.get('resolution', 4096)
        self.radians_per_count = (2 * math.pi) / resolution
        self.counts_per_radian = resolution / (2 * math.pi)

        # Dynamixel present velocity/current units are model-dependent (configured in motor_models/*.yaml)
        # - present_velocity_raw * default_velocity_units => rpm
        # - present_current_raw  * default_current_units  => mA
        self.velocity_unit_rpm = float(motor_model_config.get('default_velocity_units', 0.229))
        self.current_unit_mA = float(motor_model_config.get('default_current_units', 1.0))
        
        # Calculate joint limits in encoder counts
        self._calculate_joint_limits()
    
    def _validate_config(self):
        """Validate joint configuration."""
        if self.min_angle >= self.max_angle:
            raise ValueError(f"min_angle ({self.min_angle}) must be less than max_angle ({self.max_angle})")
        if self.transmission_ratio <= 0:
            raise ValueError(f"transmission_ratio ({self.transmission_ratio}) must be positive")
        if self.direction not in [1, -1]:
            raise ValueError(f"direction ({self.direction}) must be 1 or -1")
    
    def _calculate_joint_limits(self):
        """Calculate joint limits in encoder counts."""
        # Convert joint angles to motor angles
        min_motor_angle = self.min_angle * self.transmission_ratio * self.direction
        max_motor_angle = self.max_angle * self.transmission_ratio * self.direction
        
        # Convert to encoder counts relative to zero position
        min_counts = min_motor_angle * self.counts_per_radian
        max_counts = max_motor_angle * self.counts_per_radian
        
        # Convert to absolute encoder counts
        self.min_encoder_count = int(self.zero_position + min_counts)
        self.max_encoder_count = int(self.zero_position + max_counts)
        
        # Clamp to motor limits
        min_position = self.motor_model_config.get('min_position', 0)
        max_position = self.motor_model_config.get('max_position', 4095)
        self.min_encoder_count = max(self.min_encoder_count, min_position)
        self.max_encoder_count = min(self.max_encoder_count, max_position)
    
    def joint_to_motor(self, joint_angle: float) -> int:
        """
        Convert joint angle to motor position.
        
        Args:
            joint_angle: Joint angle in radians
            
        Returns:
            Motor position (encoder count)
            
        Raises:
            ValueError: If joint angle is outside limits
        """
        # Check joint limits
        if joint_angle < self.min_angle or joint_angle > self.max_angle:
            raise ValueError(f"Joint angle {joint_angle:.3f} rad is outside limits "
                           f"[{self.min_angle:.3f}, {self.max_angle:.3f}] rad")
        
        # Convert joint angle to motor angle
        motor_angle = joint_angle * self.transmission_ratio * self.direction
        
        # Convert to encoder counts relative to zero position
        counts = motor_angle * self.counts_per_radian
        
        # Convert to absolute encoder count
        motor_position = int(self.zero_position + counts)
        
        # Clamp to motor limits
        min_position = self.motor_model_config.get('min_position', 0)
        max_position = self.motor_model_config.get('max_position', 4095)
        motor_position = max(motor_position, min_position)
        motor_position = min(motor_position, max_position)
        
        return motor_position
    
    def motor_angle_to_joint_angle(self, motor_angle: float) -> float:
        """Convert motor shaft angle (rad) to joint angle (rad)."""
        return motor_angle / (self.transmission_ratio * self.direction)

    def motor_angular_velocity_to_joint_velocity(self, motor_angular_velocity: float) -> float:
        """Convert motor shaft angular velocity (rad/s) to joint velocity (rad/s)."""
        return motor_angular_velocity / (self.transmission_ratio * self.direction)

    def joint_to_motor_angle(self, joint_angle: float) -> float:
        """Convert joint angle (rad) to motor shaft angle (rad)."""
        return joint_angle * self.transmission_ratio * self.direction

    def motor_pos_raw_to_motor_angle(self, motor_position: int) -> float:
        """
        Convert motor position (encoder count) to motor shaft angle (radians),
        relative to this joint's configured zero_position.
        """
        relative_count = motor_position - self.zero_position
        return float(relative_count) * self.radians_per_count

    def motor_vel_raw_to_motor_angular_vel(self, motor_velocity: int) -> float:
        """
        Convert Dynamixel Present Velocity raw units to motor shaft angular velocity (rad/s).
        """
        motor_rpm = float(motor_velocity) * self.velocity_unit_rpm
        return motor_rpm * (2.0 * math.pi / 60.0)

    def motor_current_raw_to_mA(self, motor_current: int) -> float:
        """
        Convert Dynamixel Present Current raw units to milliamps (mA).
        """
        return float(motor_current) * self.current_unit_mA

    def mA_to_motor_current_raw(self, current_mA: float) -> int:
        """
        Convert milliamps (mA) to Dynamixel current raw units for Goal Current / Current Limit registers.
        """
        if self.current_unit_mA == 0:
            raise ValueError("current_unit_mA is 0; check motor model configuration")
        return int(round(float(current_mA) / self.current_unit_mA))
    
    def clamp_joint_angle(self, joint_angle: float) -> float:
        """
        Clamp joint angle to limits.
        
        Args:
            joint_angle: Joint angle in radians
            
        Returns:
            Clamped joint angle in radians
        """
        return max(self.min_angle, min(joint_angle, self.max_angle))
    
    def get_joint_limits(self) -> Tuple[float, float]:
        """Get joint limits in radians."""
        return self.min_angle, self.max_angle
    
    def get_joint_limits_degrees(self) -> Tuple[float, float]:
        """Get joint limits in degrees."""
        return math.degrees(self.min_angle), math.degrees(self.max_angle)
    
    def get_motor_limits(self) -> Tuple[int, int]:
        """Get motor limits in encoder counts."""
        return self.min_encoder_count, self.max_encoder_count
    
    def is_joint_in_limits(self, joint_angle: float) -> bool:
        """Check if joint angle is within limits."""
        return self.min_angle <= joint_angle <= self.max_angle

