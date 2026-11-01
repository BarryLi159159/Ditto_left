#!/usr/bin/env python3
"""
Multi-Joint Converter for Teleop Control

This module provides a joint converter that naturally handles multiple joints
with vectorized operations.
"""

import math
import numpy as np
from typing import Dict, List, Optional, Tuple


class JointConverter:
    """
    Joint converter between joint space (radians) and motor space (encoder counts).
    
    This class stores conversion parameters for multiple joints and provides
    vectorized operations.
    """
    
    def __init__(self, motor_ids: List[int], motor_models_config: Dict, joint_configs: Dict):
        """
        Initialize joint converter for multiple joints.
        
        Args:
            motor_ids: List of motor IDs
            motor_models_config: Motor models configuration from Hydra
            joint_configs: Joint configurations from Hydra
        """
        self.motor_ids = motor_ids
        self.num_joints = len(motor_ids)
        
        # Initialize arrays for all joints
        self._initialize_conversion_parameters(motor_models_config, joint_configs)
        
        # Validate configuration
        self._validate_config()
    
    def _initialize_conversion_parameters(self, motor_models_config: Dict, joint_configs: Dict):
        """Initialize conversion parameters for all joints."""
        # Initialize arrays
        self.zero_positions = np.zeros(self.num_joints, dtype=np.int32)
        self.min_angles = np.zeros(self.num_joints, dtype=np.float64)
        self.max_angles = np.zeros(self.num_joints, dtype=np.float64)
        self.transmission_ratios = np.ones(self.num_joints, dtype=np.float64)
        self.directions = np.ones(self.num_joints, dtype=np.int32)
        self.radians_per_count = np.zeros(self.num_joints, dtype=np.float64)
        self.counts_per_radian = np.zeros(self.num_joints, dtype=np.float64)
        
        # Get default motor model
        default_motor_model = joint_configs.get('default', {}).get('motor_model', 'XM430')
        
        # Fill parameters for each joint
        for i, motor_id in enumerate(self.motor_ids):
            # Get motor model for this joint
            motor_model = self._get_motor_model_for_joint(motor_id, joint_configs)
            
            # Get motor model configuration
            if motor_model not in motor_models_config:
                raise ValueError(f"Unknown motor model: {motor_model}")
            motor_model_config = motor_models_config[motor_model]
            
            # Get joint configuration for this joint
            joint_config = self._get_joint_config_for_motor(motor_id, joint_configs)
            
            # Extract joint configuration values
            self.zero_positions[i] = joint_config.get('zero_position', 2048)
            self.min_angles[i] = math.radians(joint_config.get('min_angle_deg', -90))
            self.max_angles[i] = math.radians(joint_config.get('max_angle_deg', 90))
            self.transmission_ratios[i] = joint_config.get('transmission_ratio', 1.0)
            self.directions[i] = joint_config.get('direction', 1)
            
            # Calculate conversion factors
            resolution = motor_model_config.get('resolution', 4096)
            self.radians_per_count[i] = (2 * math.pi) / resolution
            self.counts_per_radian[i] = resolution / (2 * math.pi)
    
    def _get_motor_model_for_joint(self, motor_id: int, joint_configs: Dict) -> str:
        """Get motor model for a specific joint."""
       
        # Check for motor-specific settings
        for key, config in joint_configs.items():
            if key != 'default' and isinstance(config, dict):
                if 'motor_model' in config:
                    return config['motor_model']
        
        # Use default
        return joint_configs.get('default', {}).get('motor_model', 'XM430')
    
    def _get_joint_config_for_motor(self, motor_id: int, joint_configs: Dict) -> Dict:
        """Get joint configuration for a specific motor."""

        # Check for motor-specific settings
        for key, config in joint_configs.items():
            if key != 'default' and isinstance(config, dict):
                if 'motor_model' in config:
                    return config
        
        # Use default
        return joint_configs.get('default', {})
    
    def _validate_config(self):
        """Validate joint configuration."""
        for i in range(self.num_joints):
            if self.min_angles[i] >= self.max_angles[i]:
                raise ValueError(f"Joint {i}: min_angle >= max_angle")
            if self.transmission_ratios[i] <= 0:
                raise ValueError(f"Joint {i}: transmission_ratio must be positive")
            if self.directions[i] not in [-1, 1]:
                raise ValueError(f"Joint {i}: direction must be -1 or 1")
    
    def joint_to_motor(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Convert joint angles to motor positions.
        
        Args:
            joint_angles: Array of joint angles in radians
            
        Returns:
            Array of motor positions
        """
        if len(joint_angles) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint angles, got {len(joint_angles)}")
        
        # Clamp to joint limits
        joint_angles = np.clip(joint_angles, self.min_angles, self.max_angles)
        
        # Convert joint angle to motor angle
        motor_angles = joint_angles * self.transmission_ratios * self.directions
        
        # Convert to encoder counts relative to zero position
        counts = motor_angles * self.counts_per_radian
        
        # Convert to absolute encoder counts
        motor_positions = self.zero_positions + counts
        
        return motor_positions.astype(np.int32)
    
    def motor_to_joint(self, motor_positions: np.ndarray) -> np.ndarray:
        """
        Convert motor positions to joint angles.
        
        Args:
            motor_positions: Array of motor positions (encoder counts)
            
        Returns:
            Array of joint angles in radians
        """
        if len(motor_positions) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} motor positions, got {len(motor_positions)}")
        
        # Convert to relative encoder counts
        relative_counts = motor_positions - self.zero_positions
        
        # Convert to motor angles
        motor_angles = relative_counts * self.radians_per_count
        
        # Convert to joint angles
        joint_angles = motor_angles / (self.transmission_ratios * self.directions)
        
        return joint_angles
    
    def motor_to_joint_velocity(self, motor_velocities: np.ndarray) -> np.ndarray:
        """
        Convert motor velocities to joint velocities.
        
        Args:
            motor_velocities: Array of motor velocities (encoder counts per second)
            
        Returns:
            Array of joint velocities in rad/s
        """
        if len(motor_velocities) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} motor velocities, got {len(motor_velocities)}")
        
        # Convert to motor angular velocities
        motor_angular_velocities = motor_velocities * self.radians_per_count
        
        # Convert to joint velocities
        joint_velocities = motor_angular_velocities / (self.transmission_ratios * self.directions)
        
        return joint_velocities
    
    def clamp_joint_angles(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Clamp joint angles to limits.
        
        Args:
            joint_angles: Array of joint angles
            
        Returns:
            Array of clamped joint angles
        """
        return np.clip(joint_angles, self.min_angles, self.max_angles)
    
    def get_joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get joint limits in radians."""
        return self.min_angles.copy(), self.max_angles.copy()
    
    def get_joint_limits_degrees(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get joint limits in degrees."""
        return np.degrees(self.min_angles), np.degrees(self.max_angles)

