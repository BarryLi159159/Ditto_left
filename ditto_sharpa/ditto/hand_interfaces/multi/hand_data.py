#!/usr/bin/env python3
"""
HandData - Simple data structure for hand state information.

This module provides a simple data structure that stores all motor data
as numpy arrays, enabling vectorized operations.
"""

import numpy as np
from typing import List, Optional
from dataclasses import dataclass


@dataclass
class HandData:
    """
    Simple data structure for hand state information.
    
    All data is stored as numpy arrays for efficient vectorized operations.
    """
    # Motor IDs (list of ints)
    motor_ids: List[int]
    
    # Raw motor data (numpy arrays)
    positions: np.ndarray      # Raw motor positions
    velocities: np.ndarray     # Raw motor velocities  
    currents: np.ndarray       # Raw motor currents
    
    # Joint space data (numpy arrays)
    joint_angles: np.ndarray      # Joint angles in radians
    joint_velocities: np.ndarray  # Joint velocities in rad/s
    
    def __init__(self, motor_ids: List[int], 
                 positions: np.ndarray,
                 velocities: np.ndarray, 
                 currents: np.ndarray,
                 joint_angles: np.ndarray,
                 joint_velocities: np.ndarray):
        """
        Initialize HandData with arrays.
        
        Args:
            motor_ids: List of motor IDs
            positions: Raw motor positions (numpy array)
            velocities: Raw motor velocities (numpy array)
            currents: Raw motor currents (numpy array)
            joint_angles: Joint angles in radians (numpy array)
            joint_velocities: Joint velocities in rad/s (numpy array)
        
        Note: Arrays are assumed to be already numpy arrays with correct dtypes.
        No conversion or validation is performed for performance.
        """
        self.motor_ids = motor_ids
        self.positions = positions
        self.velocities = velocities
        self.currents = currents
        self.joint_angles = joint_angles
        self.joint_velocities = joint_velocities
    
    def __len__(self) -> int:
        """Return number of motors."""
        return len(self.motor_ids)
    
    def __repr__(self) -> str:
        """String representation."""
        return (f"HandData(motors={len(self.motor_ids)}, "
                f"joint_angles={self.joint_angles}, "
                f"joint_velocities={self.joint_velocities})")
