"""
Source modules for hand interfaces.

This package contains the core functionality for hand control:
- HandInterface: Hardware abstraction for hand control
- JointConverter: Joint abstraction utilities
- CurrentControl: Current control algorithms
"""

from .hand_interface import BaseHandInterface, RealHandInterface, SimulatedHandInterface, create_hand_interface
from .joint_converter import JointConverter

__all__ = ["BaseHandInterface", "RealHandInterface", "SimulatedHandInterface", "create_hand_interface", "JointConverter"]
