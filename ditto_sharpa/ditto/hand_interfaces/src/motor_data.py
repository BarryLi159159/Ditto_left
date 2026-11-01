from dataclasses import dataclass

@dataclass
class MotorData:
    """Data structure for motor state information."""
    motor_id: int
    # Raw motor readings (as returned by Dynamixel registers)
    raw_position: int
    raw_velocity: int
    raw_current: int

    # Converted motor readings (standard units)
    position: float         # Motor shaft angle (rad), relative to configured zero_position
    velocity: float         # Motor shaft angular velocity (rad/s)
    current: float          # Motor current (mA)

    # Joint-space readings (standard units)
    joint_angle: float      # Joint angle (rad)
    joint_velocity: float   # Joint velocity (rad/s)