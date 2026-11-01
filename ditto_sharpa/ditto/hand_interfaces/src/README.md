# Teleop Source Code

This directory contains the core source code for the teleop control system.

## Files

### `hand_interface.py`
Provides hardware abstraction for hand control with support for different control modes and motor configurations.
- `BaseHandInterface`: Abstract base class for hand interfaces.
- `RealHandInterface`: Interface for controlling real hardware using U2D2.
- `SimulatedHandInterface`: Interface for simulated hardware for testing.
- `create_hand_interface()`: Factory function to create hand interfaces.

### `current_control.py`
Implements current control laws for teleoperation.
- `CurrentControlLaws`: Collection of control laws (friction compensation, force rendering, etc.).
- `CurrentController`: Main controller that combines multiple control laws.
- `ControlParams`: Parameters for current control (friction coefficients, gains, etc.).

### `joint_converter.py`
Joint conversion utilities for working in radians:
- `JointConverter`: Converts between joint angles (radians) and motor positions (encoder counts).
- Handles joint limits, transmission ratios, and unit conversions.

### `motor_data.py`
Defines data structures for motor state.
- `MotorData`: Dataclass containing raw and converted motor readings (position, velocity, current).

## Key Classes

### Hand Interfaces
The `hand_interface.py` module provides the main interface for interacting with the hand hardware.

```python
from hand_interface import create_hand_interface

# Create interface
interface = create_hand_interface(config, chain_type="leader", interface_type="real")

# Connect and setup
interface.connect()
interface.setup_motors()

# Read states and send commands
states = interface.read_states()
interface.send_position_commands(target_positions)
```

### Current Control
The `current_control.py` module handles advanced motor control features like friction compensation and force feedback.

```python
from current_control import CurrentController, ControlParams

# Create controller
controller = CurrentController(control_frequency=50.0)

# Compute command based on state and params
cmd = controller.compute_current_command(
    motor_data=current_state,
    params=control_params,
    follower_data=follower_state
)
```

### JointConverter
Handles conversion between joint space (radians) and motor space (encoder counts).

```python
from joint_converter import JointConverter

# Convert joint angle to motor position
motor_pos = converter.joint_to_motor(0.5)  # 0.5 radians

# Convert motor position to joint angle
joint_angle = converter.motor_to_joint(2048)  # encoder count
```

## Motor Models

Supported motor models are defined in configuration files and handled by `JointConverter`:

- **XM430**: Standard Dynamixel XM430 motor
- **XC330**: Dynamixel XC330 motor with different default settings

## Joint Configuration

Joint configurations handle:

- **Default limits**: ±90° (configurable)
- **Zero position**: 2048 encoder counts (center)
- **Gear ratio**: 1.0 (configurable)
- **Direction**: 1 (configurable)