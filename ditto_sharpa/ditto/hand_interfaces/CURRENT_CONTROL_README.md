# Current Control for Teleop Hand

This document describes the current control functionality added to the teleop hand system, which enables friction compensation and force rendering for leader motors.

## Overview

The current control system provides a flexible framework for commanding non-zero currents to leader motors in current control mode. This enables:

1. **Friction Compensation**: Compensate for coulomb, damping, and static friction
2. **Force Rendering**: Provide force feedback from follower to leader motors
3. **Free Motion**: Zero current for completely free movement

## Architecture

### Core Components

- **`current_control.py`**: Contains all control law implementations
- **`CurrentController`**: Main controller that combines multiple control laws
- **`ControlParams`**: Configuration parameters for each motor
- **`MotorData`**: Motor state information structure

### Control Laws

1. **Zero Current** (`zero_current`): Returns 0 mA command
2. **Coulomb Friction** (`coulomb_friction_compensation`): Compensates for coulomb friction
3. **Damping** (`damping_compensation`): Compensates for viscous damping
4. **Static Friction** (`static_friction_compensation`): Dithering for static friction
5. **Force Rendering** (`force_rendering`): Force feedback from follower motors

## Configuration

### Basic Structure

```yaml
leader:
  mode: "current"
  motor_ids: [11, 12]
  settings:
    current_limit: 2300
  current_control:
    "11":  # Motor ID
      enable_coulomb_comp: true
      enable_damping_comp: true
      enable_static_comp: true
      enable_force_rendering: false
      mu_coulomb_cw: 4.0
      mu_coulomb_ccw: 5.0
      mu_damping: 0.03
      mu_static_amplitude: 15.0
      static_frequency: 10.0
      static_threshold: 1.0
      force_rendering_gain: 0.0
      force_rendering_alpha: 0.95
```

### Parameters

| Parameter | Description | Units | Default |
|-----------|-------------|-------|---------|
| `enable_coulomb_comp` | Enable coulomb friction compensation | bool | false |
| `enable_damping_comp` | Enable damping compensation | bool | false |
| `enable_static_comp` | Enable static friction compensation | bool | false |
| `enable_force_rendering` | Enable force rendering | bool | false |
| `mu_coulomb_cw` | Clockwise coulomb friction coefficient | mA | 0.0 |
| `mu_coulomb_ccw` | Counterclockwise coulomb friction coefficient | mA | 0.0 |
| `mu_damping` | Damping coefficient | mA·s/rad | 0.0 |
| `mu_static_amplitude` | Static friction dithering amplitude coefficient | mA | 0.0 |
| `static_frequency` | Static friction dithering frequency | Hz | 10.0 |
| `static_threshold` | Velocity threshold for static friction | rad/s | 1.0 |
| `force_rendering_gain` | Force feedback gain | - | 0.0 |
| `force_rendering_alpha` | Low-pass filter coefficient | - | 0.95 |
| `force_rendering_threshold` | Minimum follower current for force rendering | mA | 50.0 |

## Usage Examples

### 1. Friction Compensation Only

```bash
python run_teleop.py hand_config=3dof/3dof_series_leader_current_follower_position
```

This configuration enables:
- Coulomb friction compensation
- Damping compensation  
- Static friction dithering
- No force rendering

### 2. Force Rendering + Friction Compensation

```bash
python run_teleop.py hand_config=7dof/new_7dof_series_leader_current_follower_position
```

This configuration adds:
- Force feedback from follower motors
- Low-pass filtering of force feedback
- All friction compensation features

### 3. Zero Current (Free Motion)

```bash
python run_teleop.py hand_config=3dof/3dof_series_torque_off
```

This configuration provides:
- Completely free motion
- No current commands to leader motors
- Useful for testing and calibration

### 4. Debug Mode with Current Breakdown

```bash
python run_teleop.py hand_config=3dof/3dof_series_leader_current_follower_position verbose=true show_current_breakdown=true
```

This configuration provides:
- All friction compensation features
- Detailed current command breakdown output
- Useful for debugging and parameter tuning

## Control Law Details

### Coulomb Friction Compensation

```python
if velocity > 0:
    tau_c = mu_coulomb_cw * sign(velocity)
else:
    tau_c = mu_coulomb_ccw * sign(velocity)
```

- Different coefficients for clockwise/counterclockwise directions
- Based on reference implementation from `current_teleop_alpha_tau.py`

### Damping Compensation

```python
tau_d = -mu_damping * velocity
```

- Proportional to joint velocity
- Helps reduce viscous friction effects

### Static Friction Compensation

```python
if velocity >= 0 and velocity < static_threshold_cw:
    tau_ss = mu_static_amplitude * cos(π * frequency * time)
elif velocity <= 0 and velocity > -static_threshold_ccw:
    tau_ss = mu_static_amplitude * cos(π * frequency * time)
else:
    tau_ss = 0
```

- Only active when velocity is nearly static (below thresholds, including zero velocity)
- Uses dithering to break static friction when the motor is stuck
- Asymmetric thresholds for clockwise/counterclockwise directions
- Frequency is configurable via `static_frequency` parameter
- Default thresholds: 1.0 rad/s (57 deg/s) - reasonable for normal joint movements
- **Key behavior**: Active when static/stuck, inactive when moving freely

### Force Rendering

```python
if abs(follower_current) > force_rendering_threshold:
    raw_command = -gain * follower_current
    filtered_command = alpha * raw_command + (1 - alpha) * prev_command
else:
    filtered_command = 0.0
```

- Low-pass filters force feedback
- Prevents high-frequency oscillations
- Only applies force when follower current exceeds threshold (avoids noise)
- Based on reference implementation

## Tuning Guidelines

### 1. Start with Friction Compensation

1. Begin with low coulomb coefficients (1-2 mA)
2. Gradually increase until smooth motion is achieved
3. Add damping if needed (0.01-0.05 mA·s/rad)
4. Use static friction dithering for very low velocities

### 2. Add Force Rendering

1. Start with very low gains (0.01-0.05)
2. Gradually increase while monitoring stability
3. Adjust alpha for filtering (0.9-0.99)
4. Monitor motor temperatures

### 3. Safety Considerations

- Always start with low current limits
- Monitor motor temperatures continuously
- Use emergency stop if behavior is unexpected
- Test with low gains first

## API Reference

### CurrentController

```python
controller = CurrentController(control_frequency=50.0)

# Single motor
command = controller.compute_current_command(motor_data, params, time, follower_data)

# Multiple motors
commands = controller.compute_bulk_current_commands(motor_data_list, params_list, time, follower_data_list)
```

### ControlParams

```python
params = ControlParams(
    motor_id=11,
    coulomb_pos=4.0,
    coulomb_neg=5.0,
    damping=0.03,
    enable_coulomb_comp=True,
    enable_damping_comp=True
)
```

### MotorData

```python
motor_data = MotorData(
    motor_id=11,
    position=2048,        # Raw encoder count
    velocity=100,         # Raw velocity (counts/s)
    current=50,           # Raw current (mA)
    joint_angle=0.1,      # Joint angle (rad)
    joint_velocity=0.5    # Joint velocity (rad/s)
)
```

## Integration with TeleopHand

The current control system is automatically integrated into the `TeleopHand` class:

1. Control parameters are loaded from configuration
2. Motor data is read each control cycle
3. Current commands are computed and sent
4. Force feedback is read from follower motors

No changes are needed to existing teleop scripts - just use the new configuration files.

## Debugging and Monitoring

### Current Command Breakdown

The system can display detailed breakdown of current commands showing individual contributions from each control law:

```
Motor 11 computed torques:  0.000 (cmd) +  4.000 (coulomb) + -0.060 (damping) + 15.000 (static) + -7.600 (force)
Commanded torque: 11.340
```

To enable this output, set `show_current_breakdown: true` in the general configuration:

```yaml
general:
  control_frequency: 50
  verbose: true
  show_current_breakdown: true
```

This is useful for:
- Debugging control law behavior
- Tuning parameters
- Understanding which control laws are contributing most
- Verifying that control laws are working as expected

## Troubleshooting

### Common Issues

1. **Motors not moving smoothly**: Increase coulomb friction coefficients
2. **Oscillations**: Reduce force rendering gain or increase alpha
3. **High motor temperatures**: Reduce all current parameters
4. **Unstable behavior**: Check current limits and gains

### Debug Output

Enable verbose mode to see:
- Control parameters for each motor
- Current commands being sent
- Joint angle clamping warnings
- Control loop timing

## Future Enhancements

- Adaptive friction compensation
- More sophisticated force rendering algorithms
- Real-time parameter tuning interface
- Motor temperature monitoring and protection
- Advanced control laws (PID, impedance control)
