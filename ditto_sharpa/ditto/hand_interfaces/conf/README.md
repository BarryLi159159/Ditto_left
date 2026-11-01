# Teleop Configuration

This directory contains Hydra configuration files for the teleop control system. The configuration system allows flexible setup of motor chains, control modes, and hardware settings.

## Configuration Structure

```
conf/
├── config.yaml                    # Main configuration file
├── hand_config/                   # Hand configuration files
│   ├── 1dof/
│   ├── 2dof/
│   ├── 3dof/
│   ├── 7dof/
│   └── misc/
├── motor_models/                  # Motor model configurations
│   └── default.yaml
├── joint_configs/                 # Joint configuration defaults
│   └── default.yaml
└── u2d2/                 # U2D2 interface configurations
    └── default.yaml
```

## Main Configuration (`config.yaml`)

The main configuration file defines the default composition and general settings:

```yaml
# @package _global_

defaults:
  - hand_config: 2dof/2dof_parallel_xc330_to_xm430
  - u2d2: default
  - motor_models: default
  - joint_configs: default
  - _self_

# Configuration metadata
config_name: "teleop_control"
description: "Teleop control configuration"

# General settings
general:
  control_frequency: 50
  verbose: true
```

### General Settings

| Setting | Default | Description |
|---------|---------|-------------|
| `control_frequency` | `50` | Control loop frequency (Hz) |
| `verbose` | `true` | Enable verbose output |

## Hand Configurations (`hand_config/`)

Teleop configurations define the control modes and settings for leader and follower motors, including motor-specific and joint-specific customizations.

### Configuration Structure

```yaml
# @package hand_config

leader:
  mode: "current"                    # Control mode: "torque_off" or "current"
  motor_ids: [11, 12]               # List of motor IDs
  settings:                         # Mode-specific settings
    current_limit: 0                # Current limit (for current mode)
    current_command: 0              # Default current command

follower:
  mode: "position"                  # Control mode: "position" or "current"
  motor_ids: [111, 112]            # List of motor IDs
  settings:                        # Mode-specific settings
    velocity_limit: 150            # Velocity limit (for position mode)
    p_gain: 500                    # P gain (for position mode)
    i_gain: 0                      # I gain (for position mode)
    d_gain: 0                      # D gain (for position mode)

# Motor-specific settings (overrides chain defaults)
motor_specific_settings:
  follower:
    111:  # Custom settings for motor 111
      p_gain: 800
      i_gain: 50
      d_gain: 20
      velocity_limit: 200
    112:  # Custom settings for motor 112
      p_gain: 300
      velocity_limit: 100

# Joint-specific settings (overrides joint config defaults)
joints:
  follower:
    - motor_id: 111
      motor_model: "XM430"
      zero_position: 2048
      min_angle_deg: -45  # Custom limit: -45° to +45°
      max_angle_deg: 45
      transmission_ratio: 2.0  # Custom transmission ratio: 2:1 reduction
      direction: 1
    - motor_id: 112
      motor_model: "XC330"  # Different motor model
      zero_position: 2048
      min_angle_deg: -90
      max_angle_deg: 90
      transmission_ratio: 1.0
      direction: -1       # Reversed direction
```

### Control Modes

#### Leader Motors
- **`torque_off`**: Motors are free-moving (no control)
- **`current`**: Motors respond to current commands (0 current for free movement)

#### Follower Motors
- **`position`**: Motors move to commanded positions
- **`current`**: Motors respond to current commands

### Available Configurations

#### `2dof/2dof_parallel_xc330_to_xm430_torque_off.yaml`
- **Leaders**: Torque off (completely free movement)
- **Followers**: Position control with default PID gains
- **Use Case**: Simple teleop with completely free leaders

#### `2dof/2dof_parallel_xc330_to_xm430.yaml`
- **Leaders**: Current control with 0 current (free movement)
- **Followers**: Position control with default PID gains
- **Use Case**: Standard teleop with free-moving leaders

#### `7dof/new_7dof_series_leader_current_follower_position.yaml`
- **Leaders**: Current control with 0 current
- **Followers**: Position control with custom motor and joint settings
- **Features**: Motor-specific PID gains, custom joint limits, gear ratios
- **Use Case**: Advanced teleop with customized motor behavior

### Motor-Specific Settings

Override chain defaults for individual motors:

```yaml
motor_specific_settings:
  follower:
    111:  # Custom settings for motor 111
      p_gain: 800
      i_gain: 50
      d_gain: 20
      velocity_limit: 200
    112:  # Custom settings for motor 112
      p_gain: 300
      velocity_limit: 100
```

### Joint-Specific Settings

Override joint configuration defaults for individual motors:

```yaml
joints:
  follower:
    - motor_id: 111
      motor_model: "XM430"
      zero_position: 2048
      min_angle_deg: -45  # Custom limit: -45° to +45°
      max_angle_deg: 45
      transmission_ratio: 2.0  # Custom transmission ratio: 2:1 reduction
      direction: 1
    - motor_id: 112
      motor_model: "XC330"  # Different motor model
      zero_position: 2048
      min_angle_deg: -90
      max_angle_deg: 90
      transmission_ratio: 1.0
      direction: -1       # Reversed direction
```


## U2D2 Settings (`u2d2/`)

U2D2 settings define USB port and communication settings for the U2D2 interface.

### `default.yaml`
```yaml
# @package u2d2

usb_port: "/dev/ttyUSB0"
baudrate: 4000000
```

### U2D2 Settings

| Setting | Default | Description |
|---------|---------|-------------|
| `usb_port` | `"/dev/ttyUSB0"` | USB port for U2D2 interface |
| `baudrate` | `4000000` | Communication baudrate |

Note: U2D2 settings only override USB port and baudrate. Other settings like `control_frequency` and `verbose` are defined in the main configuration.

## Motor Model Configurations (`motor_models/default.yaml`)

Motor model configurations define the specifications and default settings for different Dynamixel motor types:

```yaml
# @package motor_models

XM430:
  name: "XM430"
  resolution: 4096  # 12-bit resolution
  max_position: 4095
  min_position: 0
  max_velocity: 330  # 330 RPM
  max_current: 1193  # 1.193A
  default_p_gain: 500
  default_i_gain: 0
  default_d_gain: 0
  default_velocity_limit: 150

XC330:
  name: "XC330"
  resolution: 4096  # 12-bit resolution
  max_position: 4095
  min_position: 0
  max_velocity: 330  # 330 RPM
  max_current: 1193  # 1.193A
  default_p_gain: 400
  default_i_gain: 50
  default_d_gain: 20
  default_velocity_limit: 200
```

### Motor Model Settings

| Setting | Description |
|---------|-------------|
| `name` | Motor model name |
| `resolution` | Encoder counts per revolution |
| `max_position` | Maximum encoder count |
| `min_position` | Minimum encoder count |
| `max_velocity` | Maximum velocity in RPM |
| `max_current` | Maximum current in mA |
| `default_p_gain` | Default P gain for position control |
| `default_i_gain` | Default I gain for position control |
| `default_d_gain` | Default D gain for position control |
| `default_velocity_limit` | Default velocity limit |

## Joint Configuration Defaults (`joint_configs/default.yaml`)

Joint configuration defaults define the standard settings for joint abstraction:

```yaml
# @package joint_configs

default:
  motor_model: "XM430"
  zero_position: 2048  # Default zero position in encoder counts
  min_angle_deg: -90   # Default -90 degrees
  max_angle_deg: 90    # Default +90 degrees
  transmission_ratio: 1.0  # Default transmission ratio
  direction: 1         # Default direction (1 or -1)

XM430:
  motor_model: "XM430"
  zero_position: 2048
  min_angle_deg: -90
  max_angle_deg: 90
  gear_ratio: 1.0
  direction: 1

XC330:
  motor_model: "XC330"
  zero_position: 2048
  min_angle_deg: -90
  max_angle_deg: 90
  gear_ratio: 1.0
  direction: 1
```

### Joint Configuration Settings

| Setting | Default | Description |
|---------|---------|-------------|
| `motor_model` | `"XM430"` | Motor model to use |
| `zero_position` | `2048` | Encoder count for 0 radians |
| `min_angle_deg` | `-90` | Minimum joint angle (degrees) |
| `max_angle_deg` | `90` | Maximum joint angle (degrees) |
| `transmission_ratio` | `1.0` | Transmission ratio between motor and joint |
| `direction` | `1` | Motor direction (1 or -1) |

## Joint Configuration

For advanced joint configuration, you can specify joint-specific settings:

```yaml
joints:
  leader:
    - motor_id: 11
      motor_model: "XM430"
      zero_position: 2048
      min_angle_deg: -90
      max_angle_deg: 90
      transmission_ratio: 1.0
      direction: 1
    - motor_id: 12
      motor_model: "XM430"
      zero_position: 2048
      min_angle_deg: -90
      max_angle_deg: 90
      transmission_ratio: 1.0
      direction: 1
```

### Joint Settings

| Setting | Default | Description |
|---------|---------|-------------|
| `motor_model` | `"XM430"` | Motor model (XM430, XC330, etc.) |
| `zero_position` | `2048` | Encoder count for 0 radians |
| `min_angle_deg` | `-90` | Minimum joint angle (degrees) |
| `max_angle_deg` | `90` | Maximum joint angle (degrees) |
| `transmission_ratio` | `1.0` | Transmission ratio between motor and joint |
| `direction` | `1` | Motor direction (1 or -1) |

## Usage Examples

### Command Line Overrides

```bash
# Override USB port
python teleop/run_teleop.py u2d2.usb_port=/dev/ttyUSB1

# Override control frequency
python teleop/run_teleop.py general.control_frequency=100

# Override motor IDs
python teleop/run_teleop.py hand_config.leader.motor_ids=[21,22] hand_config.follower.motor_ids=[121,122]

# Override PID gains
python teleop/run_teleop.py hand_config.follower.settings.p_gain=800

# Use different hand configuration
python teleop/run_teleop.py hand_config=2dof/2dof_parallel_xc330_to_xm430_torque_off

# Use custom configuration with motor-specific settings
python teleop/run_teleop.py hand_config=2dof/2dof_parallel_xc330_to_xm430
```

### Programmatic Configuration

```python
import hydra
from omegaconf import DictConfig

# Load with overrides
with hydra.initialize_config_dir(config_dir="teleop/conf", version_base=None):
    cfg = hydra.compose(
        config_name="config",
        overrides=[
            "hand_config=2dof/2dof_parallel_xc330_to_xm430",
            "u2d2.usb_port=/dev/ttyUSB1",
            "general.control_frequency=100"
        ]
    )
```

## Creating Custom Configurations

### 1. Create New Teleop Configuration

Create a new file in `hand_config/`:

```yaml
# @package hand_config

leader:
  mode: "current"
  motor_ids: [31, 32, 33]
  settings:
    current_limit: 100
    current_command: 0

follower:
  mode: "position"
  motor_ids: [131, 132, 133]
  settings:
    velocity_limit: 200
    p_gain: 600
    i_gain: 50
    d_gain: 20


# Optional: Motor-specific settings
motor_specific_settings:
  follower:
    131:
      p_gain: 800
      velocity_limit: 300

# Optional: Joint-specific settings
joints:
  follower:
    - motor_id: 131
      min_angle_deg: -60
      max_angle_deg: 60
      transmission_ratio: 1.5
```

### 2. Create New U2D2 Settings

Create a new file in `u2d2/`:

```yaml
# @package u2d2

usb_port: "/dev/ttyUSB2"
baudrate: 2000000
```

### 3. Update Main Configuration

Add your new configuration to the defaults in `config.yaml`:

```yaml
defaults:
  - hand_config: 2dof/your_new_config
  - u2d2: your_new_u2d2
  - motor_models: default
  - joint_configs: default
  - _self_
```

## Troubleshooting

### Common Configuration Issues

1. **Invalid Motor IDs**: Ensure motor IDs exist and are accessible
2. **Invalid Control Modes**: Check that leader/follower modes are supported
3. **Missing Settings**: Ensure all required settings are provided for each mode

### Configuration Validation

The system validates configurations on startup:

- Motor chain lengths must match
- Control modes must be supported
- Required settings must be present
- Motor IDs must be valid

### Debug Configuration

Enable verbose mode to see configuration details:

```yaml
general:
  verbose: true
```

This will show:
- Loaded configuration values
- Motor setup details
- Control loop status
- Error messages
