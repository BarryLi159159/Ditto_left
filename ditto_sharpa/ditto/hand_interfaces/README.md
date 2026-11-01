# Teleop Control System

A flexible, configuration-driven teleop control system for Dynamixel motors with leader-follower force rendering. This package provides real-time motor control with support for multiple control modes and joint abstraction.

## Features

- **Multiple Control Modes**: Torque-off, current control, and position control
- **Joint Abstraction**: Work in radians with configurable limits and zero positions
- **Motor-Specific Settings**: Customize PID gains, velocity limits per motor ID
- **Joint-Specific Settings**: Custom joint limits, gear ratios, motor models per motor ID
- **Real-time Control**: 50Hz control loop with minimal latency
- **Hydra Configuration**: Flexible YAML-based configuration management
- **Clean Architecture**: Modular design with separated concerns
- **Hardware-Free Testing**: Fake U2D2 interface for testing without physical hardware

## Quick Start

### Basic Usage

```bash
# Run with current leader configuration (default)
PYTHONPATH=. python teleop/run_teleop.py

# Run with torque-off leader configuration
PYTHONPATH=. python teleop/run_teleop.py hand_config=2dof/2dof_parallel_xc330_to_xm430_torque_off

# Run with custom motor-specific settings
PYTHONPATH=. python teleop/run_teleop.py hand_config=2dof/2dof_parallel_xc330_to_xm430

# Run without hardware (testing mode)
PYTHONPATH=. python teleop/run_teleop.py u2d2.fake_u2d2=true
```

### Programmatic Usage

```python
from teleop import TeleopHand
import hydra
from omegaconf import DictConfig

# Load configuration
with hydra.initialize_config_dir(config_dir="teleop/conf", version_base=None):
    cfg = hydra.compose(config_name="config", overrides=["hand_config=2dof/2dof_parallel_xc330_to_xm430"])

# Create and run teleop hand
teleop = TeleopHand(cfg)
teleop.run()
```

## Architecture

### Core Components

- **`TeleopHand`**: Main control class that manages motor setup and control loops
- **`JointConverter`**: Converts between joint angles (radians) and motor positions (encoder counts)
- **`create_joint_converter()`**: Factory function to create joint converters from Hydra configs

### Control Modes

#### Leader Motors
- **Torque Off**: Motors are free-moving (no control)
- **Current Control**: Motors respond to current commands (0 current for free movement)

#### Follower Motors
- **Position Control**: Motors move to commanded positions
- **Current Control**: Motors respond to current commands

### Joint Abstraction

The system works in joint space (radians) with the following features:

- **Default Limits**: ±90° (configurable per motor)
- **Zero Position**: 2048 encoder counts (center position)
- **Motor Models**: Support for XM430, XC330, and custom models
- **Gear Ratios**: Configurable gear ratios between motor and joint
- **Direction**: Support for reversed motor directions
- **Per-Motor Customization**: Different settings for each motor ID

## Configuration

The system uses Hydra for configuration management with a hierarchical structure:

```
teleop/conf/
├── config.yaml                           # Main configuration
├── hand_config/                          # Hand configurations
│   ├── 1dof/
│   ├── 2dof/
│   ├── 3dof/
│   ├── 7dof/
│   └── misc/
├── motor_models/default.yaml             # Motor specifications
├── joint_configs/default.yaml            # Joint defaults
├── u2d2/default.yaml            # U2D2 interface settings
└── hardware/default.yaml                 # Hardware settings
```

### Available Configurations

- `2dof/2dof_parallel_xc330_to_xm430`: Current control leaders, position control followers
- `2dof/2dof_parallel_xc330_to_xm430_torque_off`: Torque-off leaders, position control followers
- `7dof/new_7dof_series_leader_current_follower_position`: Full 7dof leader/follower configuration

See `conf/README.md` for detailed configuration options and examples.

## Testing Without Hardware

The system includes a fake U2D2 interface for testing without physical hardware. This is perfect for development, debugging, and testing control algorithms.

### Using Fake U2D2 Interface

```bash
# Run with fake U2D2 interface (no hardware required)
PYTHONPATH=. python teleop/run_teleop.py u2d2.fake_u2d2=true

# Run with fake interface and verbose output
PYTHONPATH=. python teleop/run_teleop.py u2d2.fake_u2d2=true verbose=true

# Run with specific hand configuration and fake interface
PYTHONPATH=. python teleop/run_teleop.py hand_config=1dof/1dof_11_111_xm430 u2d2.fake_u2d2=true
```

### Fake Interface Features

- **Realistic Simulation**: Simulates motor behavior with position, velocity, and current responses
- **No Hardware Required**: Perfect for testing and development
- **Full Compatibility**: Works with all existing teleop configurations
- **Verbose Logging**: Shows all fake interface operations when verbose is enabled
- **Testing Mode Indicator**: Clear indication when using fake interfaces

### Configuration

The fake U2D2 interface is controlled by the `fake_u2d2` setting in the U2D2 configuration:

```yaml
# In u2d2/default.yaml
fake_u2d2: false  # Set to true to use fake interface
```

## API Reference

### TeleopHand Class

```python
class TeleopHand:
    def __init__(self, config: DictConfig)
    def connect(self) -> None
    def disconnect(self) -> None
    def setup_motors(self) -> None
    def run(self) -> None
    def start_control_loop(self) -> None
    def read_leader_joint_angles(self) -> List[float]
    def send_joint_angle_targets(self, joint_angles: List[float]) -> None
    def read_all_states(self) -> None
```

### JointConverter Class

```python
class JointConverter:
    def __init__(self, motor_id: int, motor_model_config: Dict, joint_config: Dict)
    def joint_to_motor(self, joint_angle_rad: float) -> int
    def motor_to_joint(self, motor_position: int) -> float
    def clamp_joint_angle(self, joint_angle_rad: float) -> float
    def get_joint_limits(self) -> Tuple[float, float]
    def get_motor_limits(self) -> Tuple[int, int]
    def is_joint_in_limits(self, joint_angle: float) -> bool
```

### Factory Function

```python
def create_joint_converter(motor_id: int, motor_model: str, 
                          motor_models_config: Dict, joint_configs: Dict) -> JointConverter
```

## Examples

### Custom Motor Configuration

```python
# Create custom joint converter using Hydra configs
from teleop import create_joint_converter
import hydra
from pathlib import Path

# Load configuration
config_dir = str(Path.cwd() / 'teleop' / 'conf')
with hydra.initialize_config_dir(config_dir=config_dir, version_base=None):
    cfg = hydra.compose(config_name='config')

# Create joint converter
converter = create_joint_converter(
    motor_id=11, 
    motor_model='XM430', 
    motor_models_config=cfg.motor_models,
    joint_configs=cfg.joint_configs
)

# Convert joint angle to motor position
joint_angle = 0.5  # radians
motor_position = converter.joint_to_motor(joint_angle)

# Convert motor position back to joint angle
joint_angle_back = converter.motor_to_joint(motor_position)
```

### Motor-Specific Settings

```yaml
# In teleop configuration
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

```yaml
# In teleop configuration
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

### Motor Remapping

```yaml
# In teleop configuration
remap_chains:
  0: 1  # Leader[0] controls Follower[1]
  1: 0  # Leader[1] controls Follower[0]
```

### Command-Line Usage

```bash
# Basic usage with different configurations
PYTHONPATH=. python teleop/run_teleop.py hand_config=2dof/2dof_parallel_xc330_to_xm430_torque_off
PYTHONPATH=. python teleop/run_teleop.py hand_config=2dof/2dof_parallel_xc330_to_xm430

# Override motor IDs
PYTHONPATH=. python teleop/run_teleop.py hand_config.leader.motor_ids=[21,22] hand_config.follower.motor_ids=[121,122]

# Override PID gains
PYTHONPATH=. python teleop/run_teleop.py hand_config.follower.settings.p_gain=800

# Override joint settings
PYTHONPATH=. python teleop/run_teleop.py hand_config.joints.follower.0.min_angle_deg=-45

# Override hardware settings
PYTHONPATH=. python teleop/run_teleop.py general.usb_port=/dev/ttyUSB1 general.control_frequency=100

# Use fake U2D2 interface for testing
PYTHONPATH=. python teleop/run_teleop.py u2d2.fake_u2d2=true
```

## Troubleshooting

### Common Issues

1. **Import Errors**: Make sure `PYTHONPATH=.` is set when running scripts
2. **Motor Connection**: Check USB port and motor IDs in configuration
3. **Control Loop Errors**: Verify motor setup and communication
4. **Joint Conversion**: Check joint limits and zero positions
5. **Fake Interface**: Use `u2d2.fake_u2d2=true` for testing without hardware

### Debug Mode

Enable verbose output by setting `verbose: true` in the configuration:

```yaml
general:
  verbose: true
```

## Development

### Adding New Control Modes

1. Add mode to `TeleopHand._setup_motor()` method
2. Update configuration validation in `_validate_config()`
3. Add corresponding configuration files

### Adding New Motor Models

1. Add model to `teleop/conf/motor_models/default.yaml`
2. Define specifications and default settings for the model
3. Add corresponding joint configuration in `teleop/conf/joint_configs/default.yaml`

### Adding New Teleop Configurations

1. Create new YAML file in `teleop/conf/hand_config/`
2. Define leader and follower motor chains and settings
3. Add motor-specific and joint-specific settings as needed
4. Update `teleop/conf/config.yaml` defaults if desired

## License

This project is part of the Ditto system. See the main README for license information.
