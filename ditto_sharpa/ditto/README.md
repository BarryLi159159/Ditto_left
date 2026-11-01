# Ditto Software

**📑 Table of contents**

- 📁 [Project Structure](#project-structure)
- 🚀 [Quick Start](#quick-start)
- 📦 [Setup](#setup)
- 🎮 [Usage](#usage)
- 🏗️ [Architecture](#architecture)
- ✨ [Key Features](#key-features)
- ⚙️ [Configuration](#configuration)
- 🛠️ [Development](#development)
- 📚 [Examples](#examples)
- 📂 [Configuration Files](#configuration-files)
- 🔌 [Hardware Requirements](#hardware-requirements)
- 🔧 [Troubleshooting](#troubleshooting)
- 🤝 [Contributing](#contributing)
- ⚖️ [License](#license)
- 🙏 [Acknowledgments](#acknowledgments)

A modular robotics control system for Dynamixel motor control with flexible hand interfaces and controllers.

## Project Structure

```
ditto/
├── hand_interfaces/                # Core hand interface modules
│   ├── conf/                       # Configuration files
│   │   ├── config.yaml             # Main configuration
│   │   ├── hand_config/            # Hand configs (grouped by categories: 2dof/, 7dof/, ...)
│   │   ├── u2d2/                   # U2D2 interface settings
│   │   ├── joint_configs/          # Joint conversion settings
│   │   └── motor_models/           # Motor model definitions
│   ├── src/                        # Source code
│   │   ├── hand_interface.py       # Hand interface abstractions
│   │   ├── joint_converter.py      # Joint conversion utilities
│   │   ├── current_control.py      # Current control algorithms
│   │   └── utils.py                # Utility functions
│   └── README.md
├── controllers/                    # Control applications
│   ├── gui_controller.py           # GUI motor control
│   └── teleop_controller.py        # Teleop control (class + main function)
└── README.md                       # This file
```

## Quick Start

### Teleop Control

Most supported setups live under **`hand_config/2dof/`** (parallel XC330→XM430) and **`hand_config/7dof/`** (full hand, series leader/current + position follower):

```bash
# 7 DoF — fake U2D2 for development
python controllers/teleop_controller.py hand_config=7dof/7dof u2d2=dual u2d2.fake_u2d2=true

# 7 DoF — real hardware
python controllers/teleop_controller.py hand_config=7dof/7dof u2d2=dual

# 2 DoF — fake U2D2
python controllers/teleop_controller.py hand_config=2dof/2dof u2d2=dual u2d2.fake_u2d2=true

# 2 DoF — real hardware
python controllers/teleop_controller.py hand_config=2dof/2dof u2d2=dual
```

With no `hand_config` override, Hydra uses `hand_config=none` (no motors). Pass one of the paths above for real teleop.

### GUI Motor Control

```bash
# For testing without hardware:
python controllers/gui_controller.py hand_config=2dof/2dof_parallel_agent_to_xm430 u2d2=dual u2d2.fake_u2d2=true

# For real hardware:
python controllers/gui_controller.py hand_config=2dof/2dof_parallel_agent_to_xm430 u2d2=dual
```

### Hand Config Tuning GUI

Edit leader force-rendering and follower PID parameters per motor (same fields as `hand_config/*/7dof.yaml`). Keyboard shortcuts: **`u`** saves `ORIGINAL_tuned.yaml` (original unchanged), **`f`** starts/stops teleop with that file, **`q`** quits. Use **`u2d2=dual`** for 7-DoF:

```bash
cd ditto
python controllers/hand_config_tune_gui.py --config hand_interfaces/conf/hand_config/7dof/7dof.yaml --u2d2 dual

# Fake hardware:
python controllers/hand_config_tune_gui.py --config hand_interfaces/conf/hand_config/7dof/7dof.yaml --u2d2 dual --fake-u2d2

# After colcon install:
ros2 run ditto_ros2 hand_config_tune_gui -- --config hand_interfaces/conf/hand_config/7dof/7dof.yaml
```


## Setup

### 1. Clone Repository

```bash
git clone <repository-url>
cd ditto
```

### 2. Create Conda Environment

Create a conda environment from the provided `environment.yaml` file. This installs `dynamixel_u2d2` (and other pip dependencies) automatically:

```bash
# Create environment
conda env create -f environment.yaml

# Activate environment
conda activate ditto
```

## Usage

### GUI Motor Control

The GUI controller provides real-time motor control with sliders:

```bash
# Test with fake hardware
python controllers/gui_controller.py hand_config=misc/gui_leader_position_follower u2d2=dual u2d2.fake_u2d2=true

# Use with real hardware
python controllers/gui_controller.py hand_config=misc/gui_leader_position_follower u2d2=dual
```

**Features:**
- Real-time slider control
- Joint limit awareness
- Live position display
- Easy connect/disconnect

### Teleop Control

The teleop controller provides bilateral teleoperation. Prefer **`7dof/`** for the full hand and **`2dof/`** for a parallel finger pair:

```bash
# 7 DoF (fake U2D2)
python controllers/teleop_controller.py hand_config=7dof/7dof u2d2=dual u2d2.fake_u2d2=true

# 7 DoF (hardware)
python controllers/teleop_controller.py hand_config=7dof/7dof u2d2=dual

# 2 DoF (fake U2D2)
python controllers/teleop_controller.py hand_config=2dof/2dof u2d2=dual u2d2.fake_u2d2=true

# 2 DoF torque-off leader (hardware)
python controllers/teleop_controller.py hand_config=2dof/2dof_parallel_xc330_to_xm430_torque_off u2d2=dual
```

**Features:**
- Leader-follower teleoperation
- Current control for force feedback
- Position control for follower
- Real-time state monitoring


## Architecture

### Hand Interfaces (`hand_interfaces/`)

The core abstraction layer that provides:

- **`BaseHandInterface`**: Abstract base class for hand interfaces
- **`RealHandInterface`**: Physical Dynamixel hand control
- **`SimulatedHandInterface`**: Simulated hand for testing
- **`create_hand_interface()`**: Factory function for creating interfaces

### Controllers (`controllers/`)

Application-specific control implementations:

- **`gui_controller.py`**: Real-time GUI control with sliders
- **`teleop_controller.py`**: Bilateral teleoperation control (class + main function)

## Key Features

- **Modular Design**: Clean separation between hardware interfaces and control logic
- **Flexible Configuration**: Hydra-based configuration system
- **Real-time Control**: High-frequency control loops with threading
- **Joint Awareness**: Automatic joint limit handling
- **Testing Support**: Fake interfaces for development without hardware
- **GUI Control**: Intuitive slider-based motor control

## Configuration

All configuration files are located in `hand_interfaces/conf/`:

- **`hand_config/`**: Hand-specific configurations (`2dof/`, `7dof/`, `3dof/`, `1dof/`, `misc/`, plus top-level `none.yaml` for idle default)
- **`u2d2/`**: U2D2 interface settings
- **`joint_configs/`**: Joint conversion settings
- **`motor_models/`**: Motor model definitions

## Development

The modular architecture makes it easy to:

1. **Add New Controllers**: Create new control applications in `controllers/`
2. **Extend Hand Interfaces**: Add new hardware interfaces in `hand_interfaces/src/`
3. **Customize Configurations**: Add new config files in `hand_interfaces/conf/`

## Examples

Programmatic use of the hand interface (after you have a Hydra `config`; see **Usage** for CLI entry points).

### Using Hand Interfaces

```python
from hand_interfaces.src import create_hand_interface

# Create interface
interface = create_hand_interface(config, "follower", "real", port="/dev/ttyUSB0", use_fake=True)

# Connect and setup
interface.connect()
interface.setup_motors()

# Control
states = interface.read_states()
interface.send_position_commands([0.1, 0.2, 0.3])
```

### Creating a New Controller

```python
from hand_interfaces.src.hand_interface import create_hand_interface

# Create hand interface
hand = create_hand_interface(config, "follower", "real", port="/dev/ttyUSB0")

# Connect and control
hand.connect()
hand.setup_motors()
hand.send_position_commands([0.1, 0.2, 0.3])
```

## Configuration Files

### Hand Configurations

Located in `hand_interfaces/conf/hand_config/` (Hydra overrides use paths like `7dof/<name>` without the `.yaml` suffix):

**7 DoF (primary full-hand configs)**

- **`7dof/7dof.yaml`**: Bilateral teleop — series leader chain, current-mode leader + position follower
- **`7dof/7dof_in_the_wild.yaml`**: Leader-only (field) style layout
- **`7dof/7dof_agent.yaml`**: Agent-mode follower when driving from external commands

**2 DoF**

- **`2dof/2dof.yaml`**: Main 2 DoF bilateral teleop — current-mode XC330 leaders (with force rendering), position-mode XM430 followers
- **`2dof/2dof_agent.yaml`**: Same follower chain as above, leader in **agent** mode for GUI or external commands (`schedule_command`); no physical leader motors
- **`2dof/2dof_in_the_wild.yaml`**: **Leader-only** — torque-off leaders `[11, 21]`, follower `none` (field / encoder logging without follower hardware)

**Other**

- **`misc/gui_leader_position_follower.yaml`**: GUI slider follower layout
- **`none.yaml`**: Idle default — no leader/follower hardware (compose with `hand_config=...` for real runs)

### U2D2 Settings

Located in `hand_interfaces/conf/u2d2/`:

- **`default.yaml`**: Single U2D2 configuration
- **`dual.yaml`**: Dual U2D2 configuration

### Joint Configurations

Located in `hand_interfaces/conf/joint_configs/`:

- **`default.yaml`**: Default joint settings
- **`custom.yaml`**: Custom joint configurations

## Hardware Requirements

### For Real Hardware

- Dynamixel motors (XM430, XM540, etc.)
- U2D2 interface (single or dual)
- USB connection to computer

### For Testing

- No hardware required when using `fake_u2d2=true`
- All functionality can be tested with simulated interfaces

## Troubleshooting

### Common Issues

1. **Port Not Found**: Check USB connections and port permissions
2. **Motor Not Responding**: Verify motor IDs and baudrate settings
3. **Configuration Errors**: Check YAML syntax and file paths

### Debug Mode

Enable verbose output for debugging:

```bash
python controllers/teleop_controller.py hand_config=7dof/7dof u2d2=dual u2d2.fake_u2d2=true verbose=true
```

## Contributing

### Adding New Controllers

1. Create new controller in `controllers/`
2. Import hand interfaces from `hand_interfaces.src`
3. Use Hydra for configuration management
4. Add documentation and examples

### Adding New Hand Interfaces

1. Extend `BaseHandInterface` in `hand_interfaces/src/`
2. Implement required methods
3. Add to factory function
4. Update documentation

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Dynamixel SDK for motor control
- Hydra for configuration management
- Tkinter for GUI components

This modular architecture provides a clean, extensible foundation for robotics control applications.