# Dual U2D2 Configuration

This document explains how to use the teleop system with separate U2D2 devices for leader and follower chains.

## Overview

The teleop system now supports two U2D2 configurations:

1. **Single U2D2** (default): Both leader and follower chains connected to the same U2D2 device
2. **Dual U2D2**: Leader and follower chains connected to separate U2D2 devices

The dual U2D2 configuration is useful when:
- The follower chain is mounted on a robot arm
- You need to physically separate the leader and follower systems
- You want to reduce communication overhead by using dedicated U2D2 devices

## Configuration

### Single U2D2 (Default)
```yaml
# conf/u2d2/default.yaml
usb_port: "/dev/ttyUSB0"
baudrate: 4000000
use_dual_u2d2: false
```

### Dual U2D2
```yaml
# conf/u2d2/dual.yaml
follower:
  usb_port: "/dev/ttyUSB0"  # Follower U2D2 (on robot arm)
  baudrate: 4000000

leader:
  usb_port: "/dev/ttyUSB1"  # Leader U2D2 (local)
  baudrate: 4000000

use_dual_u2d2: true
```

## Usage

### Method 1: Using Configuration Files
```bash
# Single U2D2 (default)
python run_teleop.py hand_config=2dof/2dof_parallel_xc330_to_xm430_torque_off

# Dual U2D2
python run_teleop.py hand_config=2dof/2dof_parallel_xc330_to_xm430_torque_off u2d2=dual
```

### Method 2: Programmatic Configuration
```python
from src.teleop_hand import TeleopHand
from omegaconf import DictConfig

# Create configuration
cfg = DictConfig({
    'u2d2': {
        'use_dual_u2d2': True,
        'follower': {
            'usb_port': '/dev/ttyUSB0',
            'baudrate': 4000000
        },
        'leader': {
            'usb_port': '/dev/ttyUSB1',
            'baudrate': 4000000
        }
    },
    # ... other configuration
})

# Create and run teleop
teleop = TeleopHand(cfg)
teleop.run()
```

## Hardware Setup

### Single U2D2 Setup
```
[Leader Motors] ----\
                    U2D2 ---- USB ---- Computer
[Follower Motors] --/
```

### Dual U2D2 Setup
```
[Leader Motors] ---- U2D2-1 ---- USB ---- Computer
[Follower Motors] -- U2D2-2 ---- USB ---- Computer
```

## Performance Considerations

### Communication Overhead
- **Single U2D2**: All motors communicate through one USB port
- **Dual U2D2**: Motors communicate through separate USB ports, potentially reducing contention

### Timing Synchronization
- Both configurations maintain the same control loop timing
- Dual U2D2 configuration reads from both interfaces in parallel
- No additional threading is required - the U2D2 interface operations are already optimized

### Error Handling
- Each U2D2 interface is handled independently
- Communication failures on one interface don't affect the other
- Graceful degradation if one interface becomes unavailable

## Troubleshooting

### Common Issues

1. **USB Port Not Found**
   - Check that both U2D2 devices are connected
   - Verify USB port paths in configuration
   - Use `ls /dev/ttyUSB*` to list available ports

2. **Communication Timeouts**
   - Check baudrate settings match motor configurations
   - Verify motor IDs are correct for each chain
   - Check USB cable quality and length

3. **Control Loop Performance**
   - Monitor control frequency warnings
   - Consider reducing control frequency if needed
   - Check for USB bandwidth limitations

### Debug Mode
Enable verbose output to see detailed communication information:
```bash
python run_teleop.py hand_config=2dof/2dof_parallel_xc330_to_xm430_torque_off u2d2=dual general.verbose=true
```

## Migration from Single to Dual U2D2

1. **Hardware**: Connect follower motors to second U2D2 device
2. **Configuration**: Update U2D2 settings to use dual configuration
3. **Testing**: Verify both chains work independently
4. **Deployment**: Update production configurations

## Example Scripts

See `examples/dual_u2d2_example.py` for a complete working example of dual U2D2 configuration.
