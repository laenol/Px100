# PX100 Robotic Arm Guide

Setup and usage guide for the Interbotix PX100 robotic arm on Ubuntu.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Python 3.8+
- PX100 robotic arm (optional for simulation)

## Quick Start

1. **Choose Installation**: Start with [Option 1: Automated Installation](option1.md) for beginners.

2. **Hardware Setup**:
   - Connect PX100 arm to power
   - Connect U2D2 controller to arm and USB
   - Run `lsusb` to verify connection

3. **Install Software**: Follow the chosen installation guide.

4. **Test Installation**:
   ```bash
   ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100
   ```

5. **Learn Basics**: Read [Getting Started Guide](Knowing%20your%20way%20around%20the%20Px100.md)

## Usage

### Launch Control
```bash
# Physical arm
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100

# Simulation
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100 use_sim:=true
```

### Python Control
```python
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

bot = InterbotixManipulatorXS("px100", "arm", "gripper")
bot.arm.go_to_home_pose()
bot.gripper.grasp()
```

## Documentation

- [Automated Installation](option1.md)
- [Manual Installation](option2.md)
- [First Steps & Controls](Knowing%20your%20way%20around%20the%20Px100.md)
- [Python API Reference](Px100%20and%20Python.md)

## Troubleshooting

- **USB Issues**: Check U2D2 connection with `lsusb`
- **ROS Errors**: Ensure packages installed and sourced
- **Python Errors**: Verify environment setup

For more help, check troubleshooting in each guide.

