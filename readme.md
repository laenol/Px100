# PX100 Robotic Arm Setup and Usage Guide

Welcome to the PX100 Robotic Arm project! This repository contains comprehensive guides and documentation for setting up, installing, and using the Interbotix PX100 robotic arm on Ubuntu systems.

## Quick Start for First-Time Users

If you're new to the PX100 or robotics in general, follow these steps in order:

### 1. Choose Your Installation Method

We provide two installation options depending on your experience level:

- **[Option 1: Automated Installation via Script](option1.md)** - Recommended for beginners. Uses a pre-built script to install everything automatically.
- **[Option 2: Installing from Source](option2.md)** - For advanced users who want full control over the installation process.

**Start with Option 1** if you're unsure which to choose.

### 2. Hardware Setup

Before running any software:

- Connect the PX100 arm to power
- Connect the U2D2 controller to the arm
- Connect the controller to your computer via USB
- Verify the connection using `lsusb` command

### 3. Verify Your Installation

After installation, test that everything works:

- Launch the control node and RViz
- Try basic arm movements
- Test the gripper functions

### 4. Learn the Basics

Once installed, get familiar with your arm:

- **[Getting to Know Your PX100](Knowing%20your%20way%20around%20the%20Px100.md)** - Essential first steps, torque control, and basic operations
- **[PX100 and Python](Px100%20and%20Python.md)** - Complete Python API reference for programming the arm

## Prerequisites

- Ubuntu 22.04 LTS (recommended)
- ROS 2 Humble Hawksbill
- Python 3.8+
- Physical PX100 robotic arm (optional for simulation-only use)

## Installation Options

### Automated Installation (Beginner-Friendly)
Perfect for users who want a quick setup without deep Linux knowledge.

[Read the full guide →](option1.md)

### Manual Installation (Advanced)
For developers who need custom configurations or want to understand every step.

[Read the full guide →](option2.md)

## Usage

### Basic Control
```bash
# Launch the arm control (replace with your choice)
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100

# Or for simulation
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100 use_sim:=true
```

### Python Programming
```python
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

# Initialize the arm
bot = InterbotixManipulatorXS("px100", "arm", "gripper")

# Move to home position
bot.arm.go_to_home_pose()

# Close gripper
bot.gripper.grasp()
```

## Documentation

- [Hardware Setup and First Steps](Knowing%20your%20way%20around%20the%20Px100.md)
- [Complete Python API Reference](Px100%20and%20Python.md)
- [Automated Installation Guide](option1.md)
- [Manual Installation Guide](option2.md)

## Troubleshooting

Common issues and solutions:

- **USB Connection Issues**: Ensure the U2D2 is properly connected and recognized
- **ROS 2 Errors**: Check that all packages are installed and sourced correctly
- **Python Errors**: Verify your Python environment and dependencies

## Support

If you encounter issues:

1. Check the troubleshooting sections in each guide
2. Ensure all prerequisites are met
3. Try the automated installation if manual fails
4. Review the Python examples for correct usage

## License

This project uses the Interbotix PX100 robotic arm and related software. Please refer to Interbotix documentation for licensing information.

