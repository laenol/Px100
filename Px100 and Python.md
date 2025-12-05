# Px100 and Python 

Python also enables control over the Interbotix PX100, giving you comprehensive command of the arm for various manipulations. You might encounter some issues when using the demo code, as its methods can occasionally produce "TypeError" or "AttributeError." In this section, we present the 20 most essential methods and attributes for beginners working with the PX100. We will discuss the main arm controls, gripper functions, diagnostics, debugging, advanced controls, and more. Below is a complete Python script that shows you how to use the first 9 basic controls.

## Core Arm Controls

* The moving_time in all the methods is optional, it just tells how long the move should take in seconds. 

### Go to Home Position

Sends the arm to its default "home" position (all joints at 0 radians).

Example would be:

```python
bot.arm.go_to_home_pose(moving_time=2.0)
```

### Go to Sleep Position

Tucks the arm into its safe "sleep" position.

Example using it at the end of your program in the finally block to safely end your program and make it easier when running next time:

```python
finally:
    bot.arm.go_to_sleep_pose(moving_time=2.0)
```

### Set All Joint Angles (Joint Control)

Moves all arm joints to specific angles (in radians) at the same time. For PX100 the joints are: [waist, shoulder, elbow, wrist_angle].

Example:

```python
my_angles = [0.5, 0.2, -0.5, 0.5] # Angles in radians
bot.arm.set_joint_positions(my_angles, moving_time=2.0)
```

### Set a Single Joint Angle

Moves just one specific joint to a new angle.

Example:

```python
# Move just the 'waist' joint
bot.arm.set_single_joint_position('waist', -1.0, moving_time=1.5)
```

### Move Gripper to (X, Y, Z) Coordinate using Inverse Kinematics

Moves the gripper to a specific (x, y, z) coordinate in meters.

Example:

```python
# Move to 15cm forward (x) and 18cm up (z) from the arm's base
bot.arm.set_ee_pose_components(x=0.15, z=0.18, moving_time=2.0)
``` 

## Gripper controls

### Close Gripper

Closes the gripper.

Example:

```python
bot.gripper.grasp()
```

### Open Gripper

Opens the gripper.

Example:

```python
bot.gripper.release()
```

### Get Gripper Position

Reads the current angle of the gripper joint in radians.

Example:

```python
current_pos = bot.gripper.get_gripper_position()
print(f"Gripper is at: {current_pos}")
``` 

 

## Diagnostics

### Get Actual Joint Angles

Returns a list of the current, actual angles (in radians) of all arm joints.

Example:

```python
real_angles = bot.arm.get_joint_positions()
print(f"Arm is actually at: {real_angles}")
```

### Get Actual Joint Velocities

Returns a list of the current, actual velocities (in rad/s) of all arm joints.

Note: This may return an error if your robot's driver isn't set up to publish velocity data.

Example:

```python
velocities = bot.arm.get_joint_velocities()
```

### Get Actual Joint Efforts (Load)

Returns a list of the current, actual effort (load) on each joint.

Note: This may return an error if your robot's driver isn't set up to publish effort data.

Example:

```python
efforts = bot.arm.get_joint_efforts()
```

### Get Actual Gripper Pose (Matrix)

Returns the current, actual 4x4 Transformation Matrix (a NumPy array) of the gripper's pose in space.

Example:

```python
import numpy as np

current_pose_matrix = bot.arm.get_ee_pose()
print(np.round(current_pose_matrix, 3))
``` 

## Debugging

### Get Last Commanded Joint Angles

Returns the list of joint angles from your last sent command.

Example:

```python
last_command = bot.arm.get_joint_commands()
print(f"I last told the arm to go to: {last_command}")
```

### Get Last Commanded Angle for a Single Joint

Gets the last commanded angle for just one joint.

Example:

```python
last_waist_cmd = bot.arm.get_single_joint_command('waist')
```

### Get Last Commanded Gripper Pose (Matrix)

Returns the 4x4 matrix of the last commanded end-effector pose.

Example:

```python
last_ee_matrix = bot.arm.get_ee_pose_command()
```

## Advanced Controls & More 

### Move Gripper in a Straight Line

Moves the gripper in a straight line relative to its current position.

Example:

```python
# Move 5cm straight up (positive Z) from where it is now
bot.arm.set_ee_cartesian_trajectory(z=0.05, moving_time=2.0)
```

### Move Gripper to Pose using a Matrix (IK)

The "power user" version of set_ee_pose_components. You give it a full 4x4 NumPy transformation matrix.

Example:

```python
# T_sd_home is a 4x4 matrix you define
T_sd_home = np.array([[1,0,0,0.231], [0,1,0,0], [0,0,1,0.200], [0,0,0,1]])
bot.arm.set_ee_pose_matrix(T_sd_home, moving_time=2.0)
```

### Sync Controller to Arm's Actual Position

Syncs the controller's internal "commanded" state to the arm's actual position.

Note: Call this if you ever manually move the arm (while torque is off) to tell the controller where the arm is now.

Example:

```python
bot.arm.capture_joint_positions()
```

### Get All Arm Hardware Info

This is an attribute (a variable), not a method. It holds all the setup info for your arm.

Example:

```python
# Get a list of all joint names
print(bot.arm.group_info.joint_names)

# Get the angle limits for all joints
print(bot.arm.group_info.joint_lower_limits)
print(bot.arm.group_info.joint_upper_limits)
```

### Get the Number of Joints

A simple helper that returns the number of joints in the arm.

Example:

```python
num_joints = bot.arm.get_number_of_joints()
print(f"This arm has {num_joints} joints.")
```

 
