# Getting to know your way around the Px100

After successfully installing and verifying the software for the PX100, it is time to become familiar with the important first steps for interacting with the physical robot or through simulation.

## Launching the Core control Node

As mentioned earlier in the verification steps for both installation options, the first command to run is the robot's drivers and visualisation.

Disclaimer - Depending on whether you would like to use the physical robot or only through simulation, it is important to add the use_sim argument at the end of the command to run the entire software stack entirely within simulation. This is good for testing in cases when you don't have a physical robot available or when trying something experimental.

Physical robot:

```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100
```

Simulation only:

```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100 use_sim:=true
```

## The "Torque Toggle"

This is a very useful feature, which in this case can also be used as a test. If everything went as expected with the following command, you should be able to manually manipulate the arm and see the digital twin move in RViz in sync. This is done by toggling the torque on the motors. This can unfortunately only be done with a physical robot.

### What is Motor Torque?

The DYNAMIXEL servos used in the PX100, when powered on, actively maintain their position against external forces. This is why the arm feels stiff and is hard to move manually while the driver is active. Turning off the torque puts the motors into a passive state, enabling the joints to move freely.

In a new terminal, run the following commands to enable or disable torque.

Disclaimer - Disabling the torque causes the arm to go limp and collapse instantly. Make sure it is in its home position or manually supported before running the command.

Disable Torque:

```bash
ros2 service call /px100/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: false}"
```

Enabling Torque:

```bash
ros2 service call /px100/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: true}"
```

ros2 service call: calls on a ROS 2 service, which is a request-response communication mechanism.

/px100/torque_enable: the name of the service being called.

interbotix_xs_msgs/srv/TorqueEnable: The data type of the service message.

"{cmd_type: 'group', name: 'all', enable: false}": cmd_type and name specify that the command applies to all servo motors on the arm.

enable: false: Disables torque.

enable: true: Enables torque.

## Running example programs

The Interbotix software installed previously includes multiple Python demo programs that show what the arm can do. This is a nice way to see the arm moving based on Python code. This can be done in other physical or a simulation. The demo files can be found in /interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/python_demos/ or a similar directory.

Launch the Control Node

In one terminal run:

```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100
```

Run the demo program

In another terminal, run the demo Python file you would like to execute.

```bash
python3 bartender.py
```

## Controlling it with a PS4 controller

The Interbotix PX 100 arm can also be manually controlled via a PS4 remote controller, allowing you to manipulate the arm as a toy freely. Connect the controller as you would normally to your computer and run the following command to control the arm with it manually.

```bash
ros2 launch interbotix_xsarm_joy xsarm_joy.launch.py robot_model:=px100
``` 
