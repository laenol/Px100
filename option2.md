# Option 2: Installing from source

This is a step-by-step guide for a more manual and in-control installation. Each command will be broken down and explained for users unfamiliar with ROS 2 workspace.

## Step 1: Install ROS and the needed tools

### Install ROS 2 Humble

#### Set locale

You need to make sure that you have locales which support UTF-8:

```bash
locale  # check for UTF-8
```

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

```bash
locale  # verify settings
```

#### Setup Source

Ensure that the Ubuntu Universe repository is enabled:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ros2-apt-source package:

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Install ROS 2 Packages

Update and upgrade your systems package list to include the new ROS 2 packages you just added:

```bash
sudo apt update && sudo apt upgrade -y
```

Install the full desktop version of ROS 2 Humble:

```bash
sudo apt install ros-humble-desktop -y
```

#### Environment Setup

In order to source this, you need to run this command in every new terminal:

```bash
source /opt/ros/humble/setup.bash
```

In order to avoid running the previous command every time you want to use ROS, you can add it to your .bashrc file so it automatically runs when you open a terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### Install colcon and rosdep

These two are needed development tools for ROS 2. Colcon is the stand built for ROS2, which is used to compile and source code packages. rosdep is a command-line tool for installing system dependencies for ROS packages.

```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
```

Initialise rosdep:

```bash
sudo rosdep init
rosdep update
```

## Step 2: Create Workspace and clone Repositories

Create the directory structure:

```bash
mkdir -p ~/interbotix_ws/src
```

Clone the source code:

```bash
cd ~/interbotix_ws/src
git clone https://github.com/Interbotix/interbotix_ros_manipulators.git -b humble
git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git -b humble
```

## Step 3: Install Dependencies and Build the Workspace

Install dependencies

```bash
cd ~/interbotix_ws
rosdep install --from-paths src --ignore-src -r -y
```

rosdep install: Scans the package.xml files within the src directory, finds all the listed dependencies and uses apt to install them

--from-paths src: Indicates directory to search for packages.

--ignore-src: stops rosdep from trying to install packages that are already installed.

-r: so it continues even if some dependencies fail to install

-y: automatically answers "yes" to prompts from the system

Build the workspace:

```bash
colcon build
```

## Step 4: Installation verification 

Once the installation is completed successfully, a few final steps need to be run to verify the installation and configure the terminal environment.

Source the workspace: The installation script creates a ROS 2 workspace, usually at ~/Interbotix_ws. To make use of the installed ROS packages, the terminal needs to be told about it, which is done by "sourcing" the setup file.

```bash
source ~/interbotix_ws/install/setup.bash
```

source: This is a shell command that executes the command mentioned in the specified file in the current shell's context by loading the environment variables and functions.

This command must be executed in every new terminal opened for PX100 usage. To automate this step and reduce repetition, add it to the .bash file, which will be executed automatically with every new terminal. Do this by running:

```bash
echo "source ~/interbotix_ws/install/setup.bash" >> ~/.bashrc
```

Start the control Node and Rviz:

For the main test, it is good to run the main control node for PX100. This node is responsible for talking with the arms hardware and publishing its state. This will also start RViz for visualisation.

```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100
```

ros2 launch: executes the ROS 2 file, which can start and set multiple nodes and set parameters at the same time.

interbotix_xsarm_control: the name of the ROS 2 file that contains the launch file.

xsarm_control.launch.py: specifies the launch file to be run.

robot_model:=px100: is part is important as it tells the launch file robot configuration to run, which now happens to be the px100.

Disclaimer - Make sure to add use_sim:=true at the end of this command if you plan on only using the simulator.

If successful, an RViz window should open with a 3d model of the PX100. If not, please have a look at the troubleshooting guide below
