# Option 1: Automated Installation via Script

This is a step-by-step guide for the automated installation process. Each command will be broken down and explained for users unfamiliar with the Linux command line.

## Step 1: Prepare the Ubuntu Environment

First, open up a terminal window. Can be done by pressing "Ctrl + Alt + T" or by searching for "Terminal" in the applications menu.

First, you will need to fully update the system by entering the following command in the terminal.

```bash
sudo apt update && sudo apt upgrade -y
```

sudo (Superuser Do): This gives you temporary administrator ("root" user) privileges, which are needed for installing and managing system-wide software. The system will ask you for your password.

apt (Advanced Package Tool): This is Ubuntu's command-line package manager.

update: This subcommand downloads the latest package information from Ubuntu's software repositories but doesn't install them.

upgrade: This subcommand compares the versions of all the packages with the newly updated list and installs the latest version of those packages.

-y: This is a flag that automatically answers "yes" to any confirmation prompt that may appear during the upgrade process, so you don't need to enter it yourself.

Next, you need to install cURL, which is a CLI (Command-line interface) application used for uploading and downloading individual files. Install it with the following command:

```bash
sudo apt install curl
```

install: this apt subcommand installs curl.

## Step 2: Hardware connection (skip this step if you only plan to use a simulator) 

Before running the software installation, we need to make a physical connection between the computer and the robotics arm to avoid potential issues.

- Connect the arm and the U2D2 controller using the DYNAMIXEL cable
- Connect the arm to power using the power adaptor to a wall outlet.
- Connect the arm to the pc with a MicroUSB cable.

After connecting the hardware, it is time to do a quick check to see if the computer recognises the U2D2 controller. Run the following command to check:

```bash
lsusb
```

lsusb: This command lists all the USB devices connected to the system.

Look for similar text to see if the arm is connected:

If you don't have a similar text, please refer to the troubleshooting section.

## Step 3: Download and run the Installation script 

After making all the physical connections and making sure that the environment is set, the main installation script can be run. Run these simple commands in the following order.

Download the script:

```bash
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
```

curl: curl retrieves information about the installation files from GitHub.

\>: this is a shell redirect symbol. It retrieves the information called from GitHub through curl and stores it in a new file named "xsarm_amd64_install.sh" in the current directory.

Make the script executable

```bash
chmod +x xsarm_amd64_install.sh
```

chmod (Change Mode): This command changes the permissions of a file

+x: This flag makes the file executable, allowing the system to run the program.

Run the script:

```bash
./xsarm_amd64_install.sh -d humble
```

./ : This tells the system to run the file located in the current directory.

-d humble: the -d flag specifies the correct ROS 2 distribution, which in this case is ROS 2 Humble Hawksbill.

The script should run and install the full desktop version of ROS 2 Humble if it is not already installed, along with all the necessary Interbotix repositories and required packages.

The installation might take some time, depending on the internet connection and the computer's performance.

The script will ask users for input on whether to install MATLAB-ROS API, etc, and users can choose accordingly, but the default options are generally enough.

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

If successful, an RViz window should open with a 3d model of the PX100. If not, please have a look at the troubleshooting guide below.  
