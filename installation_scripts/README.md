# Installation Scripts

Home of the installation scripts needed to facilitate the environment setup and software installation on a new Strech RE1 robot, manufactured by Hello Robot.

The purpose of the installation script `setup_install.sh` is to provide a convenient quick setup of the environment created for the Verizon (VZ) project. This should always be run for a [first-time setup on the robot](https://github.com/Visual_Acoustic_Nav_Petra_2023/vz_ros_packages#first-time-setup-on-robot). 

The setup script will first install [ROS Noetic](http://wiki.ros.org/noetic) and create a catkin workspace. The end-result of executing `setup_install.sh` should be to have all Python and ROS dependencies installed on your machine, as well as a built ROS workspace that you can immediately start running processes from, namely `~/catkin_ws/`.

The `requirements.txt` file is supplemental to the installation script and specifies which Python scripts are required for this project.

The OpenVINO installation script is not tracked anymore in this repository, but will be freshly downloaded from Intel's website when you run the setup.

Please also note that the [Structure Core SDK](https://developer.structure.io/sdk) installation commands will require the relevant camera to be connected to the robot in order for the installation procedure to proceed.

## Running the Script

the script can be run as follows:
```
./installation_scripts/setup_install.sh
```

## Two-Machine Setup

Please note that many of the sensor suite modules are too computationally intensive for the embedded Stretch robot's Intel NUC with an i5-8259U processor. As a result, you will likely have to configure another remote machine to run certain processes off the robot.

First read the [relevant ROS tutorial](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) explaining how to start a ROS system across multiple machines.

Then configure the exact same file layout on your remote "offloading" machine by running the `setup_install.sh` script as described above. A possible layout for running many of the sensor suite modules at once might look like this:

- Robot Machine
	- Sensors
	- Navigation
- Offloading Remote Machine
	- HSE
	- ASA/In-bed/ORVS/VBPR