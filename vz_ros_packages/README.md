# Robot Integration Module 

This repository glues together the different sensory modules for the Verizon (VZ) project and makes them operational on the Stretch RE1 robot via the [Robot Operating System](https://www.ros.org/) middleware. 

___

## Introduction

The packages in this repository serve two roles:
- To provide ROS wrappers around the other modules e.g. [Acoustic Scene Analysis](vz_acoustic_scene_analysis), [Optical Remote Vital Sensing](vz_optical_remote_vital_sensing) etc. This in turn will enable these modules to run on the robot, which is itself built on a ROS architecture: [stretch_ros](https://github.com/Visal_Acoustic_Nav_Petra_2023/stretch_ros). If unsure of the ROS middleware, please refer to the relevant tutorials [here](http://wiki.ros.org/ROS/Tutorials).
- To autonomously navigate the robot using mapping, localization and motion planning capabilities. The waypoints for navigation are selected based on proximity to the person of interest (POI) and how to best benefit the other sensory modules at perceiving/estimating biomarkers of this individual, e.g. maintain close proximity for remote vital sensing.

**"Wrapping" process:** The [ROS nodes](http://wiki.ros.org/Nodes) import classes from the non-ROS modules, send a stream of data to the necessary functions within those classes, receive an output from the classes, and publish this input to be used elsewhere. For a full explanation with code examples on how to re-enact this process, please check out the [ROS Wrapper Guide](#ros-wrapper-guide) at the bottom of this document.

___

## Major Dependencies

**Default Sensor Suite:** [Intel RealSense D435I RGBD Camera](https://www.intelrealsense.com/depth-camera-d435i/), [ReSpeaker V2 microphone array](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/), [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1), as well as onboard IMUs and accelerometers. See [Hello Robot Stretch RE1 docs](https://hello-robot.com/product) for more information. 

**Extended Sensor Suite:** [Structure Core camera](https://structure.io/structure-core) and [FLIR Lepton 3.5 thermal camera](https://www.digikey.com/en/product-highlight/f/flir/lepton-3-radiometry). [NVIDIA Jetson Xavier](https://developer.nvidia.com/embedded/jetson-agx-xavier-developer-kit) for accelerated compute performance.

**Software:** Python3, [ROS1-Noetic](http://wiki.ros.org/noetic), C++.

___

## Codebase Layout

The following delineates between ROS packages developed by the Northeastern (NEU) team, and those utilized that are open-source and readily available.

### NEU Packages

Essentially there is a ROS package for each of the project modules. Please refer to the `README.md` per module in the rest of the Git organization to understand their full functionality.

* [Acoustic Scene Analysis](vz_acoustic_scene_analysis)
* [Face Recognition](vz_face_recognition)
* [Human State Estimation](vz_human_state_estimation)
* [In-bed Pose Estimation](vz_inbed)
* [Optical Remote Vital Sign Sensing](vz_optical_remote_vital_sensing)
* [Pedestrian Tracker](vz_pedestrian_tracker)
* [Sensor Setup and ROS helper code](vz_ros_wrappers)
* [Robot Autonomous Navigation](vz_stretch_navigation)
* [Visual Body Pose Recognition](vz_vbpr)

### Third Party Packages

The following ROS packages have been imported into the VZ Github organization from third party repositories.

* [Stretch ROS](https://github.com/Visual_Acoustic_Nav_Petra_2023/stretch_ros): The robot comes shipped with manufacturer developed ROS drivers that help us get started. The official git repository for this can be found [here](https://github.com/hello-robot/stretch_ros).
* [Leg Tracker](https://github.com/Visual_Acoustic_Nav_Petra_2023/leg_tracker): A ROS package that uses LiDAR data (RPLidar) and outputs leg detections. This is used as one of the inputs for Human State Estimation.
* [Vision OpenCV](https://github.com/Visual_Acoustic_Nav_Petra_2023/vision_opencv): Packages for interfacing ROS with OpenCV, a library of programming functions to achieve real-time computer vision.

___

## First-Time Setup on Robot

First, follow the general instructions and advice on robot usage in the [corresponding document](RobotUsage.md). 

Then run the setup script from the [installer](https://github.com/Visual_Acoustic_Nav_Petra_2023/installation_scripts) in your `$HOME` directory on the robot's embedded machine:
```
./installation_scripts/setup_install.sh
```

This will configure your ROS workspace and the [catkin_build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) system for ROS.

___

## Launch Instructions for Individual ROS Packages

Instructions on how to launch each independent module on the Stretch RE1 robot can be found in the respective ROS packages, e.g., [here](vz_human_state_estimation/README.md) for the Human State Estimation ROS package.

Please note that for many of the hardware dependencies required to run the individual ROS packages, the following [README.md](vz_ros_wrappers) should be read first. This package serves numerous purposes, one of which is to launch the [sensors](vz_ros_wrappers/launch/sensors.launch) on the robot, i.e., fulfilling the hardware dependencies of the project packages.

___

## Target Person Registration

To register a target POI for purposes like human-following using the [human_state_estimation](vz_human_state_estimation) module, two types of registration can be run:

- Voice-based, where first the `registering` flag in the [audio_params.yaml](vz_acoustic_scene_analysis/config/audio_params.yaml) file must be set to `True`. This can be set by simply launching:
```
roslaunch vz_acoustic_scene_analysis asa.launch register:=True
```
- Face-based, where if you wish to register a new target person then you must run (stop running after a "registration complete" message appears in the terminal):  
```
roslaunch vz_face_recognition face_recognition.launch register:=True
```

In both cases, the relevant registration files will afterwards be stored in the robot's `$HOME/vz_registration` directory and utilized by other modules to identify the target person.

___

## ROS Wrapper Guide

As mentioned above, the code developed by other modules independently has to be converted into a ROS format so that it can be used on the robot. In order to do this, there are essentially two interfaces. The ROS protocol-based interface developed by the Robotics team and the `output=fn(input, parameters)` interface, which is independent of ROS and developed by the individual modules. The pseudo codes are outlined below:

- An example folder structure for "ROSification" of the Human State Estimation module. Several additional components will be added as modules grow in complexity (e.g., user defined msgs), but the basic idea shall remain the same.
 	```
	vz_human_state_estimation/
	├── CMakeLists.txt
	├── launch
	│   └── human_state_estimation.launch
	├── package.xml
	└── scripts
		└── ros_interface.py
	```
- The following is the ROS interface pseudo code developed by the Robotics team. Try to keep the function names similar. Technically this should be of minimum concern to other teams developing their independent algorithms. More specifically the file of concern is the **ros_interface.py** file shown in the above layout.  
	```
	import other_module_code as omc

	# This is the main ROS interface class on the ROS side 
	# The following is not exhaustive but captures the idea
	class ROSInterface:
		def __init__:
			# Inputs and outputs
			self.input_to_other_module_code = None
			self.output_from_other_module_code = None
			# Parameters - think neural network weights
			self.parameters_from_launch_file = rospy.get_param('yolo_one_of_the_weights')
			self.conduit_to_other_module_code = omc.ConnectToROS()
			# Getting first input from ROS sensor drivers
			rospy.Subscriber("first_input_topic_name", FirstInputMsgType, self.callback_input_to_other_module)

		def callback_input_to_other_module(self, msg_data):
			self.input_to_other_module_code = msg_data
			# Get outputs from other modules
			self.output_from_other_module_code = self.omc.process_data(self.input_to_other_module_code,
																	   self.parameters_from_launch_file)
		
	```
- The following is the expected implementation of the interface from the algorithm module side. More specifically the file of concern is the **merged_detection_algorithm.py** file.
	```
	# Please keep this name for your abstraction layer class 
	class ConnectToROS:
		def __init__(self):
			# Initialize
			self.body_pose_detection_class = BodyPoseDetection()

		# This is the main interface function between your code and ROS
		# The overall algorithm takes in inputs e.g., RGBD images
		# Second input is the parameters not expected to change over time e.g., neural network weights
		# So expect your function call to look like 'process_data(input, params)'
		# Output: output_back_to_ros: e.g., bounding boxes
		def process_data(self, input, params):
			output_back_to_ros = your_function_to_compute_bounding_boxes(input, params)
			return output_back_to_ros
	```
