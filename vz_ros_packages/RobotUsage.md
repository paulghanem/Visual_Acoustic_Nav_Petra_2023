# Robot Usage

This document provides advice and tips on robot startup, safety, as well as getting it ready for full operation. For more general best practices, please refer to [this](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/best_practices/) document.

 
## Turning On & General Safety

- Do not leave the robot in a plugged-in, unattended state for a long time. This could be a fire hazard.
- Please refer to the "Power" section [here](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/quick_start_guide_re1/#power) to understand powering on the robot and the "Get Plugged In" section [here](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/quick_start_guide_re1/#get-plugged-in) to understand different charging modes found [here](https://docs.hello-robot.com/0.2/stretch-hardware-guides/docs/battery_maintenance_guide_re2/).
- Once powered ON, you can choose to physically connect a monitor and mouse to the robot by plugging in the USB connector to one of the many USB ports available on the robot. It is recommended to use the USB ports near the ON/OFF button.
- Please familiarize yourself with how to emergency shut down the robot and general protocols for safe robot operation by referring to the ["Runstop"](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/safety_guide/#runstop) and ["Safe Handling"](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/quick_start_guide_re1/#safe-handling) sections. 
- The robot is heavy. Please do not attempt to lift it all by yourself.
- Please also go through the Robot Safety Guide found [here](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/safety_guide/).


## Initial Setup Steps

First perform the homing calibration routine as follows in a new terminal. Ensure that the robot has enough surrounding space in all directions to be able to stretch its grippers fully before running this command.

```
stretch_robot_home.py
```

- The robot should now be performing a choreographed homing routine. 
- This step is just an internal sanity check for the robot. The full calibration of the robot is performed differently, as we will see later.
- Next stow the robot by running another command in a new terminal. You should see the robot fold itself into a non-protruding pose.
	```
	stretch_robot_stow.py
	```
- Next ensure all the hardware is there and happy using the following command in a new terminal. 
	```
	stretch_robot_system_check.py
	```
- If everything is green, then the hardware is happy. If not, re-run the above steps.
- If you want to send some custom commands to the different joints in the robot you can do so using [this](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/quick_start_guide_re1/#start-coding) guide. 
- This is just for more sanity checks and can be avoided once you have become familiar with the robot. Once happy with it, you can exit ipython by typing `exit` in the terminal.
- Some useful links to read before proceeding to calibration are the [Quick Start](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/quick_start_guide_re1/) and [Stretch Body](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/) guides.

## Debugging Cameras Launch

The command [roslaunch vz_ros_wrappers sensors.launch](vz_ros_wrappers/launch/sensors.launch) launches all the sensors that have been set to `true` at the top of the launch file. Occasionally, due to hardware issues inherent to the robot, the sensors will fail to launch on the ROS system correctly. In those cases, the followng steps may prove effective:
- Ensure that the thermal and the Structure Core IR cameras, which have external wirings, have been plugged in correctly.
- It is preferable to keep the IR camera plugged into the top USB port of the robot, next to the microphone.
- It is also preferable to keep the thermal camera plugged into the bottom USB rack, where the power ON/OFF button for the robot is located.
- Use RViz to confirm that all the camera outputs look healthy. You can do so by visualizing this list of topics:
```
		RGB : /D435i/image_raw_rot 
		Thermal : /Lepton/image_raw
		Structure Core : /sc/infrared_left/image
```
- The thermal camera can sometimes be sensitive to what port it is plugged into. Therefore, you may need to change the port as indicated by '6' in [this line of code](vz_ros_wrappers/scripts/publish_thermal.py#L36). This should alleviate the problem.
- If none of the above works, reboot by physically turning the switch off. After the robot has rebooted, try homing and stowing as described above, before then checking the health of the RGB camera in RViz to validate things are fixed.


## Remote Access and Untethered Operation

- It is recommended to read [this](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/untethered_operation/) to understand remote operation on the robot. Follow the steps outlined [here](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/untethered_operation/#ros-remote-master) to make [RViz](http://wiki.ros.org/rviz) work on your machine when operating the robot untethered.


## Turning Off the Robot

- Please follow steps outlined in the ["Power Down"](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/quick_start_guide_re1/#power-down) section and the [Best Practices video](https://www.youtube.com/watch?v=mQdOGEksdYM).

## Further Assistance/Help

If something not documented here crops up when using the Stretch RE1 robot, then it would be best to contact support at Hello Robot: support@hello-robot.com
