## vz_human_state_estimation


Hardware dependencies:
- [RGBD Camera node](../vz_ros_wrappers/scripts/publish_rgbd.py)
- [RPLIDAR A1](http://wiki.ros.org/rplidar) that is default available to the robot and has a launch configuration [here](../vz_ros_wrappers/launch/lidar.launch)

Package dependencies: `vz_face_recognition`, `vz_pedestrian_tracker`, [leg_tracker](https://github.com/paulghanem/Visual_Acoustic_Nav_Petra_2023/leg_tracker)

Inputs:
- msg: [vz_ros_wrappers/Bounding_Boxes_3D](../vz_ros_wrappers/msg/Bounding_Boxes_3D.msg), [leg_tracker/PersonArray](https://github.com/Visual_Acoustic_Nav_Petra_2023/leg_tracker/blob/vz_deliverables/msg/PersonArray.msg) 
- topics: "/bb_cameraframe_face", "/bb_cameraframe_person", "/transformed_leg_poses"

Outputs:
- msg: [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html), [visualization_msgs/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html)
- topic: "/HSE/position_state", "/HSE/position_marker"

Launch command:
```
roslaunch vz_ros_wrappers HSE.launch
```

The [ros_interface.py](src/ros_interface.py) file is the actual Kalman filter implementation that fuses information from the leg tracker, pedestrian tracker and face recognition packages.

**Visualization:** For visualization purposes, we launch the `ped_face_images.py` file, which is a means of visualizing in [RViz](../vz_ros_wrappers/rviz/hse.rviz) the bounding boxes from the pedestrian and face tracker. Take a look inside the `HSE.launch` file for an example on how to launch this node.

Notes on the [HSE.launch](launch/HSE.launch) file:
- This file launches leg tracker, pedestrian tracker and face tracker
- You should first register the target person's face before expecting the state estimation module to work, as is explained [here](../README.md#target-person-registration)
- It also launches helper launch files like `bbox2D_to_pos3D.py`, which is used to convert 2D bounding box outputs of the face tracker and pedestrian tracker into 3D coordinates for Kalman filtering alongside the 3D leg points
- Also launched is the [pos3D_to_bbox2D.launch](../vz_ros_wrappers/launch/pos3D_to_bbox2D.launch) file, which converts the person of interest back into a 2D bounding box for the Visual Body Pose Recognition (VBPR) module
- Please note if you launch HSE in isolation of any other modules, but with [sensors.launch](../vz_ros_wrappers/launch/sensors.launch) to fulfil the hardware requirements, then do not forget to set the `start_lidar` argument to `true`

We also highlight that the other [HSE_bag.launch](launch/HSE_bag.launch) file is for use on pre-recorded `.bag` data collected by running the robot hardware and dependency packages, without the Kalman filtering algorithm. In other words, this bag was created by running all of the above hardware and package dependencies and then "bagging" any published topic data.
