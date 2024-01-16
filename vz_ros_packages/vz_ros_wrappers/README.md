## vz_ros_wrappers

This ROS package serves the following purposes:

- To launch individual sensors, e.g.,
  1. [RP Lidar](launch/lidar.launch)
  2. [Infrared Camera](launch/ir.launch)
  3. [RGBD Camera](launch/rgbd.launch)
  4. [Thermal Camera](launch/thermal.launch)

- To run multiple sensors at the same time, which can be launched as follows (parameters are configurable within the [sensors.launch](launch/sensors.launch) file):
```
roslaunch vz_ros_wrappers sensors.launch
```
  
- To launch any other necessary wrapper utilities, such as the [pos3D_to_bbox2D.launch](launch/pos3D_to_bbox2D.launch) functionality used by the [human_state_estimation](../vz_human_state_estimation) module on its output 3D position estimate.

Note that all the [RViz](rviz/) configurations to visualize the different module outputs are also found in this ROS wrapper package.
