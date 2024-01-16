## vz_stretch_navigation


Hardware dependencies: [RPLIDAR A1](http://wiki.ros.org/rplidar) that is default available to the robot and has a launch configuration [here](../vz_ros_wrappers/launch/lidar.launch)

Package dependencies: All the navigation packages listed in [stretch_navigation](https://github.com/paulghanem/Visual_Acoustic_Nav_Petra_2023/stretch_ros/stretch_navigation/package.xml), as well as the [TEB local planner](http://wiki.ros.org/teb_local_planner)

Inputs/Outputs: Numerous publisher-subscribers related to navigation and mapping. I recommend getting acquainted with the [Navigation Stack tutorials](http://wiki.ros.org/navigation) to build familiarity with the typical input/output topics.

### Navigation Launch Instructions

The steps outlined below are taken from [here](https://github.com/hello-robot/stretch_ros/tree/master/stretch_navigation) with a few name changes. The following is just a summary. Please look at the link for more details.

**Note:** LiDAR data is not launched by default in [sensors.launch](../vz_ros_wrappers/launch/sensors.launch) and should __NOT__ be with the following navigation launchers either. The reason for this is that the [lidar.launch](../vz_ros_wrappers/launch/lidar.launch) file is contained within the navigation launchers, e.g. at [this line](launch/vz_navigation.launch#L13) for autonomous navigation, so please keep the default sensors launch behavior.

#### Launch Mapping

Create a map of the environment as follows:
```
roslaunch vz_stretch_navigation mapping.launch
```

And then save the map (possibly after inspecting in the [RViz](../vz_ros_wrappers/rviz/mapping.rviz) view first):
```
rosrun map_server map_saver -f ${MAP_PATH}/maps/<map_name>
```

### Launch Teleoperation

You can teleoperate the Stretch by launching the following:
```
roslaunch vz_stretch_navigation navigation_teleop.launch map_yaml:=${MAP_PATH}/maps/<map_name>.yaml rviz:=false
```
Where the map is loaded from the one created in the previous step, and the `rviz` flag can be set depending on your preference for visualization or not.

### Launch Autonomous Navigation

Instead of teleoperation, in a similar fashion you can also launch autonomous navigation:
```
roslaunch vz_stretch_navigation vz_navigation.launch map_yaml:=${MAP_PATH}/maps/<map_name>.yaml rviz:=false
```
With human-following behavior enabled via the [human_state_estimation](../vz_human_state_estimation/) package, which outputs the waypoints used to guide the robot.
