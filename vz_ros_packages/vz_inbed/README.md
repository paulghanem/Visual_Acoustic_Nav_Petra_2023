## vz_inbed 

Hardware dependencies: [Thermal Camera node](../vz_ros_wrappers/scripts/publish_thermal.py)

Package dependencies: None

Inputs:
- msg: [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
- topic: "/Lepton/image_raw"

Outputs:
- msg: [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
- topic: "/vz_inbed/vis"

Launch command:
```
roslaunch vz_inbed inbed_launcher.launch
```
