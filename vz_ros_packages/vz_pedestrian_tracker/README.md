## vz_pedestrian_tracker


Hardware dependencies: [RGBD Camera node](../vz_ros_wrappers/scripts/publish_rgbd.py)

Package dependencies: None, other than those described in original module

Inputs:
- msg: [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
- topic: "/D435i/image_raw_rot"

Outputs:
- msg: [vz_face_recogntion/HSE_Objects](../vz_face_recognition/msg/HSE_Objects.msg)
- topic: "/pedestrian/object"

Launch command:
```
roslaunch vz_pedestrian_tracker pedestrian_tracker.launch
```

Note: 
- This module produces the rostopic pertaining to full body bounding boxes
- It also needs RGBD images, so if running independently make sure to run the [RGBD launch file](../vz_ros_wrappers/launch/rgbd.launch) as suggested inside the [vz_ros_wrappers package](../vz_ros_wrappers/README.md)
