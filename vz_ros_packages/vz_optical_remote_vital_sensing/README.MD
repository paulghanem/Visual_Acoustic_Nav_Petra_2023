## vz_optical_remote_vital_sensing


Hardware dependencies: [RGBD Camera node](../vz_ros_wrappers/scripts/publish_rgbd.py)
, [Thermal Camera node](../vz_ros_wrappers/scripts/publish_thermal.py)

Package dependencies: `vz_face_recognition` and all its dependencies

Inputs:
- msgs: [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html), [vz_face_recogntion/HSE_Object](../vz_face_recognition/msg/HSE_Object.msg)
- topic: "/Lepton/image_raw", "orvs_thermal_bb"

Outputs:
- msg: [vz_optical_remote_vital_sensing/VZ_BpmData](msg/VZ_BpmData.msg)
- topics: "/orvs/resp_rate", "/orvs/heart_rate"

Launch command:
```
roslaunch vz_ros_wrappers ORVS.launch
```

**Note:** The ORVS module needs a certain number of frames to be used as an input before it can produce an estimate for respiratory and heart rates. As a consequence, you will have to wait for a minute or so until the readings stabilize and are then visualized on the RViz window launched with the module by default.

What needs to be launched separately:
- The IR sensor as suggested in the [vz_ros_wrappers](../vz_ros_wrappers/README.md) package
- [Face recognition](../vz_face_recognition/) to generate 2D bounding boxes around the region of interest, i.e., the person's face

What is launched when using [ORVS.launch](launch/ORVS.launch):
- [ir_and_thermal_bb.py](src/ir_and_thermal_bb.py) -- A conversion of bounding boxes from the RealSense RGB camera into the thermal and IR camera frames of reference.
- [text_overlay.py](src/text_overlay.py) -- A utility class to superimpose text on top of the [RViz](../vz_ros_wrappers/rviz/orvs.rviz) display for respiratory rate and heart rate.
- [orvs_hr_ros_interface.py](src/ovrs_hr_ros_interface.py) and [orvs_rr_ros_interface.py](src/orvs_rr_ros_interface.py) -- ROS nodes to estimate heart rate and respiratory rates, respectively.