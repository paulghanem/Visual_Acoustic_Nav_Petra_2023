## vz_face_recognition

Complementary module: [face_recognition](https://github.com/VZ-Project/face_recognition)

Hardware dependencies: [RGBD Camera node](../vz_ros_wrappers/scripts/publish_rgbd.py)

Package dependencies: None, other than those described in original module.

Inputs:
- msg: [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
- topic: "/D435i/image_raw_rot"

Outputs:
- msg: [vz_face_recogntion/HSE_Objects](msg/HSE_Objects.msg)
- topic: "/face/object"

Launch command:
```
roslaunch vz_face_recognition face_recognition.launch
```

The above launch will still need the RGB camera to run. Thus if wanting to run the module independently, make sure to run the RGBD camera as instructed in the [vz_ros_wrappers package](../vz_ros_wrappers/README.md).

**Registration** can be achieved by following the instructions [here](../README.md#target-person-registration).
