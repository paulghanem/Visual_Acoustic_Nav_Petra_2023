## vz_vbpr

Hardware dependencies: [RGBD Camera node](../vz_ros_wrappers/scripts/publish_rgbd.py)

Package dependencies: The [ROS wrapper of the HSE submodule](../vz_human_state_estimation) and all its dependencies

Inputs:
- msgs: [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html), [vz_face_recognition/HSE_Object](../vz_face_recognition/msg/HSE_Object.msg)
- topics: "/D435i/image_raw_rot", "/ip_vbpr_pub"

Outputs:
- msgs: [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html), [std_msgs/String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) 
- topics: "/vz_vbpr/pose", "/vz_vbpr/action"

Launch command:
```
roslaunch vz_vbpr vbpr_launcher.launch
```

### Launch Instructions

First register the subject by following the instructions [here](../README.md#target-person-registration). Then afterwards run the [human_state_estimation](../vz_human_state_estimation/) module:
```sh
roslaunch vz_ros_wrappers HSE.launch
```

Before finally launching the visual body pose recognition network:
```sh
roslaunch vz_vbpr vbpr_launcher.launch
```

### ROS Action Publisher

The action recognition output is the class index (an integer [0-60]) that is published as an `std_msgs/String` message type over the "/vz_vbpr/action" topic. Example output looks like this:
```sh
rostopic echo /vz_vbpr/action
data: -1
---
data: -1
---
data: -1
---
data: -1
---
data: -1
---
data: -1

```

### Additional Information

The Hybrik algorithm can be found [here](https://github.com/Jeff-sjtu/HybrIK). We directly use its pretrained model.

The action recognition algorithm (MS-G3D) can also be found [here](https://github.com/kenziyuliu/MS-G3D). We have re-trained the model to be "model_final_a6e10b.pkl" using a configuration suitable for our application. The frame rate for skeleton estimation is changed from 30 to 15 fps. We only use the 18 joints overlapping between the Hybrik output and SMPL joints from the [NTU action recognition dataset](https://rose1.ntu.edu.sg/dataset/actionRecognition/).
