## vz_acoustic_scene_analysis


Hardware dependencies: ReSpeaker 4 Mic Array

ROS package dependencies: None

Inputs:
- msg: [vz_acoustic_scene_analysis/VZ_AudioData](msg/VZ_AudioData.msg)
- topic: "/wav_data"

Outputs:
- msg: [std_msgs/String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) 
- topic: "/speaker_pred"

Launch command:
```
roslaunch vz_acoustic_scene_analysis asa.launch
```

The above command will load all the [parameters](config/audio_params.yaml) needed to run the module. Effectively this will run the speaker verification node.

**Registration** can be achieved by following the instructions [here](../README.md#target-person-registration).