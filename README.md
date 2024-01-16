# Sporadic Audio-Visual Embodied Assistive Robot Navigation For Human Tracking

This is the companion code for Human tracking algorithm reported in the paper
Sporadic Audio-Visual Embodied Assistive Robot Navigation For Human Tracking by Singh,Ghanem and Padir, PETRA 2023. The paper can
be found here (https://dl.acm.org/doi/pdf/10.1145/3594806.3594845).Please cite the
above paper when reporting, reproducing or extending the results.

## Purpose of the project

This software is a research prototype, solely developed for and published as
part of the publication cited above. It will neither be
maintained nor monitored in any way.

## Requirements, how to build, test, install, use, etc.

The code dependencies are described in [vz_ros_packages](https://github.com/paulghanem/Visual_Acoustic_Nav_Petra_2023/tree/main/vz_ros_packages). 

### Prerequesits

In order to use our human tracker, all the required packages described in  [vz_ros_packages](https://github.com/paulghanem/Visual_Acoustic_Nav_Petra_2023/tree/main/vz_ros_packages) modules and submodiles must be installed.


### Reproducing the results

The experiments reported in the publication can be run by executing

```
python benchmarks/run_real_world_tasks/run_benchmark_experiments.py
python benchmarks/run_real_world_tasks/run_large_scale_experiment.py

# Video results
A video deomnstrating human tracking is presented in the two following videos. You can watch it with 2x speed or more to speed it up: 
https://youtu.be/qnrUitc_2Tw
The video to locate human in a multi room setup is private: 
 https://youtu.be/SMGHyzsCuuQ
