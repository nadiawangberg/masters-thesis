# Semantic SLAM for Dynamic Environments 

The main contribution of this thesis is the proposal of Semantic-Kimera-VIO, a modified version of Kimera-VIO. Semantic-Kimera-VIO is an open-source stereo inertial SLAM system designed for dynamic scenes. Our system uses semantic segmentation images to classify and discard image feature points from dynamic objects.

Currently, mobile robots cannot reliably navigate in
real-world scenarios with a high density of dynamic objects. Our goal in this project is to explore new methods to filter
out feature points produced by dynamic objects, ultimately improving mobile robots’ navigation in non-ideal real-world scenes.

## Prerequisits

Ensure [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu
) is installed for your system. 

Clone [Kimera-VIO](https://github.com/nadiawangberg/Kimera-VIO) into your catkin_ws/src directory.

Clone [Kimera-VIO-ROS](https://github.com/nadiawangberg/Kimera-VIO-ROS) into your catkin_ws/src directory. 

```
sudo apt-get install ros-$ROS_DISTRO-sensor-msgs
sudo apt-get install ros-$ROS_DISTRO-jsk-rviz-plugins
```

## How to run

### Running Semantic-Kimera-VIO with the uHumans2 Dataset
```
roslaunch robust_slam kimera_uhumans.launch scene:=SCENENAME
```

SCENENAME being either office, office_w_people, neighborhood, neighborhood_w_people, apartment, apartment_w_people, subway, subway_w_people

### Running Semantic-Kimera-VIO with VIODE Dataset

```
roslaunch robust_slam kimera_viode.launch scene:=SCENENAME
```

SCENENAME being either 0_none, 1_low, 2_mid, 3_high, 0_day, 1_day, 2_day, 3_day


## How to evaluate

The Evo tool allows to evaluate Kimera using traditional metrics such as the Relative Positional Error (RPE) and Absolute Trajectory Error (ATE). This tool was modified for this project and can be found at - https://github.com/nadiawangberg/eval
