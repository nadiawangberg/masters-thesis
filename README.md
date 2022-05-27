# Semantic SLAM for Dynamic Environments 

The main contribution of this thesis is the proposal of Semantic-Kimera-VIO, a modified version of Kimera-VIO. Semantic-Kimera-VIO is an open-source stereo inertial SLAM system designed for dynamic scenes. Our system uses semantic segmentation images to classify and discard image feature points from dynamic objects.

<!-- ![Screenshot from 2022-05-27 13-50-26](https://user-images.githubusercontent.com/29915643/170694471-e35f1d3a-740e-41c6-82a3-c0584cd20443.png) -->

<img src="https://user-images.githubusercontent.com/29915643/170694471-e35f1d3a-740e-41c6-82a3-c0584cd20443.png" width="700" >

Currently, mobile robots cannot reliably navigate in
real-world scenarios with a high density of dynamic objects. Our goal in this project is to explore new methods to filter out feature points produced by dynamic objects, ultimately improving mobile robots’ navigation in non-ideal real-world scenes.

## Prerequisits

Ensure [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu
) is installed for your system. 

Clone our version of [Kimera-VIO](https://github.com/nadiawangberg/Kimera-VIO) into your catkin_ws/src directory.

Clone our version of [Kimera-VIO-ROS](https://github.com/nadiawangberg/Kimera-VIO-ROS) into your catkin_ws/src directory. 

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

## Results

### Filtering out Dynamic Outliers by Semantic Segmentation

<!-- ![Screenshot from 2022-05-27 13-50-53](https://user-images.githubusercontent.com/29915643/170694691-7415dd20-d5ba-4877-b62d-a8ea83970fb5.png) -->

<img src="https://user-images.githubusercontent.com/29915643/170694691-7415dd20-d5ba-4877-b62d-a8ea83970fb5.png" width="700" >




### Original Kimera-VIO (left) vs Semantic-Kimera-VIO(right)

<!-- ![Screenshot from 2022-05-27 13-51-28](https://user-images.githubusercontent.com/29915643/170694585-cc99f04a-be65-4c3f-9dcc-8a30301c51fc.png) -->
<img src="https://user-images.githubusercontent.com/29915643/170694585-cc99f04a-be65-4c3f-9dcc-8a30301c51fc.png" width="720" >

<!-- ![Screenshot from 2022-05-27 13-51-17](https://user-images.githubusercontent.com/29915643/170694590-25095ae8-6bde-46d8-81df-3e248c537762.png) -->
<img src="https://user-images.githubusercontent.com/29915643/170694590-25095ae8-6bde-46d8-81df-3e248c537762.png" width="720" >

Further results and analysis are shown in the NadiaWangbergThesis2022.pdf


