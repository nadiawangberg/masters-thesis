# Wheel and visual odometry fusion in difficult light conditions 
The purpose of this project is to demonstrate the challenges of visual odometry (VO) in varying light conditions, and the added robustness of fusing VO with wheel odometry.

## The MuSe dataset

The MuSe complimentary dataset was used for this project - https://sites.google.com/view/muse-dataset-repository
Specifically the hb-s2-02/ data. 

![github](https://user-images.githubusercontent.com/29915643/145574841-da819906-59c9-4472-a366-6d73e14b8d6a.png)



## VO and wheel fusion results
![Screenshot from 2021-12-10 13-39-23](https://user-images.githubusercontent.com/29915643/145575526-a62fdd88-629d-4622-8d46-1215f05c5397.png)


## Prerequisits
```
sudo apt-get install ros-$ROS_DISTRO-robot-localization
sudo apt-get install ros-$ROS_DISTRO-rtabmap-ros
sudo apt-get install ros-$ROS_DISTRO-sensor-msgs
sudo apt-get install ros-$ROS_DISTRO-jsk-rviz-plugins
```
Clone this repository into your catkin_ws and download the required software shown above.
If this is to be run with the realsense see this - https://github.com/IntelRealSense/realsense-ros


## How to run

### Running RTAB-Map
```
roslaunch wheel_slam RTAB.launch ns:=kinect
```

### Running the EKF
```
roslaunch wheel_slam robot_localization_ekf.launch
```

### Running the Complimentory MuSe dataset
```
roslaunch wheel_slam MuSe.launch bag_type:=dark_bag start_time:=350 rviz:=true
```
