#evo_traj bag s2-s2-clearpathEKF-short.bag /odom /rtabmap/vo_odom /odometry/filtered --ref /vicon/kobuki_vicon_link/kobuki_vicon_link --plot --align_origin
evo_traj bag s2-s2-clearpathEKF-long.bag /odom /rtabmap/localization_pose /rtabmap/vo_odom /odometry/filtered --ref /vicon/kobuki_vicon_link/kobuki_vicon_link --plot --align_origin

#remove - test
#evo_traj bag s2-s2-long-2.bag /odom /rtabmap/vo_odom /odometry/filtered /filtered2 --ref /vicon/kobuki_vicon_link/kobuki_vicon_link --plot --align_origin

#/rtabmap/localization_pose 