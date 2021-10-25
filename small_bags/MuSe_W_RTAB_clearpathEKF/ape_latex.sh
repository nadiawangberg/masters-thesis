./set_latex_params.sh

#odom
evo_ape bag s2-s2-clearpathEKF-short.bag /vicon/kobuki_vicon_link/kobuki_vicon_link /odom --align_origin --plot --save_results --save_table results/odom.zip

#rtab VO
#evo_ape bag s2-s2-clearpathEKF-short.bag /vicon/kobuki_vicon_link/kobuki_vicon_link /rtabmap/vo_odom --align_origin --plot --save_results results/rtabVO.zip

#rtab POSE - need to use tf transform
#TODO, map<->kinect_link is not properly published??? Although map->vo_link and vo_link->kinect_link seems fine??
#evo_ape bag s2-s2-clearpathEKF-short.bag /vicon/kobuki_vicon_link/kobuki_vicon_link /tf:map.kinect_link --align_origin --plot --save_results results/rtabSLAM.zip

#rtab EKF
#evo_ape bag s2-s2-clearpathEKF-short.bag /vicon/kobuki_vicon_link/kobuki_vicon_link /odometry/filtered --align_origin --plot --save_results results/EKF.zip