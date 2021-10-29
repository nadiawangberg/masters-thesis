./set_latex_params.sh

#If you want, you can also store a backup of the trajectories that were used to generate the result in the .zip file
evo_config set save_traj_in_zip true

#wheel odom
#evo_ape bag s2-s2-clearpathEKF-short.bag /vicon/kobuki_vicon_link/kobuki_vicon_link /odom --align_origin --plot --save_results results/odom.zip

#rtab VO
#evo_ape bag s2-s2-clearpathEKF-short.bag /vicon/kobuki_vicon_link/kobuki_vicon_link /rtabmap/vo_odom --align_origin --plot --save_results results/rtabVO.zip

#rtab POSE - need to use tf transform
#TODO, map<->kinect_link is not properly published??? Although map->vo_link and vo_link->kinect_link seems fine??
#evo_ape bag s2-s2-clearpathEKF-short.bag /vicon/kobuki_vicon_link/kobuki_vicon_link /tf:map.kinect_link --align_origin --plot --save_results results/rtabSLAM.zip

#rtab EKF
evo_ape bag s2-s2-clearpathEKF-short.bag /vicon/kobuki_vicon_link/kobuki_vicon_link /odometry/filtered --align_origin --plot --save_results results/EKF.zip