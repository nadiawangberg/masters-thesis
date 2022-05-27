#evo_traj bag s2-s2-clearpathEKF-short.bag /wheel_odom /rtabmap/odom /odometry/filtered --ref /vicon/kobuki_vicon_link/kobuki_vicon_link --plot --plot_mode xy --align_origin --save_plot s2-s2-short.pgf
#evo_traj bag s2-s2-clearpathEKF-long.bag /wheel_odom /rtabmap/localization_pose /rtabmap/odom /odometry/filtered --ref /vicon/kobuki_vicon_link/kobuki_vicon_link --plot --align_origin --save_plot s2-s2-long.pgf

#remove - test
#evo_traj bag s2-s2-long-2.bag /wheel_odom /rtabmap/odom /odometry/filtered /filtered2 --ref /vicon/kobuki_vicon_link/kobuki_vicon_link --plot --align_origin

#/rtabmap/localization_pose 


#evo_traj bag test_delete.bag /wheel_odom /rtabmap/odom /rtabmap/localization_pose /odometry/filtered --ref /vicon/kobuki_vicon_link/kobuki_vicon_link --plot --plot_mode xyz --align
#evo_traj bag test_delete.bag /wheel_odom /rtabmap/odom /rtabmap/localization_pose /odometry/filtered --ref /vicon/kobuki_vicon_link/kobuki_vicon_link --plot --plot_mode xyz #--align --n_to_align 2000

evo_traj bag test_delete_dyn.bag /wheel_odom /rtabmap/odom /odometry/filtered --ref /vicon/kobuki_vicon_link/kobuki_vicon_link --plot --plot_mode xyz --align_origin  --save_plot s3-01-long.pgf #--align --n_to_align 2000




#NB, tf transforms doesnt work on MuSe bags, only on OpenLORIS bags
#evo_traj bag RTABlocalization_fused.bag /tf:odom.base_footprint /tf:odom.kinect_link /tf:map.kinect_link /odometry/filtered  /vicon/kobuki_vicon_link/kobuki_vicon_link --ref /tf:world_vicon.vicon/kobuki_vicon_link/kobuki_vicon_link --plot --plot_mode xyz --align

#/tf:world_vicon.kobuki_base_link_sensor
