"""
Loosly couples wheel odometry and pose estimates from SLAM
"""

import rospy
from nav_msgs.msg import Odometry
import ekf_util
import numpy as np


odom_fused_pub = rospy.Publisher('nadia_ekf_fused', Odometry, queue_size=1)

xDR = np.zeros((4, 1))  # Dead reckoning

xEst = np.zeros((4, 1))
PEst = np.eye(4)


def wheel_odom_callback(odom_msg):
    print("got wheel odom msg")
    global odom_fused_pub, xDR, xEst, PEst

    
    # u - control signal? 
    # z - gps
    xDR = ekf_util.motion_model(xDR, u)

    ekf_util.ekf_estimation(xEst, PEst, z, u)


    odom_fused_pub.publish(odom_msg)

def slam_odom_callback(odom_msg):
    print("got slam msg")

def fuse_node():
    rospy.init_node('fuse_node')#, anonymous=True)

    rospy.Subscriber("wheel_odom", Odometry, wheel_odom_callback)
    rospy.Subscriber("rtabmap/odom", Odometry, slam_odom_callback)

    rospy.spin()


if __name__ == '__main__':
    fuse_node()