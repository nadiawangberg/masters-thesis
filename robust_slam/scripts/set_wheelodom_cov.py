"""
Loosly couples wheel odometry and pose estimates from SLAM
"""

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo


odom_pub = rospy.Publisher('/wheel_odom', Odometry, queue_size=1)


def wheel_odom_callback(odom_msg):
    print("got wheel odom msg")
    global odom_pub

    odom_msg.

    odom_pub.publish(odom_msg)

def set_wheelodom_cov():
    rospy.init_node('set_wheelodom_cov')#, anonymous=True)

    rospy.Subscriber("/wheel_odom_bad_cov", Odometry, wheel_odom_callback)

    rospy.spin()


if __name__ == '__main__':
    set_wheelodom_cov()