"""
Loosly couples wheel odometry and pose estimates from SLAM
"""

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo


odom_fused_pub = rospy.Publisher('odom_fused', Odometry, queue_size=1)
right_pub = rospy.Publisher("left/camera_info", CameraInfo, queue_size=1)


def wheel_odom_callback(odom_msg):
    print("got wheel odom msg")
    global odom_fused_pub
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", odom_msg.data)
    odom_fused_pub.publish(odom_msg)

def fix_camera_info_right_cb(msg):

    global right_pub
    #ADD HEIGHT AND WIDTH DATA
    #msg.height = 376
    #msg.width = 672

    right_pub.publish(msg)

def slam_odom_callback(odom_msg):
    print("got slam msg")

def fuse_node():
    rospy.init_node('fuse_node')#, anonymous=True)

    rospy.Subscriber("wheel_odom", Odometry, wheel_odom_callback)
    rospy.Subscriber("slam_odom", Odometry, slam_odom_callback)
    rospy.Subscriber("zed/right/camera_info", CameraInfo, fix_camera_info_right_cb)

    rospy.spin()


if __name__ == '__main__':
    fuse_node()