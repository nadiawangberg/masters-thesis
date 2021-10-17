#!/usr/bin/python

import rospy
from sensor_msgs.msg import CameraInfo, Image

left_info_pub = rospy.Publisher('/zed/right/camera_info', CameraInfo, queue_size=1)
right_info_pub = rospy.Publisher("/zed/left/camera_info", CameraInfo, queue_size=1)

# right_pub = rospy.Publisher("/proc/left/image_raw", Image, queue_size=1)
# left_pub = rospy.Publisher("/proc/left/image_raw", Image, queue_size=1)

def fix_camera_info_right_cb(msg):

    global right_pub
    #ADD HEIGHT AND WIDTH DATA
    msg.height = 376
    msg.width = 672

    right_info_pub.publish(msg)

def fix_camera_info_left_cb(msg):

    global left_pub
    #ADD HEIGHT AND WIDTH DATA
    msg.height = 376
    msg.width = 672

    left_info_pub.publish(msg)


# def left_img_cb(msg):
#     """ Simply renames for stereo_image_proc"""
#     left_pub.publish(msg)


# def right_img_cb(msg):
#     right_pub.publish(msg)
    

def camera_calib_fix():
    rospy.init_node("camera_calib_fix")

    rospy.Subscriber("right/camera_info", CameraInfo, fix_camera_info_right_cb)
    rospy.Subscriber("left/camera_info", CameraInfo, fix_camera_info_left_cb)

    # rospy.Subscriber("zed/right/image_raw", Image, right_img_cb)
    # rospy.Subscriber("zed/left/image_raw", Image, left_img_cb)

    rospy.spin()


if __name__ == '__main__':
    camera_calib_fix()