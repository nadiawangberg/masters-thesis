"""
Loosly couples wheel odometry and pose estimates from SLAM
"""

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gtsam.symbol_shorthand import X, L
from geometry_msgs.msg import Pose, PoseStamped #, Twist, PoseArray, PoseStamped
import cv2
import numpy as np
import gtsam

# from wheel_slam.src.wheel_slam import slam_ros_utils
# import slam_ros_utils as utl

br = CvBridge()

class Slam(object):
    def __init__(self):

        #Init image variables
        self.br = CvBridge()
        self.image = None
        self.depth = None

        self.initial_estimate = gtsam.Values()  
        self.factor_graph = gtsam.NonlinearFactorGraph()

        isam_params = gtsam.ISAM2Params()
        self.isam = gtsam.ISAM2(isam_params)

        self.indx = 1

        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01])) #0.1
        self.factor_graph.push_back(gtsam.PriorFactorPose2(self.indx, gtsam.Pose2(0.0, 0.0, 0.0), prior_noise))
        self.initial_estimate.insert(self.indx, gtsam.Pose2(0.01, 0.01, 0.01)) #0.5, 0.0, 0.2
        # self.isam.update(self.factor_graph, self.initial_estimate)

        #Init ROS publishers and subscribers
        rospy.Subscriber("wheel_odom", Odometry, self.wheel_odom_cb) # /rtabmap/odom"
        # rospy.Subscriber("kinect/rgb/image_raw",Image, self.img_cb)
        # rospy.Subscriber("kinect/depth/image_raw", Image, self.depth_cb)
        self.odom_pub = rospy.Publisher('slam_estm', Odometry, queue_size=1)
        self.pose_pub = rospy.Publisher('pose_estm', PoseStamped, queue_size=1)

        #self.slam_estm = gtsam.Pose2(0.0, 0.0, 0.0)
        # result = self.isam.calculateEstimate()
        # self.slam_estm = result.atPose2(self.indx)
        self.slam_estm = gtsam.Pose2(0.0, 0.0, 0.0)

    def wheel_odom_cb(self, odom_msg): #NOTE, this works for all types of twist information, including from VO
        # factor_graph = gtsam.NonlinearFactorGraph()
        # initial_estimate = gtsam.Values()
        
        self.indx += 1
        odom_hz = 1

        odom_gtsam, odom_noise_gtsam = self.msg_2_gtsam(odom_msg, odom_hz)

        #Add to factor graph
        self.factor_graph.push_back(gtsam.BetweenFactorPose2(self.indx-1, self.indx, odom_gtsam, odom_noise_gtsam))
        self.initial_estimate.insert(self.indx, self.slam_estm) #TODO should odometry be added to this (aka constant velocity model)
        
        # print(factor_graph)
        # print(initial_estimate)
        
        self.isam.update(self.factor_graph, self.initial_estimate)
        
        result = self.isam.calculateEstimate()
        self.slam_estm = result.atPose2(self.indx)

        self.initial_estimate.clear()     
            

    def msg_2_gtsam(self, odom_msg, odom_hz):
        cov = odom_msg.pose.covariance # 1x36 vector representing 6x6 matrix
        x = odom_msg.twist.twist.linear.x
        y = odom_msg.twist.twist.linear.y
        yaw = odom_msg.twist.twist.angular.z

        #GTSAM pose
        odometry = gtsam.Pose2(x/odom_hz, y/odom_hz, yaw/odom_hz) #NOTE! divided by 50 as wheel odometry is updated at 1Hz, but falsely published in ROS at 50Hz

        #GTSAM covariance
        odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([cov[0], cov[7], cov[35]]))

        return odometry, odometry_noise


    def publish_pose_estm(self):

        # print(self.factor_graph)
        # print(self.initial_estimate)

        # print(self.result.dim())

        # if (self.result.dim() == 0):
        #     return

        ros_pose = self.toPose(self.slam_estm.x(),self.slam_estm.y(),self.slam_estm.theta())
            
        pose_msg = self.toPoseStamped(ros_pose, "odom", rospy.Time.now())
        self.pose_pub.publish(pose_msg)

        odom_msg = self.toOdometry(ros_pose, "odom", rospy.Time.now())
        self.odom_pub.publish(odom_msg)

    def img_cb(self, img_msg):
            # rospy.loginfo('Image received...')
            self.image = self.br.imgmsg_to_cv2(img_msg)
            self.show_image(self.image, "Image Window")

    def depth_cb(self, img_msg):
            self.depth = self.br.imgmsg_to_cv2(img_msg)
            # self.show_image(self.depth, "Depth Window")

    def solve_factor_graph(self):

        """
        # Create an optimizer.
        params = gtsam.LevenbergMarquardtParams()
        
        # Stop iterating once the change in error between steps is less than this value
        params.setRelativeErrorTol(1e-5)
        # Do not perform more than N iteration steps
        params.setMaxIterations(100)

        optimizer = gtsam.LevenbergMarquardtOptimizer(self.factor_graph, self.initial_estimate, params)
        # Solve the MAP problem.
        result = optimizer.optimize() #Does COLAMD ordering by default
        """

        result = self.isam.calculateEstimate()
        self.slam_estm = result.atPose2(self.indx)

        # Calculate marginal covariances for all variables.
        #marginals = gtsam.Marginals(self.pose_graph, result)

    def toPose(self,x,y,theta):
        pos = Pose()
        pos.position.x = x
        pos.position.y = y

        orient = self.angleaxis_to_quat([0,0,1], theta)
        pos.orientation.w = orient[0]
        pos.orientation.x = orient[1]
        pos.orientation.y = orient[2]
        pos.orientation.z = orient[3]
        return pos

    def toOdometry(self,rosPose, frame_id, time):
        odom = Odometry()
        odom.header.stamp = time
        odom.header.frame_id = frame_id
        odom.pose.pose = rosPose
        #odom.pose.covariance = ...
        #odom.twist = ...
        return odom

    def toPoseStamped(self,rosPose, frame_id, time):
        pos = PoseStamped()
        pos.header.stamp = time
        pos.header.frame_id = frame_id
        pos.pose = rosPose
        return pos

    #Helper functions
    def show_image(self, img, name):
        cv2.imshow(name, img)
        cv2.waitKey(3)

    def quaternion_to_euler(self, quaternion):
        quaternion_squared = quaternion ** 2
        phi = np.arctan2(2*quaternion[3]*quaternion[2]+2*quaternion[0]*quaternion[1],quaternion_squared[0] - quaternion_squared[1] - quaternion_squared[2] + quaternion_squared[3])
        theta = np.arcsin(2*(quaternion[0]*quaternion[2]-2*quaternion[1]*quaternion[3]))
        psi =  np.arctan2(2*quaternion[1]*quaternion[2]+2*quaternion[0]*quaternion[3],quaternion_squared[0] + quaternion_squared[1] - quaternion_squared[2] - quaternion_squared[3])
        euler_angles = np.array([phi,theta,psi])
        return euler_angles

    def ros_quat_to_yaw(self, quat): 
        #assumes rot around z only
        q_np = np.array([quat.w, quat.x, quat.y, quat.z])
        euler_angles = self.quaternion_to_euler(q_np)
        return euler_angles[2]
    
    def angleaxis_to_quat(self, axis, angle):
        ax,ay,az = axis
        qx = ax*np.sin(angle/2)
        qy = ay*np.sin(angle/2)
        qz = az*np.sin(angle/2)
        qw = np.cos(angle/2)
        return [qw,qx,qy,qz]
        

if __name__ == '__main__':
    rospy.init_node('simple_slam')
    slam_obj = Slam()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        # if (self.indx % odom_hz == 0): #TODO changing this number changes the answer... whyyy? (it does not anymore???)
        slam_obj.publish_pose_estm()

        rate.sleep()



#Thrash

#Initial estimate based on ground truth
# x_gt = odom_msg.pose.pose.position.x
# y_gt = odom_msg.pose.pose.position.y
# ros_quat_gt = odom_msg.pose.pose.orientation
# yaw_gt = self.ros_quat_to_yaw(ros_quat_gt) #in radians
# self.initial_estimate.insert(self.indx, gtsam.Pose2(x_gt, y_gt, yaw_gt))
