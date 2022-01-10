"""
Loosly couples wheel odometry and pose estimates from SLAM
"""

import rospy
# import slam_ros_utils as utl
#import "slam_ros_utils.py" as utl
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseStamped #, Twist, PoseArray, PoseStamped
import cv2
import numpy as np
import gtsam


br = CvBridge()

class Slam(object):
    def __init__(self):

        #Init image variables
        self.br = CvBridge()
        self.image = None
        self.depth = None

        self.initial_estimate = gtsam.Values()  
        self.factor_graph = gtsam.NonlinearFactorGraph()

        self.pose_index = 1

        #prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
        self.factor_graph.add(gtsam.PriorFactorPose2(self.pose_index, gtsam.Pose2(0.0, 0.0, 0.0), prior_noise))
        self.initial_estimate.insert(self.pose_index, gtsam.Pose2(0.05, 0.0, 0.02)) #0.5, 0.0, 0.2


        #Init ROS publishers and subscribers
        rospy.Subscriber("wheel_odom", Odometry, self.wheel_odom_cb)
        rospy.Subscriber("kinect/rgb/image_raw",Image, self.img_cb)
        rospy.Subscriber("kinect/depth/image_raw", Image, self.depth_cb)
        self.pose_pub = rospy.Publisher('pose_estm', PoseStamped, queue_size=1)

        #Publish initial estimate
        pose_msg = self.toPoseStamped(gtsam.Pose2(0.0, 0.0, 0.0), "odom", rospy.Time.now())
        print("publising first pose!")
        print(pose_msg)
        self.pose_pub.publish(pose_msg)

        #self.result = None
        self.slam_estm = None


    def wheel_odom_cb(self, odom_msg):
        self.pose_index += 1

        #ROS convertion
        cov = odom_msg.pose.covariance # 1x36 vector representing 6x6 matrix
        
        #TODO - do i need to use the twist messages????

        #GROUND TRUTH
        x_gt = odom_msg.pose.pose.position.x
        y_gt = odom_msg.pose.pose.position.y
        ros_quat_gt = odom_msg.pose.pose.orientation
        yaw_gt = self.ros_quat_to_yaw(ros_quat_gt) #in radians

        x = odom_msg.twist.twist.linear.x
        y = odom_msg.twist.twist.linear.y
        yaw = odom_msg.twist.twist.angular.z
        # yaw = self.ros_quat_to_yaw(ros_quat) #in radians



        # print("x: ", x, "y: ", y, "yaw: ", yaw)
        # print("covariance: ", cov[0], cov[7], cov[35])

        #Add to factor graph
        odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([cov[0], cov[7], cov[35]]))
        self.factor_graph.add(gtsam.BetweenFactorPose2(self.pose_index-1, self.pose_index, gtsam.Pose2(x/50, y/50, yaw/50), odometry_noise))
        self.initial_estimate.insert(self.pose_index, gtsam.Pose2(x_gt+0.1, y_gt-0.2, yaw_gt-0.01))

        #print(self.factor_graph)


        if (self.pose_index % 10 == 0):
            print("Solving the factor graph...")
            self.solve_factor_graph()

            print(self.slam_estm)

            #Publish slam estimate as odometry message
            pose_msg = self.toPoseStamped(self.slam_estm, "odom", odom_msg.header.stamp)
            self.pose_pub.publish(pose_msg)
        # else:
        #     print(self.factor_graph)
        #Temp
        #self.pose_estm.publish(odom_msg)

    def img_cb(self, img_msg):
            # rospy.loginfo('Image received...')
            self.image = self.br.imgmsg_to_cv2(img_msg)
            self.show_image(self.image, "Image Window")

    def depth_cb(self, img_msg):
            # rospy.loginfo('Depth received...')
            self.depth = self.br.imgmsg_to_cv2(img_msg)
            # self.show_image(self.depth, "Depth Window")

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

    def toPoseStamped(self,gtsamPose2, frame_id, time):
        pos = PoseStamped()

        pos.header.stamp = time #rospy.Time.now()
        pos.header.frame_id = frame_id
        pos.pose = self.toPose(gtsamPose2.x(),gtsamPose2.y(),gtsamPose2.theta())

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

    def solve_factor_graph(self):
        # Create (deliberately inaccurate) initial estimate        
        # print(self.initial_estimate)
        # print(self.factor_graph)


        # Create an optimizer.

        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.factor_graph, self.initial_estimate, params)

        # Solve the MAP problem.
        result = optimizer.optimize()

        # print("RESULTS: ", result)

        self.slam_estm = result.atPose2(self.pose_index)
        # print("a: ", type(self.result.atPose2(1)))
        # print("b", dir(self.result.atPose2(1)))
        # print("c", self.result.atPose2(249).x())
        # print("c", self.result.atPose2(249).y())
        # print("c", self.result.atPose2(249).theta())

        #print("dir: ", dir(self.result))
        #print(self.result.size())


        # Calculate marginal covariances for all variables.
        #marginals = gtsam.Marginals(self.pose_graph, result)
        

if __name__ == '__main__':
    rospy.init_node('simple_slam')
    Slam()
    rospy.spin()