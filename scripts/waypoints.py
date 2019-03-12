#!/usr/bin/env python
import rospy
import copy
from tf.broadcaster import TransformBroadcaster
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Pose2D, Point
from std_msgs.msg import String, Header, ColorRGBA
import tf
import numpy as np
from numpy import linalg as la
from utils import wrapToPi

#world_name = rospy.get_param("map")
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")



class waypoints:
    def __init__(self):
        rospy.init_node('turtlebot_waypoints', anonymous=True)
        self.waypoint_publisher = rospy.Publisher('/nav_pose', Pose2D, queue_size=10)

        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.trans_listener = tf.TransformListener()
        rospy.Subscriber('/cmd_pose', Pose2D,self.cmd_pose_callback)
        # waypoints for robot in real world if initialized at (0,0,0)
        waypoints = np.array([0.304800000000000,	0,	0,
        0.304800000000000,	0.685800000000000,	-1.57079632679490,
        1.14300000000000,	0.685800000000000,	1.57079632679490,
        1.14300000000000,	-1.60020000000000,	1.57079632679490,
        1.14300000000000,	-0.457200000000000,	0,
        3.20040000000000,	-0.457200000000000,	0,
        3.20040000000000,	0.685800000000000,	-1.57079632679490,
        1.14300000000000,	0.685800000000000,	1.57079632679490,
        1.14300000000000,	-0.457200000000000,	0,
        3.20040000000000,	-0.457200000000000,	0,
        3.12420000000000,	-0.914400000000000,	2.09439510239320,
        0.304800000000000,	-1.67640000000000,	-1.57079632679490])
        # converting waypoint to simulation frame
        waypoints = np.reshape(waypoints,(3,-1),order='F').T
        self.waypoints = waypoints
        # self.waypoints = np.zeros_like(waypoints)
        # self.waypoints[:,1] = 0.6858 + 0.325 - waypoints[:,1]
        # self.waypoints[:,0] = 3.55 - waypoints[:,0]
        # self.waypoints[:,2] = waypoints[:,2]

        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # goal state
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0

    def gazebo_callback(self, data):
        if "turtlebot3_burger" in data.name:
            pose = data.pose[data.name.index("turtlebot3_burger")]
            twist = data.twist[data.name.index("turtlebot3_burger")]
            self.x = pose.position.x
            self.y = pose.position.y
            quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta = euler[2]

    def cmd_pose_callback(self, data):
        self.x_g = data.x
        self.y_g = data.y
        self.theta_g = data.theta


    def get_current_location(self):
        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.rotation = rotation
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def run(self):
        rate = rospy.Rate(.2) # 30 Hz
        rate1 = rospy.Rate(.1)
        explor_flag = True
        K = 0
        waypoint = self.waypoints[K,:]
        pose_waypoint_msg = Pose2D()
        pose_waypoint_msg.x = waypoint[0]
        pose_waypoint_msg.y = waypoint[1]
        pose_waypoint_msg.theta = waypoint[2]
        self.waypoint_publisher.publish(pose_waypoint_msg)
        K = 1
        while explor_flag and not rospy.is_shutdown():
            self.waypoint_publisher.publish(pose_waypoint_msg)
            self.get_current_location()
            waypoint = self.waypoints[K,:]
            if la.norm([self.x-pose_waypoint_msg.x,self.y-pose_waypoint_msg.y])  < 0.05:
                rate1.sleep()
                print('arrived')
                pose_waypoint_msg = Pose2D()
                pose_waypoint_msg.x = waypoint[0]
                pose_waypoint_msg.y = waypoint[1]
                pose_waypoint_msg.theta = waypoint[2]
                self.waypoint_publisher.publish(pose_waypoint_msg)
                rate1.sleep()
                if K == np.shape(self.waypoints)[0]-1:
                    explor_flag = False
                K += 1
            rate.sleep()


if __name__ == '__main__':
    turtlebot_waypoints = waypoints()
    turtlebot_waypoints.run()
