#!/usr/bin/env python
import rospy
import copy

from visualization_msgs.msg import Marker
from tf.broadcaster import TransformBroadcaster
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Pose2D, Point
from std_msgs.msg import Float32MultiArray, String, Header, ColorRGBA
import tf
import numpy as np
from numpy import linalg
from utils import wrapToPi

#world_name = rospy.get_param("map")
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")



class RViz_marker:
    def __init__(self):
        rospy.init_node('turtlebot_rviz_marker', anonymous=True)
        self.pub = rospy.Publisher('/turtlebot_rviz_marker', Marker, queue_size=10)

        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.trans_listener = tf.TransformListener()
#        self.tfBuffer = tf2_ros.Buffer()
#        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.Subscriber('/cmd_pose', Pose2D,self.cmd_pose_callback)

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


    def get_marker_location(self):
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
        rate = rospy.Rate(60) # 30 Hz
        while not rospy.is_shutdown():
            self.get_marker_location()
            marker = Marker(
                type=Marker.CYLINDER,
                header=Header(frame_id="/map"),
                color=ColorRGBA(0.0, 0.0, 0.0, 1.0),
                action=Marker.ADD,
                ns='turtlebot3_loc',
                 id=0)
            marker.pose.position = Point(self.x, self.y, 0)
            marker.pose.orientation.w = 1.0
            marker.scale.x = .2
            marker.scale.y = .2
            marker.scale.z = .1
            marker.header.stamp = rospy.Time.now()
            marker.lifetime = rospy.Duration()
            self.pub.publish(marker)

            marker_goal = Marker(
                type=Marker.CUBE,
                header=Header(frame_id="/map"),
                color=ColorRGBA(1.0, 0.0, 1.0, 1.0),
                action=Marker.ADD,
                ns='turtlebot3_goal',
                 id=1)
            marker_goal.pose.position = Point(self.x_g, self.y_g, 0)
            marker_goal.pose.orientation.w = 1.0
            marker_goal.scale.x = .2
            marker_goal.scale.y = .2
            marker_goal.scale.z = .1
            marker_goal.header.stamp = rospy.Time.now()
            marker_goal.lifetime = rospy.Duration()
            self.pub.publish(marker_goal)

            rate.sleep()


if __name__ == '__main__':
    #rospy.init_node('turtlebot_rviz_marker', anonymous=True)
    #marker_pub = rospy.Publisher('/turtlebot_rviz_marker', Marker, queue_size=10)
    turtlebot_marker = RViz_marker()
    turtlebot_marker.run()
