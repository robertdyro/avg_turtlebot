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



class FoodFetcher:
    def __init__(self):
        rospy.init_node('turtlebot_foodfetcher', anonymous=True)
        self.food_waypoint_publisher = rospy.Publisher('/nav_pose', Pose2D, queue_size=10)

        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.trans_listener = tf.TransformListener()
        rospy.Subscriber('/cmd_pose', Pose2D,self.cmd_pose_callback)
        rospy.Subscriber('/delivery_request', String ,self.delivery_request_callback)
        rospy.Subscriber('/food_detected', String ,self.food_detected_callback)
        # waypoints for robot in real world if initialized at (0,0,0)
        
        self.food_dictionary = {}
        self.food_waypoints = []


        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # goal state
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0

    def delivery_request_callback(self, msg):
        allFoodsRequested = msg.data.split(',')
        print("All the foods requested are ",allFoodsRequested)
        if not self.food_dictionary:
            print("Dictionary is empty")
            return
        else:
            for fd in allFoodsRequested:
                if fd in self.food_dictionary:
                    locOfFd = self.food_dictionary[fd][0]
                    self.food_waypoints.append(locOfFd)
                else:
                    print("Did not detect the requested item ",fd, ":(")
            print("Requested food are at coordinates ",self.food_waypoints)
        return

    def food_detected_callback(self, msg):
        self.food_dictionary = eval(msg.data)
        print("Dictionary of food just detected is ",self.food_dictionary)
        return

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
        fetch_flag = True
        K = 0
        food_waypoint = self.food_waypoints[K,:]
        pose_waypoint_msg = Pose2D()
        pose_waypoint_msg.x = food_waypoint[0]
        pose_waypoint_msg.y = food_waypoint[1]
        pose_waypoint_msg.theta = food_waypoint[2]
        self.food_waypoint_publisher.publish(pose_waypoint_msg)
        K = 1
        while not explor_flag and not rospy.is_shutdown():
            self.food_waypoint_publisher.publish(pose_waypoint_msg)
            self.get_current_location()
            food_waypoint = self.food_waypoints[K,:]
            if la.norm([self.x-pose_waypoint_msg.x,self.y-pose_waypoint_msg.y])  < 0.05:
                rate1.sleep()
                print('arrived at food waypoint')
                pose_waypoint_msg = Pose2D()
                pose_waypoint_msg.x = food_waypoint[0]
                pose_waypoint_msg.y = food_waypoint[1]
                pose_waypoint_msg.theta = food_waypoint[2]
                self.food_waypoint_publisher.publish(pose_waypoint_msg)
                rate1.sleep()
                if K == np.shape(self.food_waypoints)[0]-1:
                    fetch_flag = False
                K += 1
            rate.sleep()


if __name__ == '__main__':
    turtlebot_foodfetcher = FoodFetcher()
    turtlebot_foodfetcher.run()
