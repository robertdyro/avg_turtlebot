#!/usr/bin/env python
import rospy
import copy
import json
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
        
        # self.food_dictionary = {'pizza': [[12.0, 2.0, 3.0], [2.0, 0.0, 1.0]], 'apple': [[1.0, 2.4, 0.3], [1.0, 2.4, 0.3]], 'banana': [[12.0, 2.0, 3.0]]}
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

        rospy.Subscriber('/termination_request', String, self.terminator_callback)

    def terminator_callback(self, msg):
        if msg.data == "y":
            rospy.loginfo("Food fetcher: termination request received!")
            if self.food_waypoints:
              self.food_waypoints.remove(self.food_waypoints[0])

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
        

    def food_detected_callback(self, msg):
        #print("data_received", msg.data)
        self.food_dictionary.update(json.loads(msg.data))
        #print("Dictionary of food just detected is ",self.food_dictionary)
        

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
        rate = rospy.Rate(2) # 30 Hz
        fetch_flag = True
        """
        while not fetch_flag or not self.food_waypoints:
            pass
        food_waypoint = self.food_waypoints[K]
        pose_waypoint_msg = Pose2D()
        pose_waypoint_msg.x = food_waypoint[0]
        pose_waypoint_msg.y = food_waypoint[1]
        pose_waypoint_msg.theta = food_waypoint[2]
        print "publishing waypoint 0"
        self.food_waypoint_publisher.publish(pose_waypoint_msg)
        print "publishing waypoint 0"
        K = 1
        while not fetch_flag and not rospy.is_shutdown():
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
         """

        while not rospy.is_shutdown():
            if fetch_flag == True and self.food_waypoints: 
                fd_target = self.food_waypoints[0]
                pose_waypoint_msg = Pose2D()
                pose_waypoint_msg.x = fd_target[0]
                pose_waypoint_msg.y = fd_target[1]
                pose_waypoint_msg.theta = fd_target[2]
                rospy.loginfo("Publishing waypoint")
                self.food_waypoint_publisher.publish(pose_waypoint_msg)

                self.get_current_location()

                fd_x = fd_target[0]
                fd_y = fd_target[1]
                fd_theta = fd_target[2]

                if la.norm([self.x - fd_x, self.y - fd_y]) < 0.10:
                    rospy.loginfo('Arrived at food waypoint')
                    rospy.loginfo("Publishing new waypoint")
                    self.food_waypoints.remove(fd_target)
            rate.sleep()


if __name__ == '__main__':
    turtlebot_foodfetcher = FoodFetcher()
    turtlebot_foodfetcher.run()
