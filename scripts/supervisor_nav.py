#!/usr/bin/env python

import rospy
import os
import json
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from avg_turtlebot.msg import DetectedObject
import tf
import math
import numpy as np
from enum import Enum
import numpy as np

#path to object lables
PATH_TO_LABELS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../tfmodels/coco_labels.txt')

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 3

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = 0.70

# time taken to cross an intersection
CROSSING_TIME = 3

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6


print "supervisor settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "mapping = %s\n" % mapping

class Supervisor:

    def __init__(self):
        self.food_published_last = 0

        rospy.init_node('turtlebot_supervisor_nav', anonymous=True)
        # initialize variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.mode = Mode.IDLE
        self.last_mode_printed = None
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # nav pose for controller
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=2)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # subscribers
        # stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        # if using gazebo, we have access to perfect state
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        # we can subscribe to nav goal click
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

        rospy.loginfo("Supervisor_nav: Initiating food subscribers...")
        #food detector
        self.food_detected_publisher = rospy.Publisher('/food_detected',
            String, queue_size=10)
        #list of all foods to detect
        food_list = ['apple','banana','orange','broccoli','carrot', 'pizza', 'cake', 'donut', 'fruit', 'salad','vegetable']        
        # can't detect spaces in name like hot dog
        # food_list = ['apple','banana','orange','broccoli','carrot','hot dog', 'pizza', 'cake', 'donut', 'fruit', 'salad','vegetable','food-other']
        #list of all foods to detect        
        # food_list = ['apple','banana','orange','broccoli','carrot','hot dog',
        #     'pizza', 'cake', 'donut', 'fruit',
        #     'salad','vegetable','food-other']
        self.food_location = {}
        for element in food_list:
            rospy.Subscriber('/detector/'+element.replace(" ", "_"),
                DetectedObject, self.food_detected_callback)

        rospy.Subscriber('/termination_request', String, self.terminator_callback)
        rospy.loginfo("Supervisor_nav: Done initiating")
       

    def terminator_callback(self, msg):
        if msg.data == "y":
            rospy.loginfo("Supervisor_nav: Termination request received!")
            self.mode = Mode.IDLE
            self.x_g = self.x
            self.y_g = self.y
            self.theta_g = self.theta

            nav_g_msg = Pose2D()
            nav_g_msg.x = self.x
            nav_g_msg.y = self.y
            nav_g_msg.theta = self.theta
            self.nav_goal_publisher.publish(nav_g_msg)

            vel_g_msg = Twist()
            self.cmd_vel_publisher.publish(vel_g_msg)

    def food_detected_callback(self, msg):
        #this is a food localization part
        #we need to create a seperate function for picking up food
        #print(msg.name+" detected")
        rospy.loginfo("Detected: %s", msg.name)

        dist = msg.distance
        food_xg = self.x + 0.1*dist*np.cos(msg.thetaleft) #0.1 for dm
        food_yg = self.y - 0.1*dist*np.sin(msg.thetaright)
        food_thg = self.theta
        food_xg = round(food_xg,2)
        food_yg = round(food_yg,2)
        food_thg = round(food_thg,2)
        if msg.name in self.food_location:
            food_coord = [food_xg, food_yg, food_thg]
            if food_coord in self.food_location[msg.name]:
                rospy.loginfo("Previously detected %s at this position so not adding to dictionary", msg.name)
            else:
                rospy.loginfo("Adding %s location to dictionary", msg.name)
                self.food_location[msg.name].append(food_coord)
        else:
            self.food_location[msg.name] = [[food_xg, food_yg, food_thg]]
        food_detected_dict = String()
        food_detected_dict.data = json.dumps(self.food_location)
        self.food_detected_publisher.publish(food_detected_dict)
        # food_location = {'pizza': [[12.0, 2.0, 3.0], [2.0, 0.0, 1.0]], 'apple': [[1.0, 2.4, 0.3], [1.0, 2.4, 0.3]], 'banana': [[12.0, 2.0, 3.0]]}




    def gazebo_callback(self, msg):
        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        twist = msg.twist[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if mapping else "/odom"
        print("rviz command received!")
        try:

            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.mode = Mode.NAV

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance
        rospy.loginfo("Supervisor: Stop sign at %f", dist)

        # if close enough and in nav mode, stop
        if dist > 0 and dist < STOP_MIN_DIST and self.mode == Mode.NAV:
            self.init_stop_sign()

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g
        #rospy.loginfo("Supervisor_nav: publishing from go_to_pose, mode: %s", self.mode)

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g
        #rospy.loginfo("Supervisor_nav: publishing from nav_to_pose, mode: %s", self.mode)

        self.nav_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        if self.food_published_last % 10 == 0:
          food_detected_dict = String()
          food_detected_dict.data = json.dumps(self.food_location)
          self.food_detected_publisher.publish(food_detected_dict)
        self.food_published_last += 1



        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
                #rospy.loginfo("Current position %s %s %s",self.x, self.y, self.theta)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            self.stay_idle()

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                self.stay_idle()

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                self.mode = Mode.NAV
            else:
                self.nav_to_pose()

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.nav_to_pose()

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
