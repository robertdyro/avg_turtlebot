#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
import numpy as np
from enum import Enum

#path to object lables
PATH_TO_LABELS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../tfmodels/coco_labels.txt')

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# how is nav_cmd being decided -- human manually setting it, or rviz
rviz = rospy.get_param("rviz")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 3

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

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
print "rviz = %s\n" % rviz
print "mapping = %s\n" % mapping

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        # initialize variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.mode = Mode.IDLE
        self.last_mode_printed = None
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
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
        # if using rviz, we can subscribe to nav goal click
        if rviz:
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
	       

	#food detector
	#list of all detectable objects 
	#object_list = load_object_labels(PATH_TO_LABELS)
	#food list has to be updated with a comprehensive list provided by ASL people i think. This is just an example
	food_list = ['orange', 'pizza', 'cake', 'donut', 'fruit', 'salad']
	self.food_location = {}
	for element in food_list:
		rospy.Subscriber('/detector/'+element, DetectedObject, self.food_detected_callback)



    def food_detected_callback(self, msg):
	#this is a food localization part
	#we need to create a seperate function for picking up food
	#print(msg.name+" detected")
	inter_xg = self.x + 0.1*dist*np.cos(msg.thetaleft) #0.1 for dm
	inter_yg = self.y - 0.1*dist*np.sin(msg.thetaright) + 0.2
	inter_thg = self.theta
	self.food_location[msg.name].append(inter_xg, inter_yg, inter_thg) #just in case we have multiple detection 

    

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
	if (self.mode != Mode.POSE): 
		# distance of the stop sign
		dist = msg.distance 
		#print("stop_detected")
		self.inter_xg = self.x + 0.1*dist*np.cos(msg.thetaleft)
		self.inter_yg = self.y - 0.1*dist*np.sin(msg.thetaright) + 0.2
		self.inter_thg = self.theta
		self.init_stop_sign()
		# if close enough and in nav mode, stop
		'''
		if dist > 0 and dist < STOP_MIN_DIST and self.mode == Mode.NAV:
		    self.init_stop_sign()
		'''

    ############ your code starts here ############
    # feel free to change the code here 
    # you may or may not find these functions useful
    # there is no "one answer"


    def go_to_pose(self):
	
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.inter_xg
        pose_g_msg.y = self.inter_yg
        pose_g_msg.theta = self.inter_thg

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """
	print("mode changed to pose")
        #self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.POSE

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
	#print("Hello")
	#print(self.mode)
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """



        #################################################################################
        # Do not change this for hw2 -- this won't affect your FSM since you are using gazebo
        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        #################################################################################

    # YOUR STATE MACHINE
    # Currently it will just go to the pose without stopping at the stop sign

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
	    
            if self.close_to(self.inter_xg,self.inter_yg,self.inter_thg):
		self.stop_sign_start = rospy.get_rostime()
                self.mode = Mode.STOP
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # at a stop sign
            self.stay_idle()
            if self.has_stopped():
                self.init_crossing()

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            self.nav_to_pose()
            if self.has_crossed():
                self.mode = Mode.POSE
                

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.nav_to_pose()
	
	
        
	else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    ############ your code ends here ############

    def run(self):
 	rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
	    #print("hi")
            #rate.sleep()
	    #print("finished sleeping")

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()