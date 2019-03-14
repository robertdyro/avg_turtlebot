#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys

class TerminationRequestPublisher:
    """Publishes food delivery requests to the PavoneCart food delivery service. """

    def __init__(self):
        #initialize node
        rospy.init_node('terminator_publisher', anonymous=True)
        #create publisher
        self.request_termination = rospy.Publisher('/termination_request', String, queue_size=10)
        self.termination_request = None

    def publish_request(self):
        #publish the request t times, once every s seconds
        self.request_termination.publish(self.request)
        self.request_termination2.publish(self.request)
  
    def loop(self):
        """The main loop of the script. The script will ask for food items to add to the 
        delivery_request string until an empty answer ("") is given, at which point it will 
        publish the string. The current request will be published several few times after which the user 
        will be prompted to create a new request."""
        if self.termination_request is None:
			  
            request_not_complete = True
            
            #gather requests from user input
            while request_not_complete:
                termination_decision = raw_input("Shall I be terminated? y/[n]: ")
                if termination_decision == "y":
                    request_not_complete = False
                    self.request = termination_decision
                else:
                    sys.exit()


            print "Termination initiated..."
            self.publish_request()

            #reset delivery request to be ready for new inputs
            self.request = None
            print "\n", "One more try?"

    def run(self):
        print "Create a termination request:"
        print "Enter termination request and press enter to confirm."
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    i_ll_be_back = TerminationRequestPublisher()
    i_ll_be_back.run()
