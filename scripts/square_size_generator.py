#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import random
from random import randint
from ar_week10_test.msg import square_size_param

def square_size_generator():
    pub = rospy.Publisher('params', square_size_param, queue_size=10)# Create publisher
    rospy.init_node('generator') # Initialise node called generator
    rate = rospy.Rate(0.05) #set rate to sleep for 20 seconds

    while not rospy.is_shutdown():
        l = round(random.uniform(0.05,0.2),6) #set square length to be between 0.05 and 0.2 to 6dp
        rospy.loginfo(l)
        pub.publish(l) #publish 
        rate.sleep()


if __name__ == '__main__':
    try:
        square_size_generator()
    except rospy.ROSInterruptException:
        pass
