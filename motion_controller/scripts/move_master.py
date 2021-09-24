#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import Twist
 
#---------------Global Variables------------
global Input_vel
Input_vel = Twist()
#-------------Functions --------------------

def teleop_cb(teleop_msg):
    global Input_vel
    Input_vel = teleop_msg

def main():
    # Initalize node
    rospy.init_node('master_dog')

    # Set up subscriptions
    rospy.Subscriber("/cmd_vel", Twist, teleop_cb)
    # Set up publishers
    local_vel_pub = rospy.Publisher('totalVelocityCMD', Twist, queue_size=100)
    global rate
    rate = rospy.Rate(20)

    totalVel=Twist()

    # Main loop after Initalization
    while not rospy.is_shutdown():
            
            local_vel_pub.publish(Input_vel)
            print(Input_vel)
            
            rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    
