#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion
from math import pi 
import numpy as np

#---------------Global Variables------------
global robot_pose
robot_pose = PoseStamped()

#-------------Functions --------------------

def robot_pose_cb(pose_cb_msg):
    global robot_pose
    robot_pose = pose_cb_msg

def main():
    
    yawd = 0
    # Initalize node
    rospy.init_node('Master')
    RobotID=rospy.get_param("RobotID")
    xd = rospy.get_param("xd")
    yd = rospy.get_param("yd")
    # Set up subscriptions
    firstString = "/mocap_node/Robot_"
    secondString = str(RobotID)
    thirdString = "/pose"
    fullString = firstString + secondString + thirdString #this allows each robot to know itself
    rospy.Subscriber(fullString, PoseStamped, robot_pose_cb)
    # Set up publishers
    error_vec = Float32MultiArray()
    error_pub = rospy.Publisher('Error', Float32MultiArray, queue_size=100)
    #errors[0] = 0.0 #error in x
    #errors[1] = 0.0 #error in y
    #errors[2]=0.0 #error in angle
    global rate
    rate = rospy.Rate(20)

    # Main loop after Initalization
    while not rospy.is_shutdown():

            x = robot_pose.pose.position.x
            y = robot_pose.pose.position.y
            (roll, pitch, yaw) = euler_from_quaternion([robot_pose.pose.orientation.x,robot_pose.pose.orientation.y,robot_pose.pose.orientation.z,robot_pose.pose.orientation.w])
            
            x_error = xd - x
            y_error = yd - y
            yaw_error = yawd - yaw

            error_vec.data = [x_error, y_error, yaw_error]
            print "Error X: ", x_error, "Y: " ,y_error , "yaw: " , yaw_error*(180/pi)
            error_pub.publish(error_vec)
            rate.sleep()
            

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    
