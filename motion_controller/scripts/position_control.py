#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from math import pi
import numpy as np


#---------------Global Variables------------
global robot_pose
robot_pose = PoseStamped()
global radius
global desired_pose
desired_pose = PoseStamped()

#-------------Functions --------------------
def capFunc(currentValue, newMin, newMax):
    if(currentValue > newMax):
       currentValue = newMax
    if currentValue < newMin:
       currentValue = newMin
    return currentValue
def robot_pose_cb(pose_cb_msg):
    global robot_pose
    robot_pose = pose_cb_msg

def desired_pose_cb(dpose_cb_msg):
    global desired_pose
    desired_pose = dpose_cb_msg

def main():
    # Initalize node
    rospy.init_node('positionController')
    
    #Get parameters
    RobotID=rospy.get_param("RobotID")
    yawd = rospy.get_param("yawd")
    
    # Set up subscriptions
    firstString = "/mocap_node/Robot_"
    secondString = str(RobotID)
    thirdString = "/pose"
    fullString = firstString + secondString + thirdString #this allows each robot to know itself
    rospy.Subscriber(fullString, PoseStamped, robot_pose_cb)
    rospy.Subscriber("desiredPos",PoseStamped, desired_pose_cb)

    # Set up publishers
    pos_vel_pub = rospy.Publisher('posVelocityCMD', Twist, queue_size=100)
   
    #Initialize Variables
    global rate
    rate = rospy.Rate(20)
    
    Input_pos_Vel = Twist()
    Input_pos_Vel.linear.x = 0
    Input_pos_Vel.linear.y = 0
    Input_pos_Vel.linear.z = 0
    Input_pos_Vel.angular.z = 0
   

    x_errori = 0
    x_error_int = 0
    y_error_int = 0
    y_errori = 0
    yaw_errori = 0
    Kd = 500
    Kp = 350
    Kp_x = 4000
    Kd_x = 4000

    # Main loop after Initalization
    while not rospy.is_shutdown():

           x = robot_pose.pose.position.x
           y = robot_pose.pose.position.y
           (roll, pitch, yaw) = euler_from_quaternion([robot_pose.pose.orientation.x,robot_pose.pose.orientation.y,robot_pose.pose.orientation.z,robot_pose.pose.orientation.w])
           xd = desired_pose.pose.position.x
           yd = desired_pose.pose.position.y
           x_error = xd - x
           y_error = yd - y
           yaw_error = yawd - yaw
        
           Input_pos_Vel.angular.z =(Kp*(yaw_error)+ Kd * (yaw_error - yaw_errori)) * 1.5

           if Input_pos_Vel.angular.z >= 460:
               Input_pos_Vel.angular.z = 460
           elif (Input_pos_Vel.angular.z <= -460):
               Input_pos_Vel.angular.z = -460


           if abs((yaw*(180/pi))) < 10:
                radius =np.sqrt(x_error**2+ y_error**2)
                if radius < .25:
                    Ki = 0.1
                else:
                    Ki = 0
                 
                xVel = ((Kp_x*(x_error)+ Kd_x * (x_error - x_errori)+(Ki*x_error_int)))
                yVel = ((Kp_x*(y_error)+ Kd_x * (y_error - y_errori))+(Ki*y_error_int))
                
                if radius != 0:
                    xVelCap = abs(x_error/radius) * 300
                    yVelCap = abs(y_error/radius) * 300

                    Input_pos_Vel.linear.x=capFunc(xVel,-xVelCap,xVelCap)
                    Input_pos_Vel.linear.y=capFunc(yVel,-yVelCap,yVelCap)
                else:
                    Input_pos_Vel.linear.x=0
                    Input_pos_Vel.linear.y=0
           
           pos_vel_pub.publish(Input_pos_Vel)
            
           x_errori = x_error
           x_error_int = x_error+ x_error_int
           y_errori = y_error
           y_error_int = y_error + y_error_int
           yaw_errori = yaw_error
           print "Radius:", radius
           print "x:",x, "y:",y,"yaw_angle" , yaw*(180/pi)
           print "Desired Position X:", xd, "Y:",yd
           rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    
