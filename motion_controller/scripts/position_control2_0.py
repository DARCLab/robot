#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, cos, sin, atan2,fmod
import numpy as np


#---------------Global Variables------------

global robot_pose
robot_pose = PoseStamped()
global radius
global desired_pose
desired_pose = PoseStamped()
global pose_flag
global R_pose_flag
R_pose_flag = False
pose_flag = False
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
    global R_pose_flag
    R_pose_flag = True
def desired_pose_cb(dpose_cb_msg):
    global desired_pose
    desired_pose = dpose_cb_msg
    global pose_flag
    pose_flag = True
def getDifference(b1,b2):
    r = fmod(b2-b1, 2*pi)
    if (r < -pi):
        r+=2*pi
    if(r>= pi):
        r-= 2*pi
    return r 
    
# ----------------Main -------------------

def main():
    # Initalize node
    rospy.init_node('positionController')
    
    #Get parameters
    RobotID=rospy.get_param("RobotID")
    
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
    #Kd = 500
    #Kp = 300
    Kd = 250
    Kp = 250
    Kp_x = 5000
    #Kd_x = 4000
    Kd_x = 300

    # Main loop after Initalization
    while not pose_flag :
        rate.sleep()
    while not rospy.is_shutdown():

           x = robot_pose.pose.position.x
           y = robot_pose.pose.position.y
           
           (roll, pitch, yaw) = euler_from_quaternion([robot_pose.pose.orientation.x,robot_pose.pose.orientation.y,robot_pose.pose.orientation.z,robot_pose.pose.orientation.w])
           (rolld, pitchd, yawd) = euler_from_quaternion([desired_pose.pose.orientation.x,desired_pose.pose.orientation.y,desired_pose.pose.orientation.z,desired_pose.pose.orientation.w])
           
           xd = desired_pose.pose.position.x
           yd = desired_pose.pose.position.y

           x_error = xd - x
           y_error = yd - y
           yaw_error = yawd - yaw

           yaw_error = -getDifference(yawd,yaw)
           #if yaw_error < 0
           Input_pos_Vel.angular.z =(Kp*(yaw_error)+ Kd * (yaw_error - yaw_errori)) * 1.5
           
           if Input_pos_Vel.angular.z >= 460:
               Input_pos_Vel.angular.z = 460
           elif (Input_pos_Vel.angular.z <= -460):
               Input_pos_Vel.angular.z = -460


           #if abs((yaw*(180/pi))) < 10:
           radius =np.sqrt(x_error**2+ y_error**2)
           if radius < 0.25:
               Ki = 0.1
           else:
               Ki =0

        
           #xerror_r =  cos(yaw) * x_error + sin(yaw) * y_error
           #yerror_r = -sin(yaw) * x_error + cos(yaw) * y_error

           #x_error = xerror_r
           #y_error = yerror_r
           
           xVelg = ((Kp_x*(x_error)+ Kd_x * (x_error - x_errori)+(Ki*x_error_int)))
           yVelg = ((Kp_x*(y_error)+ Kd_x * (y_error - y_errori))+(Ki*y_error_int))
           
           #Input_pos_Vel.linear.x = xVelr
           #Input_pos_Vel.linear.y = yVelr
           #xVelr =  cos(yaw) * xVelg + sin(yaw) * yVelg
           #yVelr = -sin(yaw) * xVelg + cos(yaw) * yVelg

           if radius != 0:
               xVelCap = abs(x_error/radius) * 300
               yVelCap = abs(y_error/radius) * 300

               xVelg=capFunc(xVelg,-xVelCap,xVelCap)
               yVelg=capFunc(yVelg,-yVelCap,yVelCap)
             
               xVelr =  cos(yaw) * xVelg + sin(yaw) * yVelg
               yVelr = -sin(yaw) * xVelg + cos(yaw) * yVelg
  
               Input_pos_Vel.linear.x=xVelr
               Input_pos_Vel.linear.y=yVelr
               
           else:
               Input_pos_Vel.linear.x=0
               Input_pos_Vel.linear.y=0
           
           pos_vel_pub.publish(Input_pos_Vel)
            
           x_errori = x_error
           x_error_int = x_error+ x_error_int
           y_errori = y_error
           y_error_int = y_error + y_error_int
           yaw_errori = yaw_error
           #print "Radius:", radius
           #print ("x:",x, "y:",y,"yaw_angle" , yaw, xd, yd, yawd, x_error, y_error, yaw_error)
           #print "Desired Position X:", xd, "Y:",yd
           rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    
