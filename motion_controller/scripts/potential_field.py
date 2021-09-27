#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from math import pi,sin, cos, atan2,exp 

# ------------- Global Variables ------------
global robot_pose
robot_pose = PoseStamped()

global avoid1_pose
avoid1_pose = PoseStamped()

global avoid2_pose
avoid2_pose = PoseStamped()

global avoid3_pose
avoid3_pose = PoseStamped()

global avoid4_pose
avoid4_pose = PoseStamped()

global R_pos_flag
R_pos_flag = False


#-------------- Callback Functions----------
def robot_pose_cb(pose_cb_msg):
    global robot_pose
    robot_pose = pose_cb_msg
    global R_pos_flag
    R_pos_flag = True

def avoid1_pose_cb(avoid1_pose_cb_msg):
    global avoid1_pose
    avoid1_pose = avoid1_pose_cb_msg 

def avoid2_pose_cb(avoid2_pose_cb_msg):
    global avoid2_pose
    avoid2_pose = avoid2_pose_cb_msg

def avoid3_pose_cb(avoid3_pose_cb_msg):
    global avoid3_pose
    avoid3_pose = avoid3_pose_cb_msg

def avoid4_pose_cb(avoid4_pose_cb_msg):
    global avoid4_pose
    avoid4_pose = avoid4_pose_cb_msg


#---------------- Main -------------------
def main():
    # Initalize node
    rospy.init_node('potentialFieldController')

    #Get parameters
    RobotID=rospy.get_param("RobotID")
    subToR1Pose = rospy.get_param("/subToR1Pose")
    subToR2Pose = rospy.get_param("/subToR2Pose")
    subToR3Pose = rospy.get_param("/subToR3Pose")
    subToR4Pose = rospy.get_param("/subToR4Pose")
    subToR5Pose = rospy.get_param("/subToR5Pose")

    # Set up subscriptions
    firstString = "/mocap_node/Robot_"
    secondString = str(RobotID)
    thirdString = "/pose"
    fullString = firstString + secondString + thirdString #this allows each robot to know itself
    rospy.Subscriber(fullString, PoseStamped, robot_pose_cb)

    #Figure out all the obstacles (other robots)
    
    if RobotID == 1:

        if subToR2Pose:
            secondString = '2'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid1_pose_cb)
            avoidR1 = True
        else:
            avoidR1 = False

        if subToR3Pose:
            secondString = '3'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid2_pose_cb)
            avoidR2 = True
        else:
            avoidR2 = False

        if subToR4Pose:
            secondString = '4'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid3_pose_cb)
            avoidR3 = True
        else:
            avoidR3 = False

        if subToR5Pose:
            secondString = '5'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid4_pose_cb)
            avoidR4 = True
        else:
            avoidR4 = False

         
        


    if RobotID == 2:

        if subToR1Pose:
            secondString = '1'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid1_pose_cb)
            avoidR1 = True
        else:
            avoidR1 = False

        if subToR3Pose:
            secondString = '3'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid2_pose_cb)
            avoidR2 = True
        else:
            avoidR2 = False

        if subToR4Pose:
            secondString = '4'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid3_pose_cb)
            avoidR3 = True
        else:
            avoidR3 = False

        if subToR5Pose:
            secondString = '5'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid4_pose_cb)
            avoidR4 = True
        else:
            avoidR4 = False

    if RobotID == 3:

        if subToR1Pose:
            secondString = '1'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid1_pose_cb)
            avoidR1 = True
        else:
            avoidR1 = False

        if subToR2Pose:
            secondString = '2'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid2_pose_cb)
            avoidR2 = True
        else:
            avoidR2 = False

        if subToR4Pose:
            secondString = '4'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid3_pose_cb)
            avoidR3 = True
        else:
            avoidR3 = False

        if subToR5Pose:
            secondString = '5'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid4_pose_cb)
            avoidR4 = True
        else:
            avoidR4 = False

    if RobotID == 4:

        if subToR1Pose:
            secondString = '1'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid1_pose_cb)
            avoidR1 = True
        else:
            avoidR1 = False

        if subToR2Pose:
            secondString = '2'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid2_pose_cb)
            avoidR2 = True
        else:
            avoidR2 = False

        if subToR3Pose:
            secondString = '3'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid3_pose_cb)
            avoidR3 = True
        else:
            avoidR3 = False

        if subToR5Pose:
            secondString = '5'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid4_pose_cb)
            avoidR4 = True
        else:
            avoidR4 = False

    if RobotID == 5:

        if subToR1Pose:
            secondString = '1'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid1_pose_cb)
            avoidR1 = True
        else:
            avoidR1 = False

        if subToR2Pose:
            secondString = '2'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid2_pose_cb)
            avoidR2 = True
        else:
            avoidR2 = False

        if subToR3Pose:
            secondString = '3'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid3_pose_cb)
            avoidR3 = True
        else:
            avoidR3 = False

        if subToR4Pose:
            secondString = '4'
            fullString = firstString + secondString + thirdString #this allows each robot to know itself
            rospy.Subscriber(fullString, PoseStamped, avoid4_pose_cb)
            avoidR4 = True
        else:
            avoidR4 = False


    # Set up publishers
    pot_vel_pub = rospy.Publisher('potFieldVelocityCMD', Twist, queue_size=100)

    global rate
    rate = rospy.Rate(20)

    Input_pot_Vel = Twist()
    Input_pot_Vel.linear.x = 0
    Input_pot_Vel.linear.y = 0
    Input_pot_Vel.linear.z = 0
    Input_pot_Vel.angular.z = 0
   
    # Main loop after Initalization
    while not (R_pos_flag):
        rate.sleep()
    while not rospy.is_shutdown():
            
            #This robots position
            x = robot_pose.pose.position.x
            y = robot_pose.pose.position.y
            (roll, pitch, yaw) = euler_from_quaternion([robot_pose.pose.orientation.x,robot_pose.pose.orientation.y,robot_pose.pose.orientation.z,robot_pose.pose.orientation.w])
            

            #Avoiding first obstacle
            global s
            global r
            s = 0.3
            r = 0.2
            
            potential_x1 = 0
            potential_y1 = 0
            if avoidR1:
                x_obs1 = avoid1_pose.pose.position.x
                y_obs1 = avoid1_pose.pose.position.y

                x_error_obs1 = x_obs1-x
                y_error_obs1 = y_obs1-y

                d_obs1 = np.sqrt(x_error_obs1**2+ y_error_obs1**2)
                theta1=atan2(y_error_obs1,x_error_obs1)

                if d_obs1 > s:
                    B1 = 0.0
                elif (d_obs1< r) :
                    B1 = 2000.00
                else:
                    B1 = 900.00

                potential_x1 = -B1 *(s+r -  d_obs1)*cos(theta1)
                potential_y1 = -B1 *(s+r -  d_obs1) *sin(theta1)
            
            potential_x2 = 0
            potential_y2 = 0
            if avoidR2:
                x_obs2 = avoid2_pose.pose.position.x
                y_obs2 = avoid2_pose.pose.position.y

                x_error_obs2 = x_obs2-x
                y_error_obs2 = y_obs2-y

                d_obs2 = np.sqrt(x_error_obs2**2+ y_error_obs2**2)
                theta2 =atan2(y_error_obs2,x_error_obs2)

                if d_obs2 > s:
                    B1 = 0.0
                elif (d_obs2< r) :
                    B1 = 2000.00
                else:
                    B1 = 900.00

                potential_x2 = -B1 *(s+r -  d_obs2)*cos(theta2)
                potential_y2 = -B1 *(s+r -  d_obs2) *sin(theta2)
            
            potential_x3 = 0
            potential_y3 = 0
            if avoidR3:
                x_obs3 = avoid3_pose.pose.position.x
                y_obs3 = avoid3_pose.pose.position.y

                x_error_obs3 = x_obs3-x
                y_error_obs3 = y_obs3-y

                d_obs3 = np.sqrt(x_error_obs3**2+ y_error_obs3**2)
                theta3=atan2(y_error_obs3,x_error_obs3)

                if d_obs3 > s:
                    B1 = 0.0
                elif (d_obs3< r) :
                    B1 = 2000.00
                else:
                    B1 = 900.00

                potential_x3 = -B1 *(s+r -  d_obs3)*cos(theta3)
                potential_y3 = -B1 *(s+r -  d_obs3) *sin(theta3)
            
            potential_x4 = 0
            potential_y4 = 0
            if avoidR4:
                x_obs4 = avoid4_pose.pose.position.x
                y_obs4 = avoid4_pose.pose.position.y

                x_error_obs4 = x_obs4-x
                y_error_obs4 = y_obs4-y

                d_obs4 = np.sqrt(x_error_obs4**2+ y_error_obs4**2)
                theta4=atan2(y_error_obs4,x_error_obs4)

                if d_obs4 > s:
                    B1 = 0.0
                elif (d_obs4< r) :
                    B1 = 2000.00
                else:
                    B1 = 900.00

                potential_x4 = -B1 *(s+r -  d_obs4)*cos(theta4)
                potential_y4 = -B1 *(s+r -  d_obs4) *sin(theta4)


            xVelg = potential_x1 + potential_x2 + potential_x3 +potential_x4
            yVelg = potential_y1 + potential_y2 + potential_y3 + potential_y4

            Input_pot_Vel.linear.x = cos(yaw) * xVelg + sin(yaw) * yVelg
            Input_pot_Vel.linear.y = -sin(yaw) * xVelg + cos(yaw) * yVelg

            pot_vel_pub.publish(Input_pot_Vel)
            
            print "Input Potential Field Velocities X: ",Input_pot_Vel.linear.x , "Y: " , Input_pot_Vel.linear.y
            rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    
