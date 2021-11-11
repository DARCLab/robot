#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from math import pi, cos, sin, atan2,fmod,asin,sqrt
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

def quat_product(p,q):
    quat=[(p[0]*q[0]-p[1]*q[1]-p[2]*q[2]-p[3]*q[3]),p[0]*q[1]+p[1]*q[0]+p[2]*q[3]-p[3]*q[2],p[0]*q[2]-p[1]*q[3]+p[2]*q[0]+p[3]*q[1],p[0]*q[3]+p[1]*q[2]-p[2]*q[1]+p[3]*q[0]]
    return np.array(quat)    

def quat_euler(q):
    roll = atan2(2*(q[0]*q[1]+q[2]*q[3]),q[0]**2 -q[1]**2-q[2]**2+q[3]**2)
    pitch = asin(2*(q[0]*q[2]-q[3]*q[1])) 
    yaw = atan2(2*(q[0]*q[3]+q[1]*q[2]),q[0]**2 +q[1]**2-q[2]**2-q[3]**2)
    return roll,pitch,yaw
def unit_quat(q):
    norm = sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
    unq = np.array(q)/norm
    return unq 
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
    rolli=0
    pitchi = 0
    yawi =0
    timei=rospy.get_time()

    #Kd = 500
    #Kp = 300
    Kd = 250
    Kp = 250
    Kp_x = 5000
    #Kd_x = 4000
    Kd_x = 300
    Pq = 300
    Pw = 100

    # Main loop after Initalization
    while not pose_flag :
        rate.sleep()
    while not rospy.is_shutdown():

           x = robot_pose.pose.position.x
           y = robot_pose.pose.position.y
           
           xd = desired_pose.pose.position.x
           yd = desired_pose.pose.position.y

           q_des = [desired_pose.pose.orientation.w,desired_pose.pose.orientation.x,desired_pose.pose.orientation.y,desired_pose.pose.orientation.z]
           
           q_m= [robot_pose.pose.orientation.w,robot_pose.pose.orientation.x,robot_pose.pose.orientation.y,robot_pose.pose.orientation.z]           
           
          # if q_m[0] < 0:
           #    q_m[1] = - q_m[1]
            #   q_m[2] = -q_m[2]
             #  q_m[3] = -q_m[3]
              # q_m[0] = - q_m[0]

           q_m_con = [q_m[0],-q_m[1],-q_m[2],-q_m[3]]
           x_error = xd - x
           y_error = yd - y
           #Quaternion Controller

           q_error = quat_product(unit_quat(q_des),unit_quat(q_m_con))
           
           
           print("Desired Quaternion:",q_des)
           print("Measured Quaternion:",q_m)
           print("Quaternion Error:" , q_error)
           
           #if q_error[0] < 0:
            #  q_error[1] = -q_error[1]
             # q_error[2] = -q_error[2]
             # q_error[3] = -q_error[3]

           #Finding Quaternion derivative 
           (roll,pitch,yaw)=quat_euler(q_m) #current roll,pitch yaw
           (roll_error,pitch_error,yaw_error)=quat_euler(q_error)
           print("yaw:",yaw,"yaw error:",yaw_error)
           time = rospy.get_time()
           omega_x = (getDifference(roll,rolli))/(time - timei) #Velocity around x
           omega_y = (getDifference(pitch,pitchi))/(time - timei) #Velocity around y
           omega_z = (getDifference(yaw,yawi))/(time - timei) #Velocity around z
           omega_quat = [0,0,0,omega_z] # quaternion omega
           q_w = quat_product(q_error,omega_quat)
          # print("omega quaternion:",q_w)
           q_d_error = (0.5)*q_w
          # print("Velocity Prop:", -Pq*q_error[3],"Velocity Deriv:", -Pw*q_d_error[3])
           vel = -Pq*q_error[3]- Pw*q_d_error[3]
           Input_pos_Vel.angular.z = vel
           print("Input Vel:",Input_pos_Vel.angular.z)
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
             
              # xVelr =  cos(yaw) * xVelg + sin(yaw) * yVelg
              # yVelr = -sin(yaw) * xVelg + cos(yaw) * yVelg
               xVelr = 0
               yVelr = 0
  
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
           rolli = roll
           pitchi = pitch
           yawi =yaw
           timei = time
           #print "Radius:", radius
           #print ("x:",x, "y:",y,"yaw_angle" , yaw, xd, yd, yawd, x_error, y_error, yaw_error)
           #print "Desired Position X:", xd, "Y:",yd
           rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    
