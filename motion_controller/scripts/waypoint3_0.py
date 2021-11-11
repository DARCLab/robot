#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import quaternion_from_euler

#----------- Global Variables--------
global desired_pose
desired_pose = PoseStamped()
global robot_pose
robot_pose = PoseStamped()

#---------- Functions --------
def robot_pose_cb(pose_cb_msg):
    global robot_pose
    robot_pose = pose_cb_msg

def main():
    #Initialize node
    rospy.init_node('waypoints')

    #Get parameters
    xd_vec = rospy.get_param("waypoints_x")
    yd_vec = rospy.get_param("waypoints_y")
    quat_vec = rospy.get_param("waypoints_quaternion")
    RobotID = rospy.get_param("RobotID")

    #Set up subscriptions
    firstString = "/mocap_node/Robot_"
    secondString = str(RobotID)
    thirdString = "/pose"
    fullString = firstString + secondString + thirdString
    rospy.Subscriber(fullString, PoseStamped, robot_pose_cb)

    #Set up publishers
    desired_pos_pub = rospy.Publisher("desiredPos",PoseStamped,queue_size = 100)

    #Initialize Variables
    global rate
    rate = rospy.Rate(20)
    global robot_pose
    DesirePos = PoseStamped()

    while not rospy.is_shutdown():
        # print robot_pose
        x = robot_pose.pose.position.x
        y = robot_pose.pose.position.y

        for j in range(len(xd_vec)):
            x = robot_pose.pose.position.x
            y = robot_pose.pose.position.y
            x_error = xd_vec[j] - x
            y_error = yd_vec[j] - y
            norm_error = np.sqrt(x_error**2+ y_error**2)
            while not rospy.is_shutdown() and norm_error > 0.01 :
                x = robot_pose.pose.position.x
                y = robot_pose.pose.position.y
                DesirePos.pose.position.x = xd_vec[j]
                DesirePos.pose.position.y = yd_vec[j]
                DesirePos.pose.orientation.x = quat_vec[0]
                DesirePos.pose.orientation.y = quat_vec[1]
                DesirePos.pose.orientation.z = quat_vec[2]
                DesirePos.pose.orientation.w = quat_vec[3]
                x_error = xd_vec[j] - x
                y_error = yd_vec[j] - y
                norm_error = np.sqrt(x_error**2+ y_error**2)
                desired_pos_pub.publish(DesirePos)
                # print "Desired Position X:", DesirePos.pose.position.x, "Y:", DesirePos.pose.position.y
                # print "Current Position X:", x, "Y:", y
                print(xd_vec[j], yd_vec[j], x, y)
                rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
