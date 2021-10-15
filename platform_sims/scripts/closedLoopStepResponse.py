#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent

import numpy as np
import matplotlib.pyplot as plt
import pickle
import math

from geometry_msgs.msg import PoseStamped
# from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

##################
# Functions
##################

##################
# Global variables
##################

global Robot1_pose

Robot1_pose = PoseStamped()

##################
# Callbacks
##################

def robot_1_pose_cb(pose_cb_msg):
    global Robot1_pose
    Robot1_pose = pose_cb_msg

##################
# Main function
##################

def main():

    plotResults = False
    rospy.init_node('stepReponseXY')

    rate = rospy.Rate(120)

    startTest  = rospy.get_param("/startTest")      #  m        diffusion along z

    # Set up subscriptions
    rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, robot_1_pose_cb)
    desired_pos_pub = rospy.Publisher('desiredPos', PoseStamped, queue_size=100)

    startTestFlag = False

    Step = 2 # [m] global coordinates
    yawStep = 1.5708 # 30 degrees
    testTime = 5.0
    timeBeforeTest = 0.5

    desiredPos = PoseStamped()
    desiredPos.pose.position.x  = 1
    desiredPos.pose.position.y  = 1

    x, y, z, w = quaternion_from_euler(0, 0, 0)
    desiredPos.pose.orientation.x = x
    desiredPos.pose.orientation.y = y
    desiredPos.pose.orientation.z = z
    desiredPos.pose.orientation.w = w
    

    # Go home little one
    desired_pos_pub.publish(desiredPos)

    # Wait for test to start
    while not rospy.get_param("/startTest"): # waits until the quad is armed before starting particle filtering
        rospy.sleep(1)
        desired_pos_pub.publish(desiredPos)
        # print(rospy.get_param("/startTest"), "   ,   ", sensorMsg_cb_flag)
        if rospy.get_param("/startTest"):
            break

    robotPoseListX = []
    robotPoseListY = []
    robotPoseListyaw = []

    timeListX = []
    timeListY = []
    timeListyaw = []

    stepListX = []
    stepListY = []
    stepListyaw = []

    rospy.sleep(1)
    
    # X Step
    startTime = rospy.get_rostime()
    while ((rospy.get_rostime() - startTime) < rospy.Duration.from_sec(timeBeforeTest)):

        desiredPos.pose.position.x  = 1
        desiredPos.pose.position.y  = 1
        x, y, z, w = quaternion_from_euler(0, 0, 0)
        desiredPos.pose.orientation.x = x
        desiredPos.pose.orientation.y = y
        desiredPos.pose.orientation.z = z
        desiredPos.pose.orientation.w = w
        startTestFlag = False

        seconds = rospy.get_time()

        if Robot1_pose.pose.position.x != 0.0 and Robot1_pose.pose.position.y != 0.0:
            robotPoseListX.append(Robot1_pose.pose.position.x)
            timeListX.append(seconds)
            stepListX.append(0)

        desired_pos_pub.publish(desiredPos)

    startTime = rospy.get_rostime()
    testStartX = startTime.to_sec()

    while ((rospy.get_rostime() - startTime) < rospy.Duration.from_sec(testTime)):

        desiredPos.pose.position.x  = Step
        desiredPos.pose.position.y  = 1
        x, y, z, w = quaternion_from_euler(0, 0, 0)
        desiredPos.pose.orientation.x = x
        desiredPos.pose.orientation.y = y
        desiredPos.pose.orientation.z = z
        desiredPos.pose.orientation.w = w
        startTestFlag = False

        seconds = rospy.get_time()

        if Robot1_pose.pose.position.x != 0.0 and Robot1_pose.pose.position.y != 0.0:
            robotPoseListX.append(Robot1_pose.pose.position.x)
            timeListX.append(seconds)
            stepListX.append(Step)

        desired_pos_pub.publish(desiredPos)


    desiredPos.pose.position.x  = 1
    desiredPos.pose.position.y  = 1
    x, y, z, w = quaternion_from_euler(0, 0, 0)
    desiredPos.pose.orientation.x = x
    desiredPos.pose.orientation.y = y
    desiredPos.pose.orientation.z = z
    desiredPos.pose.orientation.w = w

    # Go home little one
    desired_pos_pub.publish(desiredPos)

    rospy.sleep(10)

    # Y Step
    startTime = rospy.get_rostime()
    while ((rospy.get_rostime() - startTime) < rospy.Duration.from_sec(timeBeforeTest)):

        desiredPos.pose.position.x  = 1
        desiredPos.pose.position.y  = 1
        x, y, z, w = quaternion_from_euler(0, 0, 0)
        desiredPos.pose.orientation.x = x
        desiredPos.pose.orientation.y = y
        desiredPos.pose.orientation.z = z
        desiredPos.pose.orientation.w = w
        startTestFlag = False

        seconds = rospy.get_time()

        if Robot1_pose.pose.position.x != 0.0 and Robot1_pose.pose.position.y != 0.0:
            robotPoseListY.append(Robot1_pose.pose.position.y)
            timeListY.append(seconds)
            stepListY.append(0)

        desired_pos_pub.publish(desiredPos)

    startTime = rospy.get_rostime()
    testStartY = startTime.to_sec()

    while ((rospy.get_rostime() - startTime) < rospy.Duration.from_sec(testTime)):

        desiredPos.pose.position.x  = 1
        desiredPos.pose.position.y  = Step
        x, y, z, w = quaternion_from_euler(0, 0, 0)
        desiredPos.pose.orientation.x = x
        desiredPos.pose.orientation.y = y
        desiredPos.pose.orientation.z = z
        desiredPos.pose.orientation.w = w
        startTestFlag = False

        seconds = rospy.get_time()

        if Robot1_pose.pose.position.x != 0.0 and Robot1_pose.pose.position.y != 0.0:
            robotPoseListY.append(Robot1_pose.pose.position.y)
            timeListY.append(seconds)
            stepListY.append(Step)

        desired_pos_pub.publish(desiredPos)


    desiredPos.pose.position.x  = 1
    desiredPos.pose.position.y  = 1
    x, y, z, w = quaternion_from_euler(0, 0, 0)
    desiredPos.pose.orientation.x = x
    desiredPos.pose.orientation.y = y
    desiredPos.pose.orientation.z = z
    desiredPos.pose.orientation.w = w

    # Go home little one
    desired_pos_pub.publish(desiredPos)

    rospy.sleep(10)

    # Yaw Step
    startTime = rospy.get_rostime()
    while ((rospy.get_rostime() - startTime) < rospy.Duration.from_sec(timeBeforeTest)):

        desiredPos.pose.position.x  = 1
        desiredPos.pose.position.y  = 1
        x, y, z, w = quaternion_from_euler(0, 0, 0)
        desiredPos.pose.orientation.x = x
        desiredPos.pose.orientation.y = y
        desiredPos.pose.orientation.z = z
        desiredPos.pose.orientation.w = w
        startTestFlag = False

        seconds = rospy.get_time()

        if Robot1_pose.pose.position.x != 0.0 and Robot1_pose.pose.position.y != 0.0:
            roll, pitch, yaw =euler_from_quaternion([Robot1_pose.pose.orientation.x,Robot1_pose.pose.orientation.y,Robot1_pose.pose.orientation.z,Robot1_pose.pose.orientation.w])
            robotPoseListyaw.append(yaw)
            timeListyaw.append(seconds)
            stepListyaw.append(0)

        desired_pos_pub.publish(desiredPos)

    startTime = rospy.get_rostime()
    testStartyaw = startTime.to_sec()

    while ((rospy.get_rostime() - startTime) < rospy.Duration.from_sec(testTime)):

        desiredPos.pose.position.x  = 1
        desiredPos.pose.position.y  = 1
        x, y, z, w = quaternion_from_euler(0, 0, yawStep)
        desiredPos.pose.orientation.x = x
        desiredPos.pose.orientation.y = y
        desiredPos.pose.orientation.z = z
        desiredPos.pose.orientation.w = w
        startTestFlag = False

        seconds = rospy.get_time()

        if Robot1_pose.pose.position.x != 0.0 and Robot1_pose.pose.position.y != 0.0:
            roll, pitch, yaw =euler_from_quaternion([Robot1_pose.pose.orientation.x,Robot1_pose.pose.orientation.y,Robot1_pose.pose.orientation.z,Robot1_pose.pose.orientation.w])
            robotPoseListyaw.append(yaw)
            timeListyaw.append(seconds)
            stepListyaw.append(yawStep)

        desired_pos_pub.publish(desiredPos)

    rospy.sleep(1)

    desiredPos.pose.position.x  = 1
    desiredPos.pose.position.y  = 1
    x, y, z, w = quaternion_from_euler(0, 0, 0)
    desiredPos.pose.orientation.x = x
    desiredPos.pose.orientation.y = y
    desiredPos.pose.orientation.z = z
    desiredPos.pose.orientation.w = w

    # Go home little one
    desired_pos_pub.publish(desiredPos)

    robotPoseListX   = np.array(robotPoseListX)
    robotPoseListY   = np.array(robotPoseListY)
    robotPoseListyaw = np.array(robotPoseListyaw)

    timeListX       = np.array(timeListX)
    timeListY       = np.array(timeListY)
    timeListyaw     = np.array(timeListyaw)

    robotPoseListX   = robotPoseListX   - robotPoseListX[0]
    robotPoseListY   = robotPoseListY   - robotPoseListY[0]
    robotPoseListyaw = robotPoseListyaw - robotPoseListyaw[0]

    #robotPoseListyaw[robotPoseListyaw<-2]+=360

    timeListX       = (timeListX       - timeListX[0])
    timeListY       = (timeListY       - timeListY[0])
    timeListyaw     = (timeListyaw     - timeListyaw[0])

    testStartX   = testStartX - timeListX[0]
    testStartY   = testStartY - timeListY[0]
    testStartyaw = testStartyaw - timeListyaw[0]

    db = {}
    db["timeListX"]   = timeListX
    db["timeListY"]   = timeListY
    db["timeListyaw"] = timeListyaw

    db["stepListX"]   = np.array(stepListX)
    db["stepListY"]   = np.array(stepListY)
    db["stepListyaw"] = np.array(stepListyaw)

    db["testStartX"] = testStartX
    db["testStartY"] = testStartY
    db["testStartyaw"] = testStartyaw

    db["robotPoseListX"] = robotPoseListX
    db["robotPoseListY"] = robotPoseListY
    db["robotPoseListyaw"] = robotPoseListyaw

    pickle.dump(db, open("closedLoopStepResponse.txt", "wb"))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
