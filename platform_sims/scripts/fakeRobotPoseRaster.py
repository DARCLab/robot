#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from geometry_msgs.msg import PoseStamped

from math import sqrt
import numpy as np
##################
# Global variables
##################

##################
# Functions
##################

def rasterScanGen(xRange, xNumSteps, yRange, yNumSteps):
    xArray = np.linspace(xRange[0], xRange[1], xNumSteps, endpoint=True)
    yArray = np.linspace(yRange[0], yRange[1], yNumSteps, endpoint=True)


    xScan = []
    yScan = []

    for i, yi in enumerate(yArray):
        xScan.append(xArray[::(-1)**i]) # reverse when i is odd
        yScan.append(np.ones_like(xArray) * yi)

    # squeeze lists together to vectors
    xScan = np.concatenate(xScan)
    yScan = np.concatenate(yScan)

    desiredWaypoints = np.array([xScan, yScan]).T

    return desiredWaypoints

##################
# Callbacks
##################

##################
# Main
##################

def main():
    rospy.init_node('fakeRobotPoseRaster')

    DesiredWaypoint = PoseStamped()
    global_waypoint_pub = rospy.Publisher('/mocap_node/Robot_1/pose', PoseStamped, queue_size=1)

    xmin      = rospy.get_param("fakeRobotPoseRaster/xmin")
    xmax      = rospy.get_param("fakeRobotPoseRaster/xmax")
    Nx        = rospy.get_param("fakeRobotPoseRaster/Nx")
    ymin      = rospy.get_param("fakeRobotPoseRaster/ymin")
    ymax      = rospy.get_param("fakeRobotPoseRaster/ymax")
    Ny        = rospy.get_param("fakeRobotPoseRaster/Ny")
    stayTime  = rospy.get_param("fakeRobotPoseRaster/stayTime")
    startTest = rospy.get_param("/startTest")

    waypointIndex = 0
    desiredWaypointsList = rasterScanGen([xmin, xmax], Nx, [ymin, ymax], Ny)

    rate = rospy.Rate(120)

    DesiredWaypoint.pose.position.x = desiredWaypointsList[0,0]
    DesiredWaypoint.pose.position.y = desiredWaypointsList[0,1]

    global_waypoint_pub.publish(DesiredWaypoint);

    while ((not rospy.is_shutdown() and not startTest) ):
        rate.sleep()
        startTest = rospy.get_param("/startTest");
        if startTest:
            break

    waypointStartTime = rospy.get_rostime()
    last_request = rospy.get_rostime()

    while not rospy.is_shutdown():
        if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)):
            waypointIndex +=1
            waypointStartTime = rospy.get_rostime()

        if waypointIndex == len(desiredWaypointsList):
            waypointIndex = 0

        DesiredWaypoint.pose.position.x = desiredWaypointsList[waypointIndex,0]
        DesiredWaypoint.pose.position.y = desiredWaypointsList[waypointIndex,1]
        # z is always 0
        global_waypoint_pub.publish(DesiredWaypoint);

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
