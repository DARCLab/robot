#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped
from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, acos, sqrt, exp

##################
# Functions
##################

def gaussFunc(xFunc, yFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    con = (QFunc/(4 * pi * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * ((yFunc**2)/DyFunc + (zFunc**2)/DzFunc))
    return con * 1000 # convert from kg/m^3 to ppm

def getReading(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    # Rotate frame

    Stheta = sin(thetaFunc)
    Ctheta = cos(thetaFunc)

    xRobotRotated = (Ctheta  * xRobotDef + Stheta * yRobotDef + -Ctheta * xPlumeFunc - Stheta * yPlumeFunc)
    yRobotRotated = (-Stheta * xRobotDef + Ctheta * yRobotDef +  Stheta * xPlumeFunc - Ctheta * yPlumeFunc)

    if xRobotRotated <= 0:
        reading = 0
    else:
        reading = gaussFunc(xRobotRotated,yRobotRotated,zFunc,QFunc,vFunc,DyFunc,DzFunc)

    return reading

##################
# Global variables
##################

global Robot1_pose
Robot1_pose = PoseStamped()

global Robot1_particles
Robot1_particles = particles()

##################
# Callbacks
##################

def pose_cb(pose_cb_msg):
    global Robot1_pose
    Robot1_pose = pose_cb_msg

def Robot1_particles_cb(Robot1_particles_msg):
    global Robot1_particles
    Robot1_particles = Robot1_particles_msg

##################
# Main function
##################

def main():

    rospy.init_node('plotGuassian')
    rate = rospy.Rate(30)


    # Map parameters
    minLimX = 0
    minLimY = 0
    maxLimX = 14 / 3.28084
    maxLimY = 9 / 3.28084

    maxPPMplot = 15

    # Plume parameters
    QPlume        = rospy.get_param("/QPlume")            #  kg/s     release rate
    vPlume        = rospy.get_param("/vPlume")       #  m/s      velocity
    DyPlume       = rospy.get_param("/DyPlume")      #  m        diffusion along y
    DzPlume       = rospy.get_param("/DzPlume")      #  m        diffusion along z

    # Plume orientation translation then rotation
    xPlume     = rospy.get_param("/xPlume")       #  m
    yPlume     = rospy.get_param("/yPlume")       #  m
    thetaPlume = rospy.get_param("/thetaPlume")   #  degrees

    # xPlume = xPlume * 3.28084
    # yPlume = yPlume * 3.28084

    # Set up subscriptions
    rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, pose_cb)
    rospy.Subscriber("Robot_1/particles", particles, Robot1_particles_cb)

    xPltRobot1 = []
    yPltRobot1 = []

    thetaPlume = thetaPlume * (pi/180) # convert to radians
    xPlumePlot = np.arange(minLimX, maxLimX+1, 0.01)
    yPlumePlot = np.arange(minLimY, maxLimY+1, 0.01)

    conArray = np.zeros([len(yPlumePlot), len(xPlumePlot)])

    for xCurrentIndex in range(len(xPlumePlot)):
        for yCurrentIndex in range(len(yPlumePlot)):
            conArray[yCurrentIndex,xCurrentIndex] = getReading(xPlumePlot[xCurrentIndex], yPlumePlot[yCurrentIndex], thetaPlume, xPlume, yPlume, 0.0, QPlume, vPlume, DyPlume, DzPlume)

    conArray[conArray > maxPPMplot] = maxPPMplot

    while not rospy.is_shutdown():

        # Convert to feet (for plotting)
        # Robot1_poseXft = Robot1_pose.pose.position.x * 3.28084
        # Robot1_poseYft = Robot1_pose.pose.position.y * 3.28084
        Robot1_poseXft = Robot1_pose.pose.position.x
        Robot1_poseYft = Robot1_pose.pose.position.y

        if Robot1_poseXft != 0.0 and Robot1_poseYft != 0.0:
            xPltRobot1.append(Robot1_poseXft)
            yPltRobot1.append(Robot1_poseYft)

        # print("Robot 1 reading : ", getReading(Robot1_poseXft, Robot1_poseYft, thetaPlume, xPlume, yPlume, 0.0, QPlume, vPlume, DyPlume, DzPlume))

        # Plotting stuff
        plt.clf()

        # Plot gaussian
        plt.contour(xPlumePlot,yPlumePlot,conArray,25)
        plt.plot(xPltRobot1, yPltRobot1,"r")
        plt.plot(Robot1_poseXft, Robot1_poseYft,'ro',  markersize=6)
        plt.plot(Robot1_particles.X,Robot1_particles.Y,'k.')

        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.grid()
        plt.xlim(minLimX, maxLimX)
        plt.ylim(minLimY, maxLimY)
        plt.pause(0.01)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
