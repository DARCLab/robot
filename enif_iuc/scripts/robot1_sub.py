#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

global pose_vals
pose_vals = PoseStamped()

def callback_pose(data):
    global pose_vals
    pose_vals = data

def robot1_pose():
    rospy.init_node("robot1_pose",anonymous = True)
    rospy.Subscriber('/mocap_node/Robot_1/pose',PoseStamped,callback_pose)
   # rospy.spin()
    print("Subscriber node is working")
    while not rospy.is_shutdown():
        print(pose_vals)

if __name__ == '__main__':
    try:
        robot1_pose()
    except rospy.ROSInterruptException:
        pass
        
