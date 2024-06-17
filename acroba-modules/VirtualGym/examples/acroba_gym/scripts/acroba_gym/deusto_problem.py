#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction

rospy.init_node("test_node", anonymous=True)
act_s = actionlib.SimpleActionClient('/unity_robot/follow_joint_trajectory', FollowJointTrajectoryAction)
act_s.wait_for_server()
print("done")