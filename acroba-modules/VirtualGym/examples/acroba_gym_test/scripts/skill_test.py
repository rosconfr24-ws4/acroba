#!/usr/bin/env python3
PKG='acroba_gym'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.
 
import sys
import unittest
import rospy
import time
import math
from geometry_msgs.msg import Pose
import pcl
from acroba_unity_msgs.msg import JointControllers,JointController
from acroba_unity_msgs.msg import UnityErrorCode,UnityLight
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2
import pcl
import cv2
#skills
import virtual_skills.camera as vcamera
import virtual_skills.robot as vrobot
import virtual_skills.gripper as vgripper
from tf.transformations import *

import copy


"""
This test needs to be run after the virtual gym sample is executed

"""
class TestAcrobaSkillCommunication(unittest.TestCase):

    def test_1_robot_control(self):
        rospy.loginfo("Test execute trajectory skill")
        #EXECUTE TRAJECTORY
        #Generate custom trajectory
        req=RobotTrajectory()
        req.joint_trajectory.points=[]
        pt1=JointTrajectoryPoint()
        pt1.positions=[0,-1.57,0,0,0,0,0]
        pt2=JointTrajectoryPoint()
        pt2.positions=[3.14,-1.57,0,0,0,0]
        pt3=JointTrajectoryPoint()
        pt3.positions=[3.14,-1.57,-1.57,0,0,0]
        pt4=JointTrajectoryPoint()
        pt4.positions=[0,-1.57,-1.57,0,0,0]
        pt5=JointTrajectoryPoint()
        pt5.positions=[0,0,0,0,0,0]
        req.joint_trajectory.points.append(pt1)
        req.joint_trajectory.points.append(pt2)
        req.joint_trajectory.points.append(pt3)
        req.joint_trajectory.points.append(pt4)
        req.joint_trajectory.points.append(pt5)
        #Send execution command
        vrobot.RobotDriver(req)

    def test_2_camera_control(self):
        rospy.loginfo("Test camera")
        color, depth, pc = vcamera.RGBDSensorDriver()

        
    def test_3_gripper_control(self):
        rospy.loginfo("Test gripper")
        ret = vgripper.GripperDriver(True);


if __name__ == '__main__':
    import rostest
    rospy.init_node("tests")
    rostest.rosrun(PKG, 'test_unity_test', TestAcrobaSkillCommunication)