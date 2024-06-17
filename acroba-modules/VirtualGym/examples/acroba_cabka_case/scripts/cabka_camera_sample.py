#!/usr/bin/env python3
import enum
from os import name
from statistics import geometric_mean, mode
import copy
from unittest import case
from click import getchar
import rospy
import math
import sys
from threading import Thread
from geometry_msgs.msg import Vector3,Pose,PoseArray
##acroba msg types
from trajectory_msgs.msg import JointTrajectoryPoint,JointTrajectory
from tf.transformations import euler_from_quaternion, quaternion_from_euler
## common acroba interfaces
from acroba_gym.robot_complete_interface import RobotCompleteControl
from acroba_gym.camera_interface import CameraControl
from acroba_utils.geom_utils import euler2quatRos
from acroba_cabka_case.conveyor_interface import ConveyorInterface
from geometry_msgs.msg import Quaternion,Pose

import cv2
from cv_bridge import CvBridge,CvBridgeError
"""
This sample is to make a demo video of the cabka case interfaces features
focusses on the digital twin features
"""

class CabkaCameraPipeline:
    def __init__(self):
        self.camera_robot_control=RobotCompleteControl(namespace="/cabka_cam")
        self.camera_interface = CameraControl()
        rospy.loginfo("Is connected %i", self.camera_interface.connected)

        rospy.loginfo("It is connected %i",self.camera_robot_control.is_connected)

    def takePhotos(self):
        init_pallet_x=-1.15
        init_pallet_y=-0.04
        init_pallet_z=0.71
        
        orientation=euler2quatRos([3.141592,0,0])
        
        offset_x=0.2
        offset_y=0.1
        offset_z=0.7
        
        pose1=Pose()
        pose1.position.x=init_pallet_x-offset_x
        pose1.position.y=init_pallet_y-offset_y
        pose1.position.z=init_pallet_z+offset_z
        pose1.orientation=orientation
        
        pose2=Pose()
        pose2.position.x=init_pallet_x-offset_x
        pose2.position.y=init_pallet_y+offset_y
        pose2.position.z=init_pallet_z+offset_z
        pose2.orientation=orientation
        
        pose3=Pose()
        pose3.position.x=init_pallet_x+offset_x
        pose3.position.y=init_pallet_y+offset_y
        pose3.position.z=init_pallet_z+offset_z
        pose3.orientation=orientation
        
        pose4=Pose()
        pose4.position.x=init_pallet_x+offset_x
        pose4.position.y=init_pallet_y-offset_y
        pose4.position.z=init_pallet_z+offset_z
        pose4.orientation=orientation
        
        photo_poses=[pose1,pose2,pose3,pose4,pose1]
        
        for idx,photo in enumerate(photo_poses):
            is_ok=self.camera_robot_control.movePose(photo)
            if not is_ok:
                rospy.logerr("No ik ")
                break
            rospy.sleep(2)
            rospy.loginfo("Taking photo %i",idx)
            self.captureAndShow()
        
    
    def captureAndShow(self):
        action_result=self.camera_interface.capture()
        bridge_object=CvBridge()
        cv_rgb_image=bridge_object.imgmsg_to_cv2(action_result.image,desired_encoding="bgra8")
        print("shape of rgb image",cv_rgb_image.shape)
        cv2.imshow("rgb_img",cv_rgb_image)
        cv2.waitKey(1000)
        
if __name__=="__main__":
    rospy.init_node("cabka_camera")
    rospy.sleep(1)
    cabka_pipeline=CabkaCameraPipeline()
    
    cabka_pipeline.takePhotos()