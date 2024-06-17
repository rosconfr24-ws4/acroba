#!/usr/bin/env python3
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

class CabkaCasePipeline:
    def __init__(self):
        namespace="/cabka_drill"
        self.conveyor_interface=ConveyorInterface()
        self.drill_robot_control=RobotCompleteControl(namespace=namespace)
        self.camera_robot_control=RobotCompleteControl(namespace="/cabka_cam")
        self.camera_interface = CameraControl()
        self.trajectory_received=False
        self.trajectory_executed=False
        rospy.loginfo("Robots are connected %i %i",self.drill_robot_control.is_connected,self.camera_robot_control.is_connected)
        rospy.loginfo("camera is connected %i", self.camera_interface.connected)
        self.drill_poses=[]
        
        
    def moveWaitConveyor(self):
        rospy.loginfo("Send move command")
        self.conveyor_interface.move(True)
        arrived=self.conveyor_interface.waitTimeout(10)
        return arrived
        
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
        
    def createCartesianDrillPoints(self):
        
        #this should be the pallet position
        pallet_x=2.0025
        pallet_y=-0.045
        pallet_z=0.71+0.132
        
        #-x and +y orientation
        #roty 3.14
        quad=euler2quatRos([0,3.141592,0])
        
        x1=0.17#0.1736
        x2=0.072#0.074
        y1=-0.145#0.15
        y2=-0.225#0.23
        
        pose1=Pose()
        pose1.position.x=pallet_x+x1
        pose1.position.y=pallet_y+y1
        pose1.position.z=pallet_z
        pose1.orientation=quad
        
        pose2=Pose()
        pose2.position.x=pallet_x+x1
        pose2.position.y=pallet_y+y2
        pose2.position.z=pallet_z
        pose2.orientation=quad
        
        pose3=Pose()
        pose3.position.x=pallet_x+x2
        pose3.position.y=pallet_y+y2
        pose3.position.z=pallet_z
        pose3.orientation=quad
        
        pose4=Pose()
        pose4.position.x=pallet_x+x2
        pose4.position.y=pallet_y+y1
        pose4.position.z=pallet_z
        pose4.orientation=quad
        
        
        self.drill_poses=[pose1,pose2,pose3,pose4,pose1]
        
        self.pose_up=Pose()
        self.pose_up.position.x=pallet_x+(x1+x2)*0.5
        self.pose_up.position.y=pallet_y+(y1+y2)*0.5
        self.pose_up.position.z=pallet_z+0.15
        self.pose_up.orientation=quad
        
    def drill(self):
        
        trayectory_msgs=JointTrajectory()
        seed=self.drill_robot_control.get_current_joints()
        seed
        
        pose_target=self.drill_robot_control.get_current_pose()
        #self.pose_up.position=pose_target.position
        print("Pose up",self.pose_up)
        #self.pose_up.orientation=pose_target.orientation
        ik_sol=self.drill_robot_control.poseToJoint(seed,self.pose_up)
        
        if ik_sol is None:
            rospy.logerr("pose up ik fail")
            return 
        # Append up point
        pt=JointTrajectoryPoint()
        pt.positions=copy.deepcopy(ik_sol)
        pt.time_from_start=rospy.Duration(3)
        trayectory_msgs.points.append(pt)
        
        #Append drilling points
        for idx,drill_pose in enumerate(self.drill_poses):
            seed=ik_sol
            ik_sol=self.drill_robot_control.poseToJoint(seed,drill_pose)
            
            if ik_sol is None:
                rospy.logwarn("Drill pose %i ik failed",idx)
                continue
            
            pt_drill=JointTrajectoryPoint()
            pt_drill.positions=copy.deepcopy(ik_sol)
            pt_drill.time_from_start=trayectory_msgs.points[-1].time_from_start+rospy.Duration(2)
            trayectory_msgs.points.append(pt_drill)
        #Append up point
        pt_end=copy.deepcopy(pt)
        pt_end.time_from_start=trayectory_msgs.points[-1].time_from_start+rospy.Duration(3)
        trayectory_msgs.points.append(pt_end)
        
        rospy.loginfo("Trajectory messsage of length %i",len(trayectory_msgs.points))
        rospy.loginfo("Send execution command")
        self.drill_robot_control.execute_trajectory_topic(trayectory_msgs)
            
            
    
    
if __name__=="__main__":
    rospy.init_node("cabka_pipeline")
    cabka_pipeline=CabkaCasePipeline()
    
    cabka_pipeline.takePhotos()
    
    is_ok=cabka_pipeline.moveWaitConveyor()
    if not is_ok:
        rospy.logerr("Conveyor not arrived")
        sys.exit(0)
    
    
    
    pose=cabka_pipeline.drill_robot_control.get_current_pose()
    print("Current robot pose",pose)
    cabka_pipeline.createCartesianDrillPoints()
    cabka_pipeline.drill()
        