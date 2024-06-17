#!/usr/bin/env python3
import enum
from os import name
from statistics import geometric_mean, mode
import copy
from unittest import case
from click import getchar
import rospy
import copy
import math
import sys
from threading import Thread
from geometry_msgs.msg import Vector3,Pose,PoseArray
##acroba msg types
from trajectory_msgs.msg import JointTrajectoryPoint,JointTrajectory
from tf.transformations import euler_from_quaternion, quaternion_from_euler
## common acroba interfaces
from acroba_gym.robot_complete_interface import RobotCompleteControl
from acroba_gym.gripper_interface import ParallelYawControl
from acroba_gym.unity_scene_manager import UnitySceneManager
from acroba_gym.camera_interface import CameraControl
from acroba_utils.geom_utils import euler2quatRos
from geometry_msgs.msg import Quaternion,Pose,Point
from acroba_utils.geom_utils import euler2quatRos

#particular interface
from acroba_steripack_case.specialtool_interface import SpecialToolInterface

import cv2
from cv_bridge import CvBridge,CvBridgeError
"""
This sample is to make a demo video of the cabka case interfaces features
focusses on the digital twin features
"""

class SteriPipeline2:
    def __init__(self):
        self.robot_control_grippper=RobotCompleteControl()
        self.robot_control_drill=RobotCompleteControl(end_name="end_drill")
        self.gripper_control=ParallelYawControl()
        self.special_tool_interface=SpecialToolInterface()
        self.unity_scene_manager=UnitySceneManager()
        self.camera_interface = CameraControl()
        rospy.loginfo("Is connected %i", self.camera_interface.connected)
        
        #self.camera_interface = CameraControl()

        rospy.loginfo("Robot gripper is connected %i",self.robot_control_grippper.is_connected)
        rospy.loginfo("Robot drill  is connected %i",self.robot_control_drill.is_connected)
        self.vertical_orientation=Quaternion()
        self.vertical_orientation=euler2quatRos([0.0,3.141592,0.0])
        self.tray_position=Point()
        self.tray_position.x=0
        self.tray_position.y=0.534
        self.tray_position.z=0.7834
        self.time=2
        self.current_tool=0
        
    def set_drill(self):
        self.special_tool_interface.change_tool(0)
        self.current_tool=0
        
    def drill(self):
        self.special_tool_interface.drill(True)
        rospy.sleep(2)
        self.special_tool_interface.drill(False)
        
        
    def set_gripper(self):
        self.special_tool_interface.change_tool(1)
        self.current_tool=1
        
    def close(self):
        self.gripper_control.move([1,1],1,1)
        rospy.sleep(1)
        
    def open(self):
        self.gripper_control.move([-1,-1],1)
        rospy.sleep(1)
        
        
    def move_pose(self,pose:Pose,time=2):
        
        if self.current_tool==0:
            self.robot_control_drill.movePose(pose,time)
        elif self.current_tool==1:
            self.robot_control_grippper.movePose(pose,time)
        rospy.sleep(time)

    def remove_supports(self):
        
        offset_x=0.035
        offset_y=0.07
        offset_z=0.03
        
        offset_z_prepick=0.05

        predrill_pose=Pose()
        
        self.model_dict=copy.deepcopy(self.unity_scene_manager.get_all_models_dict())
        for name,pose in self.model_dict.items():
            
            predrill_pose.position=copy.deepcopy(self.tray_position)
            predrill_pose.orientation=copy.deepcopy(self.vertical_orientation)

            predrill_pose.position.z=predrill_pose.position.z+offset_z+offset_z_prepick
        
        
            predrill_pose.position.x=pose.position.x
            predrill_pose.position.y=pose.position.y
        
            self.move_pose(predrill_pose,self.time)
        
            drill_pose=copy.deepcopy(predrill_pose)
            drill_pose.position.z=drill_pose.position.z-offset_z_prepick
            
            self.move_pose(drill_pose,self.time)
            self.drill()
            self.move_pose(predrill_pose,self.time)
    
    def change_tool(self):
        ## go to griper change station
        curr_joints=self.robot_control_drill.get_current_joints()
        curr_joints[0]=0
        curr_joints[1]=-1.57
        curr_joints[2]=1.57
        self.robot_control_drill.move_joints(curr_joints,self.time)
        rospy.sleep(self.time)
        
        curr_joints[0]=1.57
        self.robot_control_drill.move_joints(curr_joints,self.time)
        rospy.sleep(self.time)
        
        self.set_gripper()
        rospy.sleep(1)
        
        curr_joints[0]=0
        self.robot_control_drill.move_joints(curr_joints,self.time)
        rospy.sleep(self.time)
        
    def pick_pieces(self):
        self.model_dict=copy.deepcopy(self.unity_scene_manager.get_all_models_dict())
        
        pre_pick_offset=0.08
        pick_offset_z=0.001
        
        for key in self.model_dict:
            rospy.sleep(1)
            aux_dict=copy.deepcopy(self.unity_scene_manager.get_all_models_dict())
            aux_pose=Pose()
            aux_pose=aux_dict[key]
            prepick_pose=Pose()
            prepick_pose.position=copy.deepcopy(aux_pose.position)
            prepick_pose.orientation=copy.deepcopy(self.vertical_orientation)
            
            prepick_pose.position.z=prepick_pose.position.z+pre_pick_offset+pick_offset_z
            self.move_pose(prepick_pose,self.time)
            
            pick_pose=copy.deepcopy(prepick_pose)
            pick_pose.position.z=pick_pose.position.z-pre_pick_offset
            self.move_pose(pick_pose,self.time)
            
            self.close()
            rospy.sleep(1)
            
            self.move_pose(prepick_pose,self.time)
            
            place_pose=copy.deepcopy(prepick_pose)
            place_pose.position.x=place_pose.position.x-0.3
            place_pose.position.y=place_pose.position.y-0.3
            self.move_pose(place_pose)
            self.open()
        
    def captureAndShow(self):
        
        action_result=self.camera_interface.capture()
        bridge_object=CvBridge()
        cv_rgb_image=bridge_object.imgmsg_to_cv2(action_result.image,desired_encoding="bgra8")
        cv_rgb_image=cv2.rotate(cv_rgb_image,cv2.ROTATE_180)
        print("shape of rgb image",cv_rgb_image.shape)
        cv2.imshow("rgb_image",cv_rgb_image)
        cv2.waitKey(2000)
        
                
        if action_result.depth_map.width>0:
            cv_depth_image=bridge_object.imgmsg_to_cv2(action_result.depth_map,desired_encoding="32FC1")
            cv_depth_image=cv2.rotate(cv_depth_image,cv2.ROTATE_180)
            print("depth shape",cv_depth_image.shape)
            cv2.imshow("depth_img",cv_depth_image)
            cv2.waitKey(2000)
        cv2.destroyAllWindows()
            
            
if __name__=="__main__":
    
    rospy.init_node("steripack_2")
    rospy.sleep(1)
    steri_pipeline=SteriPipeline2()
    rospy.sleep(1)
    print("set drill")
    steri_pipeline.set_drill()
    
    steri_pipeline.captureAndShow()
    
    steri_pipeline.remove_supports()
    rospy.sleep(1)
    print("Set gripper")
    steri_pipeline.change_tool()
    
    steri_pipeline.captureAndShow()
    #Check tool state
    steri_pipeline.pick_pieces()
    
    