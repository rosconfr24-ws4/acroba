#!/usr/bin/env python3
from os import name
import copy
import rospy
import copy
from geometry_msgs.msg import Vector3,Pose,PoseArray
##acroba msg types
## common acroba interfaces
from acroba_gym.robot_complete_interface import RobotCompleteControl
from acroba_gym.gripper_interface import ParallelYawControl
from acroba_utils.geom_utils import euler2quatRos
from geometry_msgs.msg import Quaternion,Pose,Point
from acroba_utils.geom_utils import euler2quatRos

#particular interface
from acroba_steripack_case.printer_interface import PrinterInterface
from acroba_steripack_case.oven_interface import OvenInterface

import cv2
from cv_bridge import CvBridge,CvBridgeError
"""
This sample is to make a demo video of the cabka case interfaces features
focusses on the digital twin features
"""

class SteriPipeline:
    def __init__(self):
        self.robot_control=RobotCompleteControl()
        self.gripper_control=ParallelYawControl()
        self.printer_interface=PrinterInterface()
        self.oven_interface=OvenInterface()
        #self.camera_interface = CameraControl()

        rospy.loginfo("It is connected %i",self.robot_control.is_connected)

        self.printer_or=Quaternion()
        self.printer_or=euler2quatRos([1.5707,0,1.5707])
        
        self.tray_start_pose=Point()
        self.tray_start_pose.z=1.122-0.05
        self.tray_start_pose.x=-0.262
        self.tray_start_pose.y=0.539
        
        self.print_pose=Point()
        self.print_pose.x=0
        self.print_pose.y=0
        self.print_pose.z=1.115-0.01
        
        offset_x=-0.1
        pose_printer=Pose()
        pose_printer.position=copy.deepcopy(self.print_pose)
        pose_printer.orientation=copy.deepcopy(self.printer_or)
        pose_printer.position.x=pose_printer.position.x+2*offset_x
        self.preprint_pose=copy.deepcopy(pose_printer)
        
        pose_printer.position.x=pose_printer.position.x-2*offset_x
        self.print_pose_robot=copy.deepcopy(pose_printer)
        
        #Washer
        self.tray_start_pose=Point()
        self.tray_start_pose.z=1.122-0.05
        self.tray_start_pose.x=-0.262
        self.tray_start_pose.y=0.539
        #Washer
        self.washer_pose=Pose()
        self.washer_pose.orientation=Quaternion()
        self.washer_pose.orientation=euler2quatRos([1.5707,0,0])
        
        self.washer_pose.position=Point()
        self.washer_pose.position.y=-1.212
        self.washer_pose.position.z=1.08+0.08
        self.washer_pose.position.x=-1.2+0.14
        
        self.pre_wash=copy.deepcopy(self.washer_pose)
        self.pre_wash.position.y=self.pre_wash.position.y+0.2
        
        #Oven
        self.oven_pose=Pose()
        self.oven_pose.position.x=-2.079+0.16
        self.oven_pose.position.y=0
        self.oven_pose.position.z=1.309+0.05
        
        self.oven_pose.orientation=euler2quatRos([1.5707,0,-1.5707])
        
        offset_x=0.5
        self.pre_oven=copy.deepcopy(self.oven_pose)
        self.pre_oven.position.x=self.pre_oven.position.x+offset_x
    
        
    def close(self):
        self.gripper_control.move([1,1],1,1)
        rospy.sleep(1)
        
    def open(self):
        self.gripper_control.move([-1,-1],1)
        rospy.sleep(1)
        
    def pick_tray(self):
        self.gripper_control.open()
        pose1=Pose()
        pose1.position=copy.deepcopy(self.tray_start_pose)
        pose1.position.x=pose1.position.x-0.3
        pose1.orientation=self.printer_or
        time=2
        print("move prepick")
        self.robot_control.movePose(pose1,time)
        rospy.sleep(2)
        self.open()
        rospy.sleep(1)
        pose1.position=copy.deepcopy(self.tray_start_pose)
        pose1.position.x=pose1.position.x+0.01
        pose1.position.z=pose1.position.z+0.05
        self.robot_control.movePose(pose1,time)
        print("move pick")
        rospy.sleep(2)
        self.close()
        rospy.loginfo("grasping")
        rospy.sleep(1)
        """
        pose1.position.z=pose1.position.z+0.1
        self.robot_control.movePose(pose1,time)
        print("move up")
        """
        
    def go_to_printer(self):
        
        
        time=2
        rospy.loginfo("move preplace")
        
       
        self.robot_control.movePose(self.preprint_pose,time)
        rospy.sleep(2)
        
        self.robot_control.movePose(self.print_pose_robot)
        rospy.sleep(3)
        self.open()
        rospy.loginfo("opening gripper")
        
        rospy.loginfo("move postplace")
        
        self.robot_control.movePose(self.preprint_pose,time)
        rospy.sleep(2)
        
    def print_action(self):
        self.printer_interface.print_action()
        is_ok=self.printer_interface.waitTimeout(10)
        if is_ok:
            print("printed successfull")
        
    def pick_printed_tray(self):
        
        time=2
        low_pre=copy.deepcopy(self.print_pose_robot)
        low_pre.position.x=self.print_pose_robot.position.x+0.01
        low_pre.position.z=self.print_pose_robot.position.z-0.05
        self.robot_control.movePose(low_pre,time)
        rospy.sleep(time)
        
        high=copy.deepcopy(self.print_pose_robot)
        #high.position.x=self.print_pose_robot.position.x+0.01
        high.position.z=self.print_pose_robot.position.z+0.05
        self.robot_control.movePose(high,time)
        rospy.sleep(time)
        
        rospy.sleep(1)
        self.close()
        rospy.sleep(1)
        
        after_pick=copy.deepcopy(self.preprint_pose)
        after_pick.position.z=after_pick.position.z+0.05
        self.robot_control.movePose(after_pick,time)
        rospy.sleep(time)
        
    def go_to_washer(self):
      
        time=2
        
        #move prewash
        self.robot_control.movePose(self.pre_wash,time)
        rospy.sleep(time)
        
        #move pose
        self.robot_control.movePose(self.washer_pose,time)
        rospy.sleep(time)
        
        #Open gripper
        self.open()
        rospy.sleep(1)
        
        #move prewash
        self.robot_control.movePose(self.pre_wash,time)
        rospy.sleep(time)
        
    def pick_washed_tray(self):
        time=2
        offset_z=0.01
        low_pre_pick=copy.deepcopy(self.pre_wash)
        low_pre_pick.position.z=low_pre_pick.position.z-offset_z
        self.robot_control.movePose(low_pre_pick)
        rospy.sleep(time)
        
        low_pick=copy.deepcopy(self.washer_pose)
        low_pick.position.z=low_pick.position.z-offset_z
        self.robot_control.movePose(low_pick)
        rospy.sleep(time)
        
        #move pose
        high_pick=copy.deepcopy(self.washer_pose)
        high_pick.position.z=high_pick.position.z+offset_z
        self.robot_control.movePose(high_pick,time)
        rospy.sleep(time)
        
        #Close gripper
        self.close()
        rospy.sleep(1)
        
        #move prewash
        self.robot_control.movePose(self.pre_wash,time)
        rospy.sleep(time)
        
    def go_to_oven(self):
        time=2
       
        self.robot_control.movePose(self.pre_oven,time)
        rospy.sleep(time)
        #low_pre_pick=copy.deepcopy(self.pre_wash)
        #low_pre_pick.position.z=low_pre_pick.position.z-offset_z
        self.robot_control.movePose(self.oven_pose)
        rospy.sleep(time)
        
        self.open()
        
        self.robot_control.movePose(self.pre_oven,time)
        rospy.sleep(time)
        
    def oven(self):
        self.oven_interface.on()
        rospy.sleep(5)
        self.oven_interface.off()
        rospy.sleep(2)
        
    def pick_ovenned_tray(self):
        time=2
        offset_z=0.02
        low_pre_pick=copy.deepcopy(self.pre_oven)
        low_pre_pick.position.z=low_pre_pick.position.z-offset_z
        self.robot_control.movePose(low_pre_pick)
        rospy.sleep(time)
        
        low_pick=copy.deepcopy(self.oven_pose)
        low_pick.position.z=low_pick.position.z-offset_z
        self.robot_control.movePose(low_pick)
        rospy.sleep(time)
        
        #move pose
        high_pick=copy.deepcopy(self.oven_pose)
        high_pick.position.z=high_pick.position.z+offset_z
        self.robot_control.movePose(high_pick,time)
        rospy.sleep(time)
        
        #Close gripper
        self.close()
        rospy.sleep(1)
        
        #move prewash
        self.robot_control.movePose(self.pre_oven,time)
        rospy.sleep(time)
        
        
        
if __name__=="__main__":
    
    rospy.init_node("cabka_camera")
    rospy.sleep(1)
    steri_pipeline=SteriPipeline()
    
    steri_pipeline.pick_tray()
    steri_pipeline.go_to_printer()
    steri_pipeline.print_action()
    #print("Press to continue")
    #getchar()
    
    steri_pipeline.pick_printed_tray()
    print("go to washer")
    steri_pipeline.go_to_washer()
    steri_pipeline.pick_washed_tray()
    print("go to oven")
    steri_pipeline.go_to_oven()
    
    steri_pipeline.oven()
    steri_pipeline.pick_ovenned_tray()