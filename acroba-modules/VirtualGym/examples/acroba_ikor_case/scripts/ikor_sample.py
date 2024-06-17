#!/usr/bin/env python3
from os import name
from statistics import mode
import copy
from unittest import case
from click import getchar
import rospy
import time
import sys
from threading import Thread
from geometry_msgs.msg import Vector3,Pose
##acroba msg types
from acroba_unity_msgs.msg import JointControllers,JointController 
from acroba_unity_msgs.msg import HumanCommand,ModelStates
import acroba_unity_msgs.srv
## common acroba interfaces
from acroba_gym.robot_complete_interface import RobotCompleteControl
#from acroba_gym.camera_interface import CameraControl
from acroba_gym.gripper_interface import ParallelYawControl
from acroba_gym.unity_scene_manager import UnitySceneManager
#from acroba_gym.light_manager import LightManager
from tf.transformations import *
from acroba_gym.simulation_manager import SimulationManager
from acroba_gym.controller_manager import ControllerManager
import acroba_utils.geom_utils 

## specific ikor case interfacs
from acroba_ikor_case.piece_spawner_interface import PieceSpawnerInterface
from acroba_ikor_case.human_interface import HumanInterface
from acroba_ikor_case.vibration_table_interface import VibratingTableClient
from acroba_ikor_case.PCBInterface import PCBInterface
"""
This sample is to make a demo video of the ikor case interfaces features
focusses on the digital twin features
"""

class IkorCasePipeline:
    def __init__(self):
        self.robot_control=RobotCompleteControl()
        self.gripper_control=ParallelYawControl()
        self.scene_manager=UnitySceneManager()
        self.piece_spawner=PieceSpawnerInterface()
        self.human_interface=HumanInterface()
        self.vibration_table_interface=VibratingTableClient()
        self.pcb_interface=PCBInterface()
        self.is_running=False
        
        self.short_time=0.5
        self.long_time=4
        self.regular_time=3
        
        self.pcb_placed_count=0
    
    def present_pcb(self):
        self.pcb_interface.present_pcb()
        
    def destroy_pcb(self):
        self.pcb_interface.remove_pcb()
        
    def close_gripper(self):
        self.gripper_control.move([0.02,0.02])
        rospy.sleep(2)
        
    def open_gripper(self):
        self.gripper_control.move([0,0])
        rospy.sleep(2)
        
    def add_human(self):
        command_msg=HumanCommand()
        command_msg.human_name="ikor_operator"
        command_msg.pose.x=1
        command_msg.pose.y=1.5
        command_msg.command=HumanCommand.CREATE
        self.human_interface.publish(command_msg)
        rospy.sleep(1)
        rospy.loginfo("human created")
        
        command_msg.human_name="ikor_operator"
        command_msg.pose.x=1.5
        command_msg.pose.y=1.5
        command_msg.command=HumanCommand.MOVE
        self.human_interface.publish(command_msg)
        rospy.sleep(1)
        rospy.loginfo("human moving")
    
    def start_human_thread(self):
        print("Start human thread")
        self.is_running=True
        self.worker=Thread(target=self.move_human)
        self.worker.start()
    def stop_human_thread(self):
        print("stopping thread")
        self.is_running=False
        self.worker.join()
        print("thread stopped")
        command_msg=HumanCommand()
        command_msg.human_name="ikor_operator"
        command_msg.command=HumanCommand.DESTROY
        self.human_interface.publish(command_msg)
        
    def move_human(self):
        # Four corner human moving
        command_msg=HumanCommand()
        start_time=time.time()
        while self.is_running and (time.time()-start_time)<20:
            command_msg.human_name="ikor_operator"
            command_msg.pose.x=-1.5
            command_msg.pose.y=1.5
            command_msg.command=HumanCommand.MOVE
            self.human_interface.publish(command_msg)
            #print("move human")
            rospy.sleep(3)
            if self.is_running==False:
                break
            command_msg.human_name="ikor_operator"
            command_msg.pose.x=-1.5
            command_msg.pose.y=-2
            command_msg.command=HumanCommand.MOVE
            self.human_interface.publish(command_msg)
            #print("move human")
            rospy.sleep(3)
            if self.is_running==False:
                break
            command_msg.human_name="ikor_operator"
            command_msg.pose.x=1.5
            command_msg.pose.y=-2
            command_msg.command=HumanCommand.MOVE
            self.human_interface.publish(command_msg)
            #print("move human")
            rospy.sleep(3)
            if self.is_running==False:
                break
            command_msg.human_name="ikor_operator"
            command_msg.pose.x=1.5
            command_msg.pose.y=1.5
            command_msg.command=HumanCommand.MOVE
            self.human_interface.publish(command_msg)
            rospy.sleep(3)
            
    def spawn_pieces(self):
        spawn_msg=acroba_unity_msgs.srv.PieceSpawnerRequest()
        spawn_msg.num_pieces=8
        spawn_msg.piece_enum=1
        rospy.sleep(1)
        res=self.piece_spawner.spawn(spawn_msg)
              
    def vibrateTable(self):
        vibration_msg=acroba_unity_msgs.msg.TableVibration()
        vibration_msg.direction=Vector3()
        vibration_msg.direction.x=1
        vibration_msg.direction.y=0
        vibration_msg.direction.z=0
        vibration_msg.duration=rospy.Duration(5,0)
        vibration_msg.intensity=70#1000
        
        result=self.vibration_table_interface.vibrateTable(vibration_msg)
    
    def select_grasp_piece(self):
        """ select the piece with less X """
        model_list=self.scene_manager.raw_model_data
        min_x=1000
        self.pick_pose=None
        for i in range(len(model_list.name)):
            if "part" in model_list.name[i] and model_list.pose[i].position.x<min_x:
                self.pick_pose=model_list.pose[i]
                min_x=model_list.pose[i].position.x
                
        ## Calculation of grasping pose
        #Z down orientation
        self.pick_pose.orientation.x=1
        self.pick_pose.orientation.y=0
        self.pick_pose.orientation.z=0
        self.pick_pose.orientation.w=0
        
    def fake_grasp_piece(self):
        names=["part2_rot_box"]#["part_2"] rot y 1.5707
        fake_pose=Pose()
        fake_pose.orientation=acroba_utils.geom_utils.euler2quatRos([0,0,0])
        fake_pose.position.x=0.105
        fake_pose.position.y=-0.47
        fake_pose.position.z=1.026
        print("Fake pose\n",fake_pose)
        self.scene_manager.add_models(names,[fake_pose])
        rospy.sleep(2)
        self.pick_pose=copy.deepcopy(fake_pose)
        self.pick_pose.position.z=self.pick_pose.position.z-0.015
        self.pick_pose.orientation.x=1
        self.pick_pose.orientation.y=0
        self.pick_pose.orientation.z=0
        self.pick_pose.orientation.w=0
        
        
        
    def perform_grasp(self):
        move_time=2
        pre_pick=Pose()
        pre_pick=self.pick_pose
        pre_pick.position.z=pre_pick.position.z+0.2
        rospy.loginfo("moving to prepick pose")
        print(pre_pick)
        self.robot_control.movePose(pre_pick,move_time,True)
        rospy.sleep(move_time)
        rospy.loginfo("moving to pick pose")
        #Reference copy
        self.pick_pose.position.z=pre_pick.position.z-0.20
        
        self.robot_control.movePose(self.pick_pose,move_time,True)
        rospy.sleep(move_time+0.2)
        ## Do grasping
        #rospy.loginfo("press to grasp")
        #getchar()
        self.close_gripper()
        rospy.sleep(1)
        #rospy.loginfo("press to continue")
        #getchar()
        rospy.loginfo("moving to pre pick pose again")
        pre_pick.position.z=pre_pick.position.z+0.2
        slow_motion=6
        self.robot_control.movePose(pre_pick,slow_motion)
        rospy.sleep(slow_motion)
        
    def fake_intermediate_surface_placing(self):
        names=["part2_rot_box"]#["pth2_box"]#["part_2"]
        intermediate_place=Pose()
        intermediate_place.position.x=0.087+0.04
        intermediate_place.position.y=-0.74
        intermediate_place.position.z=1
        euler_angles=[3.14,-0.262,0]
        intermediate_place.orientation=acroba_utils.geom_utils.euler2quatRos(euler_angles)
        #add some offset
        self.scene_manager.add_models(names,[intermediate_place])
        rospy.sleep(2)
        
    def IntermediateTable(self):
        
        intermediate_place=Pose()
        intermediate_place.position.x=0.087+0.04
        intermediate_place.position.y=-0.74
        intermediate_place.position.z=0.9202+0.002
        euler_angles=[3.14,-0.262,0]
        intermediate_place.orientation=acroba_utils.geom_utils.euler2quatRos(euler_angles)
        #add some offset
        intermediate_place.position.z=intermediate_place.position.z+0.05
        slow_move_time=10
        #self.robot_control.movePose(intermediate_place,self.long_time)
        #rospy.sleep(self.long_time)
        
        self.robot_control.movePose(intermediate_place,self.regular_time)
        rospy.sleep(self.regular_time)
        #free piece
        self.open_gripper()
        rospy.sleep(0.5)
        #Moveup
        intermediate_place.position.z=intermediate_place.position.z+0.01
        self.robot_control.movePose(intermediate_place,2)
        rospy.sleep(2)
        #Go to rectified pick pose
        rectified_pick=Pose()
        rectified_pick.position.x=0.031#0.087-0.056
        rectified_pick.position.y=-0.7025#-0.74+0.037
        rectified_pick.position.z=0.9103#0.9202+0.005
        rectified_pick.orientation=intermediate_place.orientation
        
        z_offset=0.06
        #prepick
        rectified_pick.position.z=rectified_pick.position.z+z_offset
        self.robot_control.movePose(rectified_pick,self.short_time)
        rospy.sleep(self.short_time)
        
        #pick
        rectified_pick.position.z=rectified_pick.position.z-z_offset
        self.robot_control.movePose(rectified_pick,self.short_time)
        rospy.sleep(self.short_time)
        self.close_gripper()
        rospy.sleep(1)
        
        #prepick
        #slow_motion=10
        rectified_pick.position.z=rectified_pick.position.z+z_offset;
        self.robot_control.movePose(rectified_pick,self.short_time)
        rospy.sleep(self.short_time)
        
    def place_in_pcb(self):
        pcb_pose=Pose()
        euler_angles=[3.14,0,1.5707]
        pcb_pose.orientation=acroba_utils.geom_utils.euler2quatRos(euler_angles)
        
        pcb_pose.position.x=-0.356
        pcb_pose.position.y=-1.0442
        pcb_pose.position.z=0.93
        
        incr_y=0
        incr_x=0
        if self.pcb_placed_count<3:
            incr_y=-0.1114
        else:
            incr_y=0.1045
        col_pcb=self.pcb_placed_count%3
        
        if col_pcb==0:
            incr_x=0.0755
        elif col_pcb==1:
            incr_x=0.0021
        else:
            incr_x=-0.084
        
        pcb_pose.position.y=pcb_pose.position.y+incr_y
        pcb_pose.position.x=pcb_pose.position.x+incr_x
        
        #move pose up
        z_offsset=0.08
        
        pcb_pose.position.z=pcb_pose.position.z+z_offsset
        self.robot_control.movePose(pcb_pose,self.regular_time)
        rospy.sleep(self.regular_time)
        
        pcb_pose.position.z=pcb_pose.position.z-z_offsset
        self.robot_control.movePose(pcb_pose,self.short_time)
        rospy.sleep(self.short_time)
        self.open_gripper()
        rospy.sleep(1)
        
        pcb_pose.position.z=pcb_pose.position.z+z_offsset
        self.robot_control.movePose(pcb_pose,self.short_time)
        rospy.sleep(self.short_time)
        
        
        
        if self.pcb_placed_count==5:
            self.pcb_placed_count=0
        else:
            self.pcb_placed_count=self.pcb_placed_count+1
        
        
            
            
        
    
if __name__=="__main__":
    rospy.init_node("ikor_case_demo")
    ikor_pipeline=IkorCasePipeline()
    rospy.sleep(1)
    if ikor_pipeline.vibration_table_interface.connected==False:
        print("Not possible to connect")
        exit()
    
    ikor_pipeline.add_human()
    ikor_pipeline.present_pcb()
    ikor_pipeline.start_human_thread()
    ikor_pipeline.spawn_pieces()
    rospy.sleep(1)
    rospy.loginfo("table vibrating")
    ikor_pipeline.vibrateTable()
    rospy.sleep(2)
    #print("raw data:\n",ikor_pipeline.scene_manager.raw_model_data)
    
    ### ikor_pipeline.select_grasp_piece()
  
    ikor_pipeline.fake_grasp_piece()
    ikor_pipeline.perform_grasp()
    
    #Intermediate surface
    #ikor_pipeline.fake_intermediate_surface_placing()
    if rospy.is_shutdown():
        sys.exit(0)
    ikor_pipeline.IntermediateTable()
    ikor_pipeline.place_in_pcb()
    #ikor_pipeline.stop_human_thread()
    ikor_pipeline.destroy_pcb()
    
    
        
