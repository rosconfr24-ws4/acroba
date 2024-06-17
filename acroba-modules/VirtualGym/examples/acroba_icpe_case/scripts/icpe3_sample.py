#!/usr/bin/env python3
from os import name
from statistics import mode
import copy
from tkinter import X
from click import getchar
import rospy
import time
from threading import Thread
from geometry_msgs.msg import Vector3,Pose,Point,Quaternion
##acroba msg types
from acroba_unity_msgs.msg import JointControllers,JointController 
from acroba_unity_msgs.msg import HumanCommand,ModelStates
import acroba_unity_msgs.srv
## common acroba interfaces
from acroba_gym.robot_complete_interface import RobotCompleteControl
#from acroba_gym.camera_interface import CameraControl
from acroba_gym.passive_tool_interface import PassiveToolInterface
from acroba_gym.unity_scene_manager import UnitySceneManager
#from acroba_gym.light_manager import LightManager
from tf.transformations import *
from acroba_gym.simulation_manager import SimulationManager
from acroba_gym.controller_manager import ControllerManager
import acroba_utils.geom_utils 

from acroba_icpe_case.magnet_spawner_interface import MagnetSpawnerInterface

"""
This sample is to make a demo video of the ikor case interfaces features
focusses on the digital twin features
"""

class IcpeCase3Pipeline:
    def __init__(self):
        self.robot_control=RobotCompleteControl()
        self.tool_control=PassiveToolInterface()
        self.scene_manager=UnitySceneManager()
        self.magnet_spawner=MagnetSpawnerInterface()
      
        self.is_running=False
        self.long_time=3
        self.short_time=0.5
        self.regular_time=2
        self.pick_time=3
        self.piece_per_row=6
    
    def start_scene(self):
        self.magnet_spawner.spawnNorth()
        rospy.sleep(0.5)
        self.create_pick_pose()
        self.create_pallet_vector()
       
   
    def pick_magnet(self):
        moving_time=self.regular_time
        initial_joints=[0.111,-2.1360,-1.5561,-1.019,1.49599,0]
        self.robot_control.move_joints(initial_joints,moving_time)
        rospy.sleep(moving_time)
        self.tool_control.setState(True)
        rospy.sleep(0.4)
        
    def place_magnet(self):
        moving_time=self.short_time
        middle_joints=[0.111,-1.1360,-1.5561,-1.019,1.49599,0]
        self.robot_control.move_joints(middle_joints,moving_time)
        rospy.loginfo("Going to middle pose")
        rospy.sleep(moving_time)
        
        # Go to preplace pose
        moving_time=self.short_time
        pre_place_j=[-1.216, -0.979, -2.004, -1.22, 1.47, 0]
        self.robot_control.move_joints(pre_place_j,moving_time)
        rospy.loginfo("Going to pre place")
        rospy.sleep(moving_time)
        
        #Go to place pose
        place_pose=Pose()
        place_pose.position.x= -0.0096
        place_pose.position.y= 0.40598151087760925
        place_pose.position.z= 0.2874033749103546
        place_pose.orientation.x= 0.9700403809547424
        place_pose.orientation.y= 0.16588656604290009
        place_pose.orientation.z= 0.01874486356973648
        place_pose.orientation.w= -0.17649921774864197

        #Get row
        if  self.curr_idx%self.piece_per_row==0:
            x_offset_multiplier=self.piece_per_row-(self.piece_per_row/2)-1
        else:
            x_offset_multiplier= self.curr_idx % (self.piece_per_row)-(self.piece_per_row/2)-1
        place_pose.position.x=place_pose.position.x+x_offset_multiplier*0.04
        print("for piece ",self.curr_idx," x offset is ",x_offset_multiplier)
        moving_time=self.short_time
        # place_joints=[-1.216, -1.18, -2.004, -1.22, 1.47, 0] from before
        self.robot_control.movePose(place_pose,moving_time)
        #rospy.loginfo("Going to pre place")
        rospy.sleep(moving_time)
        
        #print("robot place pose: ")
        print(self.robot_control.get_current_pose())
        
        self.tool_control.setState(False)
        rospy.sleep(1)
        
        self.robot_control.move_joints(pre_place_j,moving_time)
        #rospy.loginfo("Going to post place")
        rospy.sleep(moving_time)
        
        #pre_pick=[-1.15, -1.60, -1.70, -1.02, 1.466, 0]
        self.robot_control.move_joints(middle_joints,moving_time)
        #rospy.loginfo("Going to pre pick")
        rospy.sleep(moving_time)
        
        
    def joint_picking(self):
        rospy.sleep(1)
        self.pick_magnet()
        self.place_magnet()
        
    def one_cycle_picking(self):
        ## Get target pick pose
        pre_pick_pose=Pose()
        pre_pick_pose.position=copy.deepcopy(self.magnet_position_list[self.curr_idx])
        pre_pick_pose.orientation=self.pick_orientation
        pre_pick_pose.position.z=pre_pick_pose.position.z+0.01
        
        long_move_time=self.regular_time
        self.robot_control.movePose(pre_pick_pose,long_move_time)
        rospy.sleep(long_move_time)
        
        place_pose=Pose()
        place_pose.position=copy.deepcopy(self.magnet_position_list[self.curr_idx])
        place_pose.orientation=self.pick_orientation
        short_time=self.short_time
        self.robot_control.movePose(place_pose,short_time)
        
        rospy.sleep(short_time+0.1)
        
        self.tool_control.setState(True)
        rospy.sleep(0.1)
        
        #print("Press to continue")
        #getchar()
        
        self.robot_control.movePose(pre_pick_pose,short_time+1)
        rospy.sleep(short_time+1)
        self.curr_idx=self.curr_idx+1
           
        
    def create_pallet_vector(self):
        
        self.curr_idx=0
        self.magnet_position_list=[]
        init_x=-0.4598
        init_y=0.0499
        init_z=0.0095
        
        incr_x=-0.04
        incr_y=0.07
        
        number_rows=10
        number_cols=6
        
        for i in range(number_rows):
            x_coord=init_x+i*incr_x
            for j in range(number_cols):
                y_coord=init_y+j*incr_y
                self.magnet_position_list.append(Point(x_coord,y_coord,init_z))
                
    def create_pick_pose(self):
        self.pick_orientation=Quaternion()
        self.pick_orientation.x=-0.6663109064102173
        self.pick_orientation.y=-0.7447358965873718
        self.pick_orientation.z= -0.02480083331465721
        self.pick_orientation.w= 0.02798738144338131

    def follow_pallet_position(self):
        self.curr_idx=0
        move_time=2
        for pos in self.magnet_position_list:
            target_pose=Pose()
            target_pose.position=pos
            target_pose.orientation=self.pick_orientation
            self.robot_control.movePose(target_pose,move_time)
            rospy.sleep(move_time)
            rospy.loginfo("Move to next pose")
        
if __name__=="__main__":
    rospy.init_node("icpe_case_demo")
    icpe_pipeline=IcpeCase3Pipeline()
    
    rospy.loginfo("Start scene")
    icpe_pipeline.start_scene()
    icpe_pipeline.curr_idx=0
    for i in range(13):
        rospy.loginfo("picking")
        icpe_pipeline.one_cycle_picking()
        rospy.loginfo("placing magnet")
        icpe_pipeline.place_magnet()
    
    #icpe_pipeline.follow_pallet_position()
    
    #icpe_pipeline.create_pallet_vector()
    #print("Magnet position list \n",icpe_pipeline.magnet_position_list)
    
   
    
