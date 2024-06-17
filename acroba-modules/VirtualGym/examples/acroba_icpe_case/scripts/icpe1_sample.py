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

## common acroba interfaces
from acroba_gym.robot_complete_interface import RobotCompleteControl
#from acroba_gym.camera_interface import CameraControl
from acroba_gym.passive_tool_interface import PassiveToolInterface
from acroba_gym.unity_scene_manager import UnitySceneManager
from tf.transformations import *
from acroba_gym.simulation_manager import SimulationManager
from acroba_gym.controller_manager import ControllerManager
import acroba_utils.geom_utils 

from acroba_icpe_case.stator_interface import StatorInterface
import tf.transformations


class IcpeCase1Pipeline:
    def __init__(self):
        self.robot_control=RobotCompleteControl()
        self.tool_control=PassiveToolInterface()
        self.scene_manager=UnitySceneManager()
        self.stator_interface=StatorInterface()
      
        self.is_running=False
        self.long_time=3
        self.short_time=0.4
        self.regular_time=2
        
        self.statorA_pose=Pose()
        self.statorA_pose.position.x=0.25654
        self.statorA_pose.position.y=-0.5
        self.statorA_pose.position.z=0.101
        tool_euler_angles=[-1.5707,0,0]
        
        self.statorA_pose.orientation=acroba_utils.geom_utils.euler2quatRos(tool_euler_angles)
    
    def start_scene(self):
        self.stator_interface.spawnStatorA("Stator-TC_3001_1 - 1 - KSO152010 - Stator")
        rospy.sleep(0.5)
        self.stator_interface.coilingOperationA(True)
        
        
    def createCoilPoses(self,radius=0.058,tooth_num=24,offset_angle=7.5):
        """Create the poses of the coild from a parametrization
        of a circunference and the self.stator pose
        Args:
            radius (float, optional): radius of the circle Defaults to 0.058.
            tooth_num (int, optional): number of tooths in stator. Defaults to 24.
            offset_angle (float, optional): initial offset angle in degrees. Defaults to 7.5.
        """
        self.coil_poses=[]
        for i in range(tooth_num):
            angle_d=offset_angle+i*360./tooth_num
            angle_rad=math.radians(angle_d)
            delta_x=radius*math.sin(angle_rad)
            delta_y=radius*math.cos(angle_rad)
            
            tooth_pose=copy.deepcopy(self.statorA_pose)
            tooth_pose.position.x=tooth_pose.position.x+delta_x
            tooth_pose.position.y=tooth_pose.position.y+delta_y
            tooth_pose.position.z=tooth_pose.position.z-0.008 #for coil offset
            
            #Rotate in Y
            quaternion_offset=tf.transformations.quaternion_about_axis(angle_rad,[0,1,0])
            #print("quaternion offset")
            #print(quaternion_offset)
            q_tooth=[self.statorA_pose.orientation.x,self.statorA_pose.orientation.y,self.statorA_pose.orientation.z,self.statorA_pose.orientation.w]
            q_mult=tf.transformations.quaternion_multiply(q_tooth,quaternion_offset)
            tooth_pose.orientation.x,tooth_pose.orientation.y,tooth_pose.orientation.z,tooth_pose.orientation.w=q_mult
            self.coil_poses.append(tooth_pose)
            
            
    def moveToCoils(self):
        for coil_pose in self.coil_poses:
            self.robot_control.movePose(coil_pose,self.short_time)
            print("moving to ")
            print(coil_pose)
            rospy.sleep(self.short_time)
            print("press to continue")
            getchar()
            
    def coilingMovement(self,anchor_pose,ver_size,hor_size,deep_size,num_spin):
        mat=acroba_utils.geom_utils.geompose2mat(anchor_pose)
        print("Matrix",mat)
        x_axis=mat[0:3,0]
        y_axis=mat[0:3,1]
        z_axis=mat[0:3,2]
        print("X axis",x_axis)
        print("Y axis",y_axis)
        for i in range(num_spin):
            z_displacement=i*deep_size/num_spin*z_axis
            for j in range(4):
                if j<2:
                    y_coeff=-1
                else:
                    y_coeff=1
                if j>0 and j<3:
                    x_coeff=1
                else:
                    x_coeff=-1
                    
                y_displacement=y_axis*y_coeff*ver_size*0.5
                x_displacement=x_axis*x_coeff*hor_size*0.5
                print("X displacement",x_displacement)
                print("Y displacement",y_displacement)
                print("Z displacement",z_displacement)
                coil_pose=Pose()
                coil_pose.position.x=anchor_pose.position.x+x_displacement[0]+y_displacement[0]+z_displacement[0]
                coil_pose.position.y=anchor_pose.position.y+x_displacement[1]+y_displacement[1]+z_displacement[1]
                coil_pose.position.z=anchor_pose.position.z+x_displacement[2]+y_displacement[2]+z_displacement[2]    

                coil_pose.orientation=anchor_pose.orientation
                print("Move to ",coil_pose)
                self.robot_control.movePose(coil_pose,self.short_time)
                rospy.sleep(self.short_time)
                #getchar()
    def doCoils(self,num_coils):
        for i in range(num_coils):
            icpe_pipeline.coilingMovement(icpe_pipeline.coil_poses[i],0.018,0.012,0.012,10)           
if __name__=="__main__":
    rospy.init_node("icpe_use_case_1")
    rospy.loginfo("Start scene")
    icpe_pipeline=IcpeCase1Pipeline()
    icpe_pipeline.start_scene()
    icpe_pipeline.createCoilPoses()
    start_pose=Pose()
    start_pose=copy.deepcopy(icpe_pipeline.coil_poses[0])
    start_pose.position.z=start_pose.position.z+0.05
    icpe_pipeline.robot_control.movePose(icpe_pipeline.coil_poses[0],icpe_pipeline.regular_time)
    rospy.sleep(icpe_pipeline.regular_time)
    #icpe_pipeline.coilingMovement(icpe_pipeline.coil_poses[0],0.018,0.012,0.012,5)
    icpe_pipeline.doCoils(5)
    