#!/usr/bin/env python3
from os import name
from statistics import mode
import copy
from unittest import case
#from click import getchar
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

"""
This sample is to make a demo video of the ikor case interfaces features
focusses on the digital twin features
"""

class MosesCasePipeline:
    def __init__(self):
        self.dummy_sub=rospy.Subscriber("/dummy_tool_trayectory", PoseArray, self.dummy_callback,queue_size=1)
        self.robot_control=RobotCompleteControl()
        self.trajectory_received=False
        self.trajectory_executed=False
        
        
    def dummy_callback(self,pose_array_msg):
        """
        Create a inverse kinematics trayectory_from_the points

        Args:
            pose_array_msg (_type_): cartesian trayectory to do
        """
        rospy.loginfo("Received dummy callback")
        self.trayectory_msgs=JointTrajectory()
        seed=self.robot_control.get_current_joints()
        
        interval_time=1
        for pose in pose_array_msg.poses:
            
            #Fix orientation for test
            #quaternion = quaternion_from_euler(0, math.pi, math.pi*0.5)
            #pose.orientation.x=quaternion[0]
            #pose.orientation.y=quaternion[1]
            #pose.orientation.z=quaternion[2]
            #pose.orientation.w=quaternion[3]
            
            
            #print("pose to do inverse",pose)
            if seed==None:
                seed=[0,0,0,0,0,0]
            #print("seed",seed)
            ik_sol=self.robot_control.poseToJoint(seed,pose)
            #print("ik_sol:",ik_sol)
            if ik_sol is None:
                continue
            pt=JointTrajectoryPoint()
            pt.positions=copy.deepcopy(ik_sol)
            pt.time_from_start=rospy.Duration(interval_time)
            interval_time=interval_time+0.5
            self.trayectory_msgs.points.append(pt)
            seed=copy.deepcopy(ik_sol)
        if len(self.trayectory_msgs.points)>0:
            final_pose=pose_array_msg.poses[-1]
            final_pose.position.z=final_pose.position.z+0.08
            ik_sol=self.robot_control.poseToJoint(seed,final_pose)
            #add the last point with an offset in the Z direction
            if ik_sol is not None:
                pt_final=JointTrajectoryPoint()
                pt_final.positions=copy.deepcopy(ik_sol)
                pt_final.time_from_start=rospy.Duration(interval_time+3.0)
                self.trayectory_msgs.points.append(pt_final)
            self.trajectory_received=True
        rospy.loginfo("Trajectory processed")
        
    def execute_trajectory(self):
        rospy.loginfo("Send trajectory execution command")
        self.robot_control.execute_trajectory_topic(self.trayectory_msgs)
        #robot_interface.action_client.wait_for_result(rospy.Duration(secs=10))
        #res=robot_interface.action_client.get_result()
if __name__=="__main__":
    rospy.init_node("moses_case_demo")
    moses_pipeline=MosesCasePipeline()
    rospy.sleep(3)
    if moses_pipeline.robot_control.is_connected==False:
        sys.exit()
    rospy.loginfo("Robot connected")
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        if moses_pipeline.trajectory_received==True:
            moses_pipeline.trajectory_received=False
            moses_pipeline.execute_trajectory()
            #rospy.sleep(1)
            #rospy.signal_shutdown()
        rate.sleep()
        