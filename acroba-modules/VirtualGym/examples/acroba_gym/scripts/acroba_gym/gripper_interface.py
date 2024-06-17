#!/usr/bin/env python3
import rospy
import os

from std_msgs.msg import Float64
from acroba_unity_msgs.msg import GripperControlCommand
from sensor_msgs.msg import JointState
import numpy as np
import copy
import sys

class ParallelYawControl(object):
    def __init__(self,state_topic="tool_joint_states",move_topic="move_tool_joints"):
        self._pub1=rospy.Publisher(move_topic,GripperControlCommand,queue_size=5)
        self.joint_names=["base_finger_1","base_finger_2"]
        self._state_topic=state_topic
        self._sub1=rospy.Subscriber(self._state_topic,JointState,self.state_callback)
        self._joints=None
        self.connected=False
        #self.check_connections()
        
    def check_connections(self):
        while self._joints is None and not rospy.is_shutdown():
            try:
                state_msg = rospy.wait_for_message(self._state_topic, JointState, timeout=5.0)
                rospy.logdebug("Current "+self._state_topic+" READY=>")
                self.connected=True
            except Exception as e: 
                print(e)
                rospy.logerr("Current "+self._state_topic+" not ready yet, retrying...")
    
    def open(self,speed=1):
        self.move([0,0],speed)
        
    def close(self,speed=1):
        #depending on the model
        self.move([1,1],speed)
        
    def move(self,positions,speed=1,effort=0):
        msg=GripperControlCommand()
        msg.positions=positions
        msg.names=self.joint_names
        msg.speed=speed
        msg.effort=effort
        self._pub1.publish(msg)
        
    def get_state(self):
        if self._joints is None:
            return None
        return list(copy.deepcopy(self._joints.position))
    
    def state_callback(self,msg):
        self.connected=True
        self._joints=copy.deepcopy(msg)
        
def test():
    rospy.init_node('gripper_control')
    parallel_yaw=ParallelYawControl()
    rospy.sleep(1)
    if parallel_yaw.connected==False:
        rospy.logerr("Gripper not connected")
        return

    # rospy.loginfo("close")
    # parallel_yaw.close()
    # rospy.sleep(1)
    # rospy.loginfo("press to open")
    # var=input()
    # parallel_yaw.open()
    # rospy.sleep(1)
    rospy.loginfo("press to move[0,1]")
    var=input()
    parallel_yaw.move([-0.5,1],speed=0.5)
    rospy.sleep(2)
    rospy.loginfo("press to move[1,0]")
    var=input()
    parallel_yaw.move([1.5707,0],speed=0.1)
    rospy.sleep(5)
    rospy.loginfo("press to close")
    var=input()
    parallel_yaw.close(speed=1)
    rospy.sleep(1)
    rospy.loginfo("press to open")
    var=input()
    parallel_yaw.open()
    return
def test_gripper_robotiq2f_85():
    rospy.init_node('gripper_control')
    parallel_yaw=ParallelYawControl()
    rospy.sleep(1)
    if parallel_yaw.connected==False:
        rospy.logerr("Gripper not connected")
        return
    
    
    rospy.loginfo("close gripper max velocity")
    parallel_yaw.move([255])
    rospy.sleep(4)
    rospy.loginfo("open gripper max velocity")
    parallel_yaw.move([0])
    rospy.sleep(4)
    
    rospy.loginfo("close gripper medium velocity")
    parallel_yaw.move([100],speed=0.5)
    rospy.sleep(4)
    
    rospy.loginfo("open gripper medium velocity")
    parallel_yaw.move([0],speed=0.5)
    rospy.sleep(4)
    
    
    
    
    
if __name__=='__main__':
    #test()
    test_gripper_robotiq2f_85()
        
        
        
        
    
    
    