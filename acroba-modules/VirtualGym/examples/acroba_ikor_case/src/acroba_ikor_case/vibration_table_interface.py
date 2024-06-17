#!/usr/bin/env python3
from os import name
from typing import Set
import rospy
import actionlib
import acroba_unity_msgs.msg 
from geometry_msgs.msg import Vector3
import copy
from tf.transformations import *
import math
from typing import Type

class VibratingTableClient(object):
    """ Class that manage of the lights of unity scene"""
    connected = False

    def __init__(self):
        self._unityclient = actionlib.SimpleActionClient('vibrating_table', acroba_unity_msgs.msg.VibratingTableControlAction)
        self.connected = self._unityclient.wait_for_server(rospy.Duration(5))
        if not self.connected:
            rospy.logerr("Not possible to connect to the action server")
        else:
            rospy.loginfo("Connected to Unity Action Server")
            
    def vibrateTable(self,vibration_msg:Type[acroba_unity_msgs.msg.TableVibration]):
         # check connection
        if not self.connected:
            # Try to connect again
            rospy.logwarn("Reconnecting to action server")
            self.connected = self._unityclient.wait_for_server(rospy.Duration(secs=1))
            if not self.connected:
                rospy.logerr("Connection not possible")

        goal = acroba_unity_msgs.msg.VibratingTableControlGoal()
        goal.command=vibration_msg
        self._unityclient.send_goal(goal)
        print("waiting for results\n")
        self._unityclient.wait_for_result()
        print("result received")
        return self._unityclient.get_result()
    
    
if __name__  == '__main__':
    rospy.init_node('vibrating_table_interface')
    vibration_interface = VibratingTableClient()
    rospy.loginfo("Is connected %i", vibration_interface.connected)
    vibration_msg=acroba_unity_msgs.msg.TableVibration()
    vibration_msg.direction=Vector3()
    vibration_msg.direction.x=1
    vibration_msg.direction.y=0
    vibration_msg.direction.z=0
    vibration_msg.duration=rospy.Duration(10,0);
    vibration_msg.intensity=50
    print("Vibration ros message:\n",vibration_msg)
    result=vibration_interface.vibrateTable(vibration_msg)
    print("Results :\n",result)