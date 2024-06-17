#!/usr/bin/env python3
from json import tool
from os import name
from typing import Set
import rospy
import threading
from std_msgs.msg import Empty,Bool,Int32

from typing import Type

class SpecialToolInterface(object):
    def __init__(self):
        self.pub_drill_=rospy.Publisher("/tool_drill", Bool,queue_size=1)
        self.pub_change_=rospy.Publisher("/tool_change", Int32, queue_size=1)
      
    def change_tool(self,tool_id: int):
        msg=Int32()
        msg.data=tool_id
        self.pub_change_.publish(msg)
        
    def drill(self,state: bool):
        msg=Bool()
        msg.data=state
        self.pub_drill_.publish(msg)
            
        
if __name__=="__main__":
    rospy.init_node("special_tool_node")
    special_tool_interface=SpecialToolInterface()
    
    rospy.sleep(3)
    rospy.loginfo("remove tools")
    special_tool_interface.change_tool(-1)
    
    rospy.sleep(3)
    rospy.loginfo("tool 0")
    special_tool_interface.change_tool(0)
    
    rospy.sleep(3)
    rospy.loginfo("tool 1")
    special_tool_interface.change_tool(1)
    
    rospy.sleep(3)
    
    
    
    
        
        