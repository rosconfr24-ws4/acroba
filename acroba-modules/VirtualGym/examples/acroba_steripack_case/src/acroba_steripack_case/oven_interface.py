#!/usr/bin/env python3
from os import name
from typing import Set
import rospy
from std_msgs.msg import Bool

from typing import Type

class OvenInterface(object):
    def __init__(self):
       
        self.pub_=rospy.Publisher("/command_oven", Bool, queue_size=1)
    
    def send(self,state):
        msg=Bool()
        msg.data=state
        self.pub_.publish(msg)   
        
    def on(self):
        self.send(True)
    
    def off(self):
        self.send(False)
        
    
if __name__=="__main__":
    rospy.init_node("oven_node")
    
    oven_interface=OvenInterface()
    rospy.sleep(2)
    oven_interface.on()
    rospy.sleep(5)
    oven_interface.off() 
        
        