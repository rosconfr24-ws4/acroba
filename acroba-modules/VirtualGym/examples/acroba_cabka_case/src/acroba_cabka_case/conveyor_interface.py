#!/usr/bin/env python3
from os import name
from typing import Set
import rospy
import threading
from std_msgs.msg import Empty,Bool

from typing import Type

class ConveyorInterface(object):
    def __init__(self):
        self.sub_=rospy.Subscriber("/signal_conveyor", Empty, self.callback,queue_size=1)
        self.pub_=rospy.Publisher("/command_conveyor", Bool, queue_size=1)
        self.signal=False
        self.lock=threading.Lock()
        self.condition_variable=threading.Condition()
        
        
        
    def move(self,status):
        status_msg=Bool()
        status_msg.data=status
        self.pub_.publish(status_msg)
        
    def callback(self,msg):
        self.signal=True
        with self.condition_variable:
            self.condition_variable.notify()
            
    def waitTimeout(self, time):
        with self.condition_variable:
            # wait to be notified
            rospy.loginfo("waiting for signal")
            received=self.condition_variable.wait(timeout=time)
            if received==True:
                rospy.loginfo("signal received")
            else:
                rospy.loginfo("signal wait timeout")
            return received
        
if __name__=="__main__":
    rospy.init_node("Conveyor_node")
    conveyor_interface=ConveyorInterface()
    
    rospy.sleep(3)
    rospy.loginfo("Send move command")
    conveyor_interface.move(True)
    
    rospy.sleep(2)
    rospy.loginfo("Send stop command")
    conveyor_interface.move(False)
    
    rospy.sleep(2)
    rospy.loginfo("Send move command")
    conveyor_interface.move(True)
    
    conveyor_interface.waitTimeout(10)
    rospy.loginfo("Finish sample")
    
    
    
    
    
        
        