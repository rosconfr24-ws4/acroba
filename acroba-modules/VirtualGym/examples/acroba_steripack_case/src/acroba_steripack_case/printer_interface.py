#!/usr/bin/env python3
from os import name
from typing import Set
import rospy
import threading
from std_msgs.msg import Empty,Bool

from typing import Type

class PrinterInterface(object):
    def __init__(self):
        self.sub_=rospy.Subscriber("/signal_printer", Empty, self.callback,queue_size=1)
        self.pub_=rospy.Publisher("/command_printer", Empty, queue_size=1)
        self.signal=False
        self.lock=threading.Lock()
        self.condition_variable=threading.Condition()
        
        
        
    def print_action(self):
        msg=Empty()
        self.pub_.publish(msg)
        
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
    printer_interface=PrinterInterface()
    
    rospy.sleep(3)
    rospy.loginfo("print")
    printer_interface.print_action()
    

    printer_interface.waitTimeout(10)
    rospy.loginfo("printed signal received")
    
    
    
    
    
        
        