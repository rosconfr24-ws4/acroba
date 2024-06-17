#!/usr/bin/env python3
from os import name
from typing import Set
import rospy


from std_msgs.msg import Int32

from typing import Type

class PCBInterface(object):
    """ Class that manage of the lights of unity scene"""
    connected = False

    def __init__(self):
        self._sub=rospy.Subscriber("/pcb_on_place", Int32, self.callback,queue_size=1)
        self._pub=rospy.Publisher("/present_pcb", Int32, queue_size=2)
        self.status=None
    def send_command(self,value:int):
         # check connection
        msg=Int32()
        msg.data=value
        self._pub.publish(msg)
        
    def present_pcb(self):
        self.send_command(1)
        
    def remove_pcb(self):
        self.send_command(0)
        
    def callback(self,data):
        self.status=data.data
        
if __name__=="__main__":
    rospy.init_node("pcb_node")
    pcb_interface=PCBInterface()
    rospy.sleep(1)
    print("PCB status",pcb_interface.status)
    pcb_interface.present_pcb()
    rospy.sleep(5)
    print("PCB status after creating",pcb_interface.status)
    pcb_interface.remove_pcb()
    rospy.sleep(1)
    print("PCB status after removing",pcb_interface.status)
    