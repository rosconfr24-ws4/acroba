#!/usr/bin/env python3
from os import name
import rospy
from acroba_unity_msgs.msg import CoilOperation,Stator,StatorStation
from click import getchar

class StatorInterface(object):
    def __init__(self):
        self.pub_stator=rospy.Publisher("/stator_spawn",Stator,queue_size=1)
        self.pub_coil=rospy.Publisher("/station_operation",CoilOperation,queue_size=1)
        rospy.sleep(0.5)
        
    def spawnStatorA(self,stator_name):
        stator_msg=Stator()
        stator_msg.stator_name=stator_name
        stator_msg.station.station=StatorStation.A
        stator_msg.operation=Stator.ADD
        self.pub_stator.publish(stator_msg)
        
        
    def spawnStatorB(self,stator_name):
        stator_msg=Stator()
        stator_msg.stator_name=stator_name
        stator_msg.station.station=StatorStation.B
        stator_msg.operation=Stator.ADD
        self.pub_stator.publish(stator_msg)
        
    def removeStatorA(self):
        stator_msg=Stator()
        stator_msg.station.station=StatorStation.A
        stator_msg.operation=Stator.REMOVE
        self.pub_stator.publish(stator_msg)
    
    def removeStatorB(self):
        stator_msg=Stator()
        stator_msg.station.station=StatorStation.B
        stator_msg.operation=Stator.REMOVE
        self.pub_stator.publish(stator_msg)
        
       
    def coilingOperationA(self,command):
        """
        Coil operation on stator A
        Args:
            command (bool): true to start coiling, false to stop
        """
        coil_msg=CoilOperation()
        coil_msg.command=command
        coil_msg.station.station=StatorStation.A
        self.pub_coil.publish(coil_msg)
        
    def coilingOperationB(self,command):
        coil_msg=CoilOperation()
        coil_msg.command=command
        coil_msg.station.station=StatorStation.B
        self.pub_coil.publish(coil_msg)
        
        
if __name__=="__main__":
    
    rospy.init_node("stator_manager_node")
    stator_interface=StatorInterface()
    stator_interface.spawnStatorA("Stator-TC_3001_1 - 1 - KSO152010 - Stator")
    rospy.sleep(0.5)
    stator_interface.spawnStatorB("Stator-TC_3001_1 - 3 - KSO284010 - Stator")
    rospy.sleep(0.5)
    stator_interface.coilingOperationA(True)
    print("Press to continue")
    getchar()
    stator_interface.coilingOperationB(False)
    rospy.sleep(1)
    stator_interface.coilingOperationA(False)
    rospy.sleep(1)
    stator_interface.removeStatorA()
    rospy.sleep(1)
    stator_interface.removeStatorB()
    
    
    