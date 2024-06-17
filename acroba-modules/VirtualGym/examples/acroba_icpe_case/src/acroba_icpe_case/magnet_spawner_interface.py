#!/usr/bin/env python3
from os import name
import rospy
from std_msgs.msg import String
from typing import Type

class MagnetSpawnerInterface(object):
    
    def __init__(self):
        
        self._pub=rospy.Publisher("/magnet_spawner",String,queue_size=0)
        rospy.sleep(0.4)
        
    def spawn(self,msg: str):
        msg_send=String()
        msg_send.data=msg
        self._pub.publish(msg_send)
        
    def spawnNorth(self):
        self.spawn("North")
        
    def spawnSouth(self):
        self.spawn("South");
        
if __name__=="__main__":
    rospy.init_node("manget_spawner")
    magnet_spawner=MagnetSpawnerInterface()
    magnet_spawner.spawnNorth()
    rospy.sleep(2)
    magnet_spawner.spawnSouth()
    rospy.sleep(0.5)