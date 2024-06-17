#!/usr/bin/env python3
from os import name
import rospy
from std_msgs.msg import String,ColorRGBA
import copy
import acroba_unity_msgs.srv
from acroba_unity_msgs.srv import PieceSpawner
from tf.transformations import *
import math
from typing import Type

class PieceSpawnerInterface(object):
    """ Class that manage of the spawner of pieces of unity scene"""
    def __init__(self,service_name="spawn_service"):
        self.is_connected=False
        
        self.spawner_services=rospy.ServiceProxy(service_name,PieceSpawner)
        rospy.sleep(0.1)
        #self.check_connections()
        
    def check_connections(self):
        while self.is_connected==False and not rospy.is_shutdown():
            try:
                
                rospy.wait_for_service(self.spawner_services,5)
                self.is_connected=True
                rospy.loginfo("Connected")
            except Exception as e: 
                print(e)
                rospy.logerr("Services are not ready yet, retrying...")
        
    def spawn(self,spawn_msg:Type[acroba_unity_msgs.srv.PieceSpawnerRequest]):
        """
        Parameters
        ----------
        name : lights
            list of UnityLight: directional and spot light point to the z in Unity 
            which corresponds to the X in Ros
       """
        try:
            res=self.spawner_services(spawn_msg)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            res=None
        return res

if __name__ == '__main__':
    rospy.init_node('spawner_interface', anonymous=True)
    spawner_interface=PieceSpawnerInterface()
    #spawner_interface.check_connections()
    spawn_msg=acroba_unity_msgs.srv.PieceSpawnerRequest()
    spawn_msg.num_pieces=2
    spawn_msg.piece_enum=1
    rospy.sleep(1)
    res=spawner_interface.spawn(spawn_msg)
    print("service response",res)
    rospy.sleep(1)
    ## throw other type of pieces
    print("Type of pieces 0")
    spawn_msg=acroba_unity_msgs.srv.PieceSpawnerRequest()
    spawn_msg.num_pieces=10
    spawn_msg.piece_enum=0
    rospy.sleep(1)
    res=spawner_interface.spawn(spawn_msg)
    print("service response",res)
    print("Type of pieces 2")
    spawn_msg.num_pieces=3
    spawn_msg.piece_enum=2
    rospy.sleep(1)
    res=spawner_interface.spawn(spawn_msg)
    print("service response",res)
    rospy.sleep(2)