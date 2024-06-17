#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

class MotorInterface:
    def __init__(self):
       
        self.pub_=rospy.Publisher("/command_motor", Bool, queue_size=1)
    
    def send(self,state):
        msg=Bool()
        msg.data=state
        self.pub_.publish(msg)   
        
    def move_right(self):
        self.send(True)
    
    def move_left(self):
        self.send(False)
        
if __name__=="__main__":
    rospy.init_node("motor_node")
    
    motor_interface=MotorInterface()
    rospy.sleep(2)
    motor_interface.move_right()
    rospy.sleep(5)
    motor_interface.move_left() 