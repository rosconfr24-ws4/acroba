#!/usr/bin/env python

import rospy
import actionlib
from acroba_unity_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from skills_msgs.msg import LocateAction, LocateResult
from skills_vg.piece_description import PieceType, PieceColor


class Locate:
    
    def __init__(self, name):
        self._action_name = name
        self.server = actionlib.SimpleActionServer(
            self._action_name, LocateAction, self.execute, False)
        self.server.start()
        self.model_states = None
        rospy.Subscriber('/model_states', ModelStates, self.model_states_callback)


    def model_states_callback(self, msg):
        rospy.loginfo(f"received model states")
        self.model_states = msg


    def execute(self, goal):
        result = LocateResult()
        
        if goal.piece_type == "":
            rospy.loginfo('expecting a piece type')
            self.server.set_aborted()
            return 
        
        rospy.loginfo("-"*20)
        rospy.loginfo(f"received request")
        rospy.loginfo(f"goal.piece_type {goal.piece_type}")
        rospy.loginfo(f"goal.color {goal.color}")

        piece_type = PieceType.get(goal.piece_type)
        rospy.loginfo(f"looking for piece_type {piece_type}")
        if not piece_type: 
            rospy.loginfo(f'unknown piece type {goal.piece_type}')
            self.server.set_aborted()
            return 
        piece_name = piece_type.to_code()
    
        has_color = False
        if goal.color != "":
            has_color = True
            color = PieceColor.get(goal.color)
            if not color: 
                rospy.loginfo(f'no piece found with color {goal.color}')                    
                self.server.set_aborted()
                return
        
            rospy.loginfo(f"looking for piece color {color}")
            color_code = color.to_code()
            if color_code!="": 
                piece_name = f"{piece_name} {color_code}"
        
        if has_color:
            rospy.loginfo(f"looking for piece name {piece_name}")
        else: 
            rospy.loginfo(f"looking for piece containing name {piece_name}")

        if piece_name and self.model_states:
            for i, name in enumerate(self.model_states.name):
                if has_color and piece_name == name or not has_color and piece_name in name:
                    rospy.loginfo(f"found {name}")
                    result.grasping_pose = self.model_states.pose[i]
                    result.cartesian = True
                    result.color = PieceColor.from_name(name).name
                    self.server.set_succeeded(result)
                    return
        
        rospy.loginfo('Piece not found.')
        self.server.set_aborted()


if __name__ == '__main__':
    rospy.init_node('Locate')
    server = Locate(rospy.get_name())
    rospy.spin()

