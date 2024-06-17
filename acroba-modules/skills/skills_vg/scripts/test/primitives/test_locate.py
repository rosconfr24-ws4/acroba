#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from sensor_msgs.msg import Image
import skills_msgs.msg
import random
from skills_vg.piece_description import PieceType

import actionlib
from actionlib_msgs.msg import GoalStatus

def check_action_status(client):
    state = client.get_state()
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Action succeeded")
    elif state == GoalStatus.ABORTED:
        rospy.loginfo("Action failed")
    else:
        rospy.loginfo("Action is still running")


def main():
    rospy.init_node("test_locate", anonymous=True)
    
    client = actionlib.SimpleActionClient(
        "Locate", skills_msgs.msg.LocateAction
    )
    client.wait_for_server()
    goal = skills_msgs.msg.LocateGoal()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        color = random.choice(["BLUE", "RED", "ORANGE",""])
        piece_type = random.choice(list(map(lambda x: x.name, PieceType)))
        goal.piece_type = piece_type 
        goal.color = color 
        print(f"Sending goal {goal}")
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        print(f"got result {result}")
        check_action_status(client)
        rate.sleep()


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main()
