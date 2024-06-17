#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import skills_msgs.msg


def main():
    rospy.init_node("test_MoveGripperVG", anonymous=True)

    client = actionlib.SimpleActionClient(
        "MoveGripperVG", skills_msgs.msg.MoveGripperVGAction
    )
    client.wait_for_server()
    goal = skills_msgs.msg.MoveGripperVGGoal()
    # goal.gripper_type = "robotic_2f_85"
    # goal.gripper_type = "robotic_2f_140"
    goal.gripper_type = "robotic_hand_e"
    goal.fingers_poses = [0.01, 0]
    # goal.gripper_type = "onrobot_rg2"
    # goal.gripper_type = "dh_pgc_50"
    goal.width = 0.02
    goal.speed = 10
    client.send_goal(goal)
    client.wait_for_result()


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main()
