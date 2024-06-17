#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import skills_msgs.msg


def main():
    rospy.init_node("test_ReleaseVG", anonymous=True)

    client = actionlib.SimpleActionClient(
        "ReleaseVG", skills_msgs.msg.ReleaseVGAction
    )
    client.wait_for_server()
    goal = skills_msgs.msg.ReleaseVGGoal()
    # goal.gripper_type = "robotic_2f_85"
    # goal.gripper_type = "robotic_2f_140"
    goal.gripper_type = "robotic_hand_e"
    # goal.gripper_type = "onrobot_rg2"
    # goal.gripper_type = "dh_pgc_50"
    goal.speed = 10
    client.send_goal(goal)
    client.wait_for_result()


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main()
