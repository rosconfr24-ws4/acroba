#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from geometry_msgs.msg import Pose
import skills_msgs.msg
import math
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("test_move_to", anonymous=True)

    client = actionlib.SimpleActionClient("MoveTo", skills_msgs.msg.MoveToAction)
    client.wait_for_server()
    goal = skills_msgs.msg.MoveToGoal()
    
    # ~ goal.endeffector = "camera"
    goal.endeffector = "endeffector"

    # ~ goal.frame_id = "camera_link" #camera frame
    goal.frame_id = "base_link" #UR10 base frame
    # ~ goal.frame_id = "panda_link0" #panda base frame
    # ~ goal.frame_id = "aruco_marker_203"
    # ~ goal.frame_id = "aruco_marker_62"
    
    #define velocity and acceleration factors for the movement
    goal.velocity_factor = 1
    goal.acceleration_factor = 1

    # define roll pitch and yaw in degrees
    roll = 180
    pitch = 0
    yaw = 0

    pi = math.pi
    quaternion = quaternion_from_euler(
        roll * pi / 180, pitch * pi / 180, yaw * pi / 180
    )

    point = Pose()
    point.position.x = 0
    point.position.y = 0
    point.position.z = 0.3
    point.orientation.x = quaternion[0]
    point.orientation.y = quaternion[1]
    point.orientation.z = quaternion[2]
    point.orientation.w = quaternion[3]

    goal.points.append(point)

    point2 = Pose()
    point2.position.x = 1.0
    point2.position.y = 0.15
    point2.position.z = 0.4
    point2.orientation.x = quaternion[0]
    point2.orientation.y = quaternion[1]
    point2.orientation.z = quaternion[2]
    point2.orientation.w = quaternion[3]

    # ~ goal.points.append(point2)

    goal.articular_pose = [
        -180 * pi / 180,
        -90 * pi / 180,
        90.0 * pi / 180,
        -130 * pi / 180,
        -100 * pi / 180,
        0 * pi / 180,
    ]

    goal.cartesian = False

    client.send_goal(goal)
    if client.wait_for_result():
        print("Success")


if __name__ == "__main__":
    main()
