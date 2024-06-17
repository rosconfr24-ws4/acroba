#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from sensor_msgs.msg import Image
import skills_msgs.msg


def main():
    rospy.init_node("test_get_rgbd", anonymous=True)
    pub_rgb = rospy.Publisher("rgb_image_vg", Image, queue_size=1)
    pub_depth = rospy.Publisher("depth_image_vg", Image, queue_size=1)

    client = actionlib.SimpleActionClient(
        "GetRGBDVG", skills_msgs.msg.GetRGBDVGAction
    )
    client.wait_for_server()
    goal = skills_msgs.msg.GetRGBDVGGoal()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        pub_rgb.publish(result.image)
        pub_depth.publish(result.depth_map)
        rate.sleep()


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main()
