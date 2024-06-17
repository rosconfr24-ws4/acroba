#! /usr/bin/env python3

"""
Primitive that acquire rgbd data from unity_camera ros action server
"""

import rospy
import actionlib

import skills_msgs.msg

from skills_vg.camera_interface import CameraControl


class GetRGBDVG:
    """
    Class for GetRGBDVG primitive.
    """

    # create messages that are used to publish feedback/result
    feedback = skills_msgs.msg.GetRGBDVGFeedback()
    result = skills_msgs.msg.GetRGBDVGResult()

    def __init__(self, name):
        self._action_name = name
        self.camera_interface = CameraControl()
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            skills_msgs.msg.GetRGBDVGAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo("Server Ready...")

    def execute_cb(self, goal):
        """
        Callback function.
        """
        success = True
        if self.camera_interface.connected is False:
            success = False
            self._as.set_aborted()
            rospy.logerr("Camera not connected")
        if success:
            self.result = self.camera_interface.capture()
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self.result)


if __name__ == "__main__":
    rospy.init_node("GetRGBDVG")
    server = GetRGBDVG(rospy.get_name())
    rospy.spin()
