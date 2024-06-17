#! /usr/bin/env python3

"""
Primitive that controls the closure of the Virtual Gym gripper (robotiq Hand-e)
"""
import rospy
import actionlib

import skills_msgs.msg
from skills_vg.gripper_interface import ParallelYawControl


class GraspVG:
    """
    Class for Grasping VG primitive
    """

    # create messages that are used to publish feedback/result
    feedback = skills_msgs.msg.GraspVGFeedback()
    result = skills_msgs.msg.GraspVGResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            skills_msgs.msg.GraspVGAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()

        rospy.loginfo("Server Ready...")

    def execute_cb(self, goal):
        """
        Callback function.
        """
        success = False
        parallel_yaw = ParallelYawControl(gripper_type=goal.gripper_type)
        if parallel_yaw.connected:
            parallel_yaw.close(goal.speed)
            while parallel_yaw.check_is_moving(goal.speed):
                self.feedback.closure_percentage = (
                    parallel_yaw.get_closure_percentage()
                )
                self._as.publish_feedback(self.feedback)
            success = True
            if success:
                rospy.loginfo("%s: Succeeded" % self._action_name)
                self._as.set_succeeded(self.result)


if __name__ == "__main__":
    rospy.init_node("GraspVG")
    server = GraspVG(rospy.get_name())
    rospy.spin()
