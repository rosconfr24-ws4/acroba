#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Skill that calls GenerateTrajectory skill and ExecuteTrajectory primitives.
"""
import rospy
import actionlib
import skills_msgs.msg


class MoveTo:
    """
    Class for Move To skill.
    """

    # create messages that are used to publish feedback/result
    feedback_ = skills_msgs.msg.MoveToFeedback()
    result_ = skills_msgs.msg.MoveToResult()
    client_generate = actionlib.SimpleActionClient(
        "GenerateTrajectory", skills_msgs.msg.GenerateTrajectoryAction
    )
    client_execute = actionlib.SimpleActionClient(
        "ExecuteTrajectory", skills_msgs.msg.ExecuteTrajectoryAction
    )

    def __init__(self, name):
        self.client_generate.wait_for_server()
        self.client_execute.wait_for_server()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            skills_msgs.msg.MoveToAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo("Server ready")

    def execute_cb(self, goal):
        """
        Callback function.
        """
        rate = rospy.Rate(60)
        success = True
        goal_generate = goal
        goal_execute = skills_msgs.msg.ExecuteTrajectoryGoal()

        self.client_generate.send_goal(goal_generate)

        while self.client_generate.get_result() is None:
            print("generating trajectory")
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo("%s: Preempted" % self._action_name)
                self.client_generate.cancel_goal()
                self._as.set_preempted()
                success = False
                break
            rate.sleep()
        if self.client_generate.get_state() == 4:  # if goal aborted
            success = False
            rospy.loginfo("%s: Aborted" % self._action_name)
            self._as.set_aborted()
        if success:
            rospy.loginfo("Trajectory generated")
            result_generate = self.client_generate.get_result()
            goal_execute.trajectory = result_generate.trajectory
            goal_execute.planning_group = goal.planning_group
            self.client_execute.send_goal(goal_execute)
            while self.client_execute.get_result() is None:
                print("moving arm")
                if self._as.is_preempt_requested() or rospy.is_shutdown():
                    rospy.loginfo("%s: Preempted" % self._action_name)
                    self.client_execute.cancel_goal()
                    self._as.set_preempted()
                    success = False
                    break
                rate.sleep()
            if self.client_execute.get_state() == 4:  # if goal aborted
                success = False
                rospy.loginfo("%s: Aborted" % self._action_name)
                self._as.set_aborted()
        if success:
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded()


if __name__ == "__main__":
    rospy.init_node("MoveTo")
    server = MoveTo(rospy.get_name())
    rospy.spin()
