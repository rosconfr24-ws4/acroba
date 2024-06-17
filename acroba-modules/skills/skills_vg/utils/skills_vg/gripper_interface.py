#! /usr/bin/env python3

"""
Improved gripper_interface, from VG (/VirtualGym/examples/acroba_gym/scripts/acroba_gym/gripper_interface.py) to handle different type of simulated gripper
"""
import rospy
import copy
from sensor_msgs.msg import JointState

from acroba_unity_msgs.msg import GripperControlCommand


class ParallelYawControl(object):
    def __init__(
        self,
        state_topic="tool_joint_states",
        move_topic="move_tool_joints",
        gripper_type="robotic_hand_e",
    ):
        self._move_topic = move_topic
        self._pub1 = rospy.Publisher(
            self._move_topic, GripperControlCommand, queue_size=5
        )
        self._state_topic = state_topic
        self._sub1 = rospy.Subscriber(
            self._state_topic, JointState, self.state_callback
        )
        self._joints = None
        self._gripper_type = gripper_type
        if gripper_type == "robotic_hand_e":
            self.joint_names = ["base_finger_1", "base_finger_2"]
            self.control_range = [0, 0.025]  # control range
            self.spanning_width = 0.025  # max width in meter
        elif gripper_type == "robotic_2f_85":
            self.joint_names = ["width"]
            self.control_range = [0, 255]
            self.spanning_width = 0.085
        elif gripper_type == "robotic_2f_140":
            self.joint_names = ["width"]
            self.control_range = [0, 255]
            self.spanning_width = 0.140
        elif gripper_type == "onrobot_rg2":
            self.joint_names = ["width"]
            self.control_range = [0.11, 0]  # inverted
            self.spanning_width = 0.11
        elif gripper_type == "dh_pgc_50":
            self.joint_names = ["joint_base_finger_2", "joint_base_finger_1"]
            self.control_range = [0, 0.0175]
            self.spanning_width = 0.0175
        self.connected = False
        self.check_connections()

    def check_connections(self):
        while self._joints is None and not rospy.is_shutdown():
            try:
                state_msg = rospy.wait_for_message(
                    self._state_topic, JointState, timeout=5.0
                )
                rospy.logdebug("Current " + self._state_topic + " READY=>")
                self.connected = True
            except Exception as e:
                print(e)
                rospy.logerr(
                    "Current " + self._state_topic + " not ready yet, retrying..."
                )

    def open_custom(self, fingers_poses, width, speed=1):
        # for gripper with two independant fingers, we assume that their opening is symetrical, except if a fingers_poses goal is given
        if self._gripper_type == "robotic_hand_e" or self._gripper_type == "dh_pgc_50":
            if fingers_poses is None:
                self.move([width / 2, width / 2], speed)
            else:
                self.move(fingers_poses, speed)
        elif (
            self._gripper_type == "robotic_2f_85"
            or self._gripper_type == "robotic_2f_140"
        ):
            self.move(
                [
                    (self.spanning_width - width)
                    / self.spanning_width
                    * self.control_range[1]
                ],
                speed,
            )
        elif self._gripper_type == "onrobot_rg2":
            self.move([width], speed)

    def open_full(self, speed=1):
        if self._gripper_type == "robotic_hand_e" or self._gripper_type == "dh_pgc_50":
            self.move([self.control_range[0], self.control_range[0]], speed)
        elif (
            self._gripper_type == "robotic_2f_85"
            or self._gripper_type == "robotic_2f_140"
            or self._gripper_type == "onrobot_rg2"
        ):
            self.move([self.control_range[0]], speed)

    def close(self, speed=1):
        if self._gripper_type == "robotic_hand_e" or self._gripper_type == "dh_pgc_50":
            self.move([self.control_range[1], self.control_range[1]], speed)
        elif (
            self._gripper_type == "robotic_2f_85"
            or self._gripper_type == "robotic_2f_140"
            or self._gripper_type == "onrobot_rg2"
        ):
            self.move([self.control_range[1]], speed)

    def move(self, positions, speed=1, effort=0):
        msg = GripperControlCommand()
        msg.positions = positions
        msg.names = self.joint_names
        msg.speed = speed
        msg.effort = effort
        rospy.sleep(0.1)
        self._pub1.publish(msg)

    def get_position_state(self):
        if self._joints is None:
            return None
        return list(copy.deepcopy(self._joints.position))

    def get_velocity_state(self):
        if self._joints is None:
            return None
        return list(copy.deepcopy(self._joints.velocity))

    def get_closure_percentage(self):
        if self._joints is None:
            return None
        else:
            if (
                self._gripper_type == "robotic_hand_e"
                or self._gripper_type == "robotic_2f_85"
                or self._gripper_type == "robotic_2f_140"
                or self._gripper_type == "dh_pgc_50"
            ):
                return [
                    int(i / self.spanning_width * 100) for i in self._joints.position
                ]
            elif self._gripper_type == "onrobot_rg2":
                return [
                    int((1 - i / self.spanning_width) * 100)
                    for i in self._joints.position
                ]

    def state_callback(self, msg):
        self.connected = True
        self._joints = copy.deepcopy(msg)

    def check_is_moving(self, speed=1):
        # check two consecutive relative pose to know if the gripper has stopped or not
        position_error_threshold = abs(
            self.control_range[1] - self.control_range[0]
        ) / (
            100 / speed
        )  # we scale the threshold with the speed of the gripper
        previous_pose = rospy.wait_for_message(
            self._state_topic, JointState, timeout=1.0
        ).position
        # Discarding some poses
        for i in range(5):
            discarded_pose = rospy.wait_for_message(
                self._state_topic, JointState, timeout=1.0
            ).position
        actual_pose = rospy.wait_for_message(
            self._state_topic, JointState, timeout=1.0
        ).position
        pose_difference = [
            abs(pose_1 - pose_2) for (pose_1, pose_2) in zip(previous_pose, actual_pose)
        ]
        if not pose_difference or any(
            difference > position_error_threshold for difference in pose_difference
        ):
            rospy.loginfo("Gripper moving")
            return True
        else:
            return False
