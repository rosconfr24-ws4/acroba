import rospy
import py_trees
import actionlib
from actionlib_msgs.msg import GoalStatus
from skills_msgs.msg import LocateAction, LocateGoal, MoveToGoal, MoveToAction, MoveGripperVGAction, MoveGripperVGGoal


class Locate(py_trees.behaviour.Behaviour):

    def __init__(self, name, piece_type):
        super().__init__(name)
        self._piece_type = piece_type


    def setup(self, timeout): 
        self.logger.debug(f"[{self.name}::setup()]")
        self._c = actionlib.SimpleActionClient(self.name, LocateAction)
        self._c.wait_for_server()
        self._bb = py_trees.blackboard.Blackboard()


    def initialise(self):
        """ Sends the initial navigation action goal """
        color = self._bb.get(f"{self.name}.piece_color")
        self.logger.info(f"locating {color} {self._piece_type}")
        goal = LocateGoal()
        goal.color = color if color else "" 
        goal.piece_type = self._piece_type
        self._c.send_goal(goal)
        rospy.sleep(0.5)


    def update(self):
        """ Checks for the status of the navigation action """
        status = self._c.get_state()
        if status == GoalStatus.SUCCEEDED:
            result = self._c.get_result()
            self._bb.set("{self.name}.found_pose", result.grasping_pose)
            self._bb.set("{self.name}.detected_color", result.color)
            return py_trees.common.Status.SUCCESS
        if status == GoalStatus.ACTIVE:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")



class MoveTo(py_trees.behaviour.Behaviour):

    def __init__(self, name, pose_name):
        super().__init__(name)
        self.pose_name = pose_name


    def setup(self, timeout): 
        self._c = actionlib.SimpleActionClient(self.name, MoveToAction)
        self._c.wait_for_server(timeout)
        self._bb = py_trees.blackboard.Blackboard()


    def initialise(self):
        """ Sends the initial navigation action goal """
        goal = MoveToGoal()
        goal.endeffector = "endeffector"
        goal.frame_id = "base_link"
        goal.velocity_factor = 1
        goal.acceleration_factor = 1
        goal.cartesian = True
        pose = self._bb.get(self.pose_name)
        goal.points.append(pose)
        self._c.send_goal(goal)
        rospy.sleep(0.5)


    def update(self):
        """ Checks for the status of the navigation action """
        status = self._c.get_state()
        if status == GoalStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        if status == GoalStatus.ACTIVE:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.FAILURE


    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")



class OpenGripper(py_trees.behaviour.Behaviour):

    def __init__(self, name = "OpenGripper"):
        super().__init__(name)

    def setup(self, timeout): 
        self._c = actionlib.SimpleActionClient(self.name, MoveGripperVGAction)
        self._c.wait_for_server(timeout)

    def initialise(self):
        goal = MoveGripperVGGoal()
        goal.width = 0.07
        goal.gripper_type = "robotic_hand_e"
        goal.fingers_poses = [0.01, 0]
        goal.speed = 10
        self._c.send_goal(goal)
        rospy.sleep(0.5)



class CloseGripper(py_trees.behaviour.Behaviour):

    def __init__(self, name = "OpenGripper"):
        super().__init__(name)

    def setup(self, timeout): 
        self._c = actionlib.SimpleActionClient(self.name, MoveGripperVGAction)
        self._c.wait_for_server(timeout)

    def initialise(self):
        goal = MoveGripperVGGoal()
        goal.width = 0.02
        goal.gripper_type = "robotic_hand_e"
        goal.fingers_poses = [0.01, 0]
        goal.speed = 10
        self._c.send_goal(goal)
        rospy.sleep(0.5)



