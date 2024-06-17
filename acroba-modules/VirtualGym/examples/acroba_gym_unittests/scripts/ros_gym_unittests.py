#!/usr/bin/env python3
PKG='acroba_gym_unittests'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.
 
import sys
import unittest
unittest.TestLoader.sortTestMethodsUsing = None
import rospy
import time
import math
import actionlib
from geometry_msgs.msg import Pose,Point
from sensor_msgs.msg import JointState
from acroba_unity_msgs.msg import GripperControlCommand
from acroba_unity_msgs.msg import ModelStates
from std_msgs.msg import Empty, Bool
from acroba_unity_msgs.srv import SetLights,GetLights,SetControllers,GetControllers,ResetScene
from acroba_unity_msgs.msg import JointControllers,JointController
from acroba_unity_msgs.msg import UnityErrorCode,UnityLight
from acroba_unity_msgs.msg import RGBDSensorAction,RGBDSensorGoal,ExecuteTrajectoryAction,ExecuteTrajectoryGoal,BiState, CollisionState
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint,JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal,JointTolerance,FollowJointTrajectoryResult
from tf.transformations import *
from rosgraph_msgs.msg import Clock
import copy

initial_joint_position_g=[]
class VirtualGymInterfaces(object):
    def __init__(self):
        self.current_joints = None
        self.current_tool_joints = None
        # self.current_model_state = None
        self.robot_in_collision = None
        self.clock = None
        self.model_pose_dict = {}
        self.robot_in_collision_sub = rospy.Subscriber('/robot_collision', CollisionState, self.collision_callback,
                                                       queue_size=1)
        self.clock_sub=rospy.Subscriber("/clock",Clock,self.clock_callback,queue_size=1)
        self.joint_states_sub = rospy.Subscriber("/joint_states", JointState, self.joint_callback, queue_size=1)
        self.model_states_sub = rospy.Subscriber("/model_states", ModelStates, self.model_callback, queue_size=1)
        self.tool_joint_states_sub = rospy.Subscriber("/tool_joint_states", JointState, self.tool_joint_callback,queue_size=1)
        self.tool_joint_states_pub = rospy.Publisher("/move_tool_joints",GripperControlCommand,queue_size=1)
        self.set_joint_pub = rospy.Publisher('/set_joints', JointState, queue_size=1)
        self.reset_sim = rospy.ServiceProxy('reset_sim_srv', ResetScene)
        self.move_joints_pub = rospy.Publisher('/move_joints', JointState, queue_size=1, latch=False)
        self.move_joint_traj_pub=rospy.Publisher("/unity_robot/command",JointTrajectory,queue_size=1,latch=False)
        self.add_model_pub = rospy.Publisher("/add_model", ModelStates, queue_size=1)
        self.remove_model_pub = rospy.Publisher("/remove_model", ModelStates, queue_size=1)
        self.set_model_states_pub = rospy.Publisher("/set_model_states", ModelStates, queue_size=2)
        #services
        self.light_setter = rospy.ServiceProxy("set_lights", SetLights)
        self.light_getter = rospy.ServiceProxy("get_lights", GetLights)
        self.controller_setter = rospy.ServiceProxy("controller_setter_service", SetControllers)
        self.controller_getter = rospy.ServiceProxy("controller_getter_service", GetControllers)
        #actions
        self.exec_trajectory_action_client = actionlib.SimpleActionClient('/unity_robot/follow_joint_trajectory',FollowJointTrajectoryAction)
        self.camera_action_client = actionlib.SimpleActionClient('unity_camera', RGBDSensorAction)
        self.action_started = self.exec_trajectory_action_client.wait_for_server(rospy.Duration(secs=5))
        self.action_started = self.action_started & self.camera_action_client.wait_for_server(rospy.Duration(secs=5))
        if self.action_started == False:
            rospy.logwarn("Not possible to connect to action server")

        start_time = time.time()
        while (self.tool_joint_states_sub.get_num_connections() == 0 \
               or self.model_states_sub.get_num_connections() == 0 \
               or self.joint_states_sub.get_num_connections() == 0 \
               or self.robot_in_collision_sub.get_num_connections() == 0) \
                and (time.time() - start_time) < 20:
            rospy.sleep(0.5)

        rospy.loginfo("connections %i %i %i %i", self.tool_joint_states_sub.get_num_connections(),
                      self.model_states_sub.get_num_connections(),
                      self.joint_states_sub.get_num_connections(),
                      self.robot_in_collision_sub.get_num_connections())

    ## callbacks
    def joint_callback(self, msg):
        self.current_joints = copy.deepcopy(msg)

    def model_callback(self, data):
        self.model_pose_dict.clear()
        for i in range(len(data.name)):
            self.model_pose_dict[data.name[i]] = data.pose[i]

    def tool_joint_callback(self, msg):
        self.current_tool_joints = copy.deepcopy(msg)
    def collision_callback(self, msg):
         m = copy.deepcopy(msg)
         self.robot_in_collision = len(m.model_in_collision1) != 0
    def clock_callback(self,msg):
        self.clock=msg.clock.to_sec()

"""
This test needs to be run after the virtual gym sample is executed

"""
class UnitTestsVirtualGymInterfaces(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vgi = VirtualGymInterfaces()
        
    def test00_can_receive_clock(self):
        rospy.sleep(1)
        if self.vgi.action_started==False:
            self.assertTrue(False)
        rospy.loginfo("test_can_receive_clock")
        rospy.sleep(1)
        self.assertTrue(self.vgi.clock is not None)
        self.assertTrue(self.vgi.clock>=0)
        # test 30 times if the clock is stable
        for i in range(30):
            clock_now = copy.deepcopy(self.vgi.clock)
            rospy.sleep(0.05)
            self.assertTrue(self.vgi.clock >= clock_now)
       
    def test01_can_receive_robot_joint_state(self):
        rospy.loginfo("test_can_receive_robot_joint_state")
        rospy.sleep(0.5)
        self.assertTrue(self.vgi.current_joints is not None)
        global initial_joint_position_g
        initial_joint_position_g=copy.deepcopy(list(self.vgi.current_joints.position))

    def test02_can_receive_model_state(self):
        rospy.loginfo("test_can_receive_model_state")
        rospy.sleep(0.5)
        self.assertTrue(len(self.vgi.model_pose_dict)>0)

    def test03_can_receive_tool_joint_state(self):
        rospy.loginfo("test_can_receive_tool_joint_state")
        rospy.sleep(0.5)
        self.assertTrue(self.vgi.current_tool_joints is not None)
        
    def test04_can_receive_if_robot_is_in_collision(self):
        rospy.loginfo("test_can_receive_if_robot_is_in_collision")
        rospy.sleep(0.5)
        self.assertTrue(self.vgi.robot_in_collision is not None)
        rospy.loginfo("robot collision %i",self.vgi.robot_in_collision)
        self.assertFalse(self.vgi.robot_in_collision)

    def test05_can_move_robot_joints(self):
        rospy.loginfo("test_can_move_robot_joints")
        positions = copy.deepcopy(list(self.vgi.current_joints.position))
        positions[1]=positions[1]-0.1
        trajectory_msgs=JointTrajectory()
        pt1=JointTrajectoryPoint()
        pt1.positions=copy.deepcopy(positions)
        pt1.time_from_start=rospy.Duration(1)
        trajectory_msgs.points.append(pt1)
        self.vgi.move_joint_traj_pub.publish(trajectory_msgs)
        rospy.sleep(3)
        
        self.assertTrue(abs(pt1.positions[1] - self.vgi.current_joints.position[1]) < 0.1)
    
    def test06_can_manipulate_model_in_the_scene(self):
        rospy.loginfo("test_can_manipulate_model_in_the_scene")
        state_msg = ModelStates()
        state_msg.name = ['pth2_box']
        pose_msg = Pose()
        pose_msg.position.x = 0
        pose_msg.position.y = -0.97
        pose_msg.position.z = 0.90
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = 0
        pose_msg.orientation.w = 1
        state_msg.pose = [pose_msg]
        scale=Point()
        scale.x=1.2;scale.y=1.2;scale.z=1.2
        state_msg.scale=[scale]
        nObj = len(self.vgi.model_pose_dict)
        self.vgi.add_model_pub.publish(state_msg)
        rospy.sleep(1)
        self.assertTrue(nObj == len(self.vgi.model_pose_dict) - 1)

        last_key = list(self.vgi.model_pose_dict)[-1]
        last_key_pose = self.vgi.model_pose_dict[last_key]
        last_key_pose_origin = copy.deepcopy(last_key_pose)
        last_key_pose.position.y = last_key_pose.position.y + 0.1
        last_key_pose.position.x = last_key_pose.position.x + 0.1
        state_msg = ModelStates()
        state_msg.name = [last_key]
        state_msg.pose = [last_key_pose]
        state_msg.scale=[scale]
        self.vgi.set_model_states_pub.publish(state_msg)
        rospy.sleep(1)
        last_key_pose = self.vgi.model_pose_dict[last_key]
        self.assertFalse(last_key_pose.position.y == last_key_pose_origin.position.y)
        self.assertFalse(last_key_pose.position.x == last_key_pose_origin.position.x)

        state_msg = ModelStates()
        state_msg.name = [last_key]
        self.vgi.remove_model_pub.publish(state_msg)
        rospy.sleep(1)

        self.assertTrue(nObj == len(self.vgi.model_pose_dict))
        
    def test07_can_open_and_close_gripper(self):
        #as no clear mapping between grippers just check position change
        rospy.loginfo("test_can_open_and_close_gripper")
        msg = GripperControlCommand()
        prev_tool_joints=copy.deepcopy(self.vgi.current_tool_joints)
        num_fingers=len(prev_tool_joints.position)
        self.assertTrue(num_fingers>0)
        if num_fingers==2:
            msg.positions = [0,0]
            ## general names
            msg.names = ["base_finger_1","base_finger_2"]
            msg.speed = 100
            msg.effort = 0
            self.vgi.tool_joint_states_pub.publish(msg)
            #waiting the gripper to finish
            rospy.sleep(3)
            minus_pose=copy.deepcopy(list(self.vgi.current_tool_joints.position))
            msg.positions = [1,1]
            self.vgi.tool_joint_states_pub.publish(msg)
            rospy.sleep(3)
            maxus_pose=copy.deepcopy(list(self.vgi.current_tool_joints.position))
        elif num_fingers==1:
            msg.positions = [0]
            ## general names
            msg.names = ["width"]
            msg.speed = 100
            msg.effort = 0
            self.vgi.tool_joint_states_pub.publish(msg)
            #waiting the gripper to finish
            rospy.sleep(3)
            minus_pose=copy.deepcopy(list(self.vgi.current_tool_joints.position))
            msg.positions = [1]
            self.vgi.tool_joint_states_pub.publish(msg)
            rospy.sleep(3)
            maxus_pose=copy.deepcopy(list(self.vgi.current_tool_joints.position))
        else:
            return 
            
        if num_fingers>0:
            self.assertTrue(abs(maxus_pose[0] - minus_pose[0]) > 0.001)
        if num_fingers>1:
            if len(maxus_pose)>1 and len(minus_pose)>1:
                self.assertTrue(abs(maxus_pose[1] - minus_pose[1]) > 0.001)
        
            

    def test08_can_set_and_get_light(self):
        rospy.loginfo("test_can_set_and_get_light")
        added_light = UnityLight()
        added_light.name = "added light"
        added_light.type = UnityLight.SPOT
        added_light.status = UnityLight.ADD
        added_light.angle = 90
        added_light.range = 2
        added_light.color.r = 0
        added_light.color.g = 1
        added_light.color.b = 0
        added_light.color.a = 0
        added_light.pose.position.x = 0.2
        added_light.pose.position.y = 0.2
        added_light.pose.position.z = 1.5
        added_light.intensity = 8
        q_1 = quaternion_from_euler(0, math.pi / 2, 0)
        added_light.pose.orientation.x = q_1[0]
        added_light.pose.orientation.y = q_1[1]
        added_light.pose.orientation.z = q_1[2]
        added_light.pose.orientation.w = q_1[3]
        try:
            res = self.vgi.light_setter([added_light])
        except rospy.ServiceException as exc:
            res = None

        try:
            res2 = self.vgi.light_getter()
        except rospy.ServiceException as exc:
            res2 = None

        self.assertTrue(res.status.val == UnityErrorCode.SUCCESS)
        self.assertTrue(res2 is not None)
    
    def test09_can_set_and_get_robot_controller(self):
        rospy.loginfo("test_can_set_and_get_robot_controller")
        try:
            res = self.vgi.controller_getter()
        except rospy.ServiceException as exc:
            res = None

        self.assertTrue(res is not None)


        res.controllers.joint_controllers[0].stiffness = 100000
        res.controllers.joint_controllers[0].damping = 1000
        res.controllers.robot_vel=0.54
        res.controllers.tool_vel=0.005

        try:
            res2 = self.vgi.controller_setter(res.controllers)
        except rospy.ServiceException as exc:
            res2 = None

        self.assertTrue(res2 is not None)

        try:
            res3 = self.vgi.controller_getter()
        except rospy.ServiceException as exc:
            res3 = None

        rospy.loginfo("controller 0: %f %f",
                      res3.controllers.joint_controllers[0].stiffness,
                      res3.controllers.joint_controllers[0].damping)
        self.assertTrue(res3.controllers.joint_controllers[0].stiffness == res.controllers.joint_controllers[0].stiffness)
        self.assertTrue(res3.controllers.joint_controllers[0].damping == res.controllers.joint_controllers[0].damping)
        self.assertTrue(abs(res3.controllers.robot_vel-res.controllers.robot_vel)<0.01)
        self.assertTrue(abs(res3.controllers.tool_vel-res.controllers.tool_vel)<0.01)


class UnitTestsVirtualGymInterfaces2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vgi = VirtualGymInterfaces()
    
    def test01_can_reset_sim(self):
        rospy.loginfo("test_can_reset_sim")

        self.vgi.reset_sim()

        rospy.sleep(3)
        positions = copy.deepcopy(list(self.vgi.current_joints.position))
        global initial_joint_position_g
        self.assertTrue(len(initial_joint_position_g)== len(positions))
        error = 0.0
        for i in range(6):
            error += abs(positions[i] - initial_joint_position_g[i])

        error /= 6
        rospy.loginfo("error %f", error)
        self.assertTrue(error < 0.1)
    
    def test02_can_execute_trajectory_topic(self):
        rospy.sleep(1)
        #self.setUpClass()
        
        init_positions=copy.deepcopy(list(self.vgi.current_joints.position))
        rospy.loginfo("test_can_execute_trajectory_topic")
        trajectory_msgs=JointTrajectory()
        
        pt1=JointTrajectoryPoint()
        pt1.positions=copy.deepcopy(init_positions)
        pt1.positions[1]=init_positions[1]-0.1
        pt1.time_from_start=rospy.Duration(2)
        pt2=JointTrajectoryPoint()
        pt2.positions=copy.deepcopy(init_positions)
        pt2.positions[1]=init_positions[1]-0.1
        pt2.positions[2]=init_positions[2]-0.1
        pt2.time_from_start=pt1.time_from_start+rospy.Duration(2)
        pt3=JointTrajectoryPoint()
        pt3.positions=copy.deepcopy(init_positions)
        pt3.time_from_start=pt2.time_from_start+rospy.Duration(1)
        trajectory_msgs.points.extend([pt1,pt2,pt3])
        ## Add reutral tolerances
        #tolerance1=JointTolerance()
        #tolerance1.position=-1
        #tolerance1.velocity=-1
        #tolerance1.acceleration=-1
        #action_goal.path_tolerance.extend([tolerance1,tolerance1,tolerance1,tolerance1,tolerance1,tolerance1])
        #action_goal.goal_tolerance.extend([tolerance1,tolerance1,tolerance1,tolerance1,tolerance1,tolerance1])
        #action_goal.goal_time_tolerance=rospy.Duration(30)

        self.vgi.move_joint_traj_pub.publish(trajectory_msgs)
        rospy.sleep(3)
        curr2_positions=copy.deepcopy(list(self.vgi.current_joints.position))
        #self.assertTrue(abs(curr2_positions[1]-init_positions[1])>0.1)
        self.assertTrue(self.vgi.action_started)
        
    def test03_can_execute_trajectory_action(self):
        rospy.sleep(1)
        
        init_positions=copy.deepcopy(list(self.vgi.current_joints.position))
        rospy.loginfo("test_can_execute_trajectory_action")
        action_goal=FollowJointTrajectoryGoal()
        
        pt1=JointTrajectoryPoint()
        pt1.positions=copy.deepcopy(init_positions)
        pt1.positions[1]=init_positions[1]-0.1
        pt1.time_from_start=rospy.Duration(2)
        pt2=JointTrajectoryPoint()
        pt2.positions=copy.deepcopy(init_positions)
        pt2.positions[1]=init_positions[1]-0.1
        pt2.positions[2]=init_positions[2]-0.1
        pt2.time_from_start=pt1.time_from_start+rospy.Duration(2)
        pt3=JointTrajectoryPoint()
        pt3.positions=copy.deepcopy(init_positions)
        pt3.time_from_start=pt2.time_from_start+rospy.Duration(1)
        action_goal.trajectory.points.extend([pt1,pt2,pt3])
        ## Add reutral tolerances
        tolerance1=JointTolerance()
        tolerance1.position=-1
        tolerance1.velocity=-1
        tolerance1.acceleration=-1
        action_goal.path_tolerance.extend([tolerance1,tolerance1,tolerance1,tolerance1,tolerance1,tolerance1])
        action_goal.goal_tolerance.extend([tolerance1,tolerance1,tolerance1,tolerance1,tolerance1,tolerance1])
        action_goal.goal_time_tolerance=rospy.Duration(30)

        self.vgi.exec_trajectory_action_client.send_goal(action_goal)
        self.vgi.exec_trajectory_action_client.wait_for_result(rospy.Duration(secs=60))

        res = self.vgi.exec_trajectory_action_client.get_result()
        
        self.assertTrue(res is not None)
        rospy.loginfo("action result %i",res.error_code)
        self.assertTrue(res.error_code==0)
        
    def test04_can_execute_trajectory_action_violation(self):
        rospy.sleep(1)
        
        init_positions=copy.deepcopy(list(self.vgi.current_joints.position))
        rospy.loginfo("test_can_execute_trajectory_action_violation")
        action_goal=FollowJointTrajectoryGoal()
        
        pt1=JointTrajectoryPoint()
        pt1.positions=copy.deepcopy(init_positions)
        pt1.positions[1]=init_positions[1]-0.7
        pt1.time_from_start=rospy.Duration(2)
        
       
        action_goal.trajectory.points.extend([pt1])
        ## Add reutral tolerances
        tolerance1=JointTolerance()
        tolerance1.position=0.001
        tolerance1.velocity=-1
        tolerance1.acceleration=-1
        action_goal.path_tolerance.extend([tolerance1,tolerance1,tolerance1,tolerance1,tolerance1,tolerance1])
        action_goal.goal_tolerance.extend([tolerance1,tolerance1,tolerance1,tolerance1,tolerance1,tolerance1])
        action_goal.goal_time_tolerance=rospy.Duration(30)

        self.vgi.exec_trajectory_action_client.send_goal(action_goal)
        self.vgi.exec_trajectory_action_client.wait_for_result(rospy.Duration(secs=60))

        res = self.vgi.exec_trajectory_action_client.get_result()
        self.assertTrue(res is not None)
        rospy.loginfo("action result %i",res.error_code)
        self.assertTrue(res.error_code==-4)

        
    #def test05_can_capture(self):
    #    rospy.loginfo("test_can_capture")
    #    goal = RGBDSensorGoal()
    #    self.vgi.camera_action_client.send_goal(goal)
    #    self.vgi.camera_action_client.wait_for_result()
    #    res = self.vgi.camera_action_client.get_result()
    #    self.assertTrue(res is not None)

    #def test06_can_force_robot_joint_pose(self):
    #    rospy.loginfo("test_can_force_robot_joint_pose")
    #    joint_set_msg = JointState()
    #    joint_set_msg.position = [0,-1.57,1.57,0,0,0]
    #    self.vgi.set_joint_pub.publish(joint_set_msg)
    #    rospy.sleep(4)
    #    error = 0.0
    #    for i in range(6):
    #        error += abs(self.vgi.current_joints.position[i] - joint_set_msg.position[i])
    #    error /= 6
    #    rospy.loginfo("error %f", error)
    #    self.assertTrue(error < 0.1)
        
    def test07(self):
        self.test01_can_reset_sim()
    

if __name__ == '__main__':
    import rostest
    rospy.init_node("tests")
    rostest.rosrun(PKG, 'unity_test', __name__, sys.argv)
    #rostest.rosrun(PKG, 'unity_test', UnitTestsVirtualGymInterfaces2, 'test04_can_force_robot_joint_pose')