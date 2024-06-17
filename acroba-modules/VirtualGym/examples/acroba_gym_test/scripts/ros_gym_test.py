#!/usr/bin/env python3
PKG='acroba_gym'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.
 
import sys
import unittest
import rospy
import time
import math
from geometry_msgs.msg import Pose,Point
from acroba_unity_msgs.msg import JointControllers,JointController
from acroba_unity_msgs.msg import UnityErrorCode,UnityLight
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal,JointTolerance,FollowJointTrajectoryResult
import cv2
#Sample interfaces
from acroba_gym.robot_simple_interface import RobotJointControl
from acroba_gym.camera_interface import CameraControl
from acroba_gym.gripper_interface import ParallelYawControl
from acroba_gym.unity_scene_manager import UnitySceneManager
from acroba_gym.light_manager import LightManager
from tf.transformations import *
from acroba_gym.simulation_manager import SimulationManager
from acroba_gym.controller_manager import ControllerManager
from acroba_unity_msgs.msg import JointControllers,JointController 

import copy
import numpy as np

"""
This test needs to be run after the virtual gym sample is executed

"""
class TestAcrobaCommunication(unittest.TestCase):

    def test_1_robot_control(self):
        rospy.loginfo("Test robot")
        robot_interface=RobotJointControl()
        #Time for connection only needed inside rostest
        rospy.sleep(4)
        robot_interface.check_connection()
        rospy.loginfo("Robot initialized %i",robot_interface.is_connected)
        self.assertTrue(robot_interface.is_connected)

        #Get current joints
        joints=robot_interface.get_current_joints()
        self.assertTrue(joints is not None)

        #Move joints
        joints2=copy.deepcopy(joints)
        joints2[1]=-0.5
        robot_interface.move_joints(joints2)
        rospy.sleep(5)
        joints=robot_interface.get_current_joints()
        self.assertTrue(abs(joints[1]-joints2[1])<0.05)

        #EXECUTE TRAJECTORY
        #Generate custom trajectory
        action_goal=FollowJointTrajectoryGoal()
        pt1=JointTrajectoryPoint()
        pt1.positions=[0,-1.57,0,0,0,0,0]
        pt1.time_from_start=rospy.Duration(2)
        pt2=JointTrajectoryPoint()
        pt2.positions=[3.14,-1.57,0,0,0,0]
        pt2.time_from_start=pt1.time_from_start+rospy.Duration(2)
        pt3=JointTrajectoryPoint()
        pt3.positions=[3.14,-1.57,-1.57,0,0,0]
        pt3.time_from_start=pt2.time_from_start+rospy.Duration(1)
        pt4=JointTrajectoryPoint()
        pt4.positions=[0,-1.57,-1.57,0,0,0]
        pt4.time_from_start=pt3.time_from_start+rospy.Duration(1)
        pt5=JointTrajectoryPoint()
        pt5.positions=[0,0,0,0,0,0]
        pt5.time_from_start=pt4.time_from_start+rospy.Duration(2)
        action_goal.trajectory.points.extend([pt1,pt2,pt3,pt4,pt5])
        ## Add reutral tolerances
        tolerance1=JointTolerance()
        tolerance1.position=-1
        tolerance1.velocity=-1
        tolerance1.acceleration=-1
        action_goal.path_tolerance.extend([tolerance1,tolerance1,tolerance1,tolerance1,tolerance1,tolerance1])
        action_goal.goal_tolerance.extend([tolerance1,tolerance1,tolerance1,tolerance1,tolerance1,tolerance1])
        action_goal.goal_time_tolerance=rospy.Duration(30)
        rospy.loginfo("Sending trajectory")
        robot_interface.execute_trajectory_action(action_goal)
        result=robot_interface.action_client.wait_for_result(rospy.Duration(10))
        #print(result)
        self.assertTrue(result is not None)
        self.assertTrue(result.error_code == 0)
        
    def test_2_camera_interface(self):
        rospy.loginfo("Test camera")
        bridge_object = CvBridge()
        #Call the client of the action server
        camera = CameraControl()
        result = camera.capture()
        self.assertFalse(result.image is None)
        self.assertFalse(result.depth_map is None)
        self.assertFalse(result.point_cloud is None)
        
        #Process to plot image
        result.image.encoding = "bgra8"
        cv_rgb_image= bridge_object.imgmsg_to_cv2(result.image, desired_encoding="rgba8")
        print("rgb shape",cv_rgb_image.shape)
        result.depth_map.encoding = "32FC1"
        cv_depth_image=bridge_object.imgmsg_to_cv2(result.depth_map,desired_encoding="32FC1")
        print("depth shape",cv_depth_image.shape)
        cv2.imshow("rgb", cv_rgb_image)
        cv2.imshow("depth",cv_depth_image/np.amax(cv_depth_image))
        cv2.waitKey(-1)
        
    def test_3_gripper_control(self):
        rospy.loginfo("Test gripper")
        parallel_yaw=ParallelYawControl()
        rospy.sleep(1)
        parallel_yaw.check_connections()
        self.assertTrue(parallel_yaw.connected)

        #Test that open an close pose are different
        #parallel_yaw.open()
        speed=1
        self.move([255,255],speed)
        rospy.sleep(3)
        open_state=parallel_yaw.get_state()
        #parallel_yaw.close()
        self.move([0,0],speed)
        rospy.sleep(3)
        close_state=parallel_yaw.get_state()

        self.assertEqual(len(open_state),len(open_state))
        self.assertNotEqual(open_state[0],close_state[0])

    def test_4_unity_scene_manager(self):
        rospy.loginfo("Test Scene Manager")
        unity_manager=UnitySceneManager()
        rospy.sleep(2)
        unity_manager.check_connections()
        self.assertTrue(unity_manager.is_connected)
        all_models_dict=unity_manager.get_all_models_dict()
        print("All dict:",all_models_dict)
        self.assertTrue(all_models_dict is not None)
        model_num_init=len(all_models_dict)
        rospy.loginfo("Number of object in scene %i",model_num_init)
        self.assertTrue(model_num_init>1)

        #add models
        names=["pth2_box","pth2_box"]
        pose_msg=Pose()
        pose_msg.position.x=0
        pose_msg.position.y=-0.97
        pose_msg.position.z=0.90
        pose_msg.orientation.x=0
        pose_msg.orientation.y=0
        pose_msg.orientation.z=0
        pose_msg.orientation.w=1
        pose_msg2=copy.deepcopy(pose_msg)
        pose_msg2.position.y=-0.94
        pose_msg2.position.z=0.95
        scale=Point()
        scale.x=1.2;scale.y=1.2;scale.z=1.2
        rospy.loginfo("Add model")
        unity_manager.add_models(names,[pose_msg,pose_msg2],[scale,scale])
        rospy.sleep(1)
        all_models_dict=unity_manager.get_all_models_dict()
        model_num_init2=len(all_models_dict)
        rospy.loginfo("Number of object in scene %i" ,model_num_init2)
        self.assertTrue(model_num_init==model_num_init2-2)

        #Change model pose of the last pose
        rospy.sleep(1)
        last_key=list(all_models_dict)[-1]
        last_key_pose=all_models_dict[last_key]
        last_key_pose.position.y=last_key_pose.position.y+0.1
        last_key_pose.position.z=last_key_pose.position.z+0.1
        unity_manager.set_models_states([last_key],[last_key_pose])
        rospy.sleep(2)

        #Removing model
        first_key=list(all_models_dict)[0]
        print("Destroying ",first_key," object")
        objects_to_destroy=["ur10_pgc50",last_key]
        unity_manager.remove_models(objects_to_destroy)
        rospy.sleep(1)
        all_models_dict=unity_manager.get_all_models_dict()
        model_num_init3=len(all_models_dict)
        self.assertTrue(model_num_init3<model_num_init2)

    def test_5_light_manager(self):
        rospy.loginfo("Test Light Manager")
        #Connect Interface
        light_manager=LightManager()
        rospy.sleep(1)
        light_manager.check_connections()
        self.assertTrue(light_manager.is_connected)

        #Get current  lights in the scene
        current_lights=light_manager.get_lights()
        num_old_lights=len(current_lights.lights)
        self.assertTrue(num_old_lights>0)

        #Add new light
        req=[]
        added_light=UnityLight()
        added_light.name="added light"
        added_light.type=UnityLight.SPOT
        added_light.status=UnityLight.ADD
        added_light.angle=90
        added_light.range=2
        added_light.color.r=0
        added_light.color.g=1
        added_light.color.b=0
        added_light.color.a=0
        added_light.pose.position.x=0.2
        added_light.pose.position.y=0.2
        added_light.pose.position.z=1.5
        added_light.intensity=8
        q_1=quaternion_from_euler(0,math.pi/2,0)
        added_light.pose.orientation.x=q_1[0]
        added_light.pose.orientation.y=q_1[1]
        added_light.pose.orientation.z=q_1[2]
        added_light.pose.orientation.w=q_1[3]
        req.append(added_light)
        res=light_manager.set_lights(req)
        self.assertTrue(res.status.val==UnityErrorCode.SUCCESS)
        rospy.sleep(0.1)

        #Check that now the lights have changed
        current_lights=light_manager.get_lights()
        num_current_lights=len(current_lights.lights)
        self.assertTrue((num_current_lights-num_old_lights)==1)

    def test_6_controllerManager(self):
        rospy.loginfo("Test ControllerManager")
        controller_manager=ControllerManager()
        controller_msg=JointControllers()
        #indexes
        controller_msg.controller_ids=[0,5]
        
        #Tool and robot velocities
        controller_msg.robot_vel=0.54
        controller_msg.tool_vel=0.012
        #controller for 0,1,2 joints
        joint_cont1=JointController()
        joint_cont1.damping=1
        joint_cont1.stiffness=9000
        joint_cont1.force_limit=1000
        controller_msg.joint_controllers.append(joint_cont1)
        joint_cont2=JointController()
        joint_cont2.damping=9876
        joint_cont2.stiffness=19234
        joint_cont2.force_limit=2000
        joint_cont2.body_angular_damping=0.345
        joint_cont2.body_linear_damping=0.213
        joint_cont2.joint_friction=0.1
        controller_msg.joint_controllers.append(joint_cont2)
        
        rospy.loginfo("Set controllers")
        rospy.sleep(1)
        res=controller_manager.set_controllers(controller_msg)
        print(res)
        rospy.sleep(1)
        #Receive the controllers
        res=controller_manager.get_controllers()
        threshold=0.1
        self.assertTrue(abs(res.controllers.robot_vel-controller_msg.robot_vel)<threshold)
        self.assertTrue(abs(res.controllers.tool_vel-controller_msg.tool_vel)<threshold)
        self.assertTrue(abs(res.controllers.joint_controllers[0].stiffness-controller_msg.joint_controllers[0].stiffness)<threshold)

    def test_7_set_joint_pose(self):
        rospy.loginfo("Test set position robot")
        robot_interface=RobotJointControl()
        #Time for connection only needed inside rostest
        rospy.sleep(4)
        #robot_interface.check_connection()
        self.assertTrue(robot_interface.is_connected)
        rospy.sleep(1)
        collision_status=robot_interface.in_collision
        print(collision_status)
        self.assertFalse(collision_status)
        robot_interface.set_joint_pose([0,1.57,1.57,0,0,0])
        rospy.sleep(3)
        rospy.loginfo("Testing collision checking")
        collision_status=robot_interface.in_collision
        self.assertTrue(collision_status)
        rospy.sleep(1)

    def test_8_restart_sim(self):
        #Probably we will implement the joint setup
        rospy.loginfo("Test Simuation manager Again")
        simulation_manager=SimulationManager()
        rospy.sleep(2)
        simulation_manager.reset_sim()
        rospy.sleep(2)
        self.assertTrue(True)
        
if __name__ == '__main__':
    import rostest
    rospy.init_node("tests")
    rostest.rosrun(PKG, 'test_unity_test', TestAcrobaCommunication)