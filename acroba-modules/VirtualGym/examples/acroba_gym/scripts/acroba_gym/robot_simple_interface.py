#!/usr/bin/env python3
import rospy
import os
from rospy.core import logerr
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64,Header
from actionlib_msgs.msg import GoalID
import actionlib
from acroba_unity_msgs.msg import ExecuteTrajectoryAction,ExecuteTrajectoryGoal
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint,JointTrajectory
from acroba_unity_msgs.msg import UnityErrorCode, CollisionState
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal,JointTolerance,FollowJointTrajectoryResult,FollowJointTrajectoryActionGoal
from std_msgs.msg import Bool
import copy
import time
import math
import numpy as np
import sys
import time

class RobotJointControl(object):
    """[summary]
    Class that send and received joint states on a robot
    anlges are received via /joint_states
    angles are send via /move_joints: for this topci only the position field is taking
    into accout
    """
    def __init__(self,num_joints=6,namespace=''):
        self.current_joints=None
        self.num_joints=num_joints
        self.namespace=namespace
        self._joint_topic=self.namespace+'/joint_states'
        # Acroba interface related 
        self.sub=rospy.Subscriber(self._joint_topic, JointState, self.joint_callback,queue_size=1)
        self.pub=rospy.Publisher(self.namespace+'/move_joints',JointState,queue_size=1,latch=False)
        self.action_client=actionlib.SimpleActionClient(self.namespace+'/unity_robot/follow_joint_trajectory',FollowJointTrajectoryAction)
        self.trajectory_topic_pub=rospy.Publisher(self.namespace+'/unity_robot/command',JointTrajectory,queue_size=1,latch=False)
        #Virtual gym related
        self.in_collision=None
        self.pub_set=rospy.Publisher(self.namespace+'/set_joints',JointState,queue_size=1)
        self.sub_col=rospy.Subscriber(self.namespace+'/robot_collision', CollisionState, self.collision_callback,queue_size=None)
        self.is_connected=False
        rospy.sleep(4)
        self.check_connection()
        self.target_joint=None
    
    def check_connection(self):
        self.action_started=self.action_client.wait_for_server(rospy.Duration(secs=3))
        if self.action_started==False:
            rospy.logwarn("Not possible to connect to action server")
        start_time=time.time()
        while self.sub.get_num_connections()==0 and (time.time()-start_time)<10:
            rospy.sleep(0.5)
        if self.sub.get_num_connections()>0:
            self.is_connected=True
        else:
            self.is_connected=False
        
        """
        while self.current_joints is None and not rospy.is_shutdown():
            try:
                se
                state_msg = rospy.wait_for_message(self._joint_topic, JointState, timeout=5.0)
                rospy.logdebug("Current "+self._joint_topic+" READY=>")
                self.is_connected=True
            except Exception as e: 
                print(e)
                rospy.logerr("Current "+self._joint_topic+" not ready yet, retrying...")
        """
        
        
    def joint_callback(self,msg):
        self.current_joints=copy.deepcopy(msg)
        
    def collision_callback(self,msg):
        m = copy.deepcopy(msg)
        self.in_collision = len(m.model_in_collision1) != 0
        
        
    # Set joint set the joints while deactivating the colliders so it can go
    # through walls to get is collision   
    def set_joint_pose(self,positions):
        joint_set_msg=JointState()
        joint_set_msg.position=positions   
        print("publishing this message",joint_set_msg) 
        self.pub_set.publish(joint_set_msg)
        
    
    #[DEPRECATED]
    #def move_joints(self,positions):
    #    joint_target_msg=JointState()
    #    joint_target_msg.position=positions
    #    self.target_joint=positions
    #    self.pub.publish(joint_target_msg)
        
        
    def get_current_joints(self):
        if self.current_joints is not None:
            return copy.deepcopy(list(self.current_joints.position))
        else:
            return None
        

    def execute_trajectory_action(self,goal_trajectory):
        """[launch trajectory action

        Args:
            trajectory ([moveit_msgs/RobotTrajectory]): trajectory to be executed
        """
        if self.action_started==False:
            #Try to connect again
            rospy.logwarn("Reconnecting to action server")
            self.action_started=self.action_client.wait_for_server(rospy.Duration(secs=1))
            if self.action_started==False:
                rospy.logerr("Conection not posible")
                #return False
        
        self.action_client.send_goal(goal_trajectory,feedback_cb=self.feedback_callback)
        #Waits at most 60 seconds
        return
    
    def wait_for_action_results(self):
        self.action_client.wait_for_result(rospy.Duration(secs=60))
        res=self.action_client.get_result()
        return
    
    def feedback_callback(self,feedback):
        rospy.loginfo("Error feedback [%s]"% str(feedback.error.positions))
       
    def execute_trajectory_topic(self,trajectory):
        if len(trajectory.points)>0:
            self.trajectory_topic_pub.publish(trajectory)
        else:
            rospy.logwarn("No point to send in the trajectory")
    
    def move_joints(self,joints,time=1.0):
        trajectory_msgs=JointTrajectory()
        pt_ik=JointTrajectoryPoint()
        pt_ik.positions=copy.deepcopy(joints)
        pt_ik.time_from_start=rospy.Duration(time)
        trajectory_msgs.points.append(pt_ik)
        #topic trajectory followgin
        self.execute_trajectory_topic(trajectory_msgs)

def trajectory_topic(robot_interface):
    trajectory_msgs=JointTrajectory()
    pt1=JointTrajectoryPoint()
    pt1.positions=[0,-1.57,0,0,0,0,0]
    pt1.time_from_start=rospy.Duration(2)
    pt2=JointTrajectoryPoint()
    pt2.positions=[0,0,0,0,0,0,0]
    pt2.time_from_start=pt1.time_from_start+rospy.Duration(10)
    trajectory_msgs.points.extend([pt1,pt2])
    rospy.loginfo("Execute trajectory")
    robot_interface.execute_trajectory_topic(trajectory_msgs)
    
    #Trajectory replacement
    trajectory_msgs2=JointTrajectory()
    pt1_2=JointTrajectoryPoint()
    pt1_2.positions=[0,-2.3,0,0,0,0,0]
    pt1_2.time_from_start=rospy.Duration(2)
    pt2_2=JointTrajectoryPoint()
    pt2_2.positions=[0.3,0,0,0,0,0,0]
    pt2_2.time_from_start=pt1.time_from_start+rospy.Duration(2)
    trajectory_msgs2.points.extend([pt1_2,pt2_2])
    rospy.sleep(3)
    rospy.loginfo("Sending trajectory replacement")
    robot_interface.execute_trajectory_topic(trajectory_msgs2)
    
def trajectory_action(robot_interface):
    action_goal=FollowJointTrajectoryGoal()
    
    pt1=JointTrajectoryPoint()
    pt1.positions=[0,-0.76,0,0,0,0,0]
    pt1.time_from_start=rospy.Duration(2)
    pt2=JointTrajectoryPoint()
    pt2.positions=[0,-1.57,-0.76,0,0,0,0]
    pt2.time_from_start=pt1.time_from_start+rospy.Duration(5)
    pt3=JointTrajectoryPoint()
    pt3.positions=[0,0,0,0,0,0];
    pt3.time_from_start=pt2.time_from_start+rospy.Duration(1)
    action_goal.trajectory.points.extend([pt1,pt2,pt3])
    tolerance1=JointTolerance()
    tolerance1.position=-1
    tolerance1.velocity=-1
    tolerance1.acceleration=-1
    action_goal.path_tolerance.extend([tolerance1,tolerance1,tolerance1,tolerance1,tolerance1,tolerance1])
    action_goal.goal_tolerance.extend([tolerance1,tolerance1,tolerance1,tolerance1,tolerance1,tolerance1])
    action_goal.goal_time_tolerance=rospy.Duration(30)
    rospy.loginfo("Sending trajectory")
    robot_interface.execute_trajectory_action(action_goal)
    #result=robot_interface.action_client.wait_for_result(rospy.Duration(10))
    #print(result)
    
    #Trajectory without tolerances
    rospy.sleep(4)
    action_goal2=FollowJointTrajectoryGoal()
    pt1_2=JointTrajectoryPoint()
    pt1_2.positions=[0,-2.3,-1.57,0.5,0.5,0.5]
    pt1_2.time_from_start=rospy.Duration(2)
    pt2_2=JointTrajectoryPoint()
    pt2_2.positions=[0.3,0,0,0,0,0,0]
    pt2_2.time_from_start=pt1.time_from_start+rospy.Duration(2)
    action_goal2.trajectory.points.extend([pt1_2,pt2_2])
    tolerance2=JointTolerance()
    tolerance2.position=2.5
    tolerance2.velocity=100
    tolerance2.acceleration=100
    tolerance_goal=JointTolerance()
    tolerance_goal.position=0.05
    tolerance_goal.velocity=100
    tolerance_goal.acceleration=100
    action_goal2.path_tolerance.extend([tolerance2,tolerance2,tolerance2,tolerance2,tolerance2,tolerance2])
    action_goal2.goal_tolerance.extend([tolerance2,tolerance2,tolerance2,tolerance2,tolerance2,tolerance2])
    action_goal2.goal_time_tolerance=rospy.Duration(2,100)
    rospy.loginfo("Sending trajectory replacement with strict tolerances")
    
    robot_interface.execute_trajectory_action(action_goal2)
    robot_interface.action_client.wait_for_result(rospy.Duration(secs=10))
    res=robot_interface.action_client.get_result()
    print("action result",res)
    #rospy.sleep(0.2)
    #robot_interface.execute_trajectory_action(action_goal)
    
def trajectory_deusto(robot_interface):
    
    deusto_pub=rospy.Publisher('/unity_robot/follow_joint_trajectory/goal',FollowJointTrajectoryActionGoal,queue_size=1,latch=False)
    action_goal=FollowJointTrajectoryActionGoal()
    rospy.sleep(1)
    action_goal.header=Header()
    action_goal.header.seq=0
    action_goal.goal_id=GoalID()
    pt1=JointTrajectoryPoint()
    pt1.positions=[0,0,0,0,0,0]
    pt1.velocities=[0,0,0,0,0,0]
    pt1.accelerations=[0,0,0,0,0,0]
    pt1.effort=[0,0,0,0,0,0]
    pt1.time_from_start=rospy.Duration(3)

    action_goal.goal.trajectory.points.append(pt1)
    tolerance1=JointTolerance()
    tolerance1.name=""
    tolerance1.position=0
    tolerance1.velocity=0
    tolerance1.acceleration=0
    action_goal.goal.path_tolerance.extend([tolerance1])
    action_goal.goal.goal_tolerance.extend([tolerance1])
    action_goal.goal.goal_time_tolerance=rospy.Duration(3)
    rospy.loginfo("Sending trajectory")
    deusto_pub.publish(action_goal)
    
    #print("action result",res)
    
    
if __name__=='__main__':
    rospy.init_node("robot_interface")
    robot_interface=RobotJointControl()
    rospy.loginfo("Is connected %i",robot_interface.is_connected)
    joints=robot_interface.get_current_joints()
    print(type(joints))
    print(joints)
    rospy.sleep(0.5)
    #trajectory_topic(robot_interface)
    #trajectory_deusto(robot_interface)
    trajectory_action(robot_interface)
    rospy.sleep(1)
    robot_interface.set_joint_pose([0,0,0,0,0,0])
    """
    rospy.sleep(5)
    pose_to_set=[0,1.57,-1.57,0,0,0]
    print("Setting joitn pose")
    #robot_interface.set_joint_pose(pose_to_set)
    
    req=RobotTrajectory()
    req.joint_trajectory.points=[]
    pt1=JointTrajectoryPoint()
    pt1.positions=[0,-1.57,0,0,0,0,0]
    pt2=JointTrajectoryPoint()
    pt2.positions=[3.14,-1.57,0,0,0,0]
    pt3=JointTrajectoryPoint()
    pt3.positions=[3.14,-1.57,-1.57,0,0,0]
    pt4=JointTrajectoryPoint()
    pt4.positions=[0,-1.57,-1.57,0,0,0]
    pt5=JointTrajectoryPoint()
    pt5.positions=[0,0,0,0,0,0]
    req.joint_trajectory.points.append(pt1)
    req.joint_trajectory.points.append(pt2)
    req.joint_trajectory.points.append(pt3)
    req.joint_trajectory.points.append(pt4)
    req.joint_trajectory.points.append(pt5)
    #Send execution command
    res=robot_interface.execute_trajectory_action(req)
    print("Is finished",res)
    """


