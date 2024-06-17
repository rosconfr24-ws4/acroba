import unittest
from actionlib import SimpleActionClient
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import sys

def move_robot_arm(joint_values):
  """
  Function to move the robot arm to desired joint angles.
  :param: joint_values A list of desired angles for the joints of a robot arm 
  """
  # Create the SimpleActionClient, passing the type of the action to the constructor
  arm_client = SimpleActionClient('unity_robot/follow_joint_trajectory', FollowJointTrajectoryAction)
 
  # Wait for the server to start up and start listening for goals.
  rospy.sleep(2)  
  print("waiting for action server") 
  arm_client.wait_for_server(timeout=rospy.Duration(5))
  # Create a new goal to send to the Action Server
  arm_goal = FollowJointTrajectoryGoal()
 
  # Store the names of each joint of the robot arm
  arm_goal.trajectory.joint_names = ['shoulder_1_joint', 'shoulder_2_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
   
  # Create a trajectory point   
  point = JointTrajectoryPoint()
 
  # Store the desired joint values
  point.positions = joint_values
 
  # Set the time it should in seconds take to move the arm to the desired joint angles
  point.time_from_start = rospy.Duration(3)
 
  # Add the desired joint values to the goal
  arm_goal.trajectory.points.append(point)
 
  # Define timeout values
  exec_timeout = rospy.Duration(10)
  prmpt_timeout = rospy.Duration(5)
 
  # Send a goal to the ActionServer and wait for the server to finish performing the action
  print("sending goal")
  return arm_client.send_goal_and_wait(arm_goal, exec_timeout, prmpt_timeout)
 
if __name__=="__main__":
        
    rospy.init_node('trayectory_test1', anonymous=True)
    rospy.sleep(1)
    print("Creating node")

    # Move the joints of the robot arm to the desired angles in radians
    state=move_robot_arm([-1.57, 1.57, 1.57, 0, 0,0])
    
           
        