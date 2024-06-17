cmake_minimum_required(VERSION 3.0.2)
project(skills_msgs)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  actionlib_msgs
  genmsg
  geometry_msgs
  moveit_msgs
  sensor_msgs
  std_msgs
)

# # Generate actions in the 'action' folder
add_action_files(
  FILES
  # primitives
  primitives/ExecuteTrajectory.action
  primitives/GenerateTrajectory.action
  primitives/GetRGBDVG.action
  primitives/GetRobotPose.action
  primitives/GraspVG.action
  primitives/MoveGripperVG.action
  primitives/ReleaseVG.action

  # skills
  skills/Locate.action
  skills/MoveTo.action
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  moveit_msgs
  sensor_msgs
  std_msgs
)

catkin_package(

  # INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy

  # LIBRARIES skills
  # DEPENDS system_lib
)
