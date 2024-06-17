cmake_minimum_required(VERSION 3.0.2)
project(acroba_unity_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  moveit_msgs
  trajectory_msgs
  message_generation
  sensor_msgs
  actionlib_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
    ModelStates.msg
    UnityErrorCode.msg
    JointController.msg
    JointControllers.msg
    UnityLight.msg
    BiState.msg
    UnitySimulationState.msg
    CollisionState.msg
    GripperControlCommand.msg
    #IKOR Case
    TableVibration.msg
    HumanCommand.msg
    HumanState.msg
    HumanStates.msg
    ## ICPE CASE
    Stator.msg
    StatorStation.msg
    CoilOperation.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ExecuteTrajectory.srv
   SetControllers.srv
   GetControllers.srv
   SetLights.srv
   GetLights.srv
   GetImage.srv
   ResetScene.srv
   PieceSpawner.srv
 )

## Generate actions in the 'action' folder
 add_action_files(
  FILES
    ExecuteTrajectory.action
    RGBDSensor.action
    VibratingTableControl.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs 
  moveit_msgs
  sensor_msgs
  trajectory_msgs
  actionlib_msgs
   )

catkin_package(
  CATKIN_DEPENDS message_runtime 
)

install(DIRECTORY include/acroba_unity_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

