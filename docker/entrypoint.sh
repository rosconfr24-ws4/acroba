#!/usr/bin/env bash

#---------------------------------------
# running [optional] setup stage

if [ -f "/entrypoint_setup.sh" ]; then
  echo "Running setup stage"
  /entrypoint_setup.sh
fi

#---------------------------------------
# saving the environment for ssh access 
mkdir -p ~/.ssh
# the entrypoint_setup could define some environment variable
# but also source some ROS workspace. we exclude all ROS related vars. 
# (ROS is later sourced by the /ros_entrypoint.sh) 
env | egrep -v "^(ROS_PYTHON_VERSION=|ROS_PATH=|ROS_VERSION=|ROS_PACKAGE_PATH=|ROSLISP|ROS_ETC_DIR=|ROS_MASTER_URI=|ROS_ROOT=|ROS_DISTRO=)" > ~/.ssh/environment


/ros_entrypoint.sh "$@"
