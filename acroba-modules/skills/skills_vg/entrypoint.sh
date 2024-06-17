#!/usr/bin/env bash

ROS_SETUP=/opt/ros/noetic/setup.bash
WS_SETUP=/home/acroba/ros-workspaces/ros1-noetic/devel/setup.bash

source $ROS_SETUP
source $WS_SETUP
source $VENV_FOLDER/.pyenv/versions/skills_env/bin/activate


#-------------------------------------------------------------------------
if [ "$1" = "roscon" ]; then 
    echo "launching vg skills" 
    roslaunch skills_vg start_vg_primitives.launch 
else 
    exec "$@"
fi

