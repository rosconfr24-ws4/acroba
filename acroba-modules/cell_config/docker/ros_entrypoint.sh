#!/usr/bin/env bash

# script made to be called explicitely when calling any ros command. 

#-----------------------------
# Sourcing Ros Workspace
#-----------------------------

source /opt/ros/noetic/setup.bash
source $WS_DIR/devel/setup.bash

#-----------------------------
# Setup
#-----------------------------

if [ "$1" = 'cell_config' ]; then
 
    if ! rostopic list; then
        echo "launching roscore" 
        screen -S roscore -dm roscore
        sleep 1
    fi 

    echo "launching cell config for virtual gym"
    roslaunch imr_cell_moveit_config virtual_gym.launch 
    sleep infinity
else
    exec "$@"
fi 
