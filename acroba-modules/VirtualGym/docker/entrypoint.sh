#!/usr/bin/env bash

ROS_SETUP=/opt/ros/noetic/setup.bash
WS_SETUP=/home/acroba/ros-workspaces/ros1-noetic/devel/setup.bash

source $ROS_SETUP
source $WS_SETUP

if [ "$1" = 'roscon' ]; then 
    if [ "$2" = "headless" ]; then 
        /home/acroba/ros-workspaces/ros1-noetic/src/VirtualGym/virtual_scenes/roscon_imr_cell/roscon_imr_cell.x86_64 -batchmode -nographics &
    elif [ "$2" = "default" ]; then 
        /home/acroba/ros-workspaces/ros1-noetic/src/VirtualGym/virtual_scenes/roscon_imr_cell/roscon_imr_cell.x86_64 &
    fi
    roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=0.0.0.0 tcp_port:=10000
    sleep infinity
else
    exec "$@"
fi 
