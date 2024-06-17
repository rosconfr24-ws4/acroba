#!/usr/bin/env bash

ROS_SETUP=/opt/ros/noetic/setup.bash
WS_SETUP=${WS_DIR}/devel/setup.bash

source $ROS_SETUP
source $WS_SETUP
source $VENV_FOLDER/.pyenv/versions/skills_env/bin/activate

#-------------------------------------------------------------------------
# importing the cell description package (needed for move_to primitives)
#-------------------------------------------------------------------------

if [ -f /home/acroba/share/.setup/cell-config ]; then 
    cell_config=$(cat /home/acroba/share/.setup/cell-config)
    cell_description=$cell_config/description
    if [ -d $cell_description ]; then 
        echo "importing the cell description package" 
        local_folder=${SRC_DIR}/cell-description/$(basename $cell_description)
        if [ ! -d  "$local_folder" ] || ! diff -r -q "$cell_description" "$local_folder"; then
            echo -e "\ncopying cell description"
            mkdir -p ${SRC_DIR}/cell-description/
            cp -r $cell_description ${SRC_DIR}/cell-description/
            echo -e "\nupdating"
            sudo apt-get update -y && sudo apt-get upgrade -y
            rosdep update --rosdistro ${ROS_DISTRO}
            echo -e "\ninstalling cell configuration dependencies"
            rosdep install -y -r --from-paths ${SRC_DIR}/cell-description/ --ignore-src --rosdistro ${ROS_DISTRO}
            echo -e "\nbuilding workspace"
            packages=$(find ${SRC_DIR}/cell-description/ -type f -name 'package.xml' -exec grep -oPm1 "(?<=<name>)[^<]+" {} \; | sort -u)
            echo "cell configuration package(s): $packages"
            catkin build --workspace ${WS_DIR} $packages
            source $WS_SETUP
        else
            echo "cell description package already imported, nothing to be done" 
        fi
    fi 
fi

#-------------------------------------------------------------------------
if [ "$1" = "roscon" ]; then 
    roslaunch skills all_skills.launch
else 
    exec "$@"
fi

