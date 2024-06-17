#!/bin/bash 

# create a few useful aliases
echo "alias gotowk='cd /home/acroba/workspaces/taskplanner/'" >> ~/.bash_aliases
echo "alias clean='cd /home/acroba/ros-workspaces/ros1-noetic/ && rm -rf install build log && cd -'" >> ~/.bash_aliases
echo "alias build='bash -c \"cd /home/acroba/ros-workspaces/ros1-noetic/ && pwd && catkin build; exit\"'" >> ~/.bash_aliases
echo "alias load='source /opt/ros/noetic/setup.bash && source /home/acroba/ros-workspaces/ros1-noetic/devel/local_setup.bash'" >> ~/.bash_aliases

