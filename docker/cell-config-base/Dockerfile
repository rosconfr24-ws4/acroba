ARG BASE_IMAGE=acroba/acroba-base-ros1-gpu
FROM ${BASE_IMAGE}

RUN sudo apt-get update -y && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y\
    	ros-${ROS_DISTRO}-moveit\
        ros-${ROS_DISTRO}-robot-state-publisher\
        ros-${ROS_DISTRO}-ros-control \
        ros-${ROS_DISTRO}-ros-controllers\
        ros-${ROS_DISTRO}-tf-conversions\
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR ${WS_DIR}
COPY --from=acroba/skills_msgs ${WS_DIR} . 
COPY --from=acroba/virtualgym_msgs ${WS_DIR} . 
COPY --from=acroba/drl_skills_msgs ${WS_DIR} . 

RUN sudo apt-get update && sudo apt-get upgrade -y && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    rosdep update --rosdistro=${ROS_DISTRO} && \
    rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} && \
    catkin build"

