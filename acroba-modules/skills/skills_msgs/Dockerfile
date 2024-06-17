FROM acroba/acroba-base-ros1 as skills_msgs

WORKDIR ${SRC_DIR}
COPY . skills_msgs/

WORKDIR ${WS_DIR}
RUN sudo apt-get update -y && \
    sudo apt-get upgrade -y && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && \
     rosdep update --rosdistro ${ROS_DISTRO} &&\
     rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} && \
     catkin build" &&\
    sudo rm -rf /var/lib/apt/lists/*
