FROM acroba/skills_msgs

#-----------------------------------
# install 3rd party libs & software
#-----------------------------------

RUN pip install pyyaml py_trees

RUN sudo apt-get update -y && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-py-trees \
        # ros-${ROS_DISTRO}-py-trees-ros-interfaces \
        ros-${ROS_DISTRO}-py-trees-ros \
        ros-${ROS_DISTRO}-py-trees-msgs \
        # ros-${ROS_DISTRO}-py-trees-ros-tutorials \
        # ros-${ROS_DISTRO}-py-trees-ros-viewer \
    && sudo rm -rf /var/lib/apt/lists/*


#----------------------------------
# Setting up ROS workspace for TP
#----------------------------------

WORKDIR ${SRC_DIR}
COPY --chown=${USER}:${USER} watch_assembly ./watch_assembly

WORKDIR ${WS_DIR}
RUN sudo apt-get update -y && \
    sudo apt-get upgrade -y && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} && \
    catkin build" &&\
    sudo rm -rf /var/lib/apt/lists/*


# Entrypoint does the ssh setup to access all containers 
COPY docker/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["start_atp", "--cli", "--api"]
