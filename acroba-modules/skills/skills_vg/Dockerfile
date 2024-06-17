FROM acroba/skills_base

WORKDIR ${WS_DIR}
COPY --from=acroba/virtualgym_msgs --chown=${USER}:${USER} ${WS_DIR} .

WORKDIR ${SRC_DIR}
COPY --chown=${USER}:${USER} . skills_vg

WORKDIR ${WS_DIR}
RUN sudo apt-get update -y && \
    sudo apt-get upgrade -y && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} && \
    catkin build" &&\
    sudo rm -rf /var/lib/apt/lists/*


COPY ./entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
