FROM acroba/cell-config-base-gpu

RUN sudo apt-get update -y && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y\
    # ros 		
    ros-noetic-soem\
    ros-noetic-socketcan-interface\
    ros-noetic-gazebo-plugins\
    && sudo rm -rf /var/lib/apt/lists/*

#==========================================
# Cell setup (Drivers, moveit config, etc)
#==========================================

WORKDIR ${SRC_DIR}
COPY --chown=${USER}:${USER} cell_setup cell_setup

WORKDIR ${WS_DIR}
RUN sudo apt-get update -y && \
    sudo apt-get upgrade -y && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    rosdep update && rosdep install -y -r --from-paths src --ignore-src --rosdistro noetic --skip-keys requests --skip-keys python-pymodbus && \
    catkin build" &&\
    sudo rm -rf /var/lib/apt/lists/*

#==========================================
# Cell skills
#==========================================

WORKDIR ${SRC_DIR}
COPY --chown=${USER}:${USER} cell_skills cell_skills

WORKDIR ${WS_DIR}
RUN sudo apt-get update -y && \
    sudo apt-get upgrade -y && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    rosdep update && rosdep install -y -r --from-paths src --ignore-src --rosdistro noetic && \
    catkin build" &&\
    sudo rm -rf /var/lib/apt/lists/*


#==========================================
# Cell scripts
#==========================================

WORKDIR /home/acroba/cell-config/scripts
COPY --chown=${USER}:${USER} scripts . 

WORKDIR /home/acroba/cell-config/launch
COPY --chown=${USER}:${USER} launch . 

#========================================
# cell docker startup setup
#========================================

# shipping the docker-compose.yml file to the cell config image 
# this will be extracted and launched by the platform 

WORKDIR /home/acroba/cell-config/
COPY --chown=${USER}:${USER} docker/docker-compose.yml . 


#========================================
# Entrypoint 
#========================================

COPY docker/entrypoint.sh /
COPY docker/ros_entrypoint.sh /

RUN sudo chmod +x /entrypoint.sh /ros_entrypoint.sh 

ENTRYPOINT [ "/entrypoint.sh" ] 
CMD [ "cell_config" ] 
