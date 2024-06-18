ARG BASE_IMAGE=ros1-nvidia

#-------------------------------------------------------------
# Setting up base images 
#-------------------------------------------------------------

FROM docker.irtjv.local/dockerhub/ros:galactic-ros-base as base-ros2
# fixing GPG Error (2023-11-20 https://github.com/osrf/docker_images/issues/697)
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

FROM docker.irtjv.local/dockerhub/ros:noetic-ros-base as base-ros1

FROM acroba/ros1-base-nvidia as base-ros1-nvidia

#-------------------------------------------------------------
#  ACROBA Base image
#-------------------------------------------------------------

FROM base-${BASE_IMAGE}

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y\
		build-essential \
    	curl\
		git\
		iputils-ping\
		libncurses-dev\
        nano\
		net-tools \
		# ssh server, needed for tp control
        openssh-server\ 
        psmisc\
		python3\
		python3-catkin-tools\
        python3-rosinstall\
        python3-vcstools\
        python3-pip\
		python-is-python3\
		screen\
		sudo\
		tar\
		tzdata\
		wget\
		xterm\
		xz-utils\
    && rm -rf /var/lib/apt/lists/*

ARG PASSWD=acroba
RUN groupadd -g 1000 acroba && \
    useradd -ms /bin/bash acroba -u 1000 -g 1000 && \
    echo "acroba ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    chown acroba:acroba /home/acroba && \
	echo "acroba:${PASSWD}" | chpasswd

ENV SHELL /bin/bash
ENV USER acroba
USER ${USER}

ARG ROSx=ros1
ENV ROS_VERSION=${ROSx}-${ROS_DISTRO}

ARG WS_DIR=/home/acroba/ros-workspaces/${ROS_VERSION}
ARG INSTALL_DIR=devel
ARG SETUP_SCRIPT=setup.bash
ENV WS_DIR=${WS_DIR}
ENV SRC_DIR=${WS_DIR}/src
ENV BIN_DIR=${WS_DIR}/${INSTALL_DIR}

# Creating the volumes folders 
# this is needed to avoid permissions issues when mounting volumes:
# if the folder does not exist, the volumes will be mounted using the root user 
# which may create some "permission denied" issues
RUN mkdir /home/acroba/data && \
	mkdir /home/acroba/logs && \
	mkdir /home/acroba/share

WORKDIR ${WS_DIR}

# bash setup for interactive sessions 
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
	echo "[ -f ${WS_DIR}/${INSTALL_DIR}/${SETUP_SCRIPT} ] && source ${WS_DIR}/${INSTALL_DIR}/${SETUP_SCRIPT}" >> ~/.bashrc && \
	echo "alias gotows='cd ${WS_DIR}'" >> ~/.bashrc && \
	echo "alias gotobin='cd ${BIN_DIR}'" >> ~/.bashrc && \
	echo "alias gotosrc='cd ${SRC_DIR}'" >> ~/.bashrc && \
	echo "alias gotoshare='cd /home/acroba/share'" >> ~/.bashrc && \
	echo "alias gotodata='cd /home/acroba/data'" >> ~/.bashrc && \
	echo "alias gotologs='cd /home/acroba/logs'" >> ~/.bashrc


COPY docker/entrypoint.sh /
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
