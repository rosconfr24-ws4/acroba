#=============================================================
# Setting up image with CUDA & Open GL Support + ROS NOETIC
#============================================================

FROM docker.irtjv.local/dockerhub/nvidia/cudagl:11.4.2-runtime-ubuntu20.04

# https://github.com/NVIDIA/nvidia-docker/issues/1632
RUN rm /etc/apt/sources.list.d/cuda.list 
# RUN rm /etc/apt/sources.list.d/nvidia-ml.list

# setup timezone & useful utils
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends \
        curl \
        gnupg2 \
        lsb-release \
        tzdata \
        wget \
    && rm -rf /var/lib/apt/lists/*

#-------------------------------
# adding (minimal) ROS1 setup
#-------------------------------

ENV ROS_DISTRO noetic

RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# install ros packages
RUN apt-get update && apt-get install -y\
        ros-noetic-ros-base \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init

