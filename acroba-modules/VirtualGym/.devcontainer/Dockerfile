FROM acroba/acroba-base-ros1-gpu

ENV ROS_DISTRO=noetic

RUN sudo apt-get update -y && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y \
        ros-${ROS_DISTRO}-control-msgs\
        ros-${ROS_DISTRO}-cv-bridge\
        ros-${ROS_DISTRO}-trac-ik\
        libnlopt-dev\
        libnlopt-cxx-dev\
    && sudo rm -rf /var/lib/apt/lists/*

RUN sudo python3 -m pip install UnityPy

	