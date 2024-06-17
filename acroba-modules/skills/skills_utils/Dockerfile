FROM acroba/skills_msgs as skills_utils

#------------------------------------------------------------------
# set up skills dependencies common to both real and virtual skills
#------------------------------------------------------------------

#WORKDIR ${SRC_DIR}
#RUN git clone https://github.com/Maxpad/open3d_conversions.git
#COPY ./skills_utils skills_utils

# WORKDIR ${WS_DIR}
# RUN sudo apt-get update -y && \
#     sudo apt-get upgrade -y && \
#     /bin/bash -c "source /opt/ros/noetic/setup.bash && \
#     rosdep update --rosdistro ${ROS_DISTRO} &&\
#     rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} && \
#     catkin build" &&\
#     sudo rm -rf /var/lib/apt/lists/*


FROM acroba/acroba-base-ros1-gpu 

#------------------------------------------------
# Copying ros1 workspace from skills_msgs package
#-------------------------------------------------

ENV WS_DIR=/home/acroba/ros-workspaces/ros1-noetic/
ENV SRC_DIR=${WS_DIR}/src

COPY --from=skills_utils --chown=${USER}:${USER} ${WS_DIR} ${WS_DIR}

WORKDIR ${WS_DIR}
RUN sudo apt-get update -y && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y --no-install-recommends \
        tk-dev\ 
        # needed for empy dependency
        libffi-dev\
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    rosdep update --rosdistro ${ROS_DISTRO} &&\
    rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}" && \
    # catkin build && \
    sudo rm -rf /var/lib/apt/lists/*

#----------------------------------------
# setting up python virtual env
#----------------------------------------

ENV VENV_FOLDER=/home/acroba/env
ENV PYENV_ROOT=${VENV_FOLDER}/.pyenv
ENV PATH=${PYENV_ROOT}/bin:${PATH}

WORKDIR $VENV_FOLDER
RUN git clone https://github.com/yyuu/pyenv.git ${PYENV_ROOT} && \
    git clone https://github.com/yyuu/pyenv-virtualenv.git ${PYENV_ROOT}/plugins/pyenv-virtualenv && \
    echo 'export PATH="${PYENV_ROOT}/bin:$PATH"' >> ~/.bashrc && \
    echo 'eval "$(pyenv init - --no-rehash)"' >> ~/.bashrc && \
    echo 'eval "$(pyenv virtualenv-init -)"' >> ~/.bashrc && \
    /bin/bash -c "source ~/.bashrc" && \
    pyenv install 3.8.10 && \
    cp /usr/lib/python3.8/lib-dynload/_bz2.cpython-38-x86_64-linux-gnu.so ${PYENV_ROOT}/versions/3.8.10/lib/python3.8/lib-dynload/ && \
    cp /usr/lib/python3.8/lib-dynload/_lzma.cpython-38-x86_64-linux-gnu.so ${PYENV_ROOT}/versions/3.8.10/lib/python3.8/lib-dynload/ && \
    pyenv virtualenv 3.8.10 skills_env

WORKDIR $WS_DIR
COPY env/requirements.txt ${VENV_FOLDER}/skills_requirements.txt
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    source $VENV_FOLDER/.pyenv/versions/skills_env/bin/activate && \
    python -m pip install --upgrade pip && \
    pip install -r ${VENV_FOLDER}/skills_requirements.txt && \
    pip install gdown &&\
    pip list"

RUN echo "source $VENV_FOLDER/.pyenv/versions/skills_env/bin/activate" >> ~/.bashrc
