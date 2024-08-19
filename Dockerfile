FROM nvidia/cudagl:11.1.1-base-ubuntu20.04 as base

SHELL ["/bin/bash", "-c"]

# Install git and wget
RUN apt-get update && apt-get install git wget -y

# Install Miniconda
RUN wget https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3.sh
RUN bash ~/miniconda3.sh -b -p ~/miniconda3
RUN rm ~/miniconda3.sh

RUN export PATH=~/miniconda3/bin:$PATH

# Minimal setup
ENV ROS_DISTRO noetic
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install --no-install-recommends -y \
    locales \
    lsb-release \
    curl \
    && rm -rf /var/lib/apt/lists/*
RUN dpkg-reconfigure locales

# Install ROS Noetic Desktop Full
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update --rosdistro $ROS_DISTRO

# source setup.bash on startup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

FROM base as dev

# Set the working directory in the container
WORKDIR /root/catkin_ws

# Install general dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    python3-catkin-tools \
    python3-zipp \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies
RUN pip install pip --upgrade
RUN pip install \ 
    pyserial \
    pymodbus===2.1.0 \
    numpy \
    numpy-quaternion \
    scipy
RUN pip install six --upgrade
RUN pip install setuptools --upgrade && pip install PyQt6

# Source the workspace setup files on container startup
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
# source ros setup files on startup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Allow "python" to execute python3
RUN echo "alias python=python3" >> ~/.bashrc

# Install ROS dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-noetic-moveit \
    ros-noetic-cartesian-control-msgs \
    ros-noetic-teleop-twist-keyboard \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Copy the morpheus repo
COPY ./ ./src/

# General rosdep install (not necessary?)
RUN source /opt/ros/noetic/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install -q -y \
      --from-paths ./src \
      --ignore-src \
      --rosdistro noetic \
    && rm -rf /var/lib/apt/lists/*

# Build the ROS workspace
# RUN source /opt/ros/noetic/setup.bash \
#     catkin build