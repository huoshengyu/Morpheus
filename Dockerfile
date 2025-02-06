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

# Source ROS setup files on container startup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

FROM base as dev

# Install ROS dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-noetic-moveit \
    ros-noetic-cartesian-control-msgs \
    ros-noetic-teleop-twist-keyboard \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Install Trossen robot arm software (For AMD64 architectures, not Raspberry Pi) (This step may take up to 15 minutes)
RUN sudo apt install curl
RUN curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
RUN chmod +x xsarm_amd64_install.sh
RUN ./xsarm_amd64_install.sh -d noetic -n

# Source Trossen installation on container startup
RUN echo "source ~/interbotix_ws/devel/setup.bash" >> ~/.bashrc

# Set the working directory in the container
WORKDIR /root/catkin_ws

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

# Install general dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    python3-catkin-tools \
    python3-zipp \
    python-is-python3 \
    libspnav-dev \
    spacenavd \
    ros-noetic-spacenav-node \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies
RUN pip install --upgrade pip
RUN pip install --upgrade \ 
    pyserial \
    pymodbus===2.1.0 \
    numpy \
    numpy-quaternion \
    scipy
RUN pip install --upgrade importlib_metadata
RUN pip install --upgrade six
RUN pip install --upgrade setuptools
RUN pip install --upgrade PyQt6

RUN pip install -r ./src/gello_software/requirements.txt
RUN pip install -e ./src/gello_software/.
RUN pip install -e ./src/gello_software/third_party/DynamixelSDK/python/.

# Build the ROS workspace
RUN source /opt/ros/noetic/setup.bash \
    && catkin build

# Source the workspace setup files on container startup
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Configure display access (Unsets variable. Setting it may cause Rviz to fail.)
RUN echo "export LIBGL_ALWAYS_INDIRECT=" >> ~/.bashrc