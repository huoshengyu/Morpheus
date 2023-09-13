FROM thbarkouki/homestri-ur:minimal-v1.0.1

SHELL ["/bin/bash", "-c"]

WORKDIR /root/catkin_ws

# Install git
RUN apt-get update && apt-get install git -y

# # Install python dependencies
# RUN pip3 install \ 
#     package1 \
RUN pip3 install \
      scipy \
      pybullet \
      urdf_parser_py

# Install ROS packages
# RUN

# # Install ROS dependencies
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     package2 \
#     && rm -rf /var/lib/apt/lists/*

COPY ./ ./src/morpheus

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
RUN source /opt/ros/noetic/setup.bash \
    && catkin build -s

# Installation for ros_pybullet_interface
RUN git clone -b main https://github.com/ros-pybullet/ros_pybullet_interface.git /root/catkin_ws/src/ros_pybullet_interface
RUN source /root/catkin_ws/devel/setup.bash
RUN apt update
RUN cd /root/catkin_ws/src/ros_pybullet_interface/
#RUN bash install.sh # Might need to run catkin build -s -j1 to avoid crash