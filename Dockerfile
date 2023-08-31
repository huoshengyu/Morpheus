FROM thbarkouki/homestri-ur:minimal-v1.0.1

SHELL ["/bin/bash", "-c"]

WORKDIR /root/catkin_ws

# # Install python dependencies
# RUN pip3 install \ 
#     package1 \
RUN pip3 install \
      scipy \
      pybullet

# # Install ROS dependencies
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     package2 \
#     && rm -rf /var/lib/apt/lists/*

COPY ./ ./src/morpheus

RUN source /opt/ros/noetic/setup.bash && \
    apt-get update && rosdep install -q -y \
      --from-paths ./src \
      --ignore-src \
      --rosdistro noetic \
    && rm -rf /var/lib/apt/lists/*

# Build the ROS workspace
RUN source /opt/ros/noetic/setup.bash && \
    catkin build