FROM ros:melodic

# install build tools
RUN apt-get update && apt-get install -y \
      python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# clone ros package repo
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
RUN git -C src clone \
      -b main \
      https://github.com/zaalsabb/ROS-TCP-Endpoint.git

# install ros package dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install -y \
      --from-paths \
        src/ROS-TCP-Endpoint \
      --ignore-src && \
    rm -rf /var/lib/apt/lists/*

# build ros package source
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin build \
      ros_tcp_endpoint

# source ros package from entrypoint
#RUN sed --in-place --expression \
#      '$isource "$ROS_WS/devel/setup.bash"' \
#      /ros_entrypoint.sh

# setup ROS networking
#RUN export ROS_MASTER_URI=http://$(hostname --ip-address):11311
#RUN export ROS_HOSTNAME=$(hostname --ip-address)

# run ros package launch file
#CMD ["roslaunch", "ros_tcp_endpoint", "endpoint.launch", "port:=5000"]

COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
