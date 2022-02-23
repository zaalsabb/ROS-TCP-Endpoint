FROM ros:noetic
# install Husarnet client
RUN apt update -y && \
    apt install -y curl && \
    apt install -y gnupg2 && \
    apt install -y systemd && \
    curl https://install.husarnet.com/install.sh | bash

RUN apt-get install -y iptables arptables ebtables
RUN update-alternatives --set ip6tables /usr/sbin/ip6tables-nft

# install webserver service
RUN apt install -y nginx

# some optional modules
RUN apt install -y vim
RUN apt install -y iputils-ping

# install build tools
RUN apt-get update && apt-get install -y \
      python3-catkin-tools \
      python-is-python3 \
      git \
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

# Find your JOINCODE at https://app.husarnet.com
ENV JOINCODE=""
ENV HOSTNAME=my-container-1

# HTTP PORT
EXPOSE 80

COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
