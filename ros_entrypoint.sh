#!/bin/bash

#script to run with a host network
export ROS_IPV6=on
export ROS_MASTER_URI=http://master:11311

# setup ros environment
source $ROS_WS/devel/setup.bash
roslaunch ros_tcp_endpoint endpoint.launch public:=True port:=5000
