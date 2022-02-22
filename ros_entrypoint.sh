#!/bin/bash

function get_status() {
    local status="success"

    while read line; do
        if [[ $line == *"ERROR"* ]]; then
            status="waiting..."
        fi
    done
    echo $status
}

function get_ipv6() {
    local ipv6addr="::"
    
    while read line; do
        if [[ $line == *"Husarnet IP address:"* ]]; then
            ipv6addr=${line#*"Husarnet IP address: "}
        fi
    done
    
    echo $ipv6addr
}



sysctl net.ipv6.conf.lo.disable_ipv6=0

if [[ ${JOINCODE} == "" ]]; then
    echo ""
    echo "ERROR: no JOINCODE provided in \"docker run ... \" command. Visit https://app.husarnet.com to get a JOINCODE"
    echo ""
    /bin/bash
    exit
fi

echo ""
echo "⏳ [1/2] Initializing Husarnet Client:"
husarnet daemon > /dev/null 2>&1 &

for i in {1..10}
do
    sleep 1
    
    output=$( get_status < <(husarnet status) )
    echo "$output"
    
    if [[ $output != "waiting..." ]]; then
        break
    fi
done

echo ""
echo "🔥 [2/2] Connecting to Husarnet network as \"${HOSTNAME}\":"
husarnet join ${JOINCODE} ${HOSTNAME}
echo "done"
echo ""

#script to run with a host network
export ROS_IPV6=on
export ROS_MASTER_URI=http://master:11311

# setup ros environment
source $ROS_WS/devel/setup.bash
roslaunch ros_tcp_endpoint endpoint.launch public:=True port:=5000
