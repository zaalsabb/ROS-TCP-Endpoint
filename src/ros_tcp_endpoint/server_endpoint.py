#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", "TCPServer")

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    public = rospy.get_param("~public",False)
    tcp_port = rospy.get_param("~port",10000)

    tcp_server = TcpServer(ros_node_name, public=public, tcp_port=tcp_port)

    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
