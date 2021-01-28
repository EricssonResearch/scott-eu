#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from turtlebot2i_edge import NetworkMonitor
from turtlebot2i_edge.srv import MeasureNetwork, MeasureNetworkResponse


def measure_network(network_monitor, _):
    # important: RTT before throughput
    # otherwise, in case of timeout for the throughput, there might be packets still in the network that affect the RTT
    rtt = network_monitor.measure_rtt()
    throughput = network_monitor.measure_throughput()
    rospy.loginfo('Network measured')
    return MeasureNetworkResponse(
        header=Header(stamp=rospy.Time.now()),
        rtt=rtt,
        throughput=throughput
    )


def main():
    rospy.init_node('network_latency_monitor')

    rospy.loginfo('Getting server parameters...')
    host = rospy.get_param('/network/mec_server/host')
    rospy.loginfo('MEC server: %s' % host)

    network_monitor = NetworkMonitor(host, ns3_simulation=True)
    rospy.Service(
        name='measure_network',
        service_class=MeasureNetwork,
        handler=lambda request: measure_network(network_monitor, request),
    )
    rospy.loginfo('ROS service to measure network ready')

    rospy.spin()


if __name__ == '__main__':
    main()
