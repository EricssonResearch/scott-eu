#!/usr/bin/env python

from __future__ import division

import rospy
from std_msgs.msg import Header
from turtlebot2i_edge import NetworkMonitor
from turtlebot2i_edge.srv import MeasureNetwork, MeasureNetworkResponse


def measure_network(network_monitor, request):
    # important: RTT before throughput
    # otherwise, in case of timeout for the throughput, there might be packets still in the network that affect the RTT
    rtt = network_monitor.measure_rtt(max_rtt=request.max_rtt.to_sec())
    throughput = network_monitor.measure_throughput(max_duration=request.max_duration_throughput.to_sec())
    rospy.loginfo('Network measured')
    return MeasureNetworkResponse(
        header=Header(stamp=rospy.Time.now()),
        rtt=rospy.Duration.from_sec(rtt / 1e3),
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
