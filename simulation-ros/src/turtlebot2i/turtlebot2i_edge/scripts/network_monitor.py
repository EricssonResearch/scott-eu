#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from turtlebot2i_edge import NetworkMonitor
from turtlebot2i_edge.srv import MeasureNetwork, MeasureNetworkResponse


def measure_network(network_monitor, _):
    # rtt_min, rtt_avg, rtt_max, rtt_mdev, packet_loss = network_monitor.measure_latency()
    rtt_min = rtt_avg = rtt_max = rtt_mdev = packet_loss = 0
    throughput = network_monitor.measure_throughput()
    rospy.loginfo('Network measured')
    return MeasureNetworkResponse(
        header=Header(stamp=rospy.Time.now()),
        rtt_min=rtt_min,
        rtt_avg=rtt_avg,
        rtt_max=rtt_max,
        rtt_mdev=rtt_mdev,
        packet_loss=packet_loss,
        throughput=throughput
    )


def main():
    rospy.init_node('network_latency_monitor')

    rospy.loginfo('Getting server parameters...')
    host = rospy.get_param('/network/mec_server/host')
    rospy.loginfo('MEC server: %s' % host)

    network_monitor = NetworkMonitor(host)
    rospy.Service(
        name='measure_network',
        service_class=MeasureNetwork,
        handler=lambda request: measure_network(network_monitor, request),
    )
    rospy.loginfo('ROS service to measure network ready')

    rospy.spin()


if __name__ == '__main__':
    main()
