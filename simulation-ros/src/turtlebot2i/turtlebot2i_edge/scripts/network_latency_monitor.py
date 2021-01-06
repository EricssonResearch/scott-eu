#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from turtlebot2i_edge import NetworkMonitor
from turtlebot2i_edge.msg import NetworkLatency


def publish_network_latency(network_monitor, publisher):
    rtt_min, rtt_avg, rtt_max, rtt_mdev, packet_loss = network_monitor.measure_latency()
    publisher.publish(
        header=Header(stamp=rospy.Time.now()),
        rtt_min=rtt_min,
        rtt_avg=rtt_avg,
        rtt_max=rtt_max,
        rtt_mdev=rtt_mdev,
        packet_loss=packet_loss
    )


def main():
    rospy.init_node('network_latency_monitor')

    rospy.loginfo('Getting server parameters...')
    host = rospy.get_param('/network/mec_server/host')
    rospy.loginfo('MEC server: %s' % host)

    publisher = rospy.Publisher('network_latency', NetworkLatency, queue_size=1)
    rospy.loginfo('Publishing network latency...')
    network_monitor = NetworkMonitor(host, throughput=False)

    rate = rospy.Rate(5)
    try:
        while not rospy.is_shutdown():
            publish_network_latency(network_monitor, publisher)
            rate.sleep()
    except rospy.ROSException:
        pass


if __name__ == '__main__':
    main()
