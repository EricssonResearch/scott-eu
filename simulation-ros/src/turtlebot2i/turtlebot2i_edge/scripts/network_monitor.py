#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from turtlebot2i_edge import NetworkMonitor
from turtlebot2i_edge.msg import NetworkLatency, NetworkThroughput


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


def publish_network_throughput(network_monitor, publisher):
    throughput = network_monitor.measure_throughput()
    publisher.publish(
        header=Header(stamp=rospy.Time.now()),
        throughput=throughput
    )


def main():
    rospy.init_node('network_latency_monitor')

    rospy.loginfo('Getting server parameters...')
    host = rospy.get_param('/network/mec_server/host')
    throughput = rospy.get_param('~throughput')
    rospy.loginfo('MEC server: %s' % host)
    rospy.loginfo('Throughput: %s' % str(throughput))

    network_monitor = NetworkMonitor(host, throughput=throughput)
    pub_latency = rospy.Publisher('network/latency', NetworkLatency, queue_size=1)
    pub_throughput = rospy.Publisher('network/throughput', NetworkThroughput, queue_size=1) if throughput else None

    rospy.loginfo('Publishing network state...')
    rate = rospy.Rate(5)
    try:
        while not rospy.is_shutdown():
            publish_network_latency(network_monitor, pub_latency)
            if throughput:
                publish_network_throughput(network_monitor, pub_throughput)
            rate.sleep()
    except rospy.ROSException:
        pass


if __name__ == '__main__':
    main()
