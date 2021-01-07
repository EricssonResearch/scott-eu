#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from turtlebot2i_edge import NetworkMonitor
from turtlebot2i_edge.msg import NetworkThroughput


def publish_network_throughput(network_monitor, publisher):
    throughput = network_monitor.measure_throughput()
    publisher.publish(
        header=Header(stamp=rospy.Time.now()),
        throughput=throughput
    )


def main():
    rospy.init_node('network_throughput_monitor')

    rospy.loginfo('Getting server parameters...')
    host = rospy.get_param('/network/mec_server/host')
    rospy.loginfo('MEC server: %s' % host)

    network_monitor = NetworkMonitor(host, throughput=True)
    rospy.loginfo('Publishing network throughput...')
    publisher = rospy.Publisher('network/throughput', NetworkThroughput, queue_size=1)

    rate = rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            publish_network_throughput(network_monitor, publisher)
            rate.sleep()
    except rospy.ROSException:
        pass


if __name__ == '__main__':
    main()
