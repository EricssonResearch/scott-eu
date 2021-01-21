#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from turtlebot2i_edge.srv import Stamp, StampResponse


def stamp(request):
    rospy.loginfo('Stamped %d bytes' % len(request.bytes))
    response = StampResponse(header=Header(stamp=rospy.Time.now()))
    return response


def main():
    rospy.init_node('stamper')
    rospy.Service(
        name='stamp',
        service_class=Stamp,
        handler=stamp,
        buff_size=2**20     # 1 MB
    )
    rospy.loginfo('ROS service to stamp ready')
    rospy.spin()


if __name__ == '__main__':
    main()
