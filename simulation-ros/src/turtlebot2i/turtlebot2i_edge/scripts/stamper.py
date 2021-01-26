#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from turtlebot2i_edge.srv import Stamp, StampResponse


def stamp(request):
    to_stamp = len(request.bytes)
    rospy.loginfo('Stamped %d bytes' % to_stamp)
    return StampResponse(
        header=Header(stamp=rospy.Time.now()),
        stamped=request.to_stamp
    )


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
