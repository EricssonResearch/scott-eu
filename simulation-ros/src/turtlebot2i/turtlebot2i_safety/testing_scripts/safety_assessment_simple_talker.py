#!/usr/bin/env python

import rospy
from turtlebot2i_safety.msg import SceneGraph 
#import SceneGraph 
import std_msgs.msg

def talker(): 
    pub = rospy.Publisher('/turtlebot2i_safety/SceneGraph', SceneGraph, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        rospy.loginfo("Publish a topic")
        sg_message=SceneGraph()
        sg_message.header = std_msgs.msg.Header()
        sg_message.header.stamp = rospy.Time.now()
 
        sg_message.object_distance = 5.0
        sg_message.object_direction = 0
        sg_message.object_speed  = 0.0
        sg_message.object_orientation = 0
        pub.publish(sg_message)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
