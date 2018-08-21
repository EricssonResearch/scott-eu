#!/usr/bin/env python

import rospy
from turtlebot2i_safety.msg import SceneGraph 
#import SceneGraph 
import std_msgs.msg

def sg_cb(data):
    rospy.loginfo("I got %f at %s",data.object_distance,data.header.stamp)

def listener(): 
    pub = rospy.Publisher('topic_name', SceneGraph, queue_size=10)
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/turtlebot2i_safety/SceneGraph", SceneGraph, sg_cb)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
