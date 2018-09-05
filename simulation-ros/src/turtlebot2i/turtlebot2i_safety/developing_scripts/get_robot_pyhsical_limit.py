#!/usr/bin/env python

## This code aims to get some important parameters of robot.
# It should both work for simulator and real robot
# It will send a send a velocity command for 1 sec, so that robot is running with highest speed.
# Then it will send a stop command.
# The robot position is monitored, and the stopping time and stopping distance can be found.

## rqt_plot /turtle1/pose/x:y:z

import std_msgs.msg
import rospy # ROS library

def init():


def topic_callback(data):


""" Main program """
if __name__ == "__main__":  
    init()
    rospy.init_node("prase_ros_node",anonymous=True) #Always first

    ## SUBSCRIBERS
    # Creates a subscriber object for each topic
    rospy.Subscriber('/turtlebot2i/safety/scene_graph', SceneGraph, topic_callback)
    ## PUBLISHERS
    #
    pub = rospy.Publisher('/turtlebot2i/safety/safety_zone', SafetyZone, queue_size=10)
    
    rospy.spin()




