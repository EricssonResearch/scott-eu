#!/usr/bin/env python

"""
    Edited from navigation.py in turtlebot2i_navigation module
"""

import rospy
from geometry_msgs.msg import Twist
from turtlebot2i_safety.msg import VelocityScale
import message_filters

import numpy as np

def speed_callback(vel_data, scale_data):
    interWheelDistance = 0.137

    l_scale = scale_data.left_vel_scale
    r_scale = scale_data.right_vel_scale
    
    new_speed = Twist()

    if vel_data.linear.x < 0.0:
    	new_speed.linear.x  = max(vel_data.linear.x, -0.1)
    	new_speed.angular.z = vel_data.angular.z

    elif r_scale < 0.01 and l_scale < 0.01:
    	new_speed.linear.x  = 0.0
    	new_speed.angular.z = vel_data.angular.z

    else:
    	vel_l = (vel_data.linear.x - vel_data.angular.z * interWheelDistance) * l_scale
    	vel_r = (vel_data.linear.x + vel_data.angular.z * interWheelDistance) * r_scale
    	new_speed.linear.x  = (vel_r + vel_l) / 2.0 
    	new_speed.angular.z = (vel_r - vel_l) / interWheelDistance

    velocity_publisher.publish(new_speed)

    #print("[speed_callback] scaling speed received:",l_scale,r_scale,"| linear speed: ",new_speed.linear.x,"angular speed: ",new_speed.angular.z)
    

if __name__ == '__main__':
    rospy.init_node('speed_scaling_py')

    velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    vel_sub = message_filters.Subscriber('/mobile_base/commands/velocity_raw', Twist)
    vel_scale_sub = message_filters.Subscriber('/mobile_base/safety/vel_scale', VelocityScale)

    ts = message_filters.ApproximateTimeSynchronizer([vel_sub, vel_scale_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(speed_callback)

    rospy.spin()
