#!/usr/bin/env python
# This module is intended to convert lidar information into x-y data

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import std_msgs.msg
from turtlebot2i_safety.msg import SafetyZone, VelocityScale, SafetyRisk
import math
from kobuki_msgs.msg import BumperEvent, Led
import numpy as np

def init_var():
    global speed, risk_val, zone_size_message, sensor_reads, led_msg
    speed = 0.0
    risk_val = 0.0
    zone_size_message = SafetyZone()
    sensor_reads = [0.0] * 684 #n data from lidar (always 684 no matter the position)
    led_msg = Led()

def risk_callback(data):
	global led_msg
	max_risk = max(data.risk_value)
	if max_risk > 3.0:
		led_value = 3
	elif max_risk > 2.0:
		led_value = 2
	elif max_risk > 1.0:
		led_value = 1
	else:
		led_value = 0
	led_msg.value = led_value
	#print(data.risk_value,led_value)
	led_pub.publish(led_msg)

def speed_callback(data):
	global led_msg
	if data.left_vel_scale == 1.0 and data.right_vel_scale == 1.0:
		led_msg.value = 0
		led_pub.publish(led_msg)
    

if __name__ == '__main__':
    try:
        rospy.init_node('connect_to_robot_py')
        init_var()
        led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
        rospy.Subscriber('/turtlebot2i/safety/obstacles_risk', SafetyRisk, risk_callback)
        rospy.Subscriber('/turtlebot2i/safety/vel_scale', VelocityScale, speed_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
