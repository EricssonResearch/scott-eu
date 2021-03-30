#!/usr/bin/env python
# This module is intended to convert lidar information into x-y data

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import std_msgs.msg
from turtlebot2i_safety.msg import SafetyZone
import math


def init_var():
    global speed, risk_val, zone_size_message, sensor_reads
    speed = 0.0
    risk_val = 0.0
    zone_size_message = SafetyZone()
    sensor_reads = [0.0] * 684  # n data from lidar (always 684 no matter the position)


def speed_callback(data):
    global speed
    speed = math.sqrt(data.twist.twist.linear.x ** 2 + data.twist.twist.linear.y ** 2)


def risk_callback(risk_value):
    global risk_val
    risk_val = risk_value.data


def lidar_callback(data):
    global speed, risk_val, zone_size_message, sensor_reads
    n_sensors = 675  # 684 #if lidar on top: 684 data, if lidar in front of robot: 675 data
    for i in range(4, n_sensors + 4):  # to ignore the first 4 data
        x = data.ranges[i] * math.cos(
            data.angle_min + data.angle_increment * i) + 0.2  # substraction for transforming data to the center of the robot
        y = data.ranges[i] * math.sin(data.angle_min + data.angle_increment * i)
        sensor_reads[i] = math.sqrt(x ** 2 + y ** 2)
    data.ranges = sensor_reads
    # sensor_reads = data.ranges[4:n_sensors+4]
    min_distance = min(sensor_reads[4:n_sensors + 4])
    # robot max speed = 0.52
    clear_zone_radius = 0.32 + 12 * speed / 10.0  # at max: 0.944
    warning_zone_radius = 0.31 + 6 * speed / 10.0  # at max: 0.622
    critical_zone_radius = 0.295  # Robot diameter= 0.4

    if risk_val == 0.0:
        available_space = min_distance
        if warning_zone_radius > available_space:
            warning_zone_radius = available_space - 0.002
            clear_zone_radius = available_space + 0.02
        elif clear_zone_radius > available_space:
            warning_zone_radius = warning_zone_radius
            clear_zone_radius = available_space + 0.02
    pub_zone_size(critical_zone_radius, warning_zone_radius, clear_zone_radius)
    lidar_pub.publish(data)


def pub_zone_size(critical_zone_radius, warning_zone_radius, clear_zone_radius):
    zone_size_message.header = std_msgs.msg.Header()
    zone_size_message.header.stamp = rospy.Time.now()
    zone_size_message.critical_zone_radius = critical_zone_radius
    zone_size_message.warning_zone_radius = warning_zone_radius
    zone_size_message.clear_zone_radius = clear_zone_radius
    safe_zone_pub.publish(zone_size_message)


if __name__ == '__main__':
    try:
        rospy.init_node('zone_generation_py')
        init_var()
        rospy.Subscriber('odom', Odometry, speed_callback)  # 12hz
        rospy.Subscriber('safety/risk_val', std_msgs.msg.Float64, risk_callback)
        rospy.Subscriber('lidar/scan', LaserScan, lidar_callback)  # 12hz
        safe_zone_pub = rospy.Publisher('safety/safety_zone', SafetyZone, queue_size=10)
        lidar_pub = rospy.Publisher('lidar/scan_transformed', LaserScan, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
