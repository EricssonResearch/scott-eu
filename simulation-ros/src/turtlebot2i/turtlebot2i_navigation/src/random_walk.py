#!/usr/bin/env python

""" 
    Copyright 2018-03-02 Alberto Hata

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
"""

import rospy
import numpy as np
import actionlib
import random
import message_filters
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

sign = lambda a: int(a>0) - int(a<0)

def callback(scan_data):
    min_reading = 0.3
    range_max = 20
    fov = 180 

    critical_dist = 0.34
    warning_dist = 0.50

    #Select the index range that covers FOV in front of the robot
    #fov_min_idx = int(3.14 / scan_data.angle_increment)      # --> 180 deg FOV
    #fov_max_idx = int((2*3.14) / scan_data.angle_increment)  # --> 180 deg FOV
    fov_min_idx = int(3.14 * (7.0/6.0) / scan_data.angle_increment)      # --> 120 deg FOV
    fov_max_idx = int(3.14 * (11.0/6.0) / scan_data.angle_increment)  # --> 120 deg FOV

    # Discard measurements < min_readings (converted to inf)
    ranges = list(np.where(np.array(scan_data.ranges)<min_reading, np.inf, scan_data.ranges))

    sensor_reads = ranges[fov_min_idx:fov_max_idx]

    min_scan = min(sensor_reads)
    min_scan_idx  = np.argmin(sensor_reads)
    min_scan_angle = min_scan_idx + (-fov/2.0)

    vel_pub = Twist()

    if (min_scan <= critical_dist):
        vel_pub.linear.x = 0
        vel_pub.angular.z = max_vel_z
    elif (min_scan < warning_dist):
        vel_pub.linear.x = max_vel_x/2.0
        vel_pub.angular.z = max_vel_z * sign(-min_scan_angle) 
    else:
        vel_pub.linear.x = max_vel_x
        vel_pub.angular.z = 0

    #print(min_scan, min_scan_angle, vel_pub.angular.z, sign(min_scan_angle))

    pub.publish(vel_pub)


if __name__ == '__main__':
    rospy.init_node('random_walk')

    max_vel_x = rospy.get_param('max_vel_x', 0.2)
    max_vel_z = rospy.get_param('max_vel_x', 0.8)

    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    rospy.Subscriber("/scan", LaserScan, callback)

    rospy.spin()
