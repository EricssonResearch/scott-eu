#!/usr/bin/env python
# This module is intended to convert lidar information into x-y data

import rospy
from sensor_msgs.msg import LaserScan
import math

def lidar_callback(data):
    #print("lidar transformer")
    sensor_reads = data.ranges
    #ignore 4 first data and 5 last data
    n_sensors = 675 #684 #if lidar on top: 684 data, if lidar in front of robot: 675 data
    #sensor_reads = [0.0] * n_sensors
    sensor_reads = [0.0] * len(data.ranges)
    for i in range(4,n_sensors+4): #to ignore the first 4 data
        x = data.ranges[i]*math.cos(data.angle_min + data.angle_increment*i) + 0.2 #substraction for transforming data to the center of the robot
        y = data.ranges[i]*math.sin(data.angle_min + data.angle_increment*i) 
        sensor_reads[i] = math.sqrt(x**2 + y**2) 
    data.ranges = sensor_reads
    lidar_pub.publish(data)
    #print("published")


if __name__ == '__main__':
    try:
        rospy.init_node('lidar_transformer_py')
        #rospy.Subscriber('/turtlebot2i/lidar/scan_data_collection', LaserScan, lidar_callback) #this is used without updating cost map
        rospy.Subscriber('/turtlebot2i/lidar/scan', LaserScan, lidar_callback) #this is used to update the costmap
        lidar_pub = rospy.Publisher('/turtlebot2i/lidar/scan_transformed', LaserScan, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
