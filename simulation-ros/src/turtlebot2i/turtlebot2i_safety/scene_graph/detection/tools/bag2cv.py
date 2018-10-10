#!/usr/bin/python
# -*- coding: UTF-8 -*-

# Extract images from a bag file.

#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

# Reading bag filename from command line or roslaunch parameter.
#import os
#import sys

'''
enwhsaa@elx77440xgh:~/GetTrain$ rostopic list
/keyop/teleop
/rosout
/rosout_agg
/tf
/turtlebot2i/camera/cloud
/turtlebot2i/camera/depth/raw_image
/turtlebot2i/camera/rgb/raw_image
/turtlebot2i/camera_sr300/cloud
/turtlebot2i/camera_sr300/depth/raw_image
/turtlebot2i/camera_sr300/rgb/raw_image
/turtlebot2i/commands/motor_power
/turtlebot2i/commands/velocity
/turtlebot2i/events/bumper
/turtlebot2i/events/cliff
/turtlebot2i/events/wheel_drop
/turtlebot2i/lidar/scan
/turtlebot2i/odom
/turtlebot2i/sensors/core
/turtlebot2i/sensors/dock_ir

'''

class ImageCreator():


    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag('test001.bag', 'r') as bag:  #rosbag file name
            for topic,msg,t in bag.read_messages():
                if topic == "/turtlebot2i/camera/rgb/raw_image": #name of ros topic, from this topic getting images  
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f means the precision of the time 
                        image_name = timestr+ ".jpg" #image name: time.jpg
                        cv2.imwrite(image_name, cv_image)  #save image

if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass

