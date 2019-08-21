#!/usr/bin/env python3

""" 
    Copyright 2019-07-24 Hassam Riaz
    
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
# TODO
# Make the code work with cv2 from python2
# Import cv2 just from visualize_cv2 

# Force loading python 3 version of cv2
import importlib.util
spec = importlib.util.spec_from_file_location("cv2", "/usr/local/lib/python3.5/dist-packages/cv2/python-3.5/cv2.cpython-35m-x86_64-linux-gnu.so")
cv2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cv2)
#import cv2

import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import re
#from graphviz import Digraph, Source
#from shapely.geometry import box

import numpy as np
from visualize_cv import display_instances, class_names


class ros_mask_rcnn:

    def __init__(self):
        self.IMAGE_COUNT = 1

        # Set topics
        self.bridge = CvBridge()

        # Use ApproximateTimeSynchronizer if depth and rgb camera doesn't havse same timestamp, otherwise use Time Synchronizer if both cameras have same timestamp.
        self.image_sub = message_filters.Subscriber('/turtlebot2i/camera/rgb/raw_image', Image)
        self.image_depth_sub = message_filters.Subscriber('/turtlebot2i/camera/depth/raw_image', Image)
        #self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.image_depth_sub], queue_size=1, slop = 0.1)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.image_depth_sub], queue_size=1)
        print ('calling callback')
        self.ts.registerCallback(self.depth_callback)

        self.image_pub = rospy.Publisher("/turtlebot2i/mrcnn_out", Image, queue_size=1)


    def depth_callback(self, image, depth_image):
        try:

            farClippingPlane = 3.5
            nearClippingPlane = 0.0099999
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image,"passthrough")
            cv_depth_image = cv2.flip(cv_depth_image, 0)
            #print ("Depth Image size: ", cv_depth_image.shape)
            #print ('min', min(cv_depth_image))
            #cv2.imshow("depth image", cv_depth_image)
            #cv2.waitKey(0)

            cv_depth_image_processed = nearClippingPlane  + (cv_depth_image * (farClippingPlane - nearClippingPlane))

            #print ("Depth Image size: ", cv_depth_image_processed.shape)
            cv_image = self.bridge.imgmsg_to_cv2(image, "rgb8")

            #cv2.imshow("RGB image", cv_image)
            #cv2.waitKey(0)


            cv2.imwrite("/home/etrrhmd/dataset_images/depth_images/rgb_"+str(self.IMAGE_COUNT)+".png",cv_image)
            np.save("/home/etrrhmd/dataset_images/depth_images/depth_"+str(self.IMAGE_COUNT)+".npy",cv_depth_image)
            np.save("/home/etrrhmd/dataset_images/depth_images/depth_processed_"+str(self.IMAGE_COUNT)+".npy",cv_depth_image_processed)
            # #cv2.imwrite("/home/etrrhmd/dataset_images/depth_images/depth_"+str(self.IMAGE_COUNT)+".png",cv_depth_image)
            # image = cv2.imread("/home/etrrhmd/dataset_images/depth_images/rgb_"+str(self.IMAGE_COUNT)+".png")
            # cv2.imshow("Rgb image", image)
            # cv2.waitKey(0)

            # dep = np.load("/home/etrrhmd/dataset_images/depth_images/depth_"+str(self.IMAGE_COUNT)+".npy")
            # cv2.imshow("Depth image", dep)
            # cv2.waitKey(0)

            # dep = np.load("/home/etrrhmd/dataset_images/depth_images/depth_processed_"+str(self.IMAGE_COUNT)+".npy")
            # cv2.imshow("Depth image", dep)
            # cv2.waitKey(0)

            self.IMAGE_COUNT+=1

        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('mask_rcnn_py')

    detector = ros_mask_rcnn()
    rospy.spin()
