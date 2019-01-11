#!/usr/bin/env python3

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
# TODO
# Make the code work with cv2 from python2
# Import cv2 just from visualize_cv2 

# Force loading python 3 version of cv2
#import cv2
import importlib.util
spec = importlib.util.spec_from_file_location("cv2", "/usr/local/lib/python3.5/dist-packages/cv2/cv2.cpython-35m-x86_64-linux-gnu.so")
cv2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cv2)

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import numpy as np
from visualize_cv import model, display_instances, class_names

class ros_mask_rcnn:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/turtlebot2i/camera/rgb/raw_image", Image, self.callback)
        self.image_pub = rospy.Publisher("mrcnn_out", Image, queue_size=1)

    def callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            results = model.detect([cv_image], verbose=1)
            r = results[0]

            img_out = display_instances(
                cv_image, r['rois'], r['masks'], r['class_ids'], class_names, r['scores']
            )
            
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_out, "bgr8"))

        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('mask_rcnn_py')

    detector = ros_mask_rcnn()
    rospy.spin()
