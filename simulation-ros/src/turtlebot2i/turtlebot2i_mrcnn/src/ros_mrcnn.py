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
from graphviz import Digraph, Source
from shapely.geometry import box

import numpy as np
from visualize_cv import display_instances, class_names

# Root directory of the project
import os
import sys
ROOT_DIR = os.path.abspath(os.path.join(os.path.realpath(__file__), '../..')) 
LIB_PATH = os.path.join(ROOT_DIR, "mrcnn/lib/mask_rcnn-2.1-py3.6.1.egg/mask_rcnn-2.1-py3.6.1")
print (LIB_PATH)
sys.path.append(LIB_PATH)

dir(sys.modules[__name__])
# Import Mask RCNN
from mrcnn import model as modellib
from mrcnn.config import Config

# Path to trained weights file 
LOG_DIR = os.path.join(ROOT_DIR, "logs")
#MODEL_PATH = os.path.join(ROOT_DIR, "models/mask_rcnn_coco.h5")
MODEL_PATH = os.path.join(ROOT_DIR, "models/coco_vrepall_1002.h5")

class ShapesConfig(Config):
    """Configuration for training on the toy shapes dataset.
    Derives from the base Config class and overrides values specific
    to the toy shapes dataset.
    """
    # Give the configuration a recognizable name
    NAME = "vrepAll"

    # Train on 1 GPU and 8 images per GPU. We can put multiple images on each
    # GPU because the images are small. Batch size is 8 (GPUs * images/GPU).
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

    # Number of classes (including background)
    NUM_CLASSES = 1 + 8  # background + 3 shapes

    # Use small images for faster training. Set the limits of the small side
    # the large side, and that determines the image shape.
    IMAGE_MIN_DIM = 480
    IMAGE_MAX_DIM = 640

    # Use smaller anchors because our image and objects are small
    RPN_ANCHOR_SCALES = (8 * 4, 16 * 4, 32 * 4, 64 * 4, 128 * 4)  # anchor side in pixels

    # Reduce training ROIs per image because the images are small and have
    # few objects. Aim to allow ROI sampling to pick 33% positive ROIs.
    TRAIN_ROIS_PER_IMAGE = 40

    # Use a small epoch since the data is simple
    STEPS_PER_EPOCH = 100

    # use small validation steps since the epoch is small
    VALIDATION_STEPS = 5

class InferenceConfig(ShapesConfig):
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1


def depth_callback(image, depth_image):
    try:
        print ('inside callback')
        print (type(image),type(depth_image))

    except CvBridgeError as e:
        print(e)

class ros_mask_rcnn:

    def __init__(self):

        # Load model
        config = InferenceConfig()
        config.display()
        
        self.model = modellib.MaskRCNN(
            mode="inference", model_dir=LOG_DIR, config=config
        )
        
        self.model.load_weights(MODEL_PATH, by_name=True)

        # Set topics
        self.bridge = CvBridge()

        # Use ApproximateTimeSynchronizer if depth and rgb camera doesn't havse same timestamp, otherwise use Time Synchronizer if both cameras have same timestamp.
        self.image_sub = message_filters.Subscriber('/turtlebot2i/camera/rgb/raw_image', Image)
        self.image_depth_sub = message_filters.Subscriber('/turtlebot2i/camera/depth/raw_image', Image)
        #self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.image_depth_sub], queue_size=1, slop = 0.1)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.image_depth_sub], queue_size=1)
        print ('calling callback')

        self.ts.registerCallback(self.callback)

        #self.image_sub = rospy.Subscriber("/turtlebot2i/camera/rgb/raw_image", Image, self.callback)
        #self.image_depth_sub = rospy.Subscriber("/turtlebot2i/camera/depth/raw_image", Image, self.depth_callback)
        '''
        pubKinectDepth = simROS.advertise(robot_id..'/'..sensor_name..'/depth/raw_image','sensor_msgs/Image')
        simROS.publisherTreatUInt8ArrayAsString(pubKinectDepth)
        pubKinectCloud = simROS.advertise(robot_id..'/'..sensor_name..'/cloud','sensor_msgs/PointCloud2')
        simROS.publisherTreatUInt8ArrayAsString(pubKinectCloud) 
        '''

        self.image_pub = rospy.Publisher("/turtlebot2i/mrcnn_out", Image, queue_size=1)

    def get_overlap_bbox(self, rec1, rec2):
        #y1, x1, y2, x2 = boxes
        y1min, x1min, y1max, x1max = rec1[0], rec1[1], rec1[2], rec1[3]
        y2min, x2min, y2max, x2max = rec2[0], rec2[1], rec2[2], rec2[3]

        box1 = box(x1min, y1min, x1max, y1max)
        box2 = box(x2min, y2min, x2max, y2max)
        isOverlapping = box1.intersects(box2)
        intersectio_area = box1.intersection(box2).area/box1.area*100
        #print (pol_overl, intersectio_area)
        return isOverlapping, intersectio_area

        #isOverlapping = (i[1] < j[3] and j[1] < i[3] and i[0] < j[2] and j[0] < i[2])
        #isOverlapping = (x1min < x2max and x2min < x1max and y1min < y2max and y2min < y1max)
        #print (isOverlapping)
        #return isOverlapping

    def callback(self, image, depth_image):

        try:
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image)
            print (cv_depth_image.shape)
            #print (cv_depth_image[0], cv_depth_image)

            #print (cv_depth_image[:,:,0],cv_depth_image[:,:,1],cv_depth_image[:,:,2])

            cv_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
            results = self.model.detect([cv_image], verbose=1)
            cv2.imshow('frame', cv_depth_image)
            cv2.waitKey(0)
            # r = results[0]

            # img_out = display_instances(
            #     cv_image, r['rois'], r['masks'], r['class_ids'], class_names, r['scores']
            # )

            # # cv2.imshow('frame', img_out)
            # # cv2.waitKey(1)
            
            # if len(r['class_ids']) > 0:

            #     count_objects = [0] * len(class_names)
            #     detected_objects = []

            #     for i in range(len(r['class_ids'])):
            #         detected_objects.append(class_names[r['class_ids'][i]]+'#'+str(count_objects[r['class_ids'][i]]))
            #         count_objects[r['class_ids'][i]] += 1
            #         print ('Object : ',r['class_ids'][i], detected_objects[i], r['rois'][i])

            #     #print (detected_objects)
            #     dot = Digraph(comment='warehouse', format='svg')
            #     dot.node_attr['shape']='record'
            #     #robot_velocity = get_velocity(robot_list[robot_num])
            #     #robot_label = '{%s|%s|velocity: %.2f}'%(robot_list[robot_num].name, robot_list[robot_num].vision_sensor.name, robot_velocity)
            #     robot_label = "robocop"
                
            #     dot.node('robot', label=robot_label)
            #     dot.node('warehouse', label='warehouse')
            #     dot.node('floor', label='{floor|size: 25*25}')
            #     dot.edge('warehouse','floor')

            #     for i in range(len(r['class_ids'])):
            #         #_id = r['class_ids'][i]
            #         node_label = detected_objects[i]

            #         dot.node(detected_objects[i], label=node_label)
            #         if re.match(r'Wall*', detected_objects[i]):
            #             dot.edge('warehouse', detected_objects[i], label='on')
            #         elif re.match(r'Product*', detected_objects[i]):
            #             overlapping_check = False
            #             for j in range(len(r['class_ids'])):

            #                 if j != i:

            #                     isOverlapping, intersectio_area = self.get_overlap_bbox(r['rois'][i], r['rois'][j])
            #                     print ('Comparing :',detected_objects[i],' => ', detected_objects[j], ' Result: ', isOverlapping, ' Intersectio Area: ', intersectio_area)

            #                     if isOverlapping and intersectio_area > 50.0:                    
            #                         dot.edge(detected_objects[j], detected_objects[i], label='on')
            #                         overlapping_check = True
            #                         break
            #             if overlapping_check == False:
            #                 dot.edge('floor', detected_objects[i], label='on')
            #         else:
            #             dot.edge('floor', detected_objects[i], label='on')

            #     s = Source(dot, filename="test.gv", format="png")
            #     s.view()
                

            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_out, "bgr8"))

        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('mask_rcnn_py')

    detector = ros_mask_rcnn()
    rospy.spin()
