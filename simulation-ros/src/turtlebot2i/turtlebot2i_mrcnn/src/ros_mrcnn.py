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
spec = importlib.util.spec_from_file_location("cv2", "/usr/local/lib/python3.5/dist-packages/cv2/cv2.cpython-35m-x86_64-linux-gnu.so")
cv2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cv2)

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import numpy as np
from visualize_cv import model, display_instances, class_names

# Root directory of the project
ROOT_DIR = os.path.abspath("../")
sys.path.append(ROOT_DIR + 'mrcnn/lib')

# Import Mask RCNN
from mrcnn import model as modellib
from mrcnn.config import Config

# Path to trained weights file 
MODEL_DIR = os.path.join(ROOT_DIR, "processVideo/logs")
#MODEL_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco.h5")
MODEL_PATH = os.path.join(ROOT_DIR, "logs/imagenet_vrepall_1003.h5")

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


class ros_mask_rcnn:

    def __init__(self):

        # Set topics
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/turtlebot2i/camera/rgb/raw_image", Image, self.callback)
        self.image_pub = rospy.Publisher("/turtlebot2i/mrcnn_out", Image, queue_size=1)

        # Load model
        config = InferenceConfig()
        config.display()
        
        model = modellib.MaskRCNN(
            mode="inference", model_dir=MODEL_DIR, config=config
        )
        
        model.load_weights(MODEL_PATH, by_name=True)

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
