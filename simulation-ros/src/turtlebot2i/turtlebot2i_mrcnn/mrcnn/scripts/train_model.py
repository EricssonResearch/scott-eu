#!/usr/bin/env python
# -*- coding: utf-8 -*-

# this scipt is to train model for Vrep objects, 
# change train dataset path 'dataset_root_path' in line 182 (approximately),
# or simply put your data at folder 'train_data' similar as example
# Shaolei Wang, 2018-10-15

import os
import sys
import random
import math
import re
import time
import numpy as np
import cv2
import matplotlib
import matplotlib.pyplot as plt
import tensorflow as tf
from mrcnn.config import Config
from mrcnn import model as modellib,utils
from mrcnn import visualize
import yaml
#TODO: load mrcnn from the local lib 
sys.path.append('lib/mask_rcnn-2.1-py3.6.1.egg')
from mrcnn.model import log
from PIL import Image

# Root directory of the project
ROOT_DIR = os.getcwd()

# Directory to save logs and trained model
MODEL_DIR = os.path.join(ROOT_DIR, "logs")

# Local path to trained weights file
COCO_MODEL_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco.h5")
# Download COCO trained weights from Releases if needed
if not os.path.exists(COCO_MODEL_PATH):
    utils.download_trained_weights(COCO_MODEL_PATH)


class VrepConfig(Config):
    """Configuration for training on the toy shapes dataset.
    Derives from the base Config class and overrides values specific
    to the toy shapes dataset.
    """
    # Give the configuration a recognizable name
    NAME = "VrepObjects"

    # Train on 1 GPU and 8 images per GPU. We can put multiple images on each
    # GPU because the images are small. Batch size is 8 (GPUs * images/GPU).
    GPU_COUNT = 1
    IMAGES_PER_GPU = 2

    # Number of classes (including background)
    NUM_CLASSES = 1 + 8  # background + 8 different type objects

    # Use small images for faster training. Set the limits of the small side
    # the large side, and that determines the image shape.
    IMAGE_MIN_DIM = 480
    IMAGE_MAX_DIM = 640

    # anchor side in pixels
    RPN_ANCHOR_SCALES = (8 * 4, 16 * 4, 32 * 4, 64 * 4, 128 * 4)  

    # Reduce training ROIs per image because the images are small and have
    # few objects. Aim to allow ROI sampling to pick 33% positive ROIs.
    TRAIN_ROIS_PER_IMAGE = 40 

    # Use a small epoch since the data is simple
    STEPS_PER_EPOCH = 100

    # use small validation steps since the epoch is small
    VALIDATION_STEPS = 5 

config = VrepConfig()
config.display()

class VrepData(utils.Dataset):
    def get_obj_index(self, image):
        n = np.max(image)
        return n

    def from_yaml_get_class(self, image_id):
        info = self.image_info[image_id]
        with open(info['yaml_path']) as f:
            temp = yaml.load(f.read())
            labels = temp['label_names']
            del labels[0]
        return labels

    # rewrite draw_mask
    def draw_mask(self, num_obj, mask, image,image_id):
        info = self.image_info[image_id]
        for index in range(num_obj):
            for i in range(info['width']):
                for j in range(info['height']):
                    # add for debug
                    #print("image_id-->",image_id,"-i--->",i,"-j--->",j)
                    #print("info[width]----->",info['width'],"info[height]--->",info['height'])
                    at_pixel = image.getpixel((i, j))
                    if at_pixel == index + 1:
                        mask[j, i, index] = 1
        return mask

    # rewrite load_shapes，contain own objects class
    def load_shapes(self, count, img_floder, mask_floder, imglist, dataset_root_path):
        """Generate the requested number of synthetic images.
        count: number of images to generate.
        height, width: the size of the generated images.
        """
        # Add classes
        self.add_class("shapes", 1, "SlidingDoor")
        self.add_class("shapes", 2, "Wall")
        self.add_class("shapes", 3, "Shelf")
        self.add_class("shapes", 4, "Robot")
        self.add_class("shapes", 5, "Human")
        self.add_class("shapes", 6, "ConveyorBelt")
        self.add_class("shapes", 7, "DockStation")
        self.add_class("shapes", 8, "Product")

        for i in range(count):
            filestr = imglist[i].split(".")[0]
            # add for debug 
            # print(imglist[i],"-->",cv_img.shape[1],"--->",cv_img.shape[0])
            # print("id-->", i, " imglist[", i, "]-->", imglist[i],"filestr-->",filestr)
            # if use different name, consider using this 
            # filestr = filestr.split("_")[1]
            mask_path = mask_floder + "/" + filestr + ".png"
            yaml_path = dataset_root_path + "labelme_json/" + filestr + "_json/info.yaml"
            print(dataset_root_path + "labelme_json/" + filestr + "_json/img.png")
            cv_img = cv2.imread(dataset_root_path + "labelme_json/" + filestr + "_json/img.png")

            self.add_image("shapes", image_id=i, path=img_floder + "/" + imglist[i],
                           width=cv_img.shape[1], height=cv_img.shape[0], mask_path=mask_path, yaml_path=yaml_path)

    # rewrite load_mask
    def load_mask(self, image_id):
        """Generate instance masks for shapes of the given image ID.
        """
        # add for debug
        # print("image_id",image_id)
        info = self.image_info[image_id]
        img = Image.open(info['mask_path'])
        num_obj = self.get_obj_index(img)
        mask = np.zeros([info['height'], info['width'], num_obj], dtype=np.uint8)
        mask = self.draw_mask(num_obj, mask, img,image_id)

        labels = []
        labels = self.from_yaml_get_class(image_id)
        labels_form = []
        for i in range(len(labels)):
            if labels[i].find("SlidingDoor") != -1:
                labels_form.append("SlidingDoor")
            elif labels[i].find("Wall") != -1:
                labels_form.append("Wall")
            elif labels[i].find("Shelf") != -1:
                labels_form.append("Shelf")
            elif labels[i].find("Robot") != -1:
                labels_form.append("Robot")
            elif labels[i].find("Human") != -1:
                labels_form.append("Human")
            elif labels[i].find("ConveyorBelt") != -1:
                labels_form.append("ConveyorBelt")
            elif labels[i].find("DockStation") != -1:
                labels_form.append("DockStation")
            elif labels[i].find("Product") != -1:
                labels_form.append("Product")                            
        class_ids = np.array([self.class_names.index(s) for s in labels_form])
        return mask, class_ids.astype(np.int32)

# basic config
dataset_root_path = os.path.join(ROOT_DIR, "train_data")
img_floder = dataset_root_path + "rgb"
mask_floder = dataset_root_path + "mask"
imglist = os.listdir(img_floder)
count = len(imglist)

print("Get dataset from", dataset_root_path)
print(count, "images are loaded for training")

#prepare train and val dataset
dataset_train = VrepData()
dataset_train.load_shapes(count, img_floder, mask_floder, imglist, dataset_root_path)
dataset_train.prepare()

#print("dataset_train-->",dataset_train._image_ids)

dataset_val = VrepData()
dataset_val.load_shapes(7, img_floder, mask_floder, imglist, dataset_root_path)
dataset_val.prepare()

#print("dataset_val-->",dataset_val._image_ids)

# Load and display random samples
image_ids = np.random.choice(dataset_train.image_ids, 4)
for image_id in image_ids:
    image = dataset_train.load_image(image_id)
    mask, class_ids = dataset_train.load_mask(image_id)
    visualize.display_top_masks(image, mask, class_ids, dataset_train.class_names)
   
# Create model in training mode
model = modellib.MaskRCNN(mode="training", config=config,
                          model_dir=MODEL_DIR)

# Which weights to start with?
# use imagenet, coco, or last
init_with = "coco" 

if init_with == "imagenet":
    model.load_weights(model.get_imagenet_weights(), by_name=True)
elif init_with == "coco":
    # Load weights trained on MS COCO, but skip layers that
    # are different due to the different number of classes
    # See README for instructions to download the COCO weights
    model.load_weights(COCO_MODEL_PATH, by_name=True,
                       exclude=["mrcnn_class_logits", "mrcnn_bbox_fc",
                                "mrcnn_bbox", "mrcnn_mask"])
elif init_with == "last":
    # Load the last model you trained and continue training
    model.load_weights(model.find_last()[1], by_name=True)

# Train the head branches
# Passing layers="heads" freezes all layers except the head
# layers. You can also pass a regular expression to select
# which layers to train by name pattern.
time_start = time.time()

model.train(dataset_train, dataset_val,
            learning_rate=config.LEARNING_RATE,
            epochs=10,
            layers='heads')

# Fine tune all layers
# Passing layers="all" trains all layers. You can also
# pass a regular expression to select which layers to
# train by name pattern.
model.train(dataset_train, dataset_val,
            learning_rate=config.LEARNING_RATE / 10.0,
            epochs=30,
            layers="all")

time_end = time.time()
time_cost_training = time_end - time_start

minute_cost, second_cost = divmod(time_cost_training, 60)
hour_cost, minute_cost = divmod(minute_cost, 60)

print("--------------------------------------------------------------------------")
print("Training time cost for this model is %d hours %d minutes %d seconds" % (hour_cost, minute_cost, second_cost))
print("Finish! Save the model at", MODEL_DIR)

