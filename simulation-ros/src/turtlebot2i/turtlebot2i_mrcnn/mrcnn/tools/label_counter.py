#!/usr/bin/env python
# -*- coding: utf-8 -*-

# this script is used to count the numbers of every objects in label file.
# it will look through all the labels in one certain dataset, which could 
# do some help for deceding which object should be labeled more.  

import os
import yaml

# get dataset path, change the path if necessary
ROOT_DIR = os.getcwd()
dataset_root_path = os.path.join(ROOT_DIR, "trainData4/")

# get the number of images
img_floder = dataset_root_path + "rgb"
imglist = os.listdir(img_floder)
count = len(imglist)

print("Get dataset from", dataset_root_path)
print("There are", count, "images in this dataset")
print("-----------------------------------------------------")

# initialize number of each object
SlidingDoor_num = 0
Wall_num = 0
Shelf_num =0
Robot_num = 0
Human_num = 0
ConveyorBelt_num = 0
DockStation_num = 0
Product_num = 0
other_num = 0

# use two for loop to count numbers of objects
# this loop count all the images
for i in range(count):
    # read yaml file (label message), which get from labelme 
    filestr = imglist[i].split(".")[0]
    yaml_path = dataset_root_path + "labelme_json/" + filestr + "_json/info.yaml"
    with open(yaml_path) as f:
        temp = yaml.load(f.read())
        labels = temp['label_names']
        # delete the first one: background
        del labels[0]

    # this loop count all objects in one image
    for i in range(len(labels)):
            if labels[i].find("SlidingDoor") != -1:
                SlidingDoor_num += 1
            elif labels[i].find("Wall") != -1:
                Wall_num += 1
            elif labels[i].find("Shelf") != -1:
                Shelf_num += 1
            elif labels[i].find("Robot") != -1:
                Robot_num += 1
            elif labels[i].find("Human") != -1:
                Human_num += 1
            elif labels[i].find("ConveyorBelt") != -1:
                ConveyorBelt_num += 1
            elif labels[i].find("DockStation") != -1:
                DockStation_num += 1
            elif labels[i].find("Product") != -1:
                Product_num += 1
            else:
                other_num += 1

# print all the results, OTHERS should always be 0
print("Number of SlidingDoor labeled is ", SlidingDoor_num)
print("Number of Wall labeled is ", Wall_num)
print("Number of Shelf labeled is ", Shelf_num)
print("Number of Robot labeled is ", Robot_num)
print("Number of Human labeled is ", Human_num)
print("Number of ConveyorBelt labeled is ", ConveyorBelt_num)
print("Number of DockStation labeled is ", DockStation_num)
print("Number of Product labeled is ", Product_num)
print("Number of OTHERS labeled is ", other_num)
