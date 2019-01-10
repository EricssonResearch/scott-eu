#!/usr/bin/env python

from PIL import Image
import numpy as np
import math
import os

# change the path for your own dataset
# path is where your json folder, folder name should be rgb_1_json
# new path is the folder to save new 8 bits images
path = "/home/enwhsaa/GetTrain/TrainSetDoor/labelme_json/"
newpath = "/home/enwhsaa/GetTrain/TrainSetDoor/mask8bit/"

def toeight():
    sum = 0
    filelist = os.walk(path)
    for dirpath, dirnames, filenames in filelist:
        for file in filenames:
            if file == 'label.png':
                print(dirpath)
                whole_path = os.path.join(dirpath, file)
                img = Image.open(whole_path)  
                img = np.array(img)
                img = Image.fromarray(np.uint8(img))
                sum = sum + 1
                # rename the new 8 bits image to <number>.png
                # <number> is same as the number in json folder name rgb_<number>_json
                name = dirpath.split("/")[-1]
                newname = name.split("_")[1]
                img.save(newpath + newname + '.png', format='png')
    print(sum)

# This one can convert the all images in one folder
# def toeight():
#     filelist = os.listdir(path) 
#     for file in filelist:
#         whole_path = os.path.join(path, file)
#         img = Image.open(whole_path) 
#         img = np.array(img)
#         # img = Image.fromarray(np.uint8(img / float(math.pow(2, 16) - 1) * 255))
#         img = Image.fromarray(np.uint8(img))
#         img.save(newpath + file)

toeight()