# Make new train set
After getting rosbag from ros:
- run `bag2cv.py` to get images in one certain ros topic. 
  
  *you need to change bag name, topic name, etc. in this script.*
- use [labelme](https://github.com/wkentaro/labelme) to label the images and save the json files. You can use command like `mv *.json ./labelme_json/` to move all json files to one folder.

- after getting json file from labelme and putting them in one folder, run `json2dataset.sh` to get json folder. 

  *you need to change json file path in this script.*
- use `mask2cv.py` to convert the 16 bits mask image to 8 bits, which opencv could read.

  *you need to change image path in this script.*
