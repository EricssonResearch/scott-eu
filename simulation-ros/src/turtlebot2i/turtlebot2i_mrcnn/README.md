# 1. Overview 

This ROS package makes possible perform object detection and segmentation using Mask R-CNN method.   

# 2. Mask R-CNN for Object Detection and Segmentation

Mask R-CNN is the state-of-art method for obstacle detection and recognition. It is based on the region-based convolutional neural network (R-CNN) and has the main characteristic of returning the boundary of the detected object (mask).
The network is based on Feature Pyramid Network (FPN) and a ResNet101 backbone.

The original Mask R-CNN repository can be found in this link: https://github.com/matterport/Mask_RCNN

The Mask R-CNN paper can be accessed through this link: https://arxiv.org/abs/1703.06870

# 3. Running ROS node

This package makes possible running Mask R-CNN through a ROS node.
Before running it, some instructions should have to be followed.

## 3.1 Porting MRCNN to ROS

The following guideline is intended to make the Mask R-CNN run inside ROS. The approch shown here makes MRCNN node run using python 3.5, so basically all ROS imports are adapted to support python 3.5.

1. Prepare ROS for python 3.5:

```
sudo apt-get install python3-yaml
sudo pip3 install rospkg catkin_pkg
```

2. Install OpenCV for python 3.5:

```
pip3 install opencv-python
```

3. Install cv_bridge
Follow the instructions in https://stackoverflow.com/questions/49221565/unable-to-use-cv-bridge-with-ros-kinetic-and-python3

4. Install MRCNN library (install the required depences):

```
sudo pip3 install tensorflow
```

5. The Pretrained MRCNN models is in models folder.


## 3.2. Running Mask R-CNN node

1. Set the model path in the ros_mrcnn.py code. To do so, change the `LOG_DIR` and `MODEL_PATH` variables:

```
rosed turtlebot2i_mrcnn ros_mrcnn.py 
```

2. In the terminal that the ros_mrcnn will run, it must be unloaded the ROS environment setup. It is necessary to prevent loading python2.7 in this terminal.
To do so, it is necessary to edit the `~/.bashrc` file and comment the lines similar as this: `source /opt/ros/kinetic/setup.bash`.
Save the file and execute `source ~/.bashrc` in this terminal.
After that it is necessary to load the python 3.5 version of cv_bridge. In this case, it is necessary to load the cv_bridge installation as in the step 3 of  Section 3.1.
In this case, it is necessary to execute `source /path/to/cv_bridge_workspace/install/setup.bash`.

3. Run the following in a terminal:

```
rosrun turtlebot2i_mrcnn ros_mrcnn.py 
```

**If any problem occurs in this step, check again the step 3 of Section 3.1**

The detection output will be available in the `/turtlebot2i/mrcnn_out` topic.


# 4. Training the Model

If want to train new model, first, get images from rosbag or manually get a set of images.

1. (Optional step) If you want to create a training dataset from rosbag

    - Create the rosbag by recording camera topic
    ```
    rosbag record camera_topic_name -O output_bag.bag
    ```

    - Run `bag2cv.py` to extract images from a image topic. 
    *It may be necessary to change bag name, topic name, etc. in this script.*

2. Annotate images with labelme tool

    - From the set of images, use the [labelme](https://github.com/wkentaro/labelme) tool to draw polygons around the target object. The accuracy of the trained model depends on the quality of the annotation. (It is possible to use another annotation tooll, but different procedures may be necessary to obtain the final training dataset.)
   
    - After labeling the images through labelme, we get JSON files as the result for each single image. Then we use the script offered by labelme to convert the JSON file to get the mask image and the corresponding labels. Use the command `labelme_json_to_dataset <file_name>.json` to get a folder with 5 files: img.png, info.yaml, label.png, label_names.txt and label_viz.png. What we need in our training dataset is the mask image (label.png) and label names (info.yaml).

    - Convert the mask image into 8-bit depth image. 
    Use the following command to convert the mask images into 8 bits in order to make it possible to feed in the Mask R-CNN method.
    (Would necessary to manually set the input and output paths in the mask2cv.py code.)
    ```
    python mrcnn/tools/mask2cv.py
    ```

3. Change parameters in the `class VrepConfig` in `train_model.py`. The complete list of parametes can be found the in the Config class: https://github.com/matterport/Mask_RCNN/blob/master/mrcnn/config.py

5. Change the dataset path in `train_model.py` to your own dataset.

6. Run `python train_model.py` to start training.

# 5. Testing the Model


1. Change parameters in the `class VrepConfig` in `test_model.py`. The complete list of parametes can be found the in the Config class: https://github.com/matterport/Mask_RCNN/blob/master/mrcnn/config.py

2. Set the model path on `test_model.py`.

3. Run the test script:
    ```
    python mrcnn/test_model.py
    ```
    * During the inferences, it would randomly pick up one image and show the detection result. 

# 6. Installation (content extracted from Mask R-CNN repository)

1. Install dependencies
   ```bash
   pip3 install -r requirements.txt
   ```
2. Clone this repository
3. Run setup from the repository root directory
    ```bash
    python3 setup.py install
    ``` 
4. Download pre-trained COCO weights (mask_rcnn_coco.h5) from the [releases page](https://github.com/matterport/Mask_RCNN/releases).
5. (Optional) To train or test on MS COCO install `pycocotools` from one of these repos. They are forks of the original pycocotools with fixes for Python3 and Windows (the official repo doesn't seem to be active anymore).

    * Linux: https://github.com/waleedka/coco
    * Windows: https://github.com/philferriere/cocoapi.
    You must have the Visual C++ 2015 build tools on your path (see the repo for additional details)

