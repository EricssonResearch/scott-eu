#!/usr/bin/env python3

# Force loading python 3 version of cv2
import importlib.util
spec = importlib.util.spec_from_file_location("cv2", "/usr/local/lib/python3.5/dist-packages/cv2/python-3.5/cv2.cpython-35m-x86_64-linux-gnu.so")
cv2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cv2)

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image



def depth_callback(image, depth_image):
    try:
        print ('inside callback')
        print (type(image),type(depth_image))

    except CvBridgeError as e:
        print(e)

class ros_mask_rcnn:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub1 = rospy.Subscriber('/turtlebot2i/camera/rgb/raw_image',   Image, self.save_image_robot1)
        self.image_sub1 = rospy.Subscriber('/turtlebot2i_0/camera/rgb/raw_image', Image, self.save_image_robot2)
        self.imageCount1 = 0
        self.imageCount2 = 0


    def save_image_robot1(self,data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        fileNo = str(self.imageCount1).zfill(10)
        imageTitle = "/home/etrrhmd/dataset_images/rob1/image_"+fileNo+".png"
        cv2.imwrite( imageTitle, cv_image )
        print(imageTitle)
        self.imageCount1 += 1

    def save_image_robot2(self,data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        fileNo = str(self.imageCount2).zfill(10)
        imageTitle = "/home/etrrhmd/dataset_images/rob2/image_"+fileNo+".png"
        cv2.imwrite( imageTitle, cv_image )
        print(imageTitle)
        self.imageCount2 += 1

if __name__ == '__main__':
    rospy.init_node('mask_rcnn_py')
    detector = ros_mask_rcnn()
    rospy.spin()
















