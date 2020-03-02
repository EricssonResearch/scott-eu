#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CompressedImage

def image_callback(image_data):
    global n_counter, counter_limit
    if n_counter >= counter_limit:
        image_pub.publish(image_data)
        n_counter = 1
    else:
        n_counter += 1
    

if __name__ == '__main__':
    try:
        rospy.init_node('image_filter_py')
        n_counter = 1
        fps = 5
        counter_limit = 30//fps #30 is the original camera's fps
        image_pub = rospy.Publisher("/image_raw/compressed", CompressedImage, queue_size=1)
        image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, image_callback, queue_size=1)
        print("image_filter.py is running")
        rospy.spin()
    except rospy.ROSInterruptException:
    #except KeyboardInterrupt:
        
        rospy.loginfo("Running test done!")
        pass