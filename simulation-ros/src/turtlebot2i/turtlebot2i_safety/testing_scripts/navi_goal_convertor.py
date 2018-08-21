#!/usr/bin/env python
import rospy
import math
PI=math.pi
from geometry_msgs.msg import Quaternion#Pose,Point,Quaternion

from tf.transformations import euler_from_quaternion, quaternion_from_euler

roll = pitch = yaw = 0.0

def get_rotation ():
    global roll, pitch, yaw
    roll =0
    pitch=0
    #YAW: around Y axis
    yaw =0#PI
    ''' From quaternion to Euler
    orientation_q = Quaternion()
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print yaw
    '''
    
    ''' From Euler to quaternion
    quat = quaternion_from_euler (roll, pitch,yaw)
    print quat
    '''
    while not rospy.is_shutdown():              
        #print quat
        quat = quaternion_from_euler (roll, pitch,yaw)
        yaw=yaw+PI/2
        rospy.loginfo("\nWith Yaw=%2.2f,\n Quat =[%2.2f,%2.2f]",yaw/PI,quat[2],quat[3])
        #print x=quat[0]=0, y=quat[1]=0, z=quat[2] w=quat[3]
        print quat
        Rate.sleep()    
if __name__ == '__main__':
    try:
        rospy.init_node('my_quaternion_to_euler')
        Rate = rospy.Rate(1)
        get_rotation()

    except rospy.ROSInterruptException:
        pass




