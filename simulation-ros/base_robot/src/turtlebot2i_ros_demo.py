#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent, DockInfraRed
from geometry_msgs.msg import Twist

pub = None

def callback(msg):
    
    global pub
    
    vel = Twist()
    data = ord(msg.data)
    
    if (data != 255):
        if (data == 1):
            vel.linear.x  =  0.0
            vel.angular.z = -0.7
        elif (data == 2):
            vel.linear.x  =  0.2
            vel.angular.z = -0.7
        elif (data == 4):
            vel.linear.x  =  0.0
            vel.angular.z =  0.7
        elif (data == 8):
            vel.linear.x  =  0.0
            vel.angular.z = -0.6
        elif (data == 16):
            vel.linear.x  =  0.3
            vel.angular.z = -0.6
        elif (data == 32):
            vel.linear.x  =  0.0
            vel.angular.z =  0.6
    else:
        vel.linear.x  = 1.0
        vel.angular.z = 0.0

    pub.publish(vel)

def listener():

    global pub

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('turtlebot2_demo', anonymous=True)
    
    #rospy.Subscriber("/vrep_ros_interface/turtlebot/events/bumper", BumperEvent, callback)
    rospy.Subscriber("/vrep_ros_interface/turtlebot/sensors/dock_ir", DockInfraRed, callback)
    pub = rospy.Publisher('/vrep_ros_interface/turtlebot/commands/velocity', Twist, queue_size=10)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
