#!/usr/bin/env python

"""
    Edited from navigation.py in turtlebot2i_navigation module
"""

import rospy
from geometry_msgs.msg import Twist
from turtlebot2i_safety.msg import VelocityScale
from kobuki_msgs.msg import Led, Sound, BumperEvent
from sensor_msgs.msg import Joy

def speed_callback(data):
    global r_scale, l_scale, interWheelDistance
    #data.linear.x
    #data.angular.z
    new_speed = data
    #robot_speed     = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2) 
    if data.linear.x < 0.0:
    	new_speed.linear.x  = max(new_speed.linear.x, -0.1)
    elif r_scale < 0.01 and l_scale < 0.01:
    	 new_speed.linear.x = 0.0
    else:
    	vel_r = (data.linear.x + data.angular.z * interWheelDistance) * r_scale
    	vel_l = (data.linear.x - data.angular.z * interWheelDistance) * l_scale
    	new_speed.linear.x  = (vel_r + vel_l) / 2.0 
    	new_speed.angular.z = (vel_r - vel_l) / interWheelDistance
    velocity_publisher.publish(new_speed)
    print("scaling speed received:",l_scale,r_scale,"| linear speed: ",new_speed.linear.x,"angular speed: ",new_speed.angular.z)

def joystick_callback(joystick_data):
    global r_scale, l_scale, interWheelDistance
    #data.linear.x
    #data.angular.z
    speed = Twist()
    speed.linear.x  = 0.5 * joystick_data.axes[1] #max 1.5
    speed.angular.z = 2.0 * joystick_data.axes[0] #max 6.6
    new_speed = speed
    #robot_speed     = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2) 
    if speed.linear.x < 0.0:
        new_speed.linear.x  = max(new_speed.linear.x, -0.1)
    elif r_scale < 0.01 and l_scale < 0.01:
         new_speed.linear.x = 0.0
    else:
        vel_r = (speed.linear.x + speed.angular.z * interWheelDistance) * r_scale
        vel_l = (speed.linear.x - speed.angular.z * interWheelDistance) * l_scale
        new_speed.linear.x  = (vel_r + vel_l) / 2.0 
        new_speed.angular.z = (vel_r - vel_l) / interWheelDistance
    velocity_publisher.publish(new_speed)
    print("scaling speed received:",l_scale,r_scale,"| linear speed: ",new_speed.linear.x,"angular speed: ",new_speed.angular.z)

def speed_scale_callback(data):
    global r_scale, l_scale
    l_scale = data.left_vel_scale
    r_scale = data.right_vel_scale
    #print("scaling speed received:",l_scale,r_scale)

def led_callback(data):
    global nav_sub
    if data.value == Led.BLACK:
    	nav_sub.unregister()
    	print("Unsubscribe to any navigation module")
    elif data.value == Led.GREEN:
    	nav_sub.unregister()
        #nav_sub = rospy.Subscriber('/turtlebot2i/keyop/velocity', Twist, speed_callback)
        nav_sub = rospy.Subscriber('/joy', Joy, joystick_callback)
    	print("Subscribed to keyop")
    elif data.value == Led.ORANGE or data.value == Led.RED:
    	nav_sub.unregister()
        nav_sub = rospy.Subscriber('/turtlebot2i/navigation/velocity', Twist, speed_callback)
        print("Subscribed to move_base")
    	
def bumper_callback(data):
    global collision_flag, r_scale, l_scale, vel_scale_sub
    if data.state == 1 and not collision_flag:
        #print("collision happen!")
        vel_scale_sub.unregister()
        r_scale = 0.0
        l_scale = 0.0
        collision_flag = True
        sound_pub.publish(Sound.OFF)
        led2_pub.publish(Led.RED)
    elif data.state == 0 and collision_flag:
        collision_flag = False
        led2_pub.publish(Led.BLACK)
        vel_scale_sub = rospy.Subscriber('/turtlebot2i/safety/vel_scale', VelocityScale, speed_scale_callback)

if __name__ == '__main__':
    try:
        rospy.init_node('speed_scaling_py')
        r_scale = 1.0
        l_scale = 1.0
        interWheelDistance = 0.137
        collision_flag = False
        velocity_publisher = rospy.Publisher('/turtlebot2i/commands/velocity', Twist, queue_size=10)
        sound_pub          = rospy.Publisher('/turtlebot2i/commands/sound', Sound,  queue_size=10)
        led2_pub  = rospy.Publisher('/turtlebot2i/commands/led2', Led,  queue_size=10)
        nav_sub = rospy.Subscriber('/turtlebot2i/navigation/velocity', Twist, speed_callback)
        #print("Subscribed to move_base")
        vel_scale_sub = rospy.Subscriber('/turtlebot2i/safety/vel_scale', VelocityScale, speed_scale_callback)
        rospy.Subscriber('/turtlebot2i/commands/led1', Led, led_callback)
        rospy.Subscriber("/turtlebot2i/events/bumper", BumperEvent, bumper_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
    #except KeyboardInterrupt:
        
        rospy.loginfo("Running test done!")
        pass
