#!/usr/bin/env python

"""
    Edited from navigation.py in turtlebot2i_navigation module
"""

import rospy
from geometry_msgs.msg import Twist
from turtlebot2i_safety.msg import VelocityScale
from kobuki_msgs.msg import Led, Sound, BumperEvent
from sensor_msgs.msg import Joy

import numpy as np

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
    #print("scaling speed received:",l_scale,r_scale,"| linear speed: ",new_speed.linear.x,"angular speed: ",new_speed.angular.z)

def joystick_callback(joystick_data):
    global r_scale, l_scale, interWheelDistance, new_speed, loop_checker
    #data.linear.x
    #data.angular.z
    speed = Twist()
    #''' #commented; this part is for PS4 controller
    if joystick_data.axes[6] == 0.0 and joystick_data.axes[7] == 0.0: 
        speed.linear.x  = 0.5 * joystick_data.axes[1] #max 1.5
        speed.angular.z = 1.0 * joystick_data.axes[0] #max 6.6
    else:
        speed.linear.x  = 0.1 * joystick_data.axes[7] #max 1.5
        speed.angular.z = 0.4 * joystick_data.axes[6] #max 6.6
    #'''
    #this part is for PS3 controller
    #speed.linear.x  = 0.3 * joystick_data.axes[1] #max 1.5
    #speed.angular.z = 2.0 * joystick_data.axes[0] #max 6.6

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
    loop_checker = abs(joystick_data.axes[0]) == 1.0 or abs(joystick_data.axes[1]) == 1.0 or abs(joystick_data.axes[6]) == 1.0  or abs(joystick_data.axes[7]) == 1.0 
    velocity_publisher.publish(new_speed)
    #return new_speed, loop_checker
    print("scaling speed received:",l_scale,r_scale,"| linear speed: ",new_speed.linear.x,"angular speed: ",new_speed.angular.z)
    if np.sum(joystick_data.buttons) > 0:
    	if joystick_data.buttons[0]:
    		sound_pub.publish(0)
    	elif joystick_data.buttons[1]:
    		sound_pub.publish(1)
    	elif joystick_data.buttons[2]:
    		sound_pub.publish(2)
    	elif joystick_data.buttons[3]:
    		sound_pub.publish(3)
    	elif joystick_data.buttons[4]:
    		sound_pub.publish(4)
    	elif joystick_data.buttons[5]:
    		sound_pub.publish(5)
    	elif joystick_data.buttons[6]:
    		sound_pub.publish(6)
    	elif joystick_data.buttons[7]:
    		sound_pub.publish(7)
    
def speed_scale_callback(data):
    global r_scale, l_scale
    l_scale = data.left_vel_scale
    r_scale = data.right_vel_scale
    #print("scaling speed received:",l_scale,r_scale)

def led_callback(data):
    global nav_sub, joystick_mode
    joystick_mode = False
    if data.value == Led.BLACK:
    	nav_sub.unregister()
    	print("Unsubscribe to any navigation module")
    elif data.value == Led.GREEN:
    	nav_sub.unregister()
        joystick_mode = True
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
        led1_pub.publish(Led.BLACK)
        vel_scale_sub = rospy.Subscriber('/turtlebot2i/safety/vel_scale', VelocityScale, speed_scale_callback)

if __name__ == '__main__':
    try:
        rospy.init_node('speed_scaling_py')
        r_scale = 1.0
        l_scale = 1.0
        interWheelDistance = 0.137
        collision_flag = False
        joystick_mode = False
        new_speed = Twist()
        loop_checker = False
        velocity_publisher = rospy.Publisher('/turtlebot2i/commands/velocity', Twist, queue_size=10)
        sound_pub          = rospy.Publisher('/turtlebot2i/commands/sound', Sound,  queue_size=10)
        led2_pub  = rospy.Publisher('/turtlebot2i/commands/led2', Led,  queue_size=10)
        led1_pub  = rospy.Publisher('/turtlebot2i/commands/led1', Led,  queue_size=10)
        led1_pub.publish(Led.BLACK)
        nav_sub = rospy.Subscriber('/turtlebot2i/navigation/velocity', Twist, speed_callback)
        #print("Subscribed to move_base")
        vel_scale_sub = rospy.Subscriber('/turtlebot2i/safety/vel_scale', VelocityScale, speed_scale_callback)
        rospy.Subscriber('/turtlebot2i/commands/led1', Led, led_callback)
        rospy.Subscriber("/turtlebot2i/events/bumper", BumperEvent, bumper_callback)
        while not rospy.is_shutdown():
            if joystick_mode and loop_checker:
                velocity_publisher.publish(new_speed)
                #print("[joystick loop] linear speed: ",new_speed.linear.x,"angular speed: ",new_speed.angular.z)
        #rospy.spin()
    except rospy.ROSInterruptException:
    #except KeyboardInterrupt:
        
        rospy.loginfo("Running test done!")
        pass
