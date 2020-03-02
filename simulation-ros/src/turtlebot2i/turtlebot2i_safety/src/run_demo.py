#!/usr/bin/env python

"""
    Edited from navigation.py in turtlebot2i_navigation module
"""

import rospy
import actionlib
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
from std_msgs.msg import Float64
from turtlebot2i_safety.msg import SafetyZone
import numpy as np
from kobuki_msgs.msg import BumperEvent, ButtonEvent, Led, Sound

def init_var():
    #Here we initialize the global variables.
    global time_start, prev_pos, init_pos, pose_cb_count, travelled_distance, sum_distance_to_goal, goal
    goal = MoveBaseGoal()
    time_start = rospy.get_time()
    init_pos = geometry_msgs.msg.Point()
    prev_pos = geometry_msgs.msg.Point()
    curr_pos = geometry_msgs.msg.Point()
    pose_cb_count        = 0
    travelled_distance   = 0.0 #the less the better
    sum_distance_to_goal = 0.0 #the less the better

    global n_sensors, lidar_cb_count, sum_mean_obs_distance, min_obs_distance, collision_flag, n_collision, collision_distance
    n_sensors = 675 #684 #if lidar on top: 684 data, if lidar in front of robot: 675 data
    lidar_cb_count = 0
    sum_mean_obs_distance = 0.0 #SM1
    min_obs_distance  = [1000.0] * n_sensors #SM2
    collision_flag = False
    n_collision = 0
    collision_distance = 0.20

    global risk_sum, risk_count, risk_max, risk_x_speed_sum, risk_x_speed_max
    risk_sum   =  0.0
    risk_count =    0
    risk_max   = -1.0
    risk_x_speed_sum =  0.0
    risk_x_speed_max = -1.0

    global r_warning, r_critical, obstacle_zone, prev_obstacle_zone, clear_zone, warning_zone, critical_zone, warning_duration, critical_duration
    r_warning  = 0.0
    r_critical = 0.0
    obstacle_zone, prev_obstacle_zone = 0, 0
    clear_zone, warning_zone, critical_zone = 0,1,2
    warning_duration  = 0.0
    critical_duration = 0.0

    global robot_speed, sum_speed, speed_cb_count
    robot_speed = 0.0 
    sum_speed   = 0.0
    speed_cb_count = 0

    global client
    client = actionlib.SimpleActionClient('turtlebot2i/move_base', MoveBaseAction)
    client.wait_for_server()

    global closest_dist
    closest_dist = 5.0

    global risk_level
    risk_level = 0
    prev_risk_level = 0

    global button
    button = 0
    
    global led1, led2
    led1 = Led.BLACK
    led2 = Led.BLACK
    
    global random_run
    random_run = False

def move_random(x_size, y_size):
    global goal, client
    #client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #client = actionlib.SimpleActionClient('turtlebot2i_0/move_base', MoveBaseAction)

    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    #x_size = 4.0 #meters
    #y_size = 4.0 #meters
    robot_diameter = 0.4
    #robot must be placed in the corner
    x_pos = np.random.uniform(low=-x_size+robot_diameter, high=x_size-robot_diameter)
    y_pos = np.random.uniform(low=-y_size+robot_diameter, high=y_size-robot_diameter)
    print("goal pos sent x:",x_pos," y:",y_pos)
    goal.target_pose.pose.position.x = x_pos
    goal.target_pose.pose.position.y = y_pos
    goal.target_pose.pose.position.z = 0.063 #1.34851861
    #goal.target_pose.pose.orientation.w = 1.0

    #copied from navi_goal_talker
    orientation=geometry_msgs.msg.Quaternion()
    yaw  = -180*math.pi/180 #unit: from deg. to rad.
    orientation=quaternion_from_euler(0,0,yaw)#(roll, pitch,yaw) # return an array
    goal.target_pose.pose.orientation.x=0.0
    goal.target_pose.pose.orientation.y=0.0
    goal.target_pose.pose.orientation.z=orientation[2]
    goal.target_pose.pose.orientation.w=orientation[3]

    client.send_goal(goal)
    print("Goal position is sent! waiting the robot to finish....")
    wait = client.wait_for_result(timeout=rospy.Duration(1200.0)) #timeout in seconds
    if not wait:
        rospy.logerr("Action server not available or timeout!")
        rospy.signal_shutdown("Action server not available!")




def distance2D(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)

def init_subscription():
    time_start = rospy.get_time()
    rospy.Subscriber('/turtlebot2i/sensors/global_pose', geometry_msgs.msg.PoseStamped, update_pose_callback)
    rospy.Subscriber('/turtlebot2i/lidar/scan', LaserScan, lidar_callback) #this is used with Risk mitigation 
    rospy.Subscriber('/turtlebot2i/safety/risk_val', Float64, risk_callback)
    rospy.Subscriber('/turtlebot2i/safety/safety_zone', SafetyZone, safety_zone_callback)
    rospy.Subscriber('/turtlebot2i/odom', Odometry, speed_callback)
    rospy.Subscriber("/turtlebot2i/events/bumper", BumperEvent, bumper_callback)
    rospy.Subscriber("/turtlebot2i/events/button",ButtonEvent, ButtonEventCallback)

def init_publisher():
    #sound_pub = rospy.Publisher('/turtlebot2i/commands/sound', Sound,  queue_size=10)
    led1_pub  = rospy.Publisher('/turtlebot2i/commands/led1', Led,  queue_size=10)
    led2_pub  = rospy.Publisher('/turtlebot2i/commands/led2', Led,  queue_size=10)

def ButtonEventCallback(data):
    global button, client
    global led1, random_run
    
    if ( data.state == ButtonEvent.RELEASED ) :
        state = "released"
    else:
        state = "pressed"  
        random_run = False
        if ( data.button == ButtonEvent.Button0 ) :
            button = 0
            led1 = Led.BLACK
            led1_pub.publish(led1)
            client.cancel_all_goals()
        elif ( data.button == ButtonEvent.Button1 ) :
            button = 1
            if led1 == Led.BLACK or led1 == Led.RED:
                led1 = Led.GREEN
                led1_pub.publish(led1)
            elif led1 == Led.GREEN:
                led1 = Led.ORANGE
                led1_pub.publish(led1)
            elif led1 == Led.ORANGE:
                led1 = Led.RED
                led1_pub.publish(led1)
            #move_random(1.5, 1.5)
        elif ( data.button == ButtonEvent.Button2 ) :
            button = 2
            #client.cancel_all_goals()
            #move_forward()
            if led1 == Led.ORANGE:
                move_forward()
            elif led1 == Led.RED:
                random_run = True
        
    rospy.loginfo("Button %s was %s."%(button, state))

def move_forward():
	print("move forward")
	global robot_position, robot_orientation
	global goal, client
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	#print("rob orient",robot_orientation)
	direction = euler_from_quaternion([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])[2]
	print("direction:",direction)
	new_position = robot_position
	new_position.x = robot_position.x + np.cos(direction)
	new_position.y = robot_position.y + np.sin(direction)
	goal.target_pose.pose.position = new_position
	goal.target_pose.pose.orientation = robot_orientation
	client.send_goal(goal)
	print("rob pos: ", robot_position)
	print("rob goal:", new_position)
	print("Goal position is sent! waiting the robot to finish....")
	wait = client.wait_for_result(timeout=rospy.Duration(1200.0)) #timeout in seconds
	if not wait:
		rospy.logerr("Action server not available or timeout!")
		rospy.signal_shutdown("Action server not available!")

def summarize_running_test():
    global warning_duration, critical_duration
    rospy.loginfo("Goal reached!")
    time_finish = rospy.get_time()
    duration = time_finish - time_start
    if obstacle_zone == warning_zone:
        warning_time_end  = rospy.get_time()
        warning_duration += (warning_time_end-warning_time_start)
    elif obstacle_zone == critical_zone:
        critical_time_end = rospy.get_time()
        critical_duration += (critical_time_end - critical_time_start)
    safe_duration = duration-(warning_duration+critical_duration)
    rospy.loginfo("1. Time execution                        : %1.4f s  (less is better)",duration)
    rospy.loginfo(" a) Safe (green) zone                    : %1.4f s (%.1f%%) (more is better)",safe_duration,     100*safe_duration/duration)
    rospy.loginfo(" b) Warning (yellow) zone                : %1.4f s (%.1f%%) (less is better)",warning_duration,  100*warning_duration/duration)
    rospy.loginfo(" c) Critical (red )zone                  : %1.4f s (%.1f%%) (less is better)",critical_duration, 100*critical_duration/duration)
    rospy.loginfo("2. Travelled distance                    : %1.4f m  (less is better)",travelled_distance)
    rospy.loginfo("3. Average speed                         : %1.4f m/s(more is better)",sum_speed/speed_cb_count)
    # rospy.loginfo("4. Mean distance to goal                 : %1.4f m  (less is better)",sum_distance_to_goal/pose_cb_count)
    # rospy.loginfo("5. SM1 mean distance to obstacle         : %1.4f m  (more is better)",sum_mean_obs_distance/lidar_cb_count)
    # rospy.loginfo("6. SM2 mean minimum distance to obstacle : %1.4f m  (more is better)",sum(min_obs_distance)/n_sensors)
    # rospy.loginfo("7. Minimum distance to obstacle          : %1.4f m  (more is better)",min(min_obs_distance))
    # rospy.loginfo("8. Number of collision during operation  : %d       (less is better)",n_collision)
    # rospy.loginfo("9. Risk mean(average) value              : %1.4f    (less is better)",risk_sum/risk_count)
    # rospy.loginfo("10.Risk maximum value                    : %1.4f    (less is better)",risk_max)
    # rospy.loginfo("11.Risk x speed mean(average) value      : %1.4f    (less is better)",risk_x_speed_sum/risk_count)
    # rospy.loginfo("12.Risk x speed maximum value            : %1.4f    (less is better)",risk_x_speed_max)
    # np.savez_compressed('result_x.npz', duration=duration, safe_duration=safe_duration, warning_duration=warning_duration, critical_duration=critical_duration, travelled_distance=travelled_distance,
    #                     average_speed=sum_speed/speed_cb_count, mean_distance_to_goal=sum_distance_to_goal/pose_cb_count, sm1=sum_mean_obs_distance/lidar_cb_count, sm2=sum(min_obs_distance)/n_sensors,
    #                     min_distance_to_obstacle=min(min_obs_distance), n_collision=n_collision, risk_mean=risk_sum/risk_count, risk_max=risk_max, risk_speed_mean=risk_x_speed_sum/risk_count, risk_speed=risk_x_speed_max)

def update_pose_callback(data):
    global prev_pos, init_pos, travelled_distance, sum_distance_to_goal, pose_cb_count
    pose_cb_count += 1
    if distance2D(data.pose.position, goal.target_pose.pose.position) < 0.2: #check distance to goal
        print("goal reached!")
        client.cancel_all_goals()
    if prev_pos == init_pos:
        prev_pos = data.pose.position
        sum_distance_to_goal  = distance2D(prev_pos, goal.target_pose.pose.position)
    else:
        curr_pos              = data.pose.position
        travelled_distance   += distance2D(curr_pos, prev_pos)
        sum_distance_to_goal += distance2D(curr_pos, goal.target_pose.pose.position)
        prev_pos              = data.pose.position

def lidar_callback(data):
    global closest_dist
    #global n_sensors, lidar_cb_count, sum_mean_obs_distance, min_obs_distance, collision_flag, n_collision, collision_distance
    global r_warning, r_critical, obstacle_zone, prev_obstacle_zone, clear_zone, warning_zone, critical_zone
    #print("len data:",len(data.ranges))
    #count = 0
    normal_data = []
    for i in range(len(data.ranges)):
        if data.ranges[i] < 41.0:
            #print i, data.ranges[i]
            #count += 1
            normal_data.append(data.ranges[i])
    #print("normal data:", len(normal_data))
    #print("min10:", sorted(normal_data)[:10])
    #print("max10:", sorted(normal_data)[-10:])
    
    #print("len_data: ",len(data.ranges), "| normal data: ",len(normal_data), "| nearest index: ", np.argmin(data.ranges))
    #small index is the right of the robot
    #sensor data: 6280

    '''
    #global warning_duration, critical_duration, warning_time_start, critical_time_start
    #sensor_reads = data.ranges
    #ignore 4 first data and 5 last data
    sensor_reads = data.ranges[4:n_sensors+4]
    
    closest_dist = min(sensor_reads)
    # #print(sensor_reads[0], sensor_reads[-1])
    # #print("Lidar callback", len(sensor_reads), min(sensor_reads), max(sensor_reads))
    # sum_mean_obs_distance += (sum(sensor_reads)/n_sensors) 
    # lidar_cb_count        += 1

    # for i in range(n_sensors):
    #     min_obs_distance[i] = min(min_obs_distance[i], sensor_reads[i])

    min_dist_to_obstacle = min(sensor_reads) 

    prev_obstacle_zone = obstacle_zone
    if min_dist_to_obstacle > r_warning:
        obstacle_zone = clear_zone
        print("clear_zone")
        #light green
    elif min_dist_to_obstacle > r_critical:
        obstacle_zone = warning_zone
        print("warning_zone")
        #light yellow
    else:
        obstacle_zone = critical_zone
        print("critical_zone")
        #light red

    # if obstacle_zone!=prev_obstacle_zone:
    #     #print("prev_obstacle_zone: ", prev_obstacle_zone, "| obstacle_zone:",obstacle_zone)
    #     if obstacle_zone == warning_zone:
    #         warning_time_start = rospy.get_time()
    #     elif obstacle_zone == critical_zone:
    #         critical_time_start = rospy.get_time()

    #     if prev_obstacle_zone == warning_zone:
    #         warning_time_end  = rospy.get_time()
    #         warning_duration += (warning_time_end-warning_time_start)
    #     elif prev_obstacle_zone == critical_zone:
    #         critical_time_end = rospy.get_time()
    #         critical_duration += (critical_time_end - critical_time_start)
    '''

'''
    sound:
    'turn on': Sound.ON,
    'turn off': Sound.OFF,
    'recharge start': Sound.RECHARGE,
    'press button': Sound.BUTTON,
    'error sound': Sound.ERROR,
    'start cleaning': Sound.CLEANINGSTART,
    'cleaning end': Sound.CLEANINGEND,
'''

'''
    led_colors = {
        'off': Led.BLACK,
        'black': Led.BLACK,
        'green': Led.GREEN,
        'orange': Led.ORANGE,
        'red': Led.RED,
}

'''

def bumper_callback(data):
    global n_collision, collision_flag, button, led1
    if data.state == 1 and not collision_flag:
    	button = 0
        led1 = Led.BLACK
        led1_pub.publish(led1)
        client.cancel_all_goals()
        print("collision happen!")
        collision_flag = True
        n_collision += 1
    elif data.state == 0 and collision_flag:
        collision_flag = False
        led2_pub.publish(Led.BLACK)


def risk_callback(risk_value):
    global risk_sum, risk_count, risk_max, risk_x_speed_sum, risk_x_speed_max
    global risk_level, prev_risk_level
    risk_sum  += risk_value.data
    risk_count += 1
    risk_max   = max(risk_max, risk_value.data)
    prev_risk_level = risk_level
    risk_level = int(risk_max)
    if prev_risk_level != risk_level:
        if risk_max > 3.0:
            print("red")
        elif risk_max > 2.0:
            print("yellow")
        else:
            print("green")
    # risk_x_speed      = risk_value.data*robot_speed
    # risk_x_speed_sum += risk_x_speed
    # risk_x_speed_max  = max(risk_x_speed_max, risk_x_speed)

def safety_zone_callback(data):
    global r_warning, r_critical
    r_warning  = data.warning_zone_radius
    r_critical = data.critical_zone_radius

def speed_callback(data):
    global robot_speed, sum_speed, speed_cb_count
    global robot_position, robot_orientation
    robot_position = data.pose.pose.position
    robot_orientation = data.pose.pose.orientation
    robot_speed     = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2) 
    sum_speed      += robot_speed
    speed_cb_count += 1

if __name__ == '__main__':
    try:
        rospy.init_node('run_demo_py')
        init_var()
        print("init_var ok!")
        init_subscription()
        print("init_subs ok!")
        #sound_pub = rospy.Publisher('/turtlebot2i/commands/sound', Sound,  queue_size=10)
        led1_pub  = rospy.Publisher('/turtlebot2i/commands/led1', Led,  queue_size=10)
        led2_pub  = rospy.Publisher('/turtlebot2i/commands/led2', Led,  queue_size=10)
        #init_publisher()
        led1 = Led.BLACK
        led1_pub.publish(led1)
        print("Demo is running!")
        #rospy.spin()
        while not rospy.is_shutdown():
            if led1 == Led.RED and random_run:
                print("move randomly")
                move_random(1.5, 1.5)
        summarize_running_test()
    except rospy.ROSInterruptException:
    #except KeyboardInterrupt:
        
        rospy.loginfo("Running test done!")
        pass
