#!/usr/bin/env python

import rospy
import actionlib
import random
import vrep
import time
import sys
import numpy as np
import geometry_msgs.msg
import std_msgs.msg
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan,Imu
from std_msgs.msg import Float64
from turtlebot2i_safety.msg import SafetyZone, VelocityScale, SafetyRisk
from collections import deque
from math import pi, sqrt, sin, cos, radians
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from shapely.geometry import Polygon, box, LineString, Point
from shapely.affinity import translate
#from matplotlib import pyplot as plt

import rl_agent as agent
import rl_lp as lp
import rl_var as var

def init_vrep():
    global clientID, returnCode, robot_handle
    clientID = vrep.simxStart('127.0.0.1', 20001, True, True, 5000, 5)  
    returnCode, robot_handle = vrep.simxGetObjectHandle(clientID, 'turtlebot2i', vrep.simx_opmode_blocking) 

def init_var():
    #Here we initialize the global variables.
    global target_list
    target_list = [ [-9.0, 6.5],
                    [-9.0, 3.0],
                    #[-6.5, 4.75],
                    [-4.0, 6.5],
                    [-4.0, 3.0],
                    [-0.5, 6.0],
                    [ 1.0, 3.0],
                    [ 5.0, 2.5],
                    [ 3.0, 0.0],
                    [-8.5, 0.0],
                    [-0.5,-2.0],
                    [ 9.0,-6.5],
                    [ 5.0,-2.0],
                    [-4.5,-2.5],
                    [ 8.5,-0.5],
                    [-9.0,-6.5]]

    global prev_pos, init_pos, travelled_distance, goal, travelled_distance_list, robot_orientation
    goal = MoveBaseGoal()
    init_pos = geometry_msgs.msg.Point()
    prev_pos = geometry_msgs.msg.Point()
    travelled_distance = 0.0 
    travelled_distance_list = deque([])
    robot_orientation = 0

    global n_sensors
    n_sensors = 675 #684 #if lidar on top: 684 data, if lidar in front of robot: 675 data

    global robot_linear_speed, robot_angular_speed
    robot_linear_speed  = 0.0 
    robot_angular_speed = 0.0
    
    global n_distance, n_direction, origin
    origin = Point((0.0, 0.0))
    camera_near_clipping = 0.2 #0.01 #in meters
    camera_far_clipping  = 3.5 #in meters
    camera_fov_angle     = 57.0 #in degree
    direction_list = np.linspace(-camera_fov_angle, camera_fov_angle, 6)
    distance_list  = var.distance_list
    n_direction = len(direction_list)-1
    n_distance  = len(distance_list)-1

    var.min_dist_to_obstacle = 5.0
    var.r_warning  = 0.0
    var.r_critical = 0.0
    var.collision = False
    var.collected_distance = 0.0
    var.TASK_ID = "rm_rl_simple_800"

def movebase_client():
    global goal, client
    client = actionlib.SimpleActionClient('turtlebot2i/move_base', MoveBaseAction)
    #client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    next_goal = random.choice(target_list)
    goal.target_pose.pose.position.x = next_goal[0] 
    goal.target_pose.pose.position.y = next_goal[1] 
    goal.target_pose.pose.position.z = 0.063 

    orientation=geometry_msgs.msg.Quaternion()
    yaw  = random.uniform(-pi, pi)#-90*pi/180 #unit: from deg. to rad.
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
    return sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)

def init_subscription():
    rospy.Subscriber('/turtlebot2i/sensors/global_pose', geometry_msgs.msg.PoseStamped, update_pose_callback)
    rospy.Subscriber('/turtlebot2i/safety/obstacles_risk', SafetyRisk, risk_callback)
    rospy.Subscriber('/turtlebot2i/lidar/scan_transformed', LaserScan, lidar_callback)
    rospy.Subscriber('/turtlebot2i/safety/safety_zone', SafetyZone, safety_zone_callback)
    rospy.Subscriber('/turtlebot2i/commands/velocity', Twist, speed_callback)
    rospy.Subscriber('/turtlebot2i/events/bumper', BumperEvent, bumper_callback)

def reset_robot_pos(): #Still have problem 
    #reset robot position to origin
    global robot_handle, travelled_distance_list
    var.risk_max = 0.0
    var.collected_distance = 0.0
    var.min_dist_to_obstacle = 5.0
    var.collision = False
    var.r_warning = 0.0
    var.r_critical = 0.0
    model_location = '/home/etrrhmd/Work/scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_description/v-rep_model/warehouse_scene/vrep_models/turtlebot2i.ttm'
    vrep.simxRemoveModel(clientID, robot_handle, vrep.simx_opmode_oneshot)
    returnCode, robot_handle = vrep.simxLoadModel(clientID, model_location, 0, vrep.simx_opmode_oneshot) 
    ##print("after; robot_handle:",robot_handle,clientID,returnCode)
    #robot_origin = np.array([0.0, 0.0, 0.063])
    #returnCode = vrep.simxSetObjectPosition(clientID, robot_handle, -1, robot_origin, vrep.simx_opmode_oneshot_wait)
    travelled_distance_list = deque([])
    prev_pos = init_pos
    
def update_pose_callback(data):
    #This module has 2 main function:
    # 1. In training, it will cancel the goal once robot getting close to the goal. In this way, robot doesn't need to stop and continue navigate to other goals
    # 2. Monitor robot position. If robot get stuck for some time, robot will be placed to origin
    global prev_pos, init_pos, travelled_distance, travelled_distance_list, robot_orientation#, collected_distance
    (roll, pitch, robot_orientation) = euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
    if distance2D(data.pose.position, goal.target_pose.pose.position) < 0.2: #check distance to goal
        print("goal reached!")
        client.cancel_all_goals()
    if prev_pos == init_pos:
        prev_pos = data.pose.position
    else:
        travelled_distance  = distance2D(data.pose.position, prev_pos) #calculating displacement between callback
        var.collected_distance += travelled_distance                       #collecting displacement for RL
        prev_pos            = data.pose.position
        if len(travelled_distance_list) > 299:
            #print(sum(travelled_distance_list))
            if sum(travelled_distance_list) < 0.1: #if robot gets stuck, cancel the goal
                travelled_distance_list = deque([])
                client.cancel_all_goals()
                #print("robot must be reset to origin")
                #reset_robot_pos()
                #time.sleep(3)
            else:
                travelled_distance_list.popleft()
        travelled_distance_list.append(travelled_distance)

def rotated_pos(pointX, pointY, centerX, centerY,r00, r01, r10, r11):
    point_X_rotated = r00*pointX + r01*pointY + centerX - r00*centerX - r01*centerY
    point_Y_rotated = r10*pointX + r11*pointY + centerY - r10*centerX - r11*centerY
    return [point_X_rotated, point_Y_rotated]


def risk_callback(data):
    global current_training_step#, risk_max
    i = np.argmax(data.risk_value)
    var.risk_max = data.risk_value[i]
    #data.distance[i], data.direction[i], data.risk_value[i]
    #for property in ["object_distance", "object_direction", "object_risk_input"]
    
    obstacle_state = []
    in_values  = var.INPUT_VARIABLES["object_distance"]
    aux = np.digitize(data.distance[i], in_values, right=True)
    obstacle_state.append( np.clip(aux - 1, 0, len(in_values) - 1) )

    in_values  = var.INPUT_VARIABLES["object_direction"]
    aux = np.digitize(data.direction[i], in_values, right=True)
    obstacle_state.append( np.clip(aux - 1, 0, len(in_values) - 1) )

    in_values  = var.INPUT_VARIABLES["object_risk_input"]
    aux = np.digitize(data.risk_value[i], in_values, right=True)
    obstacle_state.append( np.clip(aux - 1, 0, len(in_values) - 1) )

    
    #print("obstacle_state: ", obstacle_state)
    agent.obstacle_state = np.array(obstacle_state)
    lp.step = current_training_step
    lp.run_training()
    current_training_step += 1
    #print(obstacle_matrix)
    #plt.show() 
        
def lidar_callback(data):
    #global min_dist_to_obstacle
    #sensor_reads = data.ranges
    #ignore 4 first data and 5 last data
    sensor_reads = data.ranges[4:n_sensors+4]
    var.min_dist_to_obstacle = min(sensor_reads) 

def safety_zone_callback(data):
    #global r_warning, r_critical
    var.r_warning  = data.warning_zone_radius
    var.r_critical = data.critical_zone_radius

def speed_callback(data):
    #getting data from move base module
    global robot_linear_speed, robot_angular_speed
    robot_linear_speed  = data.linear.x
    robot_angular_speed = data.angular.z 

    in_values  = var.INPUT_VARIABLES["steering_direction"]
    aux = np.digitize(robot_angular_speed, in_values, right=True)
    agent.steering_state = np.clip(aux - 1, 0, len(in_values) - 1)

def bumper_callback(data):
    if data.state == 1: #collision occurs
        var.collision = True

def pub_safe_vel(left_vel_scale,right_vel_scale):
    safe_vel_pub = rospy.Publisher('/turtlebot2i/safety/vel_scale', VelocityScale, queue_size=10) #init publisher
    vel_scale_message = VelocityScale()
    #global vel_scale_message
    vel_scale_message.header = std_msgs.msg.Header()
    vel_scale_message.header.stamp = rospy.Time.now()
    vel_scale_message.left_vel_scale  = left_vel_scale
    vel_scale_message.right_vel_scale = right_vel_scale
    safe_vel_pub.publish(vel_scale_message)
    

def execute_action(speed_scale):
    """ Send the scaling factor to the robot.
    input: vector of speed scale: e.g. [0.8,0.4] """
    assert len(speed_scale) == len(var.out_data), " Check output variables"
    left_vel_scale, right_vel_scale = speed_scale[0], speed_scale[1]
    pub_safe_vel(left_vel_scale, right_vel_scale)

def get_reward():  # abstract s,a,sp pointless here
    #displacement = robot.mobilebase_displacement2d
    #REWARDS = np.array([-10.0, -5.0, -1.0, -0.05, 10.0])
    #global collected_distance
    if var.collision:  # big penalty for collision
        r = min(var.REWARDS)
        var.collision = False
    elif var.min_dist_to_obstacle < var.r_critical:  # penalty for critical zone
        r = var.REWARDS[1]
    elif var.min_dist_to_obstacle < var.r_warning:  # penalty for critical zone
        r = var.REWARDS[2]
    #elif var.collected_distance > 2*var.RANGE_DISPLACEMENT: #reward after pass over an obstacle
    #    r = 2*max(var.REWARDS)
    #    var.collected_distance = 0.0 #reset the displacement once the robot has moved
    elif var.collected_distance > var.RANGE_DISPLACEMENT: #reward after moving RANGE_DISPLACEMENT distance
        r = max(var.REWARDS)
        var.collected_distance = 0.0 #reset the displacement once the robot has moved
    else:
        r = var.REWARDS[3]
    #print("get reward:", r, var.risk_max, var.collected_distance, var.min_dist_to_obstacle, var.collision)
    return r-(var.risk_max*0.1)

def overwriteVAR():
    var.INPUT_VARIABLES = {
        #"object_distance":    np.array([0.3, 0.6, 0.9, 1.4, 2.0, 3.0, 3.5]),
        "object_distance":    np.array([0.21, 0.32, 0.6, 0.9, 1.4, 2.0, 3.0, 3.5]),
        "object_direction":   np.array([-57.0, -33.0, -10.0,  10.0, 33.0]),
        "object_risk_input":  np.array([0.0, 1.0, 2.0, 3.0]),
        "steering_direction": np.array([-57.0, -33.0, -10.0,  10.0, 33.0])*np.pi/180.0
    }

def load_model(filename):
	loaded_model = np.load(filename)
	if (np.shape(lp.policy) == np.shape(loaded_model['Policy'])) and (np.shape(lp.v) == np.shape(loaded_model['V'])) and (np.shape(lp.q) == np.shape(loaded_model['Q'])) and (np.shape(lp.q_count) == np.shape(loaded_model['Q_count'])):
		lp.policy  = loaded_model['Policy']
		lp.v       = loaded_model['V']
		lp.q       = loaded_model['Q']
		lp.q_count = loaded_model['Q_count']
		print("Model loaded from: ",filename)
		print("Model has been load successfully!")
	else:
		print("Load file error!!! Check the dimension of the model!")
		print("Program stop!!")
		sys.exit()

if __name__ == '__main__':
    try:
        print("Initializating node.")
        rospy.init_node('training_rl_py')
        print("Initializating other modules.")
        overwriteVAR()
        agent.setup_task() #setup function in task.py
        lp.setup()
        current_training_step = 0
        if not var.training_mode:
        	load_model('/home/etrrhmd/RL_results/2019_05_08_04_30_rm_rl_3k_TOSL_QBIASSR/rm_rl_3k_TOSL_QBIASSR.npz')
        print("Initializating internal variables.")
        init_var()
        print("Initializating VREP.")
        init_vrep()
        print("Initializating Subscription.")
        init_subscription()
        print("initialization finished, training process is starting")
        while (not rospy.is_shutdown()) and var.training_mode:
            result = movebase_client()
        rospy.spin()
    except rospy.ROSInterruptException:
        vrep.simxFinish(clientID)
        rospy.loginfo("Navigation test finished.")
