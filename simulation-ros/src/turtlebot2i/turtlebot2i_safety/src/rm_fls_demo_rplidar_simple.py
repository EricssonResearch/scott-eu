#!/usr/bin/env python
# This module is intended to convert lidar information into x-y data

import rospy
import rospkg
from turtlebot2i_safety.msg import VelocityScale, SafetyRisk, SafetyZone
import std_msgs.msg

# Fuzzy logic packages
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

import time
import os
import cPickle as pickle

from sensor_msgs.msg import LaserScan

from kobuki_msgs.msg import  Led, Sound

def init_var():
    '''
    Here we initialize the global variables.
    '''

    global n_sensors, lidar_min_deg, lidar_max_deg, idx_per_deg, deg_per_idx
    n_sensors = 2094#6280 # scanse lidar #675 #vrep #684 #if lidar on top: 684 data, if lidar in front of robot: 675 data
    lidar_min_deg = -60.0 #-180.0
    lidar_max_deg =  60.0 # 180.0
    idx_per_deg = (lidar_max_deg - lidar_min_deg) / n_sensors
    deg_per_idx = n_sensors / (lidar_max_deg - lidar_min_deg)

    # Safety zone size
    global IZW, range_degree,range_degree,range_meter, range_meter_per_second,range_risk, idx_degree
    IZW = 0.4
    step_meter = 0.02
    step_meter_per_second = 0.02
    step_risk = 0.05
    range_degree = np.arange(-180, 180+1, 1.0)
    idx_degree = np.arange(0, n_sensors+1, 1.0)
    range_meter  = np.arange(0, 3.0+step_meter, step_meter)
    range_meter_per_second = np.arange(-0.5, 2.0+step_meter_per_second, step_meter_per_second)
    range_risk = np.arange(0, 5+step_risk, step_risk)

    global vel_scale_message
    vel_scale_message = VelocityScale()

    global time_duration_list
    time_duration_list = []

    global package_path
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('turtlebot2i_safety')
    
    global r_warning, r_critical, obstacle_zone, prev_obstacle_zone, clear_zone, warning_zone, critical_zone, warning_duration, critical_duration
    r_warning  = 0.0
    r_critical = 0.0
    obstacle_zone, prev_obstacle_zone = 0, 0
    clear_zone, warning_zone, critical_zone = 0,1,2

    

def init_fls_common_part(): #copy this function from risk_management.py; TO DO: put this parameter as class or in another module
    global object_distance,object_direction
     # New Antecedent objects
    object_distance   = ctrl.Antecedent(range_meter, 'distance')
    object_direction  = ctrl.Antecedent(range_degree , 'direction')
    #object_direction  = ctrl.Antecedent(idx_degree , 'direction')

    # Membership functions

    # Distance
    object_distance['Near']  = fuzz.trapmf(range_meter, [0, 0, IZW, 2*IZW])
    object_distance['Medium']= fuzz.trimf(range_meter, [IZW, 2*IZW, 4*IZW])
    object_distance['Far']   = fuzz.trapmf(range_meter, [2*IZW, 4*IZW, 3, 3])
    # Direction -180~180
    n_front =    0.0 # int(n_sensors/2) + 1
    n_far_r = -135.0 #int(n_front - 135 * deg_per_idx)
    n_right =  -90.0 #int(n_front - 90 * deg_per_idx)
    n_fr    =  -45.0 #int(n_front - 45 * deg_per_idx)
    n_fl    =   45.0 #int(n_front + 45 * deg_per_idx)
    n_left  =   90.0 #int(n_front + 90 * deg_per_idx)
    n_far_l =  135.0 #int(n_front + 135 * deg_per_idx)
    #rear_d_p2                       = fuzz.trapmf(range_degree, [0, 0, n_far_r, n_right])
    object_direction['Right']       = fuzz.trimf(range_degree, [n_far_r, n_right, n_fr])
    object_direction['FrontRight']  = fuzz.trimf(range_degree, [n_right, n_fr, n_front])
    object_direction['Front']       = fuzz.trimf(range_degree, [n_fr, n_front, n_fl])
    object_direction['FrontLeft']   = fuzz.trimf(range_degree, [n_front, n_fl, n_left])
    object_direction['Left']        = fuzz.trimf(range_degree, [n_fl, n_left, n_far_l])
    #rear_d_p1                       = fuzz.trapmf(range_degree, [n_left, n_far_l, n_sensors,n_sensors])
    #null,object_direction['BigRear']=fuzz.fuzzy_or(range_degree,rear_d_p1,idx_degree,rear_d_p2)
    #rospy.logdebug("init_fls_common_part")


def init_risk_assessment():
    range_type = np.arange(0, 2+1, 1)
    # New Antecedent/Consequent objects
    object_risk        = ctrl.Consequent(range_risk, 'risk')

    # Risk
    object_risk['VeryLow'] = fuzz.trimf(range_risk, [0, 0, 1])
    object_risk['Low'] =  fuzz.trimf(range_risk, [0, 1, 2])
    object_risk['Medium'] =  fuzz.trimf(range_risk, [1, 2, 3])
    object_risk['High'] =  fuzz.trimf(range_risk, [2, 3, 4])
    object_risk['VeryHigh'] =  fuzz.trimf(range_risk, [3, 4, 4])

    fls_name = "/rules/ra_demo.data"
    fls_data_path = package_path + fls_name
    #print(fls_data_path)

    #if os.path.exists(fls_name):
    if os.path.exists(fls_data_path):
        rospy.loginfo("FLS demo exists!")
        #f = open(fls_name,'rb')
        f = open(fls_data_path,'rb')
        ra_fls = pickle.load(f)
    else:
        rospy.loginfo("Init FLS demo")
        from assessment_rules_demo import rule_list_generator
        assessment_rule_list=rule_list_generator(object_distance,object_direction, object_risk)
        ra_fls = ctrl.ControlSystem(assessment_rule_list)
        #f = open(fls_name,'wb')
        f = open(fls_data_path,'wb')
        pickle.dump(ra_fls,f)
        f.close
    global risk_assessment_instance
    risk_assessment_instance = ctrl.ControlSystemSimulation(ra_fls)

def init_risk_mitigation():
    # New Antecedent/Consequent objects
    object_risk_input = ctrl.Antecedent(range_risk, 'risk_input')# Different name
    #global left_speed,right_speed # When Consequent need visualization
    left_speed  = ctrl.Consequent(range_meter_per_second, 'left')
    right_speed = ctrl.Consequent(range_meter_per_second, 'right')

    # Risk
    object_risk_input['VeryLow'] = fuzz.gaussmf(range_risk,0,0.3)
    object_risk_input['Low']     = fuzz.gaussmf(range_risk,1,0.3)
    object_risk_input['Medium']  = fuzz.gaussmf(range_risk,2,0.3)
    object_risk_input['High']    = fuzz.gaussmf(range_risk,3,0.3)
    object_risk_input['VeryHigh']= fuzz.gaussmf(range_risk,4,0.3)
    # Left Speed
    left_speed['Stop']  = fuzz.gaussmf(range_meter_per_second,0.0,0.1)
    left_speed['Slow']  = fuzz.gaussmf(range_meter_per_second,0.2,0.2)
    left_speed['Medium']= fuzz.gaussmf(range_meter_per_second,0.8,0.2)
    left_speed['Fast']  = fuzz.gaussmf(range_meter_per_second,1.2,0.2)
    # Right Speed
    right_speed['Stop']  = fuzz.gaussmf(range_meter_per_second,0.0,0.1)
    right_speed['Slow']  = fuzz.gaussmf(range_meter_per_second,0.2,0.2)
    right_speed['Medium']= fuzz.gaussmf(range_meter_per_second,0.8,0.2)
    right_speed['Fast']  = fuzz.gaussmf(range_meter_per_second,1.2,0.2)

    from mitigation_rules import rule_list_generator
    mitigation_rule_list=rule_list_generator(object_distance,object_direction, object_risk_input,left_speed,right_speed)

    global risk_mitigation_instance  # We don't need to change the FLS
    risk_mitigation_fls = ctrl.ControlSystem(mitigation_rule_list)
    risk_mitigation_instance = ctrl.ControlSystemSimulation(risk_mitigation_fls)

def cal_risk(object_distance,object_direction):
    risk_assessment_instance.input['distance'] = object_distance
    risk_assessment_instance.input['direction'] = object_direction
    risk_assessment_instance.compute()
    #risk_result = risk_assessment_instance.output['risk']
    return risk_assessment_instance.output['risk']

def cal_safe_vel(object_distance,object_direction,object_risk):
    risk_mitigation_instance.input['distance']   = object_distance
    risk_mitigation_instance.input['direction']  = object_direction
    risk_mitigation_instance.input['risk_input'] = object_risk
    risk_mitigation_instance.compute()
    #print("RM input value; distance:",object_distance,"| direction:",object_direction,"| risk value:",object_risk)
    return risk_mitigation_instance.output['left'],risk_mitigation_instance.output['right']

def pub_safe_vel(left_vel_scale,right_vel_scale):
    vel_scale_message.header = std_msgs.msg.Header()
    vel_scale_message.header.stamp = rospy.Time.now()
    vel_scale_message.left_vel_scale  = left_vel_scale
    vel_scale_message.right_vel_scale = right_vel_scale
    safe_vel_pub.publish(vel_scale_message)

def risk_callback(data):
    global time_duration_list
    #time_previous = time.time()
    if len(data.risk_value) == 0:
        pub_safe_vel(1.0, 1.0)
    elif max(data.risk_value) == 0:
        pub_safe_vel(1.0, 1.0)
    else:
        i = np.argmax(data.risk_value)
        left_vel_scale,right_vel_scale = cal_safe_vel(data.distance[i], data.direction[i], data.risk_value[i])
        pub_safe_vel(left_vel_scale, right_vel_scale)

def lidar_callback(data):
    min_reading = 0.3
    #fov = 120
    fov = 180

    obstacle_zone = 2.0
    r_clear = 0.6
    r_warning = 0.45
    r_critical = 0.35

    # Select the index range that covers 120 degrees FOV in front of the robot
    #fov_min_idx = int((3.14 + 3.14/6.0) / data.angle_increment)        # --> 120 deg FOV
    #fov_max_idx = int((3.14 + 5.0 * 3.14/6.0) / data.angle_increment)  # --> 120 deg FOV
    fov_min_idx = int(3.14 / data.angle_increment)      # --> 180 deg FOV
    fov_max_idx = int((2*3.14) / data.angle_increment)  # --> 180 deg FOV

    # Discard measurements < min_readings (converted to inf)
    ranges = list(np.where(np.array(data.ranges)<min_reading, np.inf, data.ranges))

    #sensor_reads = data.ranges[2092:4185] #only consider the object within camera's FoV
    sensor_reads = ranges[fov_min_idx:fov_max_idx] #only consider the object within camera's FoV
    
    closest_dist = min(sensor_reads)
    closest_idx  = np.argmin(sensor_reads)
    #closest_dir  = closest_idx * idx_per_deg + lidar_min_deg
    closest_dir  = closest_idx + (-fov/2.0)

    #print('fov_min_idx:',fov_min_idx,'fov_max_idx:',fov_max_idx, 'closest_idx:', closest_idx, 'closest_dir:', closest_dir, 'closest_dist:', closest_dist)

    # #print(sensor_reads[0], sensor_reads[-1])
    # #print("Lidar callback", len(sensor_reads), min(sensor_reads), max(sensor_reads))
    # sum_mean_obs_distance += (sum(sensor_reads)/n_sensors) 
    # lidar_cb_count        += 1
    # for i in range(n_sensors):
    #     min_obs_distance[i] = min(min_obs_distance[i], sensor_reads[i])
    min_dist_to_obstacle = closest_dist
    risk_val = cal_risk(closest_dist, closest_dir) #risk analysis
    speed_l, speed_r = cal_safe_vel(closest_dist, closest_dir, risk_val) #risk mitigation
    pub_safe_vel(speed_l, speed_r)
    prev_obstacle_zone = obstacle_zone
    if risk_val > 3.0:
    	led2_pub.publish(Led.RED)
    elif risk_val > 2.0:
    	led2_pub.publish(Led.ORANGE)
    elif risk_val > 1.0:
    	led2_pub.publish(Led.GREEN)
    else:
    	led2_pub.publish(Led.BLACK)
    '''
    if min_dist_to_obstacle > r_warning:
    	obstacle_zone = clear_zone
        print("clear_zone")
        led1_pub.publish(Led.GREEN)
        #light green
    elif min_dist_to_obstacle > r_critical:
        obstacle_zone = warning_zone
        print("warning_zone")
        led1_pub.publish(Led.ORANGE)
        #light yellow
    else:
        obstacle_zone = critical_zone
        print("critical_zone")
        led1_pub.publish(Led.RED)
    '''

    # Publish obstacles' safey info
    safety_data = SafetyRisk()
    safety_data.distance.append(closest_dist)
    safety_data.direction.append(closest_dir)
    safety_data.risk_value.append(risk_val)

    safety_pub.publish(safety_data)

   
    # Publish safey zone sizes
    zone_size = SafetyZone()
    zone_size.header = std_msgs.msg.Header()
    zone_size.header.stamp = rospy.Time.now()
    zone_size.header.frame_id = "base_footprint"
    zone_size.critical_zone_radius = r_critical 
    zone_size.warning_zone_radius  = r_warning
    zone_size.clear_zone_radius    = r_clear
    safe_zone_pub.publish(zone_size)


    #rospy.loginfo("closest_dist: %s  closest_dir: %s", closest_dist, closest_dir)
    #print("closest_dist: ",closest_dist, "| closest_dir: ",closest_dir, "| closest_dir: ", closest_dir, "| risk_val: ",risk_val, "| speed_l: ",speed_l,"| _r",speed_r)
    
    

if __name__ == '__main__':
    try:
        rospy.init_node('rm_fls_demo_py')
        init_var()
        rospy.logdebug("init_var ok")
        init_fls_common_part()
        rospy.logdebug("init_fls_common_part ok")
        init_risk_assessment()
        rospy.logdebug("init_risk_assessment ok")
        init_risk_mitigation()
        rospy.logdebug("init_risk_mitigation ok")

        safe_vel_pub = rospy.Publisher('/mobile_base/safety/vel_scale', VelocityScale, queue_size=10)
        led2_pub  = rospy.Publisher('/mobile_base/commands/led2', Led,  queue_size=10)
        safety_pub  = rospy.Publisher('/mobile_base/safety/obstacles', SafetyRisk,  queue_size=10)
        safe_zone_pub = rospy.Publisher('/mobile_base/safety/safety_zone', SafetyZone, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, lidar_callback)

        rospy.loginfo("risk_mitigation_fls.py is now running!")
        rospy.spin()

    except rospy.ROSInterruptException:
    	led2_pub.publish(Led.BLACK)
        pass
