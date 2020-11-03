#!/usr/bin/env python
# This module is intended to convert lidar information into x-y data

import rospy
import rospkg
from turtlebot2i_safety.msg import VelocityScale, SafetyRisk, BoundingBox
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
from std_msgs.msg import Float64, Float64MultiArray

def init_var():
    '''
    Here we initialize the global variables.
    '''

    # Safety zone size
    global fls_max_far, IZW, range_degree,range_degree,range_meter, range_meter_per_second,range_risk, idx_degree
    fls_max_far = 3.0 #meter
    IZW = 0.4
    step_meter = 0.02
    step_meter_per_second = 0.02
    step_risk = 0.05
    range_degree = np.arange(-180, 180+1, 1.0)
    idx_degree = np.arange(0, n_sensors+1, 1.0)
    range_meter  = np.arange(0, fls_max_far+step_meter, step_meter)
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

    global detected_object
    detected_object = []

    


    

def init_fls_common_part(): #copy this function from risk_management.py; TO DO: put this parameter as class or in another module
    global object_distance,object_direction
    global n_front #index for front direction
     # New Antecedent objects
    object_distance   = ctrl.Antecedent(range_meter, 'distance')
    object_direction  = ctrl.Antecedent(range_degree , 'direction')
    #object_direction  = ctrl.Antecedent(idx_degree , 'direction')

    # Membership functions

    # Distance
    object_distance['Near']  = fuzz.trapmf(range_meter, [0, 0, IZW, 2*IZW])
    object_distance['Medium']= fuzz.trimf(range_meter, [IZW, 2*IZW, 4*IZW])
    object_distance['Far']   = fuzz.trapmf(range_meter, [2*IZW, 4*IZW, fls_max_far, fls_max_far])
    n_front = int(n_sensors/2) + 1
    '''
    # Direction -180~180 ()
    n_front = int(n_sensors/2) + 1
    n_far_r = int(n_front - 135 * deg_per_idx)
    n_right = int(n_front - 90 * deg_per_idx)
    n_fr    = int(n_front - 45 * deg_per_idx)
    n_fl    = int(n_front + 45 * deg_per_idx)
    n_left  = int(n_front + 90 * deg_per_idx)
    n_far_l = int(n_front + 135 * deg_per_idx)
    #rear_d_p2                       = fuzz.trapmf(idx_degree, [0, 0, n_far_r, n_right])
    object_direction['Right']       = fuzz.trimf(idx_degree, [n_far_r, n_right, n_fr])
    object_direction['FrontRight']  = fuzz.trimf(idx_degree, [n_right, n_fr, n_front])
    object_direction['Front']       = fuzz.trimf(idx_degree, [n_fr, n_front, n_fl])
    object_direction['FrontLeft']   = fuzz.trimf(idx_degree, [n_front, n_fl, n_left])
    object_direction['Left']        = fuzz.trimf(idx_degree, [n_fl, n_left, n_far_l])
    #rear_d_p1                       = fuzz.trapmf(idx_degree, [n_left, n_far_l, n_sensors,n_sensors])
    '''

    # Direction -180~180 ()
    dir_front =  0
    dir_far_r = -135
    dir_right = -90
    dir_fr    = -45
    dir_fl    =  45
    dir_left  =  90
    dir_far_l = 135
    #rear_d_p2                       = fuzz.trapmf(idx_degree, [0, 0, dir_far_r, dir_right])
    object_direction['Right']       = fuzz.trimf(range_degree, [dir_far_r, dir_right, dir_fr])
    object_direction['FrontRight']  = fuzz.trimf(range_degree, [dir_right, dir_fr, dir_front])
    object_direction['Front']       = fuzz.trimf(range_degree, [dir_fr, dir_front, dir_fl])
    object_direction['FrontLeft']   = fuzz.trimf(range_degree, [dir_front, dir_fl, dir_left])
    object_direction['Left']        = fuzz.trimf(range_degree, [dir_fl, dir_left, dir_far_l])
    #rear_d_p1                       = fuzz.trapmf(idx_degree, [n_left, n_far_l, n_sensors,n_sensors])

    #null,object_direction['BigRear']=fuzz.fuzzy_or(idx_degree,rear_d_p1,idx_degree,rear_d_p2)
    print("init_fls_common_part")


def init_risk_assessment():
    global fls_name
    range_type = np.arange(0, 2+1, 1)
    # New Antecedent/Consequent objects
    object_risk        = ctrl.Consequent(range_risk, 'risk')

    # Risk
    object_risk['VeryLow'] = fuzz.trimf(range_risk, [0, 0, 1])
    object_risk['Low'] =  fuzz.trimf(range_risk, [0, 1, 2])
    object_risk['Medium'] =  fuzz.trimf(range_risk, [1, 2, 3])
    object_risk['High'] =  fuzz.trimf(range_risk, [2, 3, 4])
    object_risk['VeryHigh'] =  fuzz.trimf(range_risk, [3, 4, 4])

    #fls_name = "/rules/ra_full.data"
    fls_name = "/rules/ra_demo.data"
    fls_data_path = package_path + fls_name
    print(fls_data_path)

    #if os.path.exists(fls_name):
    if os.path.exists(fls_data_path):
        print("FLS demo exists!")
        #f = open(fls_name,'rb')
        f = open(fls_data_path,'rb')
        ra_fls = pickle.load(f)
    else:
        print("Init FLS demo")
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

def cal_risk(object_type,object_distance,object_direction,object_speed,object_orientation):
    if fls_name == "/rules/ra_demo.data":
        risk_assessment_instance.input['distance'] = min(fls_max_far, object_distance)
        risk_assessment_instance.input['direction'] = object_direction
        risk_assessment_instance.compute()
        #risk_result = risk_assessment_instance.output['risk']
        return risk_assessment_instance.output['risk']
    else:
        risk_assessment_instance.input['type'] = object_type
        risk_assessment_instance.input['distance'] = object_distance
        risk_assessment_instance.input['direction'] = object_direction
        risk_assessment_instance.input['speed'] =   object_speed
        risk_assessment_instance.input['orientation'] = object_orientation

        risk_assessment_instance.compute()
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

def lidar_and_camera_setup(data):
    global camera_fov, camera_res_width, camera_mid_width, lidar_initialized
    camera_fov = 60.0#90.0
    camera_res_width = 640
    camera_mid_width = camera_res_width / 2.0

    global n_sensors, lidar_min_deg, lidar_max_deg, idx_per_deg, deg_per_idx
    
    n_sensors = len(data.ranges) #512#6280 # scanse lidar #675 #vrep #684 #if lidar on top: 684 data, if lidar in front of robot: 675 data
    lidar_min_deg = data.angle_min * 180/np.pi#-120.0
    lidar_max_deg = data.angle_max * 180/np.pi# 120.0

    #The old lidar settings:
    #n_sensors = 2094#6280 # scanse lidar #675 #vrep #684 #if lidar on top: 684 data, if lidar in front of robot: 675 data
    #lidar_min_deg = -60.0 #-180.0
    #lidar_max_deg =  60.0 # 180.0

    idx_per_deg = (lidar_max_deg - lidar_min_deg) / n_sensors
    deg_per_idx = n_sensors / (lidar_max_deg - lidar_min_deg)

    #LIDAR reading within camera fov (asuming lidar always have wider fov)
    global lidar_idx_min, lidar_idx_max
    lidar_idx_min = int((-camera_fov - lidar_min_deg) * deg_per_idx)
    lidar_idx_max = int((camera_fov - lidar_min_deg) * deg_per_idx)
    print('lidar_min_deg:',lidar_min_deg,'   lidar_max_deg:', lidar_max_deg)
    print('lidar_idx_min:',lidar_idx_min,'   lidar_idx_max:', lidar_idx_max)
    lidar_initialized = True

    global lidar_max_distance
    lidar_max_distance = data.range_max
    global init_lidar_sub
    init_lidar_sub.unregister()
    print('Lidar has been setup')


def detect_multi_object(lidar_array):
    dist_threshold = 2.0
    obstacle_threshold = 0.30
    first_index_list = []
    last_index_list = []
    for i in range(len(lidar_array)-1):
        if (lidar_array[i] < dist_threshold) and (abs(lidar_array[i+1] - lidar_array[i])>obstacle_threshold) and (len(first_index_list)==len(last_index_list)):
            first_index_list.append(i)
        elif ((abs(lidar_array[i+1] - lidar_array[i])>obstacle_threshold) and (len(first_index_list)!=len(last_index_list))):
            last_index_list.append(i)
    if (len(first_index_list)!=len(last_index_list)):
        last_index_list.append(len(lidar_array)-1)
    
    index_threshold = 30#15
    current_index = 0
    total_length = len(first_index_list)-1
    while current_index < total_length:
        if (first_index_list[current_index+1]-last_index_list[current_index] <= index_threshold):
            del first_index_list[current_index+1]
            del last_index_list[current_index]
            total_length-=1
        else:
            current_index+=1

    object_dict = {}
    object_dict['distance']  = []
    object_dict['direction'] = []    
    object_dict['risk_val']  = []

    for i in range(len(first_index_list)):
        section = lidar_array[first_index_list[i]:last_index_list[i]]
        lidar_closest_dist = np.min(section)
        lidar_closest_idx = first_index_list[i] + np.argmin(section)
        lidar_closest_dir  = lidar_closest_idx * idx_per_deg + (-camera_fov) # (-camera_fov) OR lidar_min_deg
        object_dict['distance'].append(lidar_closest_dist)
        object_dict['direction'].append(lidar_closest_dir)    
        object_dict['risk_val'].append(cal_risk(0,lidar_closest_dist, lidar_closest_dir,0,0))
    return object_dict
    #TODO: make the RM calculates all the values, then take min or average of the generated scaling speed


def lidar_callback(data):
    global detected_object
    obstacle_zone = 2.0
    r_warning = 0.3
    r_critical = 0.15
    #print('data len: ',len(data.ranges))
    sensor_reads = data.ranges[lidar_idx_min:lidar_idx_max]##[2092:4185] #only consider the object within camera's FoV
    

    #lidar_closest_dist = min(sensor_reads)
    #lidar_closest_dist = (lidar_closest_dist_idx - n_front) * idx_per_deg
    sensor_reads = np.array(sensor_reads)
    nan_indexes = np.isnan(sensor_reads)
    sensor_reads[nan_indexes] = lidar_max_distance + 1
    sensor_reads = np.array(list(value if value > 0.0499 else lidar_max_distance + 1 for value in sensor_reads ))
    
    object_dict = {}
    object_dict['distance']  = []
    object_dict['direction'] = []    
    object_dict['risk_val']  = []
    #object_dict = detect_multi_object(sensor_reads) #added for multi object detection
    
    # sensor_reads_list = (value for value in sensor_reads ifnp.array( value > 0.049999999999999)
    # print (sensor_reads)
    lidar_closest_dist = min (sensor_reads) 
    lidar_closest_idx  = sensor_reads.tolist().index(lidar_closest_dist)
    # lidar_closest_dist = sensor_reads[lidar_closest_idx]
    lidar_closest_dir  = lidar_closest_idx * idx_per_deg + (-camera_fov) # (-camera_fov) OR lidar_min_deg

    #object_dict = {} #commented for multi object detection
    #adding 1 object from the reading
    #object_dict['distance']  = [lidar_closest_dist]
    #object_dict['direction'] = [lidar_closest_dir]
    #object_dict['risk_val']  = [cal_risk(lidar_closest_dist, lidar_closest_dir)]
    if len(object_dict['distance']) == 0:
        object_dict['distance'].append(lidar_closest_dist)
        object_dict['direction'].append(lidar_closest_dir)
        #object_dict['risk_val'].append(cal_risk(0, lidar_closest_dist, lidar_closest_dir,0.0, 0.0))#0:obstacle type == static
        object_dict['risk_val'].append(min(4.0,1.1*cal_risk(0, lidar_closest_dist, lidar_closest_dir,0.0, 0.0)))#0:obstacle type == static
        
    #print('len(sensor_reads) : ',len(sensor_reads))
    #print('sensor reads',sensor_reads[int(len(sensor_reads)/2)])
    camera_x_pos = 640 - int(lidar_closest_dir*320/camera_fov + 320)
    list_for_lidar_pointer = [lidar_closest_dist, camera_x_pos, object_dict['risk_val'][0]]
    #print(list_for_lidar_pointer)
    pointer_data = Float64MultiArray()
    pointer_data.data = np.array(list_for_lidar_pointer)
    pointer_pub.publish(pointer_data)

    #risk_val = cal_risk(closest_dist, closest_idx) #risk analysis
    #Calculating the risk from scenegraph
    for single_object in detected_object:
        object_dict['distance'].append(single_object['distance'])
        object_dict['direction'].append(single_object['object_direction'])
        object_dict['risk_val'].append(cal_risk(2,single_object['distance'], single_object['object_direction'], 0.0, 0.0)) #2:obstacle type == human
    riskiest_object_n = np.argmax(object_dict['risk_val'])
    
    #speed_l, speed_r = cal_safe_vel(object_dict['distance'][riskiest_object_n], object_dict['direction'][riskiest_object_n], object_dict['risk_val'][riskiest_object_n]) #risk mitigation
    speed_l_list = []
    speed_r_list = []
    for i in range(len(object_dict['distance'])):
        speed_L, speed_R = cal_safe_vel(object_dict['distance'][i], object_dict['direction'][i], object_dict['risk_val'][i]) #risk mitigation
        speed_l_list.append(speed_L* object_dict['risk_val'][i])
        speed_r_list.append(speed_R* object_dict['risk_val'][i])
    speed_l = np.sum(speed_l_list)/np.sum(object_dict['risk_val'])
    speed_r = np.sum(speed_r_list)/np.sum(object_dict['risk_val'])
    if len(object_dict['distance']) > 1:
        pub_safe_vel(0.8*speed_l, 0.8*speed_r) #when there is human, max speed is only 80%
    else:
        pub_safe_vel(speed_l, speed_r)    
    #print(riskiest_object_n, object_dict['risk_val'], object_dict['direction'],speed_l, speed_r)
    risk_val = object_dict['risk_val'][riskiest_object_n]
    #print(len(object_dict['risk_val']))
    risk_val_pub.publish(risk_val)
    #print(object_dict)
    #print('lidar closest direction: ',lidar_closest_dir, lidar_closest_idx, 'lidar closest distance : ',lidar_closest_dist)#, min(sensor_reads))
    #print('Risk value: ', risk_val, ' Speed l : ', speed_l, ' Speed r : ', speed_r)

    prev_obstacle_zone = obstacle_zone
    if risk_val > 3.0:
        led2_pub.publish(Led.RED)
    elif risk_val > 2.0:
        led2_pub.publish(Led.ORANGE)
    elif risk_val > 1.0:
        led2_pub.publish(Led.GREEN)
    else:
        led2_pub.publish(Led.BLACK)

def object_detection_callback(data):
    global detected_object
    detected_object = []
    for i in range(len(data.type)):
        single_object_dict = {}
        single_object_dict['distance'] = data.mean_distance[i]/1000.0
        single_object_dict['xmin'] = data.xmin[i]
        single_object_dict['xmax'] = data.xmax[i]
        single_object_dict['object_direction'] = -(((data.xmax[i] + data.xmin[i]) / 2.0 ) - camera_mid_width)/(camera_mid_width) * camera_fov  #object direction in degree 
        #print(single_object_dict['xmin'],single_object_dict['xmax'],single_object_dict['object_direction'])
        detected_object.append(single_object_dict)
        

if __name__ == '__main__':
    try:
        rospy.init_node('rm_fls_demo_py')
        global init_lidar_sub
        lidar_initialized = False
        #while not lidar_initialized:
        init_lidar_sub = rospy.Subscriber('/turtlebot2i/lidar/scan', LaserScan, lidar_and_camera_setup) 
        while not lidar_initialized: 
            print('waiting lidar to be set')
            rospy.sleep(2)
        #print('lidar_initialized:',lidar_initialized)
        #init_lidar_sub.unregister()
        init_var()
        print("init_var ok")
        init_fls_common_part()
        print("init_fls_common_part ok")
        init_risk_assessment()
        print("init_risk_assessment ok")
        init_risk_mitigation()
        print("init_risk_mitigation ok")
        safe_vel_pub = rospy.Publisher('/turtlebot2i/safety/vel_scale', VelocityScale, queue_size=10)
        led2_pub  = rospy.Publisher('/turtlebot2i/commands/led2', Led,  queue_size=10)
        risk_val_pub = rospy.Publisher('/turtlebot2i/safety/risk_val', Float64)
        pointer_pub = rospy.Publisher('/turtlebot2i/safety/lidar_pointer', Float64MultiArray)
        #rospy.Subscriber('/turtlebot2i/safety/obstacles_risk', SafetyRisk, risk_callback) 

        rospy.Subscriber('/turtlebot2i/lidar/scan', LaserScan, lidar_callback) 
        rospy.Subscriber('/turtlebot2i/safety/boundingbox', BoundingBox, object_detection_callback)
        print("risk_mitigation_fls.py is now running!")
        rospy.spin()
    except rospy.ROSInterruptException:
        led2_pub.publish(Led.BLACK)
        pass