#!/usr/bin/env python
import rospy # ROS library
import rospkg

# Scene Graph
from turtlebot2i_scene_graph.msg import SceneGraph
from turtlebot2i_safety.msg import VelocityScale, SafetyRisk
import std_msgs.msg

# Parse S-G
import pydot # Default pydot doesn't work. pip install -I pydot==1.2.4
import re

# Fuzzy logic packages
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Safety control MSG
from geometry_msgs.msg import Twist,Vector3

# Debug
import time
import os
import cPickle as pickle

def init_var():
    '''
    Here we initialize the global variables.
    '''
    #global time_previous
    #time_previous = time.time()

    # Safety zone size
    global IZW, range_degree,range_meter, range_meter_per_second,range_risk
    IZW = 0.4
    step_meter = 0.02
    step_meter_per_second = 0.02
    step_risk = 0.05
    range_degree = np.arange(-180, 180+1, 1.0)
    range_meter  = np.arange(0, 3.0+step_meter, step_meter)
    range_meter_per_second = np.arange(-0.5, 2.0+step_meter_per_second, step_meter_per_second)
    range_risk = np.arange(0, 5+step_risk, step_risk)
    global interWheelDistance
    ### NEED CHECK HERE!!!
    #s=sim.getObjectSizeFactor(objHandle) -- make sure that if we scale the robot during simulation, other values are scaled too!
    interWheelDistance=0.137#*s

    global left_vel_scale, right_vel_scale, highest_risk
    highest_risk = 0.0
    left_vel_scale = 1.0
    right_vel_scale = 1.0

    global package_path
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('turtlebot2i_safety')

    global safe_vel_message
    safe_vel_message=Twist()
    safe_vel_message.linear = Vector3()
    safe_vel_message.angular = Vector3()

    global vel_scale_message
    vel_scale_message = VelocityScale()

    global time_duration_list
    time_duration_list = []

def init_regEx():
    '''
    Here we initialize the regluar experssion pattern.
    '''
    name_pattern  = "(\w+#?\d?)"
    float_pattern = "(-?\d+\.\d+)"
    integer_pattern = "(\d+)"
    global sg_pattern,vel_pattern
    sg_pattern = '"{' + name_pattern+'\|type: '+integer_pattern+ '\|distance: '+float_pattern+'\|orientation: '+float_pattern+'\|direction: '+float_pattern+'\|velocity: '+float_pattern+'\|size_x: '+float_pattern+'\|size_y: '+float_pattern+'}"'
    vel_pattern= '"{turtlebot2i\|camera_rgb\|velocity: '+float_pattern+'}"'

def init_fls_common_part(): #TO DO: put this parameter as class or in another module
    global object_distance,object_direction
     # New Antecedent objects
    object_distance = ctrl.Antecedent(range_meter, 'distance')
    object_direction  = ctrl.Antecedent(range_degree , 'direction')

    # Membership functions

    # Distance
    object_distance['Near']  = fuzz.trapmf(range_meter, [0, 0, IZW, 2*IZW])
    object_distance['Medium']= fuzz.trimf(range_meter, [IZW, 2*IZW, 4*IZW])
    object_distance['Far']   = fuzz.trapmf(range_meter, [2*IZW, 4*IZW, 3, 3])
    # Direction -180~180
    rear_d_p2                       = fuzz.trapmf(range_degree, [-180, -180, -135, -90])
    object_direction['Right']       = fuzz.trimf(range_degree, [-135, -90, -45])
    object_direction['FrontRight']  = fuzz.trimf(range_degree, [-90, -45, 0])
    object_direction['Front']       = fuzz.trimf(range_degree, [-45, 0, 45])
    object_direction['FrontLeft']   = fuzz.trimf(range_degree, [0, 45, 90])
    object_direction['Left']        = fuzz.trimf(range_degree, [45, 90, 135])
    rear_d_p1                       = fuzz.trapmf(range_degree, [90, 135, 180,180])
    null,object_direction['BigRear']= fuzz.fuzzy_or(range_degree,rear_d_p1,range_degree,rear_d_p2)
    print("init_fls_common_part")

def init_risk_assessment():
    range_type = np.arange(0, 2+1, 1)
    # New Antecedent/Consequent objects
    object_type        = ctrl.Antecedent(range_type, 'type')
    object_speed       = ctrl.Antecedent(range_meter_per_second, 'speed')
    object_orientation = ctrl.Antecedent(range_degree,'orientation')
    object_risk        = ctrl.Consequent(range_risk, 'risk')
    # Type
    object_type['StaObj'] = fuzz.trimf(range_type, [0, 0, 0.1])
    object_type['DynObj'] = fuzz.trimf(range_type, [0.9, 1, 1.1])
    object_type['Human']  = fuzz.trimf(range_type, [1.9, 2, 2])
    # Speed
    object_speed['Slow']  = fuzz.trapmf(range_meter_per_second, [0, 0, 0.5, 1.0])
    object_speed['Medium']= fuzz.trapmf(range_meter_per_second, [0.5, 1.0, 1.0, 1.5])
    object_speed['Fast']  = fuzz.trimf(range_meter_per_second,[1.0,1.5,1.5])
    # Orientation
    object_orientation['Front']  = fuzz.trimf(range_degree, [-45, 0, 45])
    object_orientation['FrontLeft']=fuzz.trimf(range_degree, [0, 45, 90])
    object_orientation['Left']=  fuzz.trimf(range_degree, [45, 90, 135])
    object_orientation['RearLeft']= fuzz.trimf(range_degree, [90, 135, 180])
    rear_p1 = fuzz.trimf(range_degree, [135, 180,180])
    rear_p2 = fuzz.trimf(range_degree, [-180,-180,-135])
    null,object_orientation['Rear']  =fuzz.fuzzy_or(range_degree,rear_p1,range_degree,rear_p2)
    object_orientation['RearRight']  = fuzz.trimf(range_degree, [-180,-135,-90])
    object_orientation['Right']  = fuzz.trimf(range_degree, [-135,-90,-45])
    object_orientation['FrontRight']  = fuzz.trimf(range_degree, [-90,-45, 0])
    # Risk
    object_risk['VeryLow'] = fuzz.trimf(range_risk, [0, 0, 1])
    object_risk['Low'] =  fuzz.trimf(range_risk, [0, 1, 2])
    object_risk['Medium'] =  fuzz.trimf(range_risk, [1, 2, 3])
    object_risk['High'] =  fuzz.trimf(range_risk, [2, 3, 4])
    object_risk['VeryHigh'] =  fuzz.trimf(range_risk, [3, 4, 4])

    fls_name = "/rules/ra_full.data"
    fls_data_path = package_path + fls_name
    print(fls_data_path)

    #if os.path.exists(fls_name):
    if os.path.exists(fls_data_path):
        print("FLS exists!")
        #f = open(fls_name,'rb')
        f = open(fls_data_path,'rb')
        ra_fls = pickle.load(f)
    else:
        print("Init FLS")
        from assessment_rules import rule_list_generator
        assessment_rule_list=rule_list_generator(object_type,object_distance,object_direction, object_speed, object_orientation, object_risk)
        ra_fls = ctrl.ControlSystem(assessment_rule_list)
        #f = open(fls_name,'wb')
        f = open(fls_data_path,'wb')
        pickle.dump(ra_fls,f)
        f.close
    global risk_assessment_instance
    risk_assessment_instance = ctrl.ControlSystemSimulation(ra_fls)

def cal_risk(object_type,object_distance,object_direction,object_speed,object_orientation):
    risk_assessment_instance.input['type'] = object_type
    risk_assessment_instance.input['distance'] = object_distance
    risk_assessment_instance.input['direction'] = object_direction
    risk_assessment_instance.input['speed'] =   object_speed
    risk_assessment_instance.input['orientation'] = object_orientation

    risk_assessment_instance.compute()
    risk_result = risk_assessment_instance.output['risk']

    return risk_result

def pub_safe_vel(left_vel_scale,right_vel_scale):
    vel_scale_message.header = std_msgs.msg.Header()
    vel_scale_message.header.stamp = rospy.Time.now()

    vel_scale_message.left_vel_scale  = left_vel_scale
    vel_scale_message.right_vel_scale = right_vel_scale

    safe_vel_pub.publish(vel_scale_message)

def parse_dot_file(graph):
    global left_vel_scale, right_vel_scale, highest_risk
    min_left_vel_scale, min_right_vel_scale = 1.0, 1.0
    robot_self_node = graph.get_node('robot')[0]
    if (robot_self_node.get_name()=='robot'):
        node_info= robot_self_node.__get_attribute__("label")
        #print "==============================="

        matchObj = re.match(vel_pattern, node_info,re.M|re.I) #It Works
    node_list = graph.get_nodes()
    highest_risk = 0.0 # Only consider the object with highest risk
    target_object_distance     = 0.0
    target_object_direction    = 0.0
    risk_message = SafetyRisk()
    for x in node_list:
        #print "before analyzing target of: ",x.get_name()
        if not ( (x.get_name()=='node') or (x.get_name()=='warehouse') or (x.get_name()=='floor') or (x.get_name()=='robot') ):#All leaf nodes
            node_info= x.__get_attribute__("label")
            #print "-------------------------------"
            #print node_info
            matchObj = re.match(sg_pattern, node_info,re.M|re.I) #It Works
            #print "analyzing target of: ",x.get_name()
            if matchObj:
                object_type = int(matchObj.group(2))
                object_distance     = float(matchObj.group(3))
                object_direction    = float(matchObj.group(5))
                object_speed        = float(matchObj.group(6))
                object_orientation  = float(matchObj.group(4))
                object_size_x       = float(matchObj.group(7))
                object_size_y       = float(matchObj.group(8))
                #print("in RM:",x.get_name(),object_type,object_distance,object_direction,object_speed,object_orientation)
                if object_type < 3: # type 3 is wall.
                    object_risk = cal_risk(object_type,object_distance,object_direction,object_speed,object_orientation)
                else:
                    object_risk = 0.0
                risk_message.type.append(object_type)
                risk_message.distance.append(object_distance)
                risk_message.direction.append(object_direction)
                risk_message.risk_value.append(object_risk)
                risk_message.speed.append(object_speed)
                risk_message.orientation.append(object_orientation)
                risk_message.size_x.append(object_size_x)
                risk_message.size_y.append(object_size_y)
                if ( object_risk>highest_risk): # Update target
                    target_object_distance =object_distance
                    target_object_direction=object_direction
                    highest_risk=object_risk
                    #print "update target of ",x.get_name()," with risk=",highest_risk
            else:
               print "Node not match!!"
               print node_info
    safe_risk_pub.publish(risk_message) # comment this line for FLS and MLP
    risk_val_pub.publish(highest_risk)
    
    #global    time_previous
    #run_time = time.time() - time_previous
    #print 'Calc. time for S-G=',run_time,'sec'   #0.0139169692993 sec for calc,
    #print 'Calc. Freq. for S-G=',1/run_time,'Hz' #max. 71.8547248681 Hz
    #time_previous = time.time()

def topic_callback(data):
    #global time_duration_list
    #time_previous = time.time()

    graph = pydot.graph_from_dot_data(data.sg_data) #From string

    if (type(graph) is list):
        graph = graph[0]

    parse_dot_file(graph)

    #time_duration_list.append(time.time()-time_previous)
    #print("Risk assesment duration :",np.mean(time_duration_list))
    #if len(time_duration_list) == 100:
    #    dirPath = os.path.dirname(os.path.realpath(__file__))
    #    savePath = self.dirPath.replace('scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_safety/src', 'time_duration/time_duration_ra.npz')
    #    np.savez(savePath, time_duration_list=time_duration_list)

    #rospy.loginfo("The highest risk is %f",risk_result,data.header.stamp)


""" Main program """
if __name__ == "__main__":

    rospy.init_node("risk_assessment",anonymous=True)

    init_var()
    print("init_var ok")
    init_regEx()
    print("init_regEx ok")
    init_fls_common_part()
    print("init_fls_common_part ok")
    init_risk_assessment()
    print("init_risk_assessment ok")

    print("Initializing parse node finished")
    cal_risk(0, 1.0, 0.0, 0.0, 0.0) #FLS takes some time for initialization. Run it once before subscribe to a topic
    ## PUBLISHERS
    # Creates a publisher object
    safe_vel_pub = rospy.Publisher('/turtlebot2i/safety/vel_scale', VelocityScale, queue_size=1)
    risk_val_pub = rospy.Publisher('/turtlebot2i/safety/risk_val', std_msgs.msg.Float64, queue_size=1)
    safe_risk_pub = rospy.Publisher('/turtlebot2i/safety/obstacles_risk', SafetyRisk, queue_size=1)
    ## SUBSCRIBERS
    # Creates a subscriber object
    rospy.Subscriber('/turtlebot2i/scene_graph', SceneGraph, topic_callback)
    print("initialization finished, Risk assesment ready!")
    rospy.spin()
