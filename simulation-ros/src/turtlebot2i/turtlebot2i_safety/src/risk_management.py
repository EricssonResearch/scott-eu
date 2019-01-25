#!/usr/bin/env python
import rospy # ROS library
# Scene Graph
from turtlebot2i_safety.msg import SceneGraph,SafetyZone
import std_msgs.msg
# Parse S-G
import pydot # Default pydot doesn't work. pip install -I pydot==1.2.4
import re
# Fuzzy logic packages
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
# Safety control MSG
import std_msgs.msg 
from geometry_msgs.msg import Twist,Vector3
# Debug
import time

def init_var():
    '''
    Here we initialize the global variables.
    '''
    global time_previous
    time_previous = time.time()
    
    #global critical_zone_radius = 0.5 #constant 
    global warning_zone_radius  
    warning_zone_radius = 0.5 #Initialization
    global clear_zone_radius    
    clear_zone_radius   = 0.5 #Initialization

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

    global navi_linVel,navi_rotVel
    navi_linVel = 0  #Initialization
    navi_rotVel = 0  #Initialization
 
       
def init_regEx(): 
    '''
    Here we initialize the regluar experssion pattern.
    '''   
    name_pattern  = "(\w+#?\d?)"
    float_pattern = "(-?\d+\.\d+)"
    integer_pattern = "(\d+)"
    global sg_pattern,vel_pattern
    sg_pattern = '"{' + name_pattern+'\|type: '+integer_pattern+ '\|distance: '+float_pattern+'\|orientation: '+float_pattern+'\|direction: '+float_pattern+'\|velocity: '+float_pattern+'}"' 
    vel_pattern= '"{turtlebot2i\|camera_rgb\|velocity: '+float_pattern+'}"'

def init_fls_common_part():
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
    rear_d_p2 = fuzz.trapmf(range_degree, [-180, -180, -135, -90])
    object_direction['Right']  = fuzz.trimf(range_degree, [-135, -90, -45])
    object_direction['FrontRight']  = fuzz.trimf(range_degree, [-90, -45, 0])
    object_direction['Front']  =  fuzz.trimf(range_degree, [-45, 0, 45])
    object_direction['FrontLeft']= fuzz.trimf(range_degree, [0, 45, 90])
    object_direction['Left']= fuzz.trimf(range_degree, [45, 90, 135])
    rear_d_p1 = fuzz.trapmf(range_degree, [90, 135, 180,180]) 
    null,object_direction['BigRear']  =fuzz.fuzzy_or(range_degree,rear_d_p1,range_degree,rear_d_p2)
    print("init_fls_common_part")
def init_risk_assessment():
    range_type = np.arange(0, 2+1, 1)
    # New Antecedent/Consequent objects
    object_type =     ctrl.Antecedent(range_type, 'type') 
    object_speed = ctrl.Antecedent(range_meter_per_second, 'speed')
    object_orientation = ctrl.Antecedent(range_degree,'orientation')
    object_risk = ctrl.Consequent(range_risk, 'risk')
    # Type
    object_type['StaObj'] = fuzz.trimf(range_type, [0, 0, 0.1])
    object_type['DynObj'] = fuzz.trimf(range_type, [0.9, 1, 1.1])
    object_type['Human'] = fuzz.trimf(range_type, [1.9, 2, 2])
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
    
    import os
    import cPickle as pickle 
    fls_name = "ra_full.data"

    if os.path.exists(fls_name):
        print("FLS exists!")
        f = open(fls_name,'rb')
        ra_fls = pickle.load(f)
    else:
        print("Init FLS")
        from assessment_rules import rule_list_generator
        assessment_rule_list=rule_list_generator(object_type,object_distance,object_direction, object_speed, object_orientation, object_risk)
        ra_fls = ctrl.ControlSystem(assessment_rule_list)
        f = open(fls_name,'wb')
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
    #print "Risk is =",risk_result
    
    return risk_result


def init_risk_mitigation():
    # New Antecedent/Consequent objects
    object_risk_input = ctrl.Antecedent(range_risk, 'risk_input')# Different name
    #global left_speed,right_speed # When Consequent need visualization
    left_speed  = ctrl.Consequent(range_meter_per_second, 'left')
    right_speed = ctrl.Consequent(range_meter_per_second, 'right')

    # Risk
    object_risk_input['VeryLow'] = fuzz.gaussmf(range_risk,0,0.3)
    object_risk_input['Low'] = fuzz.gaussmf(range_risk,1,0.3)
    object_risk_input['Medium'] = fuzz.gaussmf(range_risk,2,0.3)
    object_risk_input['High'] = fuzz.gaussmf(range_risk,3,0.3)
    object_risk_input['VeryHigh'] = fuzz.gaussmf(range_risk,4,0.3)
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

def cal_safe_vel(object_distance,object_direction,object_risk):

    risk_mitigation_instance.input['distance'] = object_distance		
    risk_mitigation_instance.input['direction'] = object_direction		
    risk_mitigation_instance.input['risk_input'] = object_risk

    risk_mitigation_instance.compute()
    #print risk_mitigation_instance.print_state()
    #left_speed.view(sim=risk_mitigation_instance) # define global
    #right_speed.view(sim=risk_mitigation_instance)

    return risk_mitigation_instance.output['left'],risk_mitigation_instance.output['right']

def pub_safe_vel(left_vel_scale,right_vel_scale):
    # Global navi vel
    navi_leftVel = (navi_linVel - navi_rotVel*interWheelDistance)*2
    navi_rightVel= (navi_linVel + navi_rotVel*interWheelDistance)*2
    # Apply scale
    left_vel =navi_leftVel*left_vel_scale
    right_vel=navi_rightVel*right_vel_scale

    linVel = (right_vel+left_vel)/2
    rotVel = (right_vel-left_vel)/(2*interWheelDistance)
    # NEED to check: turtlebot2i_turtlebot2i.lua
    rospy.loginfo("Publish a safety_controller topic")
    rospy.loginfo("left_vel_scale=%1.2f,right_vel_scale=%1.2f",left_vel_scale,right_vel_scale)

    safe_vel_message=Twist()
    safe_vel_message.linear = Vector3()
    safe_vel_message.angular = Vector3()

    safe_vel_message.linear.x = linVel #linear
    safe_vel_message.angular.z= rotVel#angular

    safe_vel_pub.publish(safe_vel_message)

def pub_zone_size(speed):
    zone_size_message=SafetyZone() 
    zone_size_message.header = std_msgs.msg.Header()
    zone_size_message.header.stamp = rospy.Time.now()    

    clear_zone_radius = 0.64+12*speed/5.0
    warning_zone_radius= 0.62+6*speed/5.0
    #critical_zone_radius =0.5#Constant

    zone_size_message.clear_zone_radius =  clear_zone_radius
    zone_size_message.warning_zone_radius= warning_zone_radius
    zone_size_message.critical_zone_radius =0.39+0.2 #Robot radius= 0.4

    pub.publish(zone_size_message)

def parse_dot_file(graph):
    robot_self_node = graph.get_node('robot')[0]
    if (robot_self_node.get_name()=='robot'):
        node_info= robot_self_node.__get_attribute__("label")
        print "==============================="
        print "-------------------------------"
        #print x.get_name()
        
        matchObj = re.match(vel_pattern, node_info,re.M|re.I) #It Works
        
        if matchObj:
            robot_speed = float(matchObj.group(1))
            print "Robot Speed: ",robot_speed
            #setup_fls(robot_speed)
            pub_zone_size(robot_speed)
        else:
            print "Error! Robot node doesn't exist!"

    node_list = graph.get_nodes()
    #print len(node_list)
    highest_risk = 0 # Only consider the object with highest risk 
    target_object_distance     = 0
    target_object_direction    = 0
    for x in node_list:
        if not ( (x.get_name()=='node') or (x.get_name()=='warehouse') or (x.get_name()=='floor') or (x.get_name()=='robot') ):#All leaf nodes     
            node_info= x.__get_attribute__("label")            
            print "-------------------------------"
            print x.get_name()
            #print type(node_info),node_info
            matchObj = re.match(sg_pattern, node_info,re.M|re.I) #It Works
            if matchObj:
                object_type         = int(matchObj.group(2))
                object_distance     = float(matchObj.group(3))
                object_direction    = float(matchObj.group(5))
                object_speed        = float(matchObj.group(6))
                object_orientation  = float(matchObj.group(4))
                object_risk =   cal_risk(object_type,object_distance,object_direction,object_speed,object_orientation)
                 
                if ( object_risk>highest_risk): # Update target
                    target_object_distance =object_distance
                    target_object_direction=object_direction
                    highest_risk=object_risk
                    print "update target with risk=",highest_risk
            else:
               print "Node not match!!"   
    if (highest_risk!=0):
        left_vel,right_vel=cal_safe_vel(target_object_distance,target_object_direction,highest_risk)
        pub_safe_vel(left_vel,right_vel)
    print "-------------------------------"
    global    time_previous 
    run_time = time.time() - time_previous  
    print 'Calc. time for S-G=',run_time,'sec'   #0.0139169692993 sec for calc,
    print 'Calc. Freq. for S-G=',1/run_time,'Hz' #max. 71.8547248681 Hz 
    print "==============================="
    time_previous = time.time()

def topic_callback(data):
    time_previous = time.time()
    graphs = pydot.graph_from_dot_data(data.sg_data) #From string
    (graph,) = graphs
    parse_dot_file(graph)
    #rospy.loginfo("The highest risk is %f",risk_result,data.header.stamp)

def navi_vel_callback(data):
    #global navi_linVel,navi_rotVel
    navi_linVel=data.linear.x
    navi_rotVel=data.angular.z

""" Main program """
if __name__ == "__main__":  

    init_var()
    init_regEx()
    init_fls_common_part()
    init_risk_assessment()
    init_risk_mitigation()

    rospy.init_node("prase_ros_node",anonymous=True) #Always first
    print("Initializing parse node finished")

    ## SUBSCRIBERS
    # Creates a subscriber object
    rospy.Subscriber('/turtlebot2i/safety/scene_graph', SceneGraph, topic_callback)
    rospy.Subscriber('/turtlebot2i/cmd_vel_mux/navi', Twist, navi_vel_callback)
    ## PUBLISHERS
    # Creates a publisher object
    pub = rospy.Publisher('/turtlebot2i/safety/safety_zone', SafetyZone, queue_size=10)

    safe_vel_pub = rospy.Publisher('/turtlebot2i/cmd_vel_mux/safety_controller', Twist, queue_size=10)
    # Dont't use "/turtlebot2i/commands/velocity'
    # Use "/turtlebot2i/cmd_vel_mux/safety_controller" with "vel_mux" package.   

    rospy.spin()


