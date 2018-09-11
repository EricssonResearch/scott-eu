#!/usr/bin/env python
#from graphviz import Digraph
import pydot # Default pydot doesn't work. pip install -I pydot==1.2.4
import re
from turtlebot2i_safety.msg import SceneGraph,SafetyZone
import std_msgs.msg
import rospy # ROS library
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import warnings
warnings.filterwarnings("ignore")  #a lot of warning messages, 'ignroe' doesn't work with ROS
'''About numpy.dtype size change warnings 
https://github.com/ContinuumIO/anaconda-issues/issues/6678
https://stackoverflow.com/questions/40845304/runtimewarning-numpy-dtype-size-changed-may-indicate-binary-incompatibility
jjhelmus: The numpy.dtype size change warnings are coming from Cython and can be safely ignored, see numpy/numpy#432. Stack overflow has a good question of the topic.
'''
def init_regEx():    
    '''
    regular expression
    '''
    name_pattern  = "(\w+#?\d?)"
    float_pattern = "(-?\d+\.\d+)"
    integer_pattern = "(\d+)"
    global sg_pattern,vel_pattern
    sg_pattern = '"{' + name_pattern+'\|type: '+integer_pattern+ '\|distance: '+float_pattern+'\|orientation: '+float_pattern+'\|direction: '+float_pattern+'\|velocity: '+float_pattern+'}"' #Don't forget '\' for '|'
    vel_pattern= '"{turtlebot2i\|camera_rgb\|velocity: '+float_pattern+'}"'

def init_var():
    #global critical_zone_radius = 0.5 #constant 
    global warning_zone_radius  
    warning_zone_radius = 0.5
    global clear_zone_radius    
    clear_zone_radius   = 0.5

def pub_zone_size(speed):
    zone_size_message=SafetyZone() 
    zone_size_message.header = std_msgs.msg.Header()
    zone_size_message.header.stamp = rospy.Time.now()    

    clear_zone_radius = 0.7+8*speed
    warning_zone_radius= 0.6+4*speed
    #critical_zone_radius =0.5#Constant

    zone_size_message.clear_zone_radius =  clear_zone_radius
    zone_size_message.warning_zone_radius= warning_zone_radius
    zone_size_message.critical_zone_radius =0.5#0.5

    pub.publish(zone_size_message)


def init_RA():

    # New Antecedent/Consequent objects hold universe variables and membership functions
    #object_type =     
    global object_distance_Ant
    object_distance_Ant = ctrl.Antecedent(np.arange(0, 2, 1), 'distance')     # 0- 3  meter
    object_direction  = ctrl.Antecedent(np.arange(-180, 180, 1), 'direction') # 0-360 degree
    object_speed = ctrl.Antecedent(np.arange(0, 2, 1), 'speed')		#0- 2 m/s
    object_orientation = ctrl.Antecedent(np.arange(-180, 180, 1), 'orientation')#0-360 degree

    risk = ctrl.Consequent(np.arange(0, 3, 1), 'risk')

    # Auto-membership function population is possible with .automf(3, 5, or 7) #poor,average,good
    object_distance_Ant.automf(3)   	# near, midium, far
   
    object_direction.automf(3)	# left behind right 	#will be changed to custom MF
    object_speed.automf(3)		#slow medium fast
    object_orientation.automf(3)	# left behind right 	#will be changed to custom MF

    risk.automf(3) 

    #Add Fuzzy rules
    #-----------
    #Rulls here are manually defined, but later it will be generated by MATLAB
    rule1 = ctrl.Rule(object_distance_Ant['poor'] | object_speed['good']| object_orientation['average'], risk['good'])
    rule2 = ctrl.Rule(object_distance_Ant['average'], risk['average'])
    rule3 = ctrl.Rule(object_distance_Ant['good'] | object_direction['good'] |object_direction['poor'], risk['poor'])

    #ctrl.Rule.graph
    '''
    object_distance_Ant['poor'] = fuzz.trimf(object_distance_Ant.universe, [0, 0, 1])
    object_distance_Ant['average'] = fuzz.trimf(object_distance_Ant.universe, [0, 1, 2])
    object_distance_Ant['good'] = fuzz.trimf(object_distance_Ant.universe, [1, 2, 2])
    #'''
    #ctrl.Rule.graph
    #Control System Creation and Simulation
    #---------------------------------------
    #Now that we have our rules defined, we can simply create a control system

    risk_assessment_system = ctrl.ControlSystem([rule1, rule2, rule3]) # this is a FL system
    global risk_assessment_instance
    #In order to simulate this control system, we will create a instance (Call it Agent?)
    risk_assessment_instance = ctrl.ControlSystemSimulation(risk_assessment_system)  # this is a FLS instance
 
def cal_risk(object_distance,object_direction,object_speed,object_orientation):
    
    risk_assessment_instance.input['distance'] = object_distance		# 0- 3  meter
    risk_assessment_instance.input['direction'] = object_direction		# 0-360 degree
    risk_assessment_instance.input['speed'] =   object_speed			#0- 2 m/s
    risk_assessment_instance.input['orientation'] = object_orientation		#0-360 degree

    
    #object_distance_Ant.automf(3)    
    risk_assessment_instance.compute()
    risk_result = risk_assessment_instance.output['risk']
    print "3Risk is =",risk_result
    '''
    object_distance.automf(5)
    risk_assessment_instance.compute()
    risk_result = risk_assessment_instance.output['risk']
    print "5Risk is =",object_risk
    object_distance.automf(7)
    risk_assessment_instance.compute()
    risk_result = risk_assessment_instance.output['risk']
    print "7Risk is =",object_risk   
    ''' 
    #print risk_assessment_instance.output['risk'] #This line can not bu run with Python3
    
    return risk_result

def parse_dot_file(graph):
    
    node_list = graph.get_nodes()
    #print len(node_list)
    for x in node_list:
        if not ( (x.get_name()=='node') or (x.get_name()=='warehouse')or(x.get_name()=='floor') ): 
            if (x.get_name()=='robot'):
                node_info= x.__get_attribute__("label")
                print "-------------------------------"
                print "-------------------------------"
                #print x.get_name()
                
                matchObj = re.match(vel_pattern, node_info,re.M|re.I) #It Works
                
                if matchObj:
                    print "Robot Speed: ",float(matchObj.group(1))
                    pub_zone_size(float(matchObj.group(1)))
                else:
                    print "No match"

            else:  #All leave nodes     
                node_info= x.__get_attribute__("label")
                
                print "-------------------------------"
                print x.get_name()
                print type(node_info),node_info
                matchObj = re.match(sg_pattern, node_info,re.M|re.I) #It Works

                if matchObj:
                    locals()[matchObj.group(1)]={'Name':matchObj.group(1),'Type':int(matchObj.group(2)),'Distance':float(matchObj.group(3)),'Orientation':float(matchObj.group(4)),'Direction':float(matchObj.group(5)),'Speed':float(matchObj.group(6))}#IF you want a dict. format
                    #print "matchObj.group() : ", matchObj.group()
                    #print type(locals()[matchObj.group(1)])
                    print "*Obj Name : ", locals()[matchObj.group(1)]['Name']#matchObj.group(1)
                    print "*Obj Type : ", locals()[matchObj.group(1)]['Type']#matchObj.group(2)
                    print "*Distance : ", locals()[matchObj.group(1)]['Distance']#float(matchObj.group(3))
                    print "*Orientation : ", locals()[matchObj.group(1)]['Orientation']#float(matchObj.group(4))
                    print "*Direction: ",locals()[matchObj.group(1)]['Direction']#float(matchObj.group(5))
                    print "*Speed: ",locals()[matchObj.group(1)]['Speed']#float(matchObj.group(6))
                    #Speed is missed here. 
                    object_distance     = float(matchObj.group(3))
                    object_direction    = float(matchObj.group(5))
                    object_speed        = float(matchObj.group(6))
                    object_orientation  = float(matchObj.group(4))
                    object_risk =   cal_risk(object_distance,object_direction,object_speed,object_orientation) #+zone size(Global Variant) 
                    #print "Risk is =",object_risk
                else:
                   print "No match!!"   

def topic_callback(data):
    graphs = pydot.graph_from_dot_data(data.sg_data) #From string
    (graph,) = graphs
    parse_dot_file(graph)
    #rospy.loginfo("The highest risk is %f",risk_result,data.header.stamp)
""" Main program """
if __name__ == "__main__":  

    init_regEx()
    init_var()
    init_RA()
    rospy.init_node("prase_ros_node",anonymous=True) #Always first
    print("Initializing parse node finished")

    ## SUBSCRIBERS
    # Creates a subscriber object for each topic

    rospy.Subscriber('/turtlebot2i/safety/scene_graph', SceneGraph, topic_callback)
    ## PUBLISHERS
    #
    pub = rospy.Publisher('/turtlebot2i/safety/safety_zone', SafetyZone, queue_size=10)
    
    rospy.spin()




