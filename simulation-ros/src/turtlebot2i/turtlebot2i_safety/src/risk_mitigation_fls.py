#!/usr/bin/env python
# This module is intended to convert lidar information into x-y data

import rospy
from turtlebot2i_safety.msg import VelocityScale, SafetyRisk
import std_msgs.msg

# Fuzzy logic packages
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

def init_var():
    '''
    Here we initialize the global variables.
    '''

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

    global vel_scale_message
    vel_scale_message = VelocityScale()

def init_fls_common_part(): #copy this function from risk_management.py; TO DO: put this parameter as class or in another module
    global object_distance,object_direction
     # New Antecedent objects
    object_distance   = ctrl.Antecedent(range_meter, 'distance')
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
    null,object_direction['BigRear']=fuzz.fuzzy_or(range_degree,rear_d_p1,range_degree,rear_d_p2)
    print("init_fls_common_part")


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

def cal_safe_vel(object_distance,object_direction,object_risk):
    risk_mitigation_instance.input['distance']   = object_distance
    risk_mitigation_instance.input['direction']  = object_direction
    risk_mitigation_instance.input['risk_input'] = object_risk
    print("RM input value:",object_distance,object_direction,object_risk)
    risk_mitigation_instance.compute()
    return risk_mitigation_instance.output['left'],risk_mitigation_instance.output['right']

def pub_safe_vel(left_vel_scale,right_vel_scale):
    vel_scale_message.header = std_msgs.msg.Header()
    vel_scale_message.header.stamp = rospy.Time.now()
    vel_scale_message.left_vel_scale  = left_vel_scale
    vel_scale_message.right_vel_scale = right_vel_scale
    safe_vel_pub.publish(vel_scale_message)

def risk_callback(data):
    i = np.argmax(data.risk_value)
    left_vel_scale,right_vel_scale = cal_safe_vel(data.distance[i], data.direction[i], data.risk_value[i])
    pub_safe_vel(left_vel_scale, right_vel_scale)
    

if __name__ == '__main__':
    try:
        rospy.init_node('risk_mitigation_fls_py')
        init_var()
        print("init_var ok")
        init_fls_common_part()
        print("init_fls_common_part ok")
        init_risk_mitigation()
        print("init_risk_mitigation ok")
        rospy.Subscriber('/turtlebot2i/safety/obstacles_risk', SafetyRisk, risk_callback) 
        safe_vel_pub = rospy.Publisher('/turtlebot2i/safety/vel_scale', VelocityScale, queue_size=10)
        print("risk_mitigation_fls.py is now running!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
