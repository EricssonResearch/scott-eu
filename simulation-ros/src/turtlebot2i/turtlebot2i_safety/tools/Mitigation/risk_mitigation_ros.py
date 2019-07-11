#!/usr/bin/env python
import rospy # ROS library

import std_msgs.msg 
from geometry_msgs.msg import Twist,Vector3

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


def init_var():
    # Safety zone size
    global IZW, range_degree,range_meter, range_meter_per_second,range_risk
    IZW = 0.4
    step_meter = 0.02 # If the step are large, the Gaussian MF will regress to Triangular MF
    step_meter_per_second = 0.02
    step_risk = 0.05
    range_degree = np.arange(-180, 180+1, 1.0)    # Range: -180 degree ~ 180 degree for direction and orientation
    range_meter  = np.arange(0, 3.0+step_meter, step_meter)         # Range:  0 meter ~ 3 meter for distance
    range_meter_per_second = np.arange(-0.5, 2.0+step_meter_per_second, step_meter_per_second)#Range:  0 mps ~ 2 mps for speed
    range_risk = np.arange(0, 5+step_risk, step_risk)  # Range: 0,1,2,3,4 for risk
    global interWheelDistance
    #s=sim.getObjectSizeFactor(objHandle) -- make sure that if we scale the robot during simulation, other values are scaled too!
    interWheelDistance=0.137#*s
def init_RA():
    object_distance = ctrl.Antecedent(range_meter, 'distance')     # 0- 3  meter
    object_direction  = ctrl.Antecedent(range_degree , 'direction') # -180~180 degree
    object_risk = ctrl.Antecedent(range_risk, 'risk')

    #global left_speed,right_speed # When Consequent need visualization
    left_speed  = ctrl.Consequent(range_meter_per_second, 'left')
    right_speed = ctrl.Consequent(range_meter_per_second, 'right')


    # Custom membership functions can be built interactively with a familiar Pythonic API
    distance_p1 = fuzz.gaussmf(range_meter,IZW,0.1)
    distance_p2 = fuzz.gaussmf(range_meter,IZW,0.1) 
    # Distance
    object_distance['Near']  = fuzz.gaussmf(range_meter,0.5*IZW,0.1) #0.2
    object_distance['Medium']= fuzz.gaussmf(range_meter,IZW,0.1)     #0.4
    object_distance['Far']   = fuzz.gaussmf(range_meter,2.0*IZW,0.2) #0.8
    # Direction
    object_direction['Front']  = fuzz.gaussmf(range_degree,0,15)
    object_direction['FrontLeft']= fuzz.gaussmf(range_degree,45,15)
    object_direction['Left']= fuzz.gaussmf(range_degree,90,15)
    object_direction['FrontRight']  = fuzz.gaussmf(range_degree,-45,15)
    object_direction['Right']  = fuzz.gaussmf(range_degree,-90,15)
    rear_d_p1 = fuzz.gaussmf(range_degree,180,60)
    rear_d_p2 = fuzz.gaussmf(range_degree,-180,60) 
    null,object_direction['BigRear']  =fuzz.fuzzy_or(range_degree,rear_d_p1,range_degree,rear_d_p2)
    # Risk
    object_risk['VeryLow'] = fuzz.gaussmf(range_risk,0,0.3)
    object_risk['Low'] = fuzz.gaussmf(range_risk,1,0.3)
    object_risk['Medium'] = fuzz.gaussmf(range_risk,2,0.3)
    object_risk['High'] = fuzz.gaussmf(range_risk,3,0.3)
    object_risk['VeryHigh'] = fuzz.gaussmf(range_risk,4,0.3)

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
    rule_list=rule_list_generator(object_distance,object_direction, object_risk,left_speed,right_speed)

    global risk_mitigation_instance  # We don't need to change the FLS
    risk_mitigation_fls = ctrl.ControlSystem(rule_list)
    risk_mitigation_instance = ctrl.ControlSystemSimulation(risk_mitigation_fls)

def cal_safe_vel(object_distance,object_direction,object_risk):

    risk_mitigation_instance.input['distance'] = object_distance		
    risk_mitigation_instance.input['direction'] = object_direction		
    risk_mitigation_instance.input['risk'] = object_risk

    risk_mitigation_instance.compute()
    #print risk_mitigation_instance.print_state()
    #left_speed.view(sim=risk_mitigation_instance) # define global
    #right_speed.view(sim=risk_mitigation_instance)

    return risk_mitigation_instance.output['left'],risk_mitigation_instance.output['right']

def pub_safe_vel(left_vel,right_vel):

    linVel = (right_vel+left_vel)/2
    rotVel = (right_vel-left_vel)/(2*interWheelDistance)
    # NEED to check: turtlebot2i_turtlebot2i.lua
    rospy.loginfo("Publish a safety_controller topic")
    rospy.loginfo("left_vel=%1.2f,right_vel=%1.2f",left_vel,right_vel)

    safe_vel_message=Twist()
    safe_vel_message.linear = Vector3()
    safe_vel_message.angular = Vector3()

    safe_vel_message.linear.x = linVel #linear
    safe_vel_message.angular.z= rotVel#angular

    safe_vel_pub.publish(safe_vel_message)

def test_mitigation_ROS():
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        print("Format:")
        input_str = raw_input("distance,direction,risk\n")
        object_distance,object_direction,object_risk = input_str.split(',')
        object_distance = float(object_distance)
        object_direction= float(object_direction)		
        object_risk     = float(object_risk)

        left_vel,right_vel=cal_safe_vel(object_distance,object_direction,object_risk)
        pub_safe_vel(left_vel,right_vel)

        rate.sleep()
""" Main program """
if __name__ == "__main__":  

    init_var()
    init_RA()

    rospy.init_node("prase_ros_node",anonymous=True) #Always first
    print("Initializing parse node finished")

    ## SUBSCRIBERS
    # Creates a subscriber object

    ## PUBLISHERS
    # Creates a publisher object
    safe_vel_pub = rospy.Publisher('/turtlebot2i/cmd_vel_mux/safety_controller', Twist, queue_size=10)
    # Dont't use "/turtlebot2i/commands/velocity'
    # Use "/turtlebot2i/cmd_vel_mux/safety_controller" with "vel_mux" package.   

    test_mitigation_ROS()



