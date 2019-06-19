#!/usr/bin/env python

import time
import vrep
import re
import math
import rospy
import std_msgs.msg
import numpy as np
import sys
from graphviz import Digraph
from shapely.geometry import box
from turtlebot2i_scene_graph.msg import SceneGraph
from vrep_object_extractor import VrepObjectExtractor
from turtlebot2i_safety.msg import VelocityScale

pi = math.pi

# some functions for label message in scene graph nodes
def get_distance(i, j):
    dx = j.pose[0] - i.pose[0]
    dy = j.pose[1] - i.pose[1]
    if not re.match(r'wall*', j.name):
        #ri = math.sqrt(i.size[0]*i.size[0] + i.size[1]*i.size[1])
        #rj = math.sqrt(j.size[0]*j.size[0] + j.size[1]*j.size[1])
        temp_ij = dx*dx + dy*dy
        dist_ij = math.sqrt(temp_ij) #- ri - rj
    else:
        if posi_ix < (posi_wx + size_wx/2) and posi_ix > (posi_wx - size_wx/2):
            dist_ij = dy - size_iy - size_jy
        elif posi_iy < (posi_wy + size_wy/2) and posi_iy > (posi_wy - size_wx/2):
            dist_ij = dx - size_ix - size_jx
        else:
            temp = dx * dx + dy * dy
            dist_ij = math.sqrt(temp - size_ix / 2 - size_jx / 2)

    return dist_ij

def get_distance_bbox(i, j):
    pol_i = box(i.bbox_min[0], i.bbox_min[1], i.bbox_max[0], i.bbox_max[1])
    pol_j = box(j.bbox_min[0], j.bbox_min[1], j.bbox_max[0], j.bbox_max[1])
    min_dist = pol_i.distance(pol_j)
    return min_dist

def get_support_bbox(i, j):
    pol_i = box(i.bbox_min[0], i.bbox_min[1], i.bbox_max[0], i.bbox_max[1])
    pol_j = box(j.bbox_min[0], j.bbox_min[1], j.bbox_max[0], j.bbox_max[1])
    pol_support = pol_i.intersects(pol_j)
    return pol_support

def get_overlap_bbox(i, j):
    pol_i = box(i.bbox_min[0], i.bbox_min[1], i.bbox_max[0], i.bbox_max[1])
    pol_j = box(j.bbox_min[0], j.bbox_min[1], j.bbox_max[0], j.bbox_max[1])
    pol_overl = pol_i.overlaps(pol_j)
    return pol_overl

def get_velocity(j):
    # vel_j = j.vel
    vel_j =  math.sqrt(j.vel[0]*j.vel[0] + j.vel[1]*j.vel[1] + j.vel[2]*j.vel[2])
    return vel_j

def get_direction(i, j):
    dx = j.pose[0] - i.pose[0]
    dy = j.pose[1] - i.pose[1]
    dire_tan = math.atan2(dy, dx) - i.ori[2]
    dire_tan = dire_tan*180/pi
    if dire_tan > 180:
        dire_tan = dire_tan - 360
    elif dire_tan < -180:
        dire_tan = dire_tan + 360
    else:
        pass
    return dire_tan #BUG might hide here.

def get_type(i):
    if re.match(r'80cmHighWall*', i.name):
        obj_type = 3 #wall
    elif re.match(r'Bill*', i.name):
        obj_type = 2 #human
    elif re.match(r'turtlebot*', i.name):
        obj_type = 1 # robot # non-human dynamic objects
    else:
        obj_type = 0 # static objects
    return obj_type

def get_orientation(i, j):
    obj_ori = j.ori[2]*180/pi - i.ori[2]*180/pi
    if obj_ori > 180:
        obj_ori = obj_ori - 360
    elif obj_ori < -180:
        obj_ori = obj_ori + 360
    else:
        pass
    return obj_ori # BUG here

def get_size(j):
    size_x = j.bbox_max[0] - j.bbox_min[0]
    size_y = j.bbox_max[1] - j.bbox_min[1]
    return size_x, size_y

def init():
    global extractor
    extractor= VrepObjectExtractor('127.0.0.1', 19997)
    # List of object names to retrieve information
    # For now it is hardcoded
    extractor.set_wall_names(['80cmHighWall1000cm','80cmHighWall1000cm0','80cmHighWall1500cm','80cmHighWall1500cm0','80cmHighWall1500cm1','80cmHighWall2000cm','80cmHighWall2000cm0',
                              '80cmHighWall500cm','80cmHighWall500cm0','80cmHighWall500cm1','80cmHighWall500cm2','80cmHighWall500cm3','80cmHighWall500cm4','80cmHighWall500cm5',
                              '80cmHighWall200cm','80cmHighWall200cm0','80cmHighWall200cm1','80cmHighWall200cm2','80cmHighWall200cm3','80cmHighWall750cm','80cmHighWall750cm0',
                              '80cmHighWall100cm','80cmHighWall100cm0','80cmHighWall100cm1','80cmHighWall100cm2'])
    extractor.set_static_obj_names(['stairs', 'slidingDoor',
                                    'dockstation_body',
                                    'ConcreteBox','ConcreteBox#0','ConcreteBox#1','ConcreteBox#2','ConcreteBox#3','ConcreteBox#4','ConcreteBox#5','ConcreteBox#6',
                                    'ConcreteBox#7','ConcreteBox#8','ConcreteBox#9','80cmHighPillar100cm','80cmHighPillar100cm0',
                                    'ConveyorBeltBody', 'ConveyorBeltBody#0', 'ConveyorBeltBody#1','ConveyorBeltBody#2', 'ConveyorBeltBody#3', 'ConveyorBeltBody#4', 'ConveyorBeltBody#5', 'ConveyorBeltBody#6', 'ConveyorBeltBody#7', 'ConveyorBeltBody#8', 'ConveyorBeltBody#9'])
    extractor.set_dynamic_obj_names(['Walking_Bill#0','Walking_Bill#1','Walking_Bill#2','Walking_Bill#3','Walking_Bill#4','Bill','Bill#5'])
    extractor.set_robot_names(['turtlebot2i','turtlebot2i#0'])
    
    rospy.loginfo("Connected to remote API server")

    rospy.loginfo('Getting scene properties (this can take a while)...')

    # Get all objects info once (for static properties) and
    #  prepare the callback for the streaming mode

    extractor.operation_mode = vrep.simx_opmode_streaming
    extractor.get_all_objects_info()
    extractor.update_robots_vision_sensor_info()
    extractor.update_all_robots_vision_sensors_fov()
    #time.sleep(0.3) # streaming takes a while to get ready

    extractor.operation_mode = vrep.simx_opmode_buffer
    extractor.get_all_objects_info()
    extractor.update_robots_vision_sensor_info()
    extractor.update_all_robots_vision_sensors_fov()
    #time.sleep(0.3) # streaming takes a while to get ready

    global time_start_list, time_sg_end_list, time_all_end_list
    time_start_list = []
    time_sg_end_list = []
    time_all_end_list = []
    rospy.loginfo('Finished getting scene properties!\n')

def reset():
    global extractor
    extractor.close_connection()
    init()

def sg_generate():

    #robot_list = extractor.robot_obj_list
    robot_list = extractor.get_available_robot()
    #print("robot list:",robot_list)
    if len(robot_list) > 0:
        # Get dynamic object info (pose and vel) periodically
        extractor.update_dynamic_obj_info()

        # Update vision sensor info
        extractor.update_all_robots_vision_sensors_fov()
    else:
        return reset()
    #print("length of robot: ",len(robot_list))
    # Get objects that are in the sensor FOV
    #for robot_num in range(1):
    for robot_obj in robot_list:
        obj_list = extractor.get_objects_from_vision_sensor(robot_obj.vision_sensor) #NOT ROBUST HERE

        if (obj_list != None):
            # Remove the robot itself from the list
            obj_list = [i for i in obj_list if i.name!=robot_obj.name]

        # Print detected objects of the vision sensor
        # TODO: print just in debug mode
        #print(robot_obj.name, robot_obj.vision_sensor.name, obj_list)

        #############################################
        # generate scene graph
        #############################################
        dot = Digraph(comment='warehouse', format='svg')
        dot.node_attr['shape']='record'
        robot_velocity = get_velocity(robot_obj)
        i = robot_obj
        # print(i.bbox_min[0], i.bbox_min[1], i.bbox_max[0], i.bbox_max[1])
        # robot_label = '{%s|%s|velocity: %.2f|orientation: %.2f}'%(robot[robot_num].name, robot[robot_num].vision_sensor.name, robot_velocity, robot[robot_num].ori[2]*180/pi)
        #print("robot_obj.vision_sensor.name:",robot_obj.vision_sensor.name)
        robot_label = '{%s|%s|velocity: %.2f}'%(robot_obj.name, robot_obj.vision_sensor.name, robot_velocity)
        #robot_label = '{%s|type: 0|%s|velocity: %.2f}'%(robot_obj.name, robot_obj.vision_sensor.name, robot_velocity) #Label for itself?

        # robot_label = '{%s|%s}'%(robot[robot_num].name, robot[robot_num].vision_sensor.name)

        dot.node('robot', label=robot_label)
        dot.node('warehouse', label='warehouse')
        dot.node('floor', label='{floor|size: 25*25}')
        dot.edge('warehouse','floor')

        for obj in obj_list:
            obj_direction = get_direction(robot_obj, obj)
            obj_distance = get_distance_bbox(robot_obj, obj)
            obj_velocity = get_velocity(obj)
            obj_type = get_type(obj)
            obj_orientation = get_orientation(robot_obj, obj)
            obj_size_x, obj_size_y = get_size(obj)
            # print(obj.name, '%.3f' %obj_velocity)
            # node_label = '{%s|direction: %s|distance: %.2f}'%(obj.name, obj_direction, obj_distance)
            # if obj.name == 'Bill#3':
            #     node_label = '{%s|velocity: 0.2|distance: %.2f}'%(obj.name, obj_distance)
            # else:
            #     node_label = '{%s|Static|distance: %.2f}'%(obj.name, obj_distance)
            node_label = '{%s|type: %s|distance: %.2f|orientation: %.2f|direction: %.2f|velocity: %.2f|size_x: %.2f|size_y: %.2f}'%( obj.name, obj_type, obj_distance, obj_orientation, obj_direction, obj_velocity, abs(obj_size_x), abs(obj_size_y))
            #node_label = '{%s|type: %s|distance: %.2f|orientation: %.2f|direction: %.2f|velocity: %.2f}'%( obj.name, obj_type, obj_distance, obj_orientation, obj_direction, obj_velocity)
            # node_label = '{%s|velocity: %.2f|distance: %.2f}'%( obj.name, obj_velocity, obj_distance)

            # node_label = '{%s|distance: %.2f}'%(obj.name, obj_distance)

            dot.node(obj.name, label=node_label)
            if re.match(r'wall*', obj.name):
                dot.edge('warehouse', obj.name, label='on')
            elif re.match(r'product*', obj.name):
                for obj_support in obj_list:
                    # if get_support_bbox(obj, obj_support):
                    if get_overlap_bbox(obj, obj_support):
                        dot.edge(obj_support.name, obj.name, label='on')
                        break
                    else:
                        dot.edge('floor', obj.name, label='on')
                        break
            else:
                dot.edge('floor', obj.name, label='on')

        sg_message=SceneGraph()
        sg_message.header = std_msgs.msg.Header()
        sg_message.header.stamp = rospy.Time.now()

        sg_message.sg_data=dot.source

        #print("robot's name is:",robot_obj.name)
        if (robot_obj.name=='turtlebot2i'):
            pub1.publish(sg_message)
        elif (robot_obj.name=='turtlebot2i#0'):
            pub2.publish(sg_message)
        else:
            print("Do not publish anything, check the code")

def vel_scale_callback(data):
    global time_start_list, time_sg_end_list, time_all_end_list
    time_all_end_list.append(time.time())
    if len(time_all_end_list) == 100:
        np.savez('/home/etrrhmd/duration_result/time_duration_sg_vrep.npz', time_start_list=time_start_list, time_sg_end_list=time_sg_end_list, time_all_end_list=time_all_end_list)
        print("saving file")

if __name__ == '__main__':

    rospy.init_node('sg_generator', anonymous=True)

    try:
        init()
        rospy.loginfo('Started getting scene objects from vision sensor FOV...')
        pub1 = rospy.Publisher('/turtlebot2i/scene_graph', SceneGraph, queue_size=10)
        pub2 = rospy.Publisher('/turtlebot2i_0/scene_graph', SceneGraph, queue_size=10)
        rate = rospy.Rate(5.0) #Hz, T=1/Rate
        #rospy.Subscriber('/turtlebot2i/safety/vel_scale', VelocityScale, vel_scale_callback) 
        while not rospy.is_shutdown():
            #time_start_list.append(time.time())
            sg_generate()
            #time_sg_end_list.append(time.time())
            #last_duration = time_sg_end_list[-1]-time_start_list[-1]
            #print("Scene graph last duration:",last_duration)
            rate.sleep()
    except rospy.ROSInterruptException:
        # Close the connection to V-REP
        extractor.close_connection()
        #vrep.simxFinish(clientID)
        pass
