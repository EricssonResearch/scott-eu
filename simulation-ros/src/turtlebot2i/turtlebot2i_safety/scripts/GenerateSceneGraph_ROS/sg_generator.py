#!/usr/bin/env python
from SceneObjectExtractorV4 import SceneObjectExtractor
import time
import re
from graphviz import Digraph
import math   
pi = math.pi
from shapely.geometry import box
import rospy
from turtlebot2i_safety.msg import SceneGraph 
import std_msgs.msg

def get_distance(i, j):
    dx = j.pose[0] - i.pose[0]
    dy = j.pose[1] - i.pose[1]
    if not re.match(r'wall*', j.name):
        ri = math.sqrt(i.size[0]*i.size[0] + i.size[1]*i.size[1])
        rj = math.sqrt(j.size[0]*j.size[0] + j.size[1]*j.size[1])
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
    vel_j = 4 * math.sqrt(j.vel[0]*j.vel[0] + j.vel[1]*j.vel[1] + j.vel[2]*j.vel[2])
    return vel_j

def get_direction(i, j):
    dx = j.pose[0] - i.pose[0]
    dy = j.pose[1] - i.pose[1]
    dire_tan = math.atan2(dy, dx) - i.ori[2]
    dire_tan = dire_tan*180/pi
    return dire_tan

def init():
    global extractor 
    extractor= SceneObjectExtractor('127.0.0.1', 19997)
    print('Connected to remote API server')
    print('Getting scene properties (this can take a while)...') 

    # Get all objects info once (for static properties)
    obj_all = extractor.get_all_objects_info()
    print(obj_all) 
    print('Finished getting scene properties!\n')

def sgGenerate():
    # Get dynamic object info (pose and vel) periodically
    extractor.update_dynamic_obj_info()

    # Update vision sensor info
    extractor.update_all_robots_vision_sensors_fov()

    # Get objects that are in the sensor FOV
    # for robot in extractor.robot_obj_list:
    
    robot = extractor.robot_obj_list
    # print len(robot)
    # for robot_num in range(len(robot)):
    for robot_num in range(1):        
        # robot[robot_num] = extractor.robot_obj_list[robot_num]
        obj_list = extractor.get_objects_from_vision_sensor(robot[robot_num].vision_sensor)

        # Remove the robot itself from the list
        obj_list = [i for i in obj_list if i.name!=robot[robot_num].name]

        # Print detected objects of the vision sensor
        print(robot[robot_num].name, robot[robot_num].vision_sensor.name, obj_list)

        #############################################
        # generate scene graph
        #############################################
        dot = Digraph(comment='warehouse', format='svg')
        dot.node_attr['shape']='record'
        robot_velocity = get_velocity(robot[robot_num])
        i = robot[robot_num]

        robot_label = '{%s|%s|velocity: %.2f}'%(robot[robot_num].name, robot[robot_num].vision_sensor.name, robot_velocity)
        
        # robot_label = '{%s|%s}'%(robot[robot_num].name, robot[robot_num].vision_sensor.name)
        
        dot.node('robot', label=robot_label)
        dot.node('warehouse', label='warehouse')
        dot.node('floor', label='{floor|size: 25*25}')
        dot.edge('warehouse','floor')

        for obj in obj_list:
            obj_direction = get_direction(robot[robot_num], obj)
            obj_distance = get_distance_bbox(robot[robot_num], obj)
            obj_velocity = get_velocity(obj)

            node_label = '{%s|distance: %.2f|orientation: %.2f|direction: %.2f}'%( obj.name, obj_distance, obj.ori[2]*180/pi - robot[robot_num].ori[2]*180/pi, obj_direction)

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

        #output scene graph as string
        #print dot.source
        sg_message=SceneGraph()
        sg_message.header = std_msgs.msg.Header()
        sg_message.header.stamp = rospy.Time.now()

        sg_message.sg_data=dot.source

        pub.publish(sg_message)

if __name__ == '__main__':
    init()
    # Update rate in seconds
    rate = 0.1 #0.5
    time_previous = time.time()
    try:
        print('Started getting scene objects from vision sensor FOV...')
        pub = rospy.Publisher('/turtlebot2i_safety/SceneGraph', SceneGraph, queue_size=10)
        rospy.init_node('sg_generator', anonymous=True)
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():          
            sgGenerate()
            #rate.sleep()
            #''' #If one want to check execution time
            one_run_time = time.time() - time_previous
            #print 'execute time=',one_run_time,'s'           
            print 'execute frequency=',1/float(one_run_time),'Hz'
            time_previous =time.time()
            #'''
    except rospy.ROSInterruptException:
        # Close the connection to V-REP
        extractor.close_connection() 
        #vrep.simxFinish(clientID)       
        pass









