#!/usr/bin/env python

from vrep_object_extractor import VrepObjectExtractor
import time
import vrep
# add for generate scene graph
import re
from graphviz import Digraph
import math   
from shapely.geometry import box

# some functions for label message in scene graph nodes
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
    print(pol_support)
    return pol_support

def get_overlap_bbox(i, j):
    pol_i = box(i.bbox_min[0], i.bbox_min[1], i.bbox_max[0], i.bbox_max[1])
    pol_j = box(j.bbox_min[0], j.bbox_min[1], j.bbox_max[0], j.bbox_max[1])
    pol_overlap = pol_i.overlaps(pol_j)
    pol_intersect = pol_i.intersects(pol_j)
    pol_support = pol_overlap | pol_intersect
    # print(pol_support)
    return pol_support

def get_velocity(j):
    # vel_j = j.vel
    vel_j = math.sqrt(j.vel[0]*j.vel[0] + j.vel[1]*j.vel[1] + j.vel[2]*j.vel[2])
    return vel_j

def get_direction(i, j):
    dx = j.pose[0] - i.pose[0]
    dy = j.pose[1] - i.pose[1]
    dire_tan = math.atan2(dy, dx) - i.ori[2]
    # print math.atan2(dy, dx)*180/pi, 'robot', i.ori[2]*180/pi
    dire_tan = dire_tan*180/pi
    if dire_tan > 180:
        dire_tan = dire_tan - 360
    elif dire_tan < -180:
        dire_tan = dire_tan + 360
    else:
        pass
    '''
    if (dire_tan > -pi/8) and (dire_tan < pi/8):
        dire_label = 'right'
    elif (dire_tan >= pi/8) and (dire_tan <= 3*pi/8):
        dire_label = 'front-right'
    elif (dire_tan > 3*pi/8) and (dire_tan < 5*pi/8):
        dire_label = 'front'
    elif (dire_tan >= 5*pi/8) and (dire_tan <= 7*pi/8):
        dire_label = 'front-left'
    elif (dire_tan > 7*pi/8) or (dire_tan < -7*pi/8):
        dire_label = 'left'
    elif (dire_tan >= -7*pi/8) and (dire_tan <= -5*pi/8):
        dire_label = 'back-left'
    elif (dire_tan > -5*pi/8) and (dire_tan < -3*pi/8):
        dire_label = 'back'
    else:
        dire_label = 'back-right'
    '''
    return dire_tan

def get_type(i):
    if re.match(r'Bill*', i.name):
        obj_type = 1
    elif re.match(r'turtlebot*', i.name):
        obj_type = 0
    else:
        obj_type = 2
    return obj_type

def get_orientation(i, j):
    obj_ori = j.ori[2]*180/pi - i.ori[2]*180/pi
    return obj_ori

# Update rate in seconds
#rate = 0.1

pi = math.pi

extractor = VrepObjectExtractor('127.0.0.1', 19997)

# List of object names to retrieve information
# For now it is hardcoded
extractor.set_static_obj_names(['stairs', 'slidingDoor',      
                                'dockstation_body',\
                                'ConveyorBeltBody', 'ConveyorBeltBody#0', 'ConveyorBeltBody#1', 
                                'ShelfBody', 'ShelfBody#0', 'ShelfBody#1'])
extractor.set_dynamic_obj_names(['Bill_base#2',
                                'productGreen#0', 'productGreen#1', 'productGreen#2',
                                'productYellow#0', 'productYellow#1', 'productYellow#2',
                                'productRed#0', 'productRed#1', 'productRed#2'])
extractor.set_robot_names(['turtlebot2i', 'turtlebot2i#0'])

# extractor.set_static_obj_names(['stairs', 'slidingDoor',      
#                                 'dockstation_body',\
#                                 'ConveyorBeltBody', 'ConveyorBeltBody#0', 'ConveyorBeltBody#1', 
#                                 'ShelfBody', 'ShelfBody#0', 'ShelfBody#1'])
# extractor.set_dynamic_obj_names(['Bill#2'])
# extractor.set_robot_names(['turtlebot2i'])

print('Connected to remote API server')

print('Getting scene properties (this can take a while)...') 

# Get all objects info once (for static properties) and
#  prepare the callback for the streaming mode

extractor.operation_mode = vrep.simx_opmode_streaming
extractor.get_all_objects_info() 
extractor.update_robots_vision_sensor_info()
extractor.update_all_robots_vision_sensors_fov()
time.sleep(0.3) # streaming takes a while to get ready

extractor.operation_mode = vrep.simx_opmode_buffer
extractor.get_all_objects_info() 
extractor.update_robots_vision_sensor_info()
extractor.update_all_robots_vision_sensors_fov()


print('Finished getting scene properties!\n')

print('Started getting scene objects from vision sensor FOV...')

while True:
    # tt = 2
    time_start = time.time()

    # Get dynamic object info (pose and vel) periodically
    extractor.update_dynamic_obj_info() 

    # Update vision sensor info
    extractor.update_all_robots_vision_sensors_fov()

    robot_list = extractor.robot_obj_list
    # Get objects that are in the sensor FOV
    for robot_num in range(len(robot_list)):
        obj_list = extractor.get_objects_from_vision_sensor(robot_list[robot_num].vision_sensor)

        if (obj_list != None):
            # Remove the robot itself from the list
            obj_list = [i for i in obj_list if i.name!=robot_list[robot_num].name]

        # Print detected objects of the vision sensor
        print(robot_list[robot_num].name, robot_list[robot_num].vision_sensor.name, obj_list)

        #############################################
        # generate scene graph
        #############################################
        dot = Digraph(comment='warehouse', format='png')
        dot.node_attr['shape']='record'
        robot_velocity = get_velocity(robot_list[robot_num])
        i = robot_list[robot_num]
        # print(i.bbox_min[0], i.bbox_min[1], i.bbox_max[0], i.bbox_max[1])
        # robot_label = '{%s|%s|velocity: %.2f|orientation: %.2f}'%(robot[robot_num].name, robot[robot_num].vision_sensor.name, robot_velocity, robot[robot_num].ori[2]*180/pi)
        robot_label = '{%s|type: 0|%s|velocity: %.2f}'%(robot_list[robot_num].name, robot_list[robot_num].vision_sensor.name, robot_velocity)
        
        # robot_label = '{%s|%s}'%(robot[robot_num].name, robot[robot_num].vision_sensor.name)
        
        dot.node('robot', label=robot_label)
        dot.node('warehouse', label='warehouse')
        dot.node('floor', label='{floor|size: 25*25}')
        dot.edge('warehouse','floor')

        for obj in obj_list:
            obj_direction = get_direction(robot_list[robot_num], obj)
            obj_distance = get_distance_bbox(robot_list[robot_num], obj)
            obj_velocity = get_velocity(obj)
            obj_type = get_type(obj)
            obj_orientation = get_orientation(robot_list[robot_num], obj)
            # print(obj.name, '%.3f' %obj_velocity)
            # node_label = '{%s|direction: %s|distance: %.2f}'%(obj.name, obj_direction, obj_distance)  
            # if obj.name == 'Bill#3':
            #     node_label = '{%s|velocity: 0.2|distance: %.2f}'%(obj.name, obj_distance)
            # else:
            #     node_label = '{%s|Static|distance: %.2f}'%(obj.name, obj_distance)
            node_label = '{%s|type: %s|distance: %.2f|orientation: %.2f|direction: %.2f|velocity: %.2f|size: x %.2f, y %.2f, z %.2f}'%( obj.name, obj_type, obj_distance, obj_orientation, obj_direction, obj_velocity, obj.size[0], obj.size[1], obj.size[2])
            # node_label = '{%s|velocity: %.2f|distance: %.2f}'%( obj.name, obj_velocity, obj_distance)
                
            # node_label = '{%s|distance: %.2f}'%(obj.name, obj_distance)
            
            dot.node(obj.name, label=node_label)
            support_flg = 0
            if re.match(r'wall*', obj.name):
                dot.edge('warehouse', obj.name, label='on')
            elif re.match(r'product*', obj.name):
                # obj_list = obj_list.remove(obj)
                for obj_support in obj_list:
                    if obj_support.name[0:5] != obj.name[0:5]:
                    # if get_support_bbox(obj, obj_support):
                        if get_overlap_bbox(obj, obj_support):                    
                            dot.edge(obj_support.name, obj.name, label='on')
                            support_flg = 1
                            break

                if support_flg == 0:        
                    dot.edge('floor', obj.name, label='on')

            else:
                dot.edge('floor', obj.name, label='on')
        '''
        L = [floor]
        assign_object = []

        while jj not in assign_object:
            if len(L) != 0:
                parent = L[0]
                L.pop(0)
                for i in obj_list:
                    dot.node(i.name, label='%s'%i.name)
                    dot.edge(parent.name, i.name, label='on')
                    L.append(i)

        for i in range(len()):
            for j in range(i, len())
                dot.edge(obj_list[i].name, obj_list[j].name, label='')
        '''
        #output scene graph as .svg file in 
        sg_name = 'sg_robot/robot%d' %robot_num
        dot.render(sg_name, view=True)

    time_end = time.time()
    time_cost = time_end - time_start   
    scene_graph_fps = 1.0/time_cost
    # print("Scene graph generating fps is %.2f" % scene_graph_fps)

    #time.sleep(rate)
clientID=extractor.clientID # first method
# Close the connection to V-REP
vrep.simxFinish(clientID)
