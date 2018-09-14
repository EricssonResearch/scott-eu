#!/usr/bin/env python
#from graphviz import Digraph
import pydot # sudo pip install -I pydot==1.2.4
import re
from turtlebot2i_safety.msg import SceneGraph
import rospy # ROS library
import os
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
    
    #path = os.getcwd()  
    #print ("The current working directory is %s" % path)  #/home/user/.ros
    #print os.popen("command").read() 
    global labels_folder
    labels_folder = os.path.join(os.path.expanduser('~'),'labels')
    #sub_label_folder = os.path.join(labels_folder,'01')
    if not os.path.exists(labels_folder):
        os.makedirs(labels_folder)
        print "Label folder doesn't exist, create one"
    else:
        print "Label folder exists"
    global sub_folder_number
    sub_folder_number = 0 # counter
    string = raw_input('Wait')
''' #To creat a child folder
    sub_label_folder = os.path.join(labels_folder,'01')
    if not os.path.exists(sub_label_folder):
        os.makedirs(sub_label_folder)
        print "Label folder doesn't exist, create one"
    else:
        print "Label folder exists"
'''

    
def parse_dot_file(graph):
    #os.system('')
    sub_folder_number = sub_folder_number+1
    sub_label_folder = os.path.join(labels_folder,'01')
    if not os.path.exists(sub_label_folder):
        os.makedirs(sub_label_folder)
        print "Folder doesn't exist, create one"
    else:
        print "Folder exists"
    
    robot_self_node = graph.get_node('robot')[0]
    if (robot_self_node.get_name()=='robot'):
        node_info= robot_self_node.__get_attribute__("label")
        print "-------------------------------"
        print "-------------------------------"
        
        matchObj = re.match(vel_pattern, node_info,re.M|re.I) #It Works
        
        if matchObj:
            robot_speed = float(matchObj.group(1))
            print "Robot Speed: ",robot_speed
        else:
            print "Error! Robot node doesn't exist!"

    node_list = graph.get_nodes()
    for x in node_list:
        if not ( (x.get_name()=='node') or (x.get_name()=='warehouse') or (x.get_name()=='floor') or (x.get_name()=='robot') ):#All leaf nodes     
            node_info= x.__get_attribute__("label")            
            print "-------------------------------"
            print x.get_name()
            #print type(node_info),node_info
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

                object_distance     = float(matchObj.group(3))
                object_direction    = float(matchObj.group(5))
                object_speed        = float(matchObj.group(6))
                object_orientation  = float(matchObj.group(4))
                #object_risk =   cal_risk(object_distance,object_direction,object_speed,object_orientation) #+zone size(Global Variant) 
                #print "Risk is =",object_risk

            else:
               print "Node not match!!"  

def topic_callback(data):
    graphs = pydot.graph_from_dot_data(data.sg_data) #From string
    (graph,) = graphs
    parse_dot_file(graph)

""" Main program """
if __name__ == "__main__":  
    init_regEx()
    init_var()
    rospy.init_node("prase_ros_node",anonymous=True) #Always first
    print("Initializing parse node finished")
    ## SUBSCRIBERS
    # Creates a subscriber object for each topic

    rospy.Subscriber('/turtlebot2i/safety/scene_graph', SceneGraph, topic_callback)
    rospy.spin()




