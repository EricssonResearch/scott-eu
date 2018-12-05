#!/usr/bin/env python
#from graphviz import Digraph
import pydot # sudo pip install -I pydot==1.2.4
import re
from turtlebot2i_safety.msg import SceneGraph
import rospy # ROS library

def init():
    print("Initializing parse node") 
    '''
    regular expression
    '''
    name_pattern  = "(\w+#?\d?)"
    float_pattern = "(-?\d+\.\d+)"
    integer_pattern = "(\d+)"
    global sg_pattern,vel_pattern
    sg_pattern = '"{' + name_pattern+'\|type: '+integer_pattern+ '\|distance: '+float_pattern+'\|orientation: '+float_pattern+'\|direction: '+float_pattern+'\|velocity: '+float_pattern+'}"' #Don't forget '\' for '|'
    vel_pattern= '"{turtlebot2i\|camera_rgb\|velocity: '+float_pattern+'}"'

def parse_dot_file(graph):

    robot_self_node = graph.get_node('robot')[0]
    if (robot_self_node.get_name()=='robot'):
        node_info= robot_self_node.__get_attribute__("label")
        print "-------------------------------"
        print "-------------------------------"
        #print x.get_name()
        
        matchObj = re.match(vel_pattern, node_info,re.M|re.I) #It Works
        
        if matchObj:
            robot_speed = float(matchObj.group(1))
            print "Robot Speed: ",robot_speed
            
        else:
            print "Error! Robot node doesn't exist!"

    node_list = graph.get_nodes()
    #print len(node_list)
    highest_risk = 0 # Only consider the object with highest risk 
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


            else:
               print "Node not match!! Check RegEx pattern"   

def topic_callback(data):
    #print "Receive one scene graph"
    graphs = pydot.graph_from_dot_data(data.sg_data) #From string
    (graph,) = graphs
    parse_dot_file(graph)

""" Main program """
if __name__ == "__main__":  
    init()
    rospy.init_node("prase_ros_node",anonymous=True) #Always first

    ## SUBSCRIBERS
    # Creates a subscriber object for each topic

    rospy.Subscriber('/turtlebot2i/safety/scene_graph', SceneGraph, topic_callback)
    rospy.spin()




