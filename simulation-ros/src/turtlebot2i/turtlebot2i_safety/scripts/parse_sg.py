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
    global pattern
    pattern = '"{' + name_pattern+ '\|distance: '+float_pattern+'\|orientation: '+float_pattern+'\|direction: '+float_pattern+'}"'

def parse_dot_file(graph):
    
    node_list = graph.get_nodes()
    #print len(node_list)
    for x in node_list:
        if not ( (x.get_name()=='node') or (x.get_name()=='warehouse')or(x.get_name()=='floor')or(x.get_name()=='robot') ):        
            node_info= x.__get_attribute__("label")
            #print type(node_info),node_info
            print "-------------------------------"
            print x.get_name()
          
            matchObj = re.match(pattern, node_info,re.M|re.I) #It Works

            if matchObj:
               #print "matchObj.group() : ", matchObj.group()
               print "*Obj Name : ", matchObj.group(1)
               print "*Distance : ", float(matchObj.group(2))
               print "*Orientation : ", float(matchObj.group(3))
               print "*Direction: ",float(matchObj.group(4))
            else:
               print "No match!!"   

def topic_callback(data):
    graphs = pydot.graph_from_dot_data(data.sg_data) #From string
    (graph,) = graphs
    parse_dot_file(graph)

""" Main program """
if __name__ == "__main__":  
    init()
    rospy.init_node("prase_ros_node",anonymous=True) #Always first

    ## SUBSCRIBERS
    # Creates a subscriber object for each topic

    rospy.Subscriber('/turtlebot2i_safety/SceneGraph', SceneGraph, topic_callback)
    rospy.spin()




