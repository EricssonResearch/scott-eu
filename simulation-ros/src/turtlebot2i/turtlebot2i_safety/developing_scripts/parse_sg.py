#!/usr/bin/env python
#from graphviz import Digraph
import pydot # sudo pip install -I pydot==1.2.4
import re
from turtlebot2i_safety.msg import SceneGraph,SafetyZone
import std_msgs.msg
import rospy # ROS library

def init():
    print("Initializing parse node") 
    '''
    regular expression
    '''
    name_pattern  = "(\w+#?\d?)"
    float_pattern = "(-?\d+\.\d+)"
    global sg_pattern,vel_pattern
    sg_pattern = '"{' + name_pattern+ '\|distance: '+float_pattern+'\|orientation: '+float_pattern+'\|direction: '+float_pattern+'}"' #Don't forget '\' for '|'
    vel_pattern= '"{turtlebot2i\|camera_rgb\|velocity: '+float_pattern+'}"'
def pub_zone_size(speed):
    zone_size_message=SafetyZone() 
    zone_size_message.header = std_msgs.msg.Header()
    zone_size_message.header.stamp = rospy.Time.now()    

    zone_size_message.clear_zone_radius = 0.7+8*speed
    zone_size_message.warning_zone_radius= 0.6+4*speed
    zone_size_message.critical_zone_radius =0.5#0.5
    pub.publish(zone_size_message)
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
                #print type(node_info),node_info
                print "-------------------------------"
                print x.get_name()
              
                matchObj = re.match(sg_pattern, node_info,re.M|re.I) #It Works

                if matchObj:
                   locals()[matchObj.group(1)]={'Name':matchObj.group(1),'Distance':float(matchObj.group(2)),'Orientation':float(matchObj.group(3)),'Direction':float(matchObj.group(4)),'Speed':0}#
                   #print "matchObj.group() : ", matchObj.group()
                   #print type(locals()[matchObj.group(1)])
                   print "*Obj Name : ", locals()[matchObj.group(1)]['Name']#matchObj.group(1)
                   print "*Distance : ", locals()[matchObj.group(1)]['Distance']#float(matchObj.group(2))
                   print "*Orientation : ", locals()[matchObj.group(1)]['Orientation']#float(matchObj.group(3))
                   print "*Direction: ",locals()[matchObj.group(1)]['Direction']#float(matchObj.group(4))
                   #Speed is missed here. 
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

    rospy.Subscriber('/turtlebot2i/safety/scene_graph', SceneGraph, topic_callback)
    ## PUBLISHERS
    #
    pub = rospy.Publisher('/turtlebot2i/safety/safety_zone', SafetyZone, queue_size=10)
    
    rospy.spin()




