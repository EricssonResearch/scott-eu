#!/usr/bin/env python
#from graphviz import Digraph
import pydot # sudo pip install -I pydot==1.2.4
import re

#dot = Digraph()

#"""
dot_file="robot0.dot"
f = open(dot_file, 'rt')
graph_data = f.read()
f.close()
print (type(graph_data))
graphs = pydot.graph_from_dot_data(graph_data) #From string
#"""
#graph = pydot.graph_from_dot_file("robot0.dot")
(g,) = graphs
#print type(graphs)   #a list of graphs
#print "graph data"
#print type(g) #(dot.source) 
#print g.get_node("warehouse")
node_list = g.get_nodes()

'''
regular expression
'''
'''    "{DockStationBody#0|distance: 2.61|orientation: 174.79|direction: 350.52}" '''
name_pattern  = "(\w+#?\d?)"
float_pattern = "(\d+\.\d+)"
pattern = '"{' + name_pattern+ '\|distance: '+float_pattern+'\|orientation: '+float_pattern+'\|direction: '+float_pattern+'}"'

print len(node_list)
for x in node_list:
    if not x.get_name()=='node':        
        node_info= x.__get_attribute__("label")
        print type(node_info),node_info
        
        #''' 
        
        matchObj = re.match(pattern, node_info,re.M|re.I) #It Works

        if matchObj:
           print "matchObj.group() : ", matchObj.group()
           print "Obj Name : ", matchObj.group(1)#.replace("|", "")
           print "distance : ", matchObj.group(2)#.replace("|", "")
           print "orientation : ", matchObj.group(3)#.replace("|", "")
           print "direction: ", matchObj.group(4)#.replace("|", "")
        else:
           print "No match!!"   
        #''' 
''' # Edit Nodes
g.del_edge("floor","slidingDoor")
g.del_node("slidingDoor")
print len(g.get_nodes())
'''

#g.write_png('dot2_graph.png')
#g.write_pdf("test.pdf")

