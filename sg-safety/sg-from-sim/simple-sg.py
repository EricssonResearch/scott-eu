#http://graphviz.readthedocs.io/en/stable/manual.html


'''
Simple scene graph

map
   |___robot#0 [geometry {ball}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]
         |___arm#0 [geometry {ball}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]
         
   |___shelf#0 [geometry {box}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]
         |___productGreen#0 [geometry {box}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]

   |___cv_belt#0 [geometry {box}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]

   |___recharge_station#0 [geometry {box}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]

   |___human#0 [geometry {box}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]
'''

from graphviz import Digraph

dot = Digraph(comment='Scene Graph')

#NODES
dot.node('scene','scene')

dot.node('robot#0','robot#0, [geometry {ball}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]')
dot.node('arm#0','arm#0, [geometry {ball}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]')

dot.node('shelf#0','shelf#0, [geometry {box}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]')
dot.node('productGreen#0','productGreen#0, [geometry {box}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]')

dot.node('cv_belt#0','cv_belt#0, [geometry {box}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]')

dot.node('recharge_station#0','recharge_station#0, [geometry {box}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]')
dot.node('human#0','human#0, [geometry {box}, position {1.0, 1.0, 1.0}, rotation {0.0, 0.0, 0.0}, scale {0.1, 0.1, 0.1}]')



#EDGES

dot.edge('scene', 'robot#0')
dot.edge('robot#0', 'arm#0')

dot.edge('scene', 'shelf#0')
dot.edge('shelf#0', 'productGreen#0')

dot.edge('scene', 'cv_belt#0')
dot.edge('scene', 'recharge_station#0')
dot.edge('scene', 'human#0')


print(dot.source)  
dot.render()




