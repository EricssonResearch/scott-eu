#!/usr/bin/env python
"""
 Fuzzy logic system for risk assessment

    Input: Scene-graph content
    Output: Risk level
"""
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

global IZW # Safety zone size
IZW = 0.4  # This is an example. It will be received as a ROS topic.

# Antecedent/Consequent objects hold universe variables and membership functions
# Universe variables: Here we define the Antecedent/Consequent range 
# We can have a input outside the range: No range check
# np.arange(min, max, step) : step will influence accuracy (esp. curve e.g. Gaussian MF)
step_meter = 0.02 # If the step are large, the Gaussian MF will regress to Triangular MF
step_meter_per_second = 0.02
step_risk = 0.05
range_degree = np.arange(-180, 180+1, 1.0)    # Range: -180 degree ~ 180 degree for direction and orientation
range_meter  = np.arange(0, 3.0+step_meter, step_meter)         # Range:  0 meter ~ 3 meter for distance
range_meter_per_second = np.arange(0, 2.0+step_meter_per_second, step_meter_per_second)#Range:  0 mps ~ 2 mps for speed
range_risk = np.arange(0, 5+step_risk, step_risk)  # Range: 0,1,2,3,4 for risk

object_distance = ctrl.Antecedent(range_meter, 'distance')     # 0- 3  meter
object_direction  = ctrl.Antecedent(range_degree , 'direction') # -180~180 degree
object_speed = ctrl.Antecedent(range_meter_per_second , 'speed')		#0- 3 m/s
object_orientation = ctrl.Antecedent(range_degree , 'orientation')#-180~180 degree

object_risk = ctrl.Consequent(range_risk, 'risk')


# Custom membership functions can be built interactively with a familiar Pythonic API
distance_p1 = fuzz.gaussmf(range_meter,IZW,0.1)
distance_p2 = fuzz.gaussmf(range_meter,IZW,0.1) 
# Distance
object_distance['Near']  = fuzz.gaussmf(range_meter,0.5*IZW,0.1) # skfuzzy.membership.gaussmf(x, mean, sigma)
object_distance['Medium']= fuzz.gaussmf(range_meter,IZW,0.1)     # x: range_meter or object_distance.universe
object_distance['Far']   = fuzz.gaussmf(range_meter,2.0*IZW,0.1)
#object_distance.view()

# Speed 
object_speed['Slow']  = fuzz.gaussmf(range_meter_per_second,0.5,0.2)
object_speed['Medium']= fuzz.gaussmf(range_meter_per_second,1.0,0.2)
object_speed['Fast']  = fuzz.gaussmf(range_meter_per_second,1.5,0.2)
#object_speed.view()

# Direction
object_direction['Front']  = fuzz.gaussmf(range_degree,0,15)
object_direction['FrontLeft']= fuzz.gaussmf(range_degree,45,15)
object_direction['Left']= fuzz.gaussmf(range_degree,90,15)
object_direction['FrontRight']  = fuzz.gaussmf(range_degree,-45,15)
object_direction['Right']  = fuzz.gaussmf(range_degree,-90,15)
rear_d_p1 = fuzz.gaussmf(range_degree,180,60)
rear_d_p2 = fuzz.gaussmf(range_degree,-180,60) 
null,object_direction['BigRear']  =fuzz.fuzzy_or(range_degree,rear_d_p1,range_degree,rear_d_p2)
#object_direction.view()

# Orientation
object_orientation['Front']  = fuzz.gaussmf(range_degree,0,15)
object_orientation['FrontLeft']= fuzz.gaussmf(range_degree,45,15)
object_orientation['Left']= fuzz.gaussmf(range_degree,90,15)
object_orientation['RearLeft']= fuzz.gaussmf(range_degree,135,15)
rear_p1 = fuzz.gaussmf(range_degree,180,15)
rear_p2 = fuzz.gaussmf(range_degree,-180,15) 
null,object_orientation['Rear']  =fuzz.fuzzy_or(range_degree,rear_p1,range_degree,rear_p2)
object_orientation['RearRight']  = fuzz.gaussmf(range_degree,-135,15)
object_orientation['Right']  = fuzz.gaussmf(range_degree,-90,15)
object_orientation['FrontRight']  = fuzz.gaussmf(range_degree,-45,15) 
object_orientation.view()

object_risk['VeryLow'] = fuzz.gaussmf(range_risk,0,0.3)
object_risk['Low'] = fuzz.gaussmf(range_risk,1,0.3)
object_risk['Medium'] = fuzz.gaussmf(range_risk,2,0.3)
object_risk['High'] = fuzz.gaussmf(range_risk,3,0.3)
object_risk['VeryHigh'] = fuzz.gaussmf(range_risk,4,0.3)
# Auto-membership function population is possible with .automf(3, 5, or 7)
#object_risk.automf(5) #tri mf
#object_risk.view()

#raw_input() # Debug
"""
Fuzzy rules
-----------
    Straight forward rules. 
"""
rule1 =  ctrl.Rule(object_direction['BigRear'],object_risk['VeryLow']) # We ignore objects behind it.
rule2 =  ctrl.Rule(object_distance['Near'], object_risk['VeryHigh']  ) # Too near object is dangerous.
rule3 =  ctrl.Rule(object_distance['Far'], object_risk['VeryLow'] )    # Too far object is safe.

rule3 =  ctrl.Rule( object_speed['Medium'] & object_direction['Front'] & object_speed['Slow']  , object_risk['High']  )
rule4 =  ctrl.Rule( object_speed['Medium'] & object_direction['FrontLeft'] & object_speed['Slow']  , object_risk['Medium']  )
rule5 =  ctrl.Rule( object_speed['Medium'] & object_direction['Left'] & object_speed['Slow']  , object_risk['Low']  )
rule6 =  ctrl.Rule( object_speed['Medium'] & object_direction['FrontRight'] & object_speed['Slow']  , object_risk['Medium']  )
rule7 =  ctrl.Rule( object_speed['Medium'] & object_direction['Right'] & object_speed['Slow'] & object_orientation['Front'] , object_risk['Low']  )


#rule =  ctrl.Rule( object_speed[' '] & object_direction[' '] & object_speed[' ']  , object_risk[' ']  )


rule1.view()

"""

Control System Creation and Simulation
---------------------------------------

Now that we have our rules defined, we can simply create a control system via:
"""

ra_fls = ctrl.ControlSystem([rule1, rule2, rule3,rule4, rule5, rule6, rule7])

"""
In order to simulate this control system, we will create a
``ControlSystemSimulation``.  Think of this object representing our controller
applied to a specific set of cirucmstances. 
"""

risk_assessment_instance = ctrl.ControlSystemSimulation(ra_fls)

"""
We can now simulate our control system by simply specifying the inputs
and calling the ``compute`` method.  
"""
# Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
# Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
risk_assessment_instance.input['distance'] = 0.5		
risk_assessment_instance.input['direction'] = 15.3		
risk_assessment_instance.input['speed'] =   0.2	
risk_assessment_instance.input['orientation'] = 80.0

# Crunch the numbers
risk_assessment_instance.compute()

"""
Once computed, we can view the result as well as visualize it.
"""
print risk_assessment_instance.output['risk']
object_risk.view(sim=risk_assessment_instance)
raw_input()

