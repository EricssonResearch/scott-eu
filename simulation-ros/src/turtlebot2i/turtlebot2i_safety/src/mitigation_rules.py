#!/usr/bin/env python
from skfuzzy import control as ctrl
'''
    Here are all mitigation rules. 
'''

def rule_list_generator(object_distance,object_direction,object_risk_input,left_speed,right_speed): 
    # Mitigation Rule Format
    #rule00X= ctrl.Rule(object_distance['Near']&object_direction[]&object_risk_input , (left_speed['???'],right_speed['???']))
    rule001= ctrl.Rule(object_distance['Near'], (left_speed['Slow'],right_speed['Slow']))
    ''' Turn left
    '''
    rule002= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['VeryLow'], (left_speed['Medium'],right_speed['Fast']))
    rule003= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['Low'], (left_speed['Medium'],right_speed['Fast']))
    rule004= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['Medium'], (left_speed['Medium'],right_speed['Fast']))
    rule005= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['High'], (left_speed['Slow'],right_speed['Medium']))
    rule006= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Medium']))

    rule007= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule008= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['Low'], (left_speed['Fast'],right_speed['Fast']))
    rule009= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['Medium'], (left_speed['Fast'],right_speed['Medium']))
    rule010= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['High'], (left_speed['Medium'],right_speed['Slow']))
    rule011= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['VeryHigh'], (left_speed['Medium'],right_speed['Slow']))

    rule012= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule013= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['Low'], (left_speed['Fast'],right_speed['Fast']))
    rule014= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['Medium'], (left_speed['Medium'],right_speed['Medium']))
    rule015= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['High'], (left_speed['Slow'],right_speed['Slow']))
    rule016= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Slow']))

    rule017= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule018= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['Low'], (left_speed['Fast'],right_speed['Fast']))
    rule019= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['Medium'], (left_speed['Medium'],right_speed['Fast']))
    rule020= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['High'], (left_speed['Slow'],right_speed['Medium']))
    rule021= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Medium']))

    rule022= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))   
    rule023= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['Low'], (left_speed['Fast'],right_speed['Fast'])) 
    rule024= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['Medium'], (left_speed['Medium'],right_speed['Medium'])) 
    rule025= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['High'], (left_speed['Slow'],right_speed['Slow'])) 
    rule026= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Slow'])) 

    rule027= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule028= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['Low'], (left_speed['Fast'],right_speed['Fast']))
    rule029= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['Medium'], (left_speed['Medium'],right_speed['Slow']))
    rule030= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['High'], (left_speed['Medium'],right_speed['Medium']))
    rule031= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['VeryHigh'], (left_speed['Medium'],right_speed['Medium']))

    rule032= ctrl.Rule( object_distance['Far'] & object_direction['FrontLeft'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule033= ctrl.Rule( object_distance['Far'] & object_direction['FrontLeft'] & object_risk_input['Low'], (left_speed['Fast'],right_speed['Fast']))
    rule034= ctrl.Rule( object_distance['Far'] & object_direction['FrontLeft'] & object_risk_input['Medium'], (left_speed['Fast'],right_speed['Fast']))
    rule035= ctrl.Rule( object_distance['Far'] & object_direction['FrontLeft'] & object_risk_input['High'], (left_speed['Medium'],right_speed['Medium']))
    rule036= ctrl.Rule( object_distance['Far'] & object_direction['FrontLeft'] & object_risk_input['VeryHigh'], (left_speed['Medium'],right_speed['Medium']))

    rule037= ctrl.Rule( object_distance['Far'] & object_direction['Left'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule038= ctrl.Rule( object_distance['Far'] & object_direction['Left'] & object_risk_input['Low'], (left_speed['Fast'],right_speed['Fast']))
    rule039= ctrl.Rule( object_distance['Far'] & object_direction['Left'] & object_risk_input['Medium'], (left_speed['Fast'],right_speed['Fast']))
    rule040= ctrl.Rule( object_distance['Far'] & object_direction['Left'] & object_risk_input['High'], (left_speed['Fast'],right_speed['Fast']))
    rule041= ctrl.Rule( object_distance['Far'] & object_direction['Left'] & object_risk_input['VeryHigh'], (left_speed['Fast'],right_speed['Fast']))

    rule042= ctrl.Rule( object_distance['Far'] & object_direction['FrontRight'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule043= ctrl.Rule( object_distance['Far'] & object_direction['FrontRight'] & object_risk_input['Low'], (left_speed['Fast'],right_speed['Fast']))
    rule044= ctrl.Rule( object_distance['Far'] & object_direction['FrontRight'] & object_risk_input['Medium'], (left_speed['Fast'],right_speed['Fast']))
    rule045= ctrl.Rule( object_distance['Far'] & object_direction['FrontRight'] & object_risk_input['High'], (left_speed['Medium'],right_speed['Medium']))
    rule046= ctrl.Rule( object_distance['Far'] & object_direction['FrontRight'] & object_risk_input['VeryHigh'], (left_speed['Medium'],right_speed['Medium']))

    rule047= ctrl.Rule( object_distance['Far'] & object_direction['Right'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))   
    rule048= ctrl.Rule( object_distance['Far'] & object_direction['Right'] & object_risk_input['Low'], (left_speed['Fast'],right_speed['Fast']))
    rule049= ctrl.Rule( object_distance['Far'] & object_direction['Right'] & object_risk_input['Medium'], (left_speed['Fast'],right_speed['Fast'])) 
    rule050= ctrl.Rule( object_distance['Far'] & object_direction['Right'] & object_risk_input['High'], (left_speed['Fast'],right_speed['Fast'])) 
    rule051= ctrl.Rule( object_distance['Far'] & object_direction['Right'] & object_risk_input['VeryHigh'], (left_speed['Fast'],right_speed['Fast'])) 

    rule_list = [rule001, rule002, rule003,rule004, rule005, rule006, rule007, rule008, rule009, rule010,rule011, rule012, rule013,rule014, rule015, rule016,rule017, rule018, rule019, rule020,rule021, rule022, rule023,rule024, rule025, rule026, rule027, rule028, rule029, rule030,rule031, rule032, rule033,rule034, rule035, rule036, rule037, rule038, rule039, rule040,rule041, rule042, rule043,rule044, rule045, rule046, rule047, rule048, rule049, rule050,rule051]
    return rule_list
