#!/usr/bin/env python
from skfuzzy import control as ctrl
'''
    Here are all mitigation rules. Will be checked again.
'''

def rule_list_generator(object_distance,object_direction,object_risk_input,left_speed,right_speed): 
    # Mitigation Rule Format
    #rule00X= ctrl.Rule(object_distance['Near']&object_direction[]&object_risk_input , (left_speed['???'],right_speed['???']))
    rule001= ctrl.Rule(object_distance['Near'], (left_speed['Stop'],right_speed['Stop']))
    ''' Turn left
    TODO: More here (5 Risk level)
    '''
    rule002= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['VeryLow'], (left_speed['Medium'],right_speed['Fast']))
    rule003= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['Medium'], (left_speed['Medium'],right_speed['Fast']))
    rule004= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Medium']))

    rule005= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule006= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['Medium'], (left_speed['Fast'],right_speed['Medium']))
    rule007= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['VeryHigh'], (left_speed['Medium'],right_speed['Slow']))

    rule008= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule009= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['Medium'], (left_speed['Medium'],right_speed['Medium']))
    rule010= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Slow']))

    rule011= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule012= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['Medium'], (left_speed['Medium'],right_speed['Fast']))
    rule013= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Medium']))

    rule014= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))   
    rule015= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['Medium'], (left_speed['Medium'],right_speed['Medium'])) 
    rule016= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Slow'])) 
    ''' Reduce speed
    TODO: More here (5 Risk level)
    '''
    rule017= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule018= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['Medium'], (left_speed['Medium'],right_speed['Slow']))
    rule019= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['VeryHigh'], (left_speed['Medium'],right_speed['Medium']))

    rule020= ctrl.Rule( object_distance['Far'] & object_direction['FrontLeft'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule021= ctrl.Rule( object_distance['Far'] & object_direction['FrontLeft'] & object_risk_input['Medium'], (left_speed['Fast'],right_speed['Fast']))
    rule022= ctrl.Rule( object_distance['Far'] & object_direction['FrontLeft'] & object_risk_input['VeryHigh'], (left_speed['Medium'],right_speed['Medium']))

    rule023= ctrl.Rule( object_distance['Far'] & object_direction['Left'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule024= ctrl.Rule( object_distance['Far'] & object_direction['Left'] & object_risk_input['Medium'], (left_speed['Fast'],right_speed['Fast']))
    rule025= ctrl.Rule( object_distance['Far'] & object_direction['Left'] & object_risk_input['VeryHigh'], (left_speed['Fast'],right_speed['Fast']))

    rule026= ctrl.Rule( object_distance['Far'] & object_direction['FrontRight'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))
    rule027= ctrl.Rule( object_distance['Far'] & object_direction['FrontRight'] & object_risk_input['Medium'], (left_speed['Fast'],right_speed['Fast']))
    rule028= ctrl.Rule( object_distance['Far'] & object_direction['FrontRight'] & object_risk_input['VeryHigh'], (left_speed['Medium'],right_speed['Medium']))

    rule029= ctrl.Rule( object_distance['Far'] & object_direction['Right'] & object_risk_input['VeryLow'], (left_speed['Fast'],right_speed['Fast']))   
    rule030= ctrl.Rule( object_distance['Far'] & object_direction['Right'] & object_risk_input['Medium'], (left_speed['Fast'],right_speed['Fast'])) 
    rule031= ctrl.Rule( object_distance['Far'] & object_direction['Right'] & object_risk_input['VeryHigh'], (left_speed['Fast'],right_speed['Fast'])) 

    rule_list = [rule001, rule002, rule003,rule004, rule005, rule006, rule007, rule008, rule009, rule010,rule011, rule012, rule013,rule014, rule015, rule016,rule017, rule018, rule019, rule020,rule021, rule022, rule023,rule024, rule025, rule026, rule027, rule028, rule029, rule030,rule031]
    return rule_list
