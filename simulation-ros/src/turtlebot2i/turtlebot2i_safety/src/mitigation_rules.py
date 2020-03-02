#!/usr/bin/env python
from skfuzzy import control as ctrl
'''
    Here are all mitigation rules.
'''

def rule_list_generator(object_distance,object_direction,object_risk_input,left_speed,right_speed):
    # Mitigation Rule Format
    #rule00X= ctrl.Rule(object_distance['Near']&object_direction[]&object_risk_input , (left_speed['???'],right_speed['???']))
    rule001= ctrl.Rule(object_distance['Near'] & object_direction['Front'], (left_speed['Stop'],right_speed['Stop']))
    
    rule002= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['VeryLow'],  (left_speed['Medium'],right_speed['Medium']))
    rule003= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['Low'],      (left_speed['Medium'],right_speed['Medium']))
    rule004= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['Medium'],   (left_speed['Medium'],right_speed['Medium']))
    rule005= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['High'],     (left_speed['Slow'],right_speed['Slow']))
    rule006= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Slow']))

    rule007= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['VeryLow'],  (left_speed['Fast'],right_speed['Fast']))
    rule008= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['Low'],      (left_speed['Fast'],right_speed['Fast']))
    rule009= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['Medium'],   (left_speed['Fast'],right_speed['Medium']))
    rule010= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['High'],     (left_speed['Medium'],right_speed['Slow']))
    rule011= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['VeryHigh'], (left_speed['Medium'],right_speed['Slow']))

    rule012= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['VeryLow'],  (left_speed['Fast'],right_speed['Fast']))
    rule013= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['Low'],      (left_speed['Fast'],right_speed['Fast']))
    rule014= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['Medium'],   (left_speed['Medium'],right_speed['Medium']))
    rule015= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['High'],     (left_speed['Slow'],right_speed['Slow']))
    rule016= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Slow']))

    rule017= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['VeryLow'],  (left_speed['Fast'],right_speed['Fast']))
    rule018= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['Low'],      (left_speed['Fast'],right_speed['Fast']))
    rule019= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['Medium'],   (left_speed['Medium'],right_speed['Fast']))
    rule020= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['High'],     (left_speed['Slow'],right_speed['Medium']))
    rule021= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Medium']))

    rule022= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['VeryLow'],  (left_speed['Fast'],right_speed['Fast']))
    rule023= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['Low'],      (left_speed['Fast'],right_speed['Fast']))
    rule024= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['Medium'],   (left_speed['Medium'],right_speed['Medium']))
    rule025= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['High'],     (left_speed['Slow'],right_speed['Slow']))
    rule026= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Slow']))

    rule027= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['VeryLow'],  (left_speed['Fast'],right_speed['Fast']))
    rule028= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['Low'],      (left_speed['Fast'],right_speed['Fast']))
    rule029= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['Medium'],   (left_speed['Fast'],right_speed['Fast']))
    rule030= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['High'],     (left_speed['Medium'],right_speed['Medium']))
    rule031= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['VeryHigh'], (left_speed['Medium'],right_speed['Medium']))

    rule032= ctrl.Rule( object_distance['Far'] & object_direction['FrontLeft'], (left_speed['Fast'],right_speed['Fast']))
    rule033= ctrl.Rule( object_distance['Far'] & object_direction['Left'] ,     (left_speed['Fast'],right_speed['Fast']))
    rule034= ctrl.Rule( object_distance['Far'] & object_direction['FrontRight'],(left_speed['Fast'],right_speed['Fast']))
    rule035= ctrl.Rule( object_distance['Far'] & object_direction['Right'],     (left_speed['Fast'],right_speed['Fast']))

    rule036= ctrl.Rule(object_distance['Near'] & object_direction['FrontLeft'],  (left_speed['Stop'],right_speed['Stop']))
    rule037= ctrl.Rule(object_distance['Near'] & object_direction['FrontRight'], (left_speed['Stop'],right_speed['Stop']))
    rule038= ctrl.Rule(object_distance['Near'] & object_direction['Left'],       (left_speed['Slow'],right_speed['Stop']))
    rule039= ctrl.Rule(object_distance['Near'] & object_direction['Right'],      (left_speed['Stop'],right_speed['Slow']))
    
    #rule040= ctrl.Rule(object_direction['BigRear'],  (left_speed['Fast'], right_speed['Fast']))
    

    rule_list = [rule001, rule002, rule003,rule004, rule005, rule006, rule007, rule008, rule009, rule010,rule011, rule012, rule013,rule014, rule015, rule016,rule017, rule018, rule019, rule020,rule021, rule022, rule023,rule024, rule025, rule026, rule027, rule028, rule029, rule030,rule031, rule032, rule033,rule034, rule035, rule036, rule037, rule038, rule039 ] #rule040
    return rule_list
