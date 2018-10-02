#!/usr/bin/env python
from skfuzzy import control as ctrl
def rule_list_generator(object_type,object_distance,object_direction, object_speed, object_orientation, object_risk): 

    rule001= ctrl.Rule(object_type['StaObj'] & object_distance['Near'] & object_direction['Front'] , object_risk['VeryHigh'])
    rule002= ctrl.Rule(object_type['StaObj'] & object_distance['Near'] & object_direction['FrontLeft'] , object_risk['High'])
    rule003= ctrl.Rule(object_type['StaObj'] & object_distance['Near'] & object_direction['Left'] , object_risk['Low'])
    rule004= ctrl.Rule(object_type['StaObj'] & object_distance['Near'] & object_direction['FrontRight'] , object_risk['High'])
    rule005= ctrl.Rule(object_type['StaObj'] & object_distance['Near'] & object_direction['Right'] , object_risk['Low'])
    rule006= ctrl.Rule(object_type['StaObj'] & object_distance['Near'] & object_direction['BigRear'] , object_risk['VeryLow'])
    rule007= ctrl.Rule(object_type['StaObj'] & object_distance['Medium'] & object_direction['Front'] , object_risk['High'])
    rule008= ctrl.Rule(object_type['StaObj'] & object_distance['Medium'] & object_direction['FrontLeft'] , object_risk['Medium'])
    rule009= ctrl.Rule(object_type['StaObj'] & object_distance['Medium'] & object_direction['Left'] , object_risk['Low'])
    rule010= ctrl.Rule(object_type['StaObj'] & object_distance['Medium'] & object_direction['FrontRight'] , object_risk['Medium'])
    rule011= ctrl.Rule(object_type['StaObj'] & object_distance['Medium'] & object_direction['Right'] , object_risk['Low'])
    rule113= ctrl.Rule(object_type['DynObj'] & object_distance['Near'] & object_direction['FrontRight'] & object_speed['Fast'] & object_orientation['Right'] , object_risk['High'])
    rule114= ctrl.Rule(object_type['DynObj'] & object_distance['Near'] & object_direction['FrontRight'] & object_speed['Fast'] & object_orientation['FrontRight'] , object_risk['High'])
    rule_list = [rule001, rule002, rule003,rule004, rule005, rule006, rule113]
    return rule_list
