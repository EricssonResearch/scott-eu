#!/usr/bin/env python
from skfuzzy import control as ctrl
def rule_list_generator(object_distance, object_direction, object_risk): 

    '''The format of rules is:
    rule_NUMBER = ctrl.Rule(Antecedent1['X'] & Antecedent2['Y'] & Antecedent3['Z'], Consequent['A'])'''
    rule001= ctrl.Rule(object_distance['Near']    & object_direction['Front'] ,     object_risk['VeryHigh'])
    rule002= ctrl.Rule(object_distance['Near']   & object_direction['FrontLeft'] ,  object_risk['High'])
    rule003= ctrl.Rule(object_distance['Near']   & object_direction['Left'] ,       object_risk['Low'])
    rule004= ctrl.Rule(object_distance['Near']   & object_direction['FrontRight'] , object_risk['High'])
    rule005= ctrl.Rule(object_distance['Near']   & object_direction['Right'] ,      object_risk['Low'])
    #rule006= ctrl.Rule(object_distance['Near']   & object_direction['BigRear'] ,    object_risk['VeryLow'])
    rule007= ctrl.Rule(object_distance['Medium'] & object_direction['Front'] ,      object_risk['High'])
    rule008= ctrl.Rule(object_distance['Medium'] & object_direction['FrontLeft'] ,  object_risk['Medium'])
    rule009= ctrl.Rule(object_distance['Medium'] & object_direction['Left'] ,       object_risk['Low'])
    rule010= ctrl.Rule(object_distance['Medium'] & object_direction['FrontRight'] , object_risk['Medium'])
    rule011= ctrl.Rule(object_distance['Medium'] & object_direction['Right'] ,      object_risk['Low'])
    #rule012= ctrl.Rule(object_distance['Medium'] & object_direction['BigRear'] ,    object_risk['VeryLow'])
    rule013= ctrl.Rule(object_distance['Far']    & object_direction['Front'] ,      object_risk['Medium'])
    rule014= ctrl.Rule(object_distance['Far']    & object_direction['FrontLeft'] ,  object_risk['Low'])
    rule015= ctrl.Rule(object_distance['Far']    & object_direction['Left'] ,       object_risk['VeryLow'])
    rule016= ctrl.Rule(object_distance['Far']    & object_direction['FrontRight'] , object_risk['Low'])
    rule017= ctrl.Rule(object_distance['Far']    & object_direction['Right'] ,      object_risk['VeryLow'])
    #rule018= ctrl.Rule(object_distance['Far']    & object_direction['BigRear'] ,    object_risk['VeryLow'])
    
    rule_list=[rule001,rule002,rule003,rule004,rule005,rule007,rule008,rule009,rule010,rule011,rule013,rule014,rule015,rule016,rule017] #rule006, rule012, rule018
    return rule_list

