#!/usr/bin/env python

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import random,time

def init():

    quality = ctrl.Antecedent(np.arange(0, 11, 1), 'quality')
    service = ctrl.Antecedent(np.arange(0, 11, 1), 'service')
    tip1 = ctrl.Consequent(np.arange(0, 26, 1), 'tip1')
    tip2 = ctrl.Consequent(np.arange(0, 26, 1), 'tip2')
    quality.automf(3)
    service.automf(3)

    tip1['low'] = fuzz.trimf(tip1.universe, [0, 0, 13])
    tip1['medium'] = fuzz.trimf(tip1.universe, [0, 13, 25])
    tip1['high'] = fuzz.trimf(tip1.universe, [13, 25, 25])

    tip2['low'] = fuzz.trimf(tip2.universe, [0, 0, 13])
    tip2['medium'] = fuzz.trimf(tip2.universe, [0, 13, 25])
    tip2['high'] = fuzz.trimf(tip2.universe, [13, 25, 25])

    

    global rule1
    #rule1 = ctrl.Rule(quality['poor'] | service['poor'], (tip1['low'] , tip2['low']))
    rule1 = ctrl.Rule(None, (tip1['low'] , tip2['low']))
    rule2 = ctrl.Rule(service['average'], tip1['medium'])
    rule3 = ctrl.Rule(service['good'] | quality['good'], tip1['high'])
    
    #rule4 = ctrl.Rule(quality['poor'] | service['poor'], )
    rule5 = ctrl.Rule(service['average'], tip2['medium'])
    rule6 = ctrl.Rule(service['good'] | quality['good'], tip2['high'])
    
    quality.automf(7) ##If I put new membership function here, I will get an error
    
    rule1.antecedent(quality['poor']) ''' BUG HERE!'''   #rule1.antecedent.setter(quality['poor'])
    #rule3.antecedent(quality['poor'])
    #rule5.antecedent(quality['poor'])
    #rule6.antecedent(quality['poor'])
    '''
    #I have to define the rules again
    '''
    global tipping_ctrl
    tipping_ctrl = ctrl.ControlSystem([rule2, rule3, rule5, rule6])#([rule1, rule2, rule3, rule5, rule6])

def calc(quality,service):
    
    tipping_ctrl.addrule(rule1) #We can dynamic add rules.
    tipping = ctrl.ControlSystemSimulation(tipping_ctrl)

    # Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
    # Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
    tipping.input['quality'] = quality
    tipping.input['service'] = service

    # Crunch the numbers
    tipping.compute()

    print 'T1=',tipping.output['tip1'],'T2=',tipping.output['tip1']

if __name__ == '__main__':
    init()
    while True:
        quality = float(random.randint(0,11))
        service = float(random.randint(0,11))
        calc(quality,service)
        time.sleep(1)


