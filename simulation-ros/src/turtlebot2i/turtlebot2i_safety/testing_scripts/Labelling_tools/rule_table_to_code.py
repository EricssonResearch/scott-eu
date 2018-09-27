#!/usr/bin/env python
import os
import re
import csv
import time
def init(): #open 'labels' folder and creat a result file
    print("Init Var")
    global rules_folder
    #All files will be saved to /home/usr/labels
    rules_folder = os.path.join(os.path.expanduser('~'),'labels','rules') 

    if not os.path.exists(rules_folder):
        raise Exception("Rules folder doesn't exist! Make sure you already have rules.")
    else:
        print "Rule folder exists"
        
        global rule_code_segm        
        rule_code_segm = open(rules_folder+"/rules.py",'wb')
        rule_code_segm.write("'''The format of rules is:\n")
        rule_code_segm.write("rule_NUMBER = ctrl.Rule(Antecedent1['X'] & Antecedent2['Y'] & Antecedent3['Z'], Consequent['A'])'''\n")
        print "Result csv file created!"
    global rule_number
    rule_number = 0 # counter

def finish():
    rule_code_segm.close()    

def read_dynamic_rules(file_path):
    with open(file_path,'rb') as myFile:
        lines=csv.reader(myFile) #We want second line
        lines = list(lines) # Convert '_csv.reader' type to 'list' type
        data = lines[1] # Type: list        
        return data[0],data[1],data[2],data[3],data[4],data[5] #label:data[6] is invalid



def write_static_rule():
    print("write_static_rule")
    file_path = os.path.join(rules_folder,'static_obj_rules.csv') 
    with open(file_path,'rb') as ruleFile:
        rows=csv.reader(ruleFile) #We want second line
        rows = list(rows) # Convert '_csv.reader' type to 'list' type
        for data in rows: # data Type: list # 4 Antecedent, 1 Consequent            
            line = "rule"+data[0].zfill(3)+"= ctrl.Rule(object_type['"+data[1]+"'] & object_distance['"+data[2]+"'] & object_direction['"+data[3]+"'] , risk['"+data[4]+"'])\n"
            rule_code_segm.write(line)

def write_dynamic_rule():
    print("write_dynamic_rule")
    file_path = os.path.join(rules_folder,'dynamic_obj_rules.csv') 
    with open(file_path,'rb') as ruleFile:
        rows=csv.reader(ruleFile) #We want second line
        rows = list(rows) # Convert '_csv.reader' type to 'list' type
        for data in rows: # data Type: list # 6 Antecedent, 1 Consequent            
            line = "rule"+data[0].zfill(3)+"= ctrl.Rule(object_type['"+data[1]+"'] & object_distance['"+data[2]+"'] & object_direction['"+data[3]+"'] & object_speed['"+data[4]+"'] & object_orientation['"+data[5]+"'] , risk['"+data[6]+"'])\n"
            rule_code_segm.write(line)

def write_human_rule():
    print("write_human_rule")
    file_path = os.path.join(rules_folder,'human_obj_rules.csv') 
    with open(file_path,'rb') as ruleFile:
        rows=csv.reader(ruleFile) #We want second line
        rows = list(rows) # Convert '_csv.reader' type to 'list' type
        for data in rows: # data Type: list # 6 Antecedent, 1 Consequent            
            line = "rule"+data[0].zfill(3)+"= ctrl.Rule(object_type['"+data[1]+"'] & object_distance['"+data[2]+"'] & object_direction['"+data[3]+"'] & object_speed['"+data[4]+"'] & object_orientation['"+data[5]+"'] , risk['"+data[6]+"'])\n"
            rule_code_segm.write(line)


""" Main program """
if __name__ == "__main__": 
    init()
    write_static_rule()
    write_dynamic_rule()
    write_human_rule()    
    finish()

