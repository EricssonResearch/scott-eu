#!/usr/bin/env python
import os
import csv

def init_var():
    print("Init Var")
    global rules_folder
    #All files will be saved to /home/usr/labels
    rules_folder = os.path.join(os.path.expanduser('~'),'labels','rules_csv_format') 
    if not os.path.exists(rules_folder):
        os.makedirs(rules_folder)
        print "Label folder doesn't exist, create one"
    else:
        print "Label folder exists"

    global risk_dict
    risk_dict = {'0':'VeryLow','1':'Low','2':'Medium','3':'High','4':'VeryHigh','Missed':"TODO"}
    
    global rule_counter
    rule_counter = 595
def static_obj(): #open 'labels' folder and creat a result file
    print("static_obj")
    global rule_counter   
     
    #Creat a CSV file for rules
    with open(rules_folder+'/'+'static_obj_rules'+'.csv','wb') as myFile:    
        myWriter=csv.writer(myFile)

        myWriter.writerow(["Rule Number","Obj Type","Obj Distance","Obj Direction","Obj Risk"])#,"Obj Speed","Obj Orientation","Obj Risk"])

        for object_distance in ['Near','Medium','Far']:
            for object_direction in ['Front','FrontLeft','Left','FrontRight','Right','BigRear']:
                #for object_speed in ['Slow','Medium','Fast']:
                    #for object_orientation in ['Front','FrontLeft','Left','RearLeft','Rear','RearRight','Right','FrontRight']:      
                rule_counter = rule_counter+1
                print("Rule Number in 18","Obj Type","Obj Distance","Obj Direction")#,"Obj Speed","Obj Orientation")
                print(object_distance,object_direction)#,object_speed,object_orientation)
                number = raw_input('Number for the risk (0-4):')
                object_risk = risk_dict[number]
                print "risk is :",object_risk
                myWriter.writerow([rule_counter,"StaObj",object_distance,object_direction,object_risk])#,object_speed,object_orientation,object_risk])
    #01-18 rules
 
def dynamic_obj(): #open 'labels' folder and creat a result file
    print("dynamic_obj")
    global rule_counter       
    #Creat a CSV file for rules
    with open(rules_folder+'/'+'dynamic_obj_rules'+'.csv','a') as myFile:    #open("test.txt", "a")  #'wb' write
        myWriter=csv.writer(myFile)

        myWriter.writerow(["Rule Number","Obj Type","Obj Distance","Obj Direction","Obj Speed","Obj Orientation","Obj Risk"])

        for object_distance in ['Near','Medium','Far']:
            for object_direction in ['Front','FrontLeft','Left','FrontRight','Right','BigRear']:
                for object_speed in ['Slow','Medium','Fast']:
                    for object_orientation in ['Front','FrontLeft','Left','RearLeft','Rear','RearRight','Right','FrontRight']:      
                        rule_counter = rule_counter+1
                        print("Rule Number in 432","Obj Type","Obj Distance","Obj Direction","Obj Speed","Obj Orientation")
                        print(object_distance,object_direction,object_speed,object_orientation)
                        number = raw_input('Number for the risk (0-4):')
                        if number.isdigit():
                            object_risk = risk_dict[number]
                        else:
                            object_risk = risk_dict["Missed"]
                            print("Invalid input. Edit it later")
                        print "risk is :",object_risk
                        myWriter.writerow([rule_counter,"DynObj",object_distance,object_direction,object_speed,object_orientation,object_risk])

    #19-451

def human_obj(): #open 'labels' folder and creat a result file
    print("human_obj")
    global rule_counter     
    #Creat a CSV file for rules
    with open(rules_folder+'/'+'human_obj_rules'+'.csv','a') as myFile:    #open("test.txt", "a")  #'wb' write
        myWriter=csv.writer(myFile)

        myWriter.writerow(["Rule Number","Obj Type","Obj Distance","Obj Direction","Obj Speed","Obj Orientation","Obj Risk"])

        for object_distance in ['Medium','Far']:#'Near','Medium','Far']:
            for object_direction in ['Front','FrontLeft','Left','BigRear','Right','FrontRight']:
                for object_speed in ['Slow','Medium','Fast']:
                    for object_orientation in ['Front','FrontLeft','Left','RearLeft','Rear','RearRight','Right','FrontRight']:      
                        rule_counter = rule_counter+1
                        print("Rule Number in 432","Obj Type","Obj Distance","Obj Direction","Obj Speed","Obj Orientation")
                        print(object_distance,object_direction,object_speed,object_orientation)
                        number = raw_input('Number for the risk (0-4):')
                        if number.isdigit():
                            object_risk = risk_dict[number]
                        else:
                            object_risk = risk_dict["Missed"]
                            print("Invalid input. Edit it later")
                        print "risk is :",object_risk
                        myWriter.writerow([rule_counter,"Human",object_distance,object_direction,object_speed,object_orientation,object_risk])


   



""" Main program """
if __name__ == "__main__": 
    init_var()
    static_obj()
    dynamic_obj()
    human_obj()
