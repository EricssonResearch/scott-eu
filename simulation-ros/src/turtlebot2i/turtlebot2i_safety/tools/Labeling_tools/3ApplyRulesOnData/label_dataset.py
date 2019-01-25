#!/usr/bin/env python
'''-------------------------------------------
--We will not label the data set in this way--
-------------------------------------------'''
import os
import re
import csv
''' #Disable images
import matplotlib.pyplot as plt # plt to show img
#%matplotlib inline 
plt.ion() # Don't forget this
plt.axis('off')
import matplotlib.image as mpimg # mpimg to read img
'''
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import time

def init_Var(): 
    print("Init Var")
    global IZW # Safety zone size
    IZW = 0.4  # Static robot.

    global sample_number
    sample_number = 0 # counter

def init_Path(): #open 'labels' folder and creat a result file
    global labels_folder
    #All files will be saved to /home/usr/labels
    labels_folder = os.path.join(os.path.expanduser('~'),'labels') 
    global result_file

    if not os.path.exists(labels_folder):
        raise Exception("Label folder doesn't exist! Make sure you already have unsupervised data.")
    else:
        print "Label folder exists"
        
        result_file = open(labels_folder+"/supervised_data.csv",'wb')
        global myWriter
        myWriter = csv.writer(result_file)
        myWriter.writerow(["Obj Type","Obj Distance","Obj Orientation","Obj Direction","Obj Speed","Obj Risk"])# Remove "Obj Name"
        #result_file.close() # Don't forget to close it in the finishing part
        print "Result csv file created!"

def init_RegEx():
    global data_file_pattern,img_file_pattern
    data_file_pattern="(\w+\_?\w+#?\d*)"+"\.csv"
    img_file_pattern ="(Vrep_shot\d*)"+"\.png" #No need to open the image

def init_rule_based_system():
    # Antecedent/Consequent objects hold universe variables and membership functions

    step_meter = 0.02 # If the step are large, the Gaussian MF will regress to Triangular MF
    step_meter_per_second = 0.02
    step_risk = 0.05
    range_type = np.arange(0, 2+1, 1)
    range_degree = np.arange(-180, 180+1, 1.0)    # Range: -180 degree ~ 180 degree for direction and orientation
    range_meter  = np.arange(0, 3.0+step_meter, step_meter)         # Range:  0 meter ~ 3 meter for distance
    range_meter_per_second = np.arange(0, 2.0+step_meter_per_second, step_meter_per_second)#Range:  0 mps ~ 2 mps for speed
    range_risk = np.arange(0, 5+step_risk, step_risk)  # Range: 0,1,2,3,4 for risk

    object_type = ctrl.Antecedent(range_type, 'type') 
    object_distance = ctrl.Antecedent(range_meter, 'distance') 
    object_orientation = ctrl.Antecedent(range_degree , 'orientation')#-180~180 degree
    object_direction  = ctrl.Antecedent(range_degree , 'direction') # -180~180 degree
    object_speed = ctrl.Antecedent(range_meter_per_second , 'speed')		#0- 3 m/s

    object_risk = ctrl.Consequent(range_risk, 'risk')

    # Custom membership functions can be built interactively with a familiar Pythonic API
    # Type
    object_type['StaObj'] = fuzz.trimf(range_type, [0, 0, 0.1])
    object_type['DynObj'] = fuzz.trimf(range_type, [0.9, 1, 1.1])
    object_type['Human'] = fuzz.trimf(range_type, [1.9, 2, 2])

    # Distance
    object_distance['Near']  = fuzz.trapmf(range_meter, [0, 0, IZW, 2*IZW])
    object_distance['Medium']= fuzz.trimf(range_meter, [IZW, 2*IZW, 4*IZW])
    object_distance['Far']   = fuzz.trapmf(range_meter, [2*IZW, 4*IZW, 3, 3])
    #object_distance.view()

    # Direction -180~180
    rear_d_p2 = fuzz.trapmf(range_degree, [-180, -180, -135, -90])
    object_direction['Right']  = fuzz.trimf(range_degree, [-135, -90, -45])
    object_direction['FrontRight']  = fuzz.trimf(range_degree, [-90, -45, 0])
    object_direction['Front']  =  fuzz.trimf(range_degree, [-45, 0, 45])
    object_direction['FrontLeft']= fuzz.trimf(range_degree, [0, 45, 90])
    object_direction['Left']= fuzz.trimf(range_degree, [45, 90, 135])
    rear_d_p1 = fuzz.trapmf(range_degree, [90, 135, 180,180]) 
    null,object_direction['BigRear']  =fuzz.fuzzy_or(range_degree,rear_d_p1,range_degree,rear_d_p2)
    print("init_fls_common_part")
    #object_direction.view()

    # Speed 
    object_speed['Slow']  = fuzz.trapmf(range_meter_per_second, [0, 0, 0.5, 1.0])
    object_speed['Medium']= fuzz.trapmf(range_meter_per_second, [0.5, 1.0, 1.0, 1.5])
    object_speed['Fast']  = fuzz.trimf(range_meter_per_second,[1.0,1.5,1.5])
    #object_speed.view()

    # Orientation
    object_orientation['Front']  = fuzz.trimf(range_degree, [-45, 0, 45])
    object_orientation['FrontLeft']=fuzz.trimf(range_degree, [0, 45, 90])
    object_orientation['Left']=  fuzz.trimf(range_degree, [45, 90, 135]) 
    object_orientation['RearLeft']= fuzz.trimf(range_degree, [90, 135, 180]) 
    rear_p1 = fuzz.trimf(range_degree, [135, 180,180]) 
    rear_p2 = fuzz.trimf(range_degree, [-180,-180,-135]) 
    null,object_orientation['Rear']  =fuzz.fuzzy_or(range_degree,rear_p1,range_degree,rear_p2)
    object_orientation['RearRight']  = fuzz.trimf(range_degree, [-180,-135,-90]) 
    object_orientation['Right']  = fuzz.trimf(range_degree, [-135,-90,-45]) 
    object_orientation['FrontRight']  = fuzz.trimf(range_degree, [-90,-45, 0]) 
    #object_orientation.view()

    # Risk
    object_risk['VeryLow'] = fuzz.trimf(range_risk, [0, 0, 1]) 
    object_risk['Low'] =  fuzz.trimf(range_risk, [0, 1, 2]) 
    object_risk['Medium'] =  fuzz.trimf(range_risk, [1, 2, 3]) 
    object_risk['High'] =  fuzz.trimf(range_risk, [2, 3, 4]) 
    object_risk['VeryHigh'] =  fuzz.trimf(range_risk, [3, 4, 4]) 
    """
    Fuzzy rules
    -----------
    """

    #time_previous = time.time()  

    #from rules_demo import rule_list_generator
    #from rules import rule_list_generator
    #rule_list=rule_list_generator(object_type,object_distance,object_direction, object_speed, object_orientation, object_risk)

    #run_time = time.time() - time_previous    
    #print 'execute time=',one_run_time,'s'           
    #print 'setting rules time=',run_time,'sec'  
    """
    Control System Creation and Simulation
    ---------------------------------------
    """
    global ra_fls

    import cPickle as pickle 
    fls_name = "ra_full.data"

    if os.path.exists(fls_name):
        print("FLS exists!")
        f = open(fls_name,'rb')
        ra_fls = pickle.load(f)
    else:
        print("Init FLS")
        from assessment_rules import rule_list_generator
        assessment_rule_list=rule_list_generator(object_type,object_distance,object_direction, object_speed, object_orientation, object_risk)
        ra_fls = ctrl.ControlSystem(assessment_rule_list)
        f = open(fls_name,'wb')
        pickle.dump(ra_fls,f)
        f.close 
    """
    In order to simulate this control system, we will create a
    ``ControlSystemSimulation``.  Think of this object representing our controller
    applied to a specific set of cirucmstances. 
    """
    global risk_assessment_instance 
    risk_assessment_instance = ctrl.ControlSystemSimulation(ra_fls)
  
def add_label_with_complex_rules(object_type,object_distance,object_orientation,object_direction,object_speed): #Add risk label
    global risk_assessment_instance
    time_previous = time.time()
    
    """
    We can now simulate our control system by simply specifying the inputs
    and calling the ``compute`` method.  
    """
    # Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
    risk_assessment_instance.input['type'] = object_type
    risk_assessment_instance.input['distance'] = object_distance
    risk_assessment_instance.input['direction'] = object_direction
    if object_speed>1.5:
        object_speed = 1.5
    risk_assessment_instance.input['speed'] =   object_speed
    risk_assessment_instance.input['orientation'] = object_orientation

    # Crunch the numbers
    risk_assessment_instance.compute()

    run_time = time.time() - time_previous    
    #object_risk.view(sim=risk_assessment_instance)    #visualize the result
    print 'calculate a instance=',run_time,'sec'  

    """
    Once computed, we can view the result as well as visualize it.
    """
    return risk_assessment_instance.output['risk']


def read_csv(file_path):
    with open(file_path,'rb') as myFile:
        lines=csv.reader(myFile) #We want second line
        lines = list(lines) # Convert '_csv.reader' type to 'list' type
        str_data = lines[1] # Type: list
        #print type(str_data[0]) # <type 'str'>
        name = str_data[0]
        data = str_data[1:6]
        #hashvalue= str_data[6]
        #robot_speed = str_data[7]    
        data = [float(i) for i in data]
        return name,data[0],data[1],data[2],data[3],data[4] #label: data[5] is invalid

def add_label_result_to_file(object_name,object_type,object_distance,object_orientation,object_direction,object_speed,risk_label):
    #TODO
    object_type = int(object_type)
    myWriter.writerow([object_type,object_distance,object_orientation,object_direction,object_speed,risk_label])  #remove "Obj Name"  
    print("One new line is added")

def finish():
    result_file.close()
    print "Labeling finished!"

def label_dataset():
    folder_list = os.listdir(labels_folder)
    print "We have",len(folder_list)-1,"folders"
    print "===================================================================="
    for one_folder in folder_list: #NOTE: Not in order 0-9
        #print folder_list
        folder_path = os.path.join(labels_folder, one_folder) 
        print "One folder:",folder_path # Path
        if one_folder == "supervised_data.csv":
            folder_list.remove(one_folder)
            #print folder_list
        else:            
            # Read files
            files = os.listdir(folder_path)
            #NO need to open image
            for one_file in files:
                matchObj_img = re.match(img_file_pattern, one_file,re.M|re.I) #It Works 
                if matchObj_img:
                    print "The picture ",one_file," is here!"
                    '''
                    file_path = os.path.join(folder_path, one_file) 

                    scene_img = mpimg.imread(file_path) 
                    plt.imshow(scene_img)
                    #plt.show()   # Since we have plt.ion, this is useless.         
                    #raw_input()
                    '''
                    files.remove(one_file)
            
            for one_file in files:                    
                matchObj_csv = re.match(data_file_pattern, one_file,re.M|re.I) 
                print "--------------------------------------------------------------------"   
                print "--------------------------------------------------------------------"    
                #print one_file
                if matchObj_csv:
                    file_path = os.path.join(folder_path, one_file)        
        
                    print "We have a data file",one_file,". Now label it! "

                    object_name,object_type,object_distance,object_orientation,object_direction,object_speed=read_csv(file_path) 
                    try:
                        risk_label = add_label_with_complex_rules(object_type,object_distance,object_orientation,object_direction,object_speed)
                    except ValueError:
                        print "Value error with:",object_type,object_distance,object_orientation,object_direction,object_speed
                    else:
                        #print "Risk = ",risk_label
                        add_label_result_to_file(object_name,object_type,object_distance,object_orientation,object_direction,object_speed,risk_label)
                        
                        global sample_number
                        sample_number =  sample_number+1

                else:
                    #raise Exception("No match! What is this file?", one_file)
                    print "No match! What is this file?", one_file 
        print "--------------------------------------------------------------------"
    print "We have ",sample_number,"samples"

""" Main program """
if __name__ == "__main__": 
    init_Var()
    init_Path()
    init_RegEx()
    init_rule_based_system()
    label_dataset()
    finish()

