#!/usr/bin/env python
import os
import time
# rqt_plot /turtlebot2i/odom/pose/pose/position/x:y
def init_var():
    global labels_folder
    labels_folder = os.path.join(os.path.expanduser('~'),'labels')
    #sub_label_folder = os.path.join(labels_folder,'01')
    if not os.path.exists(labels_folder):
        os.makedirs(labels_folder)
        print "Label folder doesn't exist, create one"
    else:
        print "Label folder exists"
    global sub_folder_number
    sub_folder_number = 0 # counter



""" Main program """
if __name__ == "__main__": 
    # os.system('ls') # No result
    #result = os.popen("command").read()
    init_var()
    os.system('shutter --min_at_startup')
    time_previous = time.time()
    time.sleep(2)

    while True:
        sub_folder_number = sub_folder_number+1
        number_string4 = str(sub_folder_number).zfill(4)  #Display number with leading zeros
        sub_label_folder = os.path.join(labels_folder,number_string4)
        if not os.path.exists(sub_label_folder):
            os.makedirs(sub_label_folder)
            print "Folder doesn't exist, create one"
        else:
            print "Folder exists"
        
        #If one want to check execution time
        one_run_time = time.time() - time_previous    
        #print 'execute time=',one_run_time,'s'           
        print 'execute frequency=',1/float(one_run_time),'Hz'  
        file_name ="'./labels/"+number_string4+"/"+"Vrep_shot"+number_string4+".png'"  
        os.system("shutter --window=V-REP.* --output="+file_name+" --no_session")
        time_previous =time.time()
        time.sleep(1)
