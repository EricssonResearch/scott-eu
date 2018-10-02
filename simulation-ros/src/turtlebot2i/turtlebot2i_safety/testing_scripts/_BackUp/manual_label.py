#!/usr/bin/env python
'''-------------------------------------------
--We will not label the data set in this way--
-------------------------------------------'''
import os
import re
import csv
import matplotlib.pyplot as plt # plt to show img
#%matplotlib inline 
plt.ion() # Don't forget this
plt.axis('off')
import matplotlib.image as mpimg # mpimg to read img
import time
def init(): #open 'labels' folder and creat a result file
    print("Init Var")
    global labels_folder
    #All files will be saved to /home/usr/labels
    labels_folder = os.path.join(os.path.expanduser('~'),'labels') 
    global result_file

    if not os.path.exists(labels_folder):
        #os.makedirs(labels_folder)
        raise Exception("Label folder doesn't exist! Make sure you already have unsupervised data.")
    else:
        print "Label folder exists"
        
        result_file = open(labels_folder+"/result.csv",'wb')
        global myWriter
        myWriter = csv.writer(result_file)
        #myWriter.writerow(["Obj Type","Obj Distance","Obj Orientation","Obj Direction","Obj Speed","Obj Risk"])# Remove "Obj Name"
        #result_file.close() # Don't forget to close it 
        print "Result csv file created!"
    global sample_number
    sample_number = 0 # counter
    global data_file_pattern,img_file_pattern
    data_file_pattern="(\w+\_?\w+#?\d*)"+"\.csv"
    img_file_pattern ="(Vrep_shot\d*)"+"\.png"
    
def keyboard_label(label): #Read keyboard input and add risk label
    # TODO
    if label == "a":
        print "Risk label is LOW"
        return "1"
    if label == "w":
        print "Risk label is Medium"
        return "2"
    if label == "d":
        print "Risk label is High"
        return "3"

def read_csv(file_path):
    with open(file_path,'rb') as myFile:
        lines=csv.reader(myFile) #We want second line
        lines = list(lines) # Convert '_csv.reader' type to 'list' type
        data = lines[1] # Type: list        
        return data[0],data[1],data[2],data[3],data[4],data[5] #label:data[6] is invalid

def add_label_result_to_file(object_name,object_type,object_distance,object_orientation,object_direction,object_speed,risk_label):
    #TODO
    myWriter.writerow([object_type,object_distance,object_orientation,object_direction,object_speed,risk_label])  #remove "Obj Name"  
    print("Add one line")

def finish():
    result_file.close()

def label_dataset():
    folder_list = os.listdir(labels_folder)
    print "We have",len(folder_list)-1,"folders"
    print "===================================================================="
    for one_folder in folder_list: #NOTE: Not in order 0-9
        #print folder_list
        folder_path = os.path.join(labels_folder, one_folder) 
        print "One folder:",folder_path # Path
        if one_folder == "result.csv":
            folder_list.remove(one_folder)
            #print folder_list
        else:            
            # Read files
            files = os.listdir(folder_path)
            ''' Order here is a mess
            for one_file in files:
                file_path = os.path.join(folder_path, one_file) 
                #print file_path

                matchObj_img = re.match(img_file_pattern, one_file,re.M|re.I) #It Works 
                if matchObj_img:
                    print "We have a picture",file_path
                    #scene_img = mpimg.imread(file_path) 
                    #plt.imshow(scene_img)
                else:
                    matchObj_csv = re.match(data_file_pattern, one_file,re.M|re.I)   
                    if matchObj_csv:
                        print "We have a data file",file_path 
                        global sample_number
                        sample_number =  sample_number+1
                        #print (raw_input("Give me a label"))
                    else:
                        print "What is this file?", file_path       
            '''
            
            for one_file in files:
                matchObj_img = re.match(img_file_pattern, one_file,re.M|re.I) #It Works 
                if matchObj_img:
                    file_path = os.path.join(folder_path, one_file) 
                    print "The picture ",one_file," is showing here!"
                    scene_img = mpimg.imread(file_path) 
                    plt.imshow(scene_img)
                    #plt.show()   # Since we have plt.ion, this is useless.         
                    #raw_input()
                    files.remove(one_file)

            for one_file in files:                    
                matchObj_csv = re.match(data_file_pattern, one_file,re.M|re.I)   
                if matchObj_csv:
                    file_path = os.path.join(folder_path, one_file)                 
                    print "We have a data file",one_file,". Please label it! "
                    print "--------------------------------------------------------------------"
                    print "-------------------",one_file,"------------------------"
                    print "--------------------------------------------------------------------"                        
                    object_name,object_type,object_distance,object_orientation,object_direction,object_speed=read_csv(file_path) 
                    valid_label = 0
                    while not valid_label:
                        #print "We have a data file",one_file,". Please label it! "  
                        one_label = raw_input("Give me a label:")
                        #print "Your label is ", one_label
                        if (one_label=='a') or  (one_label=='w') or  (one_label=='d'):
                            valid_label = 1                        
                            risk_label = keyboard_label(one_label)
                            add_label_result_to_file(object_name,object_type,object_distance,object_orientation,object_direction,object_speed,risk_label)
                        else:
                            print "Invalid label! Please label it again."
                    global sample_number
                    sample_number =  sample_number+1


                else:
                    print "What is this file?", file_path 
        print "--------------------------------------------------------------------"
    print "We have ",sample_number,"samples"

""" Main program """
if __name__ == "__main__": 
    init()
    label_dataset()
    finish()

