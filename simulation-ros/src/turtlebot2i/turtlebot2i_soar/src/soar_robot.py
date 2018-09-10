"""Making sure we are running the right version of python"""
import sys
if sys.version_info[0] >= 3:
    raise "Must be using Python 2"

"""Making sure soar library path environment is set
Remember to set the environment variable to point to where soar build is located, e.g.:
export LD_LIBRARY_PATH=~/Desktop/Soar/out
"""
from os import environ as env, fsync
import sys
if "DYLD_LIBRARY_PATH" in env:
	LIB_PATH = env["DYLD_LIBRARY_PATH"]
elif "LD_LIBRARY_PATH" in env:
	LIB_PATH = env["LD_LIBRARY_PATH"]
else:
	print("Soar LIBRARY_PATH environment variable not set; quitting")
	exit(1)
sys.path.append(LIB_PATH)

import Python_sml_ClientInterface as sml # Python interface to SOAR
import rospy # ROS library
import json
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import Twist

""" Callback functions to help us see what is happening inside agent's mind"""
def register_print_callback(kernel, agent, function, user_data=None):
	agent.RegisterForPrintEvent(sml.smlEVENT_PRINT, function, user_data)
def callback_print_message(mid, user_data, agent, message):
	print(message.strip())

""" Client to interact with agent's mind"""
def cli(agent):
	cmd = raw_input("soar> ")
	while cmd not in ("exit", "quit"):
		if cmd:
			print(agent.ExecuteCommandLine(cmd).strip())
		cmd = raw_input("soar> ")

from random import *

#class ToyEnv(object):
#    """
#    A very simple 'environment': sensors return two random numbers and expects a single number as actuation.
#    """
#    def __init__(self):
#        """Return a new toy env object."""
#    def get_sensors(self):
#        """"""
#        a=randint(1, 10)
#        b=randint(1, 10)
#        sensors=[a,b]
#        #print("---> Environment sensed: ",sensors)
#        return sensors
#
#    def set_actuators(self, act):
#        """"""
#        print("---> Environment acted:",act)


## ROS variables
pub=None #pub should be visible by main and callbacks
sub=None #sub should be visible by main and callbacks

vel_output_pub=None
vel_msg=None

## SOAR variables    #TODO is there a better way of doing this?
kernel=None
agent=None
te=None
input_link=None
a_value=None
b_value=None
output_link=None

tasks_wme=None
tasks_data_json=json.dumps({})
previous_tasks_data_json=json.dumps({})

seq_wme=None
secs_wme=None
nsecs_wme=None

frame_id_wme=None
child_frame_id_wme=None

xp_wme=None
yp_wme=None
zp_wme=None
    
xo_wme=None
yo_wme=None
zo_wme=None
wo_wme=None

xtl_wme=None
ytl_wme=None
ztl_wme=None

xta_wme=None
yta_wme=None
zta_wme=None

loop_counter=0
def topic_callback(odom_data, scan_data,tasks_data):
    print("________________________")
    print("Enter ROS topic_callback")
    input_link=agent.GetInputLink()

 #   rospy.loginfo('odom_stamp: ' + str(odom_data.header.stamp.to_sec()) + ' ; scan_stamp: ' + str(scan_data.header.stamp.to_sec())+ ' ; tasks_data: ' + str(tasks_data))

    

 
    

#    for i in range(len(tasks_data_json['plan'])):
#        print('=====<<<<<< tasks_data_json: ', tasks_data_json['plan'][i])  
#        tasks_data_json['plan'][i]


#        tasks_wme_list.append(tasks_data_json['plan'][i])

#    print("tasks_wme_list: ",tasks_wme_list)


   # if(tasks_wme!=None):
   #     tasks_wme=agent.DestroyWME(tasks_wme)
   # 
   # tasks_wme=agent.CreateIdWME(input_link,"tasks")

#    tasks_wme = agent.CreateIdWME(input_link,"tasks")
#    task_wme=agent.CreateIdWME(tasks_wme,"next_task")
#    task_wme_name=agent.CreateStringWME(task_wme,"name","move") 
#    task_wme_waypoint=agent.CreateStringWME(task_wme,"waypoint","Waypoint_SH#1") 


    


    #Do something with data
    #dict=eval(data.data)

    ## Cognitive cycle goes here

  #  a_value_temp=float(dict['a_value'])
   # b_value_temp=float(dict['b_value'])

    ## 2) push senses to soar
   # a_value.Update(a_value_temp)
  #  b_value.Update(b_value_temp)

    #Odometer
    xp_wme.Update(float(odom_data.pose.pose.position.x))
    yp_wme.Update(float(odom_data.pose.pose.position.y))
    zp_wme.Update(float(odom_data.pose.pose.position.z))
        
    xo_wme.Update(float(odom_data.pose.pose.orientation.x))
    yo_wme.Update(float(odom_data.pose.pose.orientation.y))
    zo_wme.Update(float(odom_data.pose.pose.orientation.z))
    wo_wme.Update(float(odom_data.pose.pose.orientation.w))

    xtl_wme.Update(float(odom_data.twist.twist.linear.x))
    ytl_wme.Update(float(odom_data.twist.twist.linear.y))
    ztl_wme.Update(float(odom_data.twist.twist.linear.z))

    xta_wme.Update(float(odom_data.twist.twist.angular.x))
    yta_wme.Update(float(odom_data.twist.twist.angular.y))
    zta_wme.Update(float(odom_data.twist.twist.angular.z))

    #Tasks List
    global tasks_wme,tasks_data_json,previous_tasks_data_json

    tasks_data_json = json.loads(str(tasks_data.data))
    if(previous_tasks_data_json!=tasks_data_json): #Checking if list of tasks has changed before sending it to soar
        print("##########################")
        if(tasks_wme!=None):
            #print("DestroyWME: ",tasks_wme)
            tasks_wme=agent.DestroyWME(tasks_wme)

        tasks_wme = agent.CreateIdWME(input_link,"tasks")

        tasks_wme_list=[]
        previous_task_wme=tasks_wme

        for i in range(len(tasks_data_json['plan'])):
            task_wme=agent.CreateIdWME(previous_task_wme,"next_task")
            action_name=str(tasks_data_json['plan'][i][0])
            agent.CreateStringWME(task_wme,"name",action_name) 
            if(action_name=='move'):
                agent.CreateStringWME(task_wme,"waypoint",str(tasks_data_json['plan'][i][2])) 
            elif(action_name=='pick'):
                agent.CreateStringWME(task_wme,"shelf",str(tasks_data_json['plan'][i][2])) 
                agent.CreateStringWME(task_wme,"product",str(tasks_data_json['plan'][i][3]))
            elif(action_name=='drop'):
                agent.CreateStringWME(task_wme,"conveyor_belt",str(tasks_data_json['plan'][i][2])) 
                agent.CreateStringWME(task_wme,"product",str(tasks_data_json['plan'][i][3])) 
            else:
                print('unknown action: ',action_name)
            previous_task_wme=task_wme

        previous_tasks_data_json = tasks_data_json


    ## 3) make soar think about it
    result=0
    #run_result=agent.RunSelf(1)    #Run agent for one step (should run until output?)
    run_result=agent.RunSelfTilOutput() #TODO see why so many substates are being created
    
    ## 4) get results from soar
    output_link=agent.GetOutputLink()## returns an Identifier
    if output_link!= None:
        result_output_wme = output_link.FindByAttribute("result", 0) # returns a WMElement of the form (<output_link> ^result <val>)
        result=None
        if result_output_wme != None:
            result = float(result_output_wme.GetValueAsString())
            print("Result: ",result)

    ## 5) send result to environment
   # te.set_actuators(result) 
    #rospy.loginfo("output result log: "+str(result))
  #  pub.publish("output result topic: "+str(result)) # Here goes the output

    vel_msg=Twist()
    vel_msg.linear.x=0.0
    vel_msg.linear.y=0.0
    vel_msg.linear.z=0.0
    vel_msg.angular.x=0.0
    vel_msg.angular.y=0.0
    vel_msg.angular.z=0.0
    vel_output_pub.publish(vel_msg)
    #print("vel_msg: ",vel_msg)
    ##TODO Improve soar_robot.soar based on Move-north operator

    
    global loop_counter
    print("loop_counter: ",loop_counter)
    loop_counter=loop_counter+1
    if(loop_counter>3):
        cli(agent) #open client to interact with agent 

""" =============================================================== """
""" === Main program                                            === """
""" =============================================================== """
if __name__ == "__main__":   



    json_string = """
    {
        "id": "xyz",
        "init": "initURI",
        "goal": "finalURI",
        "plan":[
                    ["move", "youBot#0", "Waypoint_SH#1"],
                    ["pick", "youBot#0", "Shelf#1", "productRed"],
                    ["move", "youBot#0", "Waypoint_CB"],
                    ["drop", "youBot#0", "ConveyorBelt", "productRed"]
                    ]
    }
    """
    temp_tasks_dummy_data = json.loads(json_string)
    tasks_dummy_data=str(json.dumps(temp_tasks_dummy_data)) #makes sure it is a valid json when converting to str
   


#-- SOAR AGENT INITIALIZATION-----------------------------------
    print("Initializing SOAR Agent")
    #Instantiate link to environment
    #te = ToyEnv()

    #Create soar kernel and agent
    kernel = sml.Kernel.CreateKernelInCurrentThread()
    agent = kernel.CreateAgent("agent")
    register_print_callback(kernel, agent, callback_print_message, None)

    #Load soar sources

    agent.ExecuteCommandLine("source soar_robot.soar")
    agent.ExecuteCommandLine("soar wait-snc")


    #Get input link and create input  structure
    input_link=agent.GetInputLink()
    
    a_value=agent.CreateFloatWME(input_link, "a", -1.0)
    b_value=agent.CreateFloatWME(input_link, "b", -1.0)

    #TODO How about doing this automatically based on the topic's structure?
    odom_wme=agent.CreateIdWME(input_link,"odom")

    header_wme=agent.CreateIdWME(odom_wme,"header")  
    stamp_wme=agent.CreateIdWME(header_wme,"stamp")
      
    pose_wme=agent.CreateIdWME(odom_wme,"pose")
    pose2_wme=agent.CreateIdWME(pose_wme,"pose") #Weird, but that's how ROS odom does it
    position_wme=agent.CreateIdWME(pose2_wme,"position")
    orientation_wme=agent.CreateIdWME(pose2_wme,"orientation")
    
    twist_wme=agent.CreateIdWME(odom_wme,"twist")
    twist2_wme=agent.CreateIdWME(twist_wme,"twist") #Weird, but that's how ROS odom does it
    twist_linear_wme=agent.CreateIdWME(twist2_wme,"linear")
    twist_angular_wme=agent.CreateIdWME(twist2_wme,"angular")

    #Leaves
    seq_wme=agent.CreateIntWME(header_wme,"seq",-1)
    secs_wme=agent.CreateFloatWME(stamp_wme,"secs",-1)
    nsecs_wme=agent.CreateFloatWME(stamp_wme,"nsecs",-1)

    frame_id_wme=agent.CreateStringWME(header_wme,"frame_id","empty") 

    child_frame_id_wme=agent.CreateStringWME(odom_wme,"child_frame_id","empty") 

    xp_wme=agent.CreateFloatWME(position_wme,"x",-1.0)
    yp_wme=agent.CreateFloatWME(position_wme,"y",-1.0)
    zp_wme=agent.CreateFloatWME(position_wme,"z",-1.0)
    
    xo_wme=agent.CreateFloatWME(orientation_wme,"x",-1.0)
    yo_wme=agent.CreateFloatWME(orientation_wme,"y",-1.0)
    zo_wme=agent.CreateFloatWME(orientation_wme,"z",-1.0)
    wo_wme=agent.CreateFloatWME(orientation_wme,"w",-1.0)


    xtl_wme=agent.CreateFloatWME(twist_linear_wme,"x",-1.0)
    ytl_wme=agent.CreateFloatWME(twist_linear_wme,"y",-1.0)
    ztl_wme=agent.CreateFloatWME(twist_linear_wme,"z",-1.0)

    xta_wme=agent.CreateFloatWME(twist_angular_wme,"x",-1.0)
    yta_wme=agent.CreateFloatWME(twist_angular_wme,"y",-1.0)
    zta_wme=agent.CreateFloatWME(twist_angular_wme,"z",-1.0)


    
#    tasks_wme=agent.CreateIdWME(input_link,"tasks")
#    task_wme=agent.CreateIdWME(tasks_wme,"next_task")
#    task_wme_name=agent.CreateStringWME(task_wme,"name","move") 
#    task_wme_waypoint=agent.CreateStringWME(task_wme,"waypoint","Waypoint_SH#1") 





    #Get output link
    output_link=agent.GetOutputLink()



#-- ROS SOAR NODE INITIALIZATION-----------------------------------    
    print("Initializing ROS SOAR node")
    rospy.init_node("soar_ros_node",anonymous=True) #Always first

    ## SUBSCRIBERS
    # Creates a subscriber object for each topic
    # The messages to be synced must have the 'header' field or
    #  use the 'allow_headerless=True' in the TimeSynchronizer() function
    #  if this field is not present
    #sub=message_filters.Subscriber("/turtlebot2i/soar_sub_topic", String)
    odom_sub = message_filters.Subscriber('/turtlebot2i/odom', Odometry)
    scan_sub = message_filters.Subscriber('/turtlebot2i/lidar/scan', LaserScan)
    tasks_sub = message_filters.Subscriber('/turtlebot2i/tasks', String)

    # Make the topics sync through ApproximateTimeSynchronizer with 0.1s of tolerance
    ts =  message_filters.ApproximateTimeSynchronizer([odom_sub, scan_sub,tasks_sub], 10, 0.1, allow_headerless=True)

    # Associate the synchronizer with a callback
    ts.registerCallback(topic_callback)

    ## PUBLISHERS
    #pub=rospy.Publisher("soar_pub_topic", String, queue_size=10)
    ##TODO decide wether to go with direct input to velocities or use navigation package
    vel_output_pub=rospy.Publisher("/turtlebot2i/commands/velocity",Twist, queue_size=10) #velocity output publisher
    vel_msg=Twist()

    #dummy_pub=rospy.Publisher("/turtlebot2i/soar_sub_topic",String, queue_size=10) #dummy pubs are used for inputing debug data to input topics
    dummy_tasks_pub=rospy.Publisher("/turtlebot2i/tasks", String, queue_size=10)

 

#    sub=rospy.Subscriber("soar_sub_topic", String, topic_callback)
#    pub=rospy.Publisher("soar_pub_topic",String, queue_size=10)
#
#    dummy_pub=rospy.Publisher("soar_sub_topic",String, queue_size=10) #used for inputing debug data to soar_sub_topic
#
#    tasks_list_topic=rospy.Subscriber("soar_tasks_list_topic", String, topic_callback)
#    dummy_tasks_list_topic=rospy.Publisher("soar_tasks_list_topic",String, queue_size=10) #used for inputing debug data to soar_tasks_list_topic



#-- INPUT UPDATE LOOP (for debug purposes)-----------------------------------
    rate = rospy.Rate(1)

    while not rospy.is_shutdown(): #loop that updates agent's inputs
        

 #       sense=te.get_sensors()
 #       new_input=dict()
 #       new_input['a_value']=str(sense[0])
 #       new_input['b_value']=str(sense[1])
 #       current_time=str(rospy.get_time())
 #       new_input['time']=str(current_time)

 #       dummy_pub.publish(str(new_input)) 
#        rospy.loginfo("Log new_input: "+ str(new_input))

        dummy_tasks_pub.publish(str(tasks_dummy_data))
       
    
       
        rate.sleep()

    rospy.spin()


"""
    #Instantiate link to environment
    te = ToyEnv()

    #Create soar kernel and agent
    kernel = sml.Kernel.CreateKernelInCurrentThread()
    agent = kernel.CreateAgent("agent")
    register_print_callback(kernel, agent, callback_print_message, None)

    #Load soar sources
    agent.ExecuteCommandLine("source soar_robot.soar")

    #Get input link and create input  structure
    input_link=agent.GetInputLink()
    
    a_value=agent.CreateFloatWME(input_link, "a", -1.0)
    b_value=agent.CreateFloatWME(input_link, "b", -1.0)

    #Get output link
    output_link=agent.GetOutputLink()

    ### Start Soar cognitive cycle ###
    #
    for i in range(0,3): # replace by a "while True:" to run forever
        print(" ------------- Soar cycle: ",i," ------------- ")
    # 1) sense the environment
        sense=te.get_sensors()

    # 2) push senses to soar
        a_value.Update(sense[0])
        b_value.Update(sense[1])

    # 3) make soar think about it
        result=0
        run_result=agent.RunSelf(1)    #Run agent for one step
    
    # 4) get results from soar
        output_link=agent.GetOutputLink()## returns an Identifier
        if output_link!= None:
            result_output_wme = output_link.FindByAttribute("result", 0) # returns a WMElement of the form (<output_link> ^result <val>)
            result=None
            if result_output_wme != None:
                result = float(result_output_wme.GetValueAsString())

    #5) send result to environment
        te.set_actuators(result) 

    #
    ### End Soar cognitive cycle###

    cli(agent) #open client to interact with agent

    #Close agent and kernel
    kernel.DestroyAgent(agent)
    kernel.Shutdown()
"""


"""
---
header: 
  seq: 68884
  stamp: 
    secs: 1531936453
    nsecs: 639370680
  frame_id: "turtlebot2i/odom"
child_frame_id: "turtlebot2ikinect"
pose: 
  pose: 
    position: 
      x: 2.27206683159
      y: -0.190607577562
      z: 1.34853923321
    orientation: 
      x: -0.00055553537095
      y: 0.000350441696355
      z: 0.709878265858
      w: 0.704324126244
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 0.000438690185547
      y: 0.000247657299042
      z: -0.000138282775879
    angular: 
      x: 0.00346004939638
      y: 0.00924359634519
      z: 0.000447982223704
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---

"""


"""
^Cklaus@klaus-VirtualBox:~$ rostopic pub /turtlebot2i/commands/velocity geometrysgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
publishing and latching message. Press ctrl-C to terminate

"""

"""
Planning service output is a JSON object which looks like
{
    "id": "xyz",
    "init": "initURI",
    "goal": "finalURI",
    "plan":[
                ["move", "youBot#0", "Waypoint_SH#1"],
                ["pick", "youBot#0", "Shelf#1", "productRed"],
                ["pick", "youBot#0", "Shelf#1", "productGreen"],
                ["move", "youBot#0", "Waypoint_CB"],
                ["drop", "youBot#0", "ConveyorBelt", "productRed"],
                ["move", "youBot#0", "Waypoint_CB#0"],
                ["drop", "youBot#0", "ConveyorBelt#0", "productGreen"],
                ["move", "youBot#1", "Waypoint_SH#0"],
                ["pick", "youBot#1", "Shelf#0", "productYellow"],
                ["move", "youBot#1", "Waypoint_CB#0"],
                ["drop", "youBot#1", "ConveyorBelt#0", "productYellow"],
                ["move", "youBot#1", "Waypoint_CB#1"],
                ["drop", "youBot#1", "ConveyorBelt#1", "productGreen"]
                ]
}
"""

    
