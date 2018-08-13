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

from std_msgs.msg import String



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
class ToyEnv(object):
    """
    A very simple 'environment': sensors return two random numbers and expects a single number as actuation.
    """
    def __init__(self):
        """Return a new toy env object."""


    def get_sensors(self):
        """"""
        a=randint(1, 10)
        b=randint(1, 10)
        sensors=[a,b]
        print("---> Environment sensed: ",sensors)
        return sensors

    def set_actuators(self, act):
        """"""
        print("---> Environment acted:",act)


## ROS variables
pub=None #pub should be visible by main and callbacks
sub=None #sub should be visible by main and callbacks

## SOAR variables    
kernel=None
agent=None
te=None
input_link=None
a_value=None
b_value=None
output_link=None


def topic_callback(data):

    rospy.loginfo(data)
    if(False):
        #Do something with data
        dict=eval(data.data)
        ## Cognitive cycle goes here
        print("DDDDDDD: ",dict)
        a_value_temp=float(dict['a_value'])
        b_value_temp=float(dict['b_value'])

       # 2) push senses to soar
        a_value.Update(a_value_temp)
        b_value.Update(b_value_temp)

       # 3) make soar think about it
        result=0
        #run_result=agent.RunSelf(1)    #Run agent for one step (should run until output?)
        run_result=agent.RunSelfTilOutput() #TODO see why so many substates are being created
       # 4) get results from soar
        output_link=agent.GetOutputLink()## returns an Identifier
        if output_link!= None:
            result_output_wme = output_link.FindByAttribute("result", 0) # returns a WMElement of the form (<output_link> ^result <val>)
            result=None
            if result_output_wme != None:
                result = float(result_output_wme.GetValueAsString())

    #    #5) send result to environment
        te.set_actuators(result) 
        rospy.loginfo("output result log: "+str(result))
        pub.publish("output result topic: "+str(result)) # Here goes the output



""" Main program """
if __name__ == "__main__":   


#-- SOAR AGENT INITIALIZATION-----------------------------------
    print("Initializing SOAR Agent")
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

#-- ROS SOAR NODE INITIALIZATION-----------------------------------    
    print("Initializing ROS SOAR node")
    rospy.init_node("soar_ros_node",anonymous=True) #Always first

    sub=rospy.Subscriber("soar_sub_topic", String, topic_callback)
    pub=rospy.Publisher("soar_pub_topic",String, queue_size=10)

    dummy_pub=rospy.Publisher("soar_sub_topic",String, queue_size=10) #used for inputing debug data to soar_sub_topic

    tasks_list_topic=rospy.Subscriber("soar_tasks_list_topic", String, topic_callback)
    dummy_tasks_list_topic=rospy.Publisher("soar_tasks_list_topic",String, queue_size=10) #used for inputing debug data to soar_tasks_list_topic



#-- INPUT UPDATE LOOP (for debug purposes)-----------------------------------
    rate = rospy.Rate(1)

    while not rospy.is_shutdown(): #loop that updates agent's inputs
        

        sense=te.get_sensors()
        new_input=dict()
        new_input['a_value']=str(sense[0])
        new_input['b_value']=str(sense[1])
        current_time=str(rospy.get_time())
        new_input['time']=str(current_time)

        dummy_pub.publish(str(new_input)) 
#        rospy.loginfo("Log new_input: "+ str(new_input))
        new_input=dict()
        new_input['tasks_list']=str(randint(30,40))
        current_time=str(rospy.get_time())
        new_input['time']=str(current_time)
        dummy_tasks_list_topic.publish(str(new_input))

       
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
    
