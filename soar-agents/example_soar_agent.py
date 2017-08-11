from jsoar.SoarAgent import SoarAgent
from middleware.sim.SceneManager import * # Import libs for VREP interaction
import simplejson as json

### ==== Start scene ==== ###
global scene
scene = SceneManager("10.0.2.2")
#scene = SceneManager()
scene.start()


### Select robot from the scene ###
robots = scene.get_robots("VREP")
robot=robots[1]
robot_ID=robot.get_id()
print(robot_ID)



### ==== Agent Initialization ==== ###

""" Agent Instantiation """

soar_agent = SoarAgent("My Soar Agent", 25333)
soar_agent.initialize_agent("warehouse-robot.soar") 
#soar_agent.initialize_agent("simple-example.soar") 
#print("Running forever...")
#gateway.runForever()  
#n=1
#print("Running for ",n," decison(s)...")
#gateway.runFor(n)

allWmesInRete=soar_agent.getAllWmesInRete()
print("\n allWmesInRete: ",allWmesInRete)

""" Initialize input structure """

identifier="I2"
symbol="id"
value="mobilerobot_youBot"
name_key=soar_agent.InputWme(identifier,symbol, value)

battery_key=soar_agent.InputWme("I2","battery", "100")
c1_key=soar_agent.InputWme("I2","cargo", "C4") #TODO:can't use C1,2 because already in use by WM
c2_key=soar_agent.InputWme("I2","cargo", "C5")
c3_key=soar_agent.InputWme("I2","cargo", "C6")
location_key=soar_agent.InputWme("I2","location", "L1")

x_key=soar_agent.InputWme("L1","x", "1.5")
y_key=soar_agent.InputWme("L1","y", "-1.8")
z_key=soar_agent.InputWme("L1","z", "1.0")
theta_key=soar_agent.InputWme("L1","theta", "3.1415")

soar_agent.printAllWME()




### ====  Cognitive cycle ==== ###

""" PERCEPTION """



#Get list of waypoints
waypoints=scene.get_waypoints_list()
print("waypoints: ",waypoints)

#Get position of each waypoint




# Updating inputs
"""
x_key=soar_agent.UpdateWme(x_key, "3")
y_key=soar_agent.UpdateWme(y_key, "-1")
z_key=soar_agent.UpdateWme(z_key, "0.73455")
theta_key=soar_agent.UpdateWme(theta_key, "1.456")

soar_agent.printAllWME()
"""



""" DELIBERATION AND ACTION SELECTION """

count=0
for x in range(0,8):
    #Get robot's current (x,y) position
    ret,robot_pose=robot.get_pose()
    
    x=robot_pose["pose"]["x"]
    y=robot_pose["pose"]["y"]
    theta=robot_pose["pose"]["th"]

    #print("robot_pose: ",x)
    print("\nUpdating position: ")
    x_key=soar_agent.UpdateWme(x_key, float(x))
    y_key=soar_agent.UpdateWme(y_key, float(y))
    #z_key=soar_agent.UpdateWme(z_key, "0.73455")
    theta_key=soar_agent.UpdateWme(theta_key, float(theta))

    n=1
    soar_agent.runFor(n) 

    outputWMEs=soar_agent.getOutputWMEs()
    print("\n outputWMEs before: ",outputWMEs)
    if(len(outputWMEs)>0):
        owList=outputWMEs.split(";")
        ow=owList[0]
        w=ow.split(",")
        print("output==> ",w)
        complete_key=soar_agent.InputWme(w[2],"status", "complete")


#I3,move,V3;V3,target,Waypoint_CB#0;

    #x_key=soar_agent.InputWme("L1","x", "1.5")
    time.sleep(3)

#soar_agent.runForever()  

#allWmesInRete=soar_agent.getAllWmesInRete()
#print("\n allWmesInRete: ",allWmesInRete)
#soar_agent.printAllWME()

    


""" ACTION """


#Send moveto waypoint
waypoint_ID=waypoints[1]
robot.move_to(waypoint_ID, True) #assync mode true




soar_agent.close_agent()


#############################################
## SAND BOX CODE
"""
print("Adding input to wm...")
sensorReadings="I2,block,B1;I2,block,B2;I2,block,B3;B1,x-location,1;B1,y-location,0;B1,color,red;B2,x-location,2;B2,y-location,0;B2,color,blue;B3,x-location,3;B3,y-location,0;B3,color,yellow;"
soar_agent.addSensorReadingsToInput(sensorReadings)

inputLink_id=soar_agent.getInputLink()
print("InputLink: ", inputLink_id)

name_letter='I'
name_number=1
WMEsUnder=soar_agent.getWMEsUnder(name_letter, name_number)
print("WMEsUnder: ",WMEsUnder)
"""

