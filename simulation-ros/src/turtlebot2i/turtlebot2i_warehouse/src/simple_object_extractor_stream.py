#!/usr/bin/env python

from SceneObjectExtractor import SceneObjectExtractor
import time
import vrep
   
# Update rate in seconds
#rate = 0.1

extractor = SceneObjectExtractor('127.0.0.1', 19997)

# List of object names to retrieve information
# For now it is hardcoded
extractor.set_static_obj_names(['fake_obj', 'stairs', 'slidingDoor',      
                                'DockStationBody', 'DockStationBody#0',\
                                'ConveyorBeltBody', 'ConveyorBeltBody#0', 'ConveyorBeltBody#1', 
                                'ShelfBody', 'ShelfBody#0', 'ShelfBody#1'])
extractor.set_dynamic_obj_names(['Bill#3', 'product', 'fake_obj'])
extractor.set_robot_names(['turtlebot2i', 'turtlebot2i#0'])

print('Connected to remote API server')

print('Getting scene properties (this can take a while)...') 

# Get all objects info once (for static properties) and
#  prepare the callback for the streaming mode

extractor.operation_mode = vrep.simx_opmode_streaming
extractor.get_all_objects_info() 
extractor.update_robots_vision_sensor_info()
extractor.update_all_robots_vision_sensors_fov()
time.sleep(0.3) # streaming takes a while to get ready

extractor.operation_mode = vrep.simx_opmode_buffer
extractor.get_all_objects_info() 
extractor.update_robots_vision_sensor_info()
extractor.update_all_robots_vision_sensors_fov()


print('Finished getting scene properties!\n')

print('Started getting scene objects from vision sensor FOV...')

while True:
    # Get dynamic object info (pose and vel) periodically
    extractor.update_dynamic_obj_info() 

    # Update vision sensor info
    extractor.update_all_robots_vision_sensors_fov()

    # Get objects that are in the sensor FOV
    for robot in extractor.robot_obj_list:
        obj_list = extractor.get_objects_from_vision_sensor(robot.vision_sensor)

        if (obj_list != None):
            # Remove the robot itself from the list
            obj_list = [i for i in obj_list if i.name!=robot.name]

        # Print detected objects of the vision sensor
        print(robot.name, robot.vision_sensor.name, obj_list)

    #time.sleep(rate)

# Close the connection to V-REP
vrep.simxFinish(clientID)
