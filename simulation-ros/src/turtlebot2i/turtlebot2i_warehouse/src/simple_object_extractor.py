#!/usr/bin/env python

from SceneObjectExtractor import SceneObjectExtractor
import time
   
# Update rate in seconds
rate = 0.5

extractor = SceneObjectExtractor('127.0.0.1', 19997)
print('Connected to remote API server')

print('Getting scene properties (this can take a while)...') 

# Get all objects info once (for static properties)
extractor.get_all_objects_info() 

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

        # Remove the robot itself from the list
        obj_list = [i for i in obj_list if i.name!=robot.name]

        # Print detected objects of the vision sensor
        print(robot.name, robot.vision_sensor.name, obj_list)

    time.sleep(rate)

# Close the connection to V-REP
vrep.simxFinish(clientID)
