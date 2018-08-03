# Turtlebot2i V-rep model

This folder contains the scene files e.g. **turtlebot2i.ttt** and Lua files for each component e.g. **turtlebot2i_lidar.lua**.

The scene files are:

1. mapir_lab_turtlebot2i.ttt
2. turtlebot2i.ttt
3. turtlebot2i_multirobot.ttt
4. warehouse_turtlebot2i.ttt
5. warehouse_turtlebot2i_v2.ttt
6. warehouse_turtlebot2i_v3.ttt

**Note:** Some components are not activated, and you can edit the Lua files in the scene. e.g.  the line 16 and 17 in camera_sr300 

'''
    sim.setExplicitHandling(object_sr300_camera_rgb, 1) -- 0: enable camera rgb
    sim.setExplicitHandling(object_sr300_camera_depth, 1) -- 1:disable camera depth
'''

**Note:** Enyu: If possible, I feel that it is better to explain the difference between models, or recommand one model. 

**The scene files contain short Lua scripts. Those scripts call the outside lua scripts to improve tracebility with GIT.**

The Lua files are:

1. turtlebot2i.lua 
2. turtlebot2i_camera.lua
3. turtlebot2i_dockstation.lua
4. turtlebot2i_GPS.lua
5. turtlebot2i_IMU.lua
6. turtlebot2i_lidar.lua
7. turtlebot2i_turtlebot2i.lua



And Enyu Cao wrote a lua script "NewScriptForDockStation" that slightly change the dockstation script, and it can hide the error messages during initalization.

