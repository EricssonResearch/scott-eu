# Turtlebot2i V-rep model

This folder contains the scene files e.g. **turtlebot2i.ttt** and Lua files for each component e.g. **turtlebot2i_lidar.lua**.

- The main scene file is `warehouse_turtlebot2i.ttt`. 
- The other files are under development.


**Note:** Some sensors are disabled by default to save processing times. To activate, just comment the lines with the function call `sim.setExplicitHandling()`.
Example of line that should comment to enable the sensor:

```
    sim.setExplicitHandling(object_sr300_camera_rgb, 1) -- 0: enable camera rgb
    sim.setExplicitHandling(object_sr300_camera_depth, 1) -- 1:disable camera depth
```

**The scene files contain short Lua scripts. Those scripts call the outside lua scripts to improve tracebility with GIT.**

The Lua files are:

1. turtlebot2i.lua 
2. turtlebot2i_camera.lua
3. turtlebot2i_dockstation.lua
4. turtlebot2i_IMU.lua
5. turtlebot2i_lidar.lua
6. turtlebot2i_turtlebot2i.lua


