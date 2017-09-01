# Autonomous Warehouse Simulation in VREP

This contains simulations required by the sandbox. It currently consists of the automated warehouse simulated in [VREP](http://www.coppeliarobotics.com/downloads.html) with all its elements and robots. 
Until the multi-objective optimisation (MOO) service is present, its results should be provided by the [mission.json](warehousecontroller/mission.json) file.
Until it contains the discret event simulations for modeling the supply chain dynamics, all required information should be provided  by the [mission.json](warehousecontroller/mission.json) file.

Access to this scene (both reading and control) is currently implemented by using VREP's remoteAPI in python. This is expected to change once [ROS](http://www.ros.org/) is deployed and used for controlling the robots in the scene.

## Deployment


Step 1:  Download and install v-rep simulator#

Go to : http://www.coppeliarobotics.com/downloads.html

and download the V-REP Pro Edu version appropriate for your operational system, e.g. Linux 64 bits.

Extract it to any folder.


Step 2: Open the warehouse simulation#

Go the V-REP’s root folder and run:

 v_repLauncher.exe
 
Click, File > Open scene…

Navigate to /scott-eu/simulated-enviroment and open logistics_multiple_robots.ttt

