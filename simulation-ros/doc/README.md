# 1. Overview

This *ROS workspace* provides all the libraries for turtlebot2i robot. This robot is intended to be used inside the automated warehouse and it will be mainly resposible for picking products from shelves and placing in the conveyor belts to get delivered. The warehouse scenario also expects the presence of other robots and humans, which corresponds to a collaborative scenario. Here, the turtlebot2i workspace is comprised of three [metapackages](http://wiki.ros.org/Metapackages): 

1. **base_robot:** Contains all functionality related to the turtlebot2i base, this means the Kobuki robot and its acessories (plates, support, sensors). The base_robot is responsible for enabling the robot navigation and the environment perception.
2. **phantomx_pincher_arm:** Formed by the methods that enable the robot arm manipulation. This includes the arm motion planning through Moveit! library.
3. **turtlebot2i:** Integrates the *base_robot* and *phantomx_pincher_arm* functionalities. Includes codes related to the interaction of the robot with the warehouse.

**Note:** Each metapackage contains other ROS packages with their respective documentation.

# 2. Turtlebot2i Robot

The turtlebot is a family of robots generally used for educational and research purposes. The [turtlebot2i](http://www.trossenrobotics.com/interbotix-turtlebot-2i-mobile-ros-platform.aspx) is one of these robots and it is manufactured by Trossen Robotics. This robot is equipped with these following main parts:

<img style="float: right;" src="http://www.trossenrobotics.com/Shared/Images/Product/Interbotix-Turtlebot-2i-Mobile-ROS-Platform/Img3077_isolated.jpg" width="200">

- Kobuki Mobile Base
- Pincher MK3 Robo Arm
- Intel RealSense 3D Camera SR300-Series
- Orbbec Astra Cam
- Accelerometer/Gyro/Compass

Besides that, the turtlebot2i was equipped with the [Scanse LiDAR](http://scanse.io/), which is a low cost sensor that returns range measurements of 360 degrees of the environment.

# 3. Simulated Warehouse

Before running experiments with real robots, it is interesting to validate the code in a simulated environment. In this project, the [V-REP simulator](http://www.coppeliarobotics.com) is used to reproduce the warehouse scenario, the robot behavior and other elements that exist in this environment (e.g. trucks, workers). In this sense, the V-REP is used just for visualization purpose.
The main reason for choosing V-REP is the presence of many ready to use models, possibility to draw new models and demands relatively less computational power (compared to Gazebo).

**Note:** As V-REP does not have a native support to ROS, the [ROS Interface](http://www.coppeliarobotics.com/helpFiles/en/rosInterf.htm) plugin should be installed. 
**Note 2:** Most of the lua scripts of the scenario are stored outside V-REP. This means that the lua scripts contained in the scenario just load these files (they are inside *vrep* folder). This was necessary for version control of the scripts and better reusability.

# 4. ROS

[Robot Operating System (ROS)](http://www.ros.org/) is framework that contains several tools and libraries for robot development. ROS is maintained by the community and many important universities collaborates with it. ROS is also used by the majority of the researchers, thus, ROS methods and codes are almost a standard among them.
The main advatage of ROS is that (in most of the cases) it dispenses the necessity of developing low level algorithms for the robot by reusing the the code avaible in its repository. 

**Note:** If you are not familiar with ROS, it is recommended the following materials: [ROS Concepts](http://wiki.ros.org/ROS/Concepts), [ROS wiki](http://wiki.ros.org/), [ROS techincal overview](http://wiki.ros.org/ROS/Technical%20Overview) and [ROS architecture](www.willowgarage.com/sites/default/files/icraoss09-ROS.pdf).

# 5. Installation

1. Install ROS Kinect. Instructions can be found in this link: http://wiki.ros.org/kinetic/Installation
2. If using simulated environment, install V-REP: http://www.coppeliarobotics.com/downloads.html
    2.1. Install V-REP ROS Interface.
3. Clone this project and set this project path as a new ROS workspace. Instructions can be found in this link: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
4. Compile the workspace from the project root. Here, the catkin package structure is used.
    ```
    $ catkin build
    ```
5. In each package there is a README.md which contains instuctions to use them.

**Note:** ROS has a limited support to Windows. It is recommended to install ROS on linux distributions.

