# 1. Overview

This package provides resources to simulate a Multi-access Edge Computing (MEC) scenario and to train/test Reinforcement Learning (RL) agents for offloading the [scene understanding task](../turtlebot2i_mrcnn), which is the most demanding module of the risk management process.

The MEC scenario includes:
- A [V-REP](https://www.coppeliarobotics.com/) scene with a warehouse divided in a grid of equal-sized rooms. This constraint is necessary to add the building in the network simulation.
- A [ns-3](https://www.nsnam.org/) network with a wireless access network as well as a MEC server co-located to the base station (see the topology below). The network can be used by multiple mobile robots to communicate with the MEC server and can contain additional congesting nodes. The network simulates realistically also fading phenomena caused by the walls in the warehouse.
- A RL environment compliant with [OpenAI Gym](https://gym.openai.com/) for training and testing RL agents for task offloading. The RL environment provides integration with V-REP and ns-3 and performs the scene understanding while navigating from shelves to conveyor belts and the other way around (simulation of pick-and-place operations).

The general topology of the network is the following:
```
            Wireless
     *       *       ...     *       *
     |       |               |       |      Point-to-point
  robot1  robot2          robotN     BS ---------------------- MEC server
```

Currently, there is only one implementation of the network that uses a WiFi 802.11g as wireless part and a 100-Mbps point-to-point link. Other networks can be developed by adding classes inheriting from [WirelessNetwork](include/turtlebot2i_edge/wireless.h).

For the RL agents, [Keras-RL](https://keras-rl.readthedocs.io/en/latest/) is used. As of now, only [Deep Q-Network](https://keras-rl.readthedocs.io/en/latest/agents/dqn/) (DQN) is considered, but the library provides other working agents that can be easily added to the  [task offloading script](scripts/task_offloading.py).

Three naive agents are also available as baselines to evaluate the RL agents:
- *all_edge*: it always offloads the scene understanding to the MEC server.
- *all_robot*: it always performs locally the scene understanding. 
- *random*: it decides randomly whether to offload or perform locally the scene understanding.

The scene understanding is performed using the [turtlebot2i_scene_graph package](../turtlebot2i_scene_graph), so the scene graph is extracted from V-REP and not actually computed with [turtlebot2i_mrcnn](../turtlebot2i_mrcnn), which performs a computationally-expensive instance segmentation. This is necessary to pretend to have a powerful GPU on the MEC server, which is the assumption of the MEC scenario. Indeed, the scene graph extraction from V-REP is much more efficient than using the Mask R-CNN.  

# 2. Setup

## 2.1 ns-3 setup

To run the network simulation, you need to set up [ns-3](../../ns-3).

First, add the following environment variables:
```
# replace <path_to_repository> with your local path to this repository
echo "export NS3_ROOT=<path_to_repository>/simulation-ros/src/ns-3" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$NS3_ROOT/build/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```

Next, compile ns-3 with an optimized configuration:
```
cd $NS3_ROOT
./waf configure -d optimized
./waf
```

Finally, recompile the catkin workspace:
```
catkin clean -y
catkin build
```

## 2.2 V-REP setup

The [RL environment](src/turtlebot2i_edge/_task_offloading_env.py) communicates with V-REP by means of the V-REP remote API. Make sure you have [set it up](../../../doc/README.md#54-using-python-vrep-remote-api-optional) before continuing.

Since both the [scene graph generator](../turtlebot2i_scene_graph/src/turtlebot2i_scene_graph/_scene_graph_generator.py) and the [RL environment](src/turtlebot2i_edge/_task_offloading_env.py) use the V-REP remote API and the V-REP remote API server service allows only one client, you need to add other services. Furtherly, since the [RL environment](src/turtlebot2i_edge/_task_offloading_env.py) closes and restarts the V-REP scene after each collision, the services must be [continuous and not temporary](https://www.coppeliarobotics.com/helpFiles/en/remoteApiServerSide.htm). To do this, run the following commands:
```
for i in $(echo {2..5}); do
    let port=1998+$i
    echo "portIndex${i}_port = $port"
    echo "portIndex${i}_debug = false"
    echo "portIndex${i}_syncSimTrigger = true"
    echo ""
done >> $VREP_ROOT/remoteApiConnections.txt
```

## 2.3 Configurations

It is possible to tune several parameters in the [configurations](config). For example, the congestion of the network can be modified in the [network configurations](config/network.yaml).

The warehouse can be modified by editing the [scene builder script](../turtlebot2i_description/v-rep_model/warehouse_scene/vrep_scripts/scene_builder_edge.lua). If you modify the size of the warehouse and/or the rooms, do not forget to update the [network configurations](config/network.yaml). 

# 3. Train and test agents

## 3.1 Start V-REP scene

Before launching any launch file, you need to open and start the V-REP scene:
```
$VREP_ROOT/vrep.sh -s $(rospack find turtlebot2i_description)/v-rep_model/warehouse_scene/warehouse_scene_edge.ttt
```
## 3.2 Train DQN agent

To train the DQN agent, run the following command:
```
roslaunch turtlebot2i_edge task_offloading.launch agent:=dqn mode:=train
```
The trained weights are saved in the *models* directory, together with the training logs.

Notice that the navigation stack outputs several warnings and errors while the V-REP scene is stopped and restarted (environment reset). This is expected and you should not worry.

## 3.3 Test agent

To test the RL agent or one of the baselines, run one of the following commands:
```
roslaunch turtlebot2i_edge task_offloading.launch agent:=dqn            # DQN agent
roslaunch turtlebot2i_edge task_offloading.launch agent:=all_edge       # all_edge baseline
roslaunch turtlebot2i_edge task_offloading.launch agent:=all_robot      # all_robot baseline
roslaunch turtlebot2i_edge task_offloading.launch agent:=random         # random baseline
```
The test logs are saved in the *models* folder.

To get the results, you need to analyze the test logs:
```
roscd turtlebot2i_edge
python scripts/evaluate.py models/dqn --output results/dqn              # DQN agent
python scripts/evaluate.py models/all_edge --output results/all_edge    # all_edge baseline
python scripts/evaluate.py models/all_robot --output results/all_robot  # all_robot baseline
python scripts/evaluate.py models/random --output results/random        # random baseline
```
The results are saved in the *results* folder (or, if you changed the commands, in the path specified by the *output* argument).