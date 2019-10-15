# 1. Overview

This package provides methods and resources related to the robot safety, specifically the risk management.
The main components of this package are the risk assessment and the risk mitigation. The risk management is implementing _fuzzy logic_ method and the risk mitigation is implementing _fuzzy logic_ and _reinforcement learning_ methods.


# 2. Risk Management

The risk management comprises the pipeline to address the safety. Here safety means the injury risk associated to the human and robot collaboration.

* The first component of the risk management is the **risk assessment**, which is responsible for analyzing all the existing risks in the scenario and evaluate the risk level.

The risk assessment rules are stored in the `assessment_rules.py`.

* The second component is the **risk mitigation**, which is responsible for reducing the risk based on the risk analysis.

The risk mitigation using fuzzy logic system is implemented in ´risk_mitigation_fls.py´ and the rules are stored in the `mitigation_rules.py`.

The risk mitigation using reinforcement learning with fully connected network is implemented in  `risk_mitigation_mlp.py` 

The risk mitigation using reinforcement learning with convoloutional neural network is implemented in  `risk_mitigation_cnn.py` 

The risk mitigation using reinforcement learning with hybrid network is implemented in  `risk_mitigation_hybrid.py` 

* **Note:** Before running risk management nodes, compile the *simulation-ros* stack and run `rosrun vrep_ros_interface install.sh`.

* **Note 2:** Before running risk management nodes, it is important to install all python requirements:
```
roscd turtlebot2i_safety
pip install -r src/requirements.txt
```

## 2.1. Simulated Scenario Setup

### 2.1.1. Loading V-REP scenario

1. Open the `warehouse_scene.ttt` scenario in V-REP.
2. Open the scipt parameters of the *Scene_Builder* element.
3. In *Value* field of  *Parameter properties*, use:
- `Scene_Builder_test1.lua` for testing the risk management.
- `Scene_Builder_training.lua` for training the reinforcement learning-based risk mitigation models.

### 2.1.1. Setting Occupancy Grid Maps

1. Move to turtlebot2i mapping package:
```
roscd turtlebot2i_mapping/maps
```
2. Edit `map.yaml` and uncomment the lines for risk mitigation training or test.

## 2.2. Running the Risk Assessment 

Before running the risk assessment and risk mitigation module, make sure the simulation, navigation, and scene graph generator module is running.
The simulation scenario must be running and have the correct map. The map can be checked by running the navigation module:
 
```
roslaunch turtlebot2i_navigation turtlebot2i_navigation_single.launch
```
The scene graph generator can be run by calling:
```
rosrun turtlebot2i_scene_graph scene_graph_generator.py
```

The risk assessment can be run by executing the both commands:
```
rosrun turtlebot2i_safety risk_assessment.py
rosrun turtlebot2i_safety zone_generation.py
```

## 2.3. Running Risk Mitigation

The risk mitigation can be run by executing one of the following commands:
```
roslaunch turtlebot2i_safety turtlebot2i_safety_fls.launch  # for Fuzzy-based risk mitigation
```
or
```
roslaunch turtlebot2i_safety turtlebot2i_safety_rl.launch  # for Reinforcement Learning-based risk mitigation
```
This will launch the risk assessment (safety zones), navigation stack and rviz.

### 2.3.1. Changing the RL Model for Risk Mitigation

There approaches can be used in the RL-based risk mitigation:
- Deep Q-Network using fully-connected architecture - **FCN** (`risk_mitigation_mlp.py`). 
- Deep Q-Network using 1D convolutional architecture - **CNN** (`risk_mitigation_cnn.py`).
- Deep Q-Network using hybrid architecture (combination of FCN and CNN) -  **Hybrid** (`risk_mitigation_hybrid.py`).

To change the RL approach, edit `turtlebot2i_safety_rl.launch` and modify the **type** of *turtlebot2i_safety* node.

This snippet exemplifies the configuration for CNN:
```
<node pkg="turtlebot2i_safety" name="risk_mitigation_cnn_py"        type="risk_mitigation_cnn.py"        output="screen"/>
```


**The use case scenarios compatible to this package is located in the scenario folder (`scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_description/v-rep_model/warehouse_scenario.ttt`).**

# 3. Generating Fuzzy Rules for Risk Assessment or Management

New rules can be generated or the rule file can be appended to make the robot address new situations.
There are three ways to do that:

## 3.1. Directly edit the rule file (`assessment_rules.py` or `mitigation_rules`)

The new rule must have the following format:
```python
rule001 = ctrl.Rule(object_type['StaObj'] & object_distance['Near'] & object_direction['Front'] , object_risk['VeryHigh'])
```
The rule number must be increased accordingly.

## 3.2. Build a training dataset and train a fuzzy logic/neuro-fuzzy model

Follow the scripts in the `Labeling_tools` folder.

# 4. Reinforcement Learning Training Mode

Check Section 2 to load the training setup using reinforcement learning.
After that, change the following variables in the file `risk_mitigation_XXX.py`:

On ReinforceAgent initialization:
```
self.load_model = False
self.load_episode = 0
```

On main:
```
training_mode = True
```

# 5. Safety Message

A custom message *safety_zone_msg* that informs the size of safety zones is provided. It formed by four fields:

- **Header:** Standard ROS header message.
- **clear_safety_zone_radius:** Radius size of the robot's most external safety zone.
- **warning_safety_zone_radius:** Radius size of the robot's intermediary safety zone.
- **critical_safety_zone_radius:** Radius size of the robot's most internal safety zone.

