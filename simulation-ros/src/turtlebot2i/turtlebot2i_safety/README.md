# 1. Overview

This package provides methods and resources related to the robot safety, specifically the risk management.
The main components of this package are the risk assessment and the risk mitigation. The risk management is implementing _fuzzy logic_ method and the risk mitigation is implementing _fuzzy logic_ and _reinforcement learning_ methods.


# 2. Risk Management

The risk management comprises the pipeline to address the safety. Here safety means the injury risk associated to the human and robot collaboration.

* The first component of the risk management is the **risk assessment**, which is responsible for analyzing all the existing risks in the scenario and evaluate the risk level.

The risk assessment rules are stored in the `assessment_rules.py`.

* The second component is the **risk mitigation**, which is responsible for reducing the risk based on the risk analysis.

The risk mitigation using fuzzy logic system is implemented in ´risk_mitigation_fls.py´ and the rules are stored in the `mitigation_rules.py`.

The risk mitigation using reinforcement learning with fully connected network is implemented in  `training_mlp.py` 

The risk mitigation using reinforcement learning with convoloutional neural network is implemented in  `training_cnn.py` 

The risk mitigation using reinforcement learning with hybrid network is implemented in  `training_hybrid.py` 

# 3. Running the Risk Assessment and Mitigation (Risk Management)

Before running the risk assessment and risk mitigation module, make sure the scene graph generator module is running.

The risk assessment can be run by executing the following:
```
rosrun turtlebot2i_safety risk_assessment.py
rosrun turtlebot2i_safety zone_generation.py
```

The risk mitigation can be run by executing one of the following commands:
```
rosrun turtlebot2i_safety risk_mitigation_fls.py
```
or (replace `XXX` with either `mlp`, `cnn`, or `hybrid`)
```
rosrun turtlebot2i_safety training_mlp.py
```

Alternatively, it is possible to run the launch script (this script also runs the scenegraph generator module along with the risk assessment and risk mitigation modules):
```
roslaunch turtlebot2i_safety turtlebot2i_safety_fls.launch 
```
or 
```
roslaunch turtlebot2i_safety turtlebot2i_safety_train_rl.launch
```

**The use case scenarios compatible to this package is located in the scenario folder (`scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_description/v-rep_model/warehouse_turtlebot2i.ttt`).**

# 4. Generating Rules for Risk Assessment or Management

New rules can be generated or the rule file can be appended to make the robot address new situations.
There are three ways to do that:

## 4.1. Directly edit the rule file (`assessment_rules.py` or `mitigation_rules`)

The new rule must have the following format:
```python
rule001 = ctrl.Rule(object_type['StaObj'] & object_distance['Near'] & object_direction['Front'] , object_risk['VeryHigh'])
```
The rule number must be increased accordingly.

## 4.2. Build a training dataset and train a fuzzy logic/neuro-fuzzy model

Follow the scripts in the `Labeling_tools` folder.

# 5. Training mode

To train the risk mitigation module using reinforcement learning, change the following variables in the file `training_XXX.py`:

On ReinforceAgent initialization:
```
self.load_model = False
self.load_episode = 0
```

On main:
```
training_mode = True
```


# 6. Safety Message

A custom message *safety_zone_msg* that informs the size of safety zones is provided. It formed by four fields:

- **Header:** Standard ROS header message.
- **clear_safety_zone_radius:** Radius size of the robot's most external safety zone.
- **warning_safety_zone_radius:** Radius size of the robot's intermediary safety zone.
- **critical_safety_zone_radius:** Radius size of the robot's most internal safety zone.

