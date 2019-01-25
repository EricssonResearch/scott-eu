# 1. Overview

This package provides methods and resources related to the robot safety, specifically the risk management.
The main components of this package are the risk assessment and the risk mitigation. Both methods are based on _fuzzy logic_ and _neuro-fuzzy_ methods.


# 2. Risk Management

The risk management comprises the full pipeline to address the safety. Here safety means the injury risk associated to the human and robot collaboration.

* The first component of the risk management is the **risk assessment**, which is responsible for analyzing all the existing risks in the scenario and evaluate the risk level.

* The second component is the **risk mitigation**, which is responsible for reducing the risk based on the risk analysis.

Both risk management and mitigation is based on fuzzy logic and neuro-fuzzy.
The **trained rules** are stored in the following files:

* `assessment_rules.py` for the risk assessment.

* `mitigation_rules.py` for the risk mitigation.


# 3. Running the Risk Assessment and Mitigation (Risk Management)

The risk assessment can be run by executing the following:
```
rosrun turtlebot2i_safety risk_management.py
```

Alternatively it is possible to run the launch script:
```
roslaunch turtlebot2i_safety 
```


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


# 5. Safety Message

A custom message *safety_zone_msg* that informs the size of safety zones is provided. It formed by four fields:

- **Header:** Standard ROS header message.
- **clear_safety_zone_radius:** Radius size of the robot's most external safety zone.
- **warning_safety_zone_radius:** Radius size of the robot's intermediary safety zone.
- **critical_safety_zone_radius:** Radius size of the robot's most internal safety zone.

