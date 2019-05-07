# -*- coding: utf-8 -*-
import numpy as np
#Here we initialize the global variables for Reinforcement Learning.
print("Initializating Reinforcement Learning.")
# Basic parameters (taken from exp.py)----------------------------------------------
TASK_ID = "rm_rl_3k"  # task filename from tasks folder #has to be deleted
ENVIRONMENT_TYPE = "VREP with ROS"   
SPEED_RATE = 3.0  # Recommended: REAL ROBOT: 1.0 (x1), VREP: 3.0 (x3)

N_STEPS = 60#00 * 4# * 60#60  # 1 step ~ 1 second (Sets LE.N_STEPS)
DISPLAY_STEP = 10#00 #1800  # Policy will be printed each DISPLAY_STEP

FILE_MODEL = TASK_ID + "_model"  # MODEL environment only

# Learning parameters ----------------------------------------------------------
ALGORITHM = "TOSL" # "TOSL": true online SARSA lambda, "SL": SARSA lambda, "S": SARSA, Q: "Q-learning
ACTION_STRATEGY = "QBIASSR" # "QBIASSR"; Q-biased softmax regression, "softmax": softmax regression, "eGreedy", "random", "exploit"
ALPHA = 0.1
GAMMA = 0.9
LAMBDA = 0.9
TEMPERATURE = 1
SUFFIX = ""  # str(N_STEPS) + "steps_" + str(N_EPISODES) +"epi"

# Parameters from task.py----------------------
# Task Parameters:
NAME = "RL_customized"
DESCRIPTION = "Risk mitigation using scene graph. 5 direction. 5 ranges each. 16 actions"
ROBOT = "turtlebot2i"
ENVIRONMENT = "VREP_SIM: Scene_Builder_training.lua"
ENVIRONMENT_DETAIL = "SpeedX1."

# Physical Parameters:
STEP_TIME = 1  # s

#distance_list  = [0.2, 0.6, 1.1, 1.7, 2.4, 3.5]
distance_list  = [0.2, 0.6, 1.2, 2.0, 3.0, 3.5]
INPUT_VARIABLES = {
    "distance_front":       distance_list[:-1],
    "distance_front_left":  distance_list[:-1],
    "distance_left":        distance_list[:-1],
    "distance_right":       distance_list[:-1],
    "distance_front_right": distance_list[:-1],
    "steering_direction":   np.array([-57.0, -10.0,  10.0])*np.pi/180.0
}
RANGE_DISPLACEMENT = 2.0
OUTPUT_VARIABLES = {
    "left_scale" : np.linspace(0.0, 1.2, 4),
    "right_scale": np.linspace(0.0, 1.2, 4)
}
INITIAL_STATE = 0  # (usually overwritten by the fist observation)
INITIAL_POLICY = 0

#Rewards for [collision, critical, warning, nothing, >displacement]
REWARDS = np.array([-10.0, -5.0, -1.0, -0.05, 5.0])

n_inputs = int
in_values = [None]
in_names = [None]
in_sizes = [int]
n_states = int
in_data = [None]

n_outputs = int
out_values = [None]
out_names = [None]
out_sizes = [int]
n_actions = int
out_data = [None]

#additional parameter
risk_max = 0.0
collected_distance = 0.0
min_dist_to_obstacle = 5.0
collision = False
r_warning = 0.0
r_critical = 0.0