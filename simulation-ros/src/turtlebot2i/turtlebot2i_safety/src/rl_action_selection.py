# -*- coding: utf-8 -*-
# Modified from: https://github.com/angelmtenor/RL-ROBOT/blob/master/action_selection.py
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Exploration-exploitation strategy """

import math
import random
import sys

import numpy as np

import rl_lp as lp
import rl_var as var
import training_rl

if var.ACTION_STRATEGY == "QBIASSR":
    import rl_action_qbiassr as action_qbiassr

EPSILON = 0.3  # e-greedy
LEAST_EXPLORED = 0.3  # Probability of choose the least explored action
TEMPERATURE = var.TEMPERATURE  # Temperature Parameter for Softmax Boltzmann
initiated = False


def setup(): #called in setup in agent.py
    """ Initialize QBIASSR if needed """
    global initiated
    if var.ACTION_STRATEGY == "QBIASSR":
        action_qbiassr.setup()
    initiated = True


def execute(s):
    """ From state s select an action a """

    if var.ACTION_STRATEGY == "exploit":
        selected_action = exploit_policy(s)

    elif var.ACTION_STRATEGY == "random":
        selected_action = random_action()

    elif var.ACTION_STRATEGY == "eGreedy":
        selected_action = egreedy(s, EPSILON)

    elif var.ACTION_STRATEGY == "softmax":
        selected_action = softmax(s)

    elif var.ACTION_STRATEGY == "QBIASSR":  # novel technique
        selected_action = action_qbiassr.select_biased_action(s)

    else:
        sys.exit("ERROR:   WRONG ACTION STRATEGY: " + var.ACTION_STRATEGY)
    return selected_action


def exploit_policy(s):
    """ Exploit the action a given an state s according to the Policy """
    selected_action = lp.policy[s]
    return selected_action


def random_action():
    """ Select a random action a (uniform distribution) """
    # random.seed()
    selected_action = random.randint(0, var.n_actions - 1)
    return selected_action


def egreedy(s, e):  # if e = 0.3_: 30% exploration
    """ Select an action a given a state s based on egreedy exploration """
    # random.seed()
    if random.random() < e:
        selected_action = random_action()
    else:
        selected_action = exploit_policy(s)
    return selected_action


def egreedy_least_explored(s, e, least):
    """ Select an action a given a state s based on egreedy exploration
        improving the probability of selecting the least explored action  """
    # random.seed()
    if random.random() < e:
        if random.random() < least:
            selected_action = 0
            for i in range(var.n_actions):
                if lp.q_count[s, i] < lp.q_count[s, selected_action]:
                    # exploration holds the number of times that the cells
                    # of Q[s, a] have been explored
                    selected_action = i
        else:
            selected_action = random_action()
    else:
        selected_action = exploit_policy(s)
    return selected_action


def softmax(s):
    """ Select an action a given a state s based on Boltzmann exploration """
    selected_action = -1
    # 1: Get the probabilities
    pa = np.zeros(var.n_actions)
    for i in range(var.n_actions):
        pa[i] = math.exp(lp.q[s, i] / TEMPERATURE)
    pa = np.divide(pa, sum(pa))

    # 2: Select the action
    # random.seed()
    ran = random.random()
    accum = 0.0
    for i in range(var.n_actions):
        accum = accum + pa[i]
        if ran < accum:
            selected_action = i
            break
    return selected_action
