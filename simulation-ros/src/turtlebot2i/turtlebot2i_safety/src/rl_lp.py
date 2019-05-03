# -*- coding: utf-8 -*-
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Learning process """

import time

import numpy as np

import rl_agent as agent
import show
import training_rl
import rl_action_selection as action_selection
import rl_var as var
import save

learning_module = "algorithm_" + var.ALGORITHM.lower()
learning_algorithm = __import__(learning_module)
np.set_printoptions(precision=2)

step_time = 0
step = -1  # Current learning step
a = -1  # Current action
s = -1  # current state
sp = -1  # state reached (s')
s0 = -1  # initial state
ap = -1  # next action selected (a')
r = 0  # current reward obtained (R)

delta = 0  # Temporal Difference (TD) Error
q = None  # Q-matrix
v = None  # Value function
policy = None  # Current Policy
q_count = None
alpha = 0

elapsed_time = 0  # seconds
actual_step_time = 0  # seconds: simulation time / actual_time
final_average_reward = 0  # Resulting average R at last step

ave_v_step = None  # Average V per step (for plotting)
ave_r_step = None  # Average R obtained per step
sasr_step = None  # History of (s,a,s',R) per step

q_limit = 0

initiated = False

initial_step_time = step_time  # auxiliary
mark = time.time()

training_process = True

def setup():
    """ Create module variables """
    global step_time, step, s, sp, a, ap, r, alpha, delta, q, v, policy, q_count
    global t_sas, r_sas, elapsed_time, actual_step_time
    global final_average_reward, ave_v_step, ave_r_step, sasr_step, q_limit, s0
    global initiated, initial_step_time, training_process

    agent.setup()

    step_time = var.STEP_TIME / var.SPEED_RATE
    initial_step_time = step_time
    # only used in case we want to modify step_time from a task module
    # to speed up the experiment when the robot reaches the goal
    step = 0

    s = agent.observe_state()
    s0 = s

    sp = -1
    a = var.INITIAL_POLICY
    ap = -1
    r = 0
    alpha = var.ALPHA
    delta = 0

    q = np.zeros((var.n_states, var.n_actions), dtype=np.float32)
    v = np.zeros(var.n_states, dtype=np.float64)
    policy = np.full(var.n_states, var.INITIAL_POLICY, dtype=np.uint32)
    q_count = np.zeros((var.n_states, var.n_actions), dtype=np.uint64)

    elapsed_time = 0
    actual_step_time = 0
    final_average_reward = 0

    ave_v_step = np.zeros(var.N_STEPS)
    ave_r_step = np.zeros(var.N_STEPS)
    sasr_step = np.zeros((var.N_STEPS, 4))

    learning_algorithm.setup()

    q_limit = round(max(var.REWARDS) / (1 - var.GAMMA))
    #  q_limit = max(var.REWARDS)/(1-var.GAMMA)

    if q_limit != 100:
        print("q_limit = ",
              str(q_limit),
              ". Softmax regression will be normalized as q_limit = 100")
        time.sleep(2)

    initiated = True
    training_process = True
    return


def run_training():
    """ Execute the learning Process"""
    global step, s, sp, a, ap, r, alpha
    global q, v, policy, q_count
    global t_sas, r_sas
    global ave_r_step, sasr_step
    global elapsed_time, actual_step_time
    global final_average_reward
    global mark
    global training_process

    assert initiated, ("learning process not initiated! setup() "
                       "must be previously called")
    

    # Start learning process: -----------------------------
    if step < var.N_STEPS:  #N_STEPS = 60 * 60
        if step > 0 and step % var.DISPLAY_STEP == 0:
            print("STEP: ", step)
            
        # Execute the selected learning algorithm. Also change lp variables
        learning_algorithm.execute()

        # update analysis and model arrays
        q_count[s, a] += 1
        sasr_step[step, 0] = s
        sasr_step[step, 1] = a
        sasr_step[step, 2] = sp
        sasr_step[step, 3] = r
        ave_v_step[step] = np.mean(v)
        if step == 0:
            ave_r_step[step] = sasr_step[step, 3]
            mark = time.time()
        else:
            ave_r_step[step] = np.average(sasr_step[0:step, 3])

        # Display information
        if step > 0 and step % var.DISPLAY_STEP == 0:
            print("s:", s, " a:", a, " sp:", sp, " R: %0.2f" % r)
            print("Average Reward: %0.2f" % ave_r_step[step], "\n")

        # Update state
        s = sp
        a = ap
    else:
        # End of learning process ----------------------
        if training_process:
            training_process = False
            results_path = "RL_results/"
            caption = (var.TASK_ID + "_" + var.ALGORITHM + "_" + var.ACTION_STRATEGY)
            if var.SUFFIX:
                caption += "_" + var.SUFFIX
            save.new_dir(results_path, caption)  # create result directory

            final_average_reward = ave_r_step[step-1]
            elapsed_time = (time.time() - mark)
            actual_step_time = elapsed_time / (step + 1)
            show.process_summary()

            save.plot_mean(ave_r_step, 0)
            save.simple(ave_r_step, "aveR")
            final_r = ave_r_step[step-1]
            save.log(final_r, actual_step_time)
            save.arrays()
        # Training is finished and exploit the optimal action for each state
        s = agent.observe_state()
        a = agent.select_action(s)
        agent.execute_action(a)
    return
