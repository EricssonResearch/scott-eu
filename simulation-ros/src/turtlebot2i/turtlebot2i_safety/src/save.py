# -*- coding: utf-8 -*-
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Save resulting log, arrays and charts """

import os
import time

import matplotlib.pyplot as plt
import numpy as np

import rl_var as var 
import rl_lp as lp

N_BINS = 6

path = None  # Save path (including prefix of filename)


def new_dir(results_path, caption):
    """ Create directory in which results will be saved """
    global path

    if not os.path.exists(results_path):
        os.makedirs(results_path)
    string_date = time.strftime("%Y_%m_%d_%H_%M", time.gmtime())
    folder = results_path + "/" + string_date + "_" + caption
    if not os.path.exists(folder):
        os.makedirs(folder)
    path = folder + "/" + caption


def simple(data, suffix):
    """ Save 'data' in file 'filename'. If the data is a numpy array, it is
    saved as binary (.npy) """
    filename = path + "_" + suffix if suffix else path
    if suffix:
        filename += "_" + suffix
    if type(data) is np.ndarray:
        np.save(filename, data)
    else:
        filename += ".py"
        with open(filename, 'w') as f:
            f.write(data)


def log(mean_ave_r, mean_actual_step_time):
    """ Save logfile (textfile)"""

    txt = '""" RL-ROBOT log. '
    txt += time.strftime("%d %b %Y %H:%M", time.gmtime()) + '"""\n\n'
    # Save Parameters
    txt += "# EXPERIMENT PARAMETERS\n"
    txt += "TASK_ID = '" + var.TASK_ID + "'\n"
    txt += "ENVIRONMENT_TYPE = '" + var.ENVIRONMENT_TYPE + "'\n"
    txt += "SPEED_RATE = " + str(var.SPEED_RATE) + "\n"
    txt += "N_STEPS = " + str(var.N_STEPS) + "\n\n"

    txt += "FILE_MODEL = '" + var.FILE_MODEL + "'\n"

    txt += "ALGORITHM = '" + var.ALGORITHM + "'\n"
    txt += "ACTION_STRATEGY = '" + var.ACTION_STRATEGY + "'\n"
    txt += "GAMMA = " + str(var.GAMMA) + "\n"
    txt += "ALPHA = " + str(var.ALPHA) + "\n"
    txt += "LAMBDA = " + str(var.LAMBDA) + "\n"
    txt += "TEMPERATURE = " + str(var.TEMPERATURE) + "\n\n"

    txt += "# TASK PARAMETERS \n"
    txt += "TASK_NAME = '" + var.NAME + "'\n"
    txt += "TASK_DESCRIPTION = '" + var.DESCRIPTION + "'\n"
    txt += "TASK_ROBOT = '" + var.ROBOT + "'\n"
    txt += "TASK_ENV = '" + var.ENVIRONMENT + "'\n"
    txt += "TASK_ENV_DETAIL = '" + var.ENVIRONMENT_DETAIL + "'\n\n"

    txt += "STEP_TIME = " + str(var.STEP_TIME) + "\n"
    txt += "RANGE_DISPLACEMENT = " + str(var.RANGE_DISPLACEMENT) + "\n\n"

    txt += "N_STATES = " + str(var.n_states) + "\n"
    txt += "N_INPUTS = " + str(var.n_inputs) + "\n"
    txt += "INPUT_NAMES = " + str(var.in_names) + "\n"
    txt += "distance_list = " + str(var.distance_list) + "\n\n"

    txt += "N_ACTIONS = " + str(var.n_actions) + "\n"
    txt += "N_OUTPUTS = " + str(var.n_outputs) + "\n"
    txt += "OUTPUT_NAMES = " + str(var.out_names) + "\n\n"

    txt += "TASK_REWARDS = " + str(list(var.REWARDS)) + "\n"
    txt += "INITIAL_STATE = " + str(lp.s0) + "\n\n"

    # Save results
    txt += "# RESULTS\n"
    txt += "Elapsed_time = %0.2f" % lp.elapsed_time + "\n"
    txt += "Mean_actual_step_time = %0.6f" % mean_actual_step_time + "\n\n"
    txt += "Mean_Average_reward = %0.2f" % mean_ave_r + "\n"

    filename = path + "_log.py"
    with open(filename, 'w') as f:
        f.write(txt)


def arrays(suffix=""):
    """ Save resulting arrays: Policy, V, Q, Q_count (textfile)"""
    filename = path + "_" + suffix if suffix else path
    np.savez_compressed(
        filename, Policy=lp.policy, V=lp.v, Q=lp.q, Q_count=lp.q_count)


def plot_simple(data, suffix="", tittle=""):
    """ Plot simple average Reward per step """
    filename = path + "_" + suffix if suffix else path
    plt.figure()
    plt.plot(data)
    plt.axis([0, data.size, -5, 10])
    plt.xlabel('STEP')
    plt.ylabel('AVERAGE REWARD')
    plt.title(tittle)
    plt.savefig(filename)
    return


def plot_simple_epi(data, suffix="", tittle=""):
    """ Plot simple average Reward per episode) """
    filename = path + "_" + suffix if suffix else path
    plt.figure()
    plt.plot(data)
    plt.axis([0, data.size - 1, -5, 10])
    plt.xlabel('EPISODE')
    plt.ylabel('AVERAGE REWARD')
    plt.title(tittle)
    plt.savefig(filename)
    return


def plot_mean(data, epi):
    """ Plot mean of average Reward per step """
    filename = path
    tittle = var.TASK_ID
    labl = var.ALGORITHM + " " + var.ACTION_STRATEGY + " " + var.SUFFIX
    plt.figure()
    plt.plot(data, label=labl)
    plt.axis([0, int(data.size) - 1, -5, 10])
    if epi == "ALL":
        plt.xlabel('EPISODE')
    else:
        plt.xlabel('STEP')
    plt.ylabel('MEAN of average Reward')
    plt.title(tittle + "  (Mean)")
    plt.legend(loc='lower right', numpoints=1)
    plt.savefig(filename)
    return
