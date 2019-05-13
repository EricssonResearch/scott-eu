# -*- coding: utf-8 -*-
# Modified from: https://github.com/angelmtenor/RL-ROBOT/blob/master/show.py
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Print custom messages """

import training_rl as exp
import rl_lp as lp


def process_count(caption, rep, epi, episodic):
    """ print current repetition and episode """
    print("-" * 50)
    print(caption)
    if exp.N_REPETITIONS > 1:
        print('repetition \t{0} of {1}'.format(rep + 1, exp.N_REPETITIONS))
    if episodic:
        print('episode \t{0} of {1}'.format(epi + 1, exp.N_EPISODES))
    print("-" * 50)


def process_remaining(rep, epi):
    """ print remaining processes """
    if rep < exp.N_REPETITIONS - 1:
        remaining_processes = (exp.N_REPETITIONS * exp.N_EPISODES -
                               rep * exp.N_EPISODES + epi + 1)
        remaining_time = remaining_processes * lp.elapsed_time
        print("Remaining time: \t {0:.2f}m ({1:.2f}h)".format(
            remaining_time / 60.0, remaining_time / 3600.0), "\n")


def process_summary():
    """ print process summary """
    print("-" * 22, "END", "-" * 23)
    print("Number of steps: \t", str(lp.step + 1))
    print("Actual Step Time: \t %.6f" % lp.actual_step_time + "s")
    print("Elapsed Time:\t\t %.2f" % lp.elapsed_time,
          "s  ( %.2f" % (lp.elapsed_time / 60),
          "m %.2f" % (lp.elapsed_time / 60 / 60), "h )")
    print("Average Reward: \t %.2f" % lp.final_average_reward)
    print("-" * 50, "\n")
