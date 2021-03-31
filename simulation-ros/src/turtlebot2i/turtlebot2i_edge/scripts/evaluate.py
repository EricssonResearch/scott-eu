#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import argparse
import os
import shutil
import copy
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from turtlebot2i_edge import TaskOffloadingEnv


def preprocess(logs):
    # flatten
    logs_ = copy.deepcopy(logs)
    logs = {k: [] for k in logs_[0].keys()}
    for episode_logs in logs_:
        for k, v in episode_logs.items():
            logs[k] += v

    # boolean -> 0/1
    for k in ['published', 'edge_model']:
        logs[k] = [1 if bool_ else 0 for bool_ in logs[k]]

    # list -> numpy array
    logs = {k: np.array(v) for k, v in logs.items()}

    # remove steps in which the output was not available
    idx_published = np.where(logs['published'] == 1)[0]
    for k, v in logs.items():
        if k != 'published':
            logs[k] = logs[k][idx_published]

    return logs


def filter_logs_by_action(logs, actions):
    idx_action = np.array([idx for idx in range(len(logs['action'])) if logs['action'][idx] in actions])
    logs = {k: v[idx_action] for k, v in logs.items()} if len(idx_action) > 0 else None
    return logs


def get_metric_per_step(logs, metric):
    metric_mean = logs[metric].mean()
    metric_std = logs[metric].std()
    return metric_mean, metric_std


def get_percentage_metric(logs, metric):
    metric_perc = sum(logs[metric]) / len(logs[metric]) * 100
    return metric_perc


def print_output(output, output_dir=None, filename=None):
    print(output)
    if filename is None:
        filename = 'metrics.txt'
    if output_dir is not None:
        with open(os.path.join(output_dir, filename), mode='a') as f:
            f.write(output + '\n')


def plot_action_pie(logs, output_dir=None, filename=None):
    n_actions = TaskOffloadingEnv.action_space.n
    wedge_sizes = [len(np.where(logs['action'] == a)[0]) for a in range(n_actions)]
    labels = ['compute on robot', 'compute on edge', 'use last output']
    patches, _, _ = plt.pie(wedge_sizes, autopct='%1.1f%%', shadow=True, startangle=90)
    plt.legend(patches, labels)

    if filename is None:
        filename = 'actions.jpg'
    filepath = os.path.join(output_dir, filename)
    if output_dir is not None:
        plt.savefig(filepath)
    plt.show()          # after saving!


def plot_2d_density(logs, metric_x, metric_y, output_dir=None, filename=None):
    plot = sns.jointplot(
        data=logs,
        x=metric_x,
        y=metric_y,
        kind='hex'      # 'scatter', 'reg', 'resid', 'kde', or 'hex'
    )
    plot.set_axis_labels(xlabel=metric_x.replace('_', ' '), ylabel=metric_y.replace('_', ' '))

    if filename is None:
        filename = '%s_%s.jpg' % (metric_x, metric_y)
    filepath = os.path.join(output_dir, filename)
    if output_dir is not None:
        plot.savefig(filepath)
    plt.show()          # after saving!


def get_command_line_arguments():
    parser = argparse.ArgumentParser(description='Evaluate task offloading agent.')
    parser.add_argument('logs', type=str, help='path to the logs')
    parser.add_argument('--output', type=str, help='path where to store results')
    return parser.parse_args()


def main():
    args = get_command_line_arguments()
    if args.output is not None and os.path.isfile(args.output):
        raise ValueError('Invalid output path, it is a file')
    if args.output is not None:
        shutil.rmtree(args.output)
        os.mkdir(args.output)

    logs = np.load(args.logs, allow_pickle=True)['logs']
    logs = preprocess(logs)

    for metric in ['reward', 'latency', 'energy']:
        metric_mean, metric_std = get_metric_per_step(logs, metric)
        output = '%s per step: %.2f +- %.2f' % (metric, metric_mean, metric_std)
        print_output(output, output_dir=args.output)

    for metric in ['published', 'edge_model']:
        metric_perc = get_percentage_metric(logs, 'published')
        output = '%s: %.2f%%' % (metric, metric_perc)
        print_output(output, output_dir=args.output)

    plot_action_pie(logs, output_dir=args.output)
    plot_2d_density(logs, 'risk_value', 'latency', output_dir=args.output)
    plot_2d_density(logs, 'risk_value', 'edge_model', output_dir=args.output)

    filtered_logs = filter_logs_by_action(logs, [2])
    if filtered_logs is not None:
        filename = 'risk_value_temporal_coherence_use_last.jpg'
        plot_2d_density(filtered_logs, 'risk_value', 'temporal_coherence', output_dir=args.output, filename=filename)

    filtered_logs = filter_logs_by_action(logs, [0, 1])
    if filtered_logs is not None:
        filename = 'risk_value_temporal_coherence_compute.jpg'
        plot_2d_density(filtered_logs, 'risk_value', 'temporal_coherence', output_dir=args.output, filename=filename)


if __name__ == '__main__':
    main()
