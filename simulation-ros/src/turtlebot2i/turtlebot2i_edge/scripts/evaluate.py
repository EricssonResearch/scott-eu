#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import argparse
import os
import shutil
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from turtlebot2i_edge import TaskOffloadingEnv


def preprocess_offloading(logs):
    # boolean -> 0/1
    for k in ['published', 'edge_model']:
        logs[k] = [1 if bool_ else 0 for bool_ in logs[k]]

    # list -> numpy array
    logs = {k: np.array(v) for k, v in logs.items()}

    # remove steps in which the output was not available
    idx_published = np.where(logs['published'] == 1)[0]
    for k in logs.keys():
        if k != 'published':
            logs[k] = logs[k][idx_published]

    return logs


def preprocess_safety(logs):
    # add risk x speed
    logs['risk_value_x_speed'] = [risk_value * speed for risk_value, speed in zip(logs['risk_value'], logs['speed'])]

    # list -> numpy array
    logs = {k: np.array(v) for k, v in logs.items()}

    return logs


def filter_logs_by_action(logs, actions):
    idx_action = np.array([idx for idx in range(len(logs['action'])) if logs['action'][idx] in actions])
    logs = {k: v[idx_action] for k, v in logs.items()} if len(idx_action) > 0 else None
    return logs


def get_metric_per_step(logs, metric):
    metric_mean = logs[metric].mean()
    metric_std = logs[metric].std()
    metric_min = logs[metric].min()
    metric_max = logs[metric].max()
    return metric_mean, metric_std, metric_min, metric_max


def get_percentage_metric(logs, metric):
    metric_perc = sum(logs[metric]) / len(logs[metric]) * 100
    return metric_perc


def print_metrics(output, output_dir, filename=None):
    if filename is None:
        filename = 'metrics.txt'
    with open(os.path.join(output_dir, filename), mode='a') as f:
        f.write(output + '\n')


def plot_action_pie(logs, output_dir, filename=None):
    n_actions = TaskOffloadingEnv.action_space.n
    wedge_sizes = [len(np.where(logs['action'] == a)[0]) for a in range(n_actions)]
    labels = ['compute on robot', 'compute on edge', 'use last output']
    patches, _, _ = plt.pie(wedge_sizes, autopct='%1.1f%%', shadow=True, startangle=90)
    plt.legend(patches, labels)

    if filename is None:
        filename = 'actions.jpg'
    filepath = os.path.join(output_dir, filename)
    plt.savefig(filepath)


def plot_2d_density(logs, metric_x, metric_y, output_dir, filename=None):
    try:
        plot = sns.jointplot(
            data=logs,
            x=metric_x,
            y=metric_y,
            kind='kde'      # 'scatter', 'reg', 'resid', 'kde', or 'hex'
        )
        plot.set_axis_labels(xlabel=metric_x.replace('_', ' '), ylabel=metric_y.replace('_', ' '))

        if filename is None:
            filename = '%s_%s.jpg' % (metric_x, metric_y)
        filepath = os.path.join(output_dir, filename)
        plot.savefig(filepath)
    except np.linalg.LinAlgError:
        pass


def get_command_line_arguments():
    parser = argparse.ArgumentParser(description='Evaluate logs of task offloading agents.')
    parser.add_argument('logs', type=str, help='path to the directory containing logs (recursively analyzed)')
    return parser.parse_args()


def main():
    args = get_command_line_arguments()

    for dir_path, dir_names, filenames in os.walk(args.logs):
        if 'logs' not in dir_names:
            continue

        print('Evaluating %s... ' % dir_path, end='')

        logs_path = os.path.join(dir_path, 'logs')
        logs_offloading = np.load(os.path.join(logs_path, 'logs_offloading.npz'), allow_pickle=True)['logs']
        logs_safety = np.load(os.path.join(logs_path, 'logs_safety.npz'), allow_pickle=True)['logs']
        n_episodes = len(logs_offloading)
        assert n_episodes == len(logs_safety)

        for episode in range(n_episodes):
            episode_dir = os.path.join(dir_path, 'episode_%d' % (episode+1))
            if os.path.exists(episode_dir):
                shutil.rmtree(episode_dir)
            os.mkdir(episode_dir)

            episode_logs_offloading = preprocess_offloading(logs_offloading[episode])
            episode_logs_safety = preprocess_safety(logs_safety[episode])

            for metric in ['reward', 'latency', 'energy']:
                metric_mean, metric_std, metric_min, metric_max = get_metric_per_step(episode_logs_offloading, metric)
                output = '%s per step: %.2f +- %.2f (min=%.2f, max=%.2f)' % \
                         (metric, metric_mean, metric_std, metric_min, metric_max)
                print_metrics(output, episode_dir)

            for metric in ['published', 'edge_model']:
                metric_perc = get_percentage_metric(episode_logs_offloading, metric)
                output = '%s: %.2f%%' % (metric, metric_perc)
                print_metrics(output, episode_dir)

            for metric in ['risk_value', 'risk_value_x_speed']:
                metric_mean, metric_std, metric_min, metric_max = get_metric_per_step(episode_logs_safety, metric)
                output = '%s per step: %.2f +- %.2f (min=%.2f, max=%.2f)' % \
                         (metric, metric_mean, metric_std, metric_min, metric_max)
                print_metrics(output, episode_dir)

            for metric in ['duration', 'in_critical_zone', 'in_warning_zone', 'in_safe_zone', 'collisions',
                           'mean_distance', 'completed_pick_and_place']:
                output = '%s: %f' % (metric, episode_logs_safety[metric])
                print_metrics(output, episode_dir)

            plot_action_pie(episode_logs_offloading, episode_dir)
            plot_2d_density(episode_logs_offloading, 'risk_value', 'latency', episode_dir)
            plot_2d_density(episode_logs_offloading, 'risk_value', 'edge_model', episode_dir)

            filtered_logs = filter_logs_by_action(episode_logs_offloading, [2])
            if filtered_logs is not None:
                filename = 'risk_value_temporal_coherence_use_last.jpg'
                plot_2d_density(filtered_logs, 'risk_value', 'temporal_coherence', episode_dir, filename=filename)

            filtered_logs = filter_logs_by_action(episode_logs_offloading, [0, 1])
            if filtered_logs is not None:
                filename = 'risk_value_temporal_coherence_compute.jpg'
                plot_2d_density(filtered_logs, 'risk_value', 'temporal_coherence', episode_dir, filename=filename)

        print('done')


if __name__ == '__main__':
    main()
