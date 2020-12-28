from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from time import time
from problem import Problem
from utils import get_solver


def _prepare_and_save_figure(figure, n_mobile_devices, y_label, save_path):
    plt.figure(figure)
    plt.xticks(n_mobile_devices)
    plt.xlabel('number of MDs')
    plt.ylabel(y_label)
    # plt.yscale('log')
    plt.legend()
    plt.savefig(save_path)


def compare(solvers, min_mobile_devices, max_mobile_devices, step_mobile_devices, n_runs):
    n_mobile_devices = np.arange(min_mobile_devices, max_mobile_devices+1, step_mobile_devices)

    for i, s in enumerate(solvers):
        latencies = []
        energies = []
        latencies_ok = []
        execution_times = []

        for n in n_mobile_devices:
            latencies_run = []
            energies_run = []
            latencies_ok_run = []
            execution_times_run = []

            for r in range(n_runs):
                problem = Problem(n_mobile_devices=n, seed=r)
                solver = get_solver(s, problem)

                time_start = time()
                solution = solver.solve()
                time_end = time()

                latencies_run_, energies_run_, latencies_ok_run_ = solver.evaluate(solution)
                latencies_run.append(latencies_run_.sum())
                energies_run.append(energies_run_.sum())
                latencies_ok_run.append(latencies_ok_run_.sum() / len(latencies_ok_run_))
                execution_times_run.append(time_end - time_start)

            latencies_run = np.array(latencies_run)
            energies_run = np.array(energies_run)
            latencies_ok_run = np.array(latencies_ok_run)
            execution_times_run = np.array(execution_times_run)

            latencies.append(latencies_run.mean())
            energies.append(energies_run.mean() * 1e3)
            latencies_ok.append(latencies_ok_run.mean())
            execution_times.append(execution_times_run.mean())

        width = 1
        offset = (i - (len(solvers) - 1) / 2) * width
        plt.figure(1)
        plt.bar(n_mobile_devices + offset, latencies, width=width, label=s)
        plt.figure(2)
        plt.bar(n_mobile_devices + offset, energies, width=width, label=s)
        plt.figure(3)
        plt.bar(n_mobile_devices + offset, latencies_ok, width=width, label=s)
        plt.figure(4)
        plt.plot(n_mobile_devices, execution_times, '-o', label=s)

    save_path = Path('/') / 'home' / 'erugfra' / 'Desktop' / 'results'
    save_path.mkdir(parents=True, exist_ok=True)

    _prepare_and_save_figure(1, n_mobile_devices, 'latency (s)', save_path / 'latency.png')
    _prepare_and_save_figure(2, n_mobile_devices, 'energy consumption (mJ)', save_path / 'energy.png')
    _prepare_and_save_figure(3, n_mobile_devices, 'tasks satisfying max latency (%)', save_path / 'latency_ok.png')
    _prepare_and_save_figure(4, n_mobile_devices, 'execution time (s)', save_path / 'execution_time.png')
    plt.show()


if __name__ == '__main__':
    compare(
        solvers=['game_theory', 'naive_local', 'naive_edge'],
        min_mobile_devices=5,
        max_mobile_devices=50,
        step_mobile_devices=5,
        n_runs=50
    )
