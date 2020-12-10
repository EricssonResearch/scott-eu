from __future__ import print_function
from problem import Problem
from utils import get_solver


def solve(solver, n_mobile_devices, lambda_latency, lambda_energy):
    problem = Problem(n_mobile_devices=n_mobile_devices, seed=1)
    solver = get_solver(solver, problem)
    solution = solver.solve(lambda_latency=lambda_latency, lambda_energy=lambda_energy)
    latencies, energies, latencies_ok = solver.evaluate(solution)

    print('Offloading scheme: {}'.format(solution))
    print('Total latency: {} s'.format(latencies.sum()))
    print('Total energy consumption: {} mJ'.format(energies.sum() * 1e3))
    print('Tasks satisfying latency constraint: {}%'.format(latencies_ok.sum() / len(latencies_ok) * 100))


if __name__ == '__main__':
    solve(
        solver='brute_force',
        n_mobile_devices=10,
        lambda_latency=0.5,
        lambda_energy=0.5
    )
