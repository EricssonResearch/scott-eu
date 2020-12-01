from __future__ import print_function
import argparse
from problem import Problem
from brute_force import BruteForceSolver
from game_theory import GameTheoreticSolver


def solve(solver):
    problem = Problem(seed=1)

    if solver == 'brute_force':
        solver = BruteForceSolver(problem)
    elif solver == 'game_theory':
        solver = GameTheoreticSolver(problem)
    else:
        raise ValueError('invalid solver: ' + solver)

    solution = solver.solve()
    latencies, energies, c1_ok = solver.evaluate(solution)
    print('Offloading scheme: {}'.format(solution))
    print('Latency: {} s'.format(latencies.sum()))
    print('Energy consumption: {} mJ'.format(energies.sum() * 1e3))
    print('Satisfying latency constraint: {}%'.format(c1_ok.sum() / len(latencies) * 100))


def get_command_line_arguments():
    parser = argparse.ArgumentParser(description='Solve MEC Collaborative Computation Offloading')
    parser.add_argument('solver', type=str, help='solution to use. Supported: brute_force, game_theory')
    return parser.parse_args()


if __name__ == '__main__':
    args = get_command_line_arguments()
    solve(args.solver)
