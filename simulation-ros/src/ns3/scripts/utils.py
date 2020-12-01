from brute_force import BruteForceSolver
from game_theory import GameTheoreticSolver
from naive import NaiveSolver


def get_solver(solver, problem):
    if solver == 'brute_force':
        return BruteForceSolver(problem)
    elif solver == 'game_theory':
        return GameTheoreticSolver(problem)
    elif solver == 'naive_local':
        return NaiveSolver(problem, 'local')
    elif solver == 'naive_edge':
        return NaiveSolver(problem, 'edge')
    else:
        raise ValueError('invalid solver: ' + solver)
