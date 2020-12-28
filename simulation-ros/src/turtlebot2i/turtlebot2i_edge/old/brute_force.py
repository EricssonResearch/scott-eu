from __future__ import print_function
from __future__ import division
import itertools as it
import numpy as np
from tqdm import tqdm
from solver import Solver


class BruteForceSolver(Solver):
    def __init__(self, problem):
        Solver.__init__(self, problem)

    def solve(self, **kwargs):
        offloading_decision = [False, True]
        combinations = it.product(*[offloading_decision for _ in range(self._n_mobile_devices)])
        combinations = np.array(list(combinations))

        best_latency_ok = None
        best_energy = None
        best_solution = None

        for solution in tqdm(combinations, desc='Combinations'):
            # constraint C2: number of wireless channels
            if solution.sum() > self.network.n_channels:
                continue

            # evaluate solution
            latencies, energies, latencies_ok = self.evaluate(solution)

            # constraint C1: max permissible latency
            latency_ok = latencies_ok.sum()
            if best_latency_ok is not None and latency_ok < best_latency_ok:
                continue

            # update best solution
            energy = energies.sum()
            if best_energy is None or energy < best_energy:
                best_latency_ok = latency_ok
                best_energy = energy
                best_solution = solution

        if best_solution is None:
            raise ValueError('Problem does not admit solution')
        return best_solution
