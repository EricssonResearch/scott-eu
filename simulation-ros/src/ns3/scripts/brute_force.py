from __future__ import print_function
from __future__ import division
import itertools as it
import numpy as np
from tqdm import tqdm
from solver import Solver


class BruteForceSolver(Solver):
    def __init__(self, problem):
        Solver.__init__(self, problem)
        self.lambdas_latency = np.array([md.lambda_latency for md in self.mobile_devices])
        self.lambdas_energy = np.array([md.lambda_energy for md in self.mobile_devices])

    def solve(self, **kwargs):
        offloading_decision = [False, True]
        combinations = it.product(*[offloading_decision for _ in range(self._n_mobile_devices)])
        combinations = np.array(list(combinations))

        best_overhead = None
        best_solution = None

        for solution in tqdm(combinations, desc='Combinations'):
            # constraint about number of wireless channels
            if solution.sum() > self.network.n_channels:
                continue

            # evaluate solution
            latencies, energies, _ = self.evaluate(solution)

            # update best solution
            overhead = (self.lambdas_latency * latencies + self.lambdas_energy * energies).sum()
            if best_overhead is None or overhead < best_overhead:
                best_overhead = overhead
                best_solution = solution

        if best_solution is None:
            raise ValueError('Problem does not admit solution')
        return best_solution
