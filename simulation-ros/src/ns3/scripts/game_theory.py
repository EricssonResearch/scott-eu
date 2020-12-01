from __future__ import print_function
from __future__ import division
import numpy as np
from solver import Solver


class GameTheoreticSolver(Solver):
    def __init__(self, problem):
        Solver.__init__(self, problem)

    def solve(self, **kwargs):
        solution = np.zeros(self._n_mobile_devices, dtype=bool)

        while True:
            new_solution = np.zeros(self._n_mobile_devices, dtype=bool)

            for i, md in enumerate(self.mobile_devices):
                # local computation
                local_latency, local_energy = self._local_computations[i]

                # edge computation
                local_solution = solution.copy()    # not new_solution, it does not know new decisions of the others!
                local_solution[i] = True
                latencies, energies, _ = self.evaluate(local_solution)
                edge_latency = latencies[i]
                edge_energy = energies[i]

                # constraint C2: number of wireless channels
                if local_solution.sum() > self.network.n_channels:
                    new_solution[i] = False
                    continue

                # optimal decision
                local_overhead = md.lambda_latency * local_latency + md.lambda_energy * local_energy
                edge_overhead = md.lambda_latency * edge_latency + md.lambda_energy * edge_energy
                new_solution[i] = False if local_overhead < edge_overhead else True

            # check convergence
            idx_candidates = np.where(new_solution != solution)[0]
            if len(idx_candidates) == 0:
                break

            # update policy
            idx_winner = np.random.choice(idx_candidates)
            solution[idx_winner] = new_solution[idx_winner]

        return solution
