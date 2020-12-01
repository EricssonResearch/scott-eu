import numpy as np
from abc import ABCMeta, abstractmethod
from problem import Problem


class Solver:
    __metaclass__ = ABCMeta

    def __init__(self, problem):
        assert isinstance(problem, Problem)
        self.network = problem.network
        self.mobile_devices = problem.mobile_devices
        self.mec_server = problem.mec_server
        self._n_mobile_devices = len(self.mobile_devices)
        self._local_computations = np.array([md.compute_task() for md in self.mobile_devices])
        self._max_latencies = np.array([md.task.max_latency for md in self.mobile_devices])

    def _offload(self, idx_edge):
        mobile_devices = self.mobile_devices[idx_edge]
        network_latencies = self.network.transmit(mobile_devices)
        mec_latencies = np.array([self.mec_server.compute_task(md.task) for md in mobile_devices])
        transmit_powers = np.array([md.transmit_power for md in mobile_devices])
        latencies = network_latencies + mec_latencies
        energies = latencies * transmit_powers
        return np.array(list(zip(latencies, energies)))

    @abstractmethod
    def solve(self):
        raise NotImplementedError

    def evaluate(self, solution):
        latencies = np.zeros(self._n_mobile_devices)
        energies = np.zeros(self._n_mobile_devices)

        # local computations
        idx_local = np.where(np.logical_not(solution))[0]
        latencies[idx_local] = self._local_computations[idx_local, 0]
        energies[idx_local] = self._local_computations[idx_local, 1]

        # edge computations
        idx_edge = np.where(solution)[0]
        if len(idx_edge) > 0:
            edge_computations = self._offload(idx_edge)
            latencies[idx_edge] = edge_computations[:, 0]
            energies[idx_edge] = edge_computations[:, 1]

        # satisfying constraint C1
        c1_ok = np.where(latencies <= self._max_latencies, True, False)

        return latencies, energies, c1_ok
