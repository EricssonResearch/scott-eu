from __future__ import print_function
from __future__ import division
import numpy as np
from solver import Solver


class NaiveSolver(Solver):
    def __init__(self, problem, mode):
        Solver.__init__(self, problem)
        self.mode = mode

    def solve(self, **kwargs):
        if self.mode == 'local':
            return np.zeros(self._n_mobile_devices, dtype=bool)
        elif self.mode == 'edge':
            return np.ones(self._n_mobile_devices, dtype=bool)
        else:
            raise ValueError('invalid mode')
