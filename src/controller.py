import numpy as np

class PID:
    def __init__(self, v_ref):
        self.v_ref = v_ref
        self.uPred = np.zeros([1,2])

    def solve(self, x):
        v_ref = self.v_ref
        self.uPred[0, 0] = -0.6 * x[5] - 0.9 * x[3] + \
            np.max([-0.9, np.min([np.random.randn() * 0.25, 0.9])])
        self.uPred[0, 1] = 1.5 * (v_ref - x[0]) + \
            np.max([-0.2, np.min([np.random.randn() * 0.10, 0.2])])