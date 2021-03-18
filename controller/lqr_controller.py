import sys
import numpy as np
import scipy.linalg as scp
sys.path.append('..')


class lqr_control:
    def __init__(self, A, B, Q, R, Ts):
        self.Ts = Ts
        self.Q = Q
        self.R = R
        self.A = A
        self.B = B
        self.P = scp.solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K_lqr = np.linalg.inv(self.R) @ self.B.T @ self.P
        self.init = True

    def update(self, x):
        u = -self.K_lqr @ x
        return u