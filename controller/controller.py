import sys
import numpy as np
sys.path.append('..')
import parameters.control_parameters as CON
import parameters.quadrotor_parameters as QUAD
# from tools.transfer_function import transfer_function
# from tools.wrap import wrap
from controller.lqr_controller import lqr_control
from message_types.msg_state import msg_state


class controller:
    def __init__(self, ts_control):
        self.control = lqr_control(CON.A, CON.B, CON.Q, CON.R, ts_control)
        self.commanded_state = msg_state()
        self.trim_input = CON.trim_input
        self.inputs = self.trim_input
        self.limit = CON.limit_lqr

    def update(self, cmd, state):
        x = np.array([[state.pn - cmd.pn, state.pe - cmd.pe, state.pd - cmd.pd, state.u, state.v, state.w, state.phi, state.theta, state.psi - cmd.psi, state.p, state.q, state.r]]).T  # using beta as an estimate for v
        u_tilde = self.control.update(x)
        u = u_tilde + self.trim_input.reshape(-1, 1)
        u_sat = self._saturate(u)
        self.inputs = u_sat.reshape(-1, 1)
        # self.commanded_state.pn = cmd.
        return self.inputs, self.commanded_state

    def _saturate(self, u):
        # saturate u at +- self.limit
        u_sat = u
        for i in range(len(u)):
            if u.item(i) >= self.limit.item(2 * i):
                u_sat[i, 0] = self.limit.item(2 * i)
            elif u.item(i) <= self.limit.item(2 * i + 1):
                u_sat[i, 0] = self.limit.item(2 * i + 1)
            else:
                u_sat[i, 0] = u.item(i)
        return u_sat
