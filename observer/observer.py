import sys
import numpy as np
import control
sys.path.append('..')
import parameters.control_parameters as CTRL
import parameters.sensor_parameters as SENSOR
import parameters.simulation_parameters as SIM
from tools.tools import RotationVehicle2Body
from tools.wrap import wrap
import parameters.quadrotor_parameters as QUAD
from scipy import stats

from message_types.msg_state import msg_state

class observer:
    def __init__(self):
        self.estimated_state = msg_state()
        xhat0 = SENSOR.xhat_0
        self.estimated_state.pn = xhat0.item(0)
        self.estimated_state.pe = xhat0.item(1)
        self.estimated_state.pd = xhat0.item(2)
        self.estimated_state.u = xhat0.item(3)
        self.estimated_state.v = xhat0.item(4)
        self.estimated_state.w = xhat0.item(5)
        self.estimated_state.phi = xhat0.item(6)
        self.estimated_state.theta = xhat0.item(7)
        self.estimated_state.psi = xhat0.item(8)
        self.estimated_state.p = xhat0.item(9)
        self.estimated_state.q = xhat0.item(10)
        self.estimated_state.r = xhat0.item(11)
        self.kalman = kalman_filter(SIM.ts_simulation)

    def update(self, measurements, inputs):
        self.kalman.update(self.estimated_state, measurements, inputs)
        return self.estimated_state

class kalman_filter:
    def __init__(self, Ts):
        self.A = CTRL.A
        self.B = CTRL.B
        self.C = CTRL.C
        self.L, _, _ = control.lqe(CTRL.A, np.eye(12), CTRL.C, SENSOR.Q_N, SENSOR.R_N)
        self.Ts = Ts

    def update(self, estimated_state, measurement, inputs):
        xhat = np.array((estimated_state.pn, estimated_state.pe, estimated_state.pd, estimated_state.u, estimated_state.v, estimated_state.w, estimated_state.phi, estimated_state.theta, estimated_state.psi, estimated_state.p, estimated_state.q, estimated_state.r)).reshape(-1, 1)
        y = np.array((measurement.pn, measurement.pe, measurement.pd, measurement.u, measurement.v, measurement.w, measurement.phi, measurement.theta, measurement.psi, measurement.p, measurement.q, measurement.r)).reshape(-1, 1)
        u = inputs.reshape(-1, 1)
        xhat_k = self.propogate(xhat, u, y)
        estimated_state.pn = xhat_k.item(0)
        estimated_state.pe = xhat_k.item(1)
        estimated_state.pd = xhat_k.item(2)
        estimated_state.u = xhat_k.item(3)
        estimated_state.v = xhat_k.item(4)
        estimated_state.w = xhat_k.item(5)
        estimated_state.phi = xhat_k.item(6)
        estimated_state.theta = xhat_k.item(7)
        estimated_state.psi = xhat_k.item(8)
        estimated_state.p = xhat_k.item(9)
        estimated_state.q = xhat_k.item(10)
        estimated_state.r = xhat_k.item(11)

    def propogate(self, xhat, u, y):
        time_step = self.Ts
        k1 = self.derivatives(xhat, u, y)
        k2 = self.derivatives(xhat + time_step / 2. * k1, u, y)
        k3 = self.derivatives(xhat + time_step / 2. * k2, u, y)
        k4 = self.derivatives(xhat + time_step * k3, u, y)
        xhat = xhat + time_step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return xhat

    def derivatives(self, xhat, u, y):
        return self.A@xhat + self.B@u + self.L@(y - self.C@xhat)
