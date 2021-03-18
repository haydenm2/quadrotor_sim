import sys
sys.path.append('..')
import numpy as np

from message_types.msg_state import msg_state
import parameters.quadrotor_parameters as QUAD
from tools.tools import RotationVehicle2Body

class quad_dynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        self._state = np.array([[QUAD.pn0],  # (0)
                                [QUAD.pe0],  # (1)
                                [QUAD.pd0],  # (2)
                                [QUAD.u0],  # (3)
                                [QUAD.v0],  # (4)
                                [QUAD.w0],  # (5)
                                [QUAD.phi0],  # (6)
                                [QUAD.theta0],  # (7)
                                [QUAD.psi0],  # (8)
                                [QUAD.p0],  # (9)
                                [QUAD.q0],  # (10)
                                [QUAD.r0]])  # (11)

        # initialize true_state message
        self.msg_true_state = msg_state()
        self._update_msg_true_state()

    def update_state(self, inputs):
        # get forces and moments acting on rigid body
        forces_moments = self._forces_moments(self._state, inputs)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step / 2. * k1, forces_moments)
        k3 = self._derivatives(self._state + time_step / 2. * k2, forces_moments)
        k4 = self._derivatives(self._state + time_step * k3, forces_moments)
        self._state += time_step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        # update the message class for the true state
        self._update_msg_true_state()

    def _derivatives(self, state, forces_moments):
        # extract the states
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)

        phi = state.item(6)
        theta = state.item(7)
        psi = state.item(8)

        p = state.item(9)
        q = state.item(10)
        r = state.item(11)

        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)

        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        # position kinematics
        pn_dot = u
        pe_dot = v
        pd_dot = w

        # position dynamics
        u_dot = fx/QUAD.mass
        v_dot = fy/QUAD.mass
        w_dot = fz/QUAD.mass

        # rotational kinematics
        phi_dot = p + q*np.sin(phi)*np.tan(theta) + r*np.cos(phi)*np.tan(theta)
        theta_dot = q*np.cos(phi) - r*np.sin(phi)
        psi_dot = (q*np.sin(phi) + r*np.cos(phi))/np.cos(theta)

        # rotational dynamics
        p_dot = (QUAD.Jy - QUAD.Jz)/QUAD.Jx*q*r + l/QUAD.Jx
        q_dot = (QUAD.Jz - QUAD.Jx)/QUAD.Jy*p*r + m/QUAD.Jy
        r_dot = (QUAD.Jx - QUAD.Jy)/QUAD.Jz*p*q + n/QUAD.Jz

        # collect the derivative of the states
        x_dot = np.array([[pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot,
                           phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot]]).T

        return x_dot

    def _forces_moments(self, state, inputs):

        # define states
        phi = state.item(6)
        theta = state.item(7)
        psi = state.item(8)
        R_bi = RotationVehicle2Body(phi, theta, psi).T

        # control inputs
        thrust = inputs.item(0)
        tau_x = inputs.item(1)
        tau_y = inputs.item(2)
        tau_z = inputs.item(3)

        # forces calculations
        v_thrust = np.array([[0, 0, -thrust]]).T
        v_gravity = np.array([[0, 0, QUAD.gravity*QUAD.mass]]).T
        f_total = R_bi @ v_thrust + v_gravity

        # moments calculations
        m_body = np.array([[tau_x, tau_y, tau_z]]).T
        m_total = R_bi @ m_body

        return np.array([[f_total.item(0), f_total.item(1), f_total.item(2), m_total.item(0), m_total.item(1), m_total.item(2)]]).T

    def _update_msg_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
        self.msg_true_state.pn = self._state.item(0)
        self.msg_true_state.pe = self._state.item(1)
        self.msg_true_state.pd = self._state.item(2)
        self.msg_true_state.u = self._state.item(3)
        self.msg_true_state.v = self._state.item(4)
        self.msg_true_state.w = self._state.item(5)
        self.msg_true_state.phi = self._state.item(6)
        self.msg_true_state.theta = self._state.item(7)
        self.msg_true_state.psi = self._state.item(8)
        self.msg_true_state.p = self._state.item(9)
        self.msg_true_state.q = self._state.item(10)
        self.msg_true_state.r = self._state.item(11)
