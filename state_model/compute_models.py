import sys
sys.path.append('..')
import numpy as np
import parameters.quadrotor_parameters as QUAD


def compute_ss_model(quad, trim_state, trim_input):
    # pn_star = trim_state.item(0)
    # pe_star = trim_state.item(1)
    # pd_star = trim_state.item(2)
    # u_star = trim_state.item(3)
    # v_star = trim_state.item(4)
    # w_star = trim_state.item(5)
    # phi_star = trim_state.item(6)
    # theta_star = trim_state.item(7)
    # p_star = trim_state.item(9)
    # q_star = trim_state.item(10)
    # r_star = trim_state.item(11)
    #
    # f_star = trim_input.item(0)
    # tau_x_star = trim_input.item(1)
    # tau_y_star = trim_input.item(2)
    # tau_z_star = trim_input.item(3)

    # TODO: linearize about actual desired trim state
    # pn, pe, pd, u, v, w, phi, theta, psi, p, q, r
    A = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, -QUAD.gravity, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, QUAD.gravity, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    B = np.array([[0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [-1/QUAD.mass, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 1/QUAD.Jx, 0, 0],
                  [0, 0, 1/QUAD.Jy, 0],
                  [0, 0, 0, 1/QUAD.Jz]])
    return A, B