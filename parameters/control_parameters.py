import sys
sys.path.append('..')
import numpy as np
import control
from dynamics.quad_dynamics import quad_dynamics
from state_model.compute_models import compute_ss_model
import parameters.quadrotor_parameters as QUAD
import parameters.simulation_parameters as SIM

# --------------------------------------------------------------------------
# ------------------------ LQR PARAMETERS ----------------------------------
# --------------------------------------------------------------------------
quad = quad_dynamics(SIM.ts_simulation)

trim_state = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
trim_input = np.array([QUAD.mass*QUAD.gravity, 0, 0, 0])

A, B = compute_ss_model(quad, trim_state, trim_input)
C = np.eye(12)

tuning_type = 0     # 0 = balanced, 1 = minimal state, 2 = minimal input

# BALANCED
if tuning_type == 0:
    eN_max = 0.2
    eE_max = 0.2
    eD_max = 0.2
    u_max = 0.1
    v_max = 0.1
    w_max = 0.1
    phi_max = np.deg2rad(0.1)
    theta_max = np.deg2rad(0.1)
    psi_max = np.deg2rad(0.1)
    p_max = np.deg2rad(0.1)
    q_max = np.deg2rad(0.1)
    r_max = np.deg2rad(0.1)

if tuning_type == 1:
    # MINIMAL STATE
    eN_max = 0.1
    eE_max = 0.1
    eD_max = 0.1
    u_max = 0.05
    v_max = 0.05
    w_max = 0.05
    phi_max = np.deg2rad(0.25)
    theta_max = np.deg2rad(0.25)
    psi_max = np.deg2rad(0.25)
    p_max = np.deg2rad(0.1)
    q_max = np.deg2rad(0.1)
    r_max = np.deg2rad(0.1)

if tuning_type == 2:
    # MINIMAL INPUT
    eN_max = 0.2
    eE_max = 0.2
    eD_max = 0.2
    u_max = 0.1
    v_max = 0.1
    w_max = 0.1
    phi_max = np.deg2rad(0.1)
    theta_max = np.deg2rad(0.1)
    psi_max = np.deg2rad(0.1)
    p_max = np.deg2rad(0.1)
    q_max = np.deg2rad(0.1)
    r_max = np.deg2rad(0.1)

Q = np.diag((1.0/eN_max**2,  1.0/eE_max**2,    1.0/eD_max**2,
             1.0/u_max**2,   1.0/v_max**2,     1.0/w_max**2,
             1.0/phi_max**2, 1.0/theta_max**2, 1.0/psi_max**2,
             1.0/p_max**2,   1.0/q_max**2,     1.0/r_max**2))

if tuning_type == 0:
    # BALANCED
    f_max = 10
    tau_x_max = 0.15
    tau_y_max = 0.15
    tau_z_max = 0.15

if tuning_type == 1:
    # MINIMAL STATE
    f_max = 10
    tau_x_max = 0.15
    tau_y_max = 0.15
    tau_z_max = 0.15

if tuning_type == 2:
    # MINIMAL INPUT
    f_max = 0.1
    tau_x_max = 0.005
    tau_y_max = 0.005
    tau_z_max = 0.005

R = np.diag((1.0/f_max**2, 1.0/tau_x_max**2, 1.0/tau_y_max**2, 1.0/tau_z_max**2))

# +f, -f, +tau_x, -tau_x, +tau_y, -tau_y, +tau_z, -tau_z
# limit_lqr = np.array([[200], [0], [25.0], [-25.0], [25.0], [-25.0], [25.0], [-25.0]])
limit_lqr = np.array([[np.inf], [0], [np.inf], [-np.inf], [np.inf], [-np.inf], [np.inf], [-np.inf]])
