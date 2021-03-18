import sys
sys.path.append('..')
import numpy as np
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
# big value implies more cost for variable, low means low cost
eN_max = 0.25
eE_max = 0.25
eD_max = 0.25
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

f_max = 10
tau_x_max = 0.1
tau_y_max = 0.1
tau_z_max = 0.1

R = np.diag((1.0/f_max**2, 1.0/tau_x_max**2, 1.0/tau_y_max**2, 1.0/tau_z_max**2))

# +f, -f, +tau_x, -tau_x, +tau_y, -tau_y, +tau_z, -tau_z
limit_lqr = np.array([[200], [0], [25.0], [-25.0], [25.0], [-25.0], [25.0], [-25.0]])
