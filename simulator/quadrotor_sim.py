import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.quadrotor_parameters as QUAD
import time

from plotter.data_viewer import data_viewer
# from controller.autopilot import autopilot
from dynamics.quad_dynamics import quad_dynamics
from viewer.quad_viewer import quad_viewer
from message_types import msg_state

# initialize the visualization
quad_viewer = quad_viewer()  # initialize the viewer
data_view = data_viewer()  # initialize view of data plots

# initialize elements of the architecture
quad = quad_dynamics(SIM.ts_simulation)
# ctrl = autopilot(SIM.ts_simulation)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
t0 = time.time()
while sim_time < SIM.end_time:
    #-------observer-------------
    desired_state = msg_state
    desired_state.pn = 0
    desired_state.pe = 0
    desired_state.pd = -20
    desired_state.u = 0
    desired_state.v = 0
    desired_state.w = 0
    desired_state.phi = 0
    desired_state.theta = 0
    desired_state.psi = 0
    desired_state.p = 0
    desired_state.q = 0
    desired_state.r = 0

    #-------controller-------------
    # delta, commanded_state = ctrl.update(desired_state, estimated_state)
    commanded_state = msg_state
    commanded_state.pn = 0
    commanded_state.pe = 0
    commanded_state.pd = 0
    commanded_state.u = 0
    commanded_state.v = 0
    commanded_state.w = 0
    commanded_state.phi = 0
    commanded_state.theta = 0
    commanded_state.psi = 0
    commanded_state.p = 0
    commanded_state.q = 0
    commanded_state.r = 0

    #-------physical system-------------
    inputs = np.array([QUAD.mass*QUAD.gravity, 0, 0, 0])
    quad.update_state(inputs)  # propagate the MAV dynamics

    #-------update viewer-------------
    quad_viewer.update(quad.msg_true_state)  # plot path and MAV
    data_view.update(quad.msg_true_state,  # true states
                     commanded_state,  # commanded states
                     SIM.ts_simulation)

    #-------increment time-------------
    sim_time += SIM.ts_simulation
    if SIM.real_time:
        real_time = time.time() - t0
        if real_time < sim_time:
            time.sleep(sim_time - real_time)
