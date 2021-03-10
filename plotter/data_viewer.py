from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *

class data_viewer:
    def __init__(self):
        time_window_length=100
        self.plotter = Plotter(plotting_frequency=20, # refresh plot every 100 time steps
                               time_window=time_window_length)  # plot last time_window seconds of data
        # set up the plot window
        # define first row
        pn_plots = PlotboxArgs(plots=['pn', 'pn_e'],
                               labels={'left': 'pn(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        pe_plots = PlotboxArgs(plots=['pe', 'pe_e'],
                               labels={'left': 'pe(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        pd_plots = PlotboxArgs(plots=['pd', 'pd_e'],
                              labels={'left': 'pd(m)', 'bottom': 'Time (s)'},
                              time_window=time_window_length)
        first_row = [pn_plots, pe_plots, pd_plots]

        # define second row
        u_plots = PlotboxArgs(plots=['u', 'u_e'],
                              labels={'left': 'u(m/s)', 'bottom': 'Time (s)'},
                              time_window=time_window_length)
        v_plots = PlotboxArgs(plots=['v', 'v_e'],
                              labels={'left': 'v(m/s)', 'bottom': 'Time (s)'},
                              time_window=time_window_length)
        w_plots = PlotboxArgs(plots=['w', 'w_e'],
                              labels={'left': 'w(m/s)', 'bottom': 'Time (s)'},
                              time_window=time_window_length)
        second_row = [u_plots, v_plots, w_plots]

        # define third row
        phi_plots = PlotboxArgs(plots=['phi', 'phi_e'],
                                labels={'left': 'phi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length)
        theta_plots = PlotboxArgs(plots=['theta', 'theta_e'],
                                  labels={'left': 'theta(deg)', 'bottom': 'Time (s)'},
                                  rad2deg=True,
                                  time_window=time_window_length)
        psi_plots = PlotboxArgs(plots=['psi', 'psi_e'],
                                labels={'left': 'psi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length)
        third_row = [phi_plots, theta_plots, psi_plots]

        # define fourth row
        p_plots = PlotboxArgs(plots=['p', 'p_e'],
                              labels={'left': 'p(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        q_plots = PlotboxArgs(plots=['q', 'q_e'],
                              labels={'left': 'q(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        r_plots = PlotboxArgs(plots=['r', 'r_e'],
                              labels={'left': 'r(deg)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        fourth_row = [p_plots, q_plots, r_plots]
        plots = [first_row,
                 second_row,
                 third_row,
                 fourth_row
                 ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('true_state', ['pn', 'pe', 'pd', 'u', 'v', 'w', 'phi', 'theta', 'psi',
                                                        'p', 'q', 'r'])
        # self.plotter.define_input_vector('estimated_state', ['pn_e', 'pe_e', 'pd_e', 'u_e', 'v_e', 'w_e',
        #                                                      'phi_e', 'theta_e', 'psi_e', 'p_e', 'q_e', 'r_e'])
        self.plotter.define_input_vector('commands', ['pn_c', 'pe_c', 'pd_c', 'u_c', 'v_c', 'w_c',
                                                             'phi_c', 'theta_c', 'psi_c', 'p_c', 'q_c', 'r_c'])
        # plot timer
        self.time = 0.

    def update(self, true_state, commanded_state, ts):
        commands = [commanded_state.pn, # pn_c
                    commanded_state.pe, # pe_c
                    commanded_state.pd, # pd_c
                    commanded_state.u, # u_c
                    commanded_state.v, # v_c
                    commanded_state.w, # w_c
                    commanded_state.phi, # phi_c
                    commanded_state.theta, # theta_c
                    commanded_state.psi, # psi_c
                    commanded_state.p, # p_c
                    commanded_state.q, # q_c
                    commanded_state.r] # r_c
        ## Add the state data in vectors
        # the order has to match the order in lines 72-76
        true_state_list = [true_state.pn, true_state.pe, true_state.pd,
                           true_state.u, true_state.v, true_state.w,
                           true_state.phi, true_state.theta, true_state.psi,
                           true_state.p, true_state.q, true_state.r]
        # estimated_state_list = [estimated_state.pn, estimated_state.pe, estimated_state.pd,
        #                         estimated_state.u, estimated_state.v, estimated_state.w,
        #                         estimated_state.phi, estimated_state.theta, estimated_state.chi,
        #                         estimated_state.p, estimated_state.q, estimated_state.r]
        self.plotter.add_vector_measurement('true_state', true_state_list, self.time)
        # self.plotter.add_vector_measurement('estimated_state', estimated_state_list, self.time)
        self.plotter.add_vector_measurement('commands', commands, self.time)

        # Update and display the plot
        self.plotter.update_plots()

        # increment time
        self.time += ts



