from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *

class data_viewer:
    def __init__(self):
        time_window_length = 100
        self.plotter = Plotter(plotting_frequency=20,  # refresh plot every 20 time steps
                               time_window=time_window_length)  # plot last time_window seconds of data
        self.plotter.use_light_theme()

        # set up the plot window
        # define first row
        pn_plots = PlotboxArgs(plots=['pn', 'pn_c'],
                               labels={'left': 'pn(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length,
                              plot_min_hue=0,
                              plot_max_hue=900,
                              axis_color='k')
        pe_plots = PlotboxArgs(plots=['pe', 'pe_c'],
                               labels={'left': 'pe(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length,
                              plot_min_hue=0,
                              plot_max_hue=900,
                              axis_color='k')
        pd_plots = PlotboxArgs(plots=['pd', 'pd_c'],
                              labels={'left': 'pd(m)', 'bottom': 'Time (s)'},
                              time_window=time_window_length,
                              plot_min_hue=0,
                              plot_max_hue=900,
                              axis_color='k')
        first_row = [pn_plots, pe_plots, pd_plots]

        # define second row
        u_plots = PlotboxArgs(plots=['u', 'u_c'],
                              labels={'left': 'u(m/s)', 'bottom': 'Time (s)'},
                              time_window=time_window_length,
                              plot_min_hue=0,
                              plot_max_hue=900,
                              axis_color='k')
        v_plots = PlotboxArgs(plots=['v', 'v_c'],
                              labels={'left': 'v(m/s)', 'bottom': 'Time (s)'},
                              time_window=time_window_length,
                              plot_min_hue=0,
                              plot_max_hue=900,
                              axis_color='k')
        w_plots = PlotboxArgs(plots=['w', 'w_c'],
                              labels={'left': 'w(m/s)', 'bottom': 'Time (s)'},
                              time_window=time_window_length,
                              plot_min_hue=0,
                              plot_max_hue=900,
                              axis_color='k')
        second_row = [u_plots, v_plots, w_plots]

        # define third row
        phi_plots = PlotboxArgs(plots=['phi', 'phi_c'],
                                labels={'left': 'phi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length,
                              plot_min_hue=0,
                              plot_max_hue=900,
                              axis_color='k')
        theta_plots = PlotboxArgs(plots=['theta', 'theta_c'],
                                  labels={'left': 'theta(deg)', 'bottom': 'Time (s)'},
                                  rad2deg=True,
                                  time_window=time_window_length,
                              plot_min_hue=0,
                              plot_max_hue=900,
                              axis_color='k')
        psi_plots = PlotboxArgs(plots=['psi', 'psi_c'],
                                labels={'left': 'psi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length,
                              plot_min_hue=0,
                              plot_max_hue=900,
                              axis_color='k')
        third_row = [phi_plots, theta_plots, psi_plots]

        # define fourth row
        p_plots = PlotboxArgs(plots=['p', 'p_c'],
                              labels={'left': 'p(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length,
                              plot_min_hue=0,
                              plot_max_hue=900,
                              axis_color='k')
        q_plots = PlotboxArgs(plots=['q', 'q_c'],
                              labels={'left': 'q(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length,
                              plot_min_hue=0,
                              plot_max_hue=900,
                              axis_color='k')
        r_plots = PlotboxArgs(plots=['r', 'r_c'],
                              labels={'left': 'r(deg)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length,
                              plot_min_hue=0,
                              plot_max_hue=900,
                              axis_color='k')
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
        self.plotter.define_input_vector('estimated_state', ['pn_e', 'pe_e', 'pd_e', 'u_e', 'v_e', 'w_e',
                                                             'phi_e', 'theta_e', 'psi_e', 'p_e', 'q_e', 'r_e'])
        self.plotter.define_input_vector('commands', ['pn_c', 'pe_c', 'pd_c', 'u_c', 'v_c', 'w_c',
                                                             'phi_c', 'theta_c', 'psi_c', 'p_c', 'q_c', 'r_c'])
        # plot timer
        self.time = 0.

    def update(self, true_state, estimated_state, commanded_state, ts):
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
        estimated_state_list = [estimated_state.pn, estimated_state.pe, estimated_state.pd,
                                estimated_state.u, estimated_state.v, estimated_state.w,
                                estimated_state.phi, estimated_state.theta, estimated_state.psi,
                                estimated_state.p, estimated_state.q, estimated_state.r]
        self.plotter.add_vector_measurement('true_state', true_state_list, self.time)
        self.plotter.add_vector_measurement('estimated_state', estimated_state_list, self.time)
        self.plotter.add_vector_measurement('commands', commands, self.time)

        # Update and display the plot
        self.plotter.update_plots()

        # increment time
        self.time += ts



