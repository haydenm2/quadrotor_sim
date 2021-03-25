from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *

class input_viewer:
    def __init__(self):
        time_window_length = 100
        self.plotter = Plotter(plotting_frequency=20,  # refresh plot every 20 time steps
                               time_window=time_window_length)  # plot last time_window seconds of data
        light_theme = True

        if light_theme:
            self.plotter.use_light_theme()
            axis_color = 'k'
            min_hue = 0 #260
            max_hue = 900 #500
        else:
            axis_color = 'w'
            min_hue = 0
            max_hue = 900

        # set up the plot window
        # define first row

        ft_plots = PlotboxArgs(plots=['ft', 'ft_z'],
                               labels={'left': 'thrust', 'bottom': 'Time (s)'},
                               time_window=time_window_length,
                              plot_min_hue=min_hue,
                              plot_max_hue=max_hue,
                              axis_color=axis_color)
        first_row = [ft_plots]

        # define second row
        tau_x_plots = PlotboxArgs(plots=['tau_x', 'tau_x_z'],
                              labels={'left': 'tau_x', 'bottom': 'Time (s)'},
                              time_window=time_window_length,
                              plot_min_hue=min_hue,
                              plot_max_hue=max_hue,
                              axis_color=axis_color)
        second_row = [tau_x_plots]

        # define third row
        tau_y_plots = PlotboxArgs(plots=['tau_y', 'tau_y_z'],
                                labels={'left': 'tau_y', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length,
                              plot_min_hue=min_hue,
                              plot_max_hue=max_hue,
                              axis_color=axis_color)
        third_row = [tau_y_plots]

        # define fourth row
        tau_z_plots = PlotboxArgs(plots=['tau_z', 'tau_z_z'],
                              labels={'left': 'tau_z', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length,
                              plot_min_hue=min_hue,
                              plot_max_hue=max_hue,
                              axis_color=axis_color)
        fourth_row = [tau_z_plots]
        plots = [first_row,
                 second_row,
                 third_row,
                 fourth_row]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('true_state', ['ft', 'tau_x', 'tau_y', 'tau_z'])
        self.plotter.define_input_vector('commands', ['ft_z', 'tau_x_z', 'tau_y_z', 'tau_z_z'])
        # plot timer
        self.time = 0.

    def update(self, true_state, commanded_state, ts):
        ## Add the state data in vectors
        # the order has to match the order in lines 72-76
        true_state_list = [true_state.item(0), true_state.item(1), true_state.item(2), true_state.item(3)]
        commanded_state_list = [commanded_state.item(0), commanded_state.item(1), commanded_state.item(2), commanded_state.item(3)]
        self.plotter.add_vector_measurement('true_state', true_state_list, self.time)
        self.plotter.add_vector_measurement('commands', commanded_state_list, self.time)

        # Update and display the plot
        self.plotter.update_plots()

        # increment time
        self.time += ts



