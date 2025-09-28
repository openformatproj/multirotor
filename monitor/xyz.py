import sys
from PyQt5.QtWidgets import QApplication
import matplotlib.pyplot as plt
from ml.engine import Part, Port

class XYZ_Monitor(Part):
    """
    A self-contained Part that handles all XYZ monitoring and plotting logic.
    It is designed to be executed in the main application thread, consuming
    data from its input ports which are fed by the simulation thread.
    """
    def __init__(self, identifier: str, position_bounds, speed_bounds, plot_decimation, history_length=30, history_trim=2):
        self.position_bounds = position_bounds
        self.speed_bounds = speed_bounds
        self.history_length = history_length
        self.history_trim = history_trim
        self.plot_decimation = plot_decimation

        ports = [Port(name, Port.IN) for name in ['x', 'y', 'z', 'x_speed', 'y_speed', 'z_speed']]
        # The scheduling condition ensures this part only runs when all its inputs are updated.
        super().__init__(
            identifier=identifier,
            ports=ports,
            scheduling_condition=lambda part: all(p.is_updated() for p in part.get_ports(Port.IN))
        )

        # Plotting attributes
        self.app = None
        self.fig = None
        self.plot_elements = {}
        self.plot_history = {
            'x': [], 'y': [], 'z': [], 'zeros': [],
            'x_speed': [], 'y_speed': [], 'z_speed': []
        }
        self._is_setup = False
        self._plot_counter = 0

    def setup(self):
        """Creates and configures the matplotlib figure and axes."""
        self.app = QApplication.instance() or QApplication(sys.argv)
        plt.ion()
        self.fig, ((ax_xy_pos, ax_xy_spd), (ax_z_pos, ax_z_spd)) = plt.subplots(2, 2)
        
        elements = {
            'xy_pos_marker': ax_xy_pos.plot([], [], 'ro')[0], 'xy_pos_line': ax_xy_pos.plot([], [], '-')[0],
            'xy_spd_marker': ax_xy_spd.plot([], [], 'ro')[0], 'xy_spd_line': ax_xy_spd.plot([], [], '-')[0],
            'z_pos_marker': ax_z_pos.plot([], [], 'ro')[0], 'z_pos_line': ax_z_pos.plot([], [], '-')[0],
            'z_spd_marker': ax_z_spd.plot([], [], 'ro')[0], 'z_spd_line': ax_z_spd.plot([], [], '-')[0],
        }
        self.plot_elements = elements

        ax_xy_pos.set_title("XY position"); ax_xy_pos.set_xlim(*self.position_bounds[0]); ax_xy_pos.set_ylim(*self.position_bounds[1]); ax_xy_pos.set_aspect('equal', adjustable='box')
        ax_xy_spd.set_title("XY speed"); ax_xy_spd.set_xlim(*self.speed_bounds[0]); ax_xy_spd.set_ylim(*self.speed_bounds[1]); ax_xy_spd.set_aspect('equal', adjustable='box')
        ax_z_pos.set_title("Z position"); ax_z_pos.set_xlim(*self.position_bounds[0]); ax_z_pos.set_ylim(*self.position_bounds[2]); ax_z_pos.set_aspect('equal', adjustable='box')
        ax_z_spd.set_title("Z speed"); ax_z_spd.set_xlim(*self.speed_bounds[0]); ax_z_spd.set_ylim(*self.speed_bounds[2]); ax_z_spd.set_aspect('equal', adjustable='box')

        plt.show(block=False)
        self._is_setup = True

    def behavior(self):
        """The 'behavior' is to update the plot with new data from input ports."""
        # The scheduling condition guarantees all ports are updated, so we can get data directly.
        if not self._is_setup:
            # Lazy setup on first execution
            self.setup()

        # Handle plot decimation internally
        self._plot_counter += 1
        if self.plot_decimation <= 0 or self._plot_counter % self.plot_decimation != 0:
            # Skip this update to reduce plot frequency
            return

        current_data = {p.get_identifier(): p.get() for p in self.get_ports(Port.IN) if p.is_updated()}
        
        for key in ['x', 'y', 'z', 'x_speed', 'y_speed', 'z_speed']:
            self.plot_history[key].append(float(current_data.get(key, 0.0)))
        self.plot_history['zeros'].append(0.0)

        if len(self.plot_history['x']) >= self.history_length:
            for data_array in self.plot_history.values():
                del data_array[:self.history_trim]

        self.plot_elements['xy_pos_line'].set_data(self.plot_history['x'], self.plot_history['y'])
        self.plot_elements['xy_pos_marker'].set_data([current_data.get('x',0)], [current_data.get('y',0)])
        self.plot_elements['z_pos_line'].set_data(self.plot_history['zeros'], self.plot_history['z'])
        self.plot_elements['z_pos_marker'].set_data([0.0], [current_data.get('z',0)])
        self.plot_elements['xy_spd_line'].set_data(self.plot_history['x_speed'], self.plot_history['y_speed'])
        self.plot_elements['xy_spd_marker'].set_data([current_data.get('x_speed',0)], [current_data.get('y_speed',0)])
        self.plot_elements['z_spd_line'].set_data(self.plot_history['zeros'], self.plot_history['z_speed'])
        self.plot_elements['z_spd_marker'].set_data([0.0], [current_data.get('z_speed',0)])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        # Process GUI events as part of the behavior to keep the window responsive.
        if self.app and isinstance(self.app, QApplication):
            self.app.processEvents() # process all pending events

    def teardown(self):
        """Turns off interactive mode and shows the final plot until closed."""
        if self.fig:
            plt.ioff()
            plt.show()