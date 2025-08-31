from decimal import Decimal
from ml.engine import Part, Port
from constants import X, Y, Z, ROLL, PITCH, YAW
import pybullet

# Constants for plotting logic
PLOT_UPDATE_FREQUENCY = 100  # Update plot every 100 behavior calls
PLOT_HISTORY_LENGTH = 30     # Keep a history of 30 data points for the trail
PLOT_HISTORY_TRIM_COUNT = 2  # Number of old points to remove when history is full

class Sensors(Part):
    """
    A behavioral part that acts as the multirotor's virtual sensor suite.

    This part takes the raw state vectors from the physics simulator (position,
    orientation, linear/angular speeds) and decomposes them into individual sensor
    readings that other components can use. It populates output ports for roll,
    pitch, yaw, and their corresponding speeds, as well as individual coordinate
    positions and speeds.

    It also contains optional logic for real-time plotting of the vehicle's
    trajectory and speeds using matplotlib.
    """

    def set_plot(self, fig, ax_xy_position, xy_position, line_xy_position, xy_speed, ax_xy_speed, line_xy_speed, ax_z_position, z_position, line_z_position, ax_z_speed, z_speed, line_z_speed):
        """
        Injects matplotlib plot objects for real-time visualization.

        This method is called externally to provide the part with the necessary
        matplotlib figure, axes, and line objects to update during the simulation.
        """
        self.fig = fig
        self.ax_xy_position = ax_xy_position
        self.xy_position = xy_position
        self.line_xy_position = line_xy_position
        self.x_ = []
        self.y_ = []
        self.ax_xy_speed = ax_xy_speed
        self.xy_speed = xy_speed
        self.line_xy_speed = line_xy_speed
        self.x_speed_ = []
        self.y_speed_ = []
        self.ax_z_position = ax_z_position
        self.z_position = z_position
        self.line_z_position = line_z_position
        self.z_ = []
        self.ax_z_speed = ax_z_speed
        self.z_speed = z_speed
        self.line_z_speed = line_z_speed
        self.z_speed_ = []
        self.zeros = []
        self.counter = 0

    def _update_plots(self, x, y, z, x_speed, y_speed, z_speed):
        """Helper method to handle the logic for updating the plots."""
        self.counter += 1
        if self.counter < PLOT_UPDATE_FREQUENCY:
            return

        self.counter = 0  # Reset counter

        # Append new data for the plot trails
        self.x_.append(x)
        self.y_.append(y)
        self.z_.append(z)
        self.zeros.append(0.0)
        self.x_speed_.append(x_speed)
        self.y_speed_.append(y_speed)
        self.z_speed_.append(z_speed)

        # Update plot data
        self.line_xy_position.set_data(self.x_, self.y_)
        self.xy_position.set_data([x], [y])
        self.line_z_position.set_data(self.zeros, self.z_)
        self.z_position.set_data([0.0], [z])
        self.line_xy_speed.set_data(self.x_speed_, self.y_speed_)
        self.xy_speed.set_data([x_speed], [y_speed])
        self.line_z_speed.set_data(self.zeros, self.z_speed_)
        self.z_speed.set_data([0.0], [z_speed])

        # Redraw the canvas
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Trim old data to keep the history from growing indefinitely
        if len(self.x_) >= PLOT_HISTORY_LENGTH:
            for _ in range(PLOT_HISTORY_TRIM_COUNT):
                for data_array in [self.x_, self.y_, self.z_, self.x_speed_, self.y_speed_, self.z_speed_, self.zeros]:
                    data_array.pop(0)

    def behavior(self):
        """
        Reads raw simulation state, populates sensor output ports, and updates plots.

        This method is scheduled to run on every simulation step where its inputs
        are updated. It converts the quaternion orientation to Euler angles and
        sets the corresponding output ports. If plotting is enabled, it will
        periodically update the matplotlib figure.
        """
        position = self.get_port('position').get()
        orientation = self.get_port('orientation').get()
        roll, pitch, yaw = pybullet.getEulerFromQuaternion(orientation)
        linear_speed = self.get_port('linear_speed').get()
        angular_speed = self.get_port('angular_speed').get()
        # IMU (linear acceleration)
        self.get_port('roll').set(Decimal(roll)) # phi
        self.get_port('pitch').set(Decimal(pitch)) # theta
        # compass
        self.get_port('yaw').set(Decimal(yaw)) # psi
        # IMU (angular speed)
        self.get_port('roll_speed').set(Decimal(angular_speed[X.index()]))
        self.get_port('pitch_speed').set(Decimal(angular_speed[Y.index()]))
        self.get_port('yaw_speed').set(Decimal(angular_speed[Z.index()]))
        # Ground station
        x = position[X.index()]
        y = position[Y.index()]
        z = position[Z.index()]
        x_speed = linear_speed[X.index()]
        y_speed = linear_speed[Y.index()]
        z_speed = linear_speed[Z.index()]
        self.get_port('x').set(Decimal(x))
        self.get_port('y').set(Decimal(y))
        self.get_port('x_speed').set(Decimal(x_speed))
        self.get_port('y_speed').set(Decimal(y_speed))
        # altitude sensors
        self.get_port('z').set(Decimal(z))
        self.get_port('z_speed').set(Decimal(z_speed))
        # plot data
        if self.plot:
            self._update_plots(x, y, z, x_speed, y_speed, z_speed)

    def __init__(self, identifier, plot):
        """
        Initializes the Sensors part.

        Args:
            identifier (str): The unique name for this part.
            plot (bool): If True, enables real-time plotting of sensor data.
        """
        ports = [
            Port('time', Port.IN),
            Port('position', Port.IN),
            Port('orientation', Port.IN),
            Port('linear_speed', Port.IN),
            Port('angular_speed', Port.IN)
        ]
        for i in [ROLL, PITCH, YAW]:
            ports.append(Port(i.name(), Port.OUT))
        for i in [ROLL, PITCH, YAW]:
            ports.append(Port(f'{i.name()}_speed', Port.OUT))
        for i in [X, Y, Z]:
            ports.append(Port(i.name(), Port.OUT))
        for i in [X, Y, Z]:
            ports.append(Port(f'{i.name()}_speed', Port.OUT))
        super().__init__(
            identifier=identifier,
            ports=ports,
            # This part should only run when all its inputs are ready.
            scheduling_condition=lambda part: all(p.is_updated() for p in part.get_ports(Port.IN))
        )
        self.plot = plot