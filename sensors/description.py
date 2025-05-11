from decimal import Decimal
from ml.engine import Part, Port
from ml.parts import Part_Timed
from constants import X, Y, Z, ROLL, PITCH, YAW
import pybullet

class Sensors(Part_Timed):

    def set_plot(self, fig, ax_xy_position, xy_position, line_xy_position, xy_speed, ax_xy_speed, line_xy_speed, ax_z_position, z_position, line_z_position, ax_z_speed, z_speed, line_z_speed):
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

    def behavior(self):
        if all([self.get_port(input).is_updated() for input in self.get_ports(Port.IN)]):
            t = self.get_port('time').get()
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
                if self.counter == 100:
                    self.x_.append(x)
                    self.y_.append(y)
                    self.z_.append(z)
                    self.zeros.append(0.0)
                    self.line_xy_position.set_data(self.x_, self.y_)
                    self.xy_position.set_data([x], [y])
                    self.line_z_position.set_data(self.zeros, self.z_)
                    self.z_position.set_data([0.0], [z])
                    self.x_speed_.append(x_speed)
                    self.y_speed_.append(y_speed)
                    self.z_speed_.append(z_speed)
                    self.line_xy_speed.set_data(self.x_speed_, self.y_speed_)
                    self.xy_speed.set_data([x_speed], [y_speed])
                    self.line_z_speed.set_data(self.zeros, self.z_speed_)
                    self.z_speed.set_data([0.0], [z_speed])
                    self.fig.canvas.draw()
                    self.fig.canvas.flush_events()
                    self.counter = 0
                else:
                    self.counter += 1
                # remove old data from arrays
                if len(self.x_) == 30:
                    for _ in [0,7]:
                        self.x_.pop(0)
                        self.y_.pop(0)
                        self.z_.pop(0)
                        self.x_speed_.pop(0)
                        self.y_speed_.pop(0)
                        self.z_speed_.pop(0)
                        self.zeros.pop(0)

    def __init__(self, identifier, plot):
        ports = [
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
        super().__init__(identifier, Part.NO_ALLOCATION, ports = ports)
        self.plot = plot