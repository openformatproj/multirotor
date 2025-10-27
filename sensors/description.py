from ml.engine import Part, Port
from ml.data import Number
from constants import X, Y, Z, ROLL, PITCH, YAW
import pybullet

class Sensors(Part):
    """
    A behavioral part that acts as the multirotor's virtual sensor suite.

    This part takes the raw state vectors from the physics simulator (position,
    orientation, linear/angular speeds) and decomposes them into individual sensor
    readings that other components can use. It populates output ports for roll,
    pitch, yaw, and their corresponding speeds, as well as individual coordinate
    positions and speeds.
    """

    def behavior(self):
        """
        Reads raw simulation state and populates sensor output ports.

        This method is scheduled to run on every simulation step where its inputs
        are updated. It converts the quaternion orientation to Euler angles and
        sets the corresponding output ports.
        """
        position = self.get_port('position').get()
        orientation = self.get_port('orientation').get()
        roll, pitch, yaw = pybullet.getEulerFromQuaternion(orientation)
        linear_speed = self.get_port('linear_speed').get()
        angular_speed = self.get_port('angular_speed').get()
        # IMU (linear acceleration)
        self.get_port('roll').set(Number(roll)) # phi
        self.get_port('pitch').set(Number(pitch)) # theta
        # compass
        self.get_port('yaw').set(Number(yaw)) # psi
        # IMU (angular speed)
        self.get_port('roll_speed').set(Number(angular_speed[X.index()]))
        self.get_port('pitch_speed').set(Number(angular_speed[Y.index()]))
        self.get_port('yaw_speed').set(Number(angular_speed[Z.index()]))
        # Ground station
        x = position[X.index()]
        y = position[Y.index()]
        z = position[Z.index()]
        x_speed = linear_speed[X.index()]
        y_speed = linear_speed[Y.index()]
        z_speed = linear_speed[Z.index()]
        self.get_port('x').set(Number(x))
        self.get_port('y').set(Number(y))
        self.get_port('x_speed').set(Number(x_speed))
        self.get_port('y_speed').set(Number(y_speed))
        # altitude sensors
        self.get_port('z').set(Number(z))
        self.get_port('z_speed').set(Number(z_speed))

    def __init__(self, identifier):
        """
        Initializes the Sensors part.

        Args:
            identifier (str): The unique name for this part.
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