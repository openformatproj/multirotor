from ml.engine import Part, Port
from ml.data import Number
from ml.strategies import all_input_ports_updated
from constants import X, Y, Z, ROLL, PITCH, YAW

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
        # Get raw tuples/lists from the simulator
        position_raw = self.get_port('position').get()
        orientation_raw = self.get_port('orientation').get()
        linear_speed_raw = self.get_port('linear_speed').get()
        angular_speed_raw = self.get_port('angular_speed').get()

        roll, pitch, yaw = orientation_raw

        # Convert final scalar values to Number objects at the boundary
        # IMU (linear acceleration)
        self.get_port('roll').set(Number(roll)) # phi
        self.get_port('pitch').set(Number(pitch)) # theta
        # compass
        self.get_port('yaw').set(Number(yaw)) # psi
        # IMU (angular speed)
        self.get_port('roll_speed').set(Number(angular_speed_raw[X.index()]))
        self.get_port('pitch_speed').set(Number(angular_speed_raw[Y.index()]))
        self.get_port('yaw_speed').set(Number(angular_speed_raw[Z.index()]))
        # Ground station
        self.get_port('x').set(Number(position_raw[X.index()]))
        self.get_port('y').set(Number(position_raw[Y.index()]))
        self.get_port('x_speed').set(Number(linear_speed_raw[X.index()]))
        self.get_port('y_speed').set(Number(linear_speed_raw[Y.index()]))
        # altitude sensors
        self.get_port('z').set(Number(position_raw[Z.index()]))
        self.get_port('z_speed').set(Number(linear_speed_raw[Z.index()]))

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
            scheduling_condition=all_input_ports_updated
        )