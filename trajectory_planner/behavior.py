from decimal import Decimal
from ml.engine import Part, Port
from constants import X, Y, Z

class Trajectory_Planner(Part):
    """
    A behavioral part that implements a simple P-controller to generate speed
    setpoints based on position and speed targets.

    It calculates the desired speed for the X, Y, and Z axes to guide the
    multirotor towards a target position defined by a `set_position` function,
    while also incorporating a feed-forward speed from a `set_speed` function.
    The yaw speed is currently fixed at zero.

    Ports:
        - `time` (IN): The current simulation time.
        - `x`, `y`, `z` (IN): The current measured position coordinates.
        - `yaw` (IN): The current measured yaw angle.
        - `x_speed`, `y_speed`, `z_speed` (OUT): The calculated speed setpoints.
        - `yaw_speed` (OUT): The calculated yaw speed setpoint.
    """

    # Proportional gains for the position controller.
    KP_HORIZONTAL = Decimal('0.7')
    KP_VERTICAL = Decimal('3.0')

    def behavior(self):
        """
        Calculates the speed setpoints for each axis.

        This method reads the current time and position, gets the target position
        and feed-forward speed from the provided functions, and calculates the
        output speeds using a proportional control law.
        """
        t = self.get_port('time').get()
        x = self.get_port(X.name()).get()
        y = self.get_port(Y.name()).get()
        z = self.get_port(Z.name()).get()

        # Calculate speed for each axis using: Kp * (target_pos - current_pos) + target_speed
        x_speed_out = self.KP_HORIZONTAL * (Decimal(self.set_position(t)[X.index()]) - x) + Decimal(self.set_speed(t)[X.index()])
        y_speed_out = self.KP_HORIZONTAL * (Decimal(self.set_position(t)[Y.index()]) - y) + Decimal(self.set_speed(t)[Y.index()])
        z_speed_out = self.KP_VERTICAL * (Decimal(self.set_position(t)[Z.index()]) - z) + Decimal(self.set_speed(t)[Z.index()])

        self.get_port(f'{X.name()}_speed').set(x_speed_out)
        self.get_port(f'{Y.name()}_speed').set(y_speed_out)
        self.get_port(f'{Z.name()}_speed').set(z_speed_out)
        self.get_port('yaw_speed').set(Decimal(0.0))

    def __init__(self, identifier, set_position, set_speed):
        """
        Initializes the Trajectory_Planner part.

        Args:
            identifier (str): The unique name for this part.
            set_position (callable): A function that takes time (t) and returns
                                     a target position vector [x, y, z].
            set_speed (callable): A function that takes time (t) and returns
                                  a feed-forward speed vector [vx, vy, vz].
        """
        ports = [
            Port('time', Port.IN)
        ]
        for i in [X, Y, Z]:
            ports.append(Port(i.name(), Port.IN))
            ports.append(Port(f'{i.name()}_speed', Port.OUT))
        ports.append(Port('yaw', Port.IN))
        ports.append(Port('yaw_speed', Port.OUT))
        super().__init__(
            identifier=identifier,
            ports=ports,
            # This part should only run when all its inputs are ready.
            scheduling_condition=lambda part: all(p.is_updated() for p in part.get_ports(Port.IN))
        )
        self.set_position = set_position
        self.set_speed = set_speed