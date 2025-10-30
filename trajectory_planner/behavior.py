from ml.engine import Part, Port
from ml.data import Number
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

    def behavior(self):
        """
        Calculates the speed setpoints for each axis.

        This method reads the current time and position, gets the target position
        and feed-forward speed from the provided functions, and calculates the
        output speeds using a proportional control law.
        """
        # Convert inputs to native floats at the boundary for performance
        t_float = float(self.get_port('time').get())
        x_float = float(self.get_port(X.name()).get())
        y_float = float(self.get_port(Y.name()).get())
        z_float = float(self.get_port(Z.name()).get())

        # Get target trajectory values as raw floats
        pos_target = self.set_position(t_float)
        speed_target = self.set_speed(t_float)

        # Calculate speed for each axis using: Kp * (target_pos - current_pos) + target_speed
        x_speed_out = float(self.KP_HORIZONTAL) * (pos_target[X.index()] - x_float) + speed_target[X.index()]
        y_speed_out = float(self.KP_HORIZONTAL) * (pos_target[Y.index()] - y_float) + speed_target[Y.index()]
        z_speed_out = float(self.KP_VERTICAL) * (pos_target[Z.index()] - z_float) + speed_target[Z.index()]

        # Convert final results back to Number objects before setting output ports
        self.get_port(f'{X.name()}_speed').set(Number(x_speed_out))
        self.get_port(f'{Y.name()}_speed').set(Number(y_speed_out))
        self.get_port(f'{Z.name()}_speed').set(Number(z_speed_out))
        self.get_port('yaw_speed').set(Number(0.0))

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
        # Proportional gains for the position controller.
        self.KP_HORIZONTAL = Number('0.7')
        self.KP_VERTICAL = Number('3.0')
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