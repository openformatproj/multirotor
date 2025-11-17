from ml.engine import Port, Part
from ml.data import Number

class Motor(Part):
    """
    A behavioral part representing a single motor.

    This is a simplified model that passes an input angular speed directly to
    its output and calculates the reaction torque based on that speed. The
    reaction torque is the torque exerted by the motor on the frame of the
    multirotor, which is crucial for yaw control.

    Ports:
        - `time` (IN): The current simulation time.
        - `angular_speed_in` (IN): The commanded angular speed for the motor.
        - `angular_speed_out` (OUT): The actual angular speed, passed through from the input.
        - `reaction_torque` (OUT): The calculated reaction torque.
    """

    @staticmethod
    def torque(angular_speed: float) -> float:
        """
        Calculates torque based on a simplified linear model where torque is
        equal to the angular speed.

        Args:
            angular_speed (float): The rotational speed of the motor.

        Returns:
            float: The calculated torque.
        """
        return angular_speed

    def behavior(self):
        """
        Reads the input angular speed, calculates the reaction torque, and sets
        the output ports.

        This method is scheduled to run only when all input ports have new data.
        """
        angular_speed = self.get_port('angular_speed_in').get()
        reaction_torque = -Motor.torque(angular_speed)
        self.get_port('angular_speed_out').set(angular_speed)
        self.get_port('reaction_torque').set(reaction_torque)

    def __init__(self, identifier: str):
        """
        Initializes the Motor part.

        Args:
            identifier (str): The unique name for this part.
        """
        ports = [
            Port('time', Port.IN),
            Port('angular_speed_in', Port.IN),
            Port('angular_speed_out', Port.OUT),
            Port('reaction_torque', Port.OUT),
        ]
        super().__init__(
            identifier=identifier,
            ports=ports,
            # This part should only run when all its inputs are ready.
            scheduling_condition=lambda part: all(p.is_updated() for p in part.get_ports(Port.IN))
        )