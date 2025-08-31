from ml.engine import Port, Part

class Propeller(Part):
    """
    A behavioral part representing a single propeller.

    It calculates the thrust generated based on its angular speed and rotational
    direction (handedness). This simplified model assumes thrust is directly
    proportional to the angular speed.

    Ports:
        - `time` (IN): The current simulation time.
        - `angular_speed` (IN): The rotational speed of the propeller.
        - `thrust` (OUT): The calculated thrust value.
    """

    # By convention, positive angular speed is Counter-Clockwise (CCW).
    RIGHT_HANDED = 0  # Produces positive (upward) thrust with positive (CCW) rotation.
    LEFT_HANDED = 1   # Produces positive (upward) thrust with negative (CW) rotation.

    @staticmethod
    def thrust(angular_speed: float, direction: int) -> float:
        """
        Calculates thrust based on a simplified linear model.

        Args:
            angular_speed (float): The rotational speed. Sign indicates direction.
            direction (int): The propeller's handedness, either Propeller.RIGHT_HANDED
                             or Propeller.LEFT_HANDED.

        Returns:
            float: The calculated thrust.
        """
        if direction == Propeller.RIGHT_HANDED:
            return angular_speed
        else:
            return -angular_speed

    def behavior(self):
        """
        Reads the angular speed and sets the calculated thrust on the output port.

        This method is scheduled to run only when all input ports have new data.
        """
        thrust = Propeller.thrust(self.get_port('angular_speed').get(), self.direction)
        self.get_port('thrust').set(thrust)

    def __init__(self, identifier: str, direction: int):
        """
        Initializes the Propeller part.

        Args:
            identifier (str): The unique name for this part.
            direction (int): The rotational direction, either Propeller.RIGHT_HANDED
                             or Propeller.LEFT_HANDED.
        """
        ports = [
            Port('time', Port.IN),
            Port('angular_speed', Port.IN),
            Port('thrust', Port.OUT),
        ]
        super().__init__(
            identifier=identifier,
            ports=ports,
            # This part should only run when all its inputs are ready.
            scheduling_condition=lambda part: all(p.is_updated() for p in part.get_ports(Port.IN))
        )
        self.direction = direction