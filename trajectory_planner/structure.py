from decimal import Decimal
from ml.engine import Part, Port
from ml.strategies import sequential_execution
from ml.parts import Operator
from ml import conf as ml_conf
from constants import X, Y, Z

class Trajectory_Planner(Part):
    """
    A structural part that implements a simple P-controller to generate speed
    setpoints based on position and speed targets.

    This part is a structural decomposition of the same logic found in the
    behavioral Trajectory_Planner. It uses several inner `Operator` parts
    to calculate the control law:
    `speed_out = Kp * (target_pos - current_pos) + target_speed`
    """

    # Proportional gains for the position controller.
    KP_HORIZONTAL = Decimal('0.7')
    KP_VERTICAL = Decimal('3.0')

    def __init__(self, identifier, set_position, set_speed):
        """
        Initializes the Trajectory_Planner structural part.

        Args:
            identifier (str): The unique name for this part.
            set_position (callable): A function that takes time (t) and returns
                                     a target position vector [x, y, z].
            set_speed (callable): A function that takes time (t) and returns
                                  a feed-forward speed vector [vx, vy, vz].
        """
        # Define the external ports for this part
        ports = [
            Port('time', Port.IN),
            Port('yaw', Port.IN),
            Port('yaw_speed', Port.OUT)
        ]
        for i in [X, Y, Z]:
            ports.append(Port(i.name(), Port.IN))
            ports.append(Port(f'{i.name()}_speed', Port.OUT))

        # Define the inner parts that compose the logic
        parts = {}
        for i in [X, Y, Z]:
            # Determine gain for the axis
            kp = self.KP_HORIZONTAL if i in [X, Y] else self.KP_VERTICAL

            # Create parts for this axis's control logic
            parts[f'{i.name()}_set_pos_op'] = Operator(f'{i.name()}_set_pos_op', 1, lambda t, idx=i.index(): Decimal(set_position(t)[idx]))
            parts[f'{i.name()}_set_speed_op'] = Operator(f'{i.name()}_set_speed_op', 1, lambda t, idx=i.index(): Decimal(set_speed(t)[idx]))
            parts[f'{i.name()}_error_op'] = Operator(f'{i.name()}_error_op', 2, lambda target, measure: target - measure)
            parts[f'{i.name()}_p_term_op'] = Operator(f'{i.name()}_p_term_op', 1, lambda error, gain=kp: gain * error)
            parts[f'{i.name()}_sum_op'] = Operator(f'{i.name()}_sum_op', 2, lambda p_term, speed_ff: p_term + speed_ff)

        # Yaw speed is fixed at 0.0
        parts['yaw_speed_op'] = Operator('yaw_speed_op', 1, lambda _: Decimal(0.0))

        super().__init__(
            identifier=identifier,
            ports=ports,
            parts=parts,
            execution_strategy=sequential_execution
        )

        # --- Wire the components together ---
        time_port = self.get_port('time')

        for i in [X, Y, Z]:
            # Get part references for clarity
            set_pos_op = self.get_part(f'{i.name()}_set_pos_op')
            set_speed_op = self.get_part(f'{i.name()}_set_speed_op')
            error_op = self.get_part(f'{i.name()}_error_op')
            p_term_op = self.get_part(f'{i.name()}_p_term_op')
            sum_op = self.get_part(f'{i.name()}_sum_op')

            # The time signal drives the target generators
            self.connect(time_port, set_pos_op.get_port(ml_conf.OPERATOR_IN_PORT_TPL.format(0)))
            self.connect(time_port, set_speed_op.get_port(ml_conf.OPERATOR_IN_PORT_TPL.format(0)))

            # Wire the main data path for the P-controller
            # Error = Target Position - Measured Position
            self.connect(set_pos_op.get_port(ml_conf.OPERATOR_OUT_PORT), error_op.get_port(ml_conf.OPERATOR_IN_PORT_TPL.format(0)))
            self.connect(self.get_port(i.name()), error_op.get_port(ml_conf.OPERATOR_IN_PORT_TPL.format(1)))

            # P-Term = Kp * Error
            self.connect(error_op.get_port(ml_conf.OPERATOR_OUT_PORT), p_term_op.get_port(ml_conf.OPERATOR_IN_PORT_TPL.format(0)))

            # Final Speed = P-Term + Feed-Forward Speed
            self.connect(p_term_op.get_port(ml_conf.OPERATOR_OUT_PORT), sum_op.get_port(ml_conf.OPERATOR_IN_PORT_TPL.format(0)))
            self.connect(set_speed_op.get_port(ml_conf.OPERATOR_OUT_PORT), sum_op.get_port(ml_conf.OPERATOR_IN_PORT_TPL.format(1)))

            # Connect final output to the part's corresponding output port
            self.connect(sum_op.get_port(ml_conf.OPERATOR_OUT_PORT), self.get_port(f'{i.name()}_speed'))

        # Wire the yaw logic
        yaw_op = self.get_part('yaw_speed_op')
        # Use the yaw input port as a trigger to generate the 0.0 output each cycle
        self.connect(self.get_port('yaw'), yaw_op.get_port(ml_conf.OPERATOR_IN_PORT_TPL.format(0)))
        self.connect(yaw_op.get_port(ml_conf.OPERATOR_OUT_PORT), self.get_port('yaw_speed'))