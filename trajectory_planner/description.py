from ml.engine import Part, Port
from ml.strategies import Execution, all_input_ports_updated
from ml.data import Number
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

    def __init__(self, identifier, conf: object):
        """
        Initializes the Trajectory_Planner structural part.

        Args:
            identifier (str): The unique name for this part.
            conf: The simulation configuration object.
        """
        # Proportional gains for the position controller.
        self.KP_HORIZONTAL = Number('0.7')
        self.KP_VERTICAL = Number('3.0')

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
            # The lambda functions now return floats. The conversion to Decimal happens inside the Operator's
            # behavior, which runs in the correctly configured thread. By converting from a string, we
            # ensure the current thread's precision context is used.
            pos_lambda = lambda t, idx=i.index(): conf.SET_POSITION(float(t))[idx]
            speed_lambda = lambda t, idx=i.index(): conf.SET_SPEED(float(t))[idx]
            
            # Lambdas now perform calculations on native types and wrap the final result.
            # This avoids creating intermediate Number objects.
            parts[f'{i.name()}_set_pos_op'] = Operator(f'{i.name()}_set_pos_op', 1, 
                lambda t, op=pos_lambda: Number(op(t)))
            parts[f'{i.name()}_set_speed_op'] = Operator(f'{i.name()}_set_speed_op', 1, 
                lambda t, op=speed_lambda: Number(op(t)))
            parts[f'{i.name()}_error_op'] = Operator(f'{i.name()}_error_op', 2, 
                lambda target, measure: Number(float(target) - float(measure)))
            parts[f'{i.name()}_p_term_op'] = Operator(f'{i.name()}_p_term_op', 1, 
                lambda error, gain=kp: Number(float(gain) * float(error)))
            parts[f'{i.name()}_sum_op'] = Operator(f'{i.name()}_sum_op', 2, 
                lambda p_term, speed_ff: Number(float(p_term) + float(speed_ff)))

        # Yaw speed is fixed at 0.0
        parts['yaw_speed_op'] = Operator('yaw_speed_op', 1, lambda _: Number('0.0'))

        super().__init__(
            identifier=identifier,
            ports=ports,
            parts=parts,
            execution_strategy=Execution.sequential(),
            scheduling_condition=all_input_ports_updated,
            conf=conf
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