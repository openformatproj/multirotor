from ml.engine import Part, Port, EventQueue
from ml.strategies import Execution, all_input_ports_updated, time_updated
from ml.parts import EventToDataSynchronizer

from ml import conf as ml_conf

from physics_engines import get_physics_engine, ERR_ENGINE_NOT_CONNECTED
from propeller.description import Propeller
from motor.description import Motor
from sensors.description import Sensors
from trajectory_planner.description import Trajectory_Planner
from controller.description import Controller
from monitor.xyz import XYZ_Monitor
from constants import X, Y, Z

# --- Component Identifiers ---
SENSORS_ID = 'sensors'
TRAJECTORY_PLANNER_ID = 'trajectory_planner'
CONTROLLER_ID = 'controller'
PLOTTER_ID = 'plotter'
MOTOR_ID_TPL = 'motor_{}'
PROPELLER_ID_TPL = 'propeller_{}'
SIMULATOR_ID = 'simulator'
MULTIROTOR_ID = 'multirotor'
TIME_DIST_ID = 'time_dist'

# --- Port Identifiers ---
TIME_PORT = 'time'
POSITION_PORT = 'position'
ORIENTATION_PORT = 'orientation'
LINEAR_SPEED_PORT = 'linear_speed'
ANGULAR_SPEED_PORT = 'angular_speed'
THRUST_PORT_TPL = 'thrust_{}'
TORQUE_PORT_TPL = 'torque_{}'
ROLL_PORT = 'roll'
PITCH_PORT = 'pitch'
YAW_PORT = 'yaw'
ROLL_SPEED_PORT = 'roll_speed'
PITCH_SPEED_PORT = 'pitch_speed'
YAW_SPEED_PORT = 'yaw_speed'
SPEED_PORT_TPL = '{}_speed'
SPEED_SETPOINT_PORT_TPL = '{}_speed_setpoint'
YAW_SPEED_SETPOINT_PORT = 'yaw_speed_setpoint'
ANGULAR_SPEED_PORT_TPL = 'angular_speed_{}'
ANGULAR_SPEED_IN_PORT = 'angular_speed_in'
ANGULAR_SPEED_OUT_PORT = 'angular_speed_out'
REACTION_TORQUE_PORT = 'reaction_torque'
MULTIROTOR_POSITION_PORT = 'multirotor_position'
MULTIROTOR_ORIENTATION_PORT = 'multirotor_orientation'
MULTIROTOR_LINEAR_SPEED_PORT = 'multirotor_linear_speed'
MULTIROTOR_ANGULAR_SPEED_PORT = 'multirotor_angular_speed'
MULTIROTOR_THRUST_PORT_TPL = 'multirotor_thrust_{}'
MULTIROTOR_TORQUE_PORT_TPL = 'multirotor_torque_{}'
TIME_OUT_PORT = 'time_out'
TIME_EVENT_IN_Q = 'time_event_in'

# --- Other ---
PROPELLERS_INDEXES = lambda conf : range(1, conf.PROPELLERS + 1)

class Multirotor(Part):
    """
    A structural part representing the multirotor vehicle.

    This part composes all the core components of the multirotor, including
    motors, propellers, sensors, a trajectory planner, and a controller. It
    defines the internal wiring that connects these components and exposes
    interfaces for receiving sensor data from the simulator and providing
    actuator outputs (thrust and torque) back to it.
    """
    def __init__(self, identifier: str, conf: object):
        """
        Initializes the Multirotor structural part.

        Args:
            identifier (str): The unique name for this part.
            conf (object): The simulation configuration object.
        """
        conf.propeller_indexes = PROPELLERS_INDEXES(conf)
        ports = [
            Port(TIME_PORT, Port.IN),
            Port(POSITION_PORT, Port.IN),
            Port(ORIENTATION_PORT, Port.IN),
            Port(LINEAR_SPEED_PORT, Port.IN),
            Port(ANGULAR_SPEED_PORT, Port.IN)
        ]
        parts = {
            SENSORS_ID: Sensors(SENSORS_ID, conf),
            TRAJECTORY_PLANNER_ID: Trajectory_Planner(TRAJECTORY_PLANNER_ID, conf),
            CONTROLLER_ID: Controller(CONTROLLER_ID, conf)
        }
        if conf.PLOT:
            parts[PLOTTER_ID] = XYZ_Monitor(PLOTTER_ID, conf)

        for i in PROPELLERS_INDEXES(conf):
            ports.append(Port(THRUST_PORT_TPL.format(i), Port.OUT))
            ports.append(Port(TORQUE_PORT_TPL.format(i), Port.OUT))
            parts[MOTOR_ID_TPL.format(i)] = Motor(MOTOR_ID_TPL.format(i), conf)
            conf.direction = Propeller.LEFT_HANDED if i in [2, 4] else Propeller.RIGHT_HANDED
            parts[PROPELLER_ID_TPL.format(i)] = Propeller(PROPELLER_ID_TPL.format(i), conf)
            
        super().__init__(
            identifier=identifier,
            execution_strategy=Execution.sequential(),
            ports=ports,
            parts=parts,
            scheduling_condition=all_input_ports_updated,
            conf=conf
        )
        
        self.connect(self.get_port(TIME_PORT), self.get_part(SENSORS_ID).get_port(TIME_PORT))
        self.connect(self.get_port(TIME_PORT), self.get_part(TRAJECTORY_PLANNER_ID).get_port(TIME_PORT))
        self.connect(self.get_port(TIME_PORT), self.get_part(CONTROLLER_ID).get_port(TIME_PORT))
        self.connect(self.get_port(POSITION_PORT), self.get_part(SENSORS_ID).get_port(POSITION_PORT))
        self.connect(self.get_port(ORIENTATION_PORT), self.get_part(SENSORS_ID).get_port(ORIENTATION_PORT))
        self.connect(self.get_port(LINEAR_SPEED_PORT), self.get_part(SENSORS_ID).get_port(LINEAR_SPEED_PORT))
        self.connect(self.get_port(ANGULAR_SPEED_PORT), self.get_part(SENSORS_ID).get_port(ANGULAR_SPEED_PORT))
        self.connect(self.get_part(SENSORS_ID).get_port(ROLL_PORT), self.get_part(CONTROLLER_ID).get_port(ROLL_PORT))
        self.connect(self.get_part(SENSORS_ID).get_port(PITCH_PORT), self.get_part(CONTROLLER_ID).get_port(PITCH_PORT))
        self.connect(self.get_part(SENSORS_ID).get_port(YAW_PORT), self.get_part(TRAJECTORY_PLANNER_ID).get_port(YAW_PORT))
        self.connect(self.get_part(SENSORS_ID).get_port(ROLL_SPEED_PORT), self.get_part(CONTROLLER_ID).get_port(ROLL_SPEED_PORT))
        self.connect(self.get_part(SENSORS_ID).get_port(PITCH_SPEED_PORT), self.get_part(CONTROLLER_ID).get_port(PITCH_SPEED_PORT))
        self.connect(self.get_part(SENSORS_ID).get_port(YAW_SPEED_PORT), self.get_part(CONTROLLER_ID).get_port(YAW_SPEED_PORT))
        for i in [X, Y, Z]:
            self.connect(self.get_part(SENSORS_ID).get_port(i.name()), self.get_part(TRAJECTORY_PLANNER_ID).get_port(i.name()))
            self.connect(self.get_part(TRAJECTORY_PLANNER_ID).get_port(SPEED_PORT_TPL.format(i.name())), self.get_part(CONTROLLER_ID).get_port(SPEED_SETPOINT_PORT_TPL.format(i.name())))
            self.connect(self.get_part(SENSORS_ID).get_port(SPEED_PORT_TPL.format(i.name())), self.get_part(CONTROLLER_ID).get_port(SPEED_PORT_TPL.format(i.name())))
        self.connect(self.get_part(TRAJECTORY_PLANNER_ID).get_port(YAW_SPEED_PORT), self.get_part(CONTROLLER_ID).get_port(YAW_SPEED_SETPOINT_PORT))
        for i in PROPELLERS_INDEXES(conf):
            self.connect(self.get_port(TIME_PORT), self.get_part(PROPELLER_ID_TPL.format(i)).get_port(TIME_PORT))
            self.connect(self.get_port(TIME_PORT), self.get_part(MOTOR_ID_TPL.format(i)).get_port(TIME_PORT))
            self.connect(self.get_part(CONTROLLER_ID).get_port(ANGULAR_SPEED_PORT_TPL.format(i)), self.get_part(MOTOR_ID_TPL.format(i)).get_port(ANGULAR_SPEED_IN_PORT))
            self.connect(self.get_part(MOTOR_ID_TPL.format(i)).get_port(ANGULAR_SPEED_OUT_PORT), self.get_part(PROPELLER_ID_TPL.format(i)).get_port(ANGULAR_SPEED_PORT))
            self.connect(self.get_part(PROPELLER_ID_TPL.format(i)).get_port('thrust'), self.get_port(THRUST_PORT_TPL.format(i)))
            self.connect(self.get_part(MOTOR_ID_TPL.format(i)).get_port(REACTION_TORQUE_PORT), self.get_port(TORQUE_PORT_TPL.format(i)))
        
        if conf.PLOT:
            plotter = self.get_part(PLOTTER_ID)
            sensors = self.get_part(SENSORS_ID)
            plot_ports = ['x', 'y', 'z', 'x_speed', 'y_speed', 'z_speed']
            self.connect(self.get_port(TIME_PORT), plotter.get_port(TIME_PORT))
            for name in plot_ports:
                self.connect(sensors.get_port(name), plotter.get_port(name))


class Rigid_Body_Simulator(Part):
    """
    A behavioral part that wraps the physics engine.

    This part is responsible for stepping the physics simulation, applying
    forces and torques from the multirotor model to the simulated avatar,
    and reading back the resulting state (position, orientation, speeds) to
    be fed back into the model. It acts as the bridge between the control
    system and the simulated physical world.
    """
    def behavior(self):
        """
        Executes one step of the physics simulation.

        This method applies forces/torques from the previous control cycle,
        advances the physics engine by one time step, and then reads the new
        state (position, orientation, etc.) from the simulated avatar, setting
        the output ports for the next control cycle.

        Raises:
            RuntimeError: If the physics engine is not connected.
        """
        if not self.engine or not self.engine.is_connected():
            # If the physics engine is not connected, we cannot proceed.
            # Raise an exception to stop the simulation thread gracefully.
            raise RuntimeError(ERR_ENGINE_NOT_CONNECTED)
        
        # Check if motor inputs are ready and apply forces if they are.
        # This uses the control inputs calculated in the previous simulation step.
        if all(p.is_updated() for p in self.thrust_ports) and all(p.is_updated() for p in self.torque_ports):
            for i, (thrust_port, torque_port) in enumerate(zip(self.thrust_ports, self.torque_ports)):
                thrust = float(thrust_port.get())
                torque = float(torque_port.get())
                # The link index `i` from enumerate is 0-based, which matches PyBullet's link indexing.
                #self.engine.log_input(i, 'thrust', thrust)
                #self.engine.log_input(i, 'torque', torque)
                self.engine.set_force(i, thrust)
                self.engine.set_torque(i, torque)
            
            #self.engine.write_log_inputs()
            # Only step the simulation after new forces have been applied.
            self.engine.step()

        # After stepping, read the new state and set the output ports for the next control cycle.
        position, orientation = self.engine.get_zero_order_state()
        self.get_port(MULTIROTOR_POSITION_PORT).set(position)
        self.get_port(MULTIROTOR_ORIENTATION_PORT).set(orientation)
        linear_speed, angular_speed = self.engine.get_first_order_state()
        self.get_port(MULTIROTOR_LINEAR_SPEED_PORT).set(linear_speed)
        self.get_port(MULTIROTOR_ANGULAR_SPEED_PORT).set(angular_speed)
        #self.engine.write_log_outputs(position, orientation, linear_speed, angular_speed)

    def init(self):
        self.engine.init()
        #self.engine.init_logs()

    def term(self):
        #self.engine.term_logs()
        self.engine.term()

    def __init__(self, identifier: str, conf: object):
        """
        Initializes the Rigid_Body_Simulator behavioral part.

        Args:
            identifier (str): The unique name for this part.
            conf (object): The simulation configuration object.
        """
        ports = [
            Port(TIME_PORT, Port.IN),
            Port(MULTIROTOR_POSITION_PORT, Port.OUT),
            Port(MULTIROTOR_ORIENTATION_PORT, Port.OUT),
            Port(MULTIROTOR_LINEAR_SPEED_PORT, Port.OUT),
            Port(MULTIROTOR_ANGULAR_SPEED_PORT, Port.OUT)
        ]
        propellers_indexes = PROPELLERS_INDEXES(conf)
        for i in propellers_indexes:
            ports.append(Port(MULTIROTOR_THRUST_PORT_TPL.format(i), Port.IN))
            ports.append(Port(MULTIROTOR_TORQUE_PORT_TPL.format(i), Port.IN))
        super().__init__(identifier=identifier, ports=ports, conf=conf, scheduling_condition=time_updated)
        self.engine = get_physics_engine(conf)
        self.thrust_ports = [self.get_port(MULTIROTOR_THRUST_PORT_TPL.format(i)) for i in propellers_indexes]
        self.torque_ports = [self.get_port(MULTIROTOR_TORQUE_PORT_TPL.format(i)) for i in propellers_indexes]
        self.add_hook(ml_conf.HOOK_TYPE_INIT, Rigid_Body_Simulator.init)
        self.add_hook(ml_conf.HOOK_TYPE_TERM, Rigid_Body_Simulator.term)


class Top(Part):
    """
    The top-level structural part for the entire simulation.

    This part composes the `Multirotor` model and the `Rigid_Body_Simulator`,
    connecting them together. It also includes a time distributor to synchronize
    all components with a single time signal, driven by an external `Timer`
    event source.
    """
    def __init__(self, identifier: str, conf: object, log_queue=None, error_queue=None):
        """
        Initializes the Top structural part.

        Args:
            identifier (str): The unique name for this part.
            conf (object): The simulation configuration object.
            log_queue (multiprocessing.Queue, optional): Queue for logging in multiprocess execution.
            error_queue (multiprocessing.Queue, optional): Queue for error reporting in multiprocess execution.
        """
        event_queues = [EventQueue(TIME_EVENT_IN_Q, EventQueue.IN, size=1)]
        parts = {
            TIME_DIST_ID: EventToDataSynchronizer(
                TIME_DIST_ID,
                input_queue_id=TIME_EVENT_IN_Q,
                output_port_id=TIME_OUT_PORT
            ),
            SIMULATOR_ID: Rigid_Body_Simulator(SIMULATOR_ID, conf),
            MULTIROTOR_ID: Multirotor(MULTIROTOR_ID, conf)
        }
        execution_strategy = Execution(name='parallel_multirotor_execution',
            parallelization_condition=lambda part: part.get_identifier() == MULTIROTOR_ID,
            mode=conf.PARALLEL_EXECUTION_MODE,
            log_queue=log_queue,
            error_queue=error_queue
        )
        super().__init__(
            identifier=identifier,
            execution_strategy=execution_strategy,
            parts=parts,
            event_queues=event_queues,
            conf=conf
        )
        
        time_dist = self.get_part(TIME_DIST_ID)
        simulator = self.get_part(SIMULATOR_ID)
        multirotor = self.get_part(MULTIROTOR_ID)

        self.connect_event_queue(self.get_event_queue(TIME_EVENT_IN_Q), time_dist.get_event_queue(TIME_EVENT_IN_Q))
        
        time_out_port = time_dist.get_port(TIME_OUT_PORT)
        self.connect(time_out_port, simulator.get_port(TIME_PORT))
        self.connect(time_out_port, multirotor.get_port(TIME_PORT))
        
        self.connect(simulator.get_port(MULTIROTOR_POSITION_PORT), multirotor.get_port(POSITION_PORT))
        self.connect(simulator.get_port(MULTIROTOR_ORIENTATION_PORT), multirotor.get_port(ORIENTATION_PORT))
        self.connect(simulator.get_port(MULTIROTOR_LINEAR_SPEED_PORT), multirotor.get_port(LINEAR_SPEED_PORT))
        self.connect(simulator.get_port(MULTIROTOR_ANGULAR_SPEED_PORT), multirotor.get_port(ANGULAR_SPEED_PORT))
        for i in PROPELLERS_INDEXES(conf):
            self.connect(multirotor.get_port(THRUST_PORT_TPL.format(i)), simulator.get_port(MULTIROTOR_THRUST_PORT_TPL.format(i)))
            self.connect(multirotor.get_port(TORQUE_PORT_TPL.format(i)), simulator.get_port(MULTIROTOR_TORQUE_PORT_TPL.format(i)))
