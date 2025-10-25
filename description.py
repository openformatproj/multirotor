from ml.engine import Part, Port, EventQueue
from ml.strategies import sequential_execution
from ml.parts import EventToDataSynchronizer
import pybullet
import os
import sys

from ml import conf as ml_conf

sys.path.append(os.path.dirname(__file__))
from services import generate_urdf_model
from world.services import generate_world
from propeller.description import Propeller
from motor.description import Motor
from sensors.description import Sensors
from trajectory_planner.structure import Trajectory_Planner
from controller.description import Controller
from monitor.xyz import XYZ_Monitor
from constants import X, Y, Z

PROPELLERS_INDEXES = None # Will be initialized in Top

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

# --- Error Messages ---
ERR_URDF_NOT_FOUND = "URDF model file not found at {}. Generation failed."
ERR_URDF_LOAD_FAILED = "Failed to load URDF model {}: {}"
ERR_PYBULLET_NOT_CONNECTED = "PyBullet engine not connected."
ERR_PYBULLET_STEP_FAILED = "PyBullet simulation step failed."
ERR_PYBULLET_CONNECT_FAILED = "Failed to connect to PyBullet in '{}' mode."
ERR_PYBULLET_CONNECT_NO_EXCEPTION = "PyBullet connection failed without exception."

# --- Log constants ---
LOG_EVENT_STEP = "STEP"
LOG_DETAIL_KEY_CONNECTED = "connected"

class Multirotor(Part):
    """
    A structural part representing the multirotor vehicle.

    This part composes all the core components of the multirotor, including
    motors, propellers, sensors, a trajectory planner, and a controller. It
    defines the internal wiring that connects these components and exposes
    interfaces for receiving sensor data from the simulator and providing
    actuator outputs (thrust and torque) back to it.
    """

    def __init__(self, identifier: str, conf: object, controller_execution_strategy=sequential_execution):
        """
        Initializes the Multirotor structural part.

        Args:
            identifier (str): The unique name for this part.
            conf (object): The simulation configuration object.
            controller_execution_strategy: The execution strategy for the controller.
        """
        ports = [
            Port(TIME_PORT, Port.IN),
            Port(POSITION_PORT, Port.IN),
            Port(ORIENTATION_PORT, Port.IN),
            Port(LINEAR_SPEED_PORT, Port.IN),
            Port(ANGULAR_SPEED_PORT, Port.IN)
        ]
        parts = {
            SENSORS_ID: Sensors(SENSORS_ID),
            TRAJECTORY_PLANNER_ID: Trajectory_Planner(TRAJECTORY_PLANNER_ID, conf),
            CONTROLLER_ID: Controller(CONTROLLER_ID, PROPELLERS_INDEXES, conf=conf, execution_strategy=controller_execution_strategy)
        }
        if conf.PLOT:
            parts[PLOTTER_ID] = XYZ_Monitor(
                PLOTTER_ID,
                plot_decimation=conf.PLOT_DECIMATION
            )

        for i in PROPELLERS_INDEXES:
            ports.append(Port(THRUST_PORT_TPL.format(i), Port.OUT))
            ports.append(Port(TORQUE_PORT_TPL.format(i), Port.OUT))
            parts[MOTOR_ID_TPL.format(i)] = Motor(MOTOR_ID_TPL.format(i))
            direction = Propeller.LEFT_HANDED if i in [2, 4] else Propeller.RIGHT_HANDED
            parts[PROPELLER_ID_TPL.format(i)] = Propeller(PROPELLER_ID_TPL.format(i), direction)
            
        super().__init__(
            identifier=identifier,
            execution_strategy=sequential_execution,
            ports=ports,
            parts=parts,
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
        for i in PROPELLERS_INDEXES:
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
    A behavioral part that wraps the PyBullet physics engine.

    This part is responsible for stepping the physics simulation, applying
    forces and torques from the multirotor model to the simulated avatar,
    and reading back the resulting state (position, orientation, speeds) to
    be fed back into the model. It acts as the bridge between the control
    system and the simulated physical world.
    """
    def _set_engine(self, engine):
        """
        Initializes the simulator with a PyBullet engine instance, generating
        the world and loading the URDF model.
        """
        self.engine = engine
        generate_world(self.engine)
        if self.conf.UPDATE_URDF_MODEL:
            generate_urdf_model(self.conf.BASE_DIRECTORY, self.conf.URDF_MODEL, self.conf.URDF_TEMPLATE, self.conf.FRAME_MASS, self.conf.FRAME_SIZE, self.conf.PROPELLERS, self.conf.MAIN_RADIUS)
        
        if not os.path.exists(self.conf.URDF_MODEL):
            raise FileNotFoundError(ERR_URDF_NOT_FOUND.format(self.conf.URDF_MODEL))

        try:
            self.multirotor_avatar = self.engine.loadURDF(self.conf.URDF_MODEL, self.conf.INITIAL_POSITION, self.engine.getQuaternionFromEuler(self.conf.INITIAL_ROTATION)) # type: ignore
        except self.engine.error as e:
            raise RuntimeError(ERR_URDF_LOAD_FAILED.format(self.conf.URDF_MODEL, e))

    def behavior(self):
        """
        Executes one step of the physics simulation.

        This method applies forces/torques from the previous control cycle,
        advances the physics engine by one time step, and then reads the new
        state (position, orientation, etc.) from the simulated avatar, setting
        the output ports for the next control cycle.
        """
        if not self.engine or not self.engine.isConnected():
            # If the physics engine is not connected, we cannot proceed.
            # Raise an exception to stop the simulation thread gracefully.
            raise RuntimeError(ERR_PYBULLET_NOT_CONNECTED)

        # Check if motor inputs are ready and apply forces if they are.
        # This uses the control inputs calculated in the previous simulation step.
        if all(p.is_updated() for p in self.thrust_ports) and all(p.is_updated() for p in self.torque_ports):
            for i, (thrust_port, torque_port) in enumerate(zip(self.thrust_ports, self.torque_ports)):
                thrust = thrust_port.get()
                torque = torque_port.get()
                # The link index `i` from enumerate is 0-based, which matches PyBullet's link indexing.
                self.engine.applyExternalForce(self.multirotor_avatar, i, thrust, [0, 0, 0], self.engine.LINK_FRAME)
                self.engine.applyExternalTorque(self.multirotor_avatar, i, torque, self.engine.LINK_FRAME)
            
            # Only step the simulation after new forces have been applied.
            try:
                self.engine.stepSimulation()
            except self.engine.error as e:
                raise RuntimeError(ERR_PYBULLET_STEP_FAILED) from e

        # After stepping, read the new state and set the output ports for the next control cycle.
        position, orientation = self.engine.getBasePositionAndOrientation(self.multirotor_avatar)
        self.get_port(MULTIROTOR_POSITION_PORT).set(position)
        self.get_port(MULTIROTOR_ORIENTATION_PORT).set(orientation)
        linear_speed, angular_speed = self.engine.getBaseVelocity(self.multirotor_avatar)
        self.get_port(MULTIROTOR_LINEAR_SPEED_PORT).set(linear_speed)
        self.get_port(MULTIROTOR_ANGULAR_SPEED_PORT).set(angular_speed)

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
        for i in PROPELLERS_INDEXES:
            ports.append(Port(MULTIROTOR_THRUST_PORT_TPL.format(i), Port.IN))
            ports.append(Port(MULTIROTOR_TORQUE_PORT_TPL.format(i), Port.IN))
        super().__init__(identifier=identifier, ports=ports, conf=conf, scheduling_condition=lambda part: part.get_port(TIME_PORT).is_updated())
        self.engine = None
        self.multirotor_avatar = None
        self.thrust_ports = [self.get_port(MULTIROTOR_THRUST_PORT_TPL.format(i)) for i in PROPELLERS_INDEXES]
        self.torque_ports = [self.get_port(MULTIROTOR_TORQUE_PORT_TPL.format(i)) for i in PROPELLERS_INDEXES]


class Top(Part):
    """
    The top-level structural part for the entire simulation.

    This part composes the `Multirotor` model and the `Rigid_Body_Simulator`,
    connecting them together. It also includes a time distributor to synchronize
    all components with a single time signal, driven by an external `Timer`
    event source. It manages the lifecycle of the PyBullet connection via
    init and term hooks.
    """
    def _init_pybullet(self):
        """
        Initialization hook that connects to the PyBullet physics engine.

        It sets the connection mode (GUI or headless), configures gravity and
        the simulation time step, and passes the engine instance to the simulator part.
        """
        self.engine = pybullet
        # Use DIRECT for headless simulation, GUI for visualization.
        mode_str = 'GUI' if self.conf.GUI else 'DIRECT'
        connection_mode = self.engine.GUI if self.conf.GUI else self.engine.DIRECT
        try:
            # Connect to the physics engine. This can fail if GUI is requested
            # but no graphical environment is available.
            self.engine.connect(connection_mode)
        except self.engine.error as e:
            # Re-raise pybullet's specific error as a more general RuntimeError
            # with context, which will be handled by the main run loop.
            raise RuntimeError(ERR_PYBULLET_CONNECT_FAILED.format(mode_str)) from e

        if not self.engine.isConnected():
            # This is a fallback for the unlikely case that connect() returns
            # without error but fails to establish a connection.
            raise RuntimeError(ERR_PYBULLET_CONNECT_NO_EXCEPTION)

        # Configure the physics engine. This is critical for stability.
        self.engine.setGravity(0, 0, self.conf.G)
        self.engine.setTimeStep(self.conf.TIME_STEP)
        self.engine.setRealTimeSimulation(0) # We are driving the simulation manually

        self.get_part(SIMULATOR_ID)._set_engine(self.engine)

    def _term_pybullet(self):
        """
        Termination hook that gracefully disconnects from the PyBullet engine
        if a connection is active.
        """
        if self.engine and self.engine.isConnected():
            self.engine.disconnect()

    def __init__(self, identifier: str, conf: object, execution_strategy=sequential_execution, controller_execution_strategy=sequential_execution):
        """
        Initializes the Top structural part.

        Args:
            identifier (str): The unique name for this part.
            execution_strategy: The strategy for executing inner parts.
            controller_execution_strategy: The execution strategy for the controller,
                                           passed down to the Multirotor part.
        """
        global PROPELLERS_INDEXES
        PROPELLERS_INDEXES = range(1, conf.PROPELLERS + 1)

        self.engine = None
        event_queues = [EventQueue(TIME_EVENT_IN_Q, EventQueue.IN, size=1)]
        parts = {
            TIME_DIST_ID: EventToDataSynchronizer(
                TIME_DIST_ID,
                input_queue_id=TIME_EVENT_IN_Q,
                output_port_id=TIME_OUT_PORT
            ),
            SIMULATOR_ID: Rigid_Body_Simulator(SIMULATOR_ID, conf=conf),
            MULTIROTOR_ID: Multirotor(MULTIROTOR_ID, conf=conf, controller_execution_strategy=controller_execution_strategy)
        }
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
        for i in PROPELLERS_INDEXES:
            self.connect(multirotor.get_port(THRUST_PORT_TPL.format(i)), simulator.get_port(MULTIROTOR_THRUST_PORT_TPL.format(i)))
            self.connect(multirotor.get_port(TORQUE_PORT_TPL.format(i)), simulator.get_port(MULTIROTOR_TORQUE_PORT_TPL.format(i)))
        self.add_hook(ml_conf.HOOK_TYPE_INIT, self._init_pybullet)
        self.add_hook(ml_conf.HOOK_TYPE_TERM, self._term_pybullet)