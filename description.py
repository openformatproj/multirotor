from ml.engine import Part, Port, EventQueue
from ml.strategies import sequential_execution
from ml.parts import EventToDataSynchronizer
import pybullet
from decimal import getcontext
import os
import sys
from ml.tracer import Tracer
from ml.enums import LogLevel

sys.path.append(os.path.dirname(__file__))
import conf
from services import generate_urdf_model
from world.services import generate_world
from propeller.description import Propeller
from motor.description import Motor
from sensors.description import Sensors
from trajectory_planner.structure import Trajectory_Planner
from controller.description import Controller
from monitor.xyz import XYZ_Monitor
from constants import X, Y, Z

# TODO: Refactor data transfer to use dedicated data classes for all physical quantities.
#
# The current implementation uses primitive types (e.g., `Decimal`, `np.array`)
# for all data transfers between parts. This creates implicit contracts about
# the structure and meaning of data (e.g., is this array a position or a
# velocity?), making the system brittle and harder to understand.
#
# A more robust, object-oriented approach is to create dedicated data classes
# for all physical quantities.
#
# The refactoring plan is:
# 1.  Create a new `datatypes.py` file.
# 2.  Define classes for all physical quantities, such as `Position`,
#     `Velocity`, `Angle`, `AngularRate`, `Thrust`, and `Torque`. These
#     classes should encapsulate the data and provide any necessary helper
#     methods (e.g., `as_pybullet_vector()`).
# 3.  Refactor all parts (`Sensors`, `Trajectory_Planner`, `Controller`, etc.)
#     to use these new data types on their ports instead of primitive types.

PROPELLERS_INDEXES = range(1, conf.PROPELLERS + 1)

class Multirotor(Part):
    """
    A structural part representing the multirotor vehicle.

    This part composes all the core components of the multirotor, including
    motors, propellers, sensors, a trajectory planner, and a controller. It
    defines the internal wiring that connects these components and exposes
    interfaces for receiving sensor data from the simulator and providing
    actuator outputs (thrust and torque) back to it.
    """

    def __init__(self, identifier: str):
        """
        Initializes the Multirotor structural part.

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
        parts = {
            'sensors': Sensors('sensors'),
            'trajectory_planner': Trajectory_Planner('trajectory_planner', conf.SET_POSITION, conf.SET_SPEED),
            'controller': Controller('controller', PROPELLERS_INDEXES)
        }
        if conf.PLOT:
            parts['plotter'] = XYZ_Monitor(
                'plotter',
                position_bounds=conf.POSITION_GRAPH_BOUNDARIES,
                speed_bounds=conf.SPEED_GRAPH_BOUNDARIES,
                plot_decimation=conf.PLOT_DECIMATION
            )

        for i in PROPELLERS_INDEXES:
            ports.append(Port(f'thrust_{i}', Port.OUT))
            ports.append(Port(f'torque_{i}', Port.OUT))
            parts[f'motor_{i}'] = Motor(f'motor_{i}')
            direction = Propeller.LEFT_HANDED if i in [2, 4] else Propeller.RIGHT_HANDED
            parts[f'propeller_{i}'] = Propeller(f'propeller_{i}', direction)
            
        super().__init__(
            identifier=identifier,
            execution_strategy=sequential_execution,
            ports=ports,
            parts=parts
        )
        
        self.connect(self.get_port('time'), self.get_part('sensors').get_port('time'))
        self.connect(self.get_port('time'), self.get_part('trajectory_planner').get_port('time'))
        self.connect(self.get_port('time'), self.get_part('controller').get_port('time'))
        self.connect(self.get_port('position'), self.get_part('sensors').get_port('position'))
        self.connect(self.get_port('orientation'), self.get_part('sensors').get_port('orientation'))
        self.connect(self.get_port('linear_speed'), self.get_part('sensors').get_port('linear_speed'))
        self.connect(self.get_port('angular_speed'), self.get_part('sensors').get_port('angular_speed'))
        self.connect(self.get_part('sensors').get_port('roll'), self.get_part('controller').get_port('roll'))
        self.connect(self.get_part('sensors').get_port('pitch'), self.get_part('controller').get_port('pitch'))
        self.connect(self.get_part('sensors').get_port('yaw'), self.get_part('trajectory_planner').get_port('yaw'))
        self.connect(self.get_part('sensors').get_port('roll_speed'), self.get_part('controller').get_port('roll_speed'))
        self.connect(self.get_part('sensors').get_port('pitch_speed'), self.get_part('controller').get_port('pitch_speed'))
        self.connect(self.get_part('sensors').get_port('yaw_speed'), self.get_part('controller').get_port('yaw_speed'))
        for i in [X, Y, Z]:
            self.connect(self.get_part('sensors').get_port(i.name()), self.get_part('trajectory_planner').get_port(i.name()))
            self.connect(self.get_part('trajectory_planner').get_port(f'{i.name()}_speed'), self.get_part('controller').get_port(f'{i.name()}_speed_setpoint'))
            self.connect(self.get_part('sensors').get_port(f'{i.name()}_speed'), self.get_part('controller').get_port(f'{i.name()}_speed'))
        self.connect(self.get_part('trajectory_planner').get_port('yaw_speed'), self.get_part('controller').get_port('yaw_speed_setpoint'))
        for i in PROPELLERS_INDEXES:
            self.connect(self.get_port('time'), self.get_part(f'propeller_{i}').get_port('time'))
            self.connect(self.get_port('time'), self.get_part(f'motor_{i}').get_port('time'))
            self.connect(self.get_part('controller').get_port(f'angular_speed_{i}'), self.get_part(f'motor_{i}').get_port('angular_speed_in'))
            self.connect(self.get_part(f'motor_{i}').get_port('angular_speed_out'), self.get_part(f'propeller_{i}').get_port('angular_speed'))
            self.connect(self.get_part(f'propeller_{i}').get_port(f'thrust'), self.get_port(f'thrust_{i}'))
            self.connect(self.get_part(f'motor_{i}').get_port(f'reaction_torque'), self.get_port(f'torque_{i}'))
        
        if conf.PLOT:
            plotter = self.get_part('plotter')
            sensors = self.get_part('sensors')
            plot_ports = ['x', 'y', 'z', 'x_speed', 'y_speed', 'z_speed']
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
        if conf.UPDATE_URDF_MODEL:
            generate_urdf_model(conf.BASE_DIRECTORY, conf.URDF_MODEL, conf.URDF_TEMPLATE, conf.FRAME_MASS, conf.FRAME_SIZE, conf.PROPELLERS, conf.MAIN_RADIUS)
        
        if not os.path.exists(conf.URDF_MODEL):
            raise FileNotFoundError(f"URDF model file not found at {conf.URDF_MODEL}. Generation might have failed.")

        try:
            self.multirotor_avatar = self.engine.loadURDF(conf.URDF_MODEL, conf.INITIAL_POSITION, self.engine.getQuaternionFromEuler(conf.INITIAL_ROTATION))
        except self.engine.error as e:
            raise RuntimeError(f"Failed to load URDF model {conf.URDF_MODEL}: {e}") from e

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
            raise RuntimeError("PyBullet physics engine is not connected.")

        Tracer.log(LogLevel.DEBUG, "SIMULATOR", "STEP", {"connected": self.engine.isConnected()})

        # Check if motor inputs are ready and apply forces if they are.
        # This uses the control inputs calculated in the previous simulation step.
        if all(p.is_updated() for p in self.thrust_ports) and all(p.is_updated() for p in self.torque_ports):
            for i, (thrust_port, torque_port) in enumerate(zip(self.thrust_ports, self.torque_ports)):
                thrust = thrust_port.get()
                torque = torque_port.get()
                # The link index `i` from enumerate is 0-based, which matches PyBullet's link indexing.
                self.engine.applyExternalForce(self.multirotor_avatar, i, thrust, [0, 0, 0], self.engine.LINK_FRAME)
                self.engine.applyExternalTorque(self.multirotor_avatar, i, torque, self.engine.LINK_FRAME)

        try:
            # The call to stepSimulation should be unconditional to ensure the
            # physics world always advances in time, preventing instability.
            self.engine.stepSimulation()
        except self.engine.error as e:
            # Re-raise the specific PyBullet error as a more general RuntimeError.
            # This makes it more likely that the simulation framework will catch it
            # and trigger a clean shutdown, even in fire_and_forget mode.
            raise RuntimeError("PyBullet simulation step failed. This might be due to an unstable simulation (e.g., the time step is too small).") from e

        # After stepping, read the new state and set the output ports for the next control cycle.
        position, orientation = self.engine.getBasePositionAndOrientation(self.multirotor_avatar)
        self.get_port('multirotor_position').set(position)
        self.get_port('multirotor_orientation').set(orientation)
        linear_speed, angular_speed = self.engine.getBaseVelocity(self.multirotor_avatar)
        self.get_port('multirotor_linear_speed').set(linear_speed)
        self.get_port('multirotor_angular_speed').set(angular_speed)

    def __init__(self, identifier: str):
        """
        Initializes the Rigid_Body_Simulator behavioral part.

        Args:
            identifier (str): The unique name for this part.
        """
        ports = [
            Port('time', Port.IN),
            Port('multirotor_position', Port.OUT),
            Port('multirotor_orientation', Port.OUT),
            Port('multirotor_linear_speed', Port.OUT),
            Port('multirotor_angular_speed', Port.OUT)
        ]
        for i in PROPELLERS_INDEXES:
            ports.append(Port(f'multirotor_thrust_{i}', Port.IN))
            ports.append(Port(f'multirotor_torque_{i}', Port.IN))
        super().__init__(identifier=identifier, ports=ports, scheduling_condition=lambda part: part.get_port('time').is_updated())
        self.engine = None
        self.multirotor_avatar = None
        self.thrust_ports = [self.get_port(f'multirotor_thrust_{i}') for i in PROPELLERS_INDEXES]
        self.torque_ports = [self.get_port(f'multirotor_torque_{i}') for i in PROPELLERS_INDEXES]


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
        mode_str = 'GUI' if conf.GUI else 'DIRECT'
        connection_mode = self.engine.GUI if conf.GUI else self.engine.DIRECT
        try:
            # Connect to the physics engine. This can fail if GUI is requested
            # but no graphical environment is available.
            self.engine.connect(connection_mode)
        except self.engine.error as e:
            # Re-raise pybullet's specific error as a more general RuntimeError
            # with context, which will be handled by the main run loop.
            raise RuntimeError(f"Failed to connect to PyBullet in mode '{mode_str}'. If running without a display, ensure PLOT is False.") from e

        if not self.engine.isConnected():
            # This is a fallback for the unlikely case that connect() returns
            # without error but fails to establish a connection.
            raise RuntimeError("Failed to connect to PyBullet engine, but no exception was raised.")

        # Configure the physics engine. This is critical for stability.
        self.engine.setGravity(0, 0, conf.G)
        self.engine.setTimeStep(conf.TIME_STEP)
        self.engine.setRealTimeSimulation(0) # We are driving the simulation manually

        self.get_part('simulator')._set_engine(self.engine)

    def _term_pybullet(self):
        """
        Termination hook that gracefully disconnects from the PyBullet engine
        if a connection is active.
        """
        if self.engine and self.engine.isConnected():
            self.engine.disconnect()

    def __init__(self, identifier: str, execution_strategy=sequential_execution):
        """
        Initializes the Top structural part.

        Args:
            identifier (str): The unique name for this part.
            execution_strategy: The strategy for executing inner parts.
        """
        self.engine = None
        event_queues = [EventQueue('time_event_in', EventQueue.IN, size=1)]
        parts = {
            'time_dist': EventToDataSynchronizer(
                'time_dist',
                input_queue_id='time_event_in',
                output_port_id='time_out'
            ),
            'simulator': Rigid_Body_Simulator('simulator'),
            'multirotor': Multirotor('multirotor')
        }
        super().__init__(
            identifier=identifier,
            execution_strategy=execution_strategy,
            parts=parts,
            event_queues=event_queues
        )
        getcontext().prec = conf.DECIMAL_CONTEXT_PRECISION
        
        time_dist = self.get_part('time_dist')
        simulator = self.get_part('simulator')
        multirotor = self.get_part('multirotor')

        self.connect_event_queue(self.get_event_queue('time_event_in'), time_dist.get_event_queue('time_event_in'))
        
        time_out_port = time_dist.get_port('time_out')
        self.connect(time_out_port, simulator.get_port('time'))
        self.connect(time_out_port, multirotor.get_port('time'))
        
        self.connect(self.get_part('simulator').get_port('multirotor_position'), self.get_part('multirotor').get_port('position'))
        self.connect(self.get_part('simulator').get_port('multirotor_orientation'), self.get_part('multirotor').get_port('orientation'))
        self.connect(self.get_part('simulator').get_port('multirotor_linear_speed'), self.get_part('multirotor').get_port('linear_speed'))
        self.connect(self.get_part('simulator').get_port('multirotor_angular_speed'), self.get_part('multirotor').get_port('angular_speed'))
        for i in PROPELLERS_INDEXES:
            self.connect(self.get_part('multirotor').get_port(f'thrust_{i}'), self.get_part('simulator').get_port(f'multirotor_thrust_{i}'))
            self.connect(self.get_part('multirotor').get_port(f'torque_{i}'), self.get_part('simulator').get_port(f'multirotor_torque_{i}'))
        self.add_hook('init', self._init_pybullet)
        self.add_hook('term', self._term_pybullet)