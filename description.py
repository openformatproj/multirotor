from ml.engine import Part
from ml.engine import Port
from ml.parts import Broadcaster, Part_Timed, Timer
import pybullet
from decimal import getcontext
# import matplotlib
# matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

import os
import sys
sys.path.append(os.path.dirname(__file__))
import conf
from services import generate_urdf_model
from world.services import generate_world
from propeller.description import Propeller
from motor.description import Motor
from sensors.description import Sensors
from trajectory_planner.behavior import Trajectory_Planner
from controller.description import Controller
from constants import X, Y, Z

PROPELLERS_INDEXES = range(1, conf.PROPELLERS + 1)

class Multirotor(Part_Timed):

    def set_t_step(self, t_step):
        self.get_part('trajectory_planner').set_t_step(t_step)
        self.get_part('controller').set_t_step(t_step)

    def init_plots(self):
        plt.ion()
        fig, ((ax_xy_position, ax_xy_speed), (ax_z_position, ax_z_speed)) = plt.subplots(2, 2)
        xy_position, = ax_xy_position.plot([], [], 'ro')
        line_xy_position, = ax_xy_position.plot([], [], '-')
        ax_xy_position.set_title("XY position")
        ax_xy_position.set_xlim(conf.POSITION_GRAPH_BOUNDARIES[X.index()][0], conf.POSITION_GRAPH_BOUNDARIES[X.index()][1])
        ax_xy_position.set_ylim(conf.POSITION_GRAPH_BOUNDARIES[Y.index()][0], conf.POSITION_GRAPH_BOUNDARIES[Y.index()][1])
        ax_xy_position.set_aspect('equal', adjustable='box')
        xy_speed, = ax_xy_speed.plot([], [], 'ro')
        line_xy_speed, = ax_xy_speed.plot([], [], '-')
        ax_xy_speed.set_title("XY speed")
        ax_xy_speed.set_xlim(conf.SPEED_GRAPH_BOUNDARIES[X.index()][0], conf.SPEED_GRAPH_BOUNDARIES[X.index()][1])
        ax_xy_speed.set_ylim(conf.SPEED_GRAPH_BOUNDARIES[Y.index()][0], conf.SPEED_GRAPH_BOUNDARIES[Y.index()][1])
        ax_xy_speed.set_aspect('equal', adjustable='box')
        z_position, = ax_z_position.plot([], [], 'ro')
        line_z_position, = ax_z_position.plot([], [], '-')
        ax_z_position.set_title("Z position")
        ax_z_position.set_xlim(conf.POSITION_GRAPH_BOUNDARIES[X.index()][0], conf.POSITION_GRAPH_BOUNDARIES[X.index()][1])
        ax_z_position.set_ylim(conf.POSITION_GRAPH_BOUNDARIES[Z.index()][0], conf.POSITION_GRAPH_BOUNDARIES[Z.index()][1])
        ax_z_position.set_aspect('equal', adjustable='box')
        z_speed, = ax_z_speed.plot([], [], 'ro')
        line_z_speed, = ax_z_speed.plot([], [], '-')
        ax_z_speed.set_title("Z speed")
        ax_z_speed.set_xlim(conf.SPEED_GRAPH_BOUNDARIES[X.index()][0], conf.SPEED_GRAPH_BOUNDARIES[X.index()][1])
        ax_z_speed.set_ylim(conf.SPEED_GRAPH_BOUNDARIES[Z.index()][0], conf.SPEED_GRAPH_BOUNDARIES[Z.index()][1])
        ax_z_speed.set_aspect('equal', adjustable='box')
        self.get_part('sensors').set_plot(fig, ax_xy_position, xy_position, line_xy_position, xy_speed, ax_xy_speed, line_xy_speed, ax_z_position, z_position, line_z_position, ax_z_speed, z_speed, line_z_speed)

    def term_plots(self):
        plt.ioff()
        plt.show()

    def __init__(self, identifier):
        ports = [
            Port('position', Port.IN),
            Port('orientation', Port.IN),
            Port('linear_speed', Port.IN),
            Port('angular_speed', Port.IN)
        ]
        parts = {
            Sensors('sensors', conf.PLOT),
            Trajectory_Planner('trajectory_planner', conf.SET_POSITION, conf.SET_SPEED),
            Controller('controller', PROPELLERS_INDEXES),
            Broadcaster('broadcaster_time', outputs = 5),
            Broadcaster('broadcaster_time_propellers', outputs = conf.PROPELLERS),
            Broadcaster('broadcaster_time_motors', outputs = conf.PROPELLERS)
        }
        for i in PROPELLERS_INDEXES:
            ports.append(Port(f'thrust_{i}', Port.OUT))
            ports.append(Port(f'torque_{i}', Port.OUT))
            parts.add(Motor(f'motor_{i}'))
            direction = Propeller.LEFT_HANDED if i in [2, 4] else Propeller.RIGHT_HANDED
            parts.add(Propeller(f'propeller_{i}', direction))
        super().__init__(identifier, Part.SEQUENTIAL_ALLOCATION, ports = ports, parts = parts)
        self.connect(self.get_port('time'), self.get_part('broadcaster_time').get_port('in'))
        self.connect(self.get_part('broadcaster_time').get_port(f'out_0'), self.get_part('sensors').get_port('time'))
        self.connect(self.get_part('broadcaster_time').get_port(f'out_1'), self.get_part('trajectory_planner').get_port('time'))
        self.connect(self.get_part('broadcaster_time').get_port(f'out_2'), self.get_part('broadcaster_time_propellers').get_port('in'))
        self.connect(self.get_part('broadcaster_time').get_port(f'out_3'), self.get_part('broadcaster_time_motors').get_port('in'))
        self.connect(self.get_part('broadcaster_time').get_port(f'out_4'), self.get_part('controller').get_port('time'))
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
            self.connect(self.get_part('broadcaster_time_propellers').get_port(f'out_{i-1}'), self.get_part(f'propeller_{i}').get_port('time'))
            self.connect(self.get_part('broadcaster_time_motors').get_port(f'out_{i-1}'), self.get_part(f'motor_{i}').get_port('time'))
            self.connect(self.get_part('controller').get_port(f'angular_speed_{i}'), self.get_part(f'motor_{i}').get_port('angular_speed_in'))
            self.connect(self.get_part(f'motor_{i}').get_port('angular_speed_out'), self.get_part(f'propeller_{i}').get_port('angular_speed'))
            self.connect(self.get_part(f'propeller_{i}').get_port(f'thrust'), self.get_port(f'thrust_{i}'))
            self.connect(self.get_part(f'motor_{i}').get_port(f'reaction_torque'), self.get_port(f'torque_{i}'))
        if conf.PLOT:
            self.add_hook('init', self.init_plots)
            self.add_hook('term', self.term_plots)

class Rigid_Body_Simulator(Part_Timed):

    def set_engine(self, engine):
        self.engine = engine
        generate_world(self.engine)
        if conf.UPDATE_URDF_MODEL:
            generate_urdf_model(conf.BASE_DIRECTORY, conf.URDF_MODEL, conf.URDF_TEMPLATE, conf.FRAME_MASS, conf.FRAME_SIZE, conf.PROPELLERS, conf.MAIN_RADIUS)
        self.multirotor_avatar = self.engine.loadURDF(conf.URDF_MODEL, conf.INITIAL_POSITION, self.engine.getQuaternionFromEuler(conf.INITIAL_ROTATION))

    def behavior(self):
        if self.get_port('time').is_updated():
            self.t = self.get_port('time').get()
            position, orientation = self.engine.getBasePositionAndOrientation(self.multirotor_avatar)
            # print(f'Time {self.t}: Position: {position}, Orientation: {orientation}')
            self.get_port('multirotor_position').set(position)
            self.get_port('multirotor_orientation').set(orientation)
            linear_speed, angular_speed = self.engine.getBaseVelocity(self.multirotor_avatar)
            self.get_port('multirotor_linear_speed').set(linear_speed)
            self.get_port('multirotor_angular_speed').set(angular_speed)
            if all((self.get_port(f'multirotor_thrust_{i}').is_updated() and self.get_port(f'multirotor_torque_{i}').is_updated()) for i in PROPELLERS_INDEXES):
                for i in PROPELLERS_INDEXES:
                    thrust = self.get_port(f'multirotor_thrust_{i}').get()
                    torque = self.get_port(f'multirotor_torque_{i}').get()
                    # print(f'Time {self.t}: Thrust {i}: {thrust}, Torque: {torque}')
                    self.engine.applyExternalForce(self.multirotor_avatar, i-1, thrust, [0, 0, 0], self.engine.LINK_FRAME)
                    self.engine.applyExternalTorque(self.multirotor_avatar, i-1, torque, self.engine.LINK_FRAME)
                self.engine.stepSimulation()

    def __init__(self, identifier):
        ports = [
            Port('multirotor_position', Port.OUT),
            Port('multirotor_orientation', Port.OUT),
            Port('multirotor_linear_speed', Port.OUT),
            Port('multirotor_angular_speed', Port.OUT)
        ]
        for i in PROPELLERS_INDEXES:
            ports.append(Port(f'multirotor_thrust_{i}', Port.IN))
            ports.append(Port(f'multirotor_torque_{i}', Port.IN))
        super().__init__(identifier, Part.NO_ALLOCATION, ports = ports)

class Top(Part):

    def init_simulation_engine(self):
        engine = pybullet
        engine.connect(engine.GUI)
        if not engine.isConnected():
            raise RuntimeError("Failed to connect to PyBullet engine")
        t_step = engine.getPhysicsEngineParameters()['fixedTimeStep']
        self.get_part('simulator').set_engine(engine)
        self.get_part('multirotor').set_t_step(t_step)
        self.get_part('timer').set_t_step(t_step)
    
    def __init__(self, identifier):
        parts = [
            Rigid_Body_Simulator('simulator'),
            Multirotor('multirotor'),
            Timer('timer'),
            Broadcaster('broadcaster_time', outputs = 2)
        ]
        super().__init__(identifier, Part.SEQUENTIAL_ALLOCATION, parts = parts)
        getcontext().prec = conf.DECIMAL_CONTEXT_PRECISION
        self.connect(self.get_part('simulator').get_port('multirotor_position'), self.get_part('multirotor').get_port('position'))
        self.connect(self.get_part('simulator').get_port('multirotor_orientation'), self.get_part('multirotor').get_port('orientation'))
        self.connect(self.get_part('simulator').get_port('multirotor_linear_speed'), self.get_part('multirotor').get_port('linear_speed'))
        self.connect(self.get_part('simulator').get_port('multirotor_angular_speed'), self.get_part('multirotor').get_port('angular_speed'))
        self.connect(self.get_part('timer').get_port('time'), self.get_part('broadcaster_time').get_port('in'))
        self.connect(self.get_part('broadcaster_time').get_port('out_0'), self.get_part('simulator').get_port('time'))
        self.connect(self.get_part('broadcaster_time').get_port('out_1'), self.get_part('multirotor').get_port('time'))
        for i in PROPELLERS_INDEXES:
            self.connect(self.get_part('multirotor').get_port(f'thrust_{i}'), self.get_part('simulator').get_port(f'multirotor_thrust_{i}'))
            self.connect(self.get_part('multirotor').get_port(f'torque_{i}'), self.get_part('simulator').get_port(f'multirotor_torque_{i}'))
        self.add_hook('init', self.init_simulation_engine)