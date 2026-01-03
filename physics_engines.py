from abc import ABC, abstractmethod
import os
import math
import re
import numpy as np
from services import generate_urdf_model
from world.services import generate_world

from enum import Enum

# --- Engines ---
class PhysicsEngines(Enum):
    PYBULLET = 'PyBullet'

# --- Error Messages ---
ERR_URDF_NOT_FOUND = "URDF model file not found at {}. Generation failed."
ERR_URDF_LOAD_FAILED = "Failed to load URDF model {}: {}"
ERR_ENGINE_NOT_CONNECTED = "Physics engine not connected."
ERR_ENGINE_STEP_FAILED = "Physical simulation step failed."
ERR_ENGINE_CONNECT_FAILED = "Failed to connect to physics engine in '{}' mode."
ERR_ENGINE_CONNECT_NO_EXCEPTION = "Physics engine connection failed without exception."

class SimulatorEngine(ABC):
    def __init__(self, conf, f_thrust_path, f_state_path):
        self.backend = None
        self.conf = conf
        self.sim_time = 0.0
        self.input_log_buffer = {}
        self.f_thrust = None
        self.f_state = None
        self.f_thrust_path = f_thrust_path
        self.f_state_path = f_state_path

    @abstractmethod
    def init(self): pass

    @abstractmethod
    def term(self): pass

    @abstractmethod
    def is_connected(self): pass

    @abstractmethod
    def set_force(self, propeller_index, thrust): pass

    @abstractmethod
    def set_torque(self, propeller_index, torque): pass

    @abstractmethod
    def step(self): pass

    @abstractmethod
    def get_zero_order_state(self): pass

    @abstractmethod
    def get_first_order_state(self): pass

    def init_logs(self):
        if not os.path.exists('logs'):
            os.makedirs('logs')
        self.f_thrust = open(self.f_thrust_path, 'w')
        self.f_thrust.write('time,thrust_1,torque_1,thrust_2,torque_2,thrust_3,torque_3,thrust_4,torque_4\n')
        
        self.f_state = open(self.f_state_path, 'w')
        self.f_state.write('time,x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz\n')

    def term_logs(self):
        if self.f_thrust: self.f_thrust.close()
        if self.f_state: self.f_state.close()

    def log_input(self, index, key, value):
        if index not in self.input_log_buffer:
            self.input_log_buffer[index] = {}
        self.input_log_buffer[index][key] = value

    def write_inputs(self):
        if self.f_thrust:
            line = f"{self.sim_time}"
            for i in range(self.conf.PROPELLERS):
                vals = self.input_log_buffer.get(i, {})
                line += f",{vals.get('thrust', 0.0)},{vals.get('torque', 0.0)}"
            self.f_thrust.write(line + "\n")
            self.input_log_buffer.clear()

    def write_state(self, pos, rot, lin, ang):
        if self.f_state:
            line = f"{self.sim_time},{pos[0]},{pos[1]},{pos[2]},{rot[0]},{rot[1]},{rot[2]},{lin[0]},{lin[1]},{lin[2]},{ang[0]},{ang[1]},{ang[2]}\n"
            self.f_state.write(line)

class PyBulletEngine(SimulatorEngine):
    def init(self):
        import pybullet
        try:
            from pybullet_utils.pybullet_c_api import allowInternalThreadCaches
        except ImportError:
            from contextlib import nullcontext as allowInternalThreadCaches
        
        self.backend = pybullet
        self.allowInternalThreadCaches = allowInternalThreadCaches
        
        mode_str = 'GUI' if self.conf.GUI else 'DIRECT'
        connection_mode = self.backend.GUI if self.conf.GUI else self.backend.DIRECT
        try:
            self.backend.connect(connection_mode)
        except self.backend.error as e:
            raise RuntimeError(ERR_ENGINE_CONNECT_FAILED.format(mode_str)) from e

        if not self.backend.isConnected():
            raise RuntimeError(ERR_ENGINE_CONNECT_NO_EXCEPTION)

        self.backend.setGravity(0, 0, self.conf.G)
        self.backend.setTimeStep(self.conf.TIME_STEP)
        self.backend.setRealTimeSimulation(0)
        
        generate_world(self.backend)
        if self.conf.UPDATE_URDF_MODEL:
            generate_urdf_model(self.conf.BASE_DIRECTORY, self.conf.URDF_MODEL, self.conf.URDF_TEMPLATE, self.conf.FRAME_MASS, self.conf.FRAME_SIZE, self.conf.PROPELLERS, self.conf.MAIN_RADIUS)
        
        if not os.path.exists(self.conf.URDF_MODEL):
            raise FileNotFoundError(ERR_URDF_NOT_FOUND.format(self.conf.URDF_MODEL))

        try:
            self.multirotor_avatar = self.backend.loadURDF(self.conf.URDF_MODEL, self.conf.INITIAL_POSITION, self.backend.getQuaternionFromEuler(self.conf.INITIAL_ROTATION)) # type: ignore
        except self.backend.error as e:
            raise RuntimeError(ERR_URDF_LOAD_FAILED.format(self.conf.URDF_MODEL, e))
        
    def term(self):
        if self.backend.isConnected():
            self.backend.disconnect()

    def is_connected(self):
        return self.backend.isConnected()

    def set_force(self, propeller_index, thrust):
        # Apply force in the Z-axis of the link frame
        self.backend.applyExternalForce(self.multirotor_avatar, propeller_index, [0, 0, thrust], [0, 0, 0], self.backend.LINK_FRAME)

    def set_torque(self, propeller_index, torque):
        # Apply torque around the Z-axis of the link frame
        self.backend.applyExternalTorque(self.multirotor_avatar, propeller_index, [0, 0, torque], self.backend.LINK_FRAME)

    def step(self):
        try:
            with self.allowInternalThreadCaches(self.backend):
                self.backend.stepSimulation()
        except self.backend.error as e:
            raise RuntimeError(ERR_ENGINE_STEP_FAILED) from e
        self.sim_time += self.conf.TIME_STEP

    def get_zero_order_state(self):
        pos, quat = self.backend.getBasePositionAndOrientation(self.multirotor_avatar)
        euler = self.backend.getEulerFromQuaternion(quat)
        return pos, euler

    def get_first_order_state(self):
        return self.backend.getBaseVelocity(self.multirotor_avatar)

def get_physics_engine(conf):
    # Default to PyBullet if not specified
    engine_type = getattr(conf, 'PHYSICS_ENGINE')
    if engine_type == PhysicsEngines.PYBULLET:
        return PyBulletEngine(conf, 'logs/thrust_torque_pybullet.csv', 'logs/state_pybullet.csv')
    raise ValueError(f"Unknown physics engine: {engine_type}")