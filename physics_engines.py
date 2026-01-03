from abc import ABC, abstractmethod
import os
import math
import numpy as np
from services import generate_urdf_model
from world.services import generate_world

from enum import Enum

# --- Engines ---
class PhysicsEngines(Enum):
    PYBULLET = 'PyBullet'
    MUJOCO = 'MuJoCo'

# --- Error Messages ---
ERR_URDF_NOT_FOUND = "URDF model file not found at {}. Generation failed."
ERR_URDF_LOAD_FAILED = "Failed to load URDF model {}: {}"
ERR_ENGINE_NOT_CONNECTED = "Physics engine not connected."
ERR_ENGINE_STEP_FAILED = "Physical simulation step failed."
ERR_ENGINE_CONNECT_FAILED = "Failed to connect to physics engine in '{}' mode."
ERR_ENGINE_CONNECT_NO_EXCEPTION = "Physics engine connection failed without exception."
ERR_ENGINE_UNKNOWN = "Unknown physics engine: {}"

LOGS_PATH = 'logs'

class SimulatorEngine(ABC):
    """Abstract base class for physics engine wrappers."""
    def __init__(self, conf, engine_inputs_path, engine_outputs_path):
        """
        Initializes the SimulatorEngine.

        Args:
            conf: The simulation configuration object.
            engine_inputs_path: Path to the CSV file for logging engine inputs.
            engine_outputs_path: Path to the CSV file for logging engine outputs.
        """
        self.backend = None
        self.conf = conf
        self.name = None
        self.sim_time = 0.0
        self.input_log_buffer = {}
        self.engine_inputs = None
        self.engine_outputs = None
        self.engine_inputs_path = engine_inputs_path
        self.engine_outputs_path = engine_outputs_path

    @abstractmethod
    def init(self):
        """Initializes the physics engine backend."""
        pass

    @abstractmethod
    def term(self):
        """Terminates the physics engine backend."""
        pass

    @abstractmethod
    def is_connected(self):
        """
        Checks if the physics engine is connected.

        Returns:
            bool: True if connected, False otherwise.
        """
        pass

    @abstractmethod
    def set_force(self, propeller_index, thrust):
        """
        Sets the thrust force for a specific propeller.

        Args:
            propeller_index: The index of the propeller.
            thrust: The thrust force to apply.
        """
        pass

    @abstractmethod
    def set_torque(self, propeller_index, torque):
        """
        Sets the torque for a specific propeller.

        Args:
            propeller_index: The index of the propeller.
            torque: The torque to apply.
        """
        pass

    @abstractmethod
    def step(self):
        """Advances the simulation by one time step."""
        pass

    @abstractmethod
    def get_zero_order_state(self):
        """
        Retrieves the zero-order state (position and orientation).

        Returns:
            tuple: A tuple containing position [x, y, z] and orientation [roll, pitch, yaw].
        """
        pass

    @abstractmethod
    def get_first_order_state(self):
        """
        Retrieves the first-order state (linear and angular velocity).

        Returns:
            tuple: A tuple containing linear velocity [vx, vy, vz] and angular velocity [wx, wy, wz].
        """
        pass

    def init_logs(self):
        """Initializes the CSV logs for engine inputs and outputs."""
        if not os.path.exists(LOGS_PATH):
            os.makedirs(LOGS_PATH)
        self.engine_inputs = open(self.engine_inputs_path, 'w')
        self.engine_inputs.write('time,thrust_1,torque_1,thrust_2,torque_2,thrust_3,torque_3,thrust_4,torque_4\n')
        
        self.engine_outputs = open(self.engine_outputs_path, 'w')
        self.engine_outputs.write('time,x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz\n')

    def term_logs(self):
        """Closes the CSV log files."""
        if self.engine_inputs: self.engine_inputs.close()
        if self.engine_outputs: self.engine_outputs.close()

    def log_input(self, index, key, value):
        """
        Buffers an input value for logging.

        Args:
            index: The index of the propeller.
            key: The name of the input (e.g., 'thrust', 'torque').
            value: The value to log.
        """
        if index not in self.input_log_buffer:
            self.input_log_buffer[index] = {}
        self.input_log_buffer[index][key] = value

    def write_log_inputs(self):
        """Writes the buffered input values to the CSV log."""
        if self.engine_inputs:
            line = f"{self.sim_time}"
            for i in range(self.conf.PROPELLERS):
                vals = self.input_log_buffer.get(i, {})
                line += f",{vals.get('thrust', 0.0)},{vals.get('torque', 0.0)}"
            self.engine_inputs.write(line + "\n")
            self.input_log_buffer.clear()

    def write_log_outputs(self, pos, rot, lin, ang):
        """
        Writes the output state values to the CSV log.

        Args:
            pos: Position [x, y, z].
            rot: Rotation [roll, pitch, yaw].
            lin: Linear velocity [vx, vy, vz].
            ang: Angular velocity [wx, wy, wz].
        """
        if self.engine_outputs:
            line = f"{self.sim_time},{pos[0]},{pos[1]},{pos[2]},{rot[0]},{rot[1]},{rot[2]},{lin[0]},{lin[1]},{lin[2]},{ang[0]},{ang[1]},{ang[2]}\n"
            self.engine_outputs.write(line)

class PyBulletEngine(SimulatorEngine):
    """PyBullet physics engine implementation."""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.name = PhysicsEngines.PYBULLET

    def init(self):
        """
        Initializes the PyBullet simulation.

        Raises:
            RuntimeError: If connection fails or URDF loading fails.
            FileNotFoundError: If the URDF file is not found.
        """
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
        urdf_model_path = generate_urdf_model(self.name.value, self.conf)
        
        if not os.path.exists(urdf_model_path):
            raise FileNotFoundError(ERR_URDF_NOT_FOUND.format(urdf_model_path))

        try:
            self.multirotor_avatar = self.backend.loadURDF(urdf_model_path, self.conf.INITIAL_POSITION, self.backend.getQuaternionFromEuler(self.conf.INITIAL_ROTATION)) # type: ignore
        except self.backend.error as e:
            raise RuntimeError(ERR_URDF_LOAD_FAILED.format(urdf_model_path, e))
        
    def term(self):
        """Disconnects from the PyBullet simulation."""
        if self.backend.isConnected():
            self.backend.disconnect()

    def is_connected(self):
        """Checks if connected to PyBullet."""
        return self.backend.isConnected()

    def set_force(self, propeller_index, thrust):
        """Applies thrust force to a propeller link."""
        # Apply force in the Z-axis of the link frame
        self.backend.applyExternalForce(self.multirotor_avatar, propeller_index, [0, 0, thrust], [0, 0, 0], self.backend.LINK_FRAME)

    def set_torque(self, propeller_index, torque):
        """Applies torque to a propeller link."""
        # Apply torque around the Z-axis of the link frame
        self.backend.applyExternalTorque(self.multirotor_avatar, propeller_index, [0, 0, torque], self.backend.LINK_FRAME)

    def step(self):
        """Steps the PyBullet simulation."""
        try:
            with self.allowInternalThreadCaches(self.backend):
                self.backend.stepSimulation()
        except self.backend.error as e:
            raise RuntimeError(ERR_ENGINE_STEP_FAILED) from e
        self.sim_time += self.conf.TIME_STEP

    def get_zero_order_state(self):
        """Gets position and orientation from PyBullet."""
        pos, quat = self.backend.getBasePositionAndOrientation(self.multirotor_avatar)
        euler = self.backend.getEulerFromQuaternion(quat)
        return pos, euler

    def get_first_order_state(self):
        """Gets linear and angular velocities from PyBullet."""
        return self.backend.getBaseVelocity(self.multirotor_avatar)

class MuJoCoEngine(SimulatorEngine):
    """MuJoCo physics engine implementation."""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.name = PhysicsEngines.MUJOCO

    def init(self):
        """
        Initializes the MuJoCo simulation.

        Raises:
            FileNotFoundError: If the URDF file is not found.
            RuntimeError: If URDF loading fails.
        """
        import mujoco
        self.backend = mujoco
        self.viewer = None

        urdf_model_path = generate_urdf_model(self.name.value, self.conf)
        
        if not os.path.exists(urdf_model_path):
            raise FileNotFoundError(ERR_URDF_NOT_FOUND.format(urdf_model_path))

        try:
            self.model = self.backend.MjModel.from_xml_path(urdf_model_path)
        except ValueError as e:
            raise RuntimeError(ERR_URDF_LOAD_FAILED.format(urdf_model_path, e))
        
        self.data = self.backend.MjData(self.model)
        
        self.backend.mj_resetData(self.model, self.data)
        
        # Initial Position
        self.data.qpos[0:3] = self.conf.INITIAL_POSITION
        
        # Initial Rotation (Euler -> Quat [w, x, y, z])
        roll, pitch, yaw = self.conf.INITIAL_ROTATION
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        self.data.qpos[3:7] = [w, x, y, z]
        
        # Update kinematics to ensure xmat is valid for the first step
        self.backend.mj_forward(self.model, self.data)
        
        # Buffer for external forces/torques: body_id -> [fx, fy, fz, tx, ty, tz]
        self.pending_forces = {}
        
        # Identify the main body.
        self.frame_body_id = self.backend.mj_name2id(self.model, self.backend.mjtObj.mjOBJ_BODY, 'frame')
        if self.frame_body_id == -1:
            # Fallback if name not found, though 'frame' should exist from URDF.
            self.frame_body_id = 1

        self.pending_forces[self.frame_body_id] = np.zeros(6)

        self.xmat = self.data.xmat[self.frame_body_id].reshape(3, 3)
            
        # Pre-calculate propeller offsets relative to the frame center
        self.propeller_offsets = []
        for i in range(self.conf.PROPELLERS):
            angle = 2 * math.pi * i / self.conf.PROPELLERS
            x = self.conf.MAIN_RADIUS * math.cos(angle)
            y = self.conf.MAIN_RADIUS * math.sin(angle)
            self.propeller_offsets.append(np.array([x, y, 0.0]))
            
        if self.conf.GUI:
            # Initialize thrust storage for visualization
            self.current_thrusts = {}
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            
            # Place camera on a far point directed towards the origin
            cam_pos = np.array([-4.0, 4.0, 2.0])
            target_pos = np.array([0.0, 0.0, 0.0])
            
            self.viewer.cam.lookat[:] = target_pos
            
            rel = cam_pos - target_pos
            dist = np.linalg.norm(rel)
            if dist > 0:
                self.viewer.cam.distance = dist
                self.viewer.cam.azimuth = math.degrees(math.atan2(rel[1], rel[0]))
                self.viewer.cam.elevation = -math.degrees(math.asin(rel[2] / dist))

    def term(self):
        """Closes the MuJoCo viewer if active."""
        if self.viewer:
            self.viewer.close()

    def is_connected(self):
        """Checks if the MuJoCo model is loaded."""
        return hasattr(self, 'model') and self.model is not None

    def set_force(self, propeller_index, thrust):
        """Applies thrust force to the body via pending forces buffer."""
        # Force in body frame
        f_body = np.array([0, 0, thrust])
        
        # Torque due to lever arm (r x F)
        r = self.propeller_offsets[propeller_index]
        t_body = np.cross(r, f_body)

        f_global = self.xmat @ f_body
        t_global = self.xmat @ t_body
        
        # MuJoCo xfrc_applied is [Force, Torque]
        self.pending_forces[self.frame_body_id][3:] += t_global
        self.pending_forces[self.frame_body_id][:3] += f_global

    def set_torque(self, propeller_index, torque):
        """Applies torque to the body via pending forces buffer."""
        t_body = np.array([0, 0, torque])
        t_global = self.xmat @ t_body
        
        # MuJoCo xfrc_applied is [Force, Torque]
        self.pending_forces[self.frame_body_id][3:] += t_global

    def step(self):
        """Steps the MuJoCo simulation."""
        # Apply forces BEFORE stepping the simulation
        for body_id, wrench in self.pending_forces.items():
            self.data.xfrc_applied[body_id] = wrench
        
        self.backend.mj_step(self.model, self.data)
        self.backend.mj_kinematics(self.model, self.data)
        self.xmat = self.data.xmat[self.frame_body_id].reshape(3, 3)
        self.pending_forces.clear()
        self.pending_forces[self.frame_body_id] = np.zeros(6)
        
        if self.viewer and self.viewer.is_running():
            self.viewer.user_scn.ngeom = 0
            self.viewer.sync()
        self.sim_time += self.conf.TIME_STEP

    def get_zero_order_state(self):
        """Gets position and orientation from MuJoCo data."""
        pos = self.data.qpos[0:3]
        w, x, y, z = self.data.qpos[3:7]
        
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return pos, [roll, pitch, yaw]

    def get_first_order_state(self):
        """Gets linear and angular velocities from MuJoCo data."""
        # Use qvel (updated by mj_step) and xmat (updated by mj_kinematics) to avoid one-step delay
        lin_vel_world = self.data.qvel[0:3]
        ang_vel_body = self.data.qvel[3:6]
        ang_vel_world = self.xmat @ ang_vel_body
        return lin_vel_world, ang_vel_world

def get_physics_engine(conf):
    """
    Factory method to create a physics engine instance based on configuration.

    Args:
        conf: The simulation configuration object.

    Returns:
        SimulatorEngine: An instance of the configured physics engine.

    Raises:
        ValueError: If the configured engine type is unknown.
    """
    engine_type = getattr(conf, 'PHYSICS_ENGINE')
    if engine_type == PhysicsEngines.PYBULLET:
        return PyBulletEngine(conf=conf, engine_inputs_path=f'{LOGS_PATH}/inputs_pybullet.csv', engine_outputs_path=f'{LOGS_PATH}/outputs_pybullet.csv')
    elif engine_type == PhysicsEngines.MUJOCO:
        return MuJoCoEngine(conf=conf, engine_inputs_path=f'{LOGS_PATH}/inputs_mujoco.csv', engine_outputs_path=f'{LOGS_PATH}/outputs_mujoco.csv')
    raise ValueError(ERR_ENGINE_UNKNOWN.format(engine_type))