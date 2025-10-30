import os
from math import cos, sin
from ml.data import DECIMAL, FLOAT, NUMPY

TIME_STEP = 1./120. # Default physics time step in seconds (240 Hz)

BASE_DIRECTORY = os.path.dirname(os.path.abspath(__file__))

UPDATE_URDF_MODEL = True
URDF_TEMPLATE = f'{BASE_DIRECTORY}/model.urdf.jinja'
URDF_MODEL = f'{BASE_DIRECTORY}/.gen/model.urdf'

FRAME_MASS = 10.0
FRAME_SIZE = 0.1 # For a cube, it's its side length (inertia matrix is approximated assuming that the frame is a cube)
PROPELLERS = 4
MAIN_RADIUS = 0.1 # The circle around the multirotor center where the propellers are placed. Propellers are equally spaced, with rotor_1 always placed at position [x y] = [1 0] and the others arranged in counter-clockwise order. rotor_1 is right-handed, rotor_2 is left_handed and so on. [x y z] is a right-handed coordinate system with x pointing forward, y pointing to the left, and z pointing upwards. See https://media.springernature.com/lw685/springer-static/image/art%3A10.1007%2Fs11071-019-05002-9/MediaObjects/11071_2019_5002_Fig1_HTML.png

NUMBER_IMPLEMENTATION = NUMPY # The active implementation
DECIMAL_CONTEXT_PRECISION = 4 # Only used if NUMBER_IMPLEMENTATION is DECIMAL

G = -9.81

REAL_TIME_SIMULATION = False

# Decimation factor for plotting. A value of N means 1 out of every N data points will be plotted.
PLOT_DECIMATION = 100

# --- Default Simulation Parameters ---
INITIAL_POSITION = [1, 0, 1]
INITIAL_ROTATION = [0, 0, 0]

# Default trajectory functions
w = 0.5  # angular speed of the xy trajectory in rad/s
q = 1.0  # angular speed of the z trajectory in rad/s
a = 0.7  # amplitude of the z trajectory in rad/s
graph_margin = 1.5

SET_POSITION = lambda t: [cos(w * t), sin(w * t), 1 + a * sin(q * t)]
SET_SPEED = lambda t: [w * (-sin(w * t)), w * cos(w * t), a * q * cos(q * t)]
POSITION_GRAPH_BOUNDARIES = [(-1 * graph_margin, 1 * graph_margin), (-1 * graph_margin, 1 * graph_margin), ((1 - a * graph_margin), (1 + a * graph_margin))]
SPEED_GRAPH_BOUNDARIES = [(-w * graph_margin, w * graph_margin), (-w * graph_margin, w * graph_margin), (-a * q * graph_margin, a * q * graph_margin)]

GUI = False
PLOT = True

# Tracer configuration
TRACER_ENABLED = False # Set to True to enable tracing, False to disable