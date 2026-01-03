import os

TIME_STEP = 1./120. # Default physics time step in seconds (240 Hz)

BASE_DIRECTORY = os.path.dirname(os.path.abspath(__file__))

UPDATE_URDF_MODEL = True
URDF_TEMPLATE = f'{BASE_DIRECTORY}/model.urdf.jinja'
URDF_MODEL = f'{BASE_DIRECTORY}/.gen/model_{{engine}}.urdf'

FRAME_MASS = 10.0
FRAME_SIZE = 0.1 # For a cube, it's its side length (inertia matrix is approximated assuming that the frame is a cube)
PROPELLERS = 4
MAIN_RADIUS = 0.1 # The circle around the multirotor center where the propellers are placed. Propellers are equally spaced, with rotor_1 always placed at position [x y] = [1 0] and the others arranged in counter-clockwise order. rotor_1 is right-handed, rotor_2 is left_handed and so on. [x y z] is a right-handed coordinate system with x pointing forward, y pointing to the left, and z pointing upwards. See https://media.springernature.com/lw685/springer-static/image/art%3A10.1007%2Fs11071-019-05002-9/MediaObjects/11071_2019_5002_Fig1_HTML.png

G = -9.81

DAMPING_FACTOR = 3.5